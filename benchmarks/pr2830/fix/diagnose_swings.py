#!/usr/bin/env python3
"""Reconstruct the decision behind every cross-map swing, and classify it.

Input: a bench_2830.py results file produced with --trace, so every decision
carries the frontier candidates the arm's own detector produced, the size and
the score the arm's own scorer gave each of them, which of them the harness
failed-goal filter suppressed, and (on swing decisions only) the route length
from the robot to each of them under the planner's own cost rule.

A swing is a consecutive-goal jump longer than half the bounding-box diagonal of
the body-passable floor of that map (11.80 m on bigoffice, 11.83 m on
bigoffice_hc), which is the definition the mid-start bench used.

CLASSES, declared before looking at the traces
----------------------------------------------
R_near = 6.0 m of GEODESIC route length from the robot. That is 1.5 lidar ranges
at the 4 m range under test, and it is the smaller of the two radii candidate B
sweeps, so the diagnosis and the candidate share one definition of "vicinity".

N   not a crossing at all. The swing is defined on the GOAL sequence (goal k to
    goal k+1), and the robot does not always stand on goal k: when a goal timed
    out or was refused, the robot is somewhere else, and the next goal can be
    11.8 m from the abandoned goal while being a couple of metres from the
    robot. Those decisions are counted first and set aside, because nothing in
    them is a walk across the map. The remaining swings are the real crossings
    and only they are classified A / B / C.
A   a near frontier existed and lost the ranking. At least one candidate within
    R_near of the robot was on the arm's own ranked list at the swing decision,
    was not suppressed, and the arm ranked the remote one above it.
A-blocked   a near candidate existed but every one of them was suppressed by the
    harness failed-goal filter, i.e. the drive planner had already refused it.
    Re-ranking cannot fix this one, so it is counted apart from A.
B   a near frontier was temporarily missing. Nothing within R_near at the swing
    decision, but the region HAD a candidate at an earlier decision, and a
    candidate reappears in the same place later. Cluster filtering /
    min_frontier_perimeter / fragmentation made it blink out, and the blink is
    what forces the return.
C   legitimate. Nothing within R_near at the swing decision, and the region
    never produces a candidate in the same place again: the area really was
    exhausted, and any later return is to space that was revealed afterwards.
"""
from __future__ import annotations

import json
import math
import os
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))

R_NEAR_M = 6.0           # geodesic vicinity
SAME_SPOT_M = 1.5        # two candidates at this distance are "the same frontier"
REGION_M = 6.0           # straight-line radius of "the region being left behind"
ROUNDTRIP_WINDOW = 3     # a return within this many jumps, as the bench counted it


def load(path):
    with open(path) as fh:
        return json.load(fh)


def abandonment(t):
    """True when this decision left something behind: the goal taken is further
    than R_near from the robot while an available candidate sat inside R_near.

    Straight-line, not geodesic: the trace only carries route lengths on swing
    decisions, and this test is run on every decision. On this map the two agree
    to a median factor of 1.10x, so the proxy is stated and used as a proxy.
    """
    if t["chosen"] is None:
        return False
    P, ch = t["robot"], t["chosen"]
    if math.hypot(ch[0] - P[0], ch[1] - P[1]) <= R_NEAR_M:
        return False
    supp = t["suppressed"]
    for c in t["cands"]:
        if any(math.hypot(c[0] - sx, c[1] - sy) < 1e-3 for sx, sy in supp):
            continue
        if math.hypot(c[0] - ch[0], c[1] - ch[1]) < 1e-3:
            continue
        if math.hypot(c[0] - P[0], c[1] - P[1]) <= R_NEAR_M:
            return True
    return False


def classify_run(res):
    s = res["summary"]
    thr = s["swing_threshold_m"]
    trace = [t for t in res.get("trace", []) if t["chosen"] is not None]
    out = []
    for i, t in enumerate(trace):
        if not t["is_swing"]:
            continue
        P = t["robot"]
        cands = t["cands"]                       # [x, y, size, score]
        geo = t["geo"] or [float("inf")] * len(cands)
        geo = [float("inf") if g is None else g for g in geo]
        supp = t["suppressed"]
        chosen = t["chosen"]

        def is_suppressed(c):
            return any(math.hypot(c[0] - sx, c[1] - sy) < 1e-3 for sx, sy in supp)

        def is_chosen(c):
            return math.hypot(c[0] - chosen[0], c[1] - chosen[1]) < 1e-3

        near = [(j, c) for j, c in enumerate(cands)
                if geo[j] <= R_NEAR_M and not is_suppressed(c) and not is_chosen(c)]
        near_blocked = [(j, c) for j, c in enumerate(cands)
                        if geo[j] <= R_NEAR_M and is_suppressed(c)]
        geo_chosen = min([geo[j] for j, c in enumerate(cands) if is_chosen(c)],
                         default=float("inf"))
        s_chosen = max((c[3] for c in cands
                        if math.hypot(c[0] - chosen[0], c[1] - chosen[1]) < 1e-3),
                       default=float("nan"))

        rec = {
            "map": s["map"], "start": s["start"], "config": s["config"],
            "arm": s["arm"], "goal_index": t["goal_index"],
            "robot": P, "prev_goal": t["prev_goal"], "chosen": chosen,
            "jump_m": round(math.hypot(chosen[0] - t["prev_goal"][0],
                                       chosen[1] - t["prev_goal"][1]), 2),
            "threshold_m": round(thr, 2),
            "n_cands": len(cands),
            "n_near": len(near), "n_near_blocked": len(near_blocked),
            "d_geo_chosen_m": (None if not math.isfinite(geo_chosen)
                               else round(geo_chosen, 2)),
            "d_euc_chosen_m": round(math.hypot(chosen[0] - P[0], chosen[1] - P[1]), 2),
            "score_chosen": s_chosen,
            # every candidate of this decision, so a different R or a different
            # near-radius can be re-derived from the file without re-running
            "cands": [{"x": round(c[0], 2), "y": round(c[1], 2), "size": c[2],
                       "score": c[3],
                       "geo_m": (None if not math.isfinite(geo[j]) else round(geo[j], 2)),
                       "euc_m": round(math.hypot(c[0] - P[0], c[1] - P[1]), 2),
                       "suppressed": is_suppressed(c),
                       "is_chosen": math.hypot(c[0] - chosen[0], c[1] - chosen[1]) < 1e-3}
                      for j, c in enumerate(cands)],
            # the closest alternative to the one taken, whatever its distance
            "nearest_other_geo_m": round(min(
                [geo[j] for j, c in enumerate(cands)
                 if math.hypot(c[0] - chosen[0], c[1] - chosen[1]) >= 1e-3
                 and not is_suppressed(c) and math.isfinite(geo[j])] or [float("inf")]), 2),
            "nearest_other_available_geo_m": round(min(
                [geo[j] for j, c in enumerate(cands)
                 if not is_suppressed(c) and math.isfinite(geo[j])] or [float("inf")]), 2),
            # the most recent earlier decision that left a near candidate behind
            "last_abandonment_back": next(
                (i - j for j in range(i - 1, -1, -1) if abandonment(trace[j])), None),
        }

        if geo_chosen <= R_NEAR_M:
            # the goal jumped, the robot did not: it was never on the previous
            # goal, so this is not a walk across the map
            rec["cls"] = "N"
            out.append(rec)
            continue

        if near:
            best = max(near, key=lambda jc: jc[1][3])
            rec["cls"] = "A"
            rec["best_near"] = [round(best[1][0], 2), round(best[1][1], 2)]
            rec["best_near_geo_m"] = round(geo[best[0]], 2)
            rec["best_near_score"] = best[1][3]
            rec["best_near_detect_index"] = best[0]
            rec["k_needed"] = (s_chosen / best[1][3]) if best[1][3] > 0 else float("inf")
            out.append(rec)
            continue

        if near_blocked:
            rec["cls"] = "A-blocked"
            out.append(rec)
            continue

        # nothing near: did the region have a frontier before, and does one come
        # back to the same spot later?
        def in_region(c):
            return math.hypot(c[0] - P[0], c[1] - P[1]) <= REGION_M

        before = []
        for u in trace[:i]:
            for c in u["cands"]:
                if in_region(c):
                    before.append((c[0], c[1]))
        after = []
        for u in trace[i + 1:]:
            for c in u["cands"]:
                if in_region(c):
                    after.append((c[0], c[1]))
        returned = [b for b in before
                    if any(math.hypot(b[0] - a[0], b[1] - a[1]) <= SAME_SPOT_M
                           for a in after)]
        rec["n_before"] = len(before)
        rec["n_after"] = len(after)
        rec["n_returned"] = len(returned)
        rec["cls"] = "B" if returned else "C"
        out.append(rec)
    return out


def round_trips(res):
    """The bench's own definition: a swing followed within ROUNDTRIP_WINDOW jumps
    by another swing whose direction opposes it."""
    s = res["summary"]
    thr = s["swing_threshold_m"]
    goals = res["goals"]
    jumps = []
    for g in goals:
        if g["index"] == 0 or (isinstance(g["d_prev_goal"], float)
                               and math.isnan(g["d_prev_goal"])):
            continue
        jumps.append(g)
    sw = []
    for n, g in enumerate(goals):
        if n == 0:
            continue
        p = goals[n - 1]
        d = math.hypot(g["x"] - p["x"], g["y"] - p["y"])
        if d > thr:
            sw.append((n, g["x"] - p["x"], g["y"] - p["y"]))
    pairs = []
    for a in range(len(sw)):
        for b in range(a + 1, len(sw)):
            if sw[b][0] - sw[a][0] > ROUNDTRIP_WINDOW:
                break
            dot = sw[a][1] * sw[b][1] + sw[a][2] * sw[b][2]
            if dot < 0:
                pairs.append((sw[a][0], sw[b][0]))
                break
    return pairs


def main(argv):
    path = argv[1] if len(argv) > 1 else os.path.join(HERE, "results_trace_4m.json")
    d = load(path)
    rows = []
    rts = defaultdict(list)
    for r in d["results"]:
        rows.extend(classify_run(r))
        s = r["summary"]
        key = (s["config"], s["arm"])
        for a, b in round_trips(r):
            rts[key].append((s["map"], s["start"], a, b))

    with open(os.path.join(HERE, "swing_decisions.json"), "w") as fh:
        json.dump({"R_near_m": R_NEAR_M, "same_spot_m": SAME_SPOT_M,
                   "region_m": REGION_M, "swings": rows}, fh, indent=1)

    print(f"{len(rows)} swings classified from {path}\n")
    for cfg in ("shipped", "scoring"):
        for arm in ("stock", "pr2830"):
            sub = [r for r in rows if r["config"] == cfg and r["arm"] == arm]
            c = Counter(r["cls"] for r in sub)
            print(f"{cfg:8s} {arm:7s} n={len(sub):3d}  "
                  f"N={c[chr(78)]:3d}  A={c[chr(65)]:3d}  A-blocked={c['A-blocked']:3d}  "
                  f"B={c['B']:3d}  C={c['C']:3d}")
    print()
    c = Counter(r["cls"] for r in rows)
    tot = len(rows)
    for k in ("N", "A", "A-blocked", "B", "C"):
        print(f"  total {k:10s} {c[k]:3d}  ({100.0 * c[k] / tot:.0f} %)")

    ks = sorted(r["k_needed"] for r in rows if r["cls"] == "A")
    if ks:
        import statistics
        print(f"\nclass A: the factor by which the chosen remote frontier beat the "
              f"best near one, on the arm's own score")
        print(f"  n={len(ks)}  min {ks[0]:.2f}  p25 {ks[len(ks)//4]:.2f}  "
              f"median {statistics.median(ks):.2f}  p75 {ks[3*len(ks)//4]:.2f}  "
              f"max {ks[-1]:.2f}")
        for k in (1.0, 1.5, 2.0, 3.0):
            n = sum(1 for v in ks if v <= k)
            print(f"  a margin of k={k:<4g} would have kept the robot near on "
                  f"{n:3d} / {len(ks)} of them ({100.0*n/len(ks):.0f} %)")
    # how far the nearest available alternative was, at every swing: this says
    # whether a bigger vicinity radius would have found anything at all
    print("\nat each swing, the geodesic distance to the nearest AVAILABLE "
          "candidate other than the one taken")
    for cls in ("N", "A", "A-blocked", "B", "C"):
        v = sorted(r["nearest_other_geo_m"] for r in rows if r["cls"] == cls
                   and math.isfinite(r["nearest_other_geo_m"]))
        inf_n = sum(1 for r in rows if r["cls"] == cls
                    and not math.isfinite(r["nearest_other_geo_m"]))
        if not v:
            print(f"  {cls:10s} n=0 finite ({inf_n} with no other candidate)")
            continue
        import statistics
        print(f"  {cls:10s} n={len(v):3d}  min {v[0]:6.2f}  median "
              f"{statistics.median(v):6.2f}  max {v[-1]:6.2f} m"
              f"   ({inf_n} swings had no other candidate at all)")
    for R in (6.0, 9.0, 12.0):
        n = sum(1 for r in rows
                if math.isfinite(r["nearest_other_available_geo_m"])
                and r["nearest_other_available_geo_m"] <= R)
        print(f"  a vicinity radius R={R:g} m would have found an available "
              f"candidate at {n:3d} / {tot} swings ({100.0*n/tot:.0f} %)")

    # was the swing set up earlier? A swing with nothing near is forced AT THE
    # MOMENT it happens; the question is whether an earlier decision walked away
    # from a near frontier and made it inevitable.
    print("\nswings preceded by an earlier decision that left a near candidate "
          "behind (straight-line proxy)")
    for cls in ("N", "A", "A-blocked", "B", "C"):
        sub = [r for r in rows if r["cls"] == cls]
        if not sub:
            continue
        with_ab = [r for r in sub if r["last_abandonment_back"] is not None]
        backs = sorted(r["last_abandonment_back"] for r in with_ab)
        import statistics as _st
        med = f"{_st.median(backs):.1f}" if backs else "-"
        print(f"  {cls:10s} {len(with_ab):3d} / {len(sub):3d}   median "
              f"{med} goals before the swing")
    n_ab_all = sum(1 for r in rows if r["last_abandonment_back"] is not None)
    print(f"  all        {n_ab_all:3d} / {tot:3d}")

    print("\nround trips (bench definition)")
    for cfg in ("shipped", "scoring"):
        for arm in ("stock", "pr2830"):
            v = rts[(cfg, arm)]
            runs = len({(m, s) for m, s, _a, _b in v})
            print(f"  {cfg:8s} {arm:7s} {len(v):3d} in {runs} runs")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

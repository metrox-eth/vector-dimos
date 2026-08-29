#!/usr/bin/env python3
"""Classify the RESIDUAL cross-map crossings of the Go2-profile bench.

Same taxonomy, same radii, same code path as ../sim_2830_fix/diagnose_swings.py
(kept here as diagnose_swings_ref.py; crosscheck_classifier.py runs both on the
same traces and prints any class that differs). What is added:

  * many input files instead of one, tagged by lidar range,
  * the arm's OWN score for a POLICY arm is the policy's adjusted score, not the
    upstream one the trace also carries, so `k_needed` means the same thing in
    both arms,
  * the five weighted terms of the upstream score at every class-A decision, so
    the report can say which term loses,
  * a paired stock-vs-remedy delta per class, which is what answers "did the
    remedy suppress legitimate long walks",
  * a re-derivation check of every traced run against the recorded Go2 bench.

CLASSES, unchanged and declared in ../sim_2830_fix/diagnostic_swings.md:
  N          not a crossing: the goal jumped, the robot did not.
  A          a near frontier existed, was available, and lost the ranking.
  A-blocked  a near frontier existed but every one was planner-refused.
  B          nothing near now, but the cluster blinked out and comes back.
  C          legitimate: nothing near, and nothing comes back.

R_near = 6.0 m of GEODESIC route length, the value the fix job declared, kept
unchanged at BOTH lidar ranges so the two ranges are classified by one rule.
At 4 m that is 1.5 lidar ranges, at 12 m it is 0.5; the sensitivity of the
answer to that radius is printed (R = 4 / 6 / 9 / 12 m) rather than tuned.

2026-08-30: the vicinity predicate was corrected, see correction_sensibilite.md
and recompute_sensitivity.py. Re-running this file overwrites
resid_classification.json; the corrected numbers live in
resid_classification_v2.json, which was derived from the stored candidate lists
without re-running any simulation.
"""
from __future__ import annotations

import glob
import json
import math
import os
import statistics
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
GO2 = os.path.join(os.path.dirname(HERE), "sim_2830_go2")

R_NEAR_M = 6.0
SAME_SPOT_M = 1.5
REGION_M = 6.0
ROUNDTRIP_WINDOW = 3
TERM_NAMES = ["info_gain", "explored_goals", "distance", "obstacles", "momentum"]


# --------------------------------------------------------------------------
# the bench's own crossing counter, re-derived (analyse_cand2.swings_of)
# --------------------------------------------------------------------------
def swings_of(goals, thr):
    raw, real = [], []
    for n in range(1, len(goals)):
        a, b = goals[n - 1], goals[n]
        dx, dy = b["x"] - a["x"], b["y"] - a["y"]
        if math.hypot(dx, dy) <= thr:
            continue
        raw.append((n, dx, dy))
        if math.hypot(b["x"] - b["from_x"], b["y"] - b["from_y"]) > R_NEAR_M:
            real.append((n, dx, dy))
    return raw, real


def round_trips(sw):
    out = 0
    for i in range(len(sw)):
        for j in range(i + 1, len(sw)):
            if sw[j][0] - sw[i][0] > ROUNDTRIP_WINDOW:
                break
            if sw[i][1] * sw[j][1] + sw[i][2] * sw[j][2] < 0:
                out += 1
                break
    return out


# --------------------------------------------------------------------------
# the classifier (diagnose_swings.classify_run, plus the policy score and terms)
# --------------------------------------------------------------------------
def abandonment(t):
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


def policy_scores(t, cands):
    """The policy's adjusted score for each candidate of this decision, aligned
    to the order of `cands`, or None when this arm has no policy."""
    pol = t.get("policy")
    if not pol or "adj" not in pol or "in_order" not in pol:
        return None, None, None
    adj, route, dev = {}, {}, {}
    for (x, y), a, r, d in zip(pol["in_order"], pol["adj"],
                               pol.get("route_m", [None] * len(pol["adj"])),
                               pol.get("dev", [None] * len(pol["adj"]))):
        adj[(round(x, 3), round(y, 3))] = a
        route[(round(x, 3), round(y, 3))] = r
        dev[(round(x, 3), round(y, 3))] = d

    def look(d, c):
        return d.get((round(c[0], 3), round(c[1], 3)))

    return ([look(adj, c) for c in cands],
            [look(route, c) for c in cands],
            [look(dev, c) for c in cands])


def classify_run(res, rng):
    s = res["summary"]
    thr = s["swing_threshold_m"]
    trace = [t for t in res.get("trace", []) if t["chosen"] is not None]
    out = []
    for i, t in enumerate(trace):
        if not t["is_swing"]:
            continue
        P = t["robot"]
        cands = t["cands"]
        terms = t.get("terms") or [None] * len(cands)
        geo = t["geo"] or [float("inf")] * len(cands)
        geo = [float("inf") if g is None else g for g in geo]
        supp = t["suppressed"]
        chosen = t["chosen"]
        padj, proute, pdev = policy_scores(t, cands)

        def is_suppressed(c):
            return any(math.hypot(c[0] - sx, c[1] - sy) < 1e-3 for sx, sy in supp)

        def is_chosen(c):
            return math.hypot(c[0] - chosen[0], c[1] - chosen[1]) < 1e-3

        # "the arm's own score": the policy's adjusted score on a policy arm,
        # the upstream score on a bare arm.
        def own(j):
            if padj is not None and padj[j] is not None:
                return padj[j]
            return cands[j][3]

        near = [(j, c) for j, c in enumerate(cands)
                if geo[j] <= R_NEAR_M and not is_suppressed(c) and not is_chosen(c)]
        near_blocked = [(j, c) for j, c in enumerate(cands)
                        if geo[j] <= R_NEAR_M and is_suppressed(c)]
        geo_chosen = min([geo[j] for j, c in enumerate(cands) if is_chosen(c)],
                         default=float("inf"))
        j_chosen = next((j for j, c in enumerate(cands) if is_chosen(c)), None)
        s_chosen = own(j_chosen) if j_chosen is not None else float("nan")

        rec = {
            "range": rng, "map": s["map"], "start": s["start"],
            "config": s["config"], "arm": s["arm"], "goal_index": t["goal_index"],
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
            "score_chosen_upstream": (cands[j_chosen][3] if j_chosen is not None
                                      else None),
            "terms_chosen": (terms[j_chosen] if j_chosen is not None else None),
            "route_chosen_policy_m": (proute[j_chosen] if proute and j_chosen is not None
                                      else None),
            "dev_chosen_policy": (pdev[j_chosen] if pdev and j_chosen is not None
                                  else None),
            "cands": [{"x": round(c[0], 2), "y": round(c[1], 2), "size": c[2],
                       "score_upstream": c[3],
                       "score_own": own(j),
                       "terms": terms[j] if j < len(terms) else None,
                       "route_policy_m": proute[j] if proute else None,
                       "dev_policy": pdev[j] if pdev else None,
                       "geo_m": (None if not math.isfinite(geo[j]) else round(geo[j], 2)),
                       "euc_m": round(math.hypot(c[0] - P[0], c[1] - P[1]), 2),
                       "suppressed": is_suppressed(c),
                       "is_chosen": is_chosen(c)}
                      for j, c in enumerate(cands)],
        }
        # CORRECTED 2026-08-30. Both fields must exclude the CHOSEN candidate:
        # "the nearest OTHER candidate" cannot be the goal that was taken. The
        # second predicate used to exclude only the suppressed ones, which let
        # the chosen goal count as its own nearest alternative and inflated the
        # vicinity sensitivity table (see correction_sensibilite.md). The buggy
        # value is still emitted, under a _buggy key, for the audit trail.
        for key, pred in (("nearest_other_geo_m",
                           lambda j, c: (not is_chosen(c)) and (not is_suppressed(c))),
                          ("nearest_other_available_geo_m",
                           lambda j, c: (not is_chosen(c)) and (not is_suppressed(c))),
                          ("nearest_other_available_geo_m_buggy",
                           lambda j, c: not is_suppressed(c))):
            v = [geo[j] for j, c in enumerate(cands)
                 if pred(j, c) and math.isfinite(geo[j])]
            rec[key] = round(min(v), 2) if v else None
        rec["last_abandonment_back"] = next(
            (i - j for j in range(i - 1, -1, -1) if abandonment(trace[j])), None)

        if geo_chosen <= R_NEAR_M:
            rec["cls"] = "N"
            out.append(rec)
            continue

        if near:
            best = max(near, key=lambda jc: own(jc[0]))
            bj = best[0]
            rec["cls"] = "A"
            rec["best_near"] = [round(best[1][0], 2), round(best[1][1], 2)]
            rec["best_near_geo_m"] = round(geo[bj], 2)
            rec["best_near_score"] = own(bj)
            rec["best_near_score_upstream"] = best[1][3]
            rec["best_near_terms"] = terms[bj] if bj < len(terms) else None
            rec["best_near_route_policy_m"] = proute[bj] if proute else None
            rec["best_near_dev_policy"] = pdev[bj] if pdev else None
            bs = own(bj)
            rec["k_needed"] = ((s_chosen / bs) if (bs and bs > 0)
                               else float("inf"))
            # which weighted term the near candidate loses on, upstream score
            if rec["terms_chosen"] and rec["best_near_terms"]:
                d = [round(a - b, 6) for a, b in zip(rec["terms_chosen"],
                                                     rec["best_near_terms"])]
                rec["term_delta"] = dict(zip(TERM_NAMES, d))
                rec["term_worst"] = TERM_NAMES[max(range(5), key=lambda k: d[k])]
            out.append(rec)
            continue

        if near_blocked:
            rec["cls"] = "A-blocked"
            out.append(rec)
            continue

        def in_region(c):
            return math.hypot(c[0] - P[0], c[1] - P[1]) <= REGION_M

        before = [(c[0], c[1]) for u in trace[:i] for c in u["cands"] if in_region(c)]
        after = [(c[0], c[1]) for u in trace[i + 1:] for c in u["cands"] if in_region(c)]
        returned = [b for b in before
                    if any(math.hypot(b[0] - a[0], b[1] - a[1]) <= SAME_SPOT_M
                           for a in after)]
        rec["n_before"], rec["n_after"] = len(before), len(after)
        rec["n_returned"] = len(returned)
        rec["cls"] = "B" if returned else "C"
        out.append(rec)
    return out


# --------------------------------------------------------------------------
def load_traced(paths):
    runs = []
    for p in paths:
        with open(p) as fh:
            d = json.load(fh)
        rng = "12m" if abs(d["meta"]["lidar_range_m"] - 12.0) < 1e-6 else "4m"
        for r in d["results"]:
            runs.append((rng, r, os.path.basename(p)))
    return runs


def recorded_index():
    """(range, map, config, start, arm) -> summary, from the Go2 bench."""
    idx = {}
    for p in sorted(glob.glob(os.path.join(GO2, "go2_*.json"))):
        with open(p) as fh:
            d = json.load(fh)
        if "results" not in d:
            continue
        rng = "12m" if abs(d["meta"]["lidar_range_m"] - 12.0) < 1e-6 else "4m"
        for r in d["results"]:
            s = r["summary"]
            idx[(rng, s["map"], s["config"], s["start"], s["arm"])] = r
    return idx


CHECK_KEYS = ["n_goals", "path_m", "area_m2", "coverage_pct", "goal_jump_total_m",
              "cross_map_swings", "sim_s", "goals_reached", "goals_timed_out",
              "goals_no_path", "d_robot_to_goal_median_m", "swing_threshold_m"]


def main(argv):
    paths = sorted(glob.glob(os.path.join(HERE, "resid_*.json")))
    paths = [p for p in paths if "classification" not in p]
    if not paths:
        print("no resid_*.json to classify", file=sys.stderr)
        return 1
    runs = load_traced(paths)
    rec = recorded_index()

    # ---- re-derivation / reproduction check -------------------------------
    checked = matched = disagree = 0
    diffs = []
    for rng, r, _src in runs:
        s = r["summary"]
        key = (rng, s["map"], s["config"], s["start"], s["arm"])
        raw, real = swings_of(r["goals"], s["swing_threshold_m"])
        checked += 1
        if s["cross_map_swings"] != len(raw):
            disagree += 1
            diffs.append(f"{key}: stored {s['cross_map_swings']} raw, re-derived {len(raw)}")
        if key in rec:
            matched += 1
            o = rec[key]["summary"]
            for k in CHECK_KEYS:
                a, b = s.get(k), o.get(k)
                if isinstance(a, float) and isinstance(b, float):
                    same = (math.isnan(a) and math.isnan(b)) or abs(a - b) <= 1e-6
                else:
                    same = a == b
                if not same:
                    disagree += 1
                    diffs.append(f"{key}: {k} traced {a!r} recorded {b!r}")
            ga, gb = r["goals"], rec[key]["goals"]
            if len(ga) != len(gb) or any(
                    abs(x[f] - y[f]) > 1e-6 for x, y in zip(ga, gb)
                    for f in ("x", "y", "from_x", "from_y")):
                disagree += 1
                diffs.append(f"{key}: goal coordinates differ")

    # ---- classify ---------------------------------------------------------
    rows = []
    per_run = {}
    for rng, r, _src in runs:
        cls = classify_run(r, rng)
        rows.extend(cls)
        s = r["summary"]
        raw, real = swings_of(r["goals"], s["swing_threshold_m"])
        c = Counter(x["cls"] for x in cls)
        per_run[f"{rng}|{s['map']}|{s['config']}|{s['start']}|{s['arm']}"] = {
            "swings_raw": len(raw), "swings_real_straightline": len(real),
            "round_trips_real": round_trips(real),
            "traced_swings": len(cls),
            "N": c["N"], "A": c["A"], "A-blocked": c["A-blocked"],
            "B": c["B"], "C": c["C"],
            "real_geodesic": len(cls) - c["N"],
            "path_m": s["path_m"], "coverage_pct": s["coverage_pct"],
            "n_goals": s["n_goals"], "wall_capped": bool(s.get("wall_capped")),
        }

    # ---- output -----------------------------------------------------------
    def pct(n, d):
        return f"{100.0 * n / d:.0f} %" if d else "-"

    print(f"traced runs: {len(runs)} from {len(paths)} files")
    print(f"re-derivation check: {checked} runs re-derived from raw goal "
          f"coordinates, {matched} of them matched against the recorded Go2 "
          f"bench on {len(CHECK_KEYS)} summary fields plus every goal "
          f"coordinate: {disagree} disagreements")
    for d in diffs[:40]:
        print("   DISAGREEMENT " + d)

    print(f"\n{len(rows)} swing decisions traced\n")
    hdr = f"{'rng':4s} {'arm':11s} {'config':8s} {'map':13s} {'n':>3s} {'N':>3s} {'A':>3s} {'Abl':>3s} {'B':>3s} {'C':>3s}"
    print(hdr)
    print("-" * len(hdr))
    for rng in ("4m", "12m"):
        for arm in ("stock", "stock+M4.3"):
            for cfg in ("shipped", "scoring"):
                for m in ("hk_office", "hk_park", "hk_elevator", "hk_entrance"):
                    sub = [r for r in rows if r["range"] == rng and r["arm"] == arm
                           and r["config"] == cfg and r["map"] == m]
                    if not sub:
                        continue
                    c = Counter(r["cls"] for r in sub)
                    print(f"{rng:4s} {arm:11s} {cfg:8s} {m:13s} {len(sub):3d} "
                          f"{c['N']:3d} {c['A']:3d} {c['A-blocked']:3d} "
                          f"{c['B']:3d} {c['C']:3d}")

    print("\nPOOLED over the four discriminating maps and both configs, "
          "real crossings only (class N removed)")
    hdr2 = (f"{'rng':4s} {'arm':11s} {'real':>5s} {'A':>4s} {'Abl':>4s} {'B':>4s} "
            f"{'C':>4s} | {'fixable A+Abl+B':>16s} {'legit C':>9s}")
    print(hdr2)
    print("-" * len(hdr2))
    pooled = {}
    for rng in ("4m", "12m"):
        for arm in ("stock", "stock+M4.3"):
            sub = [r for r in rows if r["range"] == rng and r["arm"] == arm]
            if not sub:
                continue
            c = Counter(r["cls"] for r in sub)
            real = len(sub) - c["N"]
            fix = c["A"] + c["A-blocked"] + c["B"]
            pooled[(rng, arm)] = {"traced": len(sub), "N": c["N"], "real": real,
                                  "A": c["A"], "A-blocked": c["A-blocked"],
                                  "B": c["B"], "C": c["C"],
                                  "fixable": fix, "legit": c["C"],
                                  "fixable_pct": (100.0 * fix / real) if real else None,
                                  "legit_pct": (100.0 * c["C"] / real) if real else None}
            print(f"{rng:4s} {arm:11s} {real:5d} {c['A']:4d} {c['A-blocked']:4d} "
                  f"{c['B']:4d} {c['C']:4d} | {fix:5d} {pct(fix, real):>10s} "
                  f"{c['C']:4d} {pct(c['C'], real):>4s}")
    for arm in ("stock", "stock+M4.3"):
        sub = [r for r in rows if r["arm"] == arm]
        if not sub:
            continue
        c = Counter(r["cls"] for r in sub)
        real = len(sub) - c["N"]
        fix = c["A"] + c["A-blocked"] + c["B"]
        pooled[("both", arm)] = {"traced": len(sub), "N": c["N"], "real": real,
                                 "A": c["A"], "A-blocked": c["A-blocked"],
                                 "B": c["B"], "C": c["C"], "fixable": fix,
                                 "legit": c["C"],
                                 "fixable_pct": (100.0 * fix / real) if real else None,
                                 "legit_pct": (100.0 * c["C"] / real) if real else None}
        print(f"{'both':4s} {arm:11s} {real:5d} {c['A']:4d} {c['A-blocked']:4d} "
              f"{c['B']:4d} {c['C']:4d} | {fix:5d} {pct(fix, real):>10s} "
              f"{c['C']:4d} {pct(c['C'], real):>4s}")

    # ---- class A detail ---------------------------------------------------
    A = [r for r in rows if r["cls"] == "A"]
    print(f"\nclass A in full ({len(A)} decisions). 'k' = the arm's own score of "
          f"the goal taken over the best near candidate's.")
    if A:
        print(f"{'rng':4s} {'arm':11s} {'map':13s} {'start':7s} {'cfg':8s} "
              f"{'goal':>4s} {'jump':>6s} {'chosen@':>8s} {'near@':>6s} "
              f"{'s_chos':>9s} {'s_near':>9s} {'k':>5s}  loses on")
        for r in sorted(A, key=lambda r: (r["range"], r["arm"], r["map"], r["start"])):
            print(f"{r['range']:4s} {r['arm']:11s} {r['map']:13s} {r['start']:7s} "
                  f"{r['config']:8s} {r['goal_index']:4d} {r['jump_m']:6.1f} "
                  f"{(r['d_geo_chosen_m'] or float('nan')):8.1f} "
                  f"{r['best_near_geo_m']:6.1f} {r['score_chosen']:9.4f} "
                  f"{r['best_near_score']:9.4f} {r['k_needed']:5.2f}  "
                  f"{r.get('term_worst', '-')}")
        ks = sorted(r["k_needed"] for r in A if math.isfinite(r["k_needed"]))
        if ks:
            print(f"\n  k: n={len(ks)} min {ks[0]:.2f} median "
                  f"{statistics.median(ks):.2f} max {ks[-1]:.2f}")
            for k in (1.5, 2.0, 3.0):
                n = sum(1 for v in ks if v <= k)
                print(f"  a margin of k={k:g} would have kept the robot near on "
                      f"{n}/{len(ks)}")
        dn = sorted(r["best_near_geo_m"] for r in A)
        dc = sorted(r["d_geo_chosen_m"] for r in A if r["d_geo_chosen_m"])
        print(f"  best near candidate at a median {statistics.median(dn):.1f} m "
              f"of route (range {dn[0]:.1f} to {dn[-1]:.1f}); the goal taken at a "
              f"median {statistics.median(dc):.1f} m (range {dc[0]:.1f} to {dc[-1]:.1f})")
        wc = Counter(r.get("term_worst") for r in A)
        print(f"  the weighted term the near candidate loses most on: "
              f"{dict(wc)}")

    # ---- vicinity sensitivity --------------------------------------------
    print("\nvicinity sensitivity: at how many traced swings was an AVAILABLE "
          "candidate OTHER THAN THE ONE TAKEN within R metres of route length")
    for rng in ("4m", "12m", None):
        for arm in ("stock", "stock+M4.3"):
            sub = [r for r in rows if r["arm"] == arm
                   and (rng is None or r["range"] == rng)]
            if not sub:
                continue
            line = f"  {(rng or 'pooled'):7s} {arm:11s} n={len(sub):3d}  "
            for R in (4.0, 6.0, 9.0, 12.0):
                n = sum(1 for r in sub if r["nearest_other_available_geo_m"] is not None
                        and r["nearest_other_available_geo_m"] <= R)
                line += f"R={R:g}: {n:3d}   "
            print(line)

    # ---- the two N filters -----------------------------------------------
    nstr = sum(v["swings_real_straightline"] for v in per_run.values())
    ngeo = sum(v["real_geodesic"] for v in per_run.values())
    nraw = sum(v["swings_raw"] for v in per_run.values())
    print(f"\nclass N, the two filters: {nraw} raw swings; the bench's "
          f"straight-line rule keeps {nstr} as real, the geodesic rule used here "
          f"keeps {ngeo}")

    # ---- paired stock vs remedy ------------------------------------------
    print("\npaired stock -> stock+M4.3, per (range, map, config, start). "
          "Only cells where both arms were traced.")
    cells = defaultdict(dict)
    for k, v in per_run.items():
        rng, m, cfg, st, arm = k.split("|")
        cells[(rng, m, cfg, st)][arm] = v
    pair_rows = []
    for key in sorted(cells):
        d = cells[key]
        if "stock" not in d or "stock+M4.3" not in d:
            continue
        a, b = d["stock"], d["stock+M4.3"]
        pair_rows.append({
            "range": key[0], "map": key[1], "config": key[2], "start": key[3],
            "stock": a, "remedy": b,
            "d_real": b["real_geodesic"] - a["real_geodesic"],
            "d_A": b["A"] - a["A"], "d_Abl": b["A-blocked"] - a["A-blocked"],
            "d_B": b["B"] - a["B"], "d_C": b["C"] - a["C"],
            "d_path_pct": (100.0 * (b["path_m"] - a["path_m"]) / a["path_m"]
                           if a["path_m"] else None),
            "d_cov_pt": b["coverage_pct"] - a["coverage_pct"],
        })
    hdr3 = (f"{'rng':4s} {'map':13s} {'cfg':8s} {'start':7s} "
            f"{'real':>9s} {'A':>7s} {'Abl':>7s} {'B':>7s} {'C':>7s} "
            f"{'dpath%':>7s} {'dcov pt':>8s}")
    print(hdr3)
    print("-" * len(hdr3))
    for p in pair_rows:
        a, b = p["stock"], p["remedy"]
        print(f"{p['range']:4s} {p['map']:13s} {p['config']:8s} {p['start']:7s} "
              f"{a['real_geodesic']:4d}->{b['real_geodesic']:<4d} "
              f"{a['A']:3d}->{b['A']:<3d} {a['A-blocked']:3d}->{b['A-blocked']:<3d} "
              f"{a['B']:3d}->{b['B']:<3d} {a['C']:3d}->{b['C']:<3d} "
              f"{(p['d_path_pct'] if p['d_path_pct'] is not None else float('nan')):7.1f} "
              f"{p['d_cov_pt']:8.1f}")
    if pair_rows:
        tot = {k: sum(p["stock"][k] for p in pair_rows) for k in
               ("A", "A-blocked", "B", "C", "N", "real_geodesic")}
        tob = {k: sum(p["remedy"][k] for p in pair_rows) for k in
               ("A", "A-blocked", "B", "C", "N", "real_geodesic")}
        print(f"\n  paired totals over {len(pair_rows)} cells: real "
              f"{tot['real_geodesic']} -> {tob['real_geodesic']},  "
              f"A {tot['A']} -> {tob['A']},  A-blocked {tot['A-blocked']} -> "
              f"{tob['A-blocked']},  B {tot['B']} -> {tob['B']},  "
              f"C {tot['C']} -> {tob['C']}")
        rm_c = sum(max(0, p["stock"]["C"] - p["remedy"]["C"]) for p in pair_rows)
        add_c = sum(max(0, p["remedy"]["C"] - p["stock"]["C"]) for p in pair_rows)
        print(f"  class-C crossings the remedy REMOVED, cell by cell: {rm_c}; "
              f"class-C crossings it ADDED: {add_c}; net {tob['C'] - tot['C']:+d}")
        cellsC = [p for p in pair_rows if p["stock"]["C"] > p["remedy"]["C"]]
        if cellsC:
            print("  the cells where a class-C crossing disappeared, and what "
                  "they cost:")
            for p in cellsC:
                print(f"    {p['range']:4s} {p['map']:13s} {p['config']:8s} "
                      f"{p['start']:7s} C {p['stock']['C']}->{p['remedy']['C']}  "
                      f"path {p['stock']['path_m']:6.1f} -> {p['remedy']['path_m']:6.1f} m "
                      f"({p['d_path_pct']:+.1f} %)  coverage "
                      f"{p['stock']['coverage_pct']:5.1f} -> "
                      f"{p['remedy']['coverage_pct']:5.1f} % "
                      f"({p['d_cov_pt']:+.1f} pt)")

    out = {
        "R_near_m": R_NEAR_M, "same_spot_m": SAME_SPOT_M, "region_m": REGION_M,
        "roundtrip_window": ROUNDTRIP_WINDOW,
        "files": [os.path.basename(p) for p in paths],
        "reproduction": {"runs_traced": len(runs), "runs_re_derived": checked,
                         "runs_matched_to_recorded": matched,
                         "disagreements": disagree, "detail": diffs},
        "pooled": {f"{a}|{b}": v for (a, b), v in pooled.items()},
        "per_run": per_run,
        "paired": pair_rows,
        "swings": rows,
    }
    with open(os.path.join(HERE, "resid_classification.json"), "w") as fh:
        json.dump(out, fh, indent=1, default=str)
    print(f"\n-> {os.path.join(HERE, 'resid_classification.json')}")

    # per-decision dumps: the class-A RESIDUALS of the remedy arm first (the
    # thing the next fix would have to target), then the class-A decisions of
    # stock, which are the ones the remedy removed, kept for the record.
    resA = [r for r in rows if r["cls"] == "A" and r["arm"] == "stock+M4.3"]
    stockA = [r for r in rows if r["cls"] == "A" and r["arm"] == "stock"]
    with open(os.path.join(HERE, "resid_A_dumps.txt"), "w") as fh:
        fh.write("PART 1 - class-A RESIDUALS under the remedy stock+M4.3\n"
                 "=====================================================\n")
        if not resA:
            fh.write("None. No decision in the traced grid had an available "
                     "frontier within 6 m of route length that the remedy "
                     "outranked in favour of a crossing.\n\n")
        fh.write("\nPART 2 - the class-A decisions under stock, i.e. the ones "
                 "the remedy removed\n"
                 "==================================================="
                 "===================\n"
                 "Kept because they are the evidence that class A was real "
                 "before the remedy.\n\n")
        for r in resA + stockA:
            fh.write(f"=== {r['range']} {r['map']} {r['config']} {r['start']} "
                     f"{r['arm']} goal {r['goal_index']} ===\n")
            fh.write(f"robot {r['robot']}  prev_goal {r['prev_goal']}  chosen "
                     f"{r['chosen']}  jump {r['jump_m']} m (threshold "
                     f"{r['threshold_m']} m)\n")
            fh.write(f"k needed {r['k_needed']:.3f}   loses on "
                     f"{r.get('term_worst')}   term delta "
                     f"{r.get('term_delta')}\n")
            fh.write(f"{'x':>8s} {'y':>8s} {'size':>5s} {'geo_m':>7s} {'euc_m':>7s} "
                     f"{'upstream':>9s} {'route':>7s} {'dev':>6s} {'own':>11s} "
                     f"supp chosen\n")
            for c in sorted(r["cands"], key=lambda c: -(c["score_own"] or 0)):
                fh.write(f"{c['x']:8.2f} {c['y']:8.2f} {c['size']:5d} "
                         f"{(c['geo_m'] if c['geo_m'] is not None else float('nan')):7.2f} "
                         f"{c['euc_m']:7.2f} {c['score_upstream']:9.4f} "
                         f"{(c['route_policy_m'] if c['route_policy_m'] is not None else float('nan')):7.2f} "
                         f"{(c['dev_policy'] if c['dev_policy'] is not None else float('nan')):6.3f} "
                         f"{(c['score_own'] if c['score_own'] is not None else float('nan')):11.6f} "
                         f"{'S' if c['suppressed'] else ' '}    "
                         f"{'*' if c['is_chosen'] else ' '}\n")
            if c := r.get("terms_chosen"):
                fh.write(f"  weighted terms of the goal taken   "
                         f"{dict(zip(TERM_NAMES, r['terms_chosen']))}\n")
            if r.get("best_near_terms"):
                fh.write(f"  weighted terms of the best near    "
                         f"{dict(zip(TERM_NAMES, r['best_near_terms']))}\n")
            fh.write("\n")
    print(f"-> {os.path.join(HERE, 'resid_A_dumps.txt')}  "
          f"({len(resA)} class-A residuals under the remedy)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

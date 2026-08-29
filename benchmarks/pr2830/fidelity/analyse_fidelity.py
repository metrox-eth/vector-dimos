#!/usr/bin/env python3
"""Stage A / Stage B tables for sim_2830_fidelity.

Everything is re-derived from the RAW published goal coordinates of every run by
a second, independent pass; the disagreement with the field the bench wrote
while running is printed and is a deliverable of the job.

The swing metric is imported from ../sim_2830_cand2/analyse_cand2.py, unchanged,
so that a crossing here and a crossing in the go2 and resid reports are the same
object. Two things this file adds:

  * class N is PUBLISHED as its own column, "N-churn", never subtracted in
    silence. N = a raw swing whose new goal was within R_NEAR = 6 m of the robot
    AT THE MOMENT THE GOAL TOOK EFFECT: the goal sequence jumped across the
    floor and the robot did not have to move to serve it.
  * the faithful-temporality columns the bench now records: how many selection
    windows the robot spent still driving, how far it walked inside them, and
    the median staleness of a goal (how much the robot-to-goal distance moved
    between the pose the selector saw and the pose the goal took effect at).

usage: analyse_fidelity.py --results out/*.json --out stageA
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "sim_2830_cand2"))
from analyse_cand2 import R_NEAR_M, round_trips, swings_of  # noqa: E402

DISCRIMINATING = ["hk_office", "hk_park", "hk_elevator", "hk_entrance"]
SET_ASIDE = ["hk_allaround"]


def load(paths):
    rows, disagree, checked = [], 0, 0
    for p in paths:
        with open(p) as fh:
            d = json.load(fh)
        meta = d.get("meta", {})
        for r in d["results"]:
            s = dict(r["summary"])
            thr = s.get("swing_threshold_m")
            g = r["goals"]
            raw, real = swings_of(g, thr)
            s["swings_raw"] = len(raw)
            s["swings_real"] = len(real)
            # PUBLISHED, not dropped: goal churn without displacement.
            s["n_churn"] = len(raw) - len(real)
            s["round_trips_real"] = round_trips(real)
            s["round_trips_raw"] = round_trips(raw)
            s["goal_jump_total_m"] = sum(
                math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(g, g[1:]))
            if "cross_map_swings" in r["summary"]:
                checked += 1
                if r["summary"]["cross_map_swings"] != s["swings_raw"]:
                    disagree += 1
                    print(f"  DISAGREEMENT {s['map']}/{s['start']}/{s['config']}/"
                          f"{s['arm']}: stored {r['summary']['cross_map_swings']}, "
                          f"re-derived {s['swings_raw']}")
            s["lidar_m"] = meta.get("lidar_range_m")
            s["t_sel_s"] = s.get("t_sel_s", meta.get("t_sel_s", 0.0))
            s["profile"] = meta.get("profile")
            s["wall_capped"] = bool(s.get("wall_capped", False))
            s["_src"] = os.path.basename(p)
            # the goal-level records, kept for the identity check
            s["_goals"] = [(round(x["x"], 6), round(x["y"], 6)) for x in g]
            rows.append(s)
    print(f"re-derivation from raw goal coordinates: {checked} runs checked, "
          f"{disagree} disagreements with the stored cross_map_swings field")
    return rows, checked, disagree


def _med(v):
    return statistics.median(v) if v else float("nan")


def cell(rows, rng, cfg, tsel, mp, arm="stock"):
    return [r for r in rows if r["lidar_m"] == rng and r["config"] == cfg
            and abs(r["t_sel_s"] - tsel) < 1e-9 and r["map"] == mp and r["arm"] == arm]


def stage_a_table(rows, fh, tsels_by_cfg):
    ranges = sorted({r["lidar_m"] for r in rows})
    verdicts = {}
    for rng in ranges:
        for cfg in ("shipped", "scoring"):
            tsels = tsels_by_cfg.get(cfg, [])
            for t in tsels:
                any_here = [r for r in rows if r["lidar_m"] == rng
                            and r["config"] == cfg and abs(r["t_sel_s"] - t) < 1e-9]
                if not any_here:
                    continue
                print(f"\n=== range {rng:g} m | config {cfg} | T_sel {t:g} s "
                      f"| arm stock ===", file=fh)
                print(f"{'map':13s} {'n':>2s} {'REAL':>5s} {'N-churn':>8s} {'raw':>4s} "
                      f"{'rt':>3s} {'>=1':>4s} {'med goals':>9s} {'med path':>9s} "
                      f"{'med cov':>8s} {'sel drv':>8s} {'arrived':>8s} {'sel m':>7s} "
                      f"{'stale':>6s} {'cap':>4s}", file=fh)
                fired = 0
                present = 0
                for mp in DISCRIMINATING + SET_ASIDE:
                    rs = cell(rows, rng, cfg, t, mp)
                    if not rs:
                        continue
                    real = sum(r["swings_real"] for r in rs)
                    nch = sum(r["n_churn"] for r in rs)
                    raw = sum(r["swings_raw"] for r in rs)
                    rt = sum(r["round_trips_real"] for r in rs)
                    seldrv = sum(r.get("sel_driving_windows", 0) for r in rs)
                    selarr = sum(r.get("sel_arrived", 0) for r in rs)
                    selm = sum(r.get("sel_moved_m", 0.0) for r in rs)
                    stale = _med([r.get("sel_drift_median_m", float("nan")) for r in rs])
                    if mp in DISCRIMINATING:
                        present += 1
                        if real > 0:
                            fired += 1
                    tag = "" if mp in DISCRIMINATING else "  (set aside)"
                    print(f"{mp:13s} {len(rs):>2d} {real:>5d} {nch:>8d} {raw:>4d} "
                          f"{rt:>3d} {sum(1 for r in rs if r['swings_real']):>2d}/{len(rs):<1d} "
                          f"{_med([r['n_goals'] for r in rs]):>9.1f} "
                          f"{_med([r['path_m'] for r in rs]):>9.2f} "
                          f"{_med([r['coverage_pct'] for r in rs]):>7.1f}% "
                          f"{seldrv:>8d} {selarr:>8d} {selm:>7.1f} {stale:>6.2f} "
                          f"{sum(1 for r in rs if r['wall_capped']):>4d}{tag}", file=fh)
                if cfg == "shipped":
                    ok = fired >= 2
                    verdicts[f"{rng:g}m/T_sel={t:g}"] = {
                        "maps_with_a_real_crossing": fired, "maps_present": present,
                        "bar": "at least 2 of 4", "G1": "HOLDS" if ok else "FAILS"}
                    print(f"  -> G1 at {rng:g} m, T_sel {t:g} s: real crossings on "
                          f"{fired} of {present} discriminating maps. "
                          f"{'HOLDS' if ok else 'FAILS'} (bar: >= 2 of 4)", file=fh)
    return verdicts


def scoring_identity(rows, fh):
    """Pre-declared expectation: in config `scoring` no goal ever ends on a
    timeout, so no goal is ever live during a selection, so T_sel is pure
    standstill and the runs must be identical in space."""
    print(f"\n{'=' * 78}\nPRE-DECLARED CHECK: config `scoring` at T_sel 0 vs 15 must be "
          f"identical in space\n{'=' * 78}", file=fh)
    same = diff = 0
    for rng in sorted({r["lidar_m"] for r in rows}):
        for mp in DISCRIMINATING + SET_ASIDE:
            a = {r["start"]: r for r in cell(rows, rng, "scoring", 0.0, mp)}
            b = {r["start"]: r for r in cell(rows, rng, "scoring", 15.0, mp)}
            for st in sorted(set(a) & set(b)):
                ra, rb = a[st], b[st]
                fields = ["n_goals", "path_m", "area_m2", "coverage_pct",
                          "swings_real", "n_churn", "goals_reached", "goals_no_path"]
                bad = [f for f in fields
                       if not (ra[f] == rb[f] or (isinstance(ra[f], float)
                                                  and abs(ra[f] - rb[f]) < 1e-9))]
                if ra["_goals"] != rb["_goals"]:
                    bad.append("goal coordinates")
                if bad:
                    diff += 1
                    print(f"  DIFFERS {rng:g}m {mp}/{st}: {', '.join(bad)}", file=fh)
                else:
                    same += 1
    print(f"  {same} pairs identical, {diff} differ.", file=fh)
    return {"identical": same, "differ": diff}


def tsel_pairs(rows, fh, cfg="shipped"):
    """Paired per start: what the fidelity fix does to one and the same run.

    Same map, same start, same arm, same range, same config, only T_sel differs,
    so every difference below is the loop semantics and nothing else.
    """
    print(f"\n{'=' * 78}\nPAIRED BY START: T_sel 0 (frozen) against each faithful "
          f"value, config {cfg}\n{'=' * 78}", file=fh)
    out = {}
    for rng in sorted({r["lidar_m"] for r in rows}):
        base = {(r["map"], r["start"]): r for r in rows
                if r["lidar_m"] == rng and r["config"] == cfg
                and abs(r["t_sel_s"]) < 1e-9 and r["arm"] == "stock"}
        for t in sorted({r["t_sel_s"] for r in rows
                         if r["config"] == cfg and r["t_sel_s"] > 0}):
            cur = {(r["map"], r["start"]): r for r in rows
                   if r["lidar_m"] == rng and r["config"] == cfg
                   and abs(r["t_sel_s"] - t) < 1e-9 and r["arm"] == "stock"}
            keys = sorted(set(base) & set(cur))
            if not keys:
                continue
            up = sum(1 for k in keys if cur[k]["swings_real"] > base[k]["swings_real"])
            dn = sum(1 for k in keys if cur[k]["swings_real"] < base[k]["swings_real"])
            same = len(keys) - up - dn
            ca = sum(base[k]["swings_real"] for k in keys)
            cb = sum(cur[k]["swings_real"] for k in keys)
            na = sum(base[k]["n_churn"] for k in keys)
            nb = sum(cur[k]["n_churn"] for k in keys)
            pa = _med([base[k]["path_m"] for k in keys])
            pb = _med([cur[k]["path_m"] for k in keys])
            va = _med([base[k]["coverage_pct"] for k in keys])
            vb = _med([cur[k]["coverage_pct"] for k in keys])
            ga = _med([base[k]["n_goals"] for k in keys])
            gb = _med([cur[k]["n_goals"] for k in keys])
            print(f"{rng:>2g} m  T_sel 0 -> {t:<4g}  n={len(keys):2d} starts   "
                  f"REAL {ca:3d} -> {cb:<3d}  N-churn {na:2d} -> {nb:<2d}  "
                  f"per start up/flat/down {up}/{same}/{dn}   "
                  f"goals {ga:.1f} -> {gb:.1f}   path {pa:6.1f} -> {pb:6.1f} m   "
                  f"cov {va:5.1f} -> {vb:5.1f} %", file=fh)
            out[f"{rng:g}m/T{t:g}"] = dict(n=len(keys), real=[ca, cb], n_churn=[na, nb],
                                           up=up, flat=same, down=dn,
                                           path=[pa, pb], cov=[va, vb])
    return out


def plain_failures(rows, fh):
    """Every run that hit the wall cap, published nothing, or ended on a harness
    stop rather than on the explorer's own self-stop. Printed, never dropped."""
    print(f"\n{'=' * 78}\nPLAIN FAILURES AND CAPPED RUNS\n{'=' * 78}", file=fh)
    capped = [r for r in rows if r["wall_capped"]]
    empty = [r for r in rows if r["n_goals"] == 0]
    print(f"wall-capped runs (900 s): {len(capped)} of {len(rows)}", file=fh)
    for r in sorted(capped, key=lambda r: (r["lidar_m"], r["config"], r["t_sel_s"],
                                           r["map"], r["start"])):
        print(f"  {r['lidar_m']:>2g}m {r['config']:8s} T{r['t_sel_s']:<4g} "
              f"{r['map']:13s} {r['start']:7s} {r['arm']:11s} "
              f"goals {r['n_goals']:3d} path {r['path_m']:6.1f} m "
              f"cov {r['coverage_pct']:5.1f}%", file=fh)
    print(f"runs that published ZERO goals: {len(empty)}", file=fh)
    for r in empty:
        print(f"  {r['lidar_m']:>2g}m {r['config']:8s} T{r['t_sel_s']:<4g} "
              f"{r['map']:13s} {r['start']:7s} {r['arm']:11s} {r['end_reason']}", file=fh)
    reasons = {}
    for r in rows:
        key = r["end_reason"].split(":")[0]
        reasons[key] = reasons.get(key, 0) + 1
    print("end_reason distribution:", file=fh)
    for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print(f"  {v:4d}  {k}", file=fh)
    # the starts that are dead for every arm at a given (range, config)
    print("starts with <= 1 goal in EVERY cell they appear in:", file=fh)
    by_start = {}
    for r in rows:
        by_start.setdefault((r["map"], r["start"]), []).append(r["n_goals"])
    dead = [k for k, v in by_start.items() if max(v) <= 1]
    print("  " + (", ".join(f"{m}/{s}" for m, s in sorted(dead)) if dead else "none"),
          file=fh)
    return {"wall_capped": len(capped), "zero_goal_runs": len(empty),
            "end_reasons": reasons,
            "dead_starts": [f"{m}/{s}" for m, s in sorted(dead)]}


def stage_b_table(rows, fh, tsel, arm_a="stock", arm_b="stock+M4.3"):
    print(f"\n{'=' * 78}\nSTAGE B: {arm_a} vs {arm_b} at T_sel {tsel:g} s\n"
          f"per map, BOTH configs must hold: fewer real crossings, no more round\n"
          f"trips, median path within +5 %, median coverage within -5 points\n"
          f"{'=' * 78}", file=fh)
    out = {}
    for rng in sorted({r["lidar_m"] for r in rows}):
        for mp in DISCRIMINATING:
            per_cfg = {}
            for cfg in ("shipped", "scoring"):
                ra = cell(rows, rng, cfg, tsel, mp, arm_a)
                rb = cell(rows, rng, cfg, tsel, mp, arm_b)
                starts = sorted({r["start"] for r in ra} & {r["start"] for r in rb})
                if not starts:
                    continue
                ra = [r for r in ra if r["start"] in starts]
                rb = [r for r in rb if r["start"] in starts]
                ca, cb = sum(r["swings_real"] for r in ra), sum(r["swings_real"] for r in rb)
                na, nb = sum(r["n_churn"] for r in ra), sum(r["n_churn"] for r in rb)
                ta, tb = (sum(r["round_trips_real"] for r in ra),
                          sum(r["round_trips_real"] for r in rb))
                pa, pb = _med([r["path_m"] for r in ra]), _med([r["path_m"] for r in rb])
                va, vb = (_med([r["coverage_pct"] for r in ra]),
                          _med([r["coverage_pct"] for r in rb]))
                dp = 100.0 * (pb - pa) / pa if pa else float("nan")
                dv = vb - va
                why = []
                if ca == 0 and cb == 0:
                    verdict = "NO EVENTS"
                else:
                    if cb >= ca:
                        why.append("crossings did not drop")
                    if tb > ta:
                        why.append("round trips rose")
                    if dp > 5.0:
                        why.append(f"path +{dp:.1f} %")
                    if dv < -5.0:
                        why.append(f"coverage {dv:.1f} pt")
                    verdict = "PASS" if not why else "FAIL"
                per_cfg[cfg] = dict(n=len(starts), crossings=[ca, cb], n_churn=[na, nb],
                                    round_trips=[ta, tb], path=[pa, pb], cov=[va, vb],
                                    d_path_pct=dp, d_cov_pt=dv, verdict=verdict,
                                    why=why)
                print(f"{mp:13s} {rng:>2g}m {cfg:8s} n={len(starts)} "
                      f"crossings {ca:2d} -> {cb:2d}   N-churn {na:2d} -> {nb:2d}   "
                      f"rt {ta:2d} -> {tb:2d}   path {pa:6.2f} -> {pb:6.2f} m "
                      f"({dp:+6.1f} %)   cov {va:5.1f} -> {vb:5.1f} % ({dv:+5.1f} pt)"
                      f"   {verdict}{'  [' + '; '.join(why) + ']' if why else ''}",
                      file=fh)
            if per_cfg:
                vs = [v["verdict"] for v in per_cfg.values()]
                mapv = ("NO EVENTS" if all(v == "NO EVENTS" for v in vs)
                        else "PASS" if all(v in ("PASS",) for v in vs) else "FAIL")
                out[f"{rng:g}m/{mp}"] = {"per_config": per_cfg, "map_verdict": mapv}
                print(f"  -> {mp} at {rng:g} m: {mapv}\n", file=fh)
    passed = sum(1 for v in out.values() if v["map_verdict"] == "PASS")
    print(f"STAGE B: {passed} PASS of {len(out)} (map, range) cells", file=fh)
    return out


def write_csv(rows, path):
    cols = ["map", "start", "config", "arm", "lidar_m", "t_sel_s", "profile",
            "n_goals", "goals_reached", "goals_timed_out", "goals_no_path",
            "swings_raw", "swings_real", "n_churn", "round_trips_real",
            "round_trips_raw", "goal_jump_total_m", "swing_threshold_m",
            "path_m", "coverage_pct", "area_m2", "ceiling_m2", "sim_s",
            "sel_windows", "sel_driving_windows", "sel_moved_m", "sel_time_s",
            "sel_arrived", "sel_blocked", "sel_drift_median_m",
            "d_robot_to_goal_median_m", "d_robot_dec_median_m",
            "decide_ms_mean", "wall_capped", "end_reason", "_src"]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r["lidar_m"], r["config"], r["t_sel_s"],
                                             r["map"], r["start"], r["arm"])):
            w.writerow(r)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--results", nargs="+", required=True)
    ap.add_argument("--out", default="stageA")
    ap.add_argument("--stage-b-tsel", type=float, default=None)
    args = ap.parse_args(argv)
    paths = []
    for p in args.results:
        paths.extend(sorted(glob.glob(p)))
    rows, checked, disagree = load(paths)
    print(f"{len(rows)} runs loaded from {len(paths)} files")

    txt = os.path.join(HERE, args.out + ".txt")
    with open(txt, "w") as fh:
        print(f"sim_2830_fidelity - {args.out}\n"
              f"{len(rows)} runs, re-derived from raw goal coordinates: "
              f"{checked} checked, {disagree} disagreements\n"
              f"N-churn = class N = the goal sequence jumped and the robot did not "
              f"(new goal within R_NEAR = {R_NEAR_M:g} m of the robot when the goal "
              f"took effect). Published, never dropped.\n"
              f"sel drv = selection windows the robot spent still driving to the OLD "
              f"goal; sel m = metres walked inside them; stale = median |d(robot,goal) "
              f"at publish - at decision| in m.", file=fh)
        tsels = {"shipped": sorted({r["t_sel_s"] for r in rows if r["config"] == "shipped"}),
                 "scoring": sorted({r["t_sel_s"] for r in rows if r["config"] == "scoring"})}
        v = stage_a_table(rows, fh, tsels)
        pairs = tsel_pairs(rows, fh, "shipped")
        ident = scoring_identity(rows, fh) if len(tsels["scoring"]) > 1 else None
        b = (stage_b_table(rows, fh, args.stage_b_tsel)
             if args.stage_b_tsel is not None else None)
        fails = plain_failures(rows, fh)
    print(open(txt).read())
    write_csv(rows, os.path.join(HERE, args.out + ".csv"))
    with open(os.path.join(HERE, args.out + "_verdicts.json"), "w") as fh:
        json.dump({"g1": v, "tsel_pairs": pairs, "scoring_identity": ident,
                   "stage_b": b, "failures": fails,
                   "rederivation": {"checked": checked, "disagreements": disagree},
                   "n_runs": len(rows)}, fh, indent=1)
    print(f"-> {txt}, {args.out}.csv, {args.out}_verdicts.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())

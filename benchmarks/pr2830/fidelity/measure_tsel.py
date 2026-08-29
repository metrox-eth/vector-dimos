#!/usr/bin/env python3
"""The isolated measurement of T_sel: what one dimOS frontier selection costs.

Every decision of every run in the probe is one timed call to their
`get_exploration_goal` (config `shipped`), i.e. `detect_frontiers` - a full-grid
pure-python wavefront BFS - plus their ranking. The bench times exactly that call
and records it per goal (`decide_ms`).

WHAT THIS MEASURES AND WHAT IT DOES NOT
  - it measures OUR grids on OUR silicon, uncontended. It is an anchor for the
    ORDER of magnitude of T_sel, not a claim about the robot's onboard compute.
  - decisions that returned no goal are not in the sample: the bench only writes
    a decide_ms with a published goal. Those decisions cost a full detect_frontiers
    too, so the sample is if anything an under-count of the work done.

usage: measure_tsel.py probe1.json [probe2.json ...] > t_sel_measurement.txt
"""
from __future__ import annotations

import json
import statistics
import sys


def q(v, p):
    v = sorted(v)
    if not v:
        return float("nan")
    i = min(len(v) - 1, max(0, int(round(p * (len(v) - 1)))))
    return v[i]


def main(argv):
    per_map, alls = {}, []
    meta = None
    for p in argv:
        d = json.load(open(p))
        meta = meta or d.get("meta", {})
        for r in d["results"]:
            s = r["summary"]
            k = (s["map"], s["config"], d["meta"].get("lidar_range_m"))
            for g in r["goals"]:
                per_map.setdefault(k, []).append(g["decide_ms"] / 1000.0)
                alls.append(g["decide_ms"] / 1000.0)
    print("ISOLATED MEASUREMENT OF THE dimOS FRONTIER SELECTION (T_sel anchor)")
    print("=" * 78)
    print("rig      : 12th Gen Intel Core i9-12900KF, 24 threads")
    print("load     : 6 concurrent workers on 24 threads, nothing else running")
    print(f"profile  : {meta.get('profile')}, arm stock, config shipped, "
          f"lidar {meta.get('lidar_range_m')} m, T_sel {meta.get('t_sel_s')}")
    print("timed    : one call to WavefrontFrontierExplorer.get_exploration_goal")
    print("           per decision, which is detect_frontiers (full-grid python")
    print("           BFS) + _rank_frontiers, on the 0.25 m-inflated costmap")
    print()
    print(f"{'map':14s} {'cfg':8s} {'lidar':>5s} {'n':>4s} {'median':>8s} "
          f"{'p10':>7s} {'p90':>7s} {'max':>8s}")
    for k in sorted(per_map):
        v = per_map[k]
        print(f"{k[0]:14s} {k[1]:8s} {k[2]:>5g} {len(v):>4d} "
              f"{statistics.median(v):>8.2f} {q(v, 0.1):>7.2f} {q(v, 0.9):>7.2f} "
              f"{max(v):>8.2f}")
    print()
    print(f"POOLED over {len(alls)} decisions: median {statistics.median(alls):.2f} s, "
          f"p10 {q(alls, 0.1):.2f} s, p90 {q(alls, 0.9):.2f} s, max {max(alls):.2f} s")
    print()
    print("CROSS-CHECK, the recorded go2 bench's own decide_ms_mean per run, on the")
    print("384-core box under 24-60 concurrent workers, so an UPPER bound:")
    print("  4 m : hk_park 11.2 s, hk_entrance 12.4 s, hk_office 15.5 s,")
    print("        hk_elevator 16.8 s, hk_allaround 34.6 s")
    print("  12 m: hk_park 10.6 s, hk_entrance 11.5 s, hk_office 14.9 s,")
    print("        hk_elevator 16.1 s")
    print()
    print("READING. The sweep {0, 5, 15} s was fixed by hypotheses_fidelity.txt")
    print("before this ran and is not changed by it. What this number decides is")
    print("only which faithful value Stage B runs at, by the pre-declared rule:")
    print("the swept faithful value closest to the median above, ties to the smaller.")
    m = statistics.median(alls)
    pick = 5.0 if abs(m - 5.0) <= abs(m - 15.0) else 15.0
    print(f"  measured median {m:.2f} s  ->  Stage B T_sel = {pick:g} s")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

#!/usr/bin/env python3
"""Does the modified harness, at T_sel = 0, reproduce the recorded go2 bench?

The T_sel = 0 arm is supposed to be the old bench bit for bit. This checks it
against runs that were recorded before any of this job's code existed, on 13
summary fields plus EVERY goal coordinate, and additionally checks that the two
new pose fields coincide and that no selection window ever fired.

    repro_check.py new.json recorded.json

Note the key: (map, start, config, arm). A result file can hold several arms and
keying on the start alone silently compares a stock run against a policy run -
which is a mistake this script made once and is why the key is spelled out here.
"""
from __future__ import annotations

import json
import math
import sys

FIELDS = ["n_goals", "goals_reached", "goals_timed_out", "goals_no_path",
          "path_m", "area_m2", "coverage_pct", "sim_s", "cross_map_swings",
          "goal_jump_total_m", "reversals", "impacts", "end_reason"]


def main(argv):
    new = json.load(open(argv[0]))
    old = json.load(open(argv[1]))
    oi = {(r["summary"]["map"], r["summary"]["start"], r["summary"]["config"],
           r["summary"]["arm"]): r for r in old["results"]}
    bad = n = pex = 0
    for r in new["results"]:
        s = r["summary"]
        k = (s["map"], s["start"], s["config"], s["arm"])
        o = oi.get(k)
        if o is None:
            print(f"  NO COUNTERPART {k}")
            continue
        n += 1
        pex += s.get("goals_path_exhausted", 0)
        for f in FIELDS:
            a, b = s[f], o["summary"][f]
            if isinstance(a, float) and isinstance(b, float):
                if not (abs(a - b) <= 1e-9 or (math.isnan(a) and math.isnan(b))):
                    print(f"  DIFF {k} {f}: {a} vs {b}")
                    bad += 1
            elif a != b:
                print(f"  DIFF {k} {f}: {a} vs {b}")
                bad += 1
        ga = [(round(g["x"], 6), round(g["y"], 6)) for g in r["goals"]]
        gb = [(round(g["x"], 6), round(g["y"], 6)) for g in o["goals"]]
        if ga != gb:
            print(f"  GOAL COORDINATES DIFFER {k}")
            bad += 1
        if any(g["sel_moved_m"] or g["sel_s"] for g in r["goals"]):
            print(f"  A SELECTION WINDOW FIRED AT T_sel=0 {k}")
            bad += 1
        if any(abs(g["from_x"] - g["dec_x"]) > 1e-12
               or abs(g["from_y"] - g["dec_y"]) > 1e-12 for g in r["goals"]):
            print(f"  PUBLISH POSE != DECISION POSE AT T_sel=0 {k}")
            bad += 1
    print(f"{n} runs compared on {len(FIELDS)} summary fields plus every goal "
          f"coordinate: {bad} disagreements. {pex} goals flagged path-exhausted.")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))

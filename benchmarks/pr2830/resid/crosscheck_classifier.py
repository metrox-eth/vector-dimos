#!/usr/bin/env python3
"""Does the classifier used here assign the same class as the fix job's?

diagnose_resid.classify_run is diagnose_swings.classify_run with two additions
(the policy's own score standing in for the upstream one on a policy arm, and
the term decomposition). On a BARE arm the two must be the same function, so
every stock swing traced here is classified by both and the labels compared.
Any disagreement is printed. The policy arm is not comparable by construction
and is excluded, and the number excluded is printed.
"""
from __future__ import annotations

import glob
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    sys.modules[name] = m
    spec.loader.exec_module(m)
    return m


def main():
    ref = load("ref", os.path.join(HERE, "diagnose_swings_ref.py"))
    new = load("new", os.path.join(HERE, "diagnose_resid.py"))
    import json
    same = diff = skipped = 0
    rows = []
    for p in sorted(glob.glob(os.path.join(HERE, "resid_*m_hk_*.json"))):
        with open(p) as fh:
            d = json.load(fh)
        rng = "12m" if abs(d["meta"]["lidar_range_m"] - 12.0) < 1e-6 else "4m"
        for r in d["results"]:
            if r["summary"]["arm"] != "stock":
                skipped += len([t for t in r.get("trace", []) if t.get("is_swing")])
                continue
            a = ref.classify_run(r)
            b = new.classify_run(r, rng)
            if len(a) != len(b):
                diff += 1
                rows.append(f"{r['summary']} different swing counts "
                            f"{len(a)} vs {len(b)}")
                continue
            for x, y in zip(a, b):
                if x["cls"] == y["cls"]:
                    same += 1
                else:
                    diff += 1
                    rows.append(f"{y['range']} {y['map']} {y['config']} "
                                f"{y['start']} {y['arm']} goal {y['goal_index']}: "
                                f"fix job says {x['cls']}, this job says {y['cls']}")
    print(f"stock swings classified by both classifiers: {same} agree, "
          f"{diff} disagree ({skipped} policy-arm swings excluded: the two "
          f"classifiers read a different score there by design)")
    for r in rows:
        print("  " + r)
    return 0


if __name__ == "__main__":
    sys.exit(main())

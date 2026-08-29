#!/usr/bin/env python3
"""shadow.json -> the disagreement numbers the report quotes."""

from __future__ import annotations

import json
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main(argv=None):
    args = argv or sys.argv[1:]
    path = args[0] if args else os.path.join(HERE, "shadow.json")
    rows = json.load(open(path))["rows"]
    out = []

    def p(s):
        out.append(s)
        print(s)

    p(f"{len(rows)} points de decision, etat de carte identique pour les deux scoreurs")
    p("")
    for driver in ("stock", "pr2830"):
        r = [x for x in rows if x["driver"] == driver]
        if not r:
            continue
        same = sum(1 for x in r if x["same"])
        p(f"--- le conducteur est {driver} ({len(r)} decisions) ---")
        p(f"  meme frontiere choisie par les deux : {same}/{len(r)} "
          f"({100 * same / len(r):.0f} %)")
        diff = [x for x in r if not x["same"]]
        if diff:
            dd = [x["driver_euclid"] for x in diff]
            ds = [x["shadow_euclid"] for x in diff if x["shadow_euclid"] is not None]
            p(f"  quand ils different ({len(diff)}): distance a vol d'oiseau du but choisi")
            p(f"     {driver:7s} med {statistics.median(dd):5.2f} m  max {max(dd):5.2f} m")
            if ds:
                shadow = diff[0]["shadow"]
                p(f"     {shadow:7s} med {statistics.median(ds):5.2f} m  max {max(ds):5.2f} m")
                closer = sum(1 for x in diff if x["shadow_euclid"] is not None
                             and x["shadow_euclid"] < x["driver_euclid"])
                p(f"     le choix de {shadow} est plus proche dans {closer}/{len(diff)} cas")
        p("")

    # how far the real route is from the straight line, on our maps
    ratios = [x["driver_path"] / x["driver_euclid"] for x in rows
              if x["driver_path"] and x["driver_euclid"] and x["driver_euclid"] > 0.3]
    ratios += [x["shadow_path"] / x["shadow_euclid"] for x in rows
               if x["shadow_path"] and x["shadow_euclid"] and x["shadow_euclid"] > 0.3]
    ratios.sort()
    if ratios:
        p("--- ecart entre le chemin A* reel (l'instrument de #2830) et la ligne droite ---")
        p(f"  n={len(ratios)}  median {statistics.median(ratios):.2f} x  "
          f"p90 {ratios[int(0.9 * (len(ratios) - 1))]:.2f} x  max {max(ratios):.2f} x")
        p(f"  part des buts ou le detour depasse +50 % : "
          f"{100 * sum(1 for v in ratios if v > 1.5) / len(ratios):.1f} %")
        p(f"  part des buts ou le detour depasse +20 % : "
          f"{100 * sum(1 for v in ratios if v > 1.2) / len(ratios):.1f} %")
    p("")
    by_map = {}
    for x in rows:
        by_map.setdefault(x["map"], []).append(x)
    p("--- par carte ---")
    for m, r in sorted(by_map.items()):
        same = sum(1 for x in r if x["same"])
        p(f"  {m:16s} {len(r):3d} decisions, meme choix {100 * same / len(r):3.0f} %")

    with open(os.path.join(HERE, "agregats_shadow.txt"), "w") as fh:
        fh.write("\n".join(out) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

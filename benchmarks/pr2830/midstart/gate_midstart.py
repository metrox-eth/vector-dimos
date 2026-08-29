#!/usr/bin/env python3
"""Input-worthiness gate for the mid-start re-run, at a chosen lidar range.

Same question as traverse.py in the shipped bench, asked per START and at the
new range: how much of what there is to see from here can only be seen after a
walk longer than one lidar revolution?

If that share is near zero the map cannot reward or punish a travel decision
from that start - whatever the explorer picks, it sees the same thing - and the
start is not a worthy input for the travel half of the bench. The shipped
12 m bench sat at 13.8 % on this map, which was thin; the point of dropping the
range to 4 m is to make it thick.

A start is also dropped as DEGENERATE if one revolution from the spawn already
reveals DEGENERATE_ONE_TURN_PCT of everything that start can ever see.

    gate_midstart.py --range 4.0 --out gate_4m.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from collections import deque

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import midstarts as MS  # noqa: E402

ES = MS.ES

DEGENERATE_ONE_TURN_PCT = 50.0


def geodesic(mask: np.ndarray, start_yx: tuple[int, int]) -> np.ndarray:
    """Geodesic distance in CELLS inside `mask`, 8-connected (traverse.py's BFS)."""
    from scipy import ndimage
    d = np.full(mask.shape, np.inf)
    sy, sx = start_yx
    if not mask[sy, sx]:
        _, idx = ndimage.distance_transform_edt(~mask, return_indices=True)
        sy, sx = int(idx[0][sy, sx]), int(idx[1][sy, sx])
    d[sy, sx] = 0.0
    q = deque([(sy, sx)])
    while q:
        y, x = q.popleft()
        for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1] and mask[ny, nx]:
                nd = d[y, x] + (1.4142 if dy and dx else 1.0)
                if nd < d[ny, nx]:
                    d[ny, nx] = nd
                    q.append((ny, nx))
    return d


def gate_start(world, start, lidar_m: float, lattice_m: float = 0.5) -> dict:
    """Ring breakdown of what is visible from `start`, in units of lidar_m."""
    res = world.res
    body = MS.passable(world)
    gy, gx = world.cell(*start)
    d = geodesic(body, (gy, gx)) * res

    step = max(1, int(round(lattice_m / res)))
    ys, xs = np.nonzero(body)
    order = np.argsort(d[ys, xs])
    seen = np.full(world.truth.shape, ES.UNKNOWN, dtype=np.int8)

    rings = [lidar_m * k for k in range(1, 8)] + [float("inf")]
    prev, lines = 0.0, []
    for ring in rings:
        before = float((seen != ES.UNKNOWN).sum()) * res * res
        for k in order:
            y, x = int(ys[k]), int(xs[k])
            if d[y, x] > ring or d[y, x] <= prev:
                continue
            if y % step or x % step:
                continue
            ES.scan(world, seen, *world.world_xy(y, x))
        after = float((seen != ES.UNKNOWN).sum()) * res * res
        lines.append({"from_m": prev, "to_m": None if ring == float("inf") else ring,
                      "new_m2": after - before, "cum_m2": after})
        prev = ring
    ceiling = lines[-1]["cum_m2"]

    one = np.full(world.truth.shape, ES.UNKNOWN, dtype=np.int8)
    ES.scan(world, one, *start)
    one_m2 = float((one != ES.UNKNOWN).sum()) * res * res

    far = sum(r["new_m2"] for r in lines if r["from_m"] >= lidar_m - 1e-9)
    finite = d[np.isfinite(d)]
    return {
        "start_xy": list(start),
        "lidar_m": lidar_m,
        "ceiling_m2": ceiling,
        "one_revolution_m2": one_m2,
        "one_revolution_pct_of_ceiling": 100.0 * one_m2 / ceiling if ceiling else float("nan"),
        "geodesic_max_m": float(finite.max()) if finite.size else 0.0,
        "beyond_one_range_m2": far,
        "beyond_one_range_pct": 100.0 * far / ceiling if ceiling else float("nan"),
        "rings": lines,
        "degenerate": bool(ceiling and 100.0 * one_m2 / ceiling >= DEGENERATE_ONE_TURN_PCT),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--range", type=float, default=4.0)
    ap.add_argument("--maps", nargs="*", default=["bigoffice", "bigoffice_hc"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    ES.LIDAR_RANGE_M = args.range
    assert ES.LIDAR_RANGE_M == args.range

    out = {"lidar_range_m": args.range, "maps": {}}
    for name in args.maps:
        path = os.path.join(SCRATCH, f"{name}.npz")
        if not os.path.exists(path):
            continue
        world = ES.load_world(path, None, unknown_is_wall=True)
        starts, meta = MS.choose_mid_starts(world, verbose=False)
        diag = MS.body_bbox_diagonal_m(world)
        print(f"\n=== {name} ===  lidar {args.range:.1f} m  "
              f"passable-floor bbox diagonal {diag:.1f} m  "
              f"(cross-map swing threshold {diag / 2:.1f} m)")
        print(f"{'start':8s} {'x':>7s} {'y':>7s} {'clear':>6s} {'ways':>5s} "
              f"{'ceiling':>8s} {'1 turn':>8s} {'1 turn %':>9s} {'geo max':>8s} "
              f"{'beyond 1 range':>15s}")
        rows = {}
        for sname, s in starts.items():
            g = gate_start(world, s, args.range)
            g["clearance_m"] = meta["starts"][sname]["clearance_m"]
            g["open_directions"] = meta["starts"][sname]["open_directions"]
            g["eccentricity_m"] = meta["starts"][sname]["eccentricity_m"]
            rows[sname] = g
            print(f"{sname:8s} {s[0]:7.2f} {s[1]:7.2f} "
                  f"{g['clearance_m']:6.2f} {g['open_directions']:5d} "
                  f"{g['ceiling_m2']:7.1f}m2 {g['one_revolution_m2']:7.1f}m2 "
                  f"{g['one_revolution_pct_of_ceiling']:8.1f}% {g['geodesic_max_m']:7.1f}m "
                  f"{g['beyond_one_range_pct']:14.1f}%"
                  + ("   DEGENERATE" if g["degenerate"] else ""))
        out["maps"][name] = {"selection": meta, "bbox_diagonal_m": diag, "starts": rows}

    if args.out:
        with open(args.out, "w") as fh:
            json.dump(out, fh, indent=1)
        print(f"\n-> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

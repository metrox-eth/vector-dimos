#!/usr/bin/env python3
"""How much of what there is to see can only be seen after a long walk?

The sharpest dignity test for the TRAVEL half of the bench. For a given start:
  * geodesic distance (inside the body-passable floor) from the start to every
    reachable cell;
  * for each 4 m ring of that distance, the NEW area a lidar revolution from
    that ring reveals, that no closer ring already revealed.

A map where 100 % of the ceiling is revealed from within one lidar range of the
start cannot reward or punish a travel decision: whatever the explorer picks, it
sees the same thing. A map where a large share only appears past 12 m can.

    traverse.py ../bigoffice.npz ../costmap_175905.npz ...
"""

from __future__ import annotations

from collections import deque
import importlib.util
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)


def _import_explore_sim():
    spec = importlib.util.spec_from_file_location(
        "explore_sim", "/home/openclaw/vector-dimos/tools/explore_sim.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()
sys.path.insert(0, HERE)
import bench_2830 as B  # noqa: E402


def geodesic(mask, start_yx):
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


def report(path, lattice_m=0.5, rings=(4, 8, 12, 16, 20, 24, 1e9)):
    world = ES.load_world(path, None, unknown_is_wall=True)
    res = world.res
    body = B._passable(world)
    pose = np.load(path)["pose_xy"]
    gy, gx = world.cell(float(pose[0]), float(pose[1]))
    if not body[gy, gx]:
        ys, xs = np.nonzero(body)
        cy, cx = ys.mean(), xs.mean()
        i = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
        gy, gx = int(ys[i]), int(xs[i])
    start = world.world_xy(gy, gx)
    d = geodesic(body, (gy, gx)) * res

    step = max(1, int(round(lattice_m / res)))
    seen = np.full(world.truth.shape, ES.UNKNOWN, dtype=np.int8)
    ys, xs = np.nonzero(body)
    order = np.argsort(d[ys, xs])
    prev = 0.0
    print(f"\n=== {os.path.basename(path)} ===   depart ({start[0]:.2f}, {start[1]:.2f})")
    print(f"  distance geodesique max dans le sol praticable : "
          f"{np.nanmax(d[np.isfinite(d)]):.1f} m")
    total = 0.0
    lines = []
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
        total = after
        lines.append((prev, ring, after - before, after))
        prev = ring
    for lo, hi, new, cum in lines:
        if new <= 0 and cum == 0:
            continue
        hi_s = "inf" if hi > 1e8 else f"{hi:.0f}"
        print(f"  a {lo:4.0f}-{hi_s:>3s} m du depart : +{new:6.1f} m2 nouveaux  "
              f"(cumul {cum:6.1f} m2 = {100 * cum / total:5.1f} % du plafond)")
    far = sum(new for lo, hi, new, cum in lines if lo >= ES.LIDAR_RANGE_M)
    print(f"  part du plafond visible SEULEMENT depuis plus de "
          f"{ES.LIDAR_RANGE_M:.0f} m de marche : {100 * far / total:.1f} %")
    return total


def main():
    paths = sys.argv[1:] or [os.path.join(SCRATCH, "bigoffice.npz")]
    for p in paths:
        report(p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

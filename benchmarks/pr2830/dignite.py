#!/usr/bin/env python3
"""Is this map a dignified input for the TRAVEL half of the bench?

The travel claim of #2830 ("-15 to -29 % of path for the same coverage") can
only be tested on a map where there is something to walk ACROSS. Our flat was
not: 10 m wide, a 12 m lidar, so one revolution from anywhere sees most of it.

This script measures, on the extracted map, the three numbers that decide it:
  1. the extent of the map and of its body-passable floor;
  2. the geodesic diameter of that floor - the longest walk the rover can be
     asked to make - in units of the simulated 12 m lidar range;
  3. what one lidar revolution from a start actually reveals, as a fraction of
     the ceiling that start can ever reach. If that ratio is near 1, the map
     cannot separate two exploration strategies.

Printed for the big office AND for the four flat maps of the previous bench,
so the comparison is on the page rather than in the prose.
"""

from __future__ import annotations

import importlib.util
import os
import sys

import numpy as np
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)


def _import_explore_sim():
    path = "/home/openclaw/vector-dimos/tools/explore_sim.py"
    spec = importlib.util.spec_from_file_location("explore_sim", path)
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()


def passable(world):
    ok = (world.truth == ES.FREE) & (world.clearance + 1e-6 >= ES.BODY_HALF_WIDTH_M)
    labels, n = ndimage.label(ok, structure=ES._EIGHT)
    if n == 0:
        return ok, ok
    sizes = ndimage.sum(ok, labels, index=np.arange(1, n + 1))
    biggest = labels == (int(np.argmax(sizes)) + 1)
    return ok, biggest


def geodesic_diameter(mask: np.ndarray, res: float) -> float:
    """Longest shortest-path inside `mask`, by double BFS (8-connected)."""
    from collections import deque

    ys, xs = np.nonzero(mask)
    if not len(ys):
        return 0.0

    def bfs(sy, sx):
        d = np.full(mask.shape, -1.0)
        d[sy, sx] = 0.0
        q = deque([(sy, sx)])
        far, fd = (sy, sx), 0.0
        while q:
            y, x = q.popleft()
            for dy, dx in ((-1, 0), (1, 0), (0, -1), (0, 1),
                           (-1, -1), (-1, 1), (1, -1), (1, 1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < mask.shape[0] and 0 <= nx < mask.shape[1] \
                        and mask[ny, nx] and d[ny, nx] < 0:
                    d[ny, nx] = d[y, x] + (1.4142 if dy and dx else 1.0)
                    if d[ny, nx] > fd:
                        far, fd = (ny, nx), d[ny, nx]
                    q.append((ny, nx))
        return far, fd

    a, _ = bfs(int(ys[0]), int(xs[0]))
    _, d = bfs(*a)
    return d * res


def report(name: str, path: str) -> None:
    world = ES.load_world(path, None, unknown_is_wall=True)
    h, w = world.truth.shape
    ok, body = passable(world)
    res = world.res
    print(f"\n=== {name} ===")
    print(f"  grille          {w} x {h} cellules @ {res} m = {w * res:.1f} x {h * res:.1f} m")
    print(f"  observe         {world.observed_area_m2:.1f} m2")
    print(f"  sol libre       {world.free_area_m2:.1f} m2")
    print(f"  sol praticable  {ok.sum() * res * res:.1f} m2 "
          f"(plus grande piece connexe {body.sum() * res * res:.1f} m2)")
    ys, xs = np.nonzero(body)
    if len(ys):
        bw = (xs.max() - xs.min() + 1) * res
        bh = (ys.max() - ys.min() + 1) * res
        print(f"  emprise du sol  {bw:.1f} x {bh:.1f} m  "
              f"(diagonale {np.hypot(bw, bh):.1f} m)")
        diam = geodesic_diameter(body, res)
        print(f"  diametre geodesique du sol praticable  {diam:.1f} m "
              f"= {diam / ES.LIDAR_RANGE_M:.2f} x la portee lidar simulee "
              f"({ES.LIDAR_RANGE_M:.0f} m)")
        # what one revolution sees, from the centre of the passable floor
        cy, cx = ys.mean(), xs.mean()
        i = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
        start = world.world_xy(int(ys[i]), int(xs[i]))
        seen = np.full(world.truth.shape, ES.UNKNOWN, dtype=np.int8)
        ES.scan(world, seen, *start)
        one = float((seen != ES.UNKNOWN).sum()) * res * res
        ceiling = world.visible_area_m2(start)
        print(f"  un tour de lidar depuis le centre : {one:.1f} m2 "
              f"= {100 * one / ceiling:.1f} % du plafond visible ({ceiling:.1f} m2)")


def main() -> int:
    maps = [
        ("go2_bigoffice (dimOS)", os.path.join(SCRATCH, "bigoffice.npz")),
        ("map_20260823 (appart)", os.path.join(SCRATCH, "map_20260823.npz")),
        ("map_20260825 (appart)", os.path.join(SCRATCH, "map_20260825.npz")),
        ("costmap_175224 (appart)", os.path.join(SCRATCH, "costmap_175224.npz")),
        ("costmap_175905 (appart)", os.path.join(SCRATCH, "costmap_175905.npz")),
    ]
    for name, path in maps:
        if os.path.exists(path):
            report(name, path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Ground-truth check of the extraction chain, using dimensional's own pair.

The dataset ships `big_office.ply` (their accumulated global cloud) next to
`big_office_height_cost_occupancy.png` (what their pipeline makes of it). If our
call of `height_cost_occupancy` on that .ply reproduces that PNG, the chain
between "a cloud" and "an occupancy grid" is theirs, not ours.

The comparison is STRUCTURAL, not pixel-colour: the PNG's colormap is not
documented, so we compare the two things a colormap cannot hide -
  * the unknown mask  (PNG: pure black; grid: cost == -1)
  * the zero-cost mask (PNG: the single most common non-black colour;
                        grid: cost == 0)
by intersection-over-union, after matching orientation (an OccupancyGrid has
its origin bottom-left, a PNG its row 0 at the top).
"""

from __future__ import annotations

import os

import numpy as np
from PIL import Image

import extract_bigoffice as X

HERE = os.path.dirname(os.path.abspath(__file__))
PNG = "/home/openclaw/lerobot/dimos_datasets/big_office_height_cost_occupancy.png"


def iou(a: np.ndarray, b: np.ndarray) -> float:
    u = (a | b).sum()
    return float((a & b).sum() / u) if u else float("nan")


def main() -> int:
    cloud = X.read_ply()
    print(f"big_office.ply: {len(cloud)} points, passed to height_cost_occupancy "
          f"with no preprocessing of ours")
    og = X.height_cost(cloud)
    grid = np.asarray(og.grid)
    print(f"our grid: {grid.shape[1]} x {grid.shape[0]} @ {og.resolution} m, "
          f"origin ({og.origin.position.x:.3f}, {og.origin.position.y:.3f})")
    np.save(os.path.join(HERE, "grid_ply.npy"), grid)

    img = np.asarray(Image.open(PNG).convert("RGB"))
    print(f"their PNG:  {img.shape[1]} x {img.shape[0]} px")
    if img.shape[:2] != grid.shape:
        print("  !! shapes differ - no pixel comparison possible")
        return 1

    black = (img.sum(axis=2) == 0)
    cols, counts = np.unique(img[~black].reshape(-1, 3), axis=0, return_counts=True)
    zero_colour = cols[int(np.argmax(counts))]
    print(f"  most common non-black colour: {tuple(int(v) for v in zero_colour)} "
          f"({100 * counts.max() / (~black).sum():.1f} % of the drawn pixels) "
          f"-> read as cost 0")
    png_unknown = black
    png_zero = np.all(img == zero_colour, axis=2)

    best, best_score = None, -1.0
    for name, flip in (("as stored", False), ("vertically flipped", True)):
        g = grid[::-1] if flip else grid
        u = iou(png_unknown, g == -1)
        z = iou(png_zero, g == 0)
        print(f"  {name:20s}  unknown IoU {u:.4f}   cost-0 IoU {z:.4f}")
        if u + z > best_score:
            best, best_score = (name, g), u + z

    name, g = best
    print()
    print(f"orientation that matches: {name}")
    print(f"  unknown cells   theirs {png_unknown.sum():7d}   ours {(g == -1).sum():7d}")
    print(f"  cost-0 cells    theirs {png_zero.sum():7d}   ours {(g == 0).sum():7d}")
    dis = (png_unknown != (g == -1))
    print(f"  cells where 'unknown' disagrees: {dis.sum()} "
          f"({100 * dis.mean():.3f} % of the grid)")

    # Stronger than two masks: if the PNG is a colormap of OUR grid, then every
    # cell carrying the same cost must carry the same colour, and two different
    # costs must not share one. That is checkable without knowing the colormap.
    print()
    print("colour <-> cost consistency (a colormap is a function, so this must hold):")
    flat_cost = g.reshape(-1)
    flat_rgb = img.reshape(-1, 3).astype(np.int64)
    packed = (flat_rgb[:, 0] << 16) | (flat_rgb[:, 1] << 8) | flat_rgb[:, 2]
    costs = np.unique(flat_cost)
    bad = 0
    colours_seen: dict[int, int] = {}
    for c in costs:
        m = flat_cost == c
        u = np.unique(packed[m])
        if len(u) != 1:
            bad += 1
            print(f"  cost {int(c):4d}: {len(u)} distinct colours over {int(m.sum())} cells")
        else:
            colours_seen.setdefault(int(u[0]), 0)
            colours_seen[int(u[0])] += 1
    collisions = sum(1 for v in colours_seen.values() if v > 1)
    print(f"  {len(costs)} distinct cost values; "
          f"{bad} of them map to more than one colour; "
          f"{collisions} colours shared by more than one cost")
    if bad == 0 and collisions == 0:
        print("  -> the PNG is exactly a colour lookup of this grid, cell for cell.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

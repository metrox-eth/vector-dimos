#!/usr/bin/env python3
"""go2_bigoffice.db -> the map the bench eats (an explore_sim .npz).

    extract_bigoffice.py --algo simple      # the bench's input
    extract_bigoffice.py --algo height_cost # the robustness map
    extract_bigoffice.py --algo general

This is the ONLY thing added to the bench for this run: an input adapter. The
scoring side (dimos_selector.py, pr2830/, the selector modules) is untouched,
and bench_2830.py changes by exactly two data lines (the map name and its file).

Step 1 - the cloud
------------------
The store holds four streams and NONE of them is a costmap or a saved map:

    odom 5465 · lidar 2251 · color_image 4164 · color_image_embedded 267

So the map has to be computed from the lidar, and the accumulation is trivial
because the frames are recorded in the WORLD frame already (frame_id ==
"world"): concatenate the 2251 frames (55.8 M returns), then keep one return
per 0.05 m voxel to fit in memory - coordinates are never snapped, and the
occupancy kernels bin on their own grid afterwards. No registration, no pose
maths of ours. Result: 699 637 points over 26.1 x 36.9 m.

Step 2 - the occupancy grid
---------------------------
Their module, `dimos.mapping.pointclouds.occupancy`, three algorithms:

  height_cost   terrain SLOPE for a legged robot: cost 100 = a `can_climb`
                (0.15 m) height change over one 5 cm cell. The dataset ships a
                reference PNG of this one; `valider_ply.py` reproduces that PNG
                cell for cell from big_office.ply, which is what proves the
                chain db -> cloud -> their occupancy is theirs and not ours.
  general       height BAND: z < 0.10 m -> free ground, 0.10-2.00 m ->
                obstacle, then `mark_free_radius=0.4` DILATES the free ground.
  simple        the same band, no dilation.

The bench's rover is a 0.46 m WHEELED body carrying a 2D lidar, and the maps it
ate in the flat were 2D lidar costmaps, so the height band is the right
analogue; terrain slope is a legged robot's question and it speckles the
open-plan floor with gradient noise. Between the two band algorithms we take
`simple`, because `general`'s 0.4 m free dilation invents 85 m2 of floor in
cells the Go2 never observed, and the harness's whole ground-truth rule is that
the sim may not invent knowledge the real run never had. `--algo height_cost`
is kept and run as a robustness check; the verdict is reported for both.

Step 3 - the .npz
-----------------
Written in the exact format tools/explore_sim.py::load_world reads
(lidar, low, seen, res, ox, oy, n, pose_xy), so nothing in the bench had to
change: `score = max(lidar, low)`, `>= OCCUPIED_AT (2)` -> occupied, `seen` ->
free, the rest -> unknown.

    cost/occupancy == -1   -> UNKNOWN  (never observed)
    >= OCC_COST            -> OCCUPIED
    otherwise              -> FREE

For `simple`/`general` the grid is already {0, 100, -1}, so OCC_COST is a
formality. For `height_cost` it is a real choice: 50 = half of the Go2's
`can_climb` budget spent over one 5 cm cell (a 7.5 cm kerb), and the sweep
30/50/70/90 is printed so the number is not hidden.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)

DB = os.path.join(SCRATCH, "go2_bigoffice_work.db")   # a copy; the dataset stays read-only
PLY = "/home/openclaw/lerobot/dimos_datasets/big_office.ply"

VOXEL_M = 0.05
RES = 0.05
OCC_COST = 50
OCCUPIED_AT = 2        # costmap2d.OCCUPIED_AT, what load_world thresholds on

OUT_NAME = {"simple": "bigoffice.npz",
            "height_cost": "bigoffice_hc.npz",
            "general": "bigoffice_general.npz"}


def accumulate_from_db(db_path: str = DB) -> np.ndarray:
    """Every `lidar` frame in the store, concatenated and voxel-deduped."""
    from dimos.memory.store.sqlite import SqliteStore

    store = SqliteStore(path=db_path)
    chunks: list[np.ndarray] = []
    n_frames = 0
    t0 = time.time()
    for obs in store.streams.lidar:
        pts, _ = obs.data.as_numpy()
        n_frames += 1
        if len(pts):
            chunks.append(np.asarray(pts, dtype=np.float64))
    raw = np.concatenate(chunks, axis=0)
    del chunks
    print(f"  {n_frames} frames, {len(raw)} raw returns [{time.time() - t0:.0f}s]")
    return voxel_dedup(raw, VOXEL_M)


def voxel_dedup(points: np.ndarray, voxel: float) -> np.ndarray:
    """One point per voxel: the FIRST one seen, with its own coordinates.

    Coordinates are never snapped - this only drops the 55.8 M raw returns to
    something that fits in memory. Deterministic (first in time wins).
    """
    keys = np.floor(points / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    idx.sort()
    return points[idx]


def read_ply(path: str = PLY) -> np.ndarray:
    with open(path, "rb") as fh:
        header = b""
        while b"end_header" not in header:
            line = fh.readline()
            if not line:
                raise ValueError("no end_header")
            header += line
        return np.fromfile(fh, dtype=np.float64).reshape(-1, 3)


def as_cloud(points: np.ndarray):
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    return PointCloud2.from_numpy(points.astype(np.float64), frame_id="world",
                                  timestamp=0.0)


def height_cost(points: np.ndarray, resolution: float = RES):
    """THEIR algorithm, imported and called, on a cloud we only wrap."""
    from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
    return height_cost_occupancy(as_cloud(points), resolution=resolution)


def occupancy(points: np.ndarray, algo: str, resolution: float = RES):
    from dimos.mapping.pointclouds.occupancy import OCCUPANCY_ALGOS
    return OCCUPANCY_ALGOS[algo](as_cloud(points), resolution=resolution)


def to_npz(grid: np.ndarray, res: float, ox: float, oy: float, pose_xy, path: str,
           occ_cost: int = OCC_COST) -> dict:
    """An occupancy/cost grid -> the .npz explore_sim.load_world reads."""
    observed = grid >= 0
    occupied = observed & (grid >= occ_cost)
    lidar = np.zeros(grid.shape, dtype=np.int8)
    lidar[occupied] = OCCUPIED_AT + 1
    h, w = grid.shape
    out = dict(lidar=lidar, low=np.zeros_like(lidar), seen=observed,
               res=np.float64(res), ox=np.float64(ox), oy=np.float64(oy),
               n=np.int64(max(h, w)), pose_xy=np.asarray(pose_xy, dtype=np.float64),
               ts=np.float64(0.0))
    np.savez_compressed(path, **out)
    return out


def first_odom_xy(db_path: str = DB) -> tuple[float, float]:
    import sqlite3
    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    row = conn.execute("select pose_x, pose_y from odom order by ts limit 1").fetchone()
    conn.close()
    return float(row[0]), float(row[1])


def describe(grid: np.ndarray, res: float, label: str) -> None:
    obs = grid >= 0
    h, w = grid.shape
    print(f"{label}: {w} x {h} cells @ {res} m = {w * res:.1f} x {h * res:.1f} m")
    print(f"  observed {obs.sum()} cells = {obs.sum() * res * res:.1f} m2 "
          f"({100 * obs.mean():.1f} % of the grid)")
    for t in (30, 50, 70, 90):
        occ = obs & (grid >= t)
        print(f"  cost >= {t:2d}: occupied {occ.sum() * res * res:7.1f} m2   "
              f"free {(obs & ~occ).sum() * res * res:7.1f} m2")


def get_cloud(cache: str) -> np.ndarray:
    if os.path.exists(cache):
        return np.load(cache)
    print("accumulating the lidar stream from the store ...")
    cloud = accumulate_from_db()
    np.save(cache, cloud)
    return cloud


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", choices=("simple", "height_cost", "general"),
                    default="simple")
    ap.add_argument("--out", default=None)
    ap.add_argument("--cloud-out", default=os.path.join(HERE, "bigoffice_cloud.npy"))
    ap.add_argument("--occ-cost", type=int, default=OCC_COST)
    args = ap.parse_args(argv)

    cloud = get_cloud(args.cloud_out)
    print(f"cloud: {len(cloud)} points  "
          f"x [{cloud[:, 0].min():.2f}, {cloud[:, 0].max():.2f}]  "
          f"y [{cloud[:, 1].min():.2f}, {cloud[:, 1].max():.2f}]  "
          f"z [{cloud[:, 2].min():.2f}, {cloud[:, 2].max():.2f}]")

    og = occupancy(cloud, args.algo)
    grid = np.asarray(og.grid)
    ox = float(og.origin.position.x)
    oy = float(og.origin.position.y)
    describe(grid, og.resolution, f"{args.algo}(db)")

    out = args.out or os.path.join(SCRATCH, OUT_NAME[args.algo])
    pose = first_odom_xy()
    to_npz(grid, og.resolution, ox, oy, pose, out, occ_cost=args.occ_cost)
    print(f"-> {out}   origin ({ox:.3f}, {oy:.3f})  first odom pose {pose}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

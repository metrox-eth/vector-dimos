#!/usr/bin/env python3
"""Prove the relocalizer on real runs of the flat, in centimetres.

Two proofs, both "known input -> known output":

  --same-run   the map is a checkpoint of run A, the scan is a later batch of
               revolutions of the SAME run. Those points are already in run
               A's frame, so the answer is known: the identity. What comes
               back is the relocalizer's error (plus whatever odometry drifted
               between the two instants) in centimetres and degrees.

  --cross-run  the map is a checkpoint of run A, the scan is the first
               revolutions of run B - the boot case, a different arbitrary
               origin. There is no ground truth here, so the proof is
               geometric: after the returned transform, the median distance
               from a scan point to the nearest occupied cell of the saved
               map (the walls overlapping) must be a few centimetres.

The scan is read from a VectorMemory recording (`explore*.db`), `lidar`
stream, filtered to the lidar plane (z = 0.37) exactly as costmap2d does -
that stream also carries the depth camera's obstacle points.

Runs on the Jetson, in the venv (dimOS decodes the LCM payloads); reads only,
never touches the live `explore.db`.

    .venv/bin/python tools/reloc_proof.py --same-run \\
        --map ~/.local/state/vector/checkpoints/<run>/costmap_HHMMSS.npz \\
        --scan ~/.local/state/vector/recordings/explore.<T>.db
"""

from __future__ import annotations

import argparse
import math
import os
import sqlite3
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from vector_dimos.relocalize2d import MapField, relocalize

LIDAR_Z_M = 0.37
Z_TOL = 0.005


def read_revolutions(db: str, n_revs: int, skip: int = 0,
                     at: str | None = None) -> tuple[np.ndarray, list[tuple]]:
    """`n_revs` lidar-plane revolutions from a recording, plus the odom poses.

    Returns the points already in that run's world frame (which is what the
    stream carries) and the odom rows nearest in time, for reference. `at` is
    a local wall clock "HH:MM:SS" inside the run - easier to aim than a row
    offset when a checkpoint has to be matched to the instant it was written.
    """
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

    con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    if at:
        first = con.execute("SELECT MIN(ts) FROM lidar").fetchone()[0]
        day = time.localtime(first)
        h, mi, sec = (int(v) for v in at.split(":"))
        target = time.mktime((day.tm_year, day.tm_mon, day.tm_mday, h, mi, sec,
                              0, 0, day.tm_isdst))
        skip = con.execute("SELECT COUNT(*) FROM lidar WHERE ts < ?", (target,)).fetchone()[0]
    rows = con.execute(
        "SELECT l.id, l.ts, b.data FROM lidar l JOIN lidar_blob b ON b.id = l.id "
        "ORDER BY l.id LIMIT ? OFFSET ?", (n_revs * 4, skip)).fetchall()
    pts, used, t_first, t_last = [], 0, None, None
    for _id, ts, blob in rows:
        if used >= n_revs:
            break
        cloud = PointCloud2.lcm_decode(blob)
        arr = cloud.as_numpy()
        arr = np.asarray(arr[0] if isinstance(arr, tuple) else arr, dtype=np.float64)
        if arr.ndim != 2 or len(arr) == 0:
            continue
        plane = arr[np.abs(arr[:, 2] - LIDAR_Z_M) < Z_TOL]
        if len(plane) < 50:
            continue                      # a depth-camera-only message
        pts.append(plane[:, :2])
        used += 1
        t_first = ts if t_first is None else t_first
        t_last = ts
    odom = con.execute(
        "SELECT ts, pose_x, pose_y, pose_qz, pose_qw FROM odom WHERE ts BETWEEN ? AND ? "
        "ORDER BY ts", (t_first or 0, t_last or 0)).fetchall()
    con.close()
    if not pts:
        raise SystemExit(f"no lidar-plane revolution found in {db}")
    return np.concatenate(pts), odom


def load_map(path: str):
    from vector_dimos.costmap2d import ScoredGrid
    return ScoredGrid.load(path)


def describe(grid) -> str:
    occ = grid.occupancy()
    return (f"{int((occ == 100).sum())} occupied / {int((occ == 0).sum())} free cells, "
            f"{grid.n}x{grid.n} at {grid.res} m, origin ({grid.ox:+.1f}, {grid.oy:+.1f})")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", required=True, help="costmap checkpoint .npz used as the saved map")
    ap.add_argument("--scan", required=True, help="recording .db the scan is read from")
    ap.add_argument("--revs", type=int, default=8, help="revolutions accumulated into the scan")
    ap.add_argument("--skip", type=int, default=0, help="lidar messages skipped first")
    ap.add_argument("--at", help="local time HH:MM:SS inside the run to take the scan at")
    ap.add_argument("--same-run", action="store_true",
                    help="map and scan are the same run: the answer must be the identity")
    ap.add_argument("--cross-run", action="store_true",
                    help="map and scan are different runs: check the wall overlap")
    ap.add_argument("--expect-reject", action="store_true",
                    help="negative control: the verdict must be a rejection")
    args = ap.parse_args()

    grid = load_map(args.map)
    print(f"map  {os.path.basename(args.map)}: {describe(grid)}")
    t0 = time.monotonic()
    field = MapField.from_grid(grid)
    print(f"     distance + likelihood fields built in {time.monotonic() - t0:.2f} s")

    pts, odom = read_revolutions(args.scan, args.revs, args.skip, args.at)
    print(f"scan {os.path.basename(args.scan)}: {len(pts)} points over {args.revs} revolutions"
          + (f", odom at ({odom[0][1]:+.2f}, {odom[0][2]:+.2f})" if odom else ""))

    m = relocalize(field, pts)
    print(f"     {m.as_log()}")

    ok = True
    if args.same_run:
        err_xy = math.hypot(m.x, m.y)
        err_yaw = abs(math.degrees(m.yaw))
        print(f"     SAME RUN: the answer had to be the identity -> off by "
              f"{err_xy * 100:.1f} cm, {err_yaw:.2f} deg")
        ok = m.accepted and err_xy < 0.15 and err_yaw < 3.0
    if args.cross_run:
        print(f"     CROSS RUN: median wall distance {m.median_dist_m * 100:.1f} cm "
              f"(must be under 15 cm), {m.inlier_frac * 100:.0f} % of the scan within 15 cm")
        ok = m.accepted and m.median_dist_m < 0.15
    if args.expect_reject:
        print(f"     NEGATIVE CONTROL: must be rejected -> {'rejected' if not m.accepted else 'ACCEPTED (bad)'}")
        ok = not m.accepted
    print("PROOF PASSED" if ok else "PROOF FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

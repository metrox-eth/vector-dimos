"""Where does a thin obstacle vanish? Print the same top-down view at each stage.

Run next to `dimos run vector-dimos.nav` with the rover parked near a table leg.
Stages: raw lidar revolution (lidar frame) -> world cloud after odometry ->
global voxel map -> costmap. Rover at the centre of each 3 m x 3 m view
(10 cm cells), x forward (up), y left. The stage where the leg disappears is
the culprit. Usage: python stages.py [seconds]
"""

import math
import sys
import time

import numpy as np

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

HALF_M, CELL = 1.5, 0.10
N = int(2 * HALF_M / CELL)
last: dict = {}
raw_hits = np.zeros((N, N), dtype=int)
raw_revs = [0]


def keep(name):
    def cb(msg):
        last[name] = msg
    return cb


def body_of(points, odom):
    """World points -> rover frame using the odometry pose (lidar at origin)."""
    q = odom.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = points[:, 0] - odom.position.x, points[:, 1] - odom.position.y
    return np.stack([c * dx - s * dy, s * dx + c * dy, points[:, 2]], axis=1)


def view(points_xy, title, counts=None):
    grid = np.zeros((N, N), dtype=int) if counts is None else counts
    if counts is None:
        for x, y in points_xy:
            i, j = int((HALF_M - x) / CELL), int((HALF_M - y) / CELL)   # row: forward up; col: left on the left
            if 0 <= i < N and 0 <= j < N:
                grid[i, j] += 1
    print(f"--- {title} (rover at centre, forward = up, left = left; digit = hits per 10 cm cell, # = 10+)")
    for i in range(N):
        row = ""
        for j in range(N):
            if i == N // 2 and j == N // 2:
                row += "R"
            else:
                v = grid[i, j]
                row += "." if v == 0 else (str(v) if v < 10 else "#")
        print(f"{HALF_M - (i + 0.5) * CELL:+.1f} {row}")
    print("     " + "".join("|" if j == N // 2 else " " for j in range(N)))


def on_raw(msg):
    last["pointcloud"] = msg
    pts = msg.as_numpy()[0]
    raw_revs[0] += 1
    for x, y in pts[:, :2]:
        i, j = int((HALF_M - x) / CELL), int((HALF_M - y) / CELL)
        if 0 <= i < N and 0 <= j < N:
            raw_hits[i, j] += 1


make_transport("pointcloud", PointCloud2).subscribe(on_raw)
make_transport("lidar", PointCloud2).subscribe(keep("lidar"))
make_transport("global_map", PointCloud2).subscribe(keep("global_map"))
make_transport("global_costmap", OccupancyGrid).subscribe(keep("global_costmap"))
make_transport("odom", PoseStamped).subscribe(keep("odom"))

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
time.sleep(secs)
print(f"collected {secs:.0f} s: raw revolutions {raw_revs[0]}, streams seen: {sorted(last)}")
if raw_revs[0]:
    # hits per revolution: DIVIDE first. Clamping the sum before the division emptied
    # the whole grid past 99 revolutions (12 s at 10 Hz = 120); view() caps at '#'.
    view(None, f"1. RAW LIDAR, {raw_revs[0]} revolutions summed", counts=raw_hits // raw_revs[0])
odom = last.get("odom")
if odom is None:
    print("no odom -> cannot draw the later stages in the rover frame"); sys.exit(0)
print(f"odom x={odom.position.x:+.2f} y={odom.position.y:+.2f}")
for name, title in (("lidar", "2. WORLD CLOUD after odometry (last revolution)"), ("global_map", "3. GLOBAL VOXEL MAP")):
    if name in last:
        p = last[name].as_numpy()[0]
        b = body_of(p, odom)
        sel = b[(b[:, 2] > -0.30) & (b[:, 2] < 0.90)]   # everything the rover could hit
        view(sel[:, :2], f"{title}: {len(p)} pts, {len(sel)} in hit band")
    else:
        print(f"--- {title}: NOT RECEIVED")
cm = last.get("global_costmap")
if cm is None:
    print("--- 4. COSTMAP: NOT RECEIVED"); sys.exit(0)
g = cm.grid; res = cm.resolution
ox, oy = cm.origin.position.x, cm.origin.position.y
ys, xs = np.nonzero(g >= 50)          # occupied-ish cells
wx, wy = ox + (xs + 0.5) * res, oy + (ys + 0.5) * res
pts = np.stack([wx, wy, np.zeros_like(wx)], axis=1)
b = body_of(pts, odom)
view(b[:, :2], f"4. COSTMAP cells >= 50 ({len(xs)} of {g.size}, res {res} m)")

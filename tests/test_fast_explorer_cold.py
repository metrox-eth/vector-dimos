"""Cold bench for the numpy frontier search.

  1. Known map: a 4 m x 4 m free room (cells 0) inside unknown (-1), robot in
     the middle, a wall (100) closing the north side -> exactly 3 frontier
     clusters (east, south, west), none on the north, centroids on the room
     edges; the unknown region behind the wall is NOT a frontier.
  2. Real checkpoint (if present on this machine): same number of frontiers as
     dimOS's wavefront search on the same grid, in < 0.5 s instead of ~16 s.
"""

import glob
import os
import time

import numpy as np

from vector_dimos.fast_explorer import find_frontiers

RES = 0.05
N = 200                              # 10 m square
grid = np.full((N, N), -1, dtype=np.int8)
grid[60:140, 60:140] = 0             # free room 4 m x 4 m, cells 60..139
grid[59, 55:145] = 100               # wall along the north edge (row 59), 4.5 m long
found = find_frontiers(grid, (100, 100), occupancy_threshold=99, min_cells=int(0.3 / RES))
# east, south and west edges touch at the corners: one U-shaped 8-connected cluster,
# exactly as dimOS's frontier BFS would merge them; the north edge (next to the wall) is excluded
assert len(found) == 1, found
cx, cy, size = found[0]
assert 225 <= size <= 250, size                      # ~3 x 80 cells minus corners
assert cy > 100 and abs(cx - 100) < 2, (cx, cy)      # centroid pulled south (no north side), centred east-west
print(f"  room with a north wall -> 1 U-shaped frontier of {size} cells, centroid ({cx:.0f},{cy:.0f}) south of centre; north edge excluded")

grid2 = grid.copy(); grid2[100, 60:140] = 100         # a wall splits the room east-west: two rooms, robot in the north half
found_split = find_frontiers(grid2, (100, 80), occupancy_threshold=99, min_cells=6)
# dimOS's wavefront walks through UNKNOWN cells too, so the south half is still
# "reachable" around the wall through unexplored space: 3 clusters (east, west, south U)
assert len(found_split) == 3, found_split
assert sorted(s for _, _, s in found_split)[-1] > 100, found_split
print("  room split by a wall -> 3 frontiers: east/west of the north half + the south U reached through unknown space (same as dimOS)")

# robot standing in unknown space next to the room: nearest free cell is used as start
found2 = find_frontiers(grid, (150, 100), occupancy_threshold=99, min_cells=6)
assert len(found2) == 1, len(found2)
print("  robot in unknown space -> starts from the nearest free cell, same U-shaped frontier")

cks = sorted(glob.glob(os.path.expanduser("~/.local/state/vector/checkpoints/*/*.npz")), key=os.path.getmtime)
# newest first, skipping any checkpoint a power cut truncated before save()
# became atomic (26/08: the 13h00 battery death left a corrupt newest .npz)
g = z = None
for ck in reversed(cks):
    try:
        from vector_dimos.costmap2d import ScoredGrid
        g = ScoredGrid.load(ck); z = np.load(ck)
        break
    except Exception as exc:  # noqa: BLE001
        print(f"  (skipping corrupt checkpoint {os.path.basename(ck)}: {exc})")
if g is not None:
    cks = [ck]
    pose = z["pose_xy"]
    occ, ox, oy = g.cropped()
    sx, sy = int((pose[0] - ox) / RES), int((pose[1] - oy) / RES)
    t0 = time.perf_counter(); f = find_frontiers(occ, (sx, sy), 99, 6); dt = time.perf_counter() - t0
    print(f"  real checkpoint {os.path.basename(cks[-1])}: {len(f)} frontiers in {dt * 1000:.0f} ms (dimOS's search: ~16 s)")
    assert dt < 0.5
# failed-goal memory: a goal the planner rejected is skipped for a while, no hot loop
from vector_dimos.fast_explorer import VectorExplorer, FAILED_GOAL_HOLD_S
from dimos_lcm.std_msgs import Bool
ex = VectorExplorer.__new__(VectorExplorer)
ex._failed_goals = []; ex._last_goal = (1.0, 2.0)
import threading; ex.goal_reached_event = threading.Event()
t0 = time.time(); ex._on_goal_reached(Bool(data=False)); dt = time.time() - t0
assert ex.goal_reached_event.is_set() and len(ex._failed_goals) == 1 and dt >= 0.9, (dt, ex._failed_goals)
print(f"  failed goal remembered ({len(ex._failed_goals)}), explorer woken after a {dt:.1f} s breath; hold {FAILED_GOAL_HOLD_S:.0f} s")

# the hold prefers, it never starves: upstream reads [] as "no frontier" and ends
# exploration after 10 of them (20 s), a third of the 60 s hold (audit 28/08, S5).
# Known map, known metres: two 1 m free patches in unknown space, centres at
# world (1.98, 4.98) and (7.98, 4.98); robot at (3.50, 5.00) -> 1.53 m from the
# first, 4.48 m from the second.
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import WavefrontConfig

two = np.full((240, 240), -1, dtype=np.int8)
two[90:110, 30:50] = 0               # patch A, ring centroid (1.975, 4.975) m
two[90:110, 150:170] = 0             # patch B, ring centroid (7.975, 4.975) m
cm = OccupancyGrid(grid=two, resolution=RES)
POSE = Vector3(3.5, 5.0, 0.0)
A, B = (1.975, 4.975), (7.975, 4.975)
assert len(find_frontiers(two, (70, 100), 99, 6)) == 2, "map must hold exactly 2 clusters"

ex = VectorExplorer.__new__(VectorExplorer)
ex.config = WavefrontConfig(min_frontier_perimeter=0.3, safe_distance=0.35,   # nav_blueprints values
                            lookahead_distance=4.0, max_explored_distance=12.0)
ex.explored_goals = []; ex.exploration_direction = Vector3(0.0, 0.0, 0.0)


def detect(holds):
    ex._failed_goals = [(x, y, time.monotonic()) for x, y in holds]
    ex._last_goal = None
    return ex.detect_frontiers(POSE, cm)


free = detect([])
assert len(free) == 2, free
starved = detect([A, B])                       # both clusters under a fresh hold
assert len(starved) == 1, starved              # pre-fix: [] -> 10 failures, exploration over in 20 s
assert abs(starved[0].x - A[0]) < 0.1 and abs(starved[0].y - A[1]) < 0.1, starved[0]
assert ex._last_goal is not None and abs(ex._last_goal[0] - A[0]) < 0.1, ex._last_goal
print(f"  both clusters held -> yields the closest, ({starved[0].x:.2f}, {starved[0].y:.2f}) m at "
      f"{((starved[0].x - POSE.x) ** 2 + (starved[0].y - POSE.y) ** 2) ** 0.5:.2f} m, not []")
one = detect([A])                              # one held, one free -> only the free one
assert len(one) == 1 and abs(one[0].x - B[0]) < 0.1, one
print(f"  one held, one free -> only the free one, ({one[0].x:.2f}, {one[0].y:.2f}) m")
empty = ex.detect_frontiers(POSE, OccupancyGrid(grid=np.zeros((60, 60), dtype=np.int8), resolution=RES))
assert empty == [], empty                      # [] stays for "genuinely no frontier"
print("  fully known map -> [] (the give-up path is still reachable when it is true)")
print("TEST PASSED")

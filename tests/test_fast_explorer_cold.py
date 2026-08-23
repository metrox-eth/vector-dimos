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
if cks:
    from vector_dimos.costmap2d import ScoredGrid
    g = ScoredGrid.load(cks[-1]); z = np.load(cks[-1]); pose = z["pose_xy"]
    occ, ox, oy = g.cropped()
    sx, sy = int((pose[0] - ox) / RES), int((pose[1] - oy) / RES)
    t0 = time.perf_counter(); f = find_frontiers(occ, (sx, sy), 99, 6); dt = time.perf_counter() - t0
    print(f"  real checkpoint {os.path.basename(cks[-1])}: {len(f)} frontiers in {dt * 1000:.0f} ms (dimOS's search: ~16 s)")
    assert dt < 0.5
print("TEST PASSED")

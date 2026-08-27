"""Cold bench: a table is a block (2026-08-23 - the safest path, not the
shortest: the rover goes around tables, never under them).

Base-frame depth points in metres -> which become obstacles, which floor:
  * a table top at 0.75 m over the cell 1.0 m ahead -> obstacle (was ignored above 0.70)
  * the floor seen between the legs on that SAME cell -> dropped (obstacle wins)
  * bare floor on a cell with nothing above it -> floor
  * a lamp head at 1.2 m -> obstacle; a ceiling point at 2.4 m -> neither
"""

import numpy as np

from vector_dimos.lidar_odometry import OBSTACLE_Z_M, split_floor_and_obstacles

bx = np.array([1.00, 1.01, 1.50, 2.00, 2.00])
by = np.array([0.00, 0.01, 0.00, 0.50, 0.50])
bz = np.array([0.75, 0.00, 0.00, 1.20, 2.40])
floor_z = 0.03 + 0.03 * np.clip(bx - 1.0, 0.0, None)
obst, floor = split_floor_and_obstacles(bx, by, bz, floor_z)
assert OBSTACLE_Z_M[1] >= 1.3
assert obst.tolist() == [True, False, False, True, False], obst.tolist()
assert floor.tolist() == [False, False, True, False, False], floor.tolist()
print("  table top 0.75 m -> obstacle; floor under it -> dropped; bare floor -> floor; lamp 1.2 m -> obstacle; ceiling 2.4 m -> ignored")
print("TEST PASSED")

"""WavefrontFrontierExplorer with a numpy frontier search and no dead time.

Measured 23/08 on the real map (35 774 cells seen): dimOS's pure-Python
wavefront BFS took 16.5 s offline and 20-42 s live on the Jetson, growing
with the map - the rover stood still between every two goals ("il reflechit",
measured live). The same search with scipy.ndimage takes milliseconds. Semantics
reproduced exactly: start at the nearest FREE cell to the robot; reachable =
8-connected region of FREE + UNKNOWN cells from there; a frontier point is an
UNKNOWN cell with a FREE 8-neighbour and no occupied (> occupancy_threshold)
8-neighbour; clusters are 8-connected, kept if >= min_frontier_perimeter /
resolution cells; centroids ranked by their own `_rank_frontiers`.

Second fix: their explorer only wakes on goal_reached == True. On "No path
found" the planner publishes False and the explorer slept its full
goal_timeout (15 s) for nothing. Any goal_reached message wakes it now -
but a failed goal is remembered and frontiers near it are skipped for
FAILED_GOAL_HOLD_S, with a short breath before the next pick: without
that the loop re-published the same unreachable goal 3x per second and
pinned a core (23/08 21:25).
"""

from __future__ import annotations

import time

import numpy as np
from scipy import ndimage

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import WavefrontFrontierExplorer
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

logger = setup_logger()

_EIGHT = np.ones((3, 3), dtype=bool)
FAILED_GOAL_HOLD_S = 60.0    # a goal the planner could not reach is avoided this long
FAILED_GOAL_RADIUS_M = 0.6
FAIL_BREATH_S = 1.0          # pause after a failed goal before picking the next


def find_frontiers(grid: np.ndarray, start_xy: tuple[int, int], occupancy_threshold: int,
                   min_cells: int) -> list[tuple[float, float, int]]:
    """(centroid_x, centroid_y, size) in GRID coordinates for every reachable
    frontier cluster, same definition as dimOS's wavefront search."""
    free = grid == CostValues.FREE
    unknown = grid == CostValues.UNKNOWN
    occupied = grid > occupancy_threshold
    if not free.any():
        return []
    sx, sy = start_xy
    h, w = grid.shape
    sx, sy = int(np.clip(sx, 0, w - 1)), int(np.clip(sy, 0, h - 1))
    if not free[sy, sx]:
        # nearest free cell (their _find_free_space BFS)
        _, idx = ndimage.distance_transform_edt(~free, return_indices=True)
        sy, sx = int(idx[0][sy, sx]), int(idx[1][sy, sx])
    explorable = free | unknown
    labels, _ = ndimage.label(explorable, structure=_EIGHT)
    reachable = labels == labels[sy, sx]
    near_free = ndimage.binary_dilation(free, structure=_EIGHT)
    near_occ = ndimage.binary_dilation(occupied, structure=_EIGHT)
    frontier = unknown & reachable & near_free & ~near_occ
    if not frontier.any():
        return []
    flab, n = ndimage.label(frontier, structure=_EIGHT)
    sizes = ndimage.sum(frontier, flab, index=np.arange(1, n + 1))
    out = []
    for i, size in enumerate(sizes, start=1):
        if size < min_cells:
            continue
        cy, cx = ndimage.center_of_mass(frontier, flab, i)
        out.append((float(cx), float(cy), int(size)))
    return out


class VectorExplorer(WavefrontFrontierExplorer):
    """Drop-in for WavefrontFrontierExplorer: same goals, no 20-40 s pauses."""

    _failed_goals: list[tuple[float, float, float]] = []   # (x, y, monotonic time)
    _last_goal: tuple[float, float] | None = None

    def detect_frontiers(self, robot_pose: Vector3, costmap: OccupancyGrid) -> list[Vector3]:
        t0 = time.perf_counter()
        grid_pos = costmap.world_to_grid(robot_pose)
        min_cells = int(self.config.min_frontier_perimeter / costmap.resolution)
        found = find_frontiers(costmap.grid, (int(grid_pos.x), int(grid_pos.y)),
                               self.config.occupancy_threshold, min_cells)
        if not found:
            return []
        now = time.monotonic()
        self._failed_goals = [f for f in self._failed_goals if now - f[2] < FAILED_GOAL_HOLD_S]
        centroids, sizes = [], []
        for cx, cy, size in found:
            w = costmap.grid_to_world(Vector3(cx, cy, 0.0))
            if any((w.x - fx) ** 2 + (w.y - fy) ** 2 < FAILED_GOAL_RADIUS_M ** 2 for fx, fy, _ in self._failed_goals):
                continue
            centroids.append(w); sizes.append(size)
        if not centroids:
            logger.info(f"frontiers: {len(found)} clusters, all near recently failed goals")
            return []
        logger.info(f"frontiers: {len(found)} clusters in {(time.perf_counter() - t0) * 1000:.0f} ms")
        return self._rank_frontiers(centroids, sizes, robot_pose, costmap)

    def _rank_frontiers(self, centroids, sizes, robot_pose, costmap):  # type: ignore[override]
        ranked = super()._rank_frontiers(centroids, sizes, robot_pose, costmap)
        if ranked:
            self._last_goal = (float(ranked[0].x), float(ranked[0].y))
        return ranked

    def _on_goal_reached(self, msg: Bool) -> None:
        # True = arrived; False = the planner gave up (no path). Either way pick the
        # next frontier now - but remember a failed goal and breathe before retrying.
        if not getattr(msg, "data", False) and self._last_goal is not None:
            self._failed_goals.append((self._last_goal[0], self._last_goal[1], time.monotonic()))
            time.sleep(FAIL_BREATH_S)
        self.goal_reached_event.set()

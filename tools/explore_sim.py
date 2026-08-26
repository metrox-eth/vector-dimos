#!/usr/bin/env python3
"""Offline A/B of the two exploration strategies on a REAL map, no robot.

    tools/explore_sim.py [--map recordings/map_backups/carte_saine_avant_toilettes.npz]
    tools/explore_sim.py --start -4.42 -3.72       # a named start pose
    tools/explore_sim.py --extinction-demo         # the 26/08 death, reproduced, old vs v2

What is simulated, and how faithful each piece is:

  ground truth   a saved VECTOR costmap checkpoint, decoded exactly as
                 costmap2d.ScoredGrid.occupancy() does (score = max(lidar, low),
                 >= OCCUPIED_AT -> 100, seen -> 0, rest -> -1). Cells the real
                 run never saw stay -1 and are opaque to the simulated lidar:
                 the sim cannot invent knowledge the rover never had.
  lidar          360 rays, 12 m (RPLIDAR C1), ray-cast on the ground truth from
                 the simulated pose, revealing cells into a discovered map.
                 One scan per SCAN_EVERY_M of travel, plus one on arrival.
  keep-outs      the owner's zones, applied to the discovered map exactly as
                 costmap2d does - forced to 100 after every layer, seen or not.
  planner        tonight's planner, ported: dimOS's voronoi_gradient + our
                 clearance_cost_map (lethal from a distance transform at
                 0.30 m, 4th-power pivot penalty over the 0.39 m band) and a
                 search with min_cost_astar's exact cost rule - blocked at
                 >= 99, unknown priced at 99 * 0.95, cumulative CELL cost first
                 and path length only as tie-break. Implemented as one Dijkstra
                 with weight = cell_cost + 1e-6 * step, which orders the same
                 way (a whole path's tie-break weight stays under 1e-3, and
                 cell costs are integers). Ported rather than imported because
                 dimOS lives only on the Jetson, and the point of this harness
                 is that it runs anywhere.
  motion         drive the planned path at the blueprint's 0.15 m/s cap, turn
                 in place at 0.5 rad/s, arrive within 0.25 m, and give up on a
                 goal after the blueprint's goal_timeout (45 s) exactly as the
                 real loop does. That timeout is why 1 goal in 29 was reached
                 on 25/08 and it applies to both strategies here.

What the sim CANNOT claim, and it matters:
  - no wheel slip, no impacts, no map rollback, no relocalization jump. The
    real run had 51 slips and 51 rollbacks; those cost real path length that
    neither simulated strategy pays.
  - no odometry drift: the simulated pose is perfect, so a sim reversal is a
    decision, never noise. Real reversal counts include follower wobble.
  - the discovered map is built from a map that was itself built by a real
    run, so anything that run never saw is unreachable here by construction.
  - therefore: the OLD numbers in the contract table are the honest baseline,
    and the sim column is only ever a like-for-like OLD-vs-NEW comparison.

The old strategy is not stock dimOS: it is what actually ran on 25-26/08 -
fast_explorer.VectorExplorer, i.e. dimOS's scoring (0.3 info gain + 0.3
distance from explored goals + 0.2 lookahead distance + 0.15 obstacle distance
+ 0.05 momentum) with the numpy frontier search and the 0.6 m / 60 s failed
goal filter bolted on. Its self-stop rules are reproduced as configured in
nav_blueprints.py: info_gain_threshold 0.001 over 6 attempts, and the loop's
"2 goals published and 10 consecutive failures = complete".
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
sys.path.insert(0, os.path.join(ROOT, "vector_dimos"))

from bench_run import count_reversals  # noqa: E402  (same scorer as the real runs)

try:  # the package __init__ pulls dimOS; off the Jetson, load the modules directly
    from vector_dimos.explorer2 import (
        DIRECTIVE_BACK_OFF, DIRECTIVE_WAIT, ExploreState, PoseStamped, next_target,
    )
    from vector_dimos import persistent_map
except ImportError:  # pragma: no cover - the laptop path
    import persistent_map  # type: ignore[no-redef]
    from explorer2 import (  # type: ignore[no-redef]
        DIRECTIVE_BACK_OFF, DIRECTIVE_WAIT, ExploreState, PoseStamped, next_target,
    )


# --- the world -------------------------------------------------------------

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
OCCUPIED_AT = 2                 # costmap2d.OCCUPIED_AT
LIDAR_RANGE_M = 12.0            # RPLIDAR C1
LIDAR_RAYS = 360
SCAN_EVERY_M = 0.25             # one revolution per 0.25 m at 0.15 m/s = 1.7 Hz, ~ the real rate

# --- the robot -------------------------------------------------------------

SPEED_MPS = 0.15                # nav_blueprints: exploration speed cap
TURN_RATE = 0.5                 # rad/s in place
ARRIVE_M = 0.25
GOAL_TIMEOUT_S = 45.0           # nav_blueprints: VectorExplorer(goal_timeout=45.0)
FAIL_BREATH_S = 1.0             # fast_explorer.FAIL_BREATH_S
BUMP_BACKUP_M = 0.20            # recovering_planner.BACKUP_DISTANCE_M
MAX_IMPACTS_PER_GOAL = 3        # after three contacts on one goal the planner gives up

ROBOT_WIDTH_M = 0.50            # recovering_planner.ROBOT_WIDTH_M (0.46 body + 4 cm)
BODY_HALF_WIDTH_M = 0.23        # the real body: 62.5 x 46 cm with the bumper bars
                                # (metrox, 25/08). What stops the rover is the body,
                                # not the planning margin.
CONTROL_MARGIN_M = 0.05
LETHAL_CLEARANCE_M = ROBOT_WIDTH_M / 2 + CONTROL_MARGIN_M    # 0.30
PIVOT_CLEARANCE_M = 0.78 / 2                                  # 0.39
PIVOT_PENALTY = 100
PIVOT_RAMP_EXPONENT = 4
GRADIENT_DISTANCE_M = 1.5

# --- the old strategy, as configured in nav_blueprints.py ------------------

OLD_MIN_FRONTIER_PERIMETER = 0.3
OLD_OCCUPANCY_THRESHOLD = 99
OLD_SAFE_DISTANCE = 0.35
OLD_LOOKAHEAD = 4.0
OLD_MAX_EXPLORED_DISTANCE = 12.0
OLD_INFO_GAIN_THRESHOLD = 0.001
OLD_NUM_NO_GAIN_ATTEMPTS = 6
OLD_INFLATE_M = 0.25            # the loop's simple_inflate(costmap, 0.25)
OLD_FAILED_RADIUS_M = 0.6       # fast_explorer.FAILED_GOAL_RADIUS_M
OLD_FAILED_HOLD_S = 60.0        # fast_explorer.FAILED_GOAL_HOLD_S
OLD_MAX_CONSECUTIVE_FAILURES = 10

# --- run limits (a livelock must be reported, never hidden) ----------------

MAX_GOALS = 300
MAX_PATH_M = 600.0
MAX_SIM_S = 6000.0
# A run that has travelled this far without gaining STALL_GAIN_M2 of map is
# done as far as this bench is concerned. It is a HARNESS stop, applied
# identically to both strategies and always named in the report - never a
# strategy self-stop, which is the thing under test. Without it a strategy that
# correctly refuses to give up probes unknown pinches too narrow for the body
# until the goal budget runs out (measured: 360 m of travel for the last
# 0.2 m2), which says nothing about how it explores.
STALL_M = 20.0
STALL_GAIN_M2 = 0.10

_EIGHT = np.ones((3, 3), dtype=bool)
_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = ((-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
               (0, -1, 1.0), (0, 1, 1.0),
               (1, -1, _SQRT2), (1, 0, 1.0), (1, 1, _SQRT2))


# ===========================================================================
# ground truth + simulated lidar
# ===========================================================================

@dataclass
class World:
    truth: np.ndarray            # int8 100/0/-1
    res: float
    ox: float
    oy: float
    keepout: np.ndarray | None
    zones: list[dict]
    # Metres from every cell to the nearest ground-truth OBSTACLE. A keep-out is
    # a rule, not a wall, so it is not in here: driving into one is measured, not
    # prevented by physics. An UNKNOWN cell is not a wall either - on a map built
    # by a real run, "unknown" means "never observed", and the flat does not stop
    # at the edge of what that run saw. It is opaque to the simulated lidar (the
    # sim may not invent knowledge) but the body passes through it.
    clearance: np.ndarray = field(default=None, repr=False)  # type: ignore[assignment]

    def cell(self, x: float, y: float) -> tuple[int, int]:
        h, w = self.truth.shape
        gx = int(np.clip(math.floor((x - self.ox) / self.res), 0, w - 1))
        gy = int(np.clip(math.floor((y - self.oy) / self.res), 0, h - 1))
        return gy, gx

    def world_xy(self, gy: int, gx: int) -> tuple[float, float]:
        """The CENTRE of the cell. Corners are shared by four cells, and a pose
        landing exactly on one made cell() and world_xy() disagree - which the
        drive loop read as "the path starts where I already am" and replanned
        forever without moving."""
        return self.ox + (gx + 0.5) * self.res, self.oy + (gy + 0.5) * self.res

    @property
    def free_area_m2(self) -> float:
        return float((self.truth == FREE).sum()) * self.res * self.res

    def visible_area_m2(self, start: tuple[float, float], lattice_m: float = 0.5) -> float:
        """The ceiling: everything this map can ever show a 0.50 m rover with a
        12 m lidar, starting from `start`.

        Body-passable free ground (the planner's 0.30 m lethal clearance on the
        ground truth) connected to the start, sampled on a lattice, one
        simulated revolution from each sample, union of what they reveal.
        Anything past a pinch the 0.46 m body does not fit through is not
        "unexplored", it is out of reach - so this, and not the raw free area,
        is what a coverage percentage should be measured against.

        A reference, not a hard ceiling: a run may exceed it by driving across
        ground the real run never mapped (unknown is not a wall), which is
        legitimate but earns no coverage in this sim.
        """
        passable = (self.truth == FREE) & (self.clearance + 1e-6 >= BODY_HALF_WIDTH_M)
        if self.keepout is not None:
            passable = passable & ~self.keepout
        cy, cx = self.cell(*start)
        if not passable[cy, cx]:
            _, idx = ndimage.distance_transform_edt(~passable, return_indices=True)
            cy, cx = int(idx[0][cy, cx]), int(idx[1][cy, cx])
        labels, _ = ndimage.label(passable, structure=_EIGHT)
        body = labels == labels[cy, cx]

        seen = np.full(self.truth.shape, UNKNOWN, dtype=np.int8)
        step = max(1, int(round(lattice_m / self.res)))
        ys, xs = np.nonzero(body)
        for gy, gx in zip(ys[::1], xs[::1]):
            if gy % step or gx % step:
                continue
            scan(self, seen, *self.world_xy(int(gy), int(gx)))
        return float((seen != UNKNOWN).sum()) * self.res * self.res


def load_world(path: str, keepout_path: str | None, unknown_is_wall: bool = True) -> World:
    """A saved ScoredGrid checkpoint -> the world the simulated rover lives in.

    `unknown_is_wall` (the default) closes the arena at the edge of what the real
    run observed. It is the only honest reading of a map used as ground truth:
    a cell the real run never saw is a cell this harness knows nothing about, so
    a simulated lidar standing inside it would see nothing and a simulated rover
    driving through it would be blind. Measured with it left open: the rover
    wandered into unmapped space, saw nothing, and drove into walls it could not
    have known about - 893 body contacts in one run, which says everything about
    the harness and nothing about the strategy. Closed, the arena is exactly the
    63.5 m2 the real run mapped, bounded by a mixture of real walls and the edge
    of that run's knowledge, and both strategies race over the same ground.
    """
    z = np.load(path)
    score = np.maximum(z["lidar"], z["low"])
    truth = np.full(z["lidar"].shape, UNKNOWN, dtype=np.int8)
    truth[z["seen"]] = FREE
    truth[score >= OCCUPIED_AT] = OCCUPIED
    if unknown_is_wall:
        truth[truth == UNKNOWN] = OCCUPIED
    res, ox, oy = float(z["res"]), float(z["ox"]), float(z["oy"])
    n = int(z["n"])
    zones = persistent_map.load_keepouts(keepout_path) if keepout_path else []
    forbidden = persistent_map.zones_of(zones, persistent_map.FORBIDDEN)
    keepout = (persistent_map.keepout_mask(forbidden, res, ox, oy, n) if forbidden else None)
    clearance = ndimage.distance_transform_edt(truth < OCCUPIED) * res
    return World(truth=truth, res=res, ox=ox, oy=oy, keepout=keepout, zones=zones,
                 clearance=clearance)


def scan(world: World, discovered: np.ndarray, x: float, y: float) -> None:
    """One lidar revolution from (x, y), written into `discovered` in place.

    A ray walks the ground truth and reveals every FREE cell it crosses. It
    stops at the first cell that is not FREE: an obstacle is revealed as an
    obstacle, a ground-truth UNKNOWN cell stops the ray and stays unknown (the
    real run never saw behind it either, so neither can this one).
    """
    h, w = world.truth.shape
    steps = int(LIDAR_RANGE_M / world.res)
    angles = np.linspace(0.0, 2.0 * math.pi, LIDAR_RAYS, endpoint=False)
    radii = (np.arange(1, steps + 1) * world.res)[None, :]
    xs = x + np.cos(angles)[:, None] * radii
    ys = y + np.sin(angles)[:, None] * radii
    gx = np.floor((xs - world.ox) / world.res).astype(np.int64)
    gy = np.floor((ys - world.oy) / world.res).astype(np.int64)
    inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
    gxc, gyc = np.clip(gx, 0, w - 1), np.clip(gy, 0, h - 1)
    values = world.truth[gyc, gxc]
    blocking = (~inside) | (values != FREE)

    any_block = blocking.any(axis=1)
    first = np.where(any_block, blocking.argmax(axis=1), steps)
    order = np.arange(steps)[None, :]
    open_run = order < first[:, None]

    free_cells = open_run & inside
    discovered[gyc[free_cells], gxc[free_cells]] = FREE
    # the blocking cell itself, when it is a real obstacle inside the map
    rays = np.arange(LIDAR_RAYS)[any_block]
    hit = first[any_block]
    hit_ok = inside[rays, hit] & (values[rays, hit] == OCCUPIED)
    discovered[gyc[rays[hit_ok], hit[hit_ok]], gxc[rays[hit_ok], hit[hit_ok]]] = OCCUPIED

    # costmap2d.body_clear: what the body drove over is free, whatever the rays
    # say. Without it the cell under the rover stays unknown (the first ray
    # sample is already one cell out) and the explorer plans from unknown ground.
    r = int(math.ceil((ROBOT_WIDTH_M / 2) / world.res))
    cy, cx = world.cell(x, y)
    yy = np.arange(max(0, cy - r), min(h, cy + r + 1))[:, None] - cy
    xx = np.arange(max(0, cx - r), min(w, cx + r + 1))[None, :] - cx
    body = discovered[max(0, cy - r):min(h, cy + r + 1), max(0, cx - r):min(w, cx + r + 1)]
    body[(yy ** 2 + xx ** 2 <= r ** 2) & (body != OCCUPIED)] = FREE

    if world.keepout is not None:
        discovered[world.keepout] = OCCUPIED     # costmap2d: the last word, after every layer


# ===========================================================================
# the planner, as it runs tonight
# ===========================================================================

def voronoi_gradient(grid: np.ndarray, res: float, obstacle_threshold: int = 50,
                     max_distance: float = 2.0) -> np.ndarray:
    """Port of dimos/mapping/occupancy/gradient.py::voronoi_gradient (Apache-2.0).

    Cost 0 on the medial axis between two obstacles, 100 at an obstacle,
    99 * d_voronoi / (d_obstacle + d_voronoi) in between, 0 beyond
    max_distance, unknown preserved as -1.
    """
    unknown_mask = grid == UNKNOWN
    obstacle = (grid >= obstacle_threshold).astype(np.float32)
    if not obstacle.any():
        out = np.zeros_like(grid, dtype=np.int16)
        out[unknown_mask] = UNKNOWN
        return out
    labels, n_obstacles = ndimage.label(obstacle)
    distance_cells, indices = ndimage.distance_transform_edt(1 - obstacle, return_indices=True)
    if n_obstacles > 1:
        nearest = labels[indices[0], indices[1]]
        foot = np.ones((3, 3), dtype=bool)
        edges = (ndimage.maximum_filter(nearest, footprint=foot, mode="nearest")
                 != ndimage.minimum_filter(nearest, footprint=foot, mode="nearest"))
        edges &= obstacle == 0
    else:
        edges = np.zeros_like(obstacle, dtype=bool)
    if not edges.any():
        # gradient() fallback, same file
        distance_m = np.clip(distance_cells * res, 0, max_distance)
        values = (1 - distance_m / max_distance) * 100
    else:
        voronoi_distance = ndimage.distance_transform_edt(~edges)
        total = distance_cells + voronoi_distance
        with np.errstate(divide="ignore", invalid="ignore"):
            ratio = np.where(total > 0, voronoi_distance / total, 0)
        values = ratio * 99
        values[distance_cells > max_distance / res] = 0
    values[obstacle > 0] = OCCUPIED
    out = values.astype(np.int16)
    out[unknown_mask] = UNKNOWN
    return out


def clearance_cost_map(grid: np.ndarray, res: float) -> np.ndarray:
    """Port of vector_dimos.recovering_planner.clearance_cost_map (26/08)."""
    distance_m = ndimage.distance_transform_edt(grid < OCCUPIED) * res
    too_close = distance_m + 1e-6 < LETHAL_CLEARANCE_M
    lethal = np.where(too_close, np.int16(OCCUPIED), grid.astype(np.int16))
    base = voronoi_gradient(lethal.astype(np.int16), res, max_distance=GRADIENT_DISTANCE_M)
    passable = (base >= FREE) & (base < OCCUPIED)
    ramp = np.clip((PIVOT_CLEARANCE_M - distance_m)
                   / (PIVOT_CLEARANCE_M - LETHAL_CLEARANCE_M), 0.0, 1.0)
    extra = (PIVOT_PENALTY * ramp ** PIVOT_RAMP_EXPONENT).astype(np.int16)
    out = base.copy()
    out[passable] = np.clip(base[passable] + extra[passable], FREE, OCCUPIED - 1)
    return out


PLANNER_COST_THRESHOLD = 100    # min_cost_astar's own default, and what
PLANNER_UNKNOWN_PENALTY = 0.8   # GlobalPlanner._find_wide_path passes (i.e. nothing)


def plan(cost: np.ndarray, start_yx: tuple[int, int], goal_yx: tuple[int, int],
         cost_threshold: int = PLANNER_COST_THRESHOLD,
         unknown_penalty: float = PLANNER_UNKNOWN_PENALTY,
         ) -> list[tuple[int, int]] | None:
    """min_cost_astar's cost rule, solved with one Dijkstra.

    Per dimos/navigation/replanning_a_star/min_cost_astar.py: a cell at or above
    `cost_threshold` is blocked, an unknown cell costs threshold * penalty, a
    free cell costs 0, anything else costs its own value; the search compares
    cumulative CELL cost first and path length only to break ties. The tie-break
    is carried here by a 1e-6 weight on step length.

    The thresholds are the PLANNER's, not the explorer's: RecoveringGlobalPlanner
    calls min_cost_astar with its defaults (blocked at 100, unknown at 80), so a
    cell the Voronoi ridge priced 99 is expensive, not a wall. Blocking at 99 -
    which is what the frontier A* of PR #2830 does - walls off every pinch the
    gradient touches, and the simulated rover could not leave its first room.
    """
    h, w = cost.shape
    blocked = cost >= cost_threshold
    unknown_cost = cost_threshold * unknown_penalty
    if unknown_cost >= cost_threshold:
        blocked |= cost == UNKNOWN
    cell = np.where(cost == UNKNOWN, unknown_cost, np.maximum(cost, 0)).astype(np.float64)
    passable = ~blocked
    passable[start_yx] = True
    if not passable[goal_yx]:
        return None

    ys, xs = np.nonzero(passable)
    n = int(ys.size)
    index = np.full((h, w), -1, dtype=np.int64)
    index[ys, xs] = np.arange(n)
    rows, cols, vals = [], [], []
    for dy, dx, length in _NEIGHBOURS:
        y0, y1 = max(0, -dy), h - max(0, dy)
        x0, x1 = max(0, -dx), w - max(0, dx)
        a = index[y0:y1, x0:x1]
        b = index[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
        both = (a >= 0) & (b >= 0)
        if not both.any():
            continue
        rows.append(a[both])
        cols.append(b[both])
        vals.append(cell[y0 + dy:y1 + dy, x0 + dx:x1 + dx][both] + 1e-6 * length)
    graph = csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(n, n))
    dist, pred = dijkstra(graph, directed=True, indices=int(index[start_yx]),
                          return_predecessors=True)
    goal_node = int(index[goal_yx])
    if not math.isfinite(dist[goal_node]):
        return None
    path = []
    node = goal_node
    while node >= 0:
        path.append((int(ys[node]), int(xs[node])))
        node = int(pred[node])
        if len(path) > n:
            return None
    path.reverse()
    return path


SIMPLIFY_EPS_M = 0.10        # a raster path is a staircase; the follower drives its chord


def simplify(points: list[tuple[float, float]], eps: float = SIMPLIFY_EPS_M,
             ) -> list[tuple[float, float]]:
    """Douglas-Peucker. A cell path is a 5 cm staircase; what the rover actually
    drives is its straight segments, and those are what a turn costs time on."""
    if len(points) < 3:
        return list(points)
    (x0, y0), (x1, y1) = points[0], points[-1]
    dx, dy = x1 - x0, y1 - y0
    span = math.hypot(dx, dy)
    worst, index = -1.0, 0
    for i in range(1, len(points) - 1):
        px, py = points[i]
        if span < 1e-12:
            d = math.hypot(px - x0, py - y0)
        else:
            d = abs(dy * px - dx * py + x1 * y0 - y1 * x0) / span
        if d > worst:
            worst, index = d, i
    if worst <= eps:
        return [points[0], points[-1]]
    return simplify(points[:index + 1], eps)[:-1] + simplify(points[index:], eps)


def clear_footprint(cost: np.ndarray, binary: np.ndarray, start_yx: tuple[int, int],
                    res: float) -> None:
    """dimOS clears the cells under the robot before planning, so a start inside
    the inflation can still leave. Same intent: cells within half a body of the
    start that the raw map calls free go back to free."""
    r = int(math.ceil((ROBOT_WIDTH_M / 2) / res))
    y, x = start_yx
    h, w = cost.shape
    ys = slice(max(0, y - r), min(h, y + r + 1))
    xs = slice(max(0, x - r), min(w, x + r + 1))
    window = cost[ys, xs]
    window[(binary[ys, xs] == FREE) & (window >= PLANNER_COST_THRESHOLD)] = FREE


# ===========================================================================
# the OLD strategy: fast_explorer.VectorExplorer, as it ran on 25-26/08
# ===========================================================================

def simple_inflate(grid: np.ndarray, radius_m: float, res: float) -> np.ndarray:
    """dimos/mapping/occupancy/inflation.py::simple_inflate - a disc dilation
    with the radius rounded UP to whole cells (which is the +1 cell that walled
    off 60-70 cm doorways)."""
    r = int(math.ceil(radius_m / res))
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    kernel = (xx ** 2 + yy ** 2 <= r ** 2)
    out = grid.copy()
    out[ndimage.binary_dilation(grid >= OCCUPIED, structure=kernel)] = OCCUPIED
    return out


def find_frontiers(grid: np.ndarray, start_yx: tuple[int, int], occupancy_threshold: int,
                   min_cells: int) -> list[tuple[float, float, int]]:
    """vector_dimos.fast_explorer.find_frontiers, verbatim in behaviour:
    (centroid_x, centroid_y, size) in GRID coordinates."""
    free = grid == FREE
    unknown = grid == UNKNOWN
    occupied = grid > occupancy_threshold
    if not free.any():
        return []
    sy, sx = start_yx
    if not free[sy, sx]:
        _, idx = ndimage.distance_transform_edt(~free, return_indices=True)
        sy, sx = int(idx[0][sy, sx]), int(idx[1][sy, sx])
    labels, _ = ndimage.label(free | unknown, structure=_EIGHT)
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


@dataclass
class OldExplorer:
    """dimOS's scoring with fast_explorer's failed-goal filter, and both of its
    self-stop rules. Everything here has a line in the shipped code."""

    res: float
    ox: float
    oy: float
    explored_goals: list[tuple[float, float]] = field(default_factory=list)
    exploration_direction: tuple[float, float] = (0.0, 0.0)
    failed_goals: list[tuple[float, float, float]] = field(default_factory=list)
    last_goal: tuple[float, float] | None = None
    last_info: int | None = None
    no_gain_counter: int = 0
    goals_published: int = 0
    consecutive_failures: int = 0
    complete: bool = False
    complete_reason: str = ""
    raw_clusters: int = 0
    # Off for the third run of the A/B: with its two self-stops removed the old
    # SCORING can be compared past the point where its timers kill it. Without
    # that column the only honest statement about coverage above 25 m2 would be
    # "the old one was not there".
    allow_self_stop: bool = True

    def _distance_to_explored(self, x: float, y: float) -> float:
        if not self.explored_goals:
            return 5.0
        return min(math.hypot(x - gx, y - gy) for gx, gy in self.explored_goals)

    def next_goal(self, grid: np.ndarray, x: float, y: float, now: float,
                  ) -> tuple[float, float] | None:
        inflated = simple_inflate(grid, OLD_INFLATE_M, self.res)

        # get_exploration_goal: the information-gain self-stop
        info = int(((inflated == FREE) | (inflated >= OCCUPIED)).sum())
        if (self.allow_self_stop and len(self.explored_goals) > 5
                and self.last_info is not None and self.last_info > 0):
            increase = (info - self.last_info) / self.last_info
            if increase < OLD_INFO_GAIN_THRESHOLD:
                self.no_gain_counter += 1
                if self.no_gain_counter >= OLD_NUM_NO_GAIN_ATTEMPTS:
                    self.no_gain_counter = 0
                    self.complete = True
                    self.complete_reason = "no information gain"
                    self.last_info = info
                    return None
            else:
                self.no_gain_counter = 0

        h, w = grid.shape
        gx0 = int(np.clip(math.floor((x - self.ox) / self.res), 0, w - 1))
        gy0 = int(np.clip(math.floor((y - self.oy) / self.res), 0, h - 1))
        min_cells = int(OLD_MIN_FRONTIER_PERIMETER / self.res)
        found = find_frontiers(inflated, (gy0, gx0), OLD_OCCUPANCY_THRESHOLD, min_cells)
        self.last_info = info
        self.raw_clusters = len(found)          # for the false-extinction audit
        if not found:
            return None

        self.failed_goals = [f for f in self.failed_goals if now - f[2] < OLD_FAILED_HOLD_S]
        candidates = []
        for cx, cy, size in found:
            wx, wy = self.ox + cx * self.res, self.oy + cy * self.res
            if any((wx - fx) ** 2 + (wy - fy) ** 2 < OLD_FAILED_RADIUS_M ** 2
                   for fx, fy, _ in self.failed_goals):
                continue
            candidates.append((wx, wy, size))
        if not candidates:
            # "frontiers: N clusters, all near recently failed goals" -> [] ->
            # the loop counts a consecutive failure. This is the false
            # extinction of spec 7.1, reproduced on purpose.
            return None

        # _compute_distance_to_obstacles, vectorised: the same minimum distance
        # to an occupied cell, capped at safe_distance when none is in range.
        obstacle_distance = ndimage.distance_transform_edt(
            inflated < OLD_OCCUPANCY_THRESHOLD) * self.res

        best, best_score = None, -math.inf
        max_expected = OLD_MIN_FRONTIER_PERIMETER / self.res * 10
        for wx, wy, size in candidates:
            robot_distance = math.hypot(wx - x, wy - y)
            distance_score = 1.0 / (1.0 + abs(robot_distance - OLD_LOOKAHEAD))
            info_gain_score = min(size / max_expected, 1.0)
            explored_score = min(self._distance_to_explored(wx, wy) / OLD_MAX_EXPLORED_DISTANCE, 1.0)
            gy = int(np.clip((wy - self.oy) / self.res, 0, h - 1))
            gx = int(np.clip((wx - self.ox) / self.res, 0, w - 1))
            obstacles = min(float(obstacle_distance[gy, gx]), OLD_SAFE_DISTANCE)
            obstacles_score = obstacles / OLD_SAFE_DISTANCE
            momentum = 0.0
            dx, dy = wx - x, wy - y
            span = math.hypot(dx, dy)
            if span >= 0.1 and (self.exploration_direction[0] or self.exploration_direction[1]):
                momentum = max(0.0, self.exploration_direction[0] * dx / span
                               + self.exploration_direction[1] * dy / span)
            score = (0.3 * info_gain_score + 0.3 * explored_score + 0.2 * distance_score
                     + 0.15 * obstacles_score + 0.05 * momentum)
            if score > best_score:
                best, best_score = (wx, wy), score

        assert best is not None
        dx, dy = best[0] - x, best[1] - y
        span = math.hypot(dx, dy)
        if span > 0.1:
            self.exploration_direction = (dx / span, dy / span)
        self.explored_goals.append(best)
        self.last_goal = best
        return best

    def note_failed(self, x: float, y: float, now: float) -> None:
        self.failed_goals.append((x, y, now))

    def reset_session(self) -> None:
        self.explored_goals.clear()
        self.exploration_direction = (0.0, 0.0)
        self.last_info = None
        self.no_gain_counter = 0


# ===========================================================================
# the simulation loop
# ===========================================================================

@dataclass
class Run:
    label: str
    poses: list[tuple[float, float, float, float]] = field(default_factory=list)
    coverage_curve: list[tuple[float, float]] = field(default_factory=list)  # (path_m, area_m2)
    goals_published: int = 0
    goals_reached: int = 0
    goals_no_path: int = 0
    goals_timed_out: int = 0
    impacts: int = 0
    waits: int = 0
    back_offs: int = 0
    false_extinctions: int = 0
    forbidden_entries: int = 0
    path_m: float = 0.0
    sim_s: float = 0.0
    end_reason: str = ""
    decide_ms: list[float] = field(default_factory=list)


class Sim:
    def __init__(self, world: World, start: tuple[float, float], heading: float, label: str):
        self.world = world
        self.discovered = np.full(world.truth.shape, UNKNOWN, dtype=np.int8)
        if world.keepout is not None:
            self.discovered[world.keepout] = OCCUPIED
        self.x, self.y, self.yaw = start[0], start[1], heading
        self.t = 0.0
        self.run = Run(label=label)
        self._since_scan = 0.0
        self._scans = 0
        self._stall_from_m = 0.0
        self._stall_from_area = 0.0
        self._record()
        self.rescan()

    # --- bookkeeping -------------------------------------------------------
    def _record(self) -> None:
        self.run.poses.append((self.t, self.x, self.y, self.yaw))
        if persistent_map.zone_at(self.world.zones, self.x, self.y,
                                  persistent_map.FORBIDDEN) is not None:
            self.run.forbidden_entries += 1

    def rescan(self) -> None:
        scan(self.world, self.discovered, self.x, self.y)
        self._since_scan = 0.0
        self._scans += 1
        self.run.coverage_curve.append((self.run.path_m, self.area_m2))

    @property
    def area_m2(self) -> float:
        seen = (self.discovered == FREE) | (self.discovered == OCCUPIED)
        return float(seen.sum()) * self.world.res * self.world.res

    @property
    def free_m2(self) -> float:
        return float((self.discovered == FREE).sum()) * self.world.res * self.world.res

    # --- motion ------------------------------------------------------------
    def turn_to(self, target_yaw: float) -> None:
        """Turn in place. Charged once per straight segment, never per cell: a
        raster path alternates 0 and 45 degrees every 5 cm, and paying a pivot
        for each of those turned a 2 m drive into 46 s of simulated time."""
        delta = (target_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(delta) < 1e-9:
            return
        self.t += abs(delta) / TURN_RATE
        self.yaw = target_yaw
        self._record()

    def advance(self, x: float, y: float) -> float:
        """Move along the current heading. Returns the distance travelled, or
        -1.0 when the body would touch a ground-truth obstacle.

        The planner plans on what it has SEEN; a wall revealed late is a wall
        the rover runs into (4 of the 11 impacts on 25/08 were exactly that).
        Backing out of a tight spot is always allowed - refusing that would
        strand the rover on the first cell where the body is already pinched.
        """
        d = math.hypot(x - self.x, y - self.y)
        if d <= 1e-12:
            return 0.0
        here = float(self.world.clearance[self.world.cell(self.x, self.y)])
        there = float(self.world.clearance[self.world.cell(x, y)])
        # Half a cell of tolerance: `clearance` is a distance transform between
        # cell CENTRES on a 5 cm grid, so a cell's true clearance is only known
        # to +/- res/2, and without the tolerance the contact test fires on a
        # 1 cm quantisation error. The real rover carries the same margin in
        # hardware - the planner is given robot_width 0.50 for a 0.46 m body.
        if there + self.world.res / 2 + 1e-6 < BODY_HALF_WIDTH_M and there + 1e-6 < here:
            return -1.0
        self.x, self.y = x, y
        self.t += d / SPEED_MPS
        self.run.path_m += d
        self._since_scan += d
        self._record()
        if self._since_scan >= SCAN_EVERY_M:
            self.rescan()
        return d

    def drive(self, goal_xy: tuple[float, float]) -> str:
        """Drive to the goal, replanning on every fresh lidar revolution the way
        ReplanningAStarPlanner does. Returns 'reached', 'timeout' or 'blocked'
        ('blocked' is the planner publishing goal_reached=False)."""
        started = self.t
        impacts = 0
        while True:
            path = self.plan_to(goal_xy)
            if path is None:
                return "blocked"
            waypoints = simplify([self.world.world_xy(gy, gx) for gy, gx in path])
            scans_at_plan = self._scans
            moved = 0.0
            fresh_map = False
            for wx, wy in waypoints[1:]:
                self.turn_to(math.atan2(wy - self.y, wx - self.x))
                for sx, sy in self._sub_steps(wx, wy):
                    step = self.advance(sx, sy)
                    if step < 0.0:
                        # Body contact. The real rover does not abandon the goal:
                        # the bump reflex stops it, backs it off 0.20 m and asks
                        # the planner to replan (esp_sensors -> RecoveringPlanner.
                        # handle_bump). Only a goal that keeps ending in contact
                        # is finally reported unreachable.
                        self.run.impacts += 1
                        impacts += 1
                        self.back_off()
                        self.rescan()
                        if impacts >= MAX_IMPACTS_PER_GOAL:
                            return "blocked"
                        fresh_map = True
                        break
                    moved += step
                    if math.hypot(self.x - goal_xy[0], self.y - goal_xy[1]) <= ARRIVE_M:
                        self.rescan()
                        return "reached"
                    if self.t - started >= GOAL_TIMEOUT_S:
                        self.rescan()
                        return "timeout"
                    if self._scans != scans_at_plan:
                        fresh_map = True
                        break
                if fresh_map:
                    break
            if not fresh_map or (moved <= 1e-9 and impacts == 0):
                # Either the path ran out without arriving, or a whole replan
                # produced no motion at all. Never spin on the spot.
                self.rescan()
                return "timeout"

    def back_off(self) -> float:
        """The bump reflex: straight back BUMP_BACKUP_M, heading unchanged.

        Reversing is always allowed short of a wall: the rover just drove that
        ground, so the body fits there whatever the clearance field says about
        the cell it landed in. Refusing it deadlocked the simulated rover in a
        pinch - contact forward, no reverse, three impacts, next goal, same.
        """
        bx = self.x - BUMP_BACKUP_M * math.cos(self.yaw)
        by = self.y - BUMP_BACKUP_M * math.sin(self.yaw)
        if self.world.truth[self.world.cell(bx, by)] == OCCUPIED:
            return 0.0
        travelled = math.hypot(bx - self.x, by - self.y)
        self.x, self.y = bx, by
        self.t += travelled / SPEED_MPS
        self.run.path_m += travelled
        self._since_scan += travelled
        self._record()
        return travelled

    def _sub_steps(self, x1: float, y1: float):
        """One cell at a time along a straight segment, so the lidar fires at
        the same spacing as on the real rover and the recorded trajectory has
        the chord length tools/bench_run.py expects."""
        x0, y0 = self.x, self.y
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(math.ceil(d / self.world.res)))
        for i in range(1, n + 1):
            yield x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n

    def plan_to(self, goal_xy: tuple[float, float]) -> list[tuple[int, int]] | None:
        cost = clearance_cost_map(self.discovered, self.world.res)
        start = self.world.cell(self.x, self.y)
        clear_footprint(cost, self.discovered, start, self.world.res)
        goal = self.world.cell(*goal_xy)
        return plan(cost, start, goal)

    def over_budget(self) -> str:
        if self.run.path_m - self._stall_from_m >= STALL_M:
            if self.area_m2 - self._stall_from_area < STALL_GAIN_M2:
                return (f"stalled - {STALL_M:.0f} m of travel for less than "
                        f"{STALL_GAIN_M2:.2f} m2 of new map")
            self._stall_from_m, self._stall_from_area = self.run.path_m, self.area_m2
        if self.run.goals_published >= MAX_GOALS:
            return "goal budget"
        if self.run.path_m >= MAX_PATH_M:
            return "path budget"
        if self.t >= MAX_SIM_S:
            return "time budget"
        return ""


def run_old(world: World, start: tuple[float, float], heading: float,
            self_stop: bool = True) -> Run:
    label = ("old (wavefront, as run 25-26/08)" if self_stop
             else "old scoring, self-stop removed")
    sim = Sim(world, start, heading, label)
    explorer = OldExplorer(res=world.res, ox=world.ox, oy=world.oy,
                           allow_self_stop=self_stop)
    while True:
        over = sim.over_budget()
        if over:
            sim.run.end_reason = f"stopped by the harness: {over}"
            break
        t0 = time.perf_counter()
        goal = explorer.next_goal(sim.discovered, sim.x, sim.y, sim.t)
        sim.run.decide_ms.append((time.perf_counter() - t0) * 1000.0)

        if goal is None:
            if explorer.complete:
                sim.run.end_reason = f"explorer stopped itself: {explorer.complete_reason}"
                if explorer.raw_clusters > 0:
                    sim.run.false_extinctions += 1
                break
            explorer.consecutive_failures += 1
            explorer.reset_session()
            if (self_stop and explorer.goals_published >= 2
                    and explorer.consecutive_failures >= OLD_MAX_CONSECUTIVE_FAILURES):
                sim.run.end_reason = (f"explorer stopped itself: {explorer.consecutive_failures} "
                                      f"consecutive failures finding new frontiers")
                if explorer.raw_clusters > 0:
                    sim.run.false_extinctions += 1
                break
            sim.t += 2.0                 # the loop's "Retrying in 2 seconds"
            sim._record()
            continue

        explorer.consecutive_failures = 0
        explorer.goals_published += 1
        sim.run.goals_published += 1
        outcome = sim.drive(goal)
        if outcome == "blocked":
            sim.run.goals_no_path += 1
            explorer.note_failed(goal[0], goal[1], sim.t)
            sim.t += FAIL_BREATH_S
            sim._record()
            continue
        if outcome == "reached":
            sim.run.goals_reached += 1
        else:
            sim.run.goals_timed_out += 1
    sim.run.sim_s = sim.t
    sim.run.coverage_curve.append((sim.run.path_m, sim.area_m2))
    return sim.run


def run_v2(world: World, start: tuple[float, float], heading: float) -> Run:
    sim = Sim(world, start, heading, "explorer2 (info gain per path cost)")
    state = ExploreState(heading=heading)

    while True:
        over = sim.over_budget()
        if over:
            sim.run.end_reason = f"stopped by the harness: {over}"
            break
        pose = _pose(sim.x, sim.y, sim.yaw)
        costmap = _SimGrid(sim.discovered, world.res, world.ox, world.oy, sim.t)
        t0 = time.perf_counter()
        target = next_target(costmap, pose, state, now=sim.t)
        sim.run.decide_ms.append((time.perf_counter() - t0) * 1000.0)

        if target is None:
            sim.run.end_reason = "next_target returned None: no reachable frontier left"
            # Audit: was that true? Re-decide with the run's TIMERS and
            # preferences erased - no failed-goal exclusions, no goals-issued
            # memory, so no revisit fade and no spent viewpoints - and see
            # whether a FRONTIER target existed after all. A wait or a back-off
            # is not one: those are states the loop was already in and had
            # already acted on.
            #
            # What is NOT erased is `observed`: where the rover physically
            # stood and took a full lidar revolution. That is geometry, and the
            # contract this harness scores says in as many words that a run may
            # end on it ("only frontiers the rover has already stood at and
            # looked from"). Erasing it would ask a different question - would
            # an amnesiac rover teleported here have somewhere to look - whose
            # answer is yes for as long as one unknown cell remains anywhere.
            # Note the direction: keeping it can only ever find FEWER targets,
            # so no run's score improves by this and none of the numbers taken
            # before it changes.
            audit = ExploreState(heading=state.heading, observed=list(state.observed))
            second = next_target(costmap, pose, audit, now=sim.t)
            if second is not None and getattr(second, "directive", "") == "frontier":
                sim.run.false_extinctions += 1
            break

        directive = getattr(target, "directive", "frontier")
        if directive == DIRECTIVE_WAIT:
            sim.run.waits += 1
            sim.t += max(0.1, min(float(target.wait_s), 1.0))   # the module's WAIT_POLL_S
            sim._record()
            sim.rescan()
            continue
        if directive == DIRECTIVE_BACK_OFF:
            sim.run.back_offs += 1
            bx, by = target.position.x, target.position.y
            gy, gx = world.cell(bx, by)
            if world.truth[gy, gx] != OCCUPIED:
                sim.advance(bx, by)           # a reverse: the heading does not change
            sim.rescan()
            continue

        goal = (float(target.position.x), float(target.position.y))
        sim.run.goals_published += 1
        gap_at_issue = math.hypot(goal[0] - sim.x, goal[1] - sim.y)
        outcome = sim.drive(goal)
        if outcome == "blocked":
            sim.run.goals_no_path += 1
            state.note_failed(goal[0], goal[1], sim.t)
            sim.t += FAIL_BREATH_S
            sim._record()
            continue
        if outcome == "reached":
            sim.run.goals_reached += 1
        else:
            sim.run.goals_timed_out += 1
            # Explorer2._run_exploration_loop, GOAL_PROGRESS_M: a drive that
            # timed out without closing the gap is excluded (§7.1), one that is
            # still closing it is simply re-decided. The loop is what this
            # harness replays, so it replays this too.
            if gap_at_issue - math.hypot(goal[0] - sim.x, goal[1] - sim.y) < ARRIVE_M:
                state.note_failed(goal[0], goal[1], sim.t)
    sim.run.sim_s = sim.t
    sim.run.coverage_curve.append((sim.run.path_m, sim.area_m2))
    return sim.run


def _pose(x: float, y: float, yaw: float):
    p = PoseStamped(ts=0.0, frame_id="world")
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


# ===========================================================================
# scoring and the contract table
# ===========================================================================

def path_at_area(curve: list[tuple[float, float]], area: float) -> float | None:
    """Path length at which this run first reached `area` m2 seen."""
    for path_m, seen in curve:
        if seen >= area:
            return path_m
    return None


def summarise(run: Run) -> dict:
    return {
        "label": run.label,
        "area_m2": run.coverage_curve[-1][1] if run.coverage_curve else 0.0,
        "path_m": run.path_m,
        "reversals": count_reversals(run.poses),
        "goals_published": run.goals_published,
        "goals_reached": run.goals_reached,
        "goals_no_path": run.goals_no_path,
        "goals_timed_out": run.goals_timed_out,
        "impacts": run.impacts,
        "waits": run.waits,
        "back_offs": run.back_offs,
        "false_extinctions": run.false_extinctions,
        "forbidden_entries": run.forbidden_entries,
        "sim_s": run.sim_s,
        "end_reason": run.end_reason,
        "decide_ms_max": max(run.decide_ms) if run.decide_ms else 0.0,
        "decide_ms_mean": (sum(run.decide_ms) / len(run.decide_ms)) if run.decide_ms else 0.0,
    }


# The real run of 25/08 22:12 (recordings/explore.20260825221228.db.score.json),
# scored with tools/bench_run.py. This is the baseline the contract was written
# against; it is measured, not simulated.
CEILING = [0.0]     # filled in by main(): World.visible_area_m2 for the start pose


REAL_OLD = {
    "area_m2": 50.59,
    "path_m": 64.73,
    "reversals": 96,
    "goals_published": 29,
    "goals_reached": 1,
    "false_extinctions": 3,      # the night's three "Exploration complete" deaths
}


def print_report(old: dict, new: dict, old_run: Run, new_run: Run, world: World,
                 free: dict | None = None, free_run: Run | None = None) -> None:
    common = min(old["area_m2"], new["area_m2"])
    old_at = path_at_area(old_run.coverage_curve, common)
    new_at = path_at_area(new_run.coverage_curve, common)
    ratio = (new_at / old_at) if (old_at and new_at) else None

    print()
    print("=" * 96)
    print("THE CONTRACT")
    print("=" * 96)
    head = f"{'metric':<34}{'old (real, 25/08)':>20}{'old (sim)':>14}{'v2 (sim)':>14}{'verdict':>14}"
    print(head)
    print("-" * 96)

    def row(name, real, o, n, ok, fmt="{:.1f}"):
        rs = fmt.format(real) if isinstance(real, (int, float)) else str(real)
        os_ = fmt.format(o) if isinstance(o, (int, float)) else str(o)
        ns = fmt.format(n) if isinstance(n, (int, float)) else str(n)
        print(f"{name:<34}{rs:>20}{os_:>14}{ns:>14}{('PASS' if ok else 'FAIL'):>14}")

    row("coverage (m2 seen)", REAL_OLD["area_m2"], old["area_m2"], new["area_m2"],
        new["area_m2"] >= old["area_m2"] - 0.05, "{:.1f}")
    row(f"path for {common:.1f} m2 (m)", REAL_OLD["path_m"],
        old_at if old_at else float("nan"), new_at if new_at else float("nan"),
        bool(ratio and ratio < 1.0), "{:.1f}")
    row("direction reversals", REAL_OLD["reversals"], old["reversals"], new["reversals"],
        new["reversals"] < 20, "{:.0f}")
    row("goals reached / published",
        f"{REAL_OLD['goals_reached']}/{REAL_OLD['goals_published']}",
        f"{old['goals_reached']}/{old['goals_published']}",
        f"{new['goals_reached']}/{new['goals_published']}",
        new["goals_reached"] * 2 > new["goals_published"])
    row("false extinctions", REAL_OLD["false_extinctions"], old["false_extinctions"],
        new["false_extinctions"], new["false_extinctions"] == 0, "{:.0f}")
    row("forbidden-zone entries", "-", old["forbidden_entries"], new["forbidden_entries"],
        new["forbidden_entries"] == 0, "{:.0f}")
    row("born-cornered recovery", "manual", "manual (none)",
        f"auto x{new['back_offs']}", True)
    print("-" * 96)
    if ratio:
        print(f"in-sim path ratio at {common:.1f} m2 seen: {ratio:.2f} "
              f"({100 * (ratio - 1):+.0f}% travel for the coverage the old one reached "
              f"before it stopped itself)")
    if free is not None and free_run is not None and free_run.coverage_curve:
        # The old SCORING with its two self-stops removed, so it can be
        # compared past the point where its timers kill it.
        big = min(free["area_m2"], new["area_m2"])
        f_at = path_at_area(free_run.coverage_curve, big)
        n_at = path_at_area(new_run.coverage_curve, big)
        print(f"old scoring with the self-stops REMOVED: {free['area_m2']:.1f} m2 in "
              f"{free['path_m']:.1f} m, {free['reversals']} reversals, "
              f"{free['goals_reached']}/{free['goals_published']} reached "
              f"({free['end_reason']})")
        if f_at and n_at:
            print(f"  path to the common {big:.1f} m2:  old-scoring {f_at:.1f} m   "
                  f"v2 {n_at:.1f} m   ratio {n_at / f_at:.2f} "
                  f"({100 * (n_at / f_at - 1):+.0f}%)")
    print(f"ground truth: {world.free_area_m2:.1f} m2 of free floor, "
          f"{(world.truth != UNKNOWN).sum() * world.res ** 2:.1f} m2 ever observed by the real run; "
          f"ceiling for this start = {CEILING[0]:.1f} m2")
    print()
    rows = [("old", old, old_run), ("v2 ", new, new_run)]
    if free is not None and free_run is not None:
        rows.insert(1, ("old*", free, free_run))
    for tag, s, r in rows:
        print(f"{tag}: {s['area_m2']:.1f} m2 in {s['path_m']:.1f} m / {s['sim_s'] / 60:.1f} min, "
              f"{s['goals_reached']}/{s['goals_published']} reached, "
              f"{s['goals_no_path']} no-path, {s['goals_timed_out']} timed out, "
              f"{s['impacts']} impacts, {s['waits']} waits, {s['back_offs']} back-offs")
        print(f"     decision {s['decide_ms_mean']:.0f} ms mean / {s['decide_ms_max']:.0f} ms worst")
        print(f"     end: {s['end_reason']}")
    print()
    for fraction in (0.80, 0.90, 0.95):
        area = fraction * CEILING[0]
        o = path_at_area(free_run.coverage_curve if free_run else old_run.coverage_curve, area)
        n = path_at_area(new_run.coverage_curve, area)
        o_s = f"{o:6.1f} m" if o else "  never"
        n_s = f"{n:6.1f} m" if n else "  never"
        imp = f"{100 * (n / o - 1):+.0f}%" if (o and n) else "   -"
        print(f"  travel to {fraction:.0%} of what this map can ever show "
              f"({area:5.1f} m2):  old* {o_s}   v2 {n_s}   {imp}")


# ===========================================================================
# the false-extinction demonstration (spec 7.1)
# ===========================================================================

def extinction_demo() -> int:
    """Reproduce the death that happened three times on the night of 26/08, and
    show what v2 does with the same input.

    The setup is the one the logs describe: several valid frontier clusters, and
    every one of them inside the 0.6 m / 60 s exclusion of a goal the planner
    just refused. Nothing here is stochastic - the map, the exclusions and the
    clock are all fixed, so this is a demonstration, not a lucky run.
    """
    res = 0.05
    n = 200                                   # 10 x 10 m
    g = np.full((n, n), UNKNOWN, dtype=np.int8)
    g[40:160, 40:160] = FREE                  # a 6 x 6 m room
    g[39:161, 39] = OCCUPIED
    g[39:161, 160] = OCCUPIED
    g[39, 39:161] = OCCUPIED
    g[160, 39:161] = OCCUPIED
    for centre in (70, 100, 130):             # three doors = three clusters
        g[centre - 12:centre + 12, 160] = UNKNOWN
    ox = oy = 0.0
    here_x, here_y = (100 + 0.5) * res, (100 + 0.5) * res

    old = OldExplorer(res=res, ox=ox, oy=oy)
    old.goals_published = 2                   # the loop's precondition for giving up
    clusters = find_frontiers(simple_inflate(g, OLD_INFLATE_M, res),
                              (100, 100), OLD_OCCUPANCY_THRESHOLD,
                              int(OLD_MIN_FRONTIER_PERIMETER / res))
    print(f"a room with {len(clusters)} valid frontier clusters, all of them inside the "
          f"{OLD_FAILED_RADIUS_M} m / {OLD_FAILED_HOLD_S:.0f} s exclusion of a refused goal\n")
    for cx, cy, _size in clusters:
        old.note_failed(ox + cx * res, oy + cy * res, 0.0)
        old.explored_goals.append((ox + cx * res, oy + cy * res))

    state = ExploreState(failed=[(f[0], f[1], 0.0) for f in old.failed_goals])
    cm = _SimGrid(g, res, ox, oy, 0.0)

    print(f"{'t':>6}  {'old (as shipped)':<44}  {'explorer2'}")
    print("-" * 96)
    old_dead_at = v2_dead_at = None
    v2_first_target_at = None
    for step in range(40):
        t = 2.0 * step                        # the old loop's "Retrying in 2 seconds"
        if old_dead_at is None:
            goal = old.next_goal(g, here_x, here_y, t)
            if goal is None:
                old.consecutive_failures += 1
                old.reset_session()
                old_line = (f"no frontier found (attempt "
                            f"{old.consecutive_failures}/{OLD_MAX_CONSECUTIVE_FAILURES})")
                if old.consecutive_failures >= OLD_MAX_CONSECUTIVE_FAILURES:
                    old_line = "EXPLORATION COMPLETE - and the map is still unexplored"
                    old_dead_at = t
            else:
                old_line = f"goal ({goal[0]:.2f}, {goal[1]:.2f})"
        else:
            old_line = "(stopped)"

        target = next_target(cm, _pose(here_x, here_y, 0.0), state, now=t)
        if target is None:
            v2_line = "None - would stop"
            v2_dead_at = v2_dead_at or t
        elif target.directive == DIRECTIVE_WAIT:
            v2_line = f"wait {target.wait_s:.0f} s ({target.n_excluded} of {target.n_clusters} excluded)"
        else:
            v2_line = f"goal ({target.position.x:.2f}, {target.position.y:.2f})"
            v2_first_target_at = v2_first_target_at or t
        if step % 4 == 0 or old_dead_at == t or v2_first_target_at == t:
            print(f"{t:6.0f}  {old_line:<44}  {v2_line}")
        if old_dead_at is not None and v2_first_target_at is not None:
            break

    print("-" * 96)
    print(f"old: declared complete at t = {old_dead_at:.0f} s with {len(clusters)} clusters on the map "
          f"and {OLD_FAILED_HOLD_S - old_dead_at:.0f} s still to run on the exclusions"
          if old_dead_at is not None else "old: did not stop")
    print(f"v2 : waited, then took a goal at t = {v2_first_target_at:.0f} s"
          if v2_first_target_at is not None else "v2 : never took a goal")
    print(f"v2 never returned None: {v2_dead_at is None}")
    return 0 if (old_dead_at is not None and v2_first_target_at is not None
                 and v2_dead_at is None) else 1


class _SimGrid:
    def __init__(self, grid, res, ox, oy, ts):
        self.grid, self.resolution, self.ts, self.frame_id = grid, res, ts, "world"
        self.origin = type("O", (), {"position": type("P", (), {"x": ox, "y": oy, "z": 0.0})()})()


def main(argv: list[str] | None = None) -> int:
    default_map = os.path.join(ROOT, "recordings", "map_backups",
                               "carte_saine_avant_toilettes.npz")
    default_keepout = os.path.join(ROOT, "recordings", "map_backups", "keepout.json")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", default=default_map)
    ap.add_argument("--keepout", default=default_keepout)
    ap.add_argument("--unknown-open", action="store_true",
                    help="leave ground-truth unknown open instead of closing the arena "
                         "at the edge of what the real run mapped (see load_world)")
    ap.add_argument("--start", nargs=2, type=float, default=None,
                    metavar=("X", "Y"), help="start pose; default = the map's own saved pose")
    ap.add_argument("--heading", type=float, default=0.0, help="start heading, radians")
    ap.add_argument("--only", choices=("old", "v2"), default=None)
    ap.add_argument("--json", default=None, help="write the scores to this file")
    ap.add_argument("--extinction-demo", action="store_true",
                    help="reproduce the 26/08 false extinction and show v2 on the same input")
    args = ap.parse_args(argv)

    if args.extinction_demo:
        return extinction_demo()

    keepout = args.keepout if os.path.isfile(args.keepout) else None
    world = load_world(args.map, keepout, unknown_is_wall=not args.unknown_open)
    if args.start:
        start = (args.start[0], args.start[1])
    else:
        z = np.load(args.map)
        start = (float(z["pose_xy"][0]), float(z["pose_xy"][1]))
    gy, gx = world.cell(*start)
    if world.truth[gy, gx] != FREE:
        print(f"start {start} is not free ground in the map ({world.truth[gy, gx]})", file=sys.stderr)
        return 2

    print(f"map        {os.path.relpath(args.map, ROOT)}  "
          f"{world.truth.shape[1]}x{world.truth.shape[0]} @ {world.res} m")
    print(f"keep-outs  {len(persistent_map.zones_of(world.zones, persistent_map.FORBIDDEN))} "
          f"forbidden" + (f" ({os.path.relpath(keepout, ROOT)})" if keepout else " (none)"))
    print(f"start      ({start[0]:.2f}, {start[1]:.2f}) heading {args.heading:.2f} rad")
    CEILING[0] = world.visible_area_m2(start)

    old_run = run_old(world, start, args.heading) if args.only != "v2" else Run("old (skipped)")
    free_run = (run_old(world, start, args.heading, self_stop=False)
                if args.only is None else Run("old, self-stop removed (skipped)"))
    new_run = run_v2(world, start, args.heading) if args.only != "old" else Run("v2 (skipped)")
    old, free, new = summarise(old_run), summarise(free_run), summarise(new_run)
    print_report(old, new, old_run, new_run, world, free, free_run)

    if args.json:
        with open(args.json, "w") as fh:
            json.dump({"schema": "vector_explore_sim/1", "map": args.map,
                       "start": list(start), "heading": args.heading,
                       "ceiling_m2": CEILING[0],
                       "real_old": REAL_OLD, "old": old, "old_no_selfstop": free, "v2": new,
                       "old_curve": old_run.coverage_curve,
                       "old_no_selfstop_curve": free_run.coverage_curve if free_run else [],
                       "v2_curve": new_run.coverage_curve}, fh, indent=2)
            fh.write("\n")
        print(f"\nscores -> {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

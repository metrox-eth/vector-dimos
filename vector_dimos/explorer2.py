"""Frontier exploration as a pure, replayable function.

Shape asked for by lesh in dimensionalOS/dimos#1255 and written down in
`showrobotics/docs/dimos_explorer_spec.md` §3:

    next_target(costmap, pose, state) -> PoseStamped | None

`state` is the only memory (a plain dataclass), the function touches no LCM,
no clock it was not given and no robot, so a recording replays through it
unchanged. `None` is the ONLY way exploration ends.

Strategy = spec §4B, information gain per path cost, taken from
dimensionalOS/dimos PR #2830 (samuelokpor). What is reused verbatim from that
PR, and what is not:

  reused   the objective shape, multiplicative instead of the old weighted sum:
             score = info_gain / (1 + path_cost)
             score *= 1 + 0.5 * heading_alignment      (its only numeric weight)
             score *= revisit_fade                     (linear, inside a radius)
  reused   info gain = frontier cluster cell count, normalised by
           min_frontier_perimeter / resolution * 10
  reused   an unreachable cluster scores -inf and is dropped, never published
  reused   unknown space is priced near-lethal so a route may cut a corner
           through it but never prefers it (its unknown_traversal_penalty=0.95)
  dropped  the old 0.3 info + 0.3 distance-from-explored-goals + 0.2
           lookahead-distance + 0.15 obstacle-distance + 0.05 momentum sum.
           The 0.3 on "far from explored goals" is the map-jumping lesh
           opened #1255 about; the lookahead term peaked at 5 m and taxed
           every nearby frontier.
  dropped  its info_gain_threshold / num_no_gain_attempts self-stop, and the
           loop's "10 consecutive failures" give-up. Both are timers, and
           timers are how this rover died three times on 26/08 with ten valid
           clusters on the map.
  changed  path cost is ONE Dijkstra from the robot over the whole traversable
           grid instead of one A* per cluster: same currency, N clusters for
           the price of one search.
  changed  the target is a free, reachable cell standing off the cluster, not
           the cluster centroid in unknown space. VECTOR's lidar reaches 12 m;
           it does not need to enter the unknown to see it, and a goal it can
           actually stand on is a goal it can actually reach (1 of 29 reached
           on the real run of 25/08).
  changed  information gain is the unknown AREA a revolution from the viewpoint
           would actually reveal - unknown cells in LINE OF SIGHT of it, within
           Tuning.info_radius_m - not the length of the frontier. Dividing a
           boundary length by path cost makes the rover take the nearest door
           every time; measured on the real flat, that myopia cost +71 % travel.
  added    a cluster reachable only across unmapped ground is a PROBE and is
           ranked behind every safe frontier (Tuning.probe_penalty).
  added    a viewpoint the rover has already decided from is never handed out
           again (Tuning.observed_radius_m): driving to a spot it is already at
           reveals nothing. That is geometry, not a counter, and it is what
           keeps a frontier no viewpoint can resolve from being retargeted for
           ever (measured: 300 goals and 184 m of travel for the last 0.2 m2).


What run B of 26/08 changed (recordings/courseB_explorer2.db, the first real
run of this module, 10 goals in 6 min 33; tools/replay_decision.py replays any
of its decisions through this function):

  the owner watched it turn round and go back into the bedroom it had already
  mapped instead of pushing on. Replayed, goal 10 is unambiguous: the frontier
  at the bedroom wall scored 0.0160 against 0.0074 for the best unmapped-room
  cluster, and the two terms that did it were the ones that have nothing to do
  with information - a x4 probe penalty every cluster beyond the mapped floor
  pays and a mapped-floor errand does not, and 1/(1 + path cost) at 3.4 m
  against 25 m. The gain term barely separated them (0.514 vs 0.672), because a
  box filter over the unknown counts the unknown on the far side of the wall
  the rover would be standing against. Hence the two changes above and below:
  gain is what a revolution there would SEE, and the anti-revisit fade is
  quadratic (an area, not a length) over every goal published this run except
  the one just attempted. Same recording, same state, new scoring: goal 10
  becomes a cluster in the unmapped west, and the bedroom drops to sixth.

  it then ended "no reachable frontier left" at 6 min 33 with 42 frontier
  clusters still on the map and about a third of the flat unseen. Not one of
  them had been retired by the standing-point rule: every one was dropped
  because the frontier CELL did not touch the region the body can reach. The
  rover had spent the whole run in the hallway; the rooms are behind doorways
  its own inflated map prices at 0.54-0.58 m for a body that needs 0.60, so the
  flood stopped at the hallway - an 8.9 m2 pocket - and took all 42 with it.
  Eleven of them had a viewpoint the body could stand on within the standoff it
  already uses. So a cluster is now offered when there is somewhere to STAND
  and look at it, which is the question that was meant to be asked; the lidar
  reaches 12 m and a doorway is a fine place to map a room from. Replayed, the
  call that ended the run publishes a goal 7.6 m away with 65 frontier cells.

Field additions, all measured on VECTOR (spec §7):

  §7.1  failed-target memory (0.6 m) that NEVER causes extinction and holds
        no clock: a failed spot stays excluded until the WORLD reopens it -
        the map around the goal changed, or the rover looks from a viewpoint
        a metre from where it failed (owner, 26/08: triggers, never arbitrary
        timers). When every cluster is excluded the function returns a WAIT
        directive, not None. Starvation escape (lived 26/08 17h40: rover
        motionless, 9/11 excluded, nothing left to change the world): after
        WAIT_REOPEN_POLLS consecutive WAIT answers the OLDEST failed entry is
        reopened - the world stayed silent, so the oldest question is asked
        again. A count of asks, not a wall clock.
  §7.3  born-cornered detection: no reachable frontier AND a reachable free
        area under 0.5 m2 -> one back-off directive, before anything else. The
        same one back-off answers the other way a rover can be shut in: no
        reachable frontier while the MAP still holds frontier clusters. Pinched
        is not finished, and the two used to be reported with the same words.
  §4    prefer-forward, on the robot's real heading: in discovery mode the
        camera looks ahead, so a target behind costs a turn and a blind side.
  §7.2  keep-out zones need no code here: costmap2d.ScoredGrid.occupancy()
        forces them to 100, so they arrive as obstacles and the lethal
        clearance keeps every target a body-radius away from them.

The three directives all come back as a pose (the signature stays lesh's);
which one it is reads off `target.directive`.

Everything here is checked offline: tests/test_explorer2_cold.py for the
decisions, tools/explore_sim.py for a full A/B run against the old strategy on
a real saved map.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

# --- types -----------------------------------------------------------------
#
# The pure function must import with no dimOS installed (it is replayed on a
# laptop, tools/explore_sim.py). dimOS's own messages are used when they are
# there; otherwise a minimal stand-in with the same attribute names, which is
# all this module and its callers ever touch.

try:  # pragma: no cover - exercised by whichever machine runs this
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

    HAVE_DIMOS = True
except ImportError:  # no dimOS: replay/offline
    HAVE_DIMOS = False

    class _XYZ:
        __slots__ = ("x", "y", "z")

        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
            self.x, self.y, self.z = float(x), float(y), float(z)

    class _Quat:
        __slots__ = ("x", "y", "z", "w")

        def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0) -> None:
            self.x, self.y, self.z, self.w = float(x), float(y), float(z), float(w)

    class PoseStamped:  # type: ignore[no-redef]
        """Only what next_target reads and writes: position, orientation, frame, ts."""

        def __init__(self, ts: float | None = None, frame_id: str = "") -> None:
            self.position = _XYZ()
            self.orientation = _Quat()
            self.frame_id = frame_id
            self.ts = time.time() if ts is None else float(ts)

        def __repr__(self) -> str:
            return (f"PoseStamped(x={self.position.x:.3f}, y={self.position.y:.3f}, "
                    f"frame_id={self.frame_id!r})")


# --- grid conventions ------------------------------------------------------

FREE = 0
UNKNOWN = -1
OCCUPIED = 100
OCCUPANCY_THRESHOLD = 99      # dimOS WavefrontConfig.occupancy_threshold; costmap2d
                              # publishes 100 for obstacles AND for keep-out cells

_EIGHT = np.ones((3, 3), dtype=bool)
_SQRT2 = math.sqrt(2.0)
# (dy, dx, step length in cells)
_NEIGHBOURS = ((-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
               (0, -1, 1.0), (0, 1, 1.0),
               (1, -1, _SQRT2), (1, 0, 1.0), (1, 1, _SQRT2))


# --- directives ------------------------------------------------------------

DIRECTIVE_FRONTIER = "frontier"   # drive here (goal_request)
DIRECTIVE_WAIT = "wait"           # stay, re-evaluate in `wait_s` - NOT the end (§7.1)
DIRECTIVE_BACK_OFF = "back_off"   # cornered or pinched: reverse once, then re-evaluate (§7.3)

# How many of a cluster's nearest free viewpoints are tested for line of sight
# to it before the nearest one is taken anyway. Bounded on purpose: in the
# middle of a wide room every cell of the standoff window is a candidate, and
# an unbounded search would turn one decision into thousands of ray walks.
_LOS_CANDIDATES = 32

# The anti-revisit fade. Not a Tuning field: Tuning holds physical quantities,
# and this is the SHAPE of a preference, fixed by an argument rather than
# measured on the robot - what a second look at the same spot can add is the
# AREA the first revolution did not cover, and an area goes as the square of
# how far the rover moved.
_REVISIT_FADE_POWER = 2


# --- tuning ----------------------------------------------------------------

@dataclass(frozen=True)
class Tuning:
    """Every knob, in physical units. Frozen so a replay cannot drift."""

    # Clearance. Same two radii the planner uses, so the explorer never picks a
    # goal the planner will refuse (vector_dimos/recovering_planner.py:
    # LETHAL_CLEARANCE_M = width/2 + control margin, PIVOT_CLEARANCE_M = pivot
    # envelope / 2). Checked against the planner at import when dimOS is here.
    lethal_clearance_m: float = 0.30
    pivot_clearance_m: float = 0.39
    # Cost of driving one metre through the band the body fits in but cannot
    # pivot in, on top of the metre itself. Same 4th-power ramp shape as the
    # planner's gradient (recovering_planner.PIVOT_RAMP_EXPONENT): near zero at
    # the turning circle, all of it against the wall.
    pivot_cost_factor: float = 1.0
    pivot_ramp_exponent: int = 4
    # A metre of unknown counts as this many metres. dimOS's A* prices an
    # unknown cell at threshold * 0.95 against a free cell's 0 (PR #2830's
    # unknown_traversal_penalty), i.e. "only if there is nothing else"; in a
    # metres currency 4x is the same intent without forbidding it outright.
    unknown_cost_factor: float = 4.0

    # Frontiers. 0.3 m of perimeter is the blueprint's min_frontier_perimeter.
    min_frontier_perimeter_m: float = 0.30
    # A cluster the rover can only reach by betting on unmapped ground scores
    # this fraction of what a walk over seen floor scores. Not a ban: a probe
    # is how the next room gets found. Just last in the queue.
    probe_penalty: float = 0.25
    # Information gain is the UNKNOWN AREA the viewpoint can SEE within this
    # radius - _visible_unknown, one ray-cast revolution - not the length of the
    # frontier. PR #2830 uses the cluster cell count, which is the boundary's
    # length: a doorway into an unmapped room and a doorway into a cupboard have
    # the same one, and a strategy that divides it by path cost then always
    # takes the nearest door. Measured in the harness on the real flat, that
    # myopia cost +71 % travel against the old scoring, whose 4 m "lookahead
    # distance" happened to compensate for it. Area answers the question
    # actually being asked - how much map is behind this door - and line of
    # sight is what keeps the answer honest at a wall (see _visible_unknown).
    info_radius_m: float = 2.0
    # How far from the cluster the rover is asked to stand. It only has to SEE
    # the unknown (RPLIDAR C1, 12 m), not enter it.
    standoff_max_m: float = 1.00

    # Scoring.
    forward_bonus: float = 0.5     # PR #2830's momentum weight, on the real heading
    revisit_radius_m: float = 1.0  # within 1 m of a goal already published this
                                   # run, the score fades as the square of the
                                   # distance (_REVISIT_FADE_POWER)
    # Hard version of the same idea, and the only thing besides "no frontier"
    # that can end a run. A place the rover has ALREADY decided from - i.e.
    # already taken a full lidar revolution at - cannot be made to give up more
    # by going back to it: the rover would not move. That is a geometric fact,
    # not an attempt counter, and it is what lesh's "nothing reachable AND worth
    # it" means. Without it a frontier that no viewpoint can resolve (a shadow
    # the 2D scan plane never enters) is retargeted forever: measured in the
    # harness, 300 goals and 184 m of travel for the last 0.2 m2 of map.
    #
    # It is a rule about VIEWPOINTS, and only about viewpoints. A cluster whose
    # nearest viewpoint is spent still has the rest of its standoff window to be
    # looked at from; what retires the cluster itself is stated in _clusters.
    observed_radius_m: float = 0.30   # the follower's arrival tolerance (0.25 m) + a cell

    # §7.1 failed-target memory. Radius as shipped in fast_explorer.py. The
    # 60 s hold that came with it was an arbitrary clock (owner, 26/08): an
    # exclusion now lifts on a TRIGGER instead - the unknown-cell signature
    # around the goal changed, or the rover stands this far from where it
    # failed (a new viewpoint on the same spot).
    failed_goal_radius_m: float = 0.6
    failed_goal_moved_m: float = 1.0

    # The body is physically where it is: the cells under it are passable even
    # when the inflation says otherwise, or a rover that has driven into a pinch
    # can never be planned out of it. Same 0.25 m (half the body) dimOS's
    # GlobalPlanner._clear_robot_footprint uses before every A*.
    footprint_clear_m: float = 0.25

    # §7.3 born-cornered.
    cornered_area_m2: float = 0.5
    back_off_m: float = 0.22       # middle of the 0.20-0.25 m that freed it by hand


DEFAULT_TUNING = Tuning()

# Starvation escape: consecutive WAIT answers before the oldest failed entry is
# reopened. Counts ASKS (the loop polls once per WAIT_POLL_S), not wall time -
# a replay driving next_target by hand starves and recovers identically.
WAIT_REOPEN_POLLS = 12


# --- state -----------------------------------------------------------------

@dataclass
class ExploreState:
    """The only memory of the exploration. Plain data: pickles, replays, diffs.

    `next_target` writes ONLY these fields, and only these:
      visited          appended with every frontier target it hands out
      observed         appended with the pose it was called from (deduplicated);
                       every entry is a place the rover really stood, and only
                       these retire a frontier
      failed           entries whose reopening trigger fired are pruned (the
                       loop appends new ones)
      heading          refreshed from the pose it was given
      back_off_issued  set when a back-off goes out, cleared when a frontier does
      last_directive   what it just returned ("" if None)
      targets_issued   count of frontier targets handed out
    Nothing else in the process is touched: no globals, no costmap writes.
    """

    visited: list[tuple[float, float]] = field(default_factory=list)
    # Where the rover has actually stood and decided - one full lidar revolution
    # each. Appended by next_target itself from the pose it is given, so it stays
    # true without the loop having to report anything.
    observed: list[tuple[float, float]] = field(default_factory=list)
    # (goal_x, goal_y, robot_x_at_failure, robot_y_at_failure, unknown_sig)
    failed: list[tuple[float, float, float, float, int]] = field(default_factory=list)
    heading: float = 0.0
    back_off_issued: bool = False
    last_directive: str = ""
    targets_issued: int = 0
    wait_streak: int = 0        # consecutive WAITs; WAIT_REOPEN_POLLS triggers the starvation escape

    def note_failed(self, x: float, y: float, robot_xy: tuple[float, float],
                    costmap: Any, radius_m: float = DEFAULT_TUNING.failed_goal_radius_m) -> None:
        """The planner gave up on this goal. Called by the loop, not by next_target.

        Stores where the rover STOOD when it failed and the unknown-cell
        signature around the goal: the two things whose change reopens it."""
        self.failed.append((float(x), float(y), float(robot_xy[0]), float(robot_xy[1]),
                            unknown_signature(costmap, x, y, radius_m)))

    def copy(self) -> "ExploreState":
        return replace(self, visited=list(self.visited), failed=list(self.failed),
                       observed=list(self.observed))


def unknown_signature(costmap: Any, x: float, y: float, radius_m: float) -> int:
    """Number of UNKNOWN cells within radius_m of (x, y) on this costmap.

    The reopening trigger of a §7.1 exclusion: cells once observed stay
    observed, so this count only moves when genuinely new information lands
    around the failed goal - sensor noise flipping free<->occupied does not
    touch it. Computed against world coordinates, so a moving crop origin
    does not shift it; a goal that falls OFF the published crop reads 0,
    which at worst reopens it early (one retry, one re-exclusion)."""
    grid = np.asarray(costmap.grid)
    res = float(costmap.resolution)
    ox = float(costmap.origin.position.x)
    oy = float(costmap.origin.position.y)
    h, w = grid.shape
    r = max(1, int(round(radius_m / res)))
    cx = int(math.floor((x - ox) / res))
    cy = int(math.floor((y - oy) / res))
    x0, x1 = max(0, cx - r), min(w, cx + r + 1)
    y0, y1 = max(0, cy - r), min(h, cy + r + 1)
    if x0 >= x1 or y0 >= y1:
        return 0
    return int((grid[y0:y1, x0:x1] == -1).sum())


# --- geometry helpers ------------------------------------------------------

def _yaw_of(pose: Any, fallback: float) -> float:
    """Heading in the costmap frame, from the pose quaternion."""
    q = getattr(pose, "orientation", None)
    if q is None:
        return fallback
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    if x * x + y * y + z * z + w * w < 1e-9:
        return fallback
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _target_pose(x: float, y: float, *, frame_id: str, ts: float,
                 directive: str, **extra: Any) -> PoseStamped:
    p = PoseStamped(ts=ts, frame_id=frame_id)
    p.position.x, p.position.y, p.position.z = float(x), float(y), 0.0
    p.orientation.x = p.orientation.y = p.orientation.z = 0.0
    p.orientation.w = 1.0
    p.directive = directive          # type: ignore[attr-defined]
    for key, value in extra.items():
        setattr(p, key, value)
    return p


def _disc(shape: tuple[int, int], centre: tuple[int, int], radius_cells: float) -> np.ndarray:
    """Boolean disc of `radius_cells` around `centre`, clipped to the grid."""
    cy, cx = centre
    r = int(math.ceil(radius_cells))
    h, w = shape
    y0, y1 = max(0, cy - r), min(h, cy + r + 1)
    x0, x1 = max(0, cx - r), min(w, cx + r + 1)
    out = np.zeros(shape, dtype=bool)
    yy = np.arange(y0, y1)[:, None] - cy
    xx = np.arange(x0, x1)[None, :] - cx
    out[y0:y1, x0:x1] = (yy ** 2 + xx ** 2) <= radius_cells ** 2
    return out


def _nearest_true(mask: np.ndarray, yx: tuple[int, int]) -> tuple[int, int] | None:
    """Cell of `mask` closest to `yx`, Euclidean. None if the mask is empty."""
    if not mask.any():
        return None
    y, x = yx
    if mask[y, x]:
        return y, x
    _, idx = ndimage.distance_transform_edt(~mask, return_indices=True)
    return int(idx[0][y, x]), int(idx[1][y, x])


# --- what a viewpoint can see ----------------------------------------------
#
# Two questions, one primitive: does a straight line between two cells cross an
# obstacle. "How much unknown would a lidar revolution here reveal" is that
# question asked once per ray; "was this frontier ever in view from there" is it
# asked once. Both are answered on the RAW occupancy - unknown never blocks a
# ray, because unknown is exactly what the rover is going to find out.

_RAYS: dict[int, tuple[np.ndarray, np.ndarray]] = {}


def _rays(radius_cells: int) -> tuple[np.ndarray, np.ndarray]:
    """(dy, dx) of one revolution: enough rays that no cell of the disc is missed.

    One ray per cell of the outer circumference (2 pi r), sampled every cell
    outwards. Cached per radius, which is a constant of the tuning, so the whole
    cost is paid once per process.
    """
    key = int(radius_cells)
    cached = _RAYS.get(key)
    if cached is None:
        n_rays = max(8, int(math.ceil(2.0 * math.pi * key)))
        angles = np.linspace(0.0, 2.0 * math.pi, n_rays, endpoint=False)
        steps = np.arange(1, key + 1)
        dy = np.rint(np.sin(angles)[:, None] * steps[None, :]).astype(np.int32)
        dx = np.rint(np.cos(angles)[:, None] * steps[None, :]).astype(np.int32)
        _RAYS[key] = cached = (dy, dx)
    return cached


def _visible_unknown(unknown: np.ndarray, occupied: np.ndarray,
                     centre: tuple[int, int], radius_cells: int) -> float:
    """Unknown cells in LINE OF SIGHT of `centre`, as a fraction of its disc.

    This is the information gain, and it is the answer to the question actually
    being asked: how much of what I do not know would ONE revolution from that
    spot show me. A box filter over the unknown mask - what this used to be -
    counts the unknown on the far side of the wall the rover would be standing
    against, and on the real flat that is most of it: at the decision the owner
    objected to on 26/08 (recordings/courseB_explorer2.db, goal 10) the box gave
    the frontier at the bedroom wall 0.514 and the ray-cast gives it 0.327,
    against 0.70-0.78 for the frontiers that open onto the unmapped rooms.
    """
    dy, dx = _rays(radius_cells)
    h, w = unknown.shape
    cy, cx = centre
    ys, xs = cy + dy, cx + dx
    inside = (ys >= 0) & (ys < h) & (xs >= 0) & (xs < w)
    ysc = np.clip(ys, 0, h - 1)
    xsc = np.clip(xs, 0, w - 1)
    blocked = occupied[ysc, xsc] | ~inside
    steps = blocked.shape[1]
    first = np.where(blocked.any(axis=1), blocked.argmax(axis=1), steps)
    open_run = np.arange(steps)[None, :] < first[:, None]
    hit = open_run & inside & unknown[ysc, xsc]
    if not hit.any():
        return 0.0
    # rays overlap near the centre: count CELLS, not ray samples
    seen = np.zeros((2 * radius_cells + 1, 2 * radius_cells + 1), dtype=bool)
    seen[dy[hit] + radius_cells, dx[hit] + radius_cells] = True
    return float(seen.sum()) / (math.pi * radius_cells * radius_cells)


def _line_of_sight(occupied: np.ndarray, a: tuple[int, int], b: tuple[int, int]) -> bool:
    """True when no obstacle cell sits strictly between the two cells."""
    (y0, x0), (y1, x1) = a, b
    n = int(max(abs(y1 - y0), abs(x1 - x0)))
    if n <= 1:
        return True
    t = np.arange(1, n) / n
    ys = np.rint(y0 + (y1 - y0) * t).astype(np.int64)
    xs = np.rint(x0 + (x1 - x0) * t).astype(np.int64)
    return not bool(occupied[ys, xs].any())


def _cells_of(points: list[tuple[float, float]], shape: tuple[int, int], res: float,
              origin: tuple[float, float]) -> np.ndarray:
    """World points -> an (n, 2) array of (row, col), clipped to the grid."""
    if not points:
        return np.zeros((0, 2), dtype=np.int64)
    ox, oy = origin
    h, w = shape
    ys = np.clip(np.floor((np.array([p[1] for p in points]) - oy) / res), 0, h - 1)
    xs = np.clip(np.floor((np.array([p[0] for p in points]) - ox) / res), 0, w - 1)
    return np.stack([ys, xs], axis=1).astype(np.int64)


def _decided_from(observed: list[tuple[float, float]], shape: tuple[int, int], res: float,
                  origin: tuple[float, float], radius_m: float) -> np.ndarray:
    """Cells within `radius_m` of a pose the rover has already decided from.

    A viewpoint inside this mask has had its lidar revolution: sending the rover
    back to it cannot show it anything, whatever the frontier it would be aimed
    at. That is the only thing the rule below is allowed to conclude.
    """
    mask = np.zeros(shape, dtype=bool)
    cells = _cells_of(observed, shape, res, origin)
    if not cells.size:
        return mask
    mask[cells[:, 0], cells[:, 1]] = True
    r = int(math.ceil(radius_m / res))
    if r <= 0:
        return mask
    yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
    return ndimage.binary_dilation(mask, structure=(yy * yy + xx * xx <= r * r))


# --- the map, read once ----------------------------------------------------

@dataclass
class _Survey:
    """Everything next_target needs to know about this costmap, computed once."""

    free: np.ndarray
    unknown: np.ndarray
    occupied: np.ndarray          # obstacles and keep-out cells, as published
    lethal: np.ndarray            # body does not fit (obstacles + keep-outs, inflated)
    reachable: np.ndarray         # free or unknown, body fits, connected to the robot
    goal_ok: np.ndarray           # free and reachable: where a goal may sit
    walkable: np.ndarray          # ... of which this part needs no bet on unmapped ground
    cell_cost: np.ndarray         # cost multiplier per cell (1.0 = a plain free metre)
    seed: tuple[int, int] | None  # the cell the robot plans from
    reachable_free_m2: float


def _survey(grid: np.ndarray, res: float, robot_yx: tuple[int, int], tuning: Tuning) -> _Survey:
    occupied = grid >= OCCUPANCY_THRESHOLD
    free = grid == FREE
    unknown = grid == UNKNOWN

    # Metres to the nearest obstacle. Keep-outs are already 100 in the grid
    # (costmap2d.ScoredGrid.occupancy applies them after every layer), so they
    # inflate exactly like a wall and no target can land within a body radius.
    distance_m = ndimage.distance_transform_edt(~occupied) * res
    # Strictly inside the radius, with the same float tolerance as the planner:
    # a cell sitting exactly ON the radius is the centre of a door the body
    # fits through, and rounding it away is what walled off 60-70 cm gaps.
    lethal = distance_m + 1e-6 < tuning.lethal_clearance_m

    # 1. Where the rover physically is: the free+unknown region it sits in,
    #    NOT inflated. Snapping across a wall is how a cornered rover was
    #    reported as being in the room next door.
    physical = free | unknown
    ry, rx = robot_yx
    if not physical[ry, rx]:
        snapped = _nearest_true(physical, (ry, rx))
        if snapped is None:
            empty = np.zeros_like(free)
            return _Survey(free, unknown, occupied, lethal, empty, empty, empty,
                           np.ones(grid.shape), None, 0.0)
        ry, rx = snapped
    comp_labels, _ = ndimage.label(physical, structure=_EIGHT)
    component = comp_labels == comp_labels[ry, rx]

    # 2. Where the BODY can go inside that region, starting from under the
    #    body itself. The footprint is cleared exactly as the planner clears it
    #    before every A*: without that, a rover standing in a pinch has no
    #    traversable cell of its own, and picking the nearest one by Euclidean
    #    distance walks straight through the wall it is pinched against - the
    #    explorer then certifies goals the planner answers "no path" to
    #    (measured in the harness: 78 of 80 goals blocked, 7 m of travel).
    lethal = lethal.copy()
    footprint = _disc(lethal.shape, (ry, rx), tuning.footprint_clear_m / res)
    lethal[footprint] = False
    traversable = component & ~lethal
    if not traversable[ry, rx]:
        # The body is not even on free/unknown ground it fits in: nothing this
        # function can plan. That is the born-cornered signature, judged on the
        # pocket's own free area, not on the inflated one.
        empty = np.zeros_like(free)
        return _Survey(free, unknown, occupied, lethal, empty, empty, empty,
                       np.ones(grid.shape), None,
                       float((component & free).sum()) * res * res)
    seed = (ry, rx)
    trav_labels, _ = ndimage.label(traversable, structure=_EIGHT)
    reachable = trav_labels == trav_labels[seed]

    # Two reachability sets, and the difference between them is the whole
    # question "can I walk there, or am I hoping?":
    #
    #   reachable       free OR unknown, body fits, connected to the rover. This
    #                   is dimOS's own set (detect_frontiers floods free|unknown)
    #                   and it is what makes a frontier behind an unmapped pocket
    #                   visible at all - which it should be, that is exploration.
    #   walkable        the same, over floor the rover has actually SEEN. Every
    #                   cell the lidar marked free is joined to the rover by the
    #                   ray that marked it, so this set is what the rover can
    #                   reach without betting on unmapped ground.
    #
    # A cluster whose standing point is in `walkable` is a safe errand. One that
    # can only be got to across unmapped ground is a PROBE: still offered with a
    # proper free standing point, because refusing to try unmapped routes is
    # refusing to explore, but scored down (Tuning.probe_penalty) so every safe
    # frontier goes first. Measured in the harness: with probes ranked equal to
    # safe goals, one start pose spent 33 goals, 40 m and 183 direction reversals
    # driving into door frames.
    walk_labels, _ = ndimage.label((free | footprint) & ~lethal, structure=_EIGHT)
    walkable = walk_labels == walk_labels[seed]

    goal_ok = reachable & free
    reachable_free_m2 = float((walkable & free).sum()) * res * res

    # Cost per cell, in "metres per metre". Free and clear = 1.0.
    ramp = np.clip((tuning.pivot_clearance_m - distance_m)
                   / max(tuning.pivot_clearance_m - tuning.lethal_clearance_m, 1e-6), 0.0, 1.0)
    cell_cost = (1.0 + tuning.pivot_cost_factor * ramp ** tuning.pivot_ramp_exponent)
    cell_cost = np.where(unknown, cell_cost * tuning.unknown_cost_factor, cell_cost)
    return _Survey(free, unknown, occupied, lethal, reachable, goal_ok, walkable,
                   cell_cost.astype(np.float64), seed, reachable_free_m2)


def _path_cost(reachable: np.ndarray, cell_cost: np.ndarray, seed: tuple[int, int],
               res: float) -> np.ndarray:
    """Geodesic cost in metres from `seed` to every reachable cell.

    One Dijkstra over the 8-connected grid, edge weight = mean cell cost x step
    length. PR #2830 runs one A* per cluster for the same number; a single
    multi-target search costs the same as its first one, which is what let the
    16 s wavefront search go away for good.
    """
    h, w = reachable.shape
    out = np.full((h, w), np.inf)
    ys, xs = np.nonzero(reachable)
    n = int(ys.size)
    if n == 0:
        return out
    index = np.full((h, w), -1, dtype=np.int64)
    index[ys, xs] = np.arange(n)

    rows, cols, vals = [], [], []
    for dy, dx, length in _NEIGHBOURS:
        y0, y1 = max(0, -dy), h - max(0, dy)
        x0, x1 = max(0, -dx), w - max(0, dx)
        if y1 <= y0 or x1 <= x0:
            continue
        a = index[y0:y1, x0:x1]
        b = index[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
        both = (a >= 0) & (b >= 0)
        if not both.any():
            continue
        ca = cell_cost[y0:y1, x0:x1][both]
        cb = cell_cost[y0 + dy:y1 + dy, x0 + dx:x1 + dx][both]
        rows.append(a[both])
        cols.append(b[both])
        vals.append(0.5 * (ca + cb) * length * res)

    if not rows:
        out[ys[0], xs[0]] = 0.0
        return out
    graph = csr_matrix((np.concatenate(vals),
                        (np.concatenate(rows), np.concatenate(cols))), shape=(n, n))
    distances = dijkstra(graph, directed=True, indices=int(index[seed]))
    out[ys, xs] = distances
    return out


# --- frontier clusters -----------------------------------------------------

@dataclass
class Cluster:
    """One frontier, with the place the rover would look at it from."""

    size: int                        # frontier cells (PR #2830's own gain proxy)
    gain: float                      # unknown cells in line of sight of the viewpoint,
                                     # as a fraction of its disc: the gain actually used
    centroid_xy: tuple[float, float]
    look_at_xy: tuple[float, float]  # the cluster cell nearest the centroid
    goal_xy: tuple[float, float]     # where to stand
    goal_yx: tuple[int, int]
    on_frontier: bool                # True when no free standing spot was found
    probe: bool                      # getting there means betting on unmapped ground
    retired: bool = False            # every viewpoint of it has already been used
    in_sight: bool = True            # the viewpoint can actually SEE the frontier


def _frontier_mask(survey: _Survey) -> np.ndarray:
    """dimOS's own frontier definition, kept as it is: an UNKNOWN cell with a
    FREE 8-neighbour and no occupied 8-neighbour."""
    near_occupied = ndimage.binary_dilation(survey.occupied, structure=_EIGHT)
    near_free = ndimage.binary_dilation(survey.free, structure=_EIGHT)
    return survey.unknown & near_free & ~near_occupied


def _count_clusters(frontier: np.ndarray, min_cells: int) -> int:
    """How many frontier clusters the MAP holds, reachable or not.

    The difference between this number and the number of clusters the rover can
    plan to is the whole difference between "there is nothing left to see" and
    "I am pinched": see next_target. Run B of 26/08 ended on that confusion with
    37 clusters and 1492 frontier cells on the map, none of them touching the
    8.9 m2 pocket the body was closed into.
    """
    if not frontier.any():
        return 0
    labels, count = ndimage.label(frontier, structure=_EIGHT)
    if count == 0:
        return 0
    sizes = np.bincount(labels.ravel())[1:]
    return int((sizes >= min_cells).sum())


def _clusters(survey: _Survey, frontier_all: np.ndarray, grid_shape: tuple[int, int],
              res: float, origin: tuple[float, float], tuning: Tuning,
              decided_from: np.ndarray, observed_yx: np.ndarray) -> list[Cluster]:
    """The reachable frontier clusters, each with the viewpoint to look at it from.

    The body radius deliberately does NOT filter the frontier cells. A frontier
    is a measurement of unknown area, not a place to drive; narrowing it would
    throw away information and bring back the false extinctions. Where the rover
    stands to look at it is a separate question, answered below with
    `survey.goal_ok`, which is body-aware.

    `decided_from` are the cells the rover has already taken a lidar revolution
    at, and `observed_yx` those revolutions' centres. They are not allowed as
    viewpoints - driving to a spot the rover is effectively already at reveals
    nothing, which is geometry and not a counter.

    A cluster is RETIRED when the frontier itself has already been in CLEAR
    VIEW - line of sight, within standoff range - from a spot the rover stood
    at. If a full revolution with nothing in the way left those cells unknown,
    no second viewpoint at that scale is going to resolve them: that is the
    shadow the 2D scan plane never enters, and retargeting it forever is the
    300-goals-and-184-m pathology the rule exists for. The shipped rule retired
    a cluster as soon as its NEAREST viewpoint had been used, whether or not
    anything of the frontier could be seen from there - so a frontier round the
    corner from a spot the rover happened to stand at died silently, and the
    log said "exploration complete".

    A cluster is offered when there is somewhere the BODY CAN STAND within
    standoff_max_m of it. That is the real question, and it is not the same as
    the shipped test - "the frontier cell itself touches the region the body can
    reach" - which is what ended run B of 26/08: the rover spent the whole run
    in the flat's hallway, saw into every room through the doorways, and then
    dropped all 42 clusters because the rooms are behind doorways the inflated
    map prices 1 cm too narrow for the body. Eleven of those clusters had a
    viewpoint the body could stand on within the existing 1.00 m standoff. The
    lidar reaches 12 m: standing in a doorway maps the room, and that is what
    this rover has to do until a doorway is measured wide enough to drive
    through.
    """
    frontier = frontier_all
    if not frontier.any():
        return []

    labels, count = ndimage.label(frontier, structure=_EIGHT)
    if count == 0:
        return []
    min_cells = max(1, int(tuning.min_frontier_perimeter_m / res))
    boxes = ndimage.find_objects(labels)
    ox, oy = origin
    h, w = grid_shape
    pad = int(tuning.standoff_max_m / res) + 2
    standoff_cells = tuning.standoff_max_m / res
    info_cells = max(1, int(round(tuning.info_radius_m / res)))
    # "close enough for a revolution to have resolved it": one standoff, plus
    # the tolerance on where the rover actually came to a stop.
    seen_cells = (tuning.standoff_max_m + tuning.observed_radius_m) / res
    out: list[Cluster] = []
    for i, box in enumerate(boxes, start=1):
        if box is None:
            continue
        sub = labels[box] == i
        size = int(sub.sum())
        if size < min_cells:
            continue
        ys = np.nonzero(sub)[0] + box[0].start
        xs = np.nonzero(sub)[1] + box[1].start
        cy, cx = float(ys.mean()), float(xs.mean())

        # The point to LOOK AT: the cluster cell nearest the centroid, not the
        # centroid itself. A frontier that wraps around the known region (a
        # ring) has its centroid in the middle of the room, where there is
        # nothing to see; dimOS publishes that centroid as the goal and the
        # rover drives 5 cm. Snapping onto the cluster keeps the target on the
        # edge of what is known, which is the whole point of a frontier.
        rep = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
        ry_c, rx_c = int(ys[rep]), int(xs[rep])

        # Has this frontier already been looked straight at, from close enough?
        seen_clearly = False
        if observed_yx.size:
            d2 = ((observed_yx[:, 0] - ry_c) ** 2 + (observed_yx[:, 1] - rx_c) ** 2)
            close = np.nonzero(d2 <= seen_cells * seen_cells)[0]
            for i in close[np.argsort(d2[close])]:
                if _line_of_sight(survey.occupied,
                                  (int(observed_yx[i, 0]), int(observed_yx[i, 1])), (ry_c, rx_c)):
                    seen_clearly = True
                    break

        # The place to STAND: a free, reachable, body-fits cell within
        # standoff_max_m of that look-at point, searched only in a window around
        # it so a free cell on the far side of a wall cannot be picked by
        # Euclidean luck. Of those candidates: never one the rover has already
        # decided from, and, among the rest, the nearest one that can actually
        # SEE the frontier - a viewpoint round the corner from the thing it is
        # sent to look at is a wasted goal.
        wy0, wy1 = max(0, ry_c - pad), min(h, ry_c + pad + 1)
        wx0, wx1 = max(0, rx_c - pad), min(w, rx_c + pad + 1)
        window = survey.goal_ok[wy0:wy1, wx0:wx1]
        gy = gx = None
        retired = in_sight = False
        if window.any():
            wys, wxs = np.nonzero(window)
            ys_abs, xs_abs = wys + wy0, wxs + wx0
            d2 = (ys_abs - ry_c) ** 2 + (xs_abs - rx_c) ** 2
            near = d2 <= standoff_cells * standoff_cells
            if near.any():
                ys_abs, xs_abs = ys_abs[near], xs_abs[near]
                order = np.argsort(d2[near], kind="stable")
                free_slots = [i for i in order if not decided_from[ys_abs[i], xs_abs[i]]]
                if not free_slots:
                    # Not one viewpoint left that the rover is not already at.
                    retired = True
                    gy, gx = int(ys_abs[order[0]]), int(xs_abs[order[0]])
                else:
                    pick = free_slots[0]
                    for i in free_slots[:_LOS_CANDIDATES]:
                        if _line_of_sight(survey.occupied, (ys_abs[i], xs_abs[i]), (ry_c, rx_c)):
                            pick, in_sight = i, True
                            break
                    gy, gx = int(ys_abs[pick]), int(xs_abs[pick])
                    # Retired when there is nothing more this rover can learn
                    # about it from where it can stand: it has already been
                    # looked straight at, or nowhere within the standoff can
                    # see it at all. Both are read off THIS map, so both undo
                    # themselves as soon as the map near it changes.
                    retired = seen_clearly or not in_sight
        on_frontier = gy is None
        if on_frontier:
            if not survey.reachable[ry_c, rx_c]:
                # Nowhere to stand within standoff, and the frontier itself is
                # not in the region the body can reach either. This one is out
                # of reach from where the rover stands - counted by the caller
                # (a map that still holds frontiers is not an explored map),
                # never published, because the planner would refuse it.
                continue
            # Nowhere to stand within reach, but the frontier is inside the
            # region the body can reach: fall back to dimOS's own behaviour and
            # aim at the frontier cell itself, in unknown space. Harder to
            # reach, but dropping the cluster would be a silent extinction.
            gy, gx = ry_c, rx_c
            in_sight = True
            retired = seen_clearly
        # Cell CENTRES, not dimOS's grid_to_world corner. A corner is shared by
        # four cells and floor((corner - origin) / res) can land on any of them
        # (measured: floor(3.7 / 0.05) == 73, not 74), so a goal published on a
        # corner came back as the neighbouring cell - which was inside the
        # inflation, and the planner answered "no path" to a goal the explorer
        # had just certified as clear. A centre is half a cell from any edge.
        out.append(Cluster(
            size=size,
            gain=_visible_unknown(survey.unknown, survey.occupied, (gy, gx), info_cells),
            centroid_xy=(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res),
            look_at_xy=(ox + (rx_c + 0.5) * res, oy + (ry_c + 0.5) * res),
            goal_xy=(ox + (gx + 0.5) * res, oy + (gy + 0.5) * res),
            goal_yx=(gy, gx),
            on_frontier=on_frontier,
            probe=on_frontier or not survey.walkable[gy, gx],
            retired=retired,
            in_sight=in_sight,
        ))
    return out


# --- the pure function -----------------------------------------------------

def next_target(costmap: Any, pose: Any, state: ExploreState, *,
                now: float | None = None, tuning: Tuning = DEFAULT_TUNING) -> PoseStamped | None:
    """The next thing to do, or None when exploration is over.

    Args:
        costmap: anything with `.grid` (int8 HxW, 100/0/-1), `.resolution`,
            `.origin.position.x/.y` and optionally `.frame_id`/`.ts`. A dimOS
            OccupancyGrid is one; so is the stand-in the replay harness builds.
            Pass the RAW costmap - this function does its own inflation, and
            the old loop's `simple_inflate(costmap, 0.25)` would double it.
        pose: the robot pose, `.position.x/.y` and `.orientation` (quaternion).
        state: the memory. Updated in place; see ExploreState.
        now: only stamps the returned pose when the costmap carries no ts.
            Defaults to time.monotonic(); nothing behavioural reads a clock.

    Returns:
        A pose carrying `.directive`:
          "frontier"  drive there (this is a goal_request)
          "wait"      stay put, re-evaluate in `.wait_s` seconds; a cluster is
                      temporarily excluded, NOT absent (§7.1)
          "back_off"  reverse `.back_off_m`; born cornered (§7.3)
        or None, which happens on exactly one condition: nothing reachable is
        left to look at and no exclusion is holding anything back - either the
        map holds no frontier cluster at all, or every one it holds has already
        been looked straight at from where the rover stood, or nowhere the body
        can stand can see it. No counters, no attempt limits, no timers; a rover
        that is merely shut in gets a back-off, not a None.
    """
    now = time.monotonic() if now is None else float(now)
    grid = np.asarray(costmap.grid)
    res = float(costmap.resolution)
    ox = float(costmap.origin.position.x)
    oy = float(costmap.origin.position.y)
    frame_id = getattr(costmap, "frame_id", "world")
    ts = float(getattr(costmap, "ts", now))

    rx, ry_world = float(pose.position.x), float(pose.position.y)
    heading = _yaw_of(pose, state.heading)
    state.heading = heading
    # This pose has just had a lidar revolution. Record it once per half-radius
    # so the list stays short over a long run.
    half = tuning.observed_radius_m / 2.0
    if all((rx - ox_) ** 2 + (ry_world - oy_) ** 2 >= half * half for ox_, oy_ in state.observed):
        state.observed.append((rx, ry_world))

    # An exclusion whose reopening trigger fired is gone: the map around the
    # goal changed (unknown-cell signature moved), or the rover now stands a
    # viewpoint away from where it failed. No clock anywhere (owner, 26/08).
    # §7.1 exists because the old loop counted exclusions as "no frontier"
    # and completed while ten clusters were waiting.
    moved2 = tuning.failed_goal_moved_m ** 2
    state.failed = [
        f for f in state.failed
        if (rx - f[2]) ** 2 + (ry_world - f[3]) ** 2 < moved2
        and unknown_signature(costmap, f[0], f[1], tuning.failed_goal_radius_m) == f[4]
    ]

    h, w = grid.shape
    gx0 = int(np.clip(math.floor((rx - ox) / res), 0, w - 1))
    gy0 = int(np.clip(math.floor((ry_world - oy) / res), 0, h - 1))

    survey = _survey(grid, res, (gy0, gx0), tuning)
    frontier_all = _frontier_mask(survey)
    min_cells = max(1, int(tuning.min_frontier_perimeter_m / res))
    on_the_map = _count_clusters(frontier_all, min_cells)
    # A viewpoint is spent by STANDING there, and by nothing else. Counting a
    # goal that was merely issued would make this an attempt counter wearing a
    # geometric hat, and it shows: with issued goals spent too, the harness
    # runs ended having given up on a frontier whose viewpoint the planner
    # could still reach from where the rover was standing when it stopped
    # (3 of 4 starts). What stops a goal being re-issued for ever is the loop's
    # §7.1 exclusion on a drive that made no progress - trigger-reopened, and
    # one that can only ever produce a WAIT.
    decided_from = _decided_from(state.observed, (h, w), res, (ox, oy),
                                 tuning.observed_radius_m)
    observed_yx = _cells_of(state.observed, (h, w), res, (ox, oy))
    clusters = ([] if survey.seed is None
                else _clusters(survey, frontier_all, (h, w), res, (ox, oy), tuning,
                               decided_from, observed_yx))

    # --- §7.3 nothing reachable: pinched, cornered, or actually finished ----
    #
    # These are three different facts and the shipped version reported all three
    # as the same "no reachable frontier left":
    #
    #   finished   the MAP holds no frontier cluster at all. Nothing left to
    #              see, and that is the one honest end of a run.
    #   cornered   no frontier and under cornered_area_m2 of floor: the rover is
    #              wedged in a pocket (§7.3, born cornered).
    #   pinched    the map still holds frontier clusters, but not one of them
    #              touches the region the BODY can reach from where it stands.
    #              That is not "explored", it is "shut in": run B of 26/08 ended
    #              this way at 6 min 33 with 37 clusters on the map, sealed into
    #              8.9 m2 by a doorway the inflated map prices at 0.54 m for a
    #              body that needs 0.60 - a doorway its own wheels had come
    #              through minutes earlier (min clearance along its last 240 s
    #              of trajectory, on that same map: 0.269 m).
    #
    # Cornered and pinched get the same answer, which is the reflex that freed
    # the rover by hand three times: reverse once and look again. Once per
    # pocket, exactly as before - the flag is cleared only when a frontier goal
    # goes out - so this can never become a spin.
    if not clusters:
        if on_the_map > 0 or survey.reachable_free_m2 < tuning.cornered_area_m2:
            if not state.back_off_issued:
                state.back_off_issued = True
                state.last_directive = DIRECTIVE_BACK_OFF
                return _target_pose(rx - tuning.back_off_m * math.cos(heading),
                                    ry_world - tuning.back_off_m * math.sin(heading),
                                    frame_id=frame_id, ts=ts, directive=DIRECTIVE_BACK_OFF,
                                    back_off_m=tuning.back_off_m,
                                    reachable_free_m2=survey.reachable_free_m2,
                                    n_clusters=0, n_on_the_map=on_the_map)
        # Backed off once already and the picture has not changed, or the map
        # holds no frontier at all: that is the end.
        state.last_directive = ""
        return None

    # --- §7.1 temporary exclusions -----------------------------------------
    radius2 = tuning.failed_goal_radius_m ** 2
    eligible: list[Cluster] = []
    n_blocked = 0
    n_seen_from = 0
    for cluster in clusters:
        gx, gy = cluster.goal_xy
        if cluster.retired:
            # Every viewpoint within reach of this frontier has already had its
            # revolution. Not a failure and not a timer: driving to a spot the
            # rover is effectively already at reveals nothing, wherever it is
            # then asked to look.
            n_seen_from += 1
            continue
        if any((gx - f[0]) ** 2 + (gy - f[1]) ** 2 < radius2 for f in state.failed):
            n_blocked += 1
        else:
            eligible.append(cluster)

    if not eligible and not n_blocked:
        # Every cluster is one the rover has already stood at and looked from.
        # Nothing reachable is worth going to: that is the end of the run.
        state.last_directive = ""
        return None

    if not eligible:
        # Every remaining cluster sits on a failed goal whose trigger has not
        # fired: the map around it has not changed and the rover has not found
        # a new viewpoint. Waiting is the answer, dying is not - and there is
        # no expiry to wait out: the loop polls, and the WORLD reopens the
        # spot. But a motionless rover cannot change the world (lived 26/08:
        # 9/11 excluded, infinite wait), so after WAIT_REOPEN_POLLS silent
        # asks the OLDEST failed entry is reopened and asked again.
        state.wait_streak += 1
        reopened = None
        if state.failed and state.wait_streak >= WAIT_REOPEN_POLLS:
            reopened = state.failed.pop(0)[:2]
            state.wait_streak = 0
        state.last_directive = DIRECTIVE_WAIT
        return _target_pose(rx, ry_world, frame_id=frame_id, ts=ts,
                            directive=DIRECTIVE_WAIT, wait_s=0.0,
                            reopened_xy=reopened,
                            n_clusters=len(clusters), n_excluded=n_blocked,
                            n_already_seen_from=n_seen_from, n_on_the_map=on_the_map)

    # --- score (spec §4B / PR #2830) ---------------------------------------
    costs = _path_cost(survey.reachable, survey.cell_cost, survey.seed, res)

    best: Cluster | None = None
    best_score = -math.inf
    best_cost = math.inf
    for cluster in eligible:
        gy, gx = cluster.goal_yx
        cost = float(costs[gy, gx]) if survey.reachable[gy, gx] else math.inf
        if not math.isfinite(cost):
            # Unreachable: PR #2830's -inf, dropped rather than published.
            continue
        score = cluster.gain / (1.0 + cost)
        if cluster.probe:
            score *= tuning.probe_penalty

        gxw, gyw = cluster.goal_xy
        dx, dy = gxw - rx, gyw - ry_world
        span = math.hypot(dx, dy)
        if span > 0.1:   # under 10 cm a direction means nothing (dimOS uses the same floor)
            align = max(0.0, math.cos(heading) * dx / span + math.sin(heading) * dy / span)
            score *= 1.0 + tuning.forward_bonus * align

        # Anti-revisit. Two changes on what shipped, both from the run the
        # owner watched on 26/08 (his words: "re-visiting is only worth it once
        # the other rooms are done"):
        #
        #   what counts   every place the rover has already SWEPT counts, not
        #                 only the goals it was sent to. That is what
        #                 revisit_radius_m has always said it meant.
        #   how it fades  as the square of the distance, because what a second
        #                 look can add is an AREA - the crescent the first
        #                 revolution did not cover - and an area goes as the
        #                 square of how far the rover moved. Linear was too
        #                 gentle to matter: at goal 10 of run B the fade was
        #                 already 0.10 on the frontier 10 cm from a goal
        #                 published 6 minutes earlier, and it still won,
        #                 because a mapped-floor errand takes neither the x4
        #                 probe penalty nor the distance the unmapped rooms do.
        # ... every goal published this run EXCEPT the one just attempted. That
        # last one is not a revisit, it is unfinished business: fading it is how
        # a rover that timed out three metres short of a frontier turns round
        # and crosses the flat for the next-best thing, and then does it again
        # (measured in the harness: 25 m of travel over four goals for 2 m2).
        older = state.visited[:-1]
        if older:
            nearest = min(math.hypot(gxw - vx, gyw - vy) for vx, vy in older)
            if nearest < tuning.revisit_radius_m:
                score *= (nearest / tuning.revisit_radius_m) ** _REVISIT_FADE_POWER

        if score > best_score:
            best, best_score, best_cost = cluster, score, cost

    if best is None:
        # Clusters exist but none is reachable from where the body can stand.
        state.last_directive = ""
        return None

    state.visited.append(best.goal_xy)
    state.targets_issued += 1
    state.wait_streak = 0
    state.back_off_issued = False        # a new pocket later gets its own back-off
    state.last_directive = DIRECTIVE_FRONTIER
    return _target_pose(best.goal_xy[0], best.goal_xy[1], frame_id=frame_id, ts=ts,
                        directive=DIRECTIVE_FRONTIER, score=best_score,
                        path_cost_m=best_cost, info_cells=best.size, info_gain=best.gain,
                        centroid_xy=best.centroid_xy, look_at_xy=best.look_at_xy,
                        on_frontier=best.on_frontier, probe=best.probe,
                        in_sight=best.in_sight,
                        n_clusters=len(clusters), n_excluded=n_blocked,
                        n_already_seen_from=n_seen_from, n_on_the_map=on_the_map)


# ===========================================================================
# The module: a loop around next_target, and nothing else.
# ===========================================================================

if HAVE_DIMOS:  # pragma: no cover - needs the dimOS stack
    from typing import Any as _Any

    from dimos.core.core import rpc
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
        WavefrontConfig, WavefrontFrontierExplorer,
    )
    from dimos.utils.logging_config import setup_logger
    from dimos.core.stream import Out
    from dimos_lcm.std_msgs import Bool

    logger = setup_logger()

    WAIT_POLL_S = 1.0        # longest single sleep on a WAIT: the map keeps growing
    BACK_OFF_SETTLE_S = 3.0  # the planner's back-up is ~2 s at 0.10 m/s over 0.20 m
    # A goal that times out having closed less than this much of the gap is not
    # "still on its way", it is stuck against something, and re-publishing it
    # unchanged is how a rover spends 19 goals and 30 m in one corner
    # (measured in tools/explore_sim.py). It becomes a §7.1 exclusion, which
    # holds until the map around it changes or the rover finds a new viewpoint
    # and can only ever produce a WAIT - never an end of run. A goal that IS closing the gap is left alone: at the
    # exploration speed cap a 7 m goal simply takes longer than goal_timeout.
    GOAL_PROGRESS_M = 0.25   # the follower's own arrival tolerance

    class Explorer2Config(WavefrontConfig):
        """WavefrontConfig so the blueprint stays drop-in. The fields v2 ignores
        (info_gain_threshold, num_no_gain_attempts, lookahead_distance,
        max_explored_distance, safe_distance) are the self-stop timers and the
        weighted-sum terms this rewrite exists to remove: do not pass them."""

        lethal_clearance_m: float = DEFAULT_TUNING.lethal_clearance_m
        pivot_clearance_m: float = DEFAULT_TUNING.pivot_clearance_m
        unknown_cost_factor: float = DEFAULT_TUNING.unknown_cost_factor
        standoff_max_m: float = DEFAULT_TUNING.standoff_max_m
        info_radius_m: float = DEFAULT_TUNING.info_radius_m
        probe_penalty: float = DEFAULT_TUNING.probe_penalty
        footprint_clear_m: float = DEFAULT_TUNING.footprint_clear_m
        observed_radius_m: float = DEFAULT_TUNING.observed_radius_m
        forward_bonus: float = DEFAULT_TUNING.forward_bonus
        revisit_radius_m: float = DEFAULT_TUNING.revisit_radius_m
        failed_goal_radius_m: float = DEFAULT_TUNING.failed_goal_radius_m
        failed_goal_moved_m: float = DEFAULT_TUNING.failed_goal_moved_m
        cornered_area_m2: float = DEFAULT_TUNING.cornered_area_m2
        back_off_m: float = DEFAULT_TUNING.back_off_m

    class Explorer2(WavefrontFrontierExplorer):
        """Drop-in for the wavefront explorer: same In/Out names, same LCM
        topics, same rpc and skill surface (subclassing is what guarantees it).
        Everything that decides anything lives in `next_target`; what is left
        here is publish, wait, remember.

        Extra Out: `bump` - the same reflex the human triggered by hand three
        times on 26/08 to free a cornered rover. RecoveringPlanner already
        listens to it (stop, escape 0.20 m in reverse, abandon the goal); the
        contact switches publish on the same stream, and a second producer on
        one stream is how this stack is wired.
        """

        config: Explorer2Config

        bump: Out[Bool]

        def __init__(self, **kwargs: _Any) -> None:
            super().__init__(**kwargs)
            self._state = ExploreState()
            self._last_goal: tuple[float, float] | None = None
            self._last_goal_ok: bool | None = None
            self._tuning = Tuning(
                lethal_clearance_m=self.config.lethal_clearance_m,
                pivot_clearance_m=self.config.pivot_clearance_m,
                unknown_cost_factor=self.config.unknown_cost_factor,
                min_frontier_perimeter_m=self.config.min_frontier_perimeter,
                info_radius_m=self.config.info_radius_m,
                probe_penalty=self.config.probe_penalty,
                standoff_max_m=self.config.standoff_max_m,
                footprint_clear_m=self.config.footprint_clear_m,
                observed_radius_m=self.config.observed_radius_m,
                forward_bonus=self.config.forward_bonus,
                revisit_radius_m=self.config.revisit_radius_m,
                failed_goal_radius_m=self.config.failed_goal_radius_m,
                failed_goal_moved_m=self.config.failed_goal_moved_m,
                cornered_area_m2=self.config.cornered_area_m2,
                back_off_m=self.config.back_off_m,
            )
            _warn_if_planner_disagrees(self._tuning)

        # A goal_reached of False means "the planner gave up", and the stock
        # module ignores it and sleeps its whole goal_timeout for nothing
        # (measured 23/08: 15 s per failed goal).
        def _on_goal_reached(self, msg: Bool) -> None:
            self._last_goal_ok = bool(getattr(msg, "data", False))
            self.goal_reached_event.set()

        def _run_exploration_loop(self) -> None:  # type: ignore[override]
            self._state = ExploreState()
            state = self._state
            while self.exploration_active and not self.stop_event.is_set():
                if self.latest_costmap is None or self.latest_odometry is None:
                    self.stop_event.wait(0.5)
                    continue

                costmap: OccupancyGrid = self.latest_costmap
                t0 = time.perf_counter()
                target = next_target(costmap, self.latest_odometry, state, tuning=self._tuning)
                elapsed_ms = (time.perf_counter() - t0) * 1000.0

                if target is None:
                    logger.info(f"exploration complete: no reachable frontier left "
                                f"({state.targets_issued} targets, {elapsed_ms:.0f} ms)")
                    self.exploration_active = False
                    break

                directive = getattr(target, "directive", DIRECTIVE_FRONTIER)

                if directive == DIRECTIVE_WAIT:
                    reopened = getattr(target, "reopened_xy", None)
                    if reopened:
                        logger.warning(f"starvation escape: the world stayed silent for "
                                       f"{WAIT_REOPEN_POLLS} asks - reopening the oldest failed "
                                       f"goal at ({reopened[0]:.2f}, {reopened[1]:.2f})")
                    else:
                        logger.info(f"{getattr(target, 'n_excluded', 0)} of "
                                    f"{getattr(target, 'n_clusters', 0)} clusters are on a recently "
                                    "failed goal: waiting for the map or the viewpoint "
                                    "to change, not stopping")
                    self.stop_event.wait(WAIT_POLL_S)
                    continue

                if directive == DIRECTIVE_BACK_OFF:
                    logger.warning(
                        f"born cornered: {getattr(target, 'reachable_free_m2', 0.0):.2f} m2 of "
                        f"reachable floor and no frontier - one "
                        f"{getattr(target, 'back_off_m', 0.0):.2f} m back-off via the bump reflex")
                    if self.bump.transport is not None:
                        self.bump.publish(Bool(data=True))
                    self.stop_event.wait(BACK_OFF_SETTLE_S)
                    continue

                goal = PoseStamped()
                goal.position.x = target.position.x
                goal.position.y = target.position.y
                goal.position.z = 0.0
                goal.orientation.w = 1.0
                goal.frame_id = "world"
                goal.ts = costmap.ts
                self._last_goal = (float(goal.position.x), float(goal.position.y))
                self._last_goal_ok = None
                gap_at_issue = self._gap_to(self._last_goal)
                self.goal_reached_event.clear()
                self.goal_request.publish(goal)
                logger.info(
                    f"goal {state.targets_issued}: ({goal.position.x:.2f}, {goal.position.y:.2f}) "
                    f"{getattr(target, 'path_cost_m', float('nan')):.1f} m away, "
                    f"{getattr(target, 'info_cells', 0)} frontier cells, "
                    f"{getattr(target, 'n_clusters', 0)} clusters, chosen in {elapsed_ms:.0f} ms")

                arrived = self.goal_reached_event.wait(timeout=self.config.goal_timeout)
                if arrived and self._last_goal_ok is False:
                    # The planner gave up (no path, or stuck -> goal abandoned).
                    # An exclusion, never a reason to stop (§7.1); it reopens
                    # when the map around it changes or the rover stands a
                    # viewpoint away from here.
                    self._note_failed(state)
                    logger.info("planner gave up on that goal: excluded until "
                                "the map or the viewpoint changes")
                elif not arrived:
                    # Timed out mid-drive. Whether that is a failure depends on
                    # one measurement: did the gap close (see GOAL_PROGRESS_M).
                    closed = gap_at_issue - self._gap_to(self._last_goal)
                    if closed < GOAL_PROGRESS_M:
                        self._note_failed(state)
                        logger.info(f"goal timeout after {self.config.goal_timeout:.0f} s having "
                                    f"closed {closed:.2f} m of {gap_at_issue:.2f} m: excluded until "
                                    "the map or the viewpoint changes")
                    else:
                        logger.info(f"goal timeout after {self.config.goal_timeout:.0f} s, "
                                    f"{closed:.2f} m closer, re-deciding")

        def _note_failed(self, state: ExploreState) -> None:
            """Exclude the goal that just failed, with its reopening triggers
            (where the rover stood + the unknown signature around the goal)."""
            odom = self.latest_odometry
            robot = ((float(odom.position.x), float(odom.position.y))
                     if odom is not None else self._last_goal)
            state.note_failed(*self._last_goal, robot, self.latest_costmap,
                              self._tuning.failed_goal_radius_m)

        def _gap_to(self, goal: tuple[float, float]) -> float:
            """Metres from where the rover is now to `goal`, or inf if unknown."""
            odom = self.latest_odometry
            if odom is None:
                return float("inf")
            return math.hypot(goal[0] - float(odom.position.x), goal[1] - float(odom.position.y))

        @rpc
        def explore_state(self) -> dict[str, _Any]:
            """The memory, for `dimos shell` and for a post-run replay."""
            return {"visited": list(self._state.visited),
                    "failed": list(self._state.failed),
                    "heading": self._state.heading,
                    "targets_issued": self._state.targets_issued,
                    "last_directive": self._state.last_directive}

    def _warn_if_planner_disagrees(tuning: Tuning) -> None:
        """The explorer picks goals the planner has to accept. If the two
        clearances ever drift apart, say so at start-up rather than discover it
        as 'no path' in the field."""
        try:
            from vector_dimos.recovering_planner import LETHAL_CLEARANCE_M, PIVOT_CLEARANCE_M
        except Exception:  # noqa: BLE001 - a missing planner is not our failure
            return
        if abs(tuning.lethal_clearance_m - LETHAL_CLEARANCE_M) > 1e-9:
            logger.warning(f"explorer2 lethal clearance {tuning.lethal_clearance_m} m != planner "
                           f"{LETHAL_CLEARANCE_M} m: goals may be picked the planner refuses")
        if abs(tuning.pivot_clearance_m - PIVOT_CLEARANCE_M) > 1e-9:
            logger.warning(f"explorer2 pivot clearance {tuning.pivot_clearance_m} m != planner "
                           f"{PIVOT_CLEARANCE_M} m")


# --- the A/B switch, read by nav_blueprints --------------------------------

EXPLORER_V2_ENV = "EXPLORER_V2"


def explorer_v2_enabled() -> bool:
    """Which explorer the `explore` blueprint builds. Default: the NEW one.

        EXPLORER_V2=0 | false | no | off | old   -> fast_explorer.VectorExplorer
        anything else, or unset                  -> explorer2.Explorer2

    Read once, when the blueprint is built, so a run is one or the other for
    its whole life - that is what makes a real-world A/B comparable.
    """
    raw = os.environ.get(EXPLORER_V2_ENV)
    if raw is None:
        return True
    return raw.strip().lower() not in ("0", "false", "no", "off", "old")

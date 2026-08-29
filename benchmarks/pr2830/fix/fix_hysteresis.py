#!/usr/bin/env python3
"""HARNESS-SIDE policy wrappers for the frontier-exploration ping-pong.

NOTHING IN THIS FILE IS UPSTREAM CODE, AND NOTHING IN THIS FILE MODIFIES
UPSTREAM CODE. The two selector files in pr2830/ are read-only inputs and stay
byte-identical (md5 e77c328643c959a49077115e8a341f2c for the stock base,
1ccc0c69fe88a72e402565feca988d26 for the PR head). A policy here is installed on
one INSTANCE of the selector class, by `install()`, and it does exactly one
thing: it calls the selector's own `detect_frontiers`, which ends with the
selector's own `_rank_frontiers`, and then re-orders the list that came back.
The selector's own scores decide everything inside the policy; the policy never
computes a score of its own.

Because both bench configurations reach the goal through `detect_frontiers`
(config `shipped` inside `get_exploration_goal`, config `scoring` by calling it
directly), installing the wrapper there makes the policy apply to both arms and
both configurations with no other change, and the selector's own bookkeeping
(`_update_exploration_direction`, `mark_explored_goal`) follows the policy's
choice exactly as it would have followed its own.

Two policies, each naming its precedent.

  H, switch margin on the arm's own score
      Precedent: explore_lite (Jiri Horner, ROS; 1.0.0 in 2016, algorithm
      unchanged since 2.1.1 in December 2017, shipped in Kinetic through
      Noetic). Its whole frontier cost is

          cost = potential_scale * min_distance * resolution
                 - gain_scale * size * resolution      (frontier_search.cpp)

      sorted ascending, lowest cost wins. Distance from the robot is one of only
      two terms, and the launch files ship potential_scale = 3.0 against
      gain_scale = 1.0, so a remote frontier has to be a great deal larger
      before it is worth walking to. That is the established element being
      borrowed: distance from the robot as a FIRST-CLASS term rather than one
      of five weighted at 0.2, expressed here as a switch margin because the
      arm's scorer may not be modified.

      Stated plainly, because it matters for how this is cited: explore_lite
      itself has NO hysteresis. Its only "do not resend" rule is exact equality
      with the previous goal point, which is a de-duplication guard, not an
      incumbent preference; it re-runs the full argmin every planner_frequency
      tick and switches target freely. The margin k is our expression of the
      distance-dominant cost, not a mechanism copied from that code.

  B, finish-the-branch
      Precedent: room / segment based coverage, Bormann et al. (Fraunhofer IPA,
      ipa_coverage_planning). ipa_room_segmentation cuts the floor plan into
      rooms, ipa_building_navigation orders them as a travelling-salesman tour
      over room centres with A* edge weights, and ipa_room_exploration plans a
      path that covers ONE room; the robot finishes the room it is in before the
      sequence moves it to the next. Room Segmentation: Survey, Implementation,
      and Analysis, Bormann, Jordan, Li, Hampp, Hagele, ICRA 2016; Indoor
      Coverage Path Planning: Survey, Implementation, Analysis, Bormann, Jordan,
      Hampp, Hagele, ICRA 2018. Here the "room" is the ball of geodesic radius R
      around the robot: while any candidate lies inside it, the candidates
      outside it are not eligible. The segmentation is a radius and not a room
      detector, and that difference is named in the report.

B is H with k = infinity, so both are the same code with one parameter.

Spec strings, as passed to bench_2830.py --arms:

    stock+H6k2        H, vicinity radius 6 m, margin k = 2
    pr2830+H6k1.5     the same on the PR arm
    stock+B6          B, radius 6 m   (== H with k = infinity)
    stock+B9e         B, radius 9 m, straight-line vicinity instead of geodesic

The default vicinity metric is GEODESIC: the A* path length from the robot to
the candidate over the costmap the selector was handed, computed with the same
`min_cost_astar` and the same two alignment settings PR #2830 uses
(cost_threshold = the selector's occupancy_threshold, unknown_penalty = 0.95).
Suffix 'e' on the spec switches to straight-line, which is what a 2017-era
implementation would have had; it is offered so the cost of the geodesic call
can be paid only when it buys something.
"""

from __future__ import annotations

import math
import re

import numpy as np

# The 8 neighbours and their step length, as explore_sim.plan uses them.
_SQRT2 = math.sqrt(2.0)
_NEIGHBOURS = ((-1, -1, _SQRT2), (-1, 0, 1.0), (-1, 1, _SQRT2),
               (0, -1, 1.0), (0, 1, 1.0),
               (1, -1, _SQRT2), (1, 0, 1.0), (1, 1, _SQRT2))
UNKNOWN = -1

# The planner's thresholds, NOT the explorer's: this is
# explore_sim.PLANNER_COST_THRESHOLD / PLANNER_UNKNOWN_PENALTY, i.e. what
# RecoveringGlobalPlanner passes to min_cost_astar and therefore the rule the
# simulated rover's own routes obey. Blocking at 99 instead (which is what the
# frontier A* of PR #2830 does) walls off every pinch the Voronoi gradient
# touches and makes almost everything look unreachable.
PLANNER_COST_THRESHOLD = 100
PLANNER_UNKNOWN_PENALTY = 0.8


def geodesic_field(grid: np.ndarray, res: float, ox: float, oy: float,
                   robot_xy, targets_xy,
                   cost_threshold: int = PLANNER_COST_THRESHOLD,
                   unknown_penalty: float = PLANNER_UNKNOWN_PENALTY):
    """Route length in metres from the robot to each target, one wave.

    This is explore_sim.plan's cost rule (a cell at or above `cost_threshold` is
    blocked, an unknown cell costs threshold * penalty, a free cell costs 0, the
    tie-break is path length), solved ONCE from the robot for the whole grid and
    then read off at each target, instead of once per target. That is the shape
    explore_lite uses: navfn spreads one potential field from the robot per
    cycle and every frontier's cost is read out of it.

    Returns a list of metres, inf where no route exists.
    """
    from scipy.sparse import csr_matrix
    from scipy.sparse.csgraph import dijkstra

    h, w = grid.shape
    cost = grid.astype(np.float64)
    blocked = grid >= cost_threshold
    unknown_cost = cost_threshold * unknown_penalty
    if unknown_cost >= cost_threshold:
        blocked |= grid == UNKNOWN
    cell = np.where(grid == UNKNOWN, unknown_cost, np.maximum(cost, 0.0))
    passable = ~blocked

    def to_cell(x, y):
        gx = int(np.clip(math.floor((x - ox) / res), 0, w - 1))
        gy = int(np.clip(math.floor((y - oy) / res), 0, h - 1))
        return gy, gx

    start = to_cell(robot_xy[0], robot_xy[1])
    passable[start] = True
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
    dist, pred = dijkstra(graph, directed=True, indices=int(index[start]),
                          return_predecessors=True)

    out = []
    for tx, ty in targets_xy:
        gy, gx = to_cell(tx, ty)
        node = int(index[gy, gx])
        if node < 0 or not math.isfinite(dist[node]):
            out.append(float("inf"))
            continue
        # walk the predecessors back and sum the real step lengths
        length_m, cur, guard = 0.0, node, 0
        while cur != int(index[start]):
            nxt = int(pred[cur])
            if nxt < 0:
                length_m = float("inf")
                break
            length_m += math.hypot(int(ys[cur]) - int(ys[nxt]),
                                   int(xs[cur]) - int(xs[nxt])) * res
            cur = nxt
            guard += 1
            if guard > n:
                length_m = float("inf")
                break
        out.append(length_m)
    return out


class LocalFirst:
    """Prefer the best candidate near the robot; leave only for a big enough win.

    radius_m  the vicinity, in metres, measured by `metric`
    k         the switching margin. The best candidate outside the vicinity is
              taken only if its score is strictly greater than k times the score
              of the best candidate inside it. k = inf is candidate B.
    metric    'geo' (A* over the costmap) or 'euc' (straight line)
    """

    def __init__(self, radius_m: float, k: float, metric: str = "geo",
                 occupancy_threshold: int = 99, unknown_penalty: float = 0.95):
        self.radius_m = float(radius_m)
        self.k = float(k)
        self.metric = metric
        self.occ = occupancy_threshold
        self.unk = unknown_penalty
        self.name = (f"{'B' if math.isinf(self.k) else 'H'}"
                     f"{self.radius_m:g}" + ("" if math.isinf(self.k) else f"k{self.k:g}")
                     + ("" if metric == "geo" else "e"))
        # counters, reported with the run
        self.n_decisions = 0
        self.n_local_available = 0
        self.n_overridden = 0

    # -- vicinity -----------------------------------------------------------
    def _distances(self, pose, cands, costmap):
        if self.metric == "euc":
            return [math.hypot(f.x - pose.x, f.y - pose.y) for f in cands]
        return geodesic_field(
            np.asarray(costmap.grid), float(costmap.resolution),
            float(costmap.origin.position.x), float(costmap.origin.position.y),
            (float(pose.x), float(pose.y)),
            [(float(f.x), float(f.y)) for f in cands])

    # -- the policy ---------------------------------------------------------
    def reorder(self, pose, costmap, ranked, tap):
        """`ranked` is the arm's own output, already best-first. `tap` maps a
        rounded (x, y) to the score the arm's own scorer gave it."""
        self.n_decisions += 1
        if len(ranked) < 2:
            return ranked

        def score(f):
            return tap.get((round(float(f.x), 4), round(float(f.y), 4)), float("-inf"))

        d = self._distances(pose, ranked, costmap)
        near = [i for i, v in enumerate(d) if v <= self.radius_m]
        if not near:
            return ranked                       # nothing in the vicinity, unchanged
        self.n_local_available += 1
        if 0 in near:
            return ranked                       # the arm already chose a near one
        best_local = max(near, key=lambda i: score(ranked[i]))
        s_local, s_remote = score(ranked[best_local]), score(ranked[0])
        if not (math.isfinite(s_local) and math.isfinite(s_remote)):
            return ranked                       # score not recovered, do nothing
        if math.isfinite(self.k) and s_remote > self.k * s_local:
            return ranked                       # the remote candidate earns the walk
        self.n_overridden += 1
        return [ranked[best_local]] + [f for i, f in enumerate(ranked) if i != best_local]


class Persist:
    """Candidate P: hold a frontier cluster for one extra decision cycle.

    Precedent: the cross-cycle frontier memory in explore_lite's blacklist. That
    code keeps frontier POSITIONS from earlier cycles and matches a freshly
    detected frontier against them within a 5-cell box tolerance
    (explore.cpp, `goalOnBlacklist`), so a frontier is an object that persists
    between detections rather than being re-derived from scratch each tick. The
    same memory, used in the other direction: a cluster the previous decision
    produced and this one did not is held for one more cycle.

    Honest limit of the citation: explore_lite remembers frontiers in order to
    REFUSE them, never to keep them alive. The structure is the precedent; the
    direction is ours, and the report says so.

    A cluster the PREVIOUS decision produced and this one did not is re-injected
    once, with the size it had, scored by the ARM'S OWN scorer, and then the
    whole list is re-sorted by that score. A re-injected cluster is dropped if
    there is no unknown cell left within `alive_m` of it: that means the
    frontier is genuinely gone, not blinking.

    This is a stabiliser for cause B (a near frontier temporarily missing). It
    does nothing about cause A.
    """

    def __init__(self, hold: int = 1, same_spot_m: float = 1.0, alive_m: float = 0.6):
        self.hold = int(hold)
        self.same_spot_m = float(same_spot_m)
        self.alive_m = float(alive_m)
        self.name = f"P{hold}"
        self.n_decisions = 0
        self.n_reinjected = 0
        self._memory: list = []          # [(x, y, size, cycles_left)]

    def _alive(self, x, y, costmap):
        grid = np.asarray(costmap.grid)
        res = float(costmap.resolution)
        ox = float(costmap.origin.position.x)
        oy = float(costmap.origin.position.y)
        h, w = grid.shape
        gx = int(np.clip(math.floor((x - ox) / res), 0, w - 1))
        gy = int(np.clip(math.floor((y - oy) / res), 0, h - 1))
        r = max(1, int(round(self.alive_m / res)))
        sub = grid[max(0, gy - r):gy + r + 1, max(0, gx - r):gx + r + 1]
        return bool((sub == UNKNOWN).any()) and bool((sub == 0).any())

    def reorder(self, pose, costmap, ranked, tap, ex=None):
        self.n_decisions += 1
        cur = [(float(f.x), float(f.y)) for f in ranked]

        def seen(x, y):
            return any(math.hypot(x - a, y - b) <= self.same_spot_m for a, b in cur)

        extra = []
        for (mx, my, msize, left) in self._memory:
            if left <= 0 or seen(mx, my):
                continue
            if not self._alive(mx, my, costmap):
                continue
            extra.append((mx, my, msize))

        merged = list(ranked)
        if extra and ex is not None:
            V3 = _vector3()
            for mx, my, msize in extra:
                f = V3(mx, my, 0.0)
                s = ex._orig_score(f, msize, pose, costmap)
                if s == float("-inf"):
                    continue
                tap[(round(mx, 4), round(my, 4))] = float(s)
                merged.append(f)
                self.n_reinjected += 1
            merged.sort(key=lambda f: tap.get((round(float(f.x), 4), round(float(f.y), 4)),
                                              float("-inf")), reverse=True)

        # refresh the memory: everything currently detected, with its size
        sizes = getattr(ex, "_last_sizes", {}) if ex is not None else {}
        mem = []
        for f in ranked:
            k = (round(float(f.x), 4), round(float(f.y), 4))
            mem.append((float(f.x), float(f.y), sizes.get(k, 1), self.hold))
        for (mx, my, msize, left) in self._memory:
            if left - 1 > 0 and not seen(mx, my):
                mem.append((mx, my, msize, left - 1))
        self._memory = mem
        return merged


def _vector3():
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    return Vector3


_SPEC = re.compile(r"^(?P<kind>[HB])(?P<r>\d+(?:\.\d+)?)(?:k(?P<k>\d+(?:\.\d+)?))?(?P<e>e?)$")
_SPEC_P = re.compile(r"^P(?P<hold>\d+)$")


def make(spec: str):
    mp = _SPEC_P.match(spec)
    if mp:
        return Persist(hold=int(mp.group("hold")))
    m = _SPEC.match(spec)
    if not m:
        raise ValueError(f"policy spec {spec!r} not understood; e.g. H6k2, B9, H6k1.5e, P1")
    kind, r, k, e = m.group("kind"), float(m.group("r")), m.group("k"), m.group("e")
    if kind == "B":
        if k is not None:
            raise ValueError("candidate B takes no k (it IS k = infinity)")
        kk = float("inf")
    else:
        if k is None:
            raise ValueError("candidate H needs a margin, e.g. H6k2")
        kk = float(k)
    return LocalFirst(r, kk, metric="euc" if e else "geo")


def install(ex, policy) -> None:
    """Put `policy` on one selector instance. Upstream files are not touched."""
    orig_detect = ex.detect_frontiers
    orig_score = ex._compute_comprehensive_frontier_score
    tap: dict = {}
    sizes: dict = {}

    def tapped_score(frontier, frontier_size, robot_pose, costmap):
        v = orig_score(frontier, frontier_size, robot_pose, costmap)
        k = (round(float(frontier.x), 4), round(float(frontier.y), 4))
        tap[k] = float(v)
        sizes[k] = int(frontier_size)
        return v

    def wrapped_detect(robot_pose, costmap):
        tap.clear()
        sizes.clear()
        ranked = orig_detect(robot_pose, costmap)
        if not ranked:
            return ranked
        if isinstance(policy, Persist):
            return policy.reorder(robot_pose, costmap, ranked, tap, ex)
        return policy.reorder(robot_pose, costmap, ranked, tap)

    ex._compute_comprehensive_frontier_score = tapped_score
    ex.detect_frontiers = wrapped_detect
    ex._orig_score = orig_score
    ex._last_sizes = sizes
    ex._policy = policy

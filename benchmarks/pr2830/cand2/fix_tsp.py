#!/usr/bin/env python3
"""Candidate T: a LAZY TSP ordering over the selector's own frontier clusters.

NOTHING IN THIS FILE IS UPSTREAM CODE, AND NOTHING IN THIS FILE MODIFIES
UPSTREAM CODE. The two selector files in pr2830/ are read-only inputs and stay
byte-identical (md5 e77c328643c959a49077115e8a341f2c for the stock base,
1ccc0c69fe88a72e402565feca988d26 for the PR head). A policy here is installed on
one INSTANCE of the selector class by fix_hysteresis.install(), which calls the
selector's own detect_frontiers (ending in the selector's own _rank_frontiers)
and hands this policy the list that came back. This policy does not read the
selector's scores at all: it re-orders the selector's own candidate list by tour
position, and it invents no candidate.


WHY
---
The confrontation memo (../sim_2830_fleet/confrontation_externe.md, section
(iii), form C of "les trois seules formes que prend la memoire des anciens
buts", and (v).3) puts the number on it:

    P2 Explore (arXiv 2409.10878v4, Table II, single robot, metric = metres):
    going from greedy to a GLOBAL TSP ORDER buys about 31 % of travel on the
    large scenes; adding room-awareness on top of that buys only about 4.9 %
    more.

    "L'argent est dans 'cesser de decider glouton coup par coup', pas dans la
    segmentation."

The same form appears in every planner in the memo's table: FUEL solves an ATSP
over frontier viewpoints with LKH every cycle (fast_exploration_manager.cpp:345,
383); TARE orders uncovered subspaces with OR-Tools; FALCON commits to a
coverage path. dimOS has none of it: it re-runs an argmax over five weighted
terms at every decision and the order the frontiers are visited in is whatever
falls out.

Candidate T is that global order, done cheaply and LAZILY:

    1. one Dijkstra wave from the robot per REPLAN gives the route length to
       every current frontier cluster centroid, under the planner's own cost
       rule (the same wave fix_hysteresis used for its vicinity);
    2. a tour is built over those centroids: nearest-neighbour from the robot,
       then 2-opt to convergence (capped at TWO_OPT_PASSES sweeps);
    3. the policy COMMITS to that order. At every decision it moves the next
       un-consumed tour stop to the head of the selector's own list;
    4. the tour is re-planned only when one of three declared things happens,
       never per decision.

REPLAN TRIGGERS, all three declared before the bench:
    a. the next tour stop is no longer among the current candidates (matched
       within MATCH_M = 1.0 m): it was explored away, or it vanished;
    b. the frontier set has churned by more than `change_frac` (declared, 0.30
       in the arm that is benched): churn is the symmetric difference over the
       union of the centroid set at the last replan and the current one, under
       the same 1.0 m matching;
    c. the same stop has been put at the head MAX_ISSUES = 2 times in a row, so
       a stop the drive planner cannot reach cannot lock the tour forever. This
       one is a safety valve, not part of the precedent, and it is counted and
       reported separately.
An empty or exhausted tour also replans, which is the same thing as (a).

HONEST APPROXIMATION, stated because it is the weakest part of the candidate.
The brief allows ONE Dijkstra wave per replan, so only the ROBOT-to-centroid
edges are geodesic. The centroid-to-centroid edges of the tour are STRAIGHT
LINE. FUEL pays for the full pairwise cost matrix (A* between every pair of
viewpoints) and then calls LKH; we pay for one wave and nearest-neighbour plus
2-opt. On a floor where two centroids are 3 m apart across a wall this tour is
wrong, and the report says so rather than calling it a TSP solver.

WHAT IS NOT CLAIMED. This is not P2 Explore and not FUEL. There is no room
decomposition, no LKH, no viewpoint sampling, no coverage path. The borrowed
element is exactly one thing: a stable global ORDER over what is left, committed
to across decisions instead of re-argmaxed at every one.

Specs, as passed to bench_2830.py --arms:

    stock+T30         lazy tour, replan when the frontier set churns > 30 %
    pr2830+T30        the same on the PR arm
    stock+T100        replan only when the next stop disappears (churn never
                      exceeds 100 %), i.e. the laziest possible version
"""

from __future__ import annotations

import math
import re

import numpy as np

from fix_hysteresis import geodesic_field

MATCH_M = 1.0            # two centroids are "the same frontier" within this
MAX_ISSUES = 2           # safety valve: stops one stop locking the tour
TWO_OPT_PASSES = 20


def _euc(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _tour_cost(order, pts, geo):
    if not order:
        return 0.0
    c = geo[order[0]]
    for k in range(len(order) - 1):
        c += _euc(pts[order[k]], pts[order[k + 1]])
    return c


def _nearest_neighbour(pts, geo):
    """Open path from the robot: first edge geodesic, the rest straight line."""
    left = list(range(len(pts)))
    order = []
    cur = None
    while left:
        if cur is None:
            j = min(left, key=lambda i: geo[i])
        else:
            j = min(left, key=lambda i: _euc(pts[cur], pts[i]))
        order.append(j)
        left.remove(j)
        cur = j
    return order


def _two_opt(order, pts, geo, max_passes=TWO_OPT_PASSES):
    n = len(order)
    if n < 3:
        return order
    best = list(order)
    best_c = _tour_cost(best, pts, geo)
    for _ in range(max_passes):
        improved = False
        for i in range(0, n - 1):
            for j in range(i + 1, n):
                cand = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                c = _tour_cost(cand, pts, geo)
                if c < best_c - 1e-9:
                    best, best_c, improved = cand, c, True
        if not improved:
            break
    return best


class LazyTour:
    """Commit to a tour over the current frontier centroids; replan rarely."""

    def __init__(self, change_frac: float = 0.30, match_m: float = MATCH_M,
                 max_issues: int = MAX_ISSUES):
        self.change_frac = float(change_frac)
        self.match_m = float(match_m)
        self.max_issues = int(max_issues)
        self.name = f"T{int(round(self.change_frac * 100))}"
        # state
        self._tour: list = []            # remaining stops, (x, y)
        self._at_replan: list = []       # centroid set at the last replan
        self._issues = 0                 # times the head stop was put in front
        # counters, reported with the run
        self.n_decisions = 0
        self.n_replans = 0
        self.n_replan_gone = 0           # trigger (a)
        self.n_replan_churn = 0          # trigger (b)
        self.n_replan_stuck = 0          # trigger (c), the safety valve
        self.n_replan_empty = 0          # tour exhausted
        self.n_advanced = 0              # stops worked through, no replan
        self.n_committed = 0             # decisions where a tour stop was head
        self.n_head_changed = 0          # ... and it was not the arm's own top
        self.tour_lengths: list = []

    # -- matching -----------------------------------------------------------
    def _find(self, pt, pts):
        best, bd = -1, self.match_m
        for i, q in enumerate(pts):
            d = _euc(pt, q)
            if d <= bd:
                best, bd = i, d
        return best

    def _churn(self, cur):
        """Symmetric difference over union of two centroid sets, greedy match."""
        old = list(self._at_replan)
        if not old and not cur:
            return 0.0
        used = set()
        matched = 0
        for p in old:
            best, bd = -1, self.match_m
            for i, q in enumerate(cur):
                if i in used:
                    continue
                d = _euc(p, q)
                if d <= bd:
                    best, bd = i, d
            if best >= 0:
                used.add(best)
                matched += 1
        union = len(old) + len(cur) - matched
        return (union - matched) / union if union else 0.0

    # -- tour ---------------------------------------------------------------
    def _replan(self, pose, costmap, ranked, why):
        pts = [(float(f.x), float(f.y)) for f in ranked]
        geo = geodesic_field(
            np.asarray(costmap.grid), float(costmap.resolution),
            float(costmap.origin.position.x), float(costmap.origin.position.y),
            (float(pose.x), float(pose.y)), pts)
        keep = [i for i, v in enumerate(geo) if math.isfinite(v)]
        if not keep:
            self._tour, self._at_replan, self._issues = [], pts, 0
            return
        sub_pts = [pts[i] for i in keep]
        sub_geo = [geo[i] for i in keep]
        order = _two_opt(_nearest_neighbour(sub_pts, sub_geo), sub_pts, sub_geo)
        self._tour = [sub_pts[i] for i in order]
        self._at_replan = pts
        self._issues = 0
        self.n_replans += 1
        self.tour_lengths.append(len(self._tour))
        if why == "gone":
            self.n_replan_gone += 1
        elif why == "churn":
            self.n_replan_churn += 1
        elif why == "empty":
            self.n_replan_empty += 1
        else:
            self.n_replan_stuck += 1

    # -- the policy ---------------------------------------------------------
    def reorder(self, pose, costmap, ranked, tap):
        self.n_decisions += 1
        if len(ranked) < 2:
            return ranked
        pts = [(float(f.x), float(f.y)) for f in ranked]

        # ADVANCE, not replan: the head stop was issued as a goal on the previous
        # decision and is no longer a frontier, i.e. it was worked through. The
        # tour moves on to its next stop and nothing is re-optimised. This is the
        # whole point of the candidate: the order survives.
        if self._tour and self._issues > 0 and self._find(self._tour[0], pts) < 0:
            self._tour.pop(0)
            self._issues = 0
            self.n_advanced += 1

        why = None
        if not self._tour:
            why = "empty"                       # tour exhausted
        elif self._find(self._tour[0], pts) < 0:
            why = "gone"                        # a stop we never issued vanished
        elif self._issues >= self.max_issues:
            why = "stuck"
        elif self._churn(pts) > self.change_frac:
            why = "churn"
        if why is not None:
            self._replan(pose, costmap, ranked, why)

        if not self._tour:
            return ranked
        j = self._find(self._tour[0], pts)
        if j < 0:
            return ranked
        self._issues += 1
        self.n_committed += 1
        if j != 0:
            self.n_head_changed += 1
        return [ranked[j]] + [f for i, f in enumerate(ranked) if i != j]


_SPEC = re.compile(r"^T(?P<pct>\d+)$")


def make(spec: str):
    m = _SPEC.match(spec)
    if not m:
        raise ValueError(f"policy spec {spec!r} is not a candidate T spec; "
                         f"e.g. T30 (replan above 30 % churn), T100")
    return LazyTour(change_frac=int(m.group("pct")) / 100.0)

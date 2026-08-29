#!/usr/bin/env python3
"""Candidate M: a SIGNED direction penalty on the selector's own candidates.

NOTHING IN THIS FILE IS UPSTREAM CODE, AND NOTHING IN THIS FILE MODIFIES
UPSTREAM CODE. The two selector files in pr2830/ are read-only inputs and stay
byte-identical (md5 e77c328643c959a49077115e8a341f2c for the stock base,
1ccc0c69fe88a72e402565feca988d26 for the PR head). A policy here is installed on
one INSTANCE of the selector class by fix_hysteresis.install(), which calls the
selector's own detect_frontiers (ending in the selector's own _rank_frontiers)
and hands this policy the list that came back plus a tap of the selector's own
scores. The policy never computes a score of its own: it multiplies the
selector's score by two penalties and re-sorts.


WHY
---
The confrontation memo (../sim_2830_fleet/confrontation_externe.md, sections
(iii) and (v).4) makes the case in one line: dimOS has a direction term and it is
inert.

    momentum_score = max(0.0, dot_product)          # wavefront selector, l. 467
    ... + 0.05 * momentum_score

Truncated at zero, a U-turn scores exactly the same as a sideways move: zero.
So a reversal is FREE. The memo's table "les trois seules formes que prend la
memoire des anciens buts" lists the established alternative under form B, a
POSITIVE engagement term on the current direction, and every stack that tunes it
tunes it heavy AND signed:

    GBPlanner   gain * exp(-0.07 * path_length) * exp(-0.3 * direction_deviation)
                i.e. the direction coefficient is 0.3 / 0.07 = 4.3x the distance
                coefficient  (Dang et al. IROS 2019; Kulkarni et al. ICRA 2022,
                arXiv 2111.06482 Sec. IV-A; gbplanner/src/rrg.cpp:657-676 and
                config/smb/gbplanner_config.yaml, read in the memo)
    FUEL        an arccos term on the first tour edge, w_dir = 1.5, up to about
                4.7 s equivalent for a U-turn (RA-L 2021, arXiv 2010.11561)
    Umari       hysteresis_gain = 2.0 inside 3 m (rrt_exploration/assigner.py)

Candidate M is the GBPlanner form, verbatim in shape, applied on top of whatever
score the arm under test produced:

    adjusted = base_score
               * exp(-LAMBDA_DIST * route_m)
               * exp(-ratio * LAMBDA_DIST * deviation)

    LAMBDA_DIST = 0.07 per metre                     GBPlanner path_length_penalty
    deviation   = (1 - cos(theta)) / 2   in [0, 1]   0 straight on, 0.5 across,
                                                     1.0 a full U-turn
    ratio       the direction-to-distance weight ratio, SWEPT in {1, 2, 4.3};
                4.3 is GBPlanner's own 0.3 / 0.07.

`deviation` is the signed reading the memo asks for: it is a strictly decreasing
function of the dot product over the whole range [-1, +1], where dimOS's
max(0, dot) is flat over the whole backwards half. At ratio = 4.3 a U-turn costs
exp(-0.301) = 0.740 of the score, the same as walking 4.3 m further. A U-turn
actually costs.

HEADING. GBPlanner's recipe, as the memo states it: an exponential moving
average of the REAL DISPLACEMENT (position direction, not body heading),
alpha = 0.3, updated only after 0.75 m of effective movement. The harness gives
the policy the robot's pose at every decision, so the trajectory is read from
those poses: the anchor is the pose at the last heading update, and the heading
is refreshed the first time the robot is more than 0.75 m from it. Before the
first update there is no heading and the direction term is 1.0 (no penalty), so
the very first decisions of a run are ranked by score and distance alone.

DISTANCE. Route length from the robot under the PLANNER's own cost rule, one
Dijkstra wave per decision, read off at every candidate. Exactly the metric
fix_hysteresis used for its vicinity, reused unchanged so the two candidates are
measured the same way, and the same shape explore_lite uses (one navfn potential
field per cycle, every frontier's cost read out of it).

A NAMED CONTROL. Spec `M0` sets ratio = 0: the distance penalty alone, no
direction term. It is not a candidate and does not compete for the best ratio.
It exists so the report can say whether anything M does is attributable to the
direction term at all, since the GBPlanner form necessarily carries both.

WHAT IS NOT CLAIMED. This is not GBPlanner. GBPlanner penalises a gain computed
on an RRG with a hard kBackward path rejection on top; here the two exponentials
multiply somebody else's frontier score and there is no hard rejection. The
borrowed element is the shape and the coefficients, and that is all.

Specs, as passed to bench_2830.py --arms:

    stock+M4.3        signed direction penalty at GBPlanner's own ratio
    pr2830+M2         the same on the PR arm, ratio 2
    stock+M4.3e       straight-line distance instead of the geodesic wave
    stock+M0          the distance-only control
"""

from __future__ import annotations

import math
import re

import numpy as np

from fix_hysteresis import geodesic_field

# GBPlanner's path_length_penalty, per metre. Fixed for every arm of the sweep;
# only the direction-to-distance ratio moves.
LAMBDA_DIST = 0.07

# GBPlanner's heading recipe.
EMA_ALPHA = 0.3
EMA_MOVE_M = 0.75


class SignedMomentum:
    """Re-rank by base_score * exp(-lam_d * route) * exp(-ratio * lam_d * dev)."""

    def __init__(self, ratio: float, metric: str = "geo",
                 lambda_dist: float = LAMBDA_DIST,
                 alpha: float = EMA_ALPHA, move_m: float = EMA_MOVE_M):
        self.ratio = float(ratio)
        self.metric = metric
        self.lam_d = float(lambda_dist)
        self.lam_dir = float(ratio) * float(lambda_dist)
        self.alpha = float(alpha)
        self.move_m = float(move_m)
        self.name = f"M{self.ratio:g}" + ("" if metric == "geo" else "e")
        # state
        self._heading = None            # unit vector, or None until 0.75 m moved
        self._anchor = None             # pose at the last heading update
        # counters, reported with the run
        self.n_decisions = 0
        self.n_with_heading = 0
        self.n_head_changed = 0         # our top != the arm's top
        self.n_arm_top_backwards = 0    # the arm's own top had dev > 0.5

    # -- heading ------------------------------------------------------------
    def _update_heading(self, pose):
        p = (float(pose.x), float(pose.y))
        if self._anchor is None:
            self._anchor = p
            return
        dx, dy = p[0] - self._anchor[0], p[1] - self._anchor[1]
        step = math.hypot(dx, dy)
        if step < self.move_m:
            return
        u = (dx / step, dy / step)
        if self._heading is None:
            self._heading = u
        else:
            hx = (1.0 - self.alpha) * self._heading[0] + self.alpha * u[0]
            hy = (1.0 - self.alpha) * self._heading[1] + self.alpha * u[1]
            n = math.hypot(hx, hy)
            self._heading = (hx / n, hy / n) if n > 1e-9 else u
        self._anchor = p

    def _deviation(self, pose, f):
        """(1 - cos) / 2 in [0, 1]; 0.0 when there is no heading yet."""
        if self._heading is None:
            return 0.0
        dx, dy = float(f.x) - float(pose.x), float(f.y) - float(pose.y)
        n = math.hypot(dx, dy)
        if n < 1e-6:
            return 0.0
        cos = (self._heading[0] * dx + self._heading[1] * dy) / n
        cos = max(-1.0, min(1.0, cos))
        return 0.5 * (1.0 - cos)

    # -- distance -----------------------------------------------------------
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
        self.n_decisions += 1
        self._update_heading(pose)
        if self._heading is not None:
            self.n_with_heading += 1
        if len(ranked) < 2:
            return ranked

        def base(f):
            return tap.get((round(float(f.x), 4), round(float(f.y), 4)),
                           float("-inf"))

        scored = [i for i, f in enumerate(ranked)
                  if math.isfinite(base(f)) and base(f) > 0.0]
        if len(scored) < 2:
            return ranked                    # scores not recovered, do nothing

        d = self._distances(pose, ranked, costmap)
        if self._deviation(pose, ranked[0]) > 0.5:
            self.n_arm_top_backwards += 1

        good, rest = [], []
        for i, f in enumerate(ranked):
            if i not in scored or not math.isfinite(d[i]):
                rest.append(i)
                continue
            adj = (base(f)
                   * math.exp(-self.lam_d * max(0.0, d[i]))
                   * math.exp(-self.lam_dir * self._deviation(pose, f)))
            good.append((adj, i))
        if not good:
            return ranked
        good.sort(key=lambda t: (-t[0], t[1]))
        order = [i for _a, i in good] + rest
        if order[0] != 0:
            self.n_head_changed += 1
        return [ranked[i] for i in order]


_SPEC = re.compile(r"^M(?P<ratio>\d+(?:\.\d+)?)(?P<e>e?)$")


def make(spec: str):
    m = _SPEC.match(spec)
    if not m:
        raise ValueError(f"policy spec {spec!r} is not a candidate M spec; "
                         f"e.g. M1, M2, M4.3, M4.3e, M0 (control)")
    return SignedMomentum(float(m.group("ratio")),
                          metric="euc" if m.group("e") else "geo")

#!/usr/bin/env python3
"""The COMPOSITE candidate: three established pieces, no fourth mechanism.

NOTHING IN THIS FILE IS UPSTREAM CODE, AND NOTHING IN THIS FILE MODIFIES
UPSTREAM CODE. The two selector files in pr2830/ are read-only inputs and stay
byte-identical (md5 e77c328643c959a49077115e8a341f2c for the stock base,
1ccc0c69fe88a72e402565feca988d26 for the PR head). A policy here is installed
on ONE INSTANCE of the selector class by fix_hysteresis.install(), which calls
the selector's own detect_frontiers (ending in the selector's own
_rank_frontiers) and hands this policy the list that came back plus a tap of
the selector's own scores. The policy never computes a score of its own.

THE THREE PIECES, AND WHERE EACH ONE IS SOURCED
------------------------------------------------
Every piece was already established in this workspace before this file existed.
Nothing new is invented here; the only new thing is that they run together.

(1) A SIGNED DIRECTION TERM AT THE GBPLANNER RATIO 4.3.
    Source: ../sim_2830_fleet/confrontation_externe.md sections (iii) and
    (v).4 - dimOS has a direction term and it is inert, because
    `momentum_score = max(0.0, dot_product)` at 5 % weight makes a U-turn cost
    exactly what a sideways move costs, i.e. nothing. The established
    alternative is form B of the memo's table, and GBPlanner weights direction
    0.3 against 0.07 for path length, a ratio of 4.3 (Dang et al. IROS 2019;
    Kulkarni et al. ICRA 2022, arXiv 2111.06482 Sec. IV-A).
    Implementation: ../sim_2830_cand2/fix_momentum.py, IMPORTED here rather
    than copied, so the arm `stock+M4.3` and the direction half of the
    composite are literally the same code.
    It won its declared ratio sweep at 4.3 (../sim_2830_cand2/rapport_cand2.md
    section 3 and sweep_M_ratio_table.txt).

(2) THE DISTANCE PENALTY, AS IN THE M0 CONTROL.
    Source: the same GBPlanner form carries `exp(-0.07 * path_length)` beside
    the direction term, and cand2's own control M0 - the same wrapper with the
    direction term switched off - reached the same real crossings and the same
    round trips as every declared ratio on `bigoffice`
    (rapport_cand2.md summary point 2 and section 3). On single-room floors the
    distance penalty is what did the work. It is kept for that reason and not
    because it is decoration on piece (1).
    Pieces (1) and (2) are ONE formula in GBPlanner and they are one formula
    here: SignedMomentum(ratio=4.3) is exactly

        adjusted = base_score * exp(-0.07 * route_m) * exp(-4.3 * 0.07 * dev)

    so the composite arm and the `stock+M4.3` arm differ ONLY by piece (3).
    That is deliberate: it is what makes "what the composite adds over M alone"
    a clean question.

(3) A REACHABILITY FILTER ON THE BODY.
    A candidate frontier whose goal cell the BODY cannot reach is dropped
    before ranking. Precedent, both external:
      - explore_lite (ROS `explore_lite`, frontier_search/explore.cpp) asks the
        navigation stack for a plan to each frontier and discards the ones with
        no valid plan; a frontier that cannot be planned to is not a candidate.
      - nav2's feasibility checking does the same thing at the planner
        boundary: a goal that fails `isPathValid` / has no plan is refused
        rather than pursued.
    Neither scorer under test models the body at all: cand2 caveat 13, "the
    costmap they are handed is inflated 0.25 m while the rover's lethal radius
    is 0.30 m, so both keep aiming at frontiers in pinches the body does not
    fit through". This piece is the missing feasibility check and nothing else.

    HOW IT IS COMPUTED, AND WHAT IT IS ALLOWED TO SEE. One Dijkstra wave from
    the robot over the BODY-INFLATED costmap - explore_sim's own
    `clearance_cost_map`, which blocks every cell closer than
    LETHAL_CLEARANCE_M to an obstacle and prices the pivot band, followed by
    `clear_footprint`, then the planner's own cost rule (blocked at 100,
    unknown priced at 80). That is verbatim what `Sim.plan_to` builds one
    millisecond later to decide whether the goal is drivable: a frontier this
    filter drops is a frontier the drive planner would have answered "blocked"
    on. It is THE SAME KIND of wave the other wrappers already compute
    (fix_hysteresis.geodesic_field, reused unchanged), on a different grid.

    On a real robot the exploration node reads this from the navigation
    costmap it can already subscribe to, which is what explore_lite and nav2 do.
    In this harness there is no ROS graph, so the bench binds the policy to the
    simulator through one explicit hook, `bind_nav(sim)`, and the policy reads
    exactly two things from it: the discovered occupancy grid and the robot's
    own pose. It reads no ground truth, no ceiling, no future.

    SAFETY VALVE, DECLARED AND COUNTED. If the filter would drop EVERY
    candidate the list is passed through unchanged, because a policy that
    returns nothing ends the run and that would be the filter deciding the
    experiment instead of measuring it. Counted as `all_dropped`.

WHAT IS NOT CLAIMED. This is not GBPlanner, not explore_lite and not nav2. The
borrowed elements are one scoring shape with its published coefficients and one
feasibility test with its published place in the loop. There is no fourth
mechanism: no room segmentation, no tour, no hysteresis, no new tuning
constant. The only number that is not inherited is which wave the filter runs
on, and that is fixed by the body profile, not chosen.

Specs, as passed to bench_2830.py --arms:

    stock+CMP         the composite on the stock selector
    pr2830+CMP        the composite on PR #2830's head
    stock+CMP0        the composite with the direction term off (M0 + filter),
                      the same control cand2 declared for M, available but not
                      part of the pre-declared grid
    stock+CMPr        piece (3) ALONE, the reachability filter with no
                      re-scoring, available for diagnosis
"""

from __future__ import annotations

import math
import re

import numpy as np

from fix_hysteresis import (PLANNER_COST_THRESHOLD, PLANNER_UNKNOWN_PENALTY,
                            geodesic_field)
from fix_momentum import LAMBDA_DIST, SignedMomentum

# GBPlanner's direction-to-distance ratio, the winner of cand2's declared sweep.
RATIO = 4.3


class Composite:
    """(1)+(2) the GBPlanner-shaped score, (3) a body reachability filter."""

    wants_nav = True

    def __init__(self, ratio: float = RATIO, rescore: bool = True):
        self.ratio = float(ratio)
        self.rescore = bool(rescore)
        self.mom = SignedMomentum(self.ratio) if rescore else None
        self.name = ("CMPr" if not rescore
                     else "CMP" if abs(self.ratio - RATIO) < 1e-9
                     else f"CMP{self.ratio:g}")
        self._sim = None
        # counters, reported with the run
        self.n_decisions = 0
        self.n_filtered_decisions = 0    # decisions where the filter dropped >= 1
        self.n_dropped = 0               # candidates dropped, total
        self.n_seen = 0                  # candidates examined, total
        self.n_all_dropped = 0           # safety valve fired
        self.n_top_dropped = 0           # the arm's OWN first choice was unreachable
        self.n_no_nav = 0                # bind_nav was never called (must stay 0)

    # -- the one hook into the harness --------------------------------------
    def bind_nav(self, sim) -> None:
        """The navigation view: the discovered grid and the robot pose.

        On a real robot this is the nav costmap the exploration node already
        subscribes to. Nothing else on `sim` is read; see `_body_reachable`.
        """
        self._sim = sim

    # -- piece (3) ----------------------------------------------------------
    def _body_reachable(self, cands):
        """Route length in metres to each candidate FOR THE BODY, inf if none.

        explore_sim.clearance_cost_map + clear_footprint, then the planner's own
        cost rule, exactly as Sim.plan_to builds it. Returns None when the hook
        was never bound, in which case the filter does nothing and says so.
        """
        sim = self._sim
        if sim is None:
            self.n_no_nav += 1
            return None
        import explore_sim as ES     # already in sys.modules, put there by the bench

        world = sim.world
        cost = ES.clearance_cost_map(sim.discovered, world.res)
        start = world.cell(sim.x, sim.y)
        ES.clear_footprint(cost, sim.discovered, start, world.res)
        return geodesic_field(
            np.asarray(cost), float(world.res), float(world.ox), float(world.oy),
            (float(sim.x), float(sim.y)),
            [(float(f.x), float(f.y)) for f in cands],
            cost_threshold=PLANNER_COST_THRESHOLD,
            unknown_penalty=PLANNER_UNKNOWN_PENALTY)

    # -- the policy ---------------------------------------------------------
    def reorder(self, pose, costmap, ranked, tap):
        self.n_decisions += 1
        if len(ranked) < 2:
            # Nothing to choose between. The filter is still pointless here and
            # the momentum policy still needs its heading update, so delegate.
            return self.mom.reorder(pose, costmap, ranked, tap) if self.mom else ranked

        d = self._body_reachable(ranked)
        keep = ranked
        if d is not None:
            self.n_seen += len(ranked)
            kept = [f for f, dist in zip(ranked, d) if math.isfinite(dist)]
            dropped = len(ranked) - len(kept)
            if dropped:
                self.n_dropped += dropped
                self.n_filtered_decisions += 1
                if not math.isfinite(d[0]):
                    self.n_top_dropped += 1
            if not kept:
                self.n_all_dropped += 1       # safety valve: pass everything through
            else:
                keep = kept

        if self.mom is None:
            return keep
        return self.mom.reorder(pose, costmap, keep, tap)

    # -- what the run report prints -----------------------------------------
    def counters(self) -> dict:
        c = {"decisions": self.n_decisions,
             "filtered_decisions": self.n_filtered_decisions,
             "cands_seen": self.n_seen,
             "cands_dropped": self.n_dropped,
             "top_choice_unreachable": self.n_top_dropped,
             "all_dropped_valve": self.n_all_dropped,
             "no_nav_hook": self.n_no_nav}
        if self.mom is not None:
            c.update({"mom_with_heading": self.mom.n_with_heading,
                      "mom_head_changed": self.mom.n_head_changed,
                      "mom_arm_top_backwards": self.mom.n_arm_top_backwards})
        return c


_SPEC = re.compile(r"^CMP(?P<what>r|0)?$")


def make(spec: str):
    m = _SPEC.match(spec)
    if not m:
        raise ValueError(f"policy spec {spec!r} is not a composite spec; "
                         f"e.g. CMP, CMP0 (direction off), CMPr (filter only)")
    what = m.group("what")
    if what == "r":
        return Composite(rescore=False)
    if what == "0":
        return Composite(ratio=0.0)
    return Composite()

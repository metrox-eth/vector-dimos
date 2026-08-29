#!/usr/bin/env python3
"""Offline A/B: dimOS stock wavefront frontier selection vs PR #2830, on our maps.

    bench_2830.py --out results.json

Everything about the WORLD and the ROBOT is tools/explore_sim.py, imported and
used unmodified: the saved ScoredGrid decoded as ground truth, the 360-ray 12 m
RPLIDAR C1 raycast that builds the discovered map one revolution per 0.25 m of
travel, the ported voronoi+clearance planner, the 0.15 m/s / 0.5 rad/s motion
model, the body-contact test, the bump reflex, and the harness stop conditions.
That is the house's own tool and it is the same for both arms.

The ONLY thing that differs between the two arms is which
WavefrontFrontierExplorer.get_exploration_goal is called:

    stock    dimensionalOS/dimos @ 6fcc4e2 (= the PR's base = the file installed
             in /home/openclaw/dimos-rig/.venv, md5-identical)
    pr2830   samuelokpor/dimos @ ff9d5ae (the PR's head)

Both are the authors' files, executed as written. See dimos_selector.py for the
(short) list of what had to be shimmed to run them without an LCM bus.

The loop around them is dimOS's own `_run_exploration_loop`, reproduced:
simple_inflate(costmap, 0.25) -> get_exploration_goal -> publish -> wait for the
goal or time out -> repeat; give up after 10 consecutive failures once 2 goals
have been published. Both arms run the identical loop.

MID-START RE-RUN (this copy of the file). dimOS maintainer lesh, on the shipped
bench: "starting from the middle of a space with several hallways, stock
explores part of one hallway, then walks all the way across to another hallway,
then back. From a dead-end start of an L-shaped hallway you can't see the bug.
Spawn the synthetic robot at the middle of the space, and use a low-range lidar
(3-5 m)." Two parameters change, and nothing else:

  1. --lidar-range   the simulated RPLIDAR range, ES.LIDAR_RANGE_M. It is a
                     module constant read at call time by explore_sim.scan, so
                     assigning it is enough; it is assigned inside every worker
                     as well as in the parent, so no worker is left at 12 m.
  2. the start set   midstarts.choose_mid_starts replaces choose_starts:
                     `centre` (unchanged) plus mid1..mid5 in the middle of the
                     passable floor. See midstarts.py.

Plus two new REPORTED numbers, which change no behaviour: the total inter-goal
jump distance of a run, and its count of cross-map swings. Both arms, both
configs, the same planner, the same paired starts, everything else identical.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)


def _import_explore_sim():
    for _p in ("/home/openclaw/vector-dimos/tools/explore_sim.py",
               "/root/vector-dimos/tools/explore_sim.py"):
        if os.path.exists(_p):
            path = _p
            break
    else:
        raise RuntimeError("explore_sim.py not found")
    spec = importlib.util.spec_from_file_location("explore_sim", path)
    assert spec and spec.loader
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()

sys.path.insert(0, HERE)
import dimos_selector as DS  # noqa: E402
import midstarts as MS  # noqa: E402
import go2_profile as PROF  # noqa: E402  (robot profile: rover or go2)

assert MS.ES is ES, "midstarts must share this process's explore_sim, or the lidar range splits"


# --- silence dimOS's per-frontier INFO logging ------------------------------
class _Quiet:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def debug(self, *a, **k): pass
    def error(self, *a, **k): pass


for _mod in (DS.sel_stock, DS.sel_pr):
    _mod.logger = _Quiet()

ARMS = {"stock": DS.sel_stock, "pr2830": DS.sel_pr}

import fix_hysteresis as FIX  # noqa: E402  (harness-side policy wrappers)
import fix_momentum as MOM  # noqa: E402   (candidate M, signed direction)
import fix_tsp as TSP  # noqa: E402        (candidate T, lazy tour)
import fix_composite as CMP  # noqa: E402  (the COMPOSITE: M4.3 + body reachability)

# Every policy module offers make(spec) and raises ValueError on a spec that is
# not its own. The spec grammars do not overlap (H/B/P, M, T, CMP), so the first
# module that accepts a spec owns it, and an unknown spec fails loudly.
POLICY_MODULES = (FIX, MOM, TSP, CMP)


class ChainedPolicy:
    """Two or more wrappers applied left to right; the last one has the last word.

    Only used for the optional combined arm of hypotheses_cand2.txt section 4,
    and only if its gate fires. Each policy re-orders the list the previous one
    produced; none of them sees anything but the selector's own candidates and
    the selector's own scores.
    """

    def __init__(self, parts):
        self.parts = list(parts)
        self.name = "+".join(p.name for p in self.parts)
        self.wants_nav = any(getattr(p, "wants_nav", False) for p in self.parts)

    def bind_nav(self, sim):
        for p in self.parts:
            if getattr(p, "wants_nav", False):
                p.bind_nav(sim)

    def reorder(self, pose, costmap, ranked, tap):
        for p in self.parts:
            ranked = p.reorder(pose, costmap, ranked, tap)
        return ranked


def _make_one(spec: str):
    for mod in POLICY_MODULES:
        try:
            return mod.make(spec)
        except ValueError:
            continue
    raise ValueError(f"policy spec {spec!r} is not understood by any of "
                     f"{[m.__name__ for m in POLICY_MODULES]}")


def make_policy(spec: str):
    parts = [s for s in spec.split("+") if s]
    if len(parts) == 1:
        return _make_one(parts[0])
    return ChainedPolicy([_make_one(s) for s in parts])


def parse_arm(spec: str):
    """'stock' -> (stock module, no policy); 'stock+M4.3' -> (module, policy).

    The part before '+' names one of the two UNMODIFIED upstream selectors. The
    part after it, if any, names a harness-side policy wrapper from
    fix_hysteresis.py, fix_momentum.py or fix_tsp.py, which re-ranks the
    candidate list the selector produced.
    """
    base, _, fix = spec.partition("+")
    return ARMS[base], (make_policy(fix) if fix else None)

# dimOS's own loop constants (wavefront_frontier_goal_selector._run_exploration_loop)
MAX_CONSECUTIVE_FAILURES = 10
RETRY_WAIT_S = 2.0
INFLATE_M = 0.25

# A harness stop, applied identically to both arms and always named in the
# report: a selector with no failed-goal memory (neither arm has one - the PR
# does not add one) can re-issue an unreachable goal forever. If this many
# decisions in a row move the rover less than IDLE_MOVE_M and gain less than
# IDLE_GAIN_M2 of map, the run is over. Never a strategy self-stop, which is the
# thing under test.
MAX_IDLE_DECISIONS = 6
IDLE_MOVE_M = 0.10
IDLE_GAIN_M2 = 0.05

# A WALL-CLOCK cap per run, off by default (0.0) so every earlier bench in this
# workspace is unaffected. explore_sim's own budgets are in SIM units (300 goals,
# 600 m, 6000 s of simulated time); on a floor three times the size of bigoffice
# a single run can spend hours of real time inside the frontier detector's
# full-grid Python BFS before it reaches any of them. The cap is checked at the
# top of the decision loop, applies identically to every arm and every
# configuration, and a run that hits it is reported with end_reason "wall_cap"
# and counted as capped in the tables. A capped run is data, not a failure to
# hide: it says the run was still going when the clock ran out.
RUN_CAP_S = 0.0

# The failed-goal filter used in config "scoring" only. It is OUR addition
# (vector_dimos.fast_explorer.FAILED_GOAL_RADIUS_M / _HOLD_S), applied to BOTH
# arms identically and outside their code: a goal the drive planner refused is
# suppressed for 60 s within 0.6 m. Without it a run ends the moment the first
# frontier sits in a pinch the 0.46 m body does not fit through - measured, on
# every start of the pilot: 2 to 4 goals reached, then 12 to 18 re-issues of the
# same unreachable centroid. That tail says nothing about the scoring.
FAILED_RADIUS_M = 0.6
FAILED_HOLD_S = 60.0

# ---------------------------------------------------------------------------
# DECISION TRACE (diagnosis only, off by default, changes no behaviour)
#
# When TRACE is on, every decision records the frontier candidates the arm's own
# detector produced, with the size and the score the arm's own scorer gave each
# of them, plus which of them the harness failed-goal filter suppressed, plus
# the one that was chosen. The score is captured by wrapping the bound method
# on the INSTANCE (the upstream file is never touched): the wrapper calls the
# original and records the value it returns, so the ranking is bit-identical
# with and without the trace.
#
# On a decision whose jump from the previous goal is longer than the swing
# threshold, the geodesic (A*) distance from the robot to EVERY candidate is
# also computed, with the same planner PR #2830 uses. That is the only extra
# work, it happens on ~40 decisions out of ~5000, and it feeds nothing back.
# ---------------------------------------------------------------------------
TRACE = False

# ---------------------------------------------------------------------------
# FAITHFUL LOOP TEMPORALITY  (the sim_2830_fidelity correction)
#
# Every earlier bench in this workspace FROZE the robot while the next goal was
# being selected: sim.drive() returned, the loop computed the next goal at zero
# cost in simulated time, and only then did the robot move again. That is not
# what dimOS does. Verified against the installed package and written up with
# line references in loop_semantics.md; the short version:
#
#   wavefront_frontier_goal_selector.py:812  publish goal G_k
#   ...                             :819  goal_reached_event.clear()
#   ...                             :823  goal_reached_event.wait(goal_timeout)
#   ...                             :828  on TIMEOUT: log a warning and LOOP.
#                                          Nothing is cancelled. The navigator
#                                          (ReplanningAStarPlanner) is still
#                                          driving to G_k and keeps driving.
#   ...                             :794-800  read odom, inflate, RUN THE
#                                          SELECTION - which is their pure-python
#                                          full-grid BFS, seconds not milliseconds
#   ...                             :812  publish G_{k+1}; only NOW does
#                                          GlobalPlanner.handle_goal_request
#                                          (global_planner.py:134-140) drop G_k.
#
# So on a timed-out goal the robot walks toward the OLD goal for the whole
# selection compute, and the goal it is handed next was chosen for the pose it
# had when the compute STARTED. T_SEL_S is that compute, in simulated seconds.
#
#   T_SEL_S = 0.0  reproduces every earlier bench exactly (frozen robot), and is
#                  kept as an arm so the two can be compared rather than swapped.
#   T_SEL_S > 0    the faithful loop.
#
# On a goal that was REACHED (or refused) the robot really is standing still
# while the next selection runs - the local planner stopped it - so there
# T_SEL_S is standstill, which costs simulated time and nothing else. That
# asymmetry is the whole point and it is modelled, not averaged.
# ---------------------------------------------------------------------------
T_SEL_S = 0.0


def _elapse(sim, dt: float) -> None:
    """Simulated time passes with the robot standing still."""
    if dt > 0.0:
        sim.t += dt
        sim._record()


def _drive_during(sim, dt: float, goal_xy) -> tuple[str | None, float, float]:
    """`dt` simulated seconds in which the robot does what the navigator was
    already doing.

    goal_xy is not None  -> the previous goal was never cancelled (the selector
        timed out on it), so the navigator is still driving to it. Their drive
        loop is re-entered on the SAME goal with a `dt` budget: same replanning,
        same lidar revolutions, same bump reflex. If the robot arrives, is
        refused, or runs out of path inside `dt`, it stands still for the rest.
    goal_xy is None      -> the robot is stopped (arrived, or the navigator
        cancelled), so `dt` is pure standstill.

    Returns (outcome or None, metres travelled, seconds actually consumed).
    The seconds can exceed `dt` by at most one cell-step plus one pivot, because
    explore_sim checks its clock between sub-steps; that overshoot is the same
    one the main drive already has and it is reported, not corrected.
    """
    if dt <= 0.0:
        return None, 0.0, 0.0
    t0, m0 = sim.t, sim.run.path_m
    res = None
    if goal_xy is not None:
        old = ES.GOAL_TIMEOUT_S
        ES.GOAL_TIMEOUT_S = dt
        try:
            res = sim.drive(goal_xy)
        finally:
            ES.GOAL_TIMEOUT_S = old
        # explore_sim returns "timeout" for TWO different things: the clock ran
        # out (drive:793) and the path ran out or produced no motion
        # (drive:801-805). Only the first means the robot is still going. See
        # _still_pursuing.
        if res == "timeout" and not _still_pursuing(res, sim.t - t0, dt):
            res = "exhausted"
    _elapse(sim, dt - (sim.t - t0))
    return res, sim.run.path_m - m0, sim.t - t0


def _still_pursuing(outcome: str, elapsed_s: float, budget_s: float) -> bool:
    """Is the navigator still driving to this goal?

    `explore_sim.Sim.drive` returns "timeout" for two different events and the
    difference is the whole question here:

      drive:793  the clock ran out while the robot was walking. In dimOS this is
                 SEL:823 returning False, and NOTHING is cancelled: the local
                 planner is still following its path. STILL PURSUING.
      drive:801-805  the path ran out without arriving, or a whole replan
                 produced no motion. In dimOS the local planner would have
                 reached the end of its path, gone to final_rotation and then
                 "arrived" (LP:277-310), which stops it and publishes
                 goal_reached=True. NOT pursuing.

    They are told apart by the clock, which is exact here: the first can only
    happen at (or just past) the budget, the second only before it. In config
    `scoring` the budget is 1e9 s, so a "timeout" there is ALWAYS path
    exhaustion and the robot is always stopped - which is what makes the
    pre-declared `scoring` identity check hold.
    """
    return outcome == "timeout" and elapsed_s >= budget_s - 1e-6


def _geodesic_to_candidates(pose, cands, costmap):
    """Route length robot -> each candidate, over the costmap it was handed,
    under the PLANNER's cost rule (the one the simulated rover's own drives
    obey). One wave for all candidates; see fix_hysteresis.geodesic_field."""
    return FIX.geodesic_field(
        np.asarray(costmap.grid), float(costmap.resolution),
        float(costmap.origin.position.x), float(costmap.origin.position.y),
        (float(pose.x), float(pose.y)), cands)


@dataclass
class GoalRecord:
    index: int
    x: float
    y: float
    # WHERE THE ROBOT WAS WHEN THIS GOAL TOOK EFFECT (was published). At
    # T_SEL_S = 0 this is also where it was when the goal was chosen, and every
    # earlier bench's `from_x/from_y` means exactly this field. The class-N
    # filter (analyse_cand2.swings_of) measures displacement from here, and the
    # physically right pose to measure it from is the one the robot is at when
    # it starts serving the goal.
    from_x: float
    from_y: float
    # WHERE THE ROBOT WAS WHEN THE SELECTOR CHOSE THIS GOAL, i.e. T_SEL_S
    # earlier: wavefront_frontier_goal_selector.py:794 reads latest_odometry at
    # the TOP of the loop, before the selection runs. Equal to from_x/from_y
    # when T_SEL_S = 0.
    dec_x: float
    dec_y: float
    d_robot: float          # straight-line robot -> goal when the goal took effect
    d_robot_dec: float      # straight-line robot -> goal when the goal was CHOSEN
    d_prev_goal: float      # straight-line previous goal -> this goal
    outcome: str
    path_m_at_issue: float
    area_m2_at_issue: float
    decide_ms: float
    n_clusters: int
    # what the robot did during the T_SEL_S window that preceded this goal
    sel_outcome: str = ""   # "" = frozen/standstill; else the drive result on the OLD goal
    sel_moved_m: float = 0.0
    sel_s: float = 0.0
    # explore_sim returned "timeout" because the PATH ran out, not because the
    # clock did. On the real robot that is an arrival, not an abandonment.
    path_exhausted: bool = False


@dataclass
class ArmRun:
    arm: str
    map_name: str
    start_name: str
    config_name: str
    goals: list[GoalRecord] = field(default_factory=list)
    poses: list = field(default_factory=list)
    coverage_curve: list = field(default_factory=list)
    path_m: float = 0.0
    sim_s: float = 0.0
    area_m2: float = 0.0
    ceiling_m2: float = 0.0
    goals_reached: int = 0
    goals_timed_out: int = 0
    goals_no_path: int = 0
    impacts: int = 0
    reversals: int = 0
    end_reason: str = ""
    self_stopped: bool = False
    wall_capped: bool = False
    trace: list = field(default_factory=list)
    # Whatever the harness-side policy counted about itself, if any. Reported,
    # never fed back: no policy reads these.
    policy_counters: dict = field(default_factory=dict)
    # Half the bounding-box diagonal of the body-passable floor of this map. A
    # consecutive-goal jump longer than this is counted as a cross-map swing.
    swing_threshold_m: float = float("nan")
    # --- the faithful-temporality bookkeeping (reported, never fed back) -----
    t_sel_s: float = 0.0            # the declared selection compute, per decision
    sel_windows: int = 0            # selection windows that were entered
    sel_driving_windows: int = 0    # ... of which the robot was still driving
    sel_moved_m: float = 0.0        # metres walked inside selection windows
    sel_time_s: float = 0.0         # simulated seconds spent inside them
    sel_arrived: int = 0            # old goal reached DURING a selection window
    sel_blocked: int = 0            # old goal refused during a selection window
    goals_path_exhausted: int = 0   # "timeout" returns that were path exhaustion


def run_arm(world, start, heading, arm: str, config_name: str,
            goal_timeout_s: float, shipped_loop: bool,
            ceiling_m2: float, map_name: str, start_name: str,
            swing_threshold_m: float = float("nan"),
            t_sel_s: float = 0.0) -> ArmRun:
    """dimOS's exploration loop, driven against explore_sim's world.

    shipped_loop=True  -> config "shipped": get_exploration_goal() is called
        exactly as _run_exploration_loop calls it, so ALL of the shipped
        behaviour runs, self-stops included, and the goal timeout applies. In
        THIS bench that timeout is the UPSTREAM default, 15.0 s
        (WavefrontConfig.goal_timeout), not the 45 s of our own blueprint that
        every earlier bench in this workspace used. Nothing of theirs is
        bypassed.

    shipped_loop=False -> config "scoring": the same decision decomposed into
        the three of their methods that _run_exploration_loop's happy path
        reaches - detect_frontiers (which ends with their _rank_frontiers, so
        the ordering IS the scoring under test), _update_exploration_direction,
        mark_explored_goal - with two of their timers left out and one filter of
        ours added:
          - the info-gain self-stop is not run (this config exists to measure
            the scoring past the point where that timer ends the run),
          - the goal timeout is not applied (so each arm PAYS for the travel its
            own choices cost, instead of being rescued mid-walk),
          - a failed-goal filter is applied to their ranked list (see above).
        The scoring itself is untouched in both configs.
    """
    module, policy = parse_arm(arm)
    label = f"{arm} / {config_name}"
    sim = ES.Sim(world, start, heading, label)

    # The goal timeout lives as a module constant inside explore_sim.Sim.drive.
    # Both arms are driven with the same value; it is restored after the run.
    old_timeout = ES.GOAL_TIMEOUT_S
    ES.GOAL_TIMEOUT_S = goal_timeout_s

    ex = DS.make_explorer(module, goal_timeout=goal_timeout_s)
    if policy is not None:
        # Harness-side wrapper: detect_frontiers is replaced ON THE INSTANCE by
        # one that calls the original and then re-ranks what it returned. Both
        # configs go through detect_frontiers, so the policy applies to both,
        # and the selector's own bookkeeping (direction, explored goals) follows
        # the policy's choice exactly as it would follow its own.
        FIX.install(ex, policy)
        if getattr(policy, "wants_nav", False):
            # The one explicit hook a body-aware policy gets: the discovered
            # occupancy grid and the robot pose, i.e. what a navigation costmap
            # subscriber has on a real robot. No ground truth passes through it;
            # see fix_composite.Composite._body_reachable.
            policy.bind_nav(sim)
    out = ArmRun(arm=arm, map_name=map_name, start_name=start_name,
                 config_name=config_name, ceiling_m2=ceiling_m2,
                 swing_threshold_m=swing_threshold_m, t_sel_s=t_sel_s)

    goals_published = 0
    consecutive_failures = 0
    prev_goal = None
    # The goal the navigator is STILL pursuing, if any. Set when a goal's
    # pursuit ended on the selector's timeout (the selector loops without
    # cancelling: selector line 823 -> 828 -> 787), cleared when the goal was
    # reached or the planner refused it (both of which stop the local planner:
    # global_planner.py:149-170, 268-284).
    live_goal = None
    # T_SEL_S = 0 is the OLD arm in full: robot frozen during selection AND
    # frozen through the retry wait, i.e. every earlier bench in this workspace,
    # bit for bit. The faithful semantics are one switch, not a dial with a
    # half-corrected middle.
    faithful = t_sel_s > 0.0
    idle = 0
    failed: list[tuple[float, float, float]] = []
    last_path_m, last_area = 0.0, sim.area_m2

    # --- trace wrapper on the instance; the upstream file is not touched -----
    #
    # RESIDUAL JOB (sim_2830_resid) adds two REPORT-ONLY fields to the trace and
    # changes no behaviour and no returned value:
    #
    #   "terms"   the five weighted terms of the arm's own score, per candidate.
    #             Four of them (info gain, explored-goals repeller, distance,
    #             momentum) are recomputed from the selector's OWN config and
    #             OWN read-only helpers; the fifth (obstacles) is DERIVED from
    #             the total the original returned, so the expensive
    #             _compute_distance_to_obstacles square search is never run a
    #             second time and the five terms sum to the returned total by
    #             construction.
    #   "policy"  for an arm carrying a harness policy, the policy's own
    #             adjusted score per candidate, with the route length and the
    #             direction deviation it used. Recomputed from the policy's own
    #             methods immediately AFTER its reorder() returned, i.e. with
    #             the same heading state that produced the ranking. Needed
    #             because the score captured above is the UPSTREAM score, and on
    #             a policy arm the ranking under test is the policy's.
    scored: list = []
    terms: list = []
    poltrace: list = []
    if TRACE:
        _orig_score = ex._compute_comprehensive_frontier_score
        _cfg = ex.config

        def _weighted_terms(frontier, frontier_size, robot_pose, costmap, total):
            """(info, explored, distance, obstacles, momentum), each already
            multiplied by its weight, summing to `total`."""
            try:
                rd = math.hypot(float(frontier.x) - float(robot_pose.x),
                                float(frontier.y) - float(robot_pose.y))
                ds = 1.0 / (1.0 + abs(rd - _cfg.lookahead_distance))
                mx = _cfg.min_frontier_perimeter / costmap.resolution * 10
                ig = min(frontier_size / mx, 1.0)
                eg = min(ex._compute_distance_to_explored_goals(frontier)
                         / _cfg.max_explored_distance, 1.0)
                mo = ex._compute_direction_momentum_score(frontier, robot_pose)
                w = [0.3 * ig, 0.3 * eg, 0.2 * ds, None, 0.05 * mo]
                w[3] = total - (w[0] + w[1] + w[2] + w[4])
                return [round(v, 6) for v in w]
            except Exception:
                return None

        def _traced_score(frontier, frontier_size, robot_pose, costmap, _o=_orig_score):
            v = _o(frontier, frontier_size, robot_pose, costmap)
            scored.append((float(frontier.x), float(frontier.y), int(frontier_size), float(v)))
            terms.append(_weighted_terms(frontier, frontier_size, robot_pose,
                                         costmap, float(v)))
            return v

        ex._compute_comprehensive_frontier_score = _traced_score

        if policy is not None and hasattr(policy, "reorder"):
            _orig_reorder = policy.reorder

            def _traced_reorder(pose, costmap, ranked, tap, *a, _o=_orig_reorder, **kw):
                out_ = _o(pose, costmap, ranked, tap, *a, **kw)
                try:
                    base = [tap.get((round(float(f.x), 4), round(float(f.y), 4)),
                                    float("nan")) for f in ranked]
                    if hasattr(policy, "_distances"):
                        dd = policy._distances(pose, ranked, costmap)
                        dev = [policy._deviation(pose, f) for f in ranked]
                        adj = []
                        for b, x, v in zip(base, dd, dev):
                            if not (math.isfinite(b) and math.isfinite(x)) or b <= 0.0:
                                adj.append(None)
                            else:
                                adj.append(b * math.exp(-policy.lam_d * max(0.0, x))
                                           * math.exp(-policy.lam_dir * v))
                        rec = {
                            "in_order": [[round(float(f.x), 3), round(float(f.y), 3)]
                                         for f in ranked],
                            "base": [None if not math.isfinite(b) else round(b, 8)
                                     for b in base],
                            "route_m": [None if not math.isfinite(x) else round(x, 3)
                                        for x in dd],
                            "dev": [round(v, 5) for v in dev],
                            "adj": [None if v is None else round(v, 10) for v in adj],
                            "heading": (None if policy._heading is None else
                                        [round(policy._heading[0], 5),
                                         round(policy._heading[1], 5)]),
                            "out_order": [[round(float(f.x), 3), round(float(f.y), 3)]
                                          for f in out_],
                        }
                    else:
                        rec = {"in_order": [[round(float(f.x), 3), round(float(f.y), 3)]
                                            for f in ranked],
                               "base": [None if not math.isfinite(b) else round(b, 8)
                                        for b in base],
                               "note": "policy has no _distances; adjusted score not recovered"}
                    poltrace.append(rec)
                except Exception as exc:              # never let the report break a run
                    poltrace.append({"error": repr(exc)})
                return out_

            policy.reorder = _traced_reorder

    def decide(pose, costmap):
        """One goal, plus how many clusters the detector found."""
        if TRACE:
            scored.clear()
            terms.clear()
            poltrace.clear()
        if shipped_loop:
            g = ex.get_exploration_goal(pose, costmap)
            return g, (len(ex.explored_goals) if g is not None else 0), []
        ranked = ex.detect_frontiers(pose, costmap)      # their code, their ranking
        n = len(ranked)
        dead = [(f.x, f.y) for f in ranked
                if any((f.x - fx) ** 2 + (f.y - fy) ** 2 < FAILED_RADIUS_M ** 2
                       for fx, fy, ft in failed if sim.t - ft < FAILED_HOLD_S)]
        keep = [f for f in ranked
                if not any((f.x - fx) ** 2 + (f.y - fy) ** 2 < FAILED_RADIUS_M ** 2
                           for fx, fy, ft in failed if sim.t - ft < FAILED_HOLD_S)]
        if not keep:
            ex.last_costmap = costmap
            return None, n, dead
        g = keep[0]
        ex._update_exploration_direction(pose, g)        # their code
        ex.mark_explored_goal(g)                         # their code
        ex.last_costmap = costmap
        return g, n, dead

    wall_t0 = time.time()
    try:
        while True:
            if RUN_CAP_S and (time.time() - wall_t0) >= RUN_CAP_S:
                out.wall_capped = True
                out.end_reason = f"wall_cap: {RUN_CAP_S:.0f} s of real time"
                break
            over = sim.over_budget()
            if over:
                out.end_reason = f"harness: {over}"
                break
            if idle >= MAX_IDLE_DECISIONS:
                out.end_reason = (f"harness: {MAX_IDLE_DECISIONS} decisions in a row with "
                                  f"< {IDLE_MOVE_M} m of motion and < {IDLE_GAIN_M2} m2 of map")
                break

            inflated = ES.simple_inflate(sim.discovered, INFLATE_M, world.res)
            costmap = DS.to_occupancy_grid(inflated, world.res, world.ox, world.oy, sim.t)
            pose = DS.Vector3(sim.x, sim.y, 0.0)
            dec_x, dec_y = sim.x, sim.y      # the pose the SELECTOR sees

            t0 = time.perf_counter()
            goal, n_clusters, dead = decide(pose, costmap)
            decide_ms = (time.perf_counter() - t0) * 1000.0

            # --- the selection compute, in SIMULATED time -------------------
            # The selector has read its odometry and its costmap (above) and is
            # now inside detect_frontiers. Nothing has been published yet, so
            # the navigator is still executing whatever it last received.
            sel_res, sel_m, sel_s = _drive_during(sim, t_sel_s,
                                                  live_goal if faithful else None)
            if faithful:
                out.sel_windows += 1
                out.sel_moved_m += sel_m
                out.sel_time_s += sel_s
                if live_goal is not None:
                    out.sel_driving_windows += 1
                    if sel_res == "reached":
                        out.sel_arrived += 1
                        live_goal = None
                    elif sel_res in ("blocked", "exhausted"):
                        out.sel_blocked += 1
                        live_goal = None

            if TRACE:
                tr = {
                    "goal_index": goals_published if goal is not None else -1,
                    # the pose the SELECTOR saw, i.e. what `pose` above holds
                    "robot": [round(dec_x, 3), round(dec_y, 3)],
                    # where the robot actually is when this goal takes effect
                    "robot_pub": [round(sim.x, 3), round(sim.y, 3)],
                    "sel_moved_m": round(sel_m, 3),
                    "t": round(sim.t, 2),
                    "path_m": round(sim.run.path_m, 3),
                    "prev_goal": list(prev_goal) if prev_goal is not None else None,
                    # (x, y, cluster_size, the arm's own score), in the order the
                    # arm's own _rank_frontiers scored them
                    "cands": [[round(a, 3), round(b, 3), c, d] for a, b, c, d in scored],
                    # report-only, see the comment on the trace wrapper
                    "terms": list(terms),
                    "policy": (dict(poltrace[-1]) if poltrace else None),
                    "suppressed": [[round(a, 3), round(b, 3)] for a, b in dead],
                    "chosen": ([round(float(goal.x), 3), round(float(goal.y), 3)]
                               if goal is not None else None),
                    "is_swing": False,
                    "geo": None,
                }
                if goal is not None and prev_goal is not None:
                    jump = math.hypot(float(goal.x) - prev_goal[0],
                                      float(goal.y) - prev_goal[1])
                    if jump > swing_threshold_m:
                        tr["is_swing"] = True
                        pts = [(a, b) for a, b, _c, _d in scored]
                        tr["geo"] = [round(v, 3) if math.isfinite(v) else None
                                     for v in _geodesic_to_candidates(pose, pts, costmap)]
                out.trace.append(tr)

            if goal is None:
                if ex.self_stopped:
                    out.self_stopped = True
                    out.end_reason = "explorer stopped itself: no information gain"
                    break
                consecutive_failures += 1
                if goals_published >= 2 and consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                    out.self_stopped = True
                    out.end_reason = (f"explorer stopped itself: {consecutive_failures} "
                                      f"consecutive failures finding new frontiers")
                    break
                # Their retry wait (selector lines 841-848). No goal was
                # published, so the navigator still holds the previous one and
                # the robot keeps walking to it through the wait as well.
                r_res, r_m, r_s = _drive_during(sim, RETRY_WAIT_S,
                                                live_goal if faithful else None)
                if faithful and live_goal is not None:
                    out.sel_moved_m += r_m
                    out.sel_time_s += r_s
                    if r_res in ("reached", "blocked", "exhausted"):
                        live_goal = None
                idle += 1
                continue

            consecutive_failures = 0
            gx, gy = float(goal.x), float(goal.y)
            # THE PUBLISH. Everything below is measured from where the robot is
            # NOW, which after a selection window is not where it was when the
            # selector chose this goal.
            d_robot = math.hypot(gx - sim.x, gy - sim.y)
            d_prev = (math.hypot(gx - prev_goal[0], gy - prev_goal[1])
                      if prev_goal is not None else float("nan"))
            rec = GoalRecord(index=goals_published, x=gx, y=gy,
                             from_x=sim.x, from_y=sim.y,
                             dec_x=dec_x, dec_y=dec_y, d_robot=d_robot,
                             d_robot_dec=math.hypot(gx - dec_x, gy - dec_y),
                             d_prev_goal=d_prev, outcome="", decide_ms=decide_ms,
                             path_m_at_issue=sim.run.path_m, area_m2_at_issue=sim.area_m2,
                             n_clusters=n_clusters,
                             sel_outcome=(sel_res or ""), sel_moved_m=sel_m,
                             sel_s=sel_s)
            goals_published += 1
            prev_goal = (gx, gy)

            gap_at_issue = d_robot
            t_drive0 = sim.t
            outcome = sim.drive((gx, gy))
            drive_elapsed = sim.t - t_drive0
            rec.outcome = outcome
            out.goals.append(rec)
            if outcome == "reached":
                out.goals_reached += 1
                live_goal = None            # navigator stopped: arrived
            elif outcome == "timeout":
                out.goals_timed_out += 1
                if (gap_at_issue - math.hypot(gx - sim.x, gy - sim.y)) < ES.ARRIVE_M:
                    failed.append((gx, gy, sim.t))
                # The selector timed out and cancelled NOTHING (selector 823 ->
                # 828). The navigator is still driving to this goal and will
                # keep driving until the next publish - but ONLY if the clock is
                # what ended the pursuit; a path that ran out is an arrival on
                # the real robot and stops it. See _still_pursuing.
                # NOTE: rec.outcome stays "timeout", the value every earlier
                # bench recorded, so a T_sel = 0 run is byte-identical to the
                # recorded ones. The distinction is carried by a NEW field.
                if _still_pursuing(outcome, drive_elapsed, goal_timeout_s):
                    live_goal = (gx, gy)
                else:
                    live_goal = None
                    out.goals_path_exhausted += 1
                    rec.path_exhausted = True
            else:
                out.goals_no_path += 1
                failed.append((gx, gy, sim.t))
                # The planner refused the goal, which in dimOS means
                # GlobalPlanner.cancel_goal() and a stopped local planner
                # (global_planner.py:331-341, 149-170): the robot is standing
                # still until the next publish.
                live_goal = None
                sim.t += ES.FAIL_BREATH_S
                sim._record()

            if (sim.run.path_m - last_path_m < IDLE_MOVE_M
                    and sim.area_m2 - last_area < IDLE_GAIN_M2):
                idle += 1
            else:
                idle = 0
            last_path_m, last_area = sim.run.path_m, sim.area_m2
    finally:
        ES.GOAL_TIMEOUT_S = old_timeout

    sim.run.sim_s = sim.t
    sim.run.coverage_curve.append((sim.run.path_m, sim.area_m2))
    out.poses = sim.run.poses
    out.coverage_curve = sim.run.coverage_curve
    out.path_m = sim.run.path_m
    out.sim_s = sim.t
    out.area_m2 = sim.area_m2
    out.impacts = sim.run.impacts
    out.reversals = ES.count_reversals(sim.run.poses)
    if policy is not None:
        if hasattr(policy, "counters"):
            out.policy_counters = dict(policy.counters(), name=policy.name)
        else:
            out.policy_counters = {k: v for k, v in vars(policy).items()
                                   if k.startswith("n_") or k == "name"}
    if not out.end_reason:
        out.end_reason = "loop exited"
    return out


# ===========================================================================
# maps, starts, scoring
# ===========================================================================

MAPS = [
    # The dimOS go2_bigoffice dataset, extracted by extract_bigoffice.py. This is
    # the ONLY change made to this file for the big-office run: two data lines.
    ("bigoffice", "bigoffice.npz"),
    ("bigoffice_hc", "bigoffice_hc.npz"),
    # The fleet: four more dimOS public recordings, extracted by the same chain
    # (extract_fleet.py, which reproduces bigoffice.npz array for array). Data
    # lines only; nothing else in this file changed for them.
    ("hk_office", "hk_office.npz"),
    ("hk_allaround", "hk_allaround.npz"),
    ("hk_elevator", "hk_elevator.npz"),
    ("go2_short", "go2_short.npz"),
    ("hk_entrance", "hk_entrance.npz"),
    ("hk_park", "hk_park.npz"),
    ("map_20260823", "map_20260823.npz"),
    ("map_20260825", "map_20260825.npz"),
    ("costmap_175224", "costmap_175224.npz"),
    ("costmap_175905", "costmap_175905.npz"),
    # map_20260827.npz is NOT in this list. Measured: 7743 seen cells and ZERO
    # cells at or above OCCUPIED_AT, i.e. a map with no obstacles in it at all.
    # A frontier explorer on a map with no walls has nothing to route around and
    # nothing to be surprised by: one 12 m lidar revolution from the start
    # reveals 95% of it before any goal is issued (measured: 0.5 m of travel for
    # 95.4% coverage, identically for both arms). It cannot separate the two
    # strategies, so it is excluded and said so rather than quietly averaged in.
]

# THE 15 s CORRECTION, and it applies to every baseline in this bench.
# Every earlier bench in this workspace ran config `shipped` with OUR blueprint's
# 45 s goal timeout. The UPSTREAM default is 15.0: WavefrontConfig.goal_timeout
# in dimos/navigation/frontier_exploration/wavefront_frontier_goal_selector.py
# (line 93 of the installed package, md5 e77c328643c959a49077115e8a341f2c, the
# same file as pr2830/selector_base.py), and the go2 blueprint instantiates
# WavefrontFrontierExplorer.blueprint() with no override. This bench is
# upstream-faithful: SHIPPED_GOAL_TIMEOUT_S = 15.0, and every arm including the
# baselines is re-run at it. Numbers here are NOT comparable to the 45 s history.
SHIPPED_GOAL_TIMEOUT_S = 15.0

CONFIGS = {
    # Upstream, untouched: get_exploration_goal() as _run_exploration_loop calls
    # it, with the upstream 15 s goal timeout (set at run time from
    # --shipped-timeout-s, default SHIPPED_GOAL_TIMEOUT_S).
    "shipped": dict(goal_timeout_s=SHIPPED_GOAL_TIMEOUT_S, shipped_loop=True),
    # The scoring alone: their ranking, no info-gain self-stop, no goal timeout
    # (each arm pays for the travel its own choices cost), plus the house
    # failed-goal filter so one pinch does not end the run.
    "scoring": dict(goal_timeout_s=1e9, shipped_loop=False),
}


def _passable(world) -> np.ndarray:
    """Free ground the 0.46 m body actually fits on, largest connected piece.

    A start has to be somewhere the rover can drive OUT of. Taking the plain
    centroid of free space instead put the first pilot's start in a 5.8 m2 nook
    behind a pinch: both arms sat there for the whole run at 0.0 m travelled,
    which measures the nook, not the strategies.
    """
    from scipy import ndimage
    ok = (world.truth == ES.FREE) & (world.clearance + 1e-6 >= ES.BODY_HALF_WIDTH_M)
    labels, n = ndimage.label(ok, structure=ES._EIGHT)
    if n == 0:
        return ok
    sizes = ndimage.sum(ok, labels, index=np.arange(1, n + 1))
    return labels == (int(np.argmax(sizes)) + 1)


def choose_starts(world, saved_pose, n_extra: int = 2) -> dict[str, tuple[float, float]]:
    """The two starts the run form asks for, plus a couple of spread ones.

    origin  (0, 0) - the dock, where every real run of ours begins.
    centre  the body-passable cell closest to the centroid of the largest
            body-passable region.
    pose    the pose saved in the map file (where the real run ended).
    spreadN farthest-point samples over the same region, so the pair of
            strategies is also compared from corners nobody chose by hand.
    Any candidate that is not in the largest body-passable region is dropped.
    """
    body = _passable(world)
    ys, xs = np.nonzero(body)
    if ys.size == 0:
        return {}
    starts: dict[str, tuple[float, float]] = {}

    def snap(x, y, name):
        gy, gx = world.cell(x, y)
        if not body[gy, gx]:
            return False
        starts[name] = world.world_xy(gy, gx)
        return True

    snap(0.0, 0.0, "origin")
    cy, cx = ys.mean(), xs.mean()
    i = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
    starts["centre"] = world.world_xy(int(ys[i]), int(xs[i]))
    snap(float(saved_pose[0]), float(saved_pose[1]), "pose")

    # farthest-point sampling, seeded on what we already have
    chosen = [world.cell(*s) for s in starts.values()]
    for k in range(n_extra):
        d = np.full(ys.shape, np.inf)
        for gy, gx in chosen:
            d = np.minimum(d, (ys - gy) ** 2 + (xs - gx) ** 2)
        j = int(np.argmax(d))
        chosen.append((int(ys[j]), int(xs[j])))
        starts[f"spread{k + 1}"] = world.world_xy(int(ys[j]), int(xs[j]))
    return starts


def _med(v):
    return statistics.median(v) if v else float("nan")


def summarise(run: ArmRun) -> dict:
    d_robot = [g.d_robot for g in run.goals]
    d_prev = [g.d_prev_goal for g in run.goals if not math.isnan(g.d_prev_goal)]
    reached = [g for g in run.goals if g.outcome == "reached"]
    last_ok = max((g.index for g in run.goals if g.outcome == "reached"), default=-1)
    cov = 100.0 * run.area_m2 / run.ceiling_m2 if run.ceiling_m2 else float("nan")
    s = {
        "n_goals_reached_only": len(reached),
        "d_robot_to_goal_median_reached_m": _med([g.d_robot for g in reached]),
        "d_robot_to_goal_max_reached_m": max([g.d_robot for g in reached], default=float("nan")),
        # goals issued after the last one the rover actually reached: how long
        # each scorer keeps re-issuing a frontier nothing can drive to
        "dead_tail_goals": len(run.goals) - 1 - last_ok if last_ok >= 0 else len(run.goals),
        "map": run.map_name, "start": run.start_name, "config": run.config_name,
        "arm": run.arm,
        "n_goals": len(run.goals),
        "d_goal_to_goal_median_m": statistics.median(d_prev) if d_prev else float("nan"),
        "d_goal_to_goal_max_m": max(d_prev) if d_prev else float("nan"),
        "d_robot_to_goal_median_m": statistics.median(d_robot) if d_robot else float("nan"),
        "d_robot_to_goal_max_m": max(d_robot) if d_robot else float("nan"),
        # --- the two ping-pong indicators, added for the mid-start re-run -----
        # (a) how far the sequence of goals itself travels: the sum of the
        #     straight-line distances between consecutive ISSUED goals. A run
        #     that works one area through before moving on keeps this near the
        #     length of the tour; a run that ping-pongs inflates it.
        "goal_jump_total_m": sum(d_prev),
        # (b) how many of those jumps cross the whole place: consecutive goals
        #     further apart than HALF the bounding-box diagonal of the
        #     body-passable floor of this map (11.8 m here). This is the direct
        #     count of "walks all the way across, then back".
        "cross_map_swings": sum(1 for v in d_prev if v > run.swing_threshold_m),
        "swing_threshold_m": run.swing_threshold_m,
        "share_goals_beyond_5m_pct": (100.0 * sum(1 for v in d_robot if v > 5.0) / len(d_robot)
                                      if d_robot else float("nan")),
        "path_m": run.path_m,
        "area_m2": run.area_m2,
        "ceiling_m2": run.ceiling_m2,
        "coverage_pct": cov,
        "goals_reached": run.goals_reached,
        "goals_timed_out": run.goals_timed_out,
        "goals_no_path": run.goals_no_path,
        "impacts": run.impacts,
        "reversals": run.reversals,
        "sim_s": run.sim_s,
        "decide_ms_mean": (statistics.mean([g.decide_ms for g in run.goals])
                           if run.goals else float("nan")),
        "end_reason": run.end_reason,
        "wall_capped": run.wall_capped,
        "policy": run.policy_counters,
        # --- faithful temporality, reported per run --------------------------
        "t_sel_s": run.t_sel_s,
        "sel_windows": run.sel_windows,
        "sel_driving_windows": run.sel_driving_windows,
        "sel_moved_m": run.sel_moved_m,
        "sel_time_s": run.sel_time_s,
        "sel_arrived": run.sel_arrived,
        "sel_blocked": run.sel_blocked,
        "goals_path_exhausted": run.goals_path_exhausted,
        # how far the goal was from the robot when it was CHOSEN, against how
        # far it was when it took effect: the staleness the fidelity fix adds
        "d_robot_dec_median_m": _med([g.d_robot_dec for g in run.goals]),
        "sel_drift_median_m": _med([abs(g.d_robot - g.d_robot_dec) for g in run.goals]),
    }
    for frac in (0.5, 0.8, 0.9):
        area = frac * run.ceiling_m2
        s[f"path_to_{int(frac * 100)}pct_m"] = ES.path_at_area(run.coverage_curve, area)
    return s


def job(args):
    (map_name, map_path, start_name, start, heading, arm, config_name, ceiling,
     lidar_m, swing_m, trace_on, run_cap_s, shipped_timeout_s, profile,
     t_sel_s) = args
    # The robot profile is set inside the worker for the same reason the lidar
    # range is: under spawn nothing is inherited, and a worker silently left on
    # the 0.46 m rover body would be the whole experiment quietly not run.
    PROF.apply(ES, profile)
    # Set inside the worker too. Under fork this is already inherited from the
    # parent; under spawn it would not be, and a worker silently left at 12 m
    # would be the whole experiment quietly not run.
    ES.LIDAR_RANGE_M = lidar_m
    global TRACE, RUN_CAP_S
    TRACE = trace_on
    RUN_CAP_S = run_cap_s
    CONFIGS["shipped"]["goal_timeout_s"] = shipped_timeout_s
    world = ES.load_world(map_path, None, unknown_is_wall=True)
    cfg = CONFIGS[config_name]
    t0 = time.time()
    run = run_arm(world, start, heading, arm, config_name,
                  cfg["goal_timeout_s"], cfg["shipped_loop"],
                  ceiling, map_name, start_name, swing_threshold_m=swing_m,
                  t_sel_s=t_sel_s)
    wall = time.time() - t0
    out = {
        "summary": summarise(run),
        "goals": [g.__dict__ for g in run.goals],
        "poses": [(round(p[1], 3), round(p[2], 3)) for p in run.poses],
        "coverage_curve": run.coverage_curve,
        "wall_s": wall,
    }
    if trace_on:
        out["trace"] = run.trace
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "results.json"))
    ap.add_argument("--maps", nargs="*", default=None)
    ap.add_argument("--configs", nargs="*", default=list(CONFIGS))
    ap.add_argument("--jobs", type=int, default=10)
    ap.add_argument("--extra-starts", type=int, default=2)
    ap.add_argument("--pilot", action="store_true")
    ap.add_argument("--lidar-range", type=float, default=4.0,
                    help="ES.LIDAR_RANGE_M for this sweep (shipped bench: 12.0)")
    ap.add_argument("--starts", default="mid", choices=("mid", "shipped"),
                    help="'mid' = midstarts.choose_mid_starts (this re-run), "
                         "'shipped' = the original choose_starts")
    ap.add_argument("--drop-starts", nargs="*", default=[],
                    help="starts the worthiness gate rejected, as map/start")
    ap.add_argument("--arms", nargs="*", default=["stock", "pr2830"],
                    help="'stock', 'pr2830', or '<base>+<policy>' e.g. stock+H6k2")
    ap.add_argument("--run-cap-s", type=float, default=0.0,
                    help="wall-clock cap per run in seconds, 0 = off. A run that "
                         "hits it ends with end_reason 'wall_cap' and is counted "
                         "as capped; the cap applies to every arm identically.")
    ap.add_argument("--shipped-timeout-s", type=float,
                    default=SHIPPED_GOAL_TIMEOUT_S,
                    help="goal timeout of config 'shipped'. Default 15.0, the "
                         "UPSTREAM WavefrontConfig.goal_timeout. Earlier benches "
                         "in this workspace used our blueprint's 45.0; those "
                         "numbers are not comparable to these.")
    ap.add_argument("--profile", default="rover", choices=sorted(PROF.PROFILES),
                    help="robot profile: 'rover' (0.46 m body, 0.15 m/s, every "
                         "earlier bench in this workspace) or 'go2' (0.31 m "
                         "body, 0.60 m/s, derived in speed_derivation.txt)")
    ap.add_argument("--t-sel-s", type=float, default=T_SEL_S,
                    help="the selection compute, in SIMULATED seconds. 0.0 (the "
                         "default) is every earlier bench in this workspace: the "
                         "robot is frozen while the next goal is chosen. Above 0 "
                         "is the faithful loop - on a goal that TIMED OUT the "
                         "navigator was never cancelled (selector line 823 -> "
                         "828), so the robot keeps walking to the old goal for "
                         "this long and the goal it gets next was chosen for the "
                         "pose it had when the compute started. See "
                         "loop_semantics.md.")
    ap.add_argument("--trace", action="store_true",
                    help="dump the per-decision candidate/score trace (diagnosis)")
    args = ap.parse_args(argv)

    for a in args.arms:
        parse_arm(a)          # fail here, not in a worker
    ES.LIDAR_RANGE_M = args.lidar_range
    prof = PROF.apply(ES, args.profile)
    CONFIGS["shipped"]["goal_timeout_s"] = args.shipped_timeout_s
    print(PROF.describe(args.profile))
    print(f"  goal timeout {args.shipped_timeout_s:g} s x {prof['SPEED_MPS']:g} m/s "
          f"= {args.shipped_timeout_s * prof['SPEED_MPS']:.2f} m of walk before the "
          f"loop pulls the robot off a goal")
    _sem = ("FAITHFUL: robot keeps driving to the old goal through it"
            if args.t_sel_s > 0 else "FROZEN: the old, unfaithful arm")
    print(f"  selection compute T_sel = {args.t_sel_s:g} s ({_sem})")
    print(f"lidar range {ES.LIDAR_RANGE_M} m, starts '{args.starts}', "
          f"run cap {args.run_cap_s:.0f} s"
          f"{' (off)' if not args.run_cap_s else ''}, "
          f"shipped goal timeout {args.shipped_timeout_s:g} s\n")

    maps = [(n, p) for n, p in MAPS if args.maps is None or n in args.maps]
    tasks = []
    meta = {"lidar_range_m": args.lidar_range, "starts_mode": args.starts,
            "profile": args.profile, "profile_constants": prof,
            "body": PROF.BODY[args.profile],
            "arms": list(args.arms), "trace": bool(args.trace),
            "run_cap_s": args.run_cap_s,
            "shipped_goal_timeout_s": args.shipped_timeout_s,
            "t_sel_s": args.t_sel_s,
            "loop_semantics": ("faithful: the navigator is not cancelled on a "
                               "goal timeout, so the robot keeps driving to the "
                               "old goal for t_sel_s while the next selection "
                               "computes" if args.t_sel_s > 0 else
                               "frozen: the robot stands still while the next "
                               "goal is selected (every earlier bench)"),
            "configs": {k: dict(v) for k, v in CONFIGS.items()}}
    for name, fname in maps:
        path = os.path.join(SCRATCH, fname)
        world = ES.load_world(path, None, unknown_is_wall=True)
        if args.starts == "mid":
            starts, sel_meta = MS.choose_mid_starts(world, verbose=False)
            meta[f"{name}/_selection"] = sel_meta
        else:
            starts = choose_starts(world, np.load(path)["pose_xy"],
                                   n_extra=args.extra_starts)
            if "origin" not in starts:
                print(f"{name}: (0,0) is not body-passable free ground here, dropped")
        swing = MS.body_bbox_diagonal_m(world) / 2.0
        for sname, s in starts.items():
            if f"{name}/{sname}" in args.drop_starts:
                print(f"{name:16s} {sname:7s} dropped by the worthiness gate")
                continue
            ceiling = world.visible_area_m2(s)
            meta[f"{name}/{sname}"] = {
                "start": list(s), "ceiling_m2": ceiling,
                "free_m2": world.free_area_m2, "observed_m2": world.observed_area_m2,
                "res": world.res, "ox": world.ox, "oy": world.oy,
                "shape": list(world.truth.shape),
                "swing_threshold_m": swing,
            }
            print(f"{name:16s} {sname:7s} start=({s[0]:6.2f},{s[1]:6.2f})  "
                  f"free {world.free_area_m2:5.1f} m2  ceiling {ceiling:5.1f} m2  "
                  f"swing > {swing:.1f} m")
            for cfg in args.configs:
                for arm in args.arms:
                    tasks.append((name, path, sname, s, 0.0, arm, cfg, ceiling,
                                  args.lidar_range, swing, args.trace,
                                  args.run_cap_s, args.shipped_timeout_s,
                                  args.profile, args.t_sel_s))
    if args.pilot:
        tasks = tasks[:2]

    print(f"\n{len(tasks)} runs on {args.jobs} workers\n")
    results = []
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(job, tasks):
            s = r["summary"]
            print(f"  {s['map']:16s} {s['start']:7s} {s['config']:8s} {s['arm']:14s} "
                  f"goals {s['n_goals']:3d}  med d(robot->goal) "
                  f"{s['d_robot_to_goal_median_m']:5.2f} m  max {s['d_robot_to_goal_max_m']:5.2f} m"
                  f"  path {s['path_m']:6.1f} m  cov {s['coverage_pct']:5.1f}%  "
                  f"jumps {s['goal_jump_total_m']:6.1f} m  swings {s['cross_map_swings']:2d}  "
                  f"[{r['wall_s']:.0f}s] {'WALL_CAP ' if s.get('wall_capped') else ''}"
                  f"{s['end_reason']}")
            results.append(r)

    with open(args.out, "w") as fh:
        json.dump({"schema": "bench_2830/1", "meta": meta,
                   "results": results}, fh)
    print(f"\n-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

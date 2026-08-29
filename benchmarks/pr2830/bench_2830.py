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
    path = "/home/openclaw/vector-dimos/tools/explore_sim.py"
    spec = importlib.util.spec_from_file_location("explore_sim", path)
    assert spec and spec.loader
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()

sys.path.insert(0, HERE)
import dimos_selector as DS  # noqa: E402


# --- silence dimOS's per-frontier INFO logging ------------------------------
class _Quiet:
    def info(self, *a, **k): pass
    def warning(self, *a, **k): pass
    def debug(self, *a, **k): pass
    def error(self, *a, **k): pass


for _mod in (DS.sel_stock, DS.sel_pr):
    _mod.logger = _Quiet()

ARMS = {"stock": DS.sel_stock, "pr2830": DS.sel_pr}

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

# The failed-goal filter used in config "scoring" only. It is OUR addition
# (vector_dimos.fast_explorer.FAILED_GOAL_RADIUS_M / _HOLD_S), applied to BOTH
# arms identically and outside their code: a goal the drive planner refused is
# suppressed for 60 s within 0.6 m. Without it a run ends the moment the first
# frontier sits in a pinch the 0.46 m body does not fit through - measured, on
# every start of the pilot: 2 to 4 goals reached, then 12 to 18 re-issues of the
# same unreachable centroid. That tail says nothing about the scoring.
FAILED_RADIUS_M = 0.6
FAILED_HOLD_S = 60.0


@dataclass
class GoalRecord:
    index: int
    x: float
    y: float
    from_x: float
    from_y: float
    d_robot: float          # straight-line robot -> goal at the moment it was issued
    d_prev_goal: float      # straight-line previous goal -> this goal
    outcome: str
    path_m_at_issue: float
    area_m2_at_issue: float
    decide_ms: float
    n_clusters: int


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


def run_arm(world, start, heading, arm: str, config_name: str,
            goal_timeout_s: float, shipped_loop: bool,
            ceiling_m2: float, map_name: str, start_name: str) -> ArmRun:
    """dimOS's exploration loop, driven against explore_sim's world.

    shipped_loop=True  -> config "shipped": get_exploration_goal() is called
        exactly as _run_exploration_loop calls it, so ALL of the shipped
        behaviour runs, self-stops included, and the 45 s goal timeout of our
        blueprint applies. Nothing of theirs is bypassed.

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
    module = ARMS[arm]
    label = f"{arm} / {config_name}"
    sim = ES.Sim(world, start, heading, label)

    # The goal timeout lives as a module constant inside explore_sim.Sim.drive.
    # Both arms are driven with the same value; it is restored after the run.
    old_timeout = ES.GOAL_TIMEOUT_S
    ES.GOAL_TIMEOUT_S = goal_timeout_s

    ex = DS.make_explorer(module, goal_timeout=goal_timeout_s)
    out = ArmRun(arm=arm, map_name=map_name, start_name=start_name,
                 config_name=config_name, ceiling_m2=ceiling_m2)

    goals_published = 0
    consecutive_failures = 0
    prev_goal = None
    idle = 0
    failed: list[tuple[float, float, float]] = []
    last_path_m, last_area = 0.0, sim.area_m2

    def decide(pose, costmap):
        """One goal, plus how many clusters the detector found."""
        if shipped_loop:
            g = ex.get_exploration_goal(pose, costmap)
            return g, (len(ex.explored_goals) if g is not None else 0)
        ranked = ex.detect_frontiers(pose, costmap)      # their code, their ranking
        n = len(ranked)
        keep = [f for f in ranked
                if not any((f.x - fx) ** 2 + (f.y - fy) ** 2 < FAILED_RADIUS_M ** 2
                           for fx, fy, ft in failed if sim.t - ft < FAILED_HOLD_S)]
        if not keep:
            ex.last_costmap = costmap
            return None, n
        g = keep[0]
        ex._update_exploration_direction(pose, g)        # their code
        ex.mark_explored_goal(g)                         # their code
        ex.last_costmap = costmap
        return g, n

    try:
        while True:
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

            t0 = time.perf_counter()
            goal, n_clusters = decide(pose, costmap)
            decide_ms = (time.perf_counter() - t0) * 1000.0

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
                sim.t += RETRY_WAIT_S
                sim._record()
                idle += 1
                continue

            consecutive_failures = 0
            gx, gy = float(goal.x), float(goal.y)
            d_robot = math.hypot(gx - sim.x, gy - sim.y)
            d_prev = (math.hypot(gx - prev_goal[0], gy - prev_goal[1])
                      if prev_goal is not None else float("nan"))
            rec = GoalRecord(index=goals_published, x=gx, y=gy,
                             from_x=sim.x, from_y=sim.y, d_robot=d_robot,
                             d_prev_goal=d_prev, outcome="", decide_ms=decide_ms,
                             path_m_at_issue=sim.run.path_m, area_m2_at_issue=sim.area_m2,
                             n_clusters=n_clusters)
            goals_published += 1
            prev_goal = (gx, gy)

            gap_at_issue = d_robot
            outcome = sim.drive((gx, gy))
            rec.outcome = outcome
            out.goals.append(rec)
            if outcome == "reached":
                out.goals_reached += 1
            elif outcome == "timeout":
                out.goals_timed_out += 1
                if (gap_at_issue - math.hypot(gx - sim.x, gy - sim.y)) < ES.ARRIVE_M:
                    failed.append((gx, gy, sim.t))
            else:
                out.goals_no_path += 1
                failed.append((gx, gy, sim.t))
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
    if not out.end_reason:
        out.end_reason = "loop exited"
    return out


# ===========================================================================
# maps, starts, scoring
# ===========================================================================

MAPS = [
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

CONFIGS = {
    # Upstream, untouched: get_exploration_goal() as _run_exploration_loop calls
    # it, with our blueprint's 45 s goal timeout.
    "shipped": dict(goal_timeout_s=45.0, shipped_loop=True),
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
    }
    for frac in (0.5, 0.8, 0.9):
        area = frac * run.ceiling_m2
        s[f"path_to_{int(frac * 100)}pct_m"] = ES.path_at_area(run.coverage_curve, area)
    return s


def job(args):
    map_name, map_path, start_name, start, heading, arm, config_name, ceiling = args
    world = ES.load_world(map_path, None, unknown_is_wall=True)
    cfg = CONFIGS[config_name]
    t0 = time.time()
    run = run_arm(world, start, heading, arm, config_name,
                  cfg["goal_timeout_s"], cfg["shipped_loop"],
                  ceiling, map_name, start_name)
    wall = time.time() - t0
    return {
        "summary": summarise(run),
        "goals": [g.__dict__ for g in run.goals],
        "poses": [(round(p[1], 3), round(p[2], 3)) for p in run.poses],
        "coverage_curve": run.coverage_curve,
        "wall_s": wall,
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "results.json"))
    ap.add_argument("--maps", nargs="*", default=None)
    ap.add_argument("--configs", nargs="*", default=list(CONFIGS))
    ap.add_argument("--jobs", type=int, default=10)
    ap.add_argument("--extra-starts", type=int, default=2)
    ap.add_argument("--pilot", action="store_true")
    args = ap.parse_args(argv)

    maps = [(n, p) for n, p in MAPS if args.maps is None or n in args.maps]
    tasks = []
    meta = {}
    for name, fname in maps:
        path = os.path.join(SCRATCH, fname)
        world = ES.load_world(path, None, unknown_is_wall=True)
        starts = choose_starts(world, np.load(path)["pose_xy"], n_extra=args.extra_starts)
        if "origin" not in starts:
            print(f"{name}: (0,0) is not body-passable free ground here, dropped")
        for sname, s in starts.items():
            ceiling = world.visible_area_m2(s)
            meta[f"{name}/{sname}"] = {
                "start": list(s), "ceiling_m2": ceiling,
                "free_m2": world.free_area_m2, "observed_m2": world.observed_area_m2,
                "res": world.res, "ox": world.ox, "oy": world.oy,
                "shape": list(world.truth.shape),
            }
            print(f"{name:16s} {sname:7s} start=({s[0]:6.2f},{s[1]:6.2f})  "
                  f"free {world.free_area_m2:5.1f} m2  ceiling {ceiling:5.1f} m2")
            for cfg in args.configs:
                for arm in ("stock", "pr2830"):
                    tasks.append((name, path, sname, s, 0.0, arm, cfg, ceiling))
    if args.pilot:
        tasks = tasks[:2]

    print(f"\n{len(tasks)} runs on {args.jobs} workers\n")
    results = []
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(job, tasks):
            s = r["summary"]
            print(f"  {s['map']:16s} {s['start']:7s} {s['config']:8s} {s['arm']:7s} "
                  f"goals {s['n_goals']:3d}  med d(robot->goal) "
                  f"{s['d_robot_to_goal_median_m']:5.2f} m  max {s['d_robot_to_goal_max_m']:5.2f} m"
                  f"  path {s['path_m']:6.1f} m  cov {s['coverage_pct']:5.1f}%  "
                  f"[{r['wall_s']:.0f}s] {s['end_reason']}")
            results.append(r)

    with open(args.out, "w") as fh:
        json.dump({"schema": "bench_2830/1", "meta": meta,
                   "results": results}, fh)
    print(f"\n-> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

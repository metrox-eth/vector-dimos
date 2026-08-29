#!/usr/bin/env python3
"""Do the two scorers actually disagree? Same map state, same candidate set.

    shadow_2830.py --out shadow.json

The A/B runs diverge after the first goal, so every later comparison is between
two different worlds. This pass removes that: one arm DRIVES, and at each of its
decisions the other arm is handed the identical frontier centroids, the identical
sizes, the identical costmap and the identical explored-goal history, and asked
which one it would have taken. Nothing of either scorer is reimplemented -
_rank_frontiers is wrapped by a spy that records the arguments and forwards them
unchanged, and the shadow arm's own _rank_frontiers is then called on those same
arguments.

Reported per decision:
  same        the two picked the same centroid
  d_driver    straight-line robot -> the driver's pick
  d_shadow    straight-line robot -> the shadow's pick
  euclid/path for the driver's pick and the shadow's, via the PR's own
              _compute_path_cost (its A*), so the report can say how far the
              real route is from the straight line ON OUR MAPS - which is the
              whole premise of the PR's change.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import bench_2830 as B  # noqa: E402
import dimos_selector as DS  # noqa: E402

ES = B.ES


def spy_rank(ex, box):
    real = ex._rank_frontiers

    def wrapped(centroids, sizes, robot_pose, costmap):
        box["centroids"], box["sizes"] = centroids, sizes
        return real(centroids, sizes, robot_pose, costmap)

    ex._rank_frontiers = wrapped
    return ex


def run_shadow(world, start, heading, driver_arm, ceiling, map_name, start_name):
    shadow_arm = "pr2830" if driver_arm == "stock" else "stock"
    sim = ES.Sim(world, start, heading, f"{driver_arm} drives")
    old_timeout = ES.GOAL_TIMEOUT_S
    ES.GOAL_TIMEOUT_S = 1e9

    ex = DS.make_explorer(B.ARMS[driver_arm])
    box: dict = {}
    spy_rank(ex, box)
    sh = DS.make_explorer(B.ARMS[shadow_arm])
    # the PR's path-cost function, used purely as an instrument here
    probe = DS.make_explorer(B.ARMS["pr2830"])

    rows = []
    failed: list[tuple[float, float, float]] = []
    goals_published = 0
    idle = 0
    last_path, last_area = 0.0, sim.area_m2
    try:
        while True:
            if sim.over_budget() or idle >= B.MAX_IDLE_DECISIONS:
                break
            inflated = ES.simple_inflate(sim.discovered, B.INFLATE_M, world.res)
            costmap = DS.to_occupancy_grid(inflated, world.res, world.ox, world.oy, sim.t)
            pose = DS.Vector3(sim.x, sim.y, 0.0)

            box.clear()
            ranked = ex.detect_frontiers(pose, costmap)
            if "centroids" not in box or not ranked:
                break
            centroids, sizes = box["centroids"], box["sizes"]

            keep = [f for f in ranked
                    if not any((f.x - fx) ** 2 + (f.y - fy) ** 2 < B.FAILED_RADIUS_M ** 2
                               for fx, fy, ft in failed if sim.t - ft < B.FAILED_HOLD_S)]
            if not keep:
                idle += 1
                sim.t += B.RETRY_WAIT_S
                sim._record()
                continue
            goal = keep[0]

            # ---- the counterfactual, on exactly the same inputs -------------
            sh.explored_goals = list(ex.explored_goals)
            sh.exploration_direction = copy.copy(ex.exploration_direction)
            sh.last_costmap = costmap
            sh_ranked = sh._rank_frontiers(centroids, sizes, pose, costmap)
            sh_keep = [f for f in sh_ranked
                       if not any((f.x - fx) ** 2 + (f.y - fy) ** 2 < B.FAILED_RADIUS_M ** 2
                                  for fx, fy, ft in failed if sim.t - ft < B.FAILED_HOLD_S)]
            sh_goal = sh_keep[0] if sh_keep else None

            def probe_costs(v):
                if v is None:
                    return None, None
                e = math.hypot(v.x - sim.x, v.y - sim.y)
                p = probe._compute_path_cost(v, pose, costmap)   # the PR's own A*
                return e, (None if math.isinf(p) else p)

            de, dp = probe_costs(goal)
            se, sp = probe_costs(sh_goal)
            rows.append({
                "map": map_name, "start": start_name, "driver": driver_arm,
                "shadow": shadow_arm, "i": goals_published,
                "n_candidates": len(centroids),
                "same": bool(sh_goal is not None
                             and abs(sh_goal.x - goal.x) < 1e-9
                             and abs(sh_goal.y - goal.y) < 1e-9),
                "driver_euclid": de, "driver_path": dp,
                "shadow_euclid": se, "shadow_path": sp,
                "robot_x": sim.x, "robot_y": sim.y,
            })

            ex._update_exploration_direction(pose, goal)
            ex.mark_explored_goal(goal)
            ex.last_costmap = costmap
            goals_published += 1

            gap = math.hypot(goal.x - sim.x, goal.y - sim.y)
            outcome = sim.drive((float(goal.x), float(goal.y)))
            if outcome == "blocked":
                failed.append((float(goal.x), float(goal.y), sim.t))
                sim.t += ES.FAIL_BREATH_S
                sim._record()
            elif outcome == "timeout":
                if gap - math.hypot(goal.x - sim.x, goal.y - sim.y) < ES.ARRIVE_M:
                    failed.append((float(goal.x), float(goal.y), sim.t))
            if (sim.run.path_m - last_path < B.IDLE_MOVE_M
                    and sim.area_m2 - last_area < B.IDLE_GAIN_M2):
                idle += 1
            else:
                idle = 0
            last_path, last_area = sim.run.path_m, sim.area_m2
    finally:
        ES.GOAL_TIMEOUT_S = old_timeout
    return rows


def job(args):
    map_name, path, start_name, start, driver, ceiling, lidar_m = args
    ES.LIDAR_RANGE_M = lidar_m          # see bench_2830.job: set in the worker too
    world = ES.load_world(path, None, unknown_is_wall=True)
    t0 = time.time()
    rows = run_shadow(world, start, 0.0, driver, ceiling, map_name, start_name)
    return {"rows": rows, "wall_s": time.time() - t0}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "shadow.json"))
    ap.add_argument("--jobs", type=int, default=12)
    # Added for the big-office run so the same script can be pointed at the new
    # map with the same start count as bench_2830.py. Nothing else changed.
    ap.add_argument("--maps", nargs="*", default=None)
    ap.add_argument("--extra-starts", type=int, default=2)
    # Added for the mid-start re-run: the same two parameters as bench_2830.py,
    # so the head-to-head runs on the identical inputs as the A/B.
    ap.add_argument("--lidar-range", type=float, default=4.0)
    ap.add_argument("--starts", default="mid", choices=("mid", "shipped"))
    args = ap.parse_args(argv)

    ES.LIDAR_RANGE_M = args.lidar_range
    print(f"lidar range {ES.LIDAR_RANGE_M} m, starts '{args.starts}'")

    tasks = []
    for name, fname in B.MAPS:
        if args.maps is not None and name not in args.maps:
            continue
        path = os.path.join(B.SCRATCH, fname)
        world = ES.load_world(path, None, unknown_is_wall=True)
        if args.starts == "mid":
            starts, _ = B.MS.choose_mid_starts(world, verbose=False)
        else:
            starts = B.choose_starts(world, np.load(path)["pose_xy"],
                                     n_extra=args.extra_starts)
        for sname, s in starts.items():
            ceiling = 0.0
            for driver in ("stock", "pr2830"):
                tasks.append((name, path, sname, s, driver, ceiling,
                              args.lidar_range))

    print(f"{len(tasks)} shadow runs on {args.jobs} workers")
    rows = []
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for r in pool.map(job, tasks):
            rows += r["rows"]
            if r["rows"]:
                a = r["rows"][0]
                same = sum(1 for x in r["rows"] if x["same"])
                print(f"  {a['map']:16s} {a['start']:7s} driver={a['driver']:7s} "
                      f"{len(r['rows']):3d} decisions, meme choix {same:3d} "
                      f"[{r['wall_s']:.0f}s]")
    with open(args.out, "w") as fh:
        json.dump({"schema": "shadow_2830/1", "rows": rows}, fh)
    print(f"\n{len(rows)} decisions -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

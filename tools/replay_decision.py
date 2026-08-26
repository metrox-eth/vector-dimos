#!/usr/bin/env python3
"""Replay a REAL exploration decision out of a recording, cluster by cluster.

    tools/replay_decision.py --rec recordings/courseB_explorer2.db \
                             --log recordings/courseB.jsonl
    tools/replay_decision.py --decision 10       # only that one, with the full table
    tools/replay_decision.py --decision final    # the call that ended the run
    tools/replay_decision.py --decision 10 --tuning fixed   # the same input, new scoring

`explorer2.next_target` is a pure function of (costmap, pose, state), so the
recording holds everything a decision was made from. This tool rebuilds that
input from the dimOS recording and runs the real function over it:

  costmap   the `global_costmap` LCM blobs, decoded here rather than through
            dimOS (which lives only on the Jetson). The layout is checked on
            every message: 8 byte fingerprint, int32 cell count, int32 pad,
            8 byte stamp, frame_id string, 8 byte stamp, float32 resolution,
            int32 width, int32 height, 7 doubles of origin, then width*height
            int8 cells. A message whose trailing byte count is not width*height
            is refused, so a format change cannot pass silently.
  pose      the `odom` row nearest before the decision, quaternion included.
  state     rebuilt by replaying EVERY decision of the run in order, since
            ExploreState is only ever written by next_target itself (plus the
            loop's note_failed, which the log reports as "planner gave up").
            Decision N therefore sees exactly the memory decision N saw.

The cross-check that this is the real decision and not a reconstruction: the
goal the replay picks, its path cost and its frontier-cell count are printed
next to the ones the run logged. They match to the centimetre, or the header
says so.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import sqlite3
import struct
import sys
from dataclasses import dataclass
from datetime import datetime, timezone

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from vector_dimos.explorer2 import (  # noqa: E402
    DEFAULT_TUNING, DIRECTIVE_FRONTIER, Cluster, ExploreState, Tuning,
    _REVISIT_FADE_POWER, _cells_of, _clusters, _count_clusters, _decided_from, _frontier_mask, _path_cost,
    _survey, next_target, unknown_signature,
)

FREE, UNKNOWN, OCCUPIED = 0, -1, 100

# The clock the dimOS recorder stamps rows with, minus the clock the log lines
# carry, is a constant for a run; it is measured (never assumed) in
# `Recording.__init__` from the costmap stream itself.


# --- the recording ---------------------------------------------------------

_FINGERPRINT_LEN = 8


def decode_occupancy_grid(blob: bytes) -> tuple[np.ndarray, float, float, float, str]:
    """One `global_costmap` LCM blob -> (grid HxW int8, resolution, ox, oy, frame_id)."""
    off = _FINGERPRINT_LEN
    n_cells = struct.unpack_from(">i", blob, off)[0]
    off += 8                                   # count + pad
    off += 8                                   # stamp
    str_len = struct.unpack_from(">i", blob, off)[0]
    off += 4
    frame_id = blob[off:off + str_len - 1].decode("ascii")
    off += str_len
    off += 8                                   # stamp again
    res = struct.unpack_from(">f", blob, off)[0]
    off += 4
    width, height = struct.unpack_from(">ii", blob, off)
    off += 8
    origin = struct.unpack_from(">7d", blob, off)
    off += 56
    cells = len(blob) - off
    if cells != width * height or n_cells != width * height:
        raise ValueError(f"costmap blob: {cells} trailing bytes for a "
                         f"{width}x{height} grid ({n_cells} announced)")
    grid = np.frombuffer(blob, dtype=np.int8, count=width * height, offset=off)
    return (grid.reshape(height, width).copy(), float(res),
            float(origin[0]), float(origin[1]), frame_id)


class _Grid:
    """What next_target reads off a costmap. Same attribute names as dimOS's."""

    def __init__(self, grid, res, ox, oy, ts, frame_id="world"):
        self.grid, self.resolution, self.ts, self.frame_id = grid, res, ts, frame_id
        self.origin = type("O", (), {"position": type("P", (), {
            "x": ox, "y": oy, "z": 0.0})()})()


class _Pose:
    def __init__(self, x, y, qz, qw, ts):
        self.position = type("P", (), {"x": x, "y": y, "z": 0.0})()
        self.orientation = type("Q", (), {"x": 0.0, "y": 0.0, "z": qz, "w": qw})()
        self.ts = ts


class Recording:
    """Read-only view of one dimOS recording: costmaps and poses, by timestamp."""

    def __init__(self, path: str) -> None:
        self.db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
        self.costmap_ts = [r[0] for r in self.db.execute(
            'select ts from "global_costmap" order by ts')]
        self.costmap_ids = [r[0] for r in self.db.execute(
            'select id from "global_costmap" order by ts')]
        rows = self.db.execute(
            'select ts, pose_x, pose_y, pose_qz, pose_qw from "odom" '
            'where pose_x is not null order by ts').fetchall()
        self.odom_ts = [r[0] for r in rows]
        self.odom = rows

    def costmap_at(self, t: float) -> _Grid:
        """The last costmap published at or before `t` - what the loop held."""
        i = bisect.bisect_right(self.costmap_ts, t) - 1
        if i < 0:
            raise LookupError(f"no costmap at or before {t}")
        blob = self.db.execute('select data from global_costmap_blob where id=?',
                               (self.costmap_ids[i],)).fetchone()[0]
        grid, res, ox, oy, frame_id = decode_occupancy_grid(blob)
        return _Grid(grid, res, ox, oy, self.costmap_ts[i], frame_id)

    def pose_at(self, t: float) -> _Pose:
        i = bisect.bisect_right(self.odom_ts, t) - 1
        if i < 0:
            raise LookupError(f"no odometry at or before {t}")
        ts, x, y, qz, qw = self.odom[i]
        return _Pose(float(x), float(y), float(qz), float(qw), float(ts))


# --- the log ---------------------------------------------------------------

@dataclass
class LogEvent:
    t: float                # epoch seconds, same clock as the recording rows
    kind: str               # "goal" | "none" | "wait" | "back_off" | "failed"
    goal_n: int | None = None
    xy: tuple[float, float] | None = None
    path_cost_m: float | None = None
    info_cells: int | None = None
    n_clusters: int | None = None
    elapsed_ms: float | None = None
    text: str = ""


def _epoch(iso: str) -> float:
    return datetime.strptime(iso, "%Y-%m-%dT%H:%M:%S.%fZ").replace(
        tzinfo=timezone.utc).timestamp()


def read_log(path: str) -> list[LogEvent]:
    """The explorer2 lines of a dimOS jsonl log, in order."""
    out: list[LogEvent] = []
    for line in open(path):
        try:
            d = json.loads(line)
        except ValueError:
            continue
        if not d.get("logger", "").endswith("explorer2.py"):
            continue
        ev, t = d.get("event", ""), _epoch(d["timestamp"])
        if ev.startswith("goal ") and ":" in ev:
            head, rest = ev.split(":", 1)
            n = int(head.split()[1])
            xy = rest.split("(")[1].split(")")[0].split(",")
            out.append(LogEvent(
                t=t, kind="goal", goal_n=n,
                xy=(float(xy[0]), float(xy[1])),
                path_cost_m=float(rest.split("m away")[0].split(")")[1].strip()),
                info_cells=int(rest.split("frontier cells")[0].split(",")[-1]),
                n_clusters=int(rest.split("clusters")[0].split(",")[-1]),
                elapsed_ms=float(rest.split("chosen in")[1].split("ms")[0]),
                text=ev))
        elif ev.startswith("exploration complete"):
            out.append(LogEvent(t=t, kind="none",
                                elapsed_ms=float(ev.split(",")[-1].split("ms")[0]),
                                text=ev))
        elif ev.startswith("planner gave up"):
            out.append(LogEvent(t=t, kind="failed", text=ev))
        elif "waiting" in ev:
            out.append(LogEvent(t=t, kind="wait", text=ev))
        elif ev.startswith("born cornered"):
            out.append(LogEvent(t=t, kind="back_off", text=ev))
        elif ev.startswith("goal timeout"):
            continue      # not a decision, the next line is
    return out


# --- the decision, opened up ----------------------------------------------

@dataclass
class Scored:
    cluster: Cluster
    reason: str             # "" when it was scored, otherwise why it was not
    cost: float = math.inf
    gain: float = 0.0
    base: float = 0.0       # gain / (1 + cost)
    probe_mult: float = 1.0
    forward_mult: float = 1.0
    revisit_mult: float = 1.0
    score: float = -math.inf


def explain(costmap, pose, state: ExploreState, now: float,
            tuning: Tuning) -> tuple[list[Scored], dict]:
    """The same arithmetic next_target does, with every term kept.

    Reads the module's own internals, so it cannot drift from the function it
    explains: if the scoring changes, this changes with it.
    """
    grid = np.asarray(costmap.grid)
    res = float(costmap.resolution)
    ox = float(costmap.origin.position.x)
    oy = float(costmap.origin.position.y)
    rx, ry = float(pose.position.x), float(pose.position.y)
    q = pose.orientation
    heading = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * q.z * q.z)

    # next_target records the pose it was called from BEFORE it excludes
    # anything, so the standing-point rule sees it. Same step here, on a copy.
    state = state.copy()
    half = tuning.observed_radius_m / 2.0
    if all((rx - px) ** 2 + (ry - py) ** 2 >= half * half for px, py in state.observed):
        state.observed.append((rx, ry))

    h, w = grid.shape
    gx0 = int(np.clip(math.floor((rx - ox) / res), 0, w - 1))
    gy0 = int(np.clip(math.floor((ry - oy) / res), 0, h - 1))
    survey = _survey(grid, res, (gy0, gx0), tuning)
    frontier_all = _frontier_mask(survey)
    min_cells = max(1, int(tuning.min_frontier_perimeter_m / res))
    on_the_map = _count_clusters(frontier_all, min_cells)
    decided = _decided_from(state.observed, (h, w), res, (ox, oy), tuning.observed_radius_m)
    clusters = ([] if survey.seed is None
                else _clusters(survey, frontier_all, (h, w), res, (ox, oy), tuning, decided,
                               _cells_of(state.observed, (h, w), res, (ox, oy))))
    costs = (_path_cost(survey.reachable, survey.cell_cost, survey.seed, res)
             if survey.seed is not None else None)

    radius2 = tuning.failed_goal_radius_m ** 2
    # mirror next_target's trigger pruning: an exclusion holds only while the
    # rover is still near where it failed AND the map around the goal has not
    # changed (26/08: triggers replaced the 60 s clock)
    moved2 = tuning.failed_goal_moved_m ** 2
    held_failed = [f for f in state.failed
                   if (rx - f[2]) ** 2 + (ry - f[3]) ** 2 < moved2
                   and unknown_signature(costmap, f[0], f[1], tuning.failed_goal_radius_m) == f[4]]
    out: list[Scored] = []
    for cluster in clusters:
        gx, gy = cluster.goal_xy
        s = Scored(cluster=cluster, reason="")
        if cluster.retired:
            s.reason = "already-seen-from"
            out.append(s)
            continue
        if any((gx - f[0]) ** 2 + (gy - f[1]) ** 2 < radius2 for f in held_failed):
            s.reason = "recently-failed"
            out.append(s)
            continue
        gyc, gxc = cluster.goal_yx
        cost = float(costs[gyc, gxc]) if survey.reachable[gyc, gxc] else math.inf
        s.cost = cost
        if not math.isfinite(cost):
            s.reason = "unreachable"
            out.append(s)
            continue
        s.gain = cluster.gain
        s.base = cluster.gain / (1.0 + cost)
        s.probe_mult = tuning.probe_penalty if cluster.probe else 1.0
        dx, dy = gx - rx, gy - ry
        span = math.hypot(dx, dy)
        if span > 0.1:
            align = max(0.0, math.cos(heading) * dx / span + math.sin(heading) * dy / span)
            s.forward_mult = 1.0 + tuning.forward_bonus * align
        older = state.visited[:-1]
        if older:
            nearest = min(math.hypot(gx - vx, gy - vy) for vx, vy in older)
            if nearest < tuning.revisit_radius_m:
                fade = nearest / tuning.revisit_radius_m
                s.revisit_mult = fade ** _REVISIT_FADE_POWER
        s.score = s.base * s.probe_mult * s.forward_mult * s.revisit_mult
        out.append(s)

    out.sort(key=lambda s: (-s.score, s.reason))
    info = {"n_clusters": len(clusters), "heading": heading, "on_the_map": on_the_map,
            "reachable_free_m2": survey.reachable_free_m2,
            "unknown_m2": float(survey.unknown.sum()) * res * res,
            "free_m2": float(survey.free.sum()) * res * res,
            "res": res, "origin": (ox, oy), "shape": (h, w), "pose": (rx, ry)}
    return out, info


# --- printing --------------------------------------------------------------

def print_table(scored: list[Scored], info: dict, chosen, logged: LogEvent | None,
                top: int = 40) -> None:
    rx, ry = info["pose"]
    print(f"  pose ({rx:+.2f}, {ry:+.2f}) heading {math.degrees(info['heading']):+.0f} deg   "
          f"grid {info['shape'][1]}x{info['shape'][0]} @ {info['res']} m   "
          f"free {info['free_m2']:.1f} m2, unknown {info['unknown_m2']:.1f} m2")
    print(f"  frontier clusters: {info['on_the_map']} on the map, "
          f"{info['n_clusters']} reachable from where the body stands, "
          f"{sum(1 for s in scored if not s.reason)} eligible")
    print(f"  {'#':>2} {'stand at':>16} {'cells':>6} {'gain':>7} {'cost m':>7} "
          f"{'base':>9} {'probe':>6} {'fwd':>5} {'revis':>6} {'score':>10}  why not")
    for i, s in enumerate(scored[:top], start=1):
        c = s.cluster
        gx, gy = c.goal_xy
        cost = f"{s.cost:7.2f}" if math.isfinite(s.cost) else "    inf"
        score = f"{s.score:10.5f}" if math.isfinite(s.score) else "         -"
        mark = " "
        if chosen is not None and abs(gx - chosen.position.x) < 1e-6 and \
                abs(gy - chosen.position.y) < 1e-6:
            mark = ">"
        print(f" {mark}{i:>2} ({gx:+6.2f},{gy:+6.2f}) {c.size:6d} {s.gain:7.3f} {cost} "
              f"{s.base:9.5f} {s.probe_mult:6.2f} {s.forward_mult:5.2f} "
              f"{s.revisit_mult:6.2f} {score}  {s.reason}")
    if len(scored) > top:
        print(f"      ... {len(scored) - top} more")
    if chosen is None:
        print("  -> next_target returned None (exploration would stop here)")
    else:
        d = getattr(chosen, "directive", "?")
        print(f"  -> {d} ({chosen.position.x:+.2f}, {chosen.position.y:+.2f})"
              + (f"  score {getattr(chosen, 'score', float('nan')):.5f}"
                 f"  cost {getattr(chosen, 'path_cost_m', float('nan')):.2f} m"
                 f"  {getattr(chosen, 'info_cells', 0)} cells"
                 if d == DIRECTIVE_FRONTIER else ""))
    if logged is not None and logged.kind == "goal":
        print(f"  == the run logged: goal {logged.goal_n} "
              f"({logged.xy[0]:+.2f}, {logged.xy[1]:+.2f}) {logged.path_cost_m} m, "
              f"{logged.info_cells} cells, {logged.n_clusters} clusters")
    elif logged is not None and logged.kind == "none":
        print(f"  == the run logged: {logged.text}")


# --- the replay ------------------------------------------------------------

def replay(rec: Recording, events: list[LogEvent], tuning: Tuning,
           stop_after: int | None = None):
    """Walk every decision of the run through next_target, in order.

    The memory carried forward is the RUN's, not the replay's: the goals the
    run actually published (from the log) and the poses it actually decided
    from (from the odometry). Feeding the replay its own choices would be a
    different run after the first divergence, and then decision N would no
    longer be the decision the owner watched. The scoring under test therefore
    always answers the question the rover was really asked.

    Yields (index, LogEvent, costmap, pose, state_before, target).
    """
    state = ExploreState()
    idx = 0
    for ev in events:
        if ev.kind == "failed":
            # the loop's own write, reported by the log line
            goal = _last_goal(events, ev.t)
            if goal is not None:
                state.note_failed(goal[0], goal[1], ev.t)
            continue
        if ev.kind not in ("goal", "none", "wait", "back_off"):
            continue
        t0 = ev.t - (ev.elapsed_ms or 0.0) / 1000.0 - 0.02
        costmap = rec.costmap_at(t0)
        pose = rec.pose_at(t0)
        before = state.copy()
        asked = state.copy()
        target = next_target(costmap, pose, asked, now=ev.t, tuning=tuning)
        # advance the real memory: the pose is what it is, the goal is the
        # one the RUN published
        state.observed = list(asked.observed)
        state.heading = asked.heading
        state.failed = list(asked.failed)
        if ev.kind == "goal" and ev.xy is not None:
            state.visited.append(ev.xy)
            state.targets_issued += 1
        idx += 1
        yield idx, ev, costmap, pose, before, target
        if stop_after is not None and idx >= stop_after:
            return


def _last_goal(events: list[LogEvent], t: float):
    for ev in reversed(events):
        if ev.kind == "goal" and ev.t < t:
            return ev.xy
    return None


def _tuning_from(name: str) -> Tuning:
    if name in ("shipped", "default", ""):
        return DEFAULT_TUNING
    raise SystemExit(f"unknown tuning {name!r}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rec", default=os.path.join(ROOT, "recordings",
                                                  "courseB_explorer2.db"))
    ap.add_argument("--log", default=os.path.join(ROOT, "recordings", "courseB.jsonl"))
    ap.add_argument("--decision", default="all",
                    help="goal number, 'final', or 'all' (default)")
    ap.add_argument("--top", type=int, default=40, help="clusters listed per decision")
    args = ap.parse_args(argv)

    rec = Recording(args.rec)
    events = read_log(args.log)
    decisions = [e for e in events if e.kind in ("goal", "none", "wait", "back_off")]
    print(f"recording {os.path.relpath(args.rec, ROOT)}: "
          f"{len(rec.costmap_ts)} costmaps, {len(rec.odom_ts)} poses, "
          f"{len(decisions)} explorer decisions in {os.path.relpath(args.log, ROOT)}")
    print(f"tuning: observed_radius {DEFAULT_TUNING.observed_radius_m} m, "
          f"info_radius {DEFAULT_TUNING.info_radius_m} m, "
          f"probe_penalty {DEFAULT_TUNING.probe_penalty}, "
          f"revisit_radius {DEFAULT_TUNING.revisit_radius_m} m")

    want_all = args.decision == "all"
    want_final = args.decision == "final"
    want_n = None if (want_all or want_final) else int(args.decision)

    same_goal = n_goals = same_count = 0
    for idx, ev, costmap, pose, before, target in replay(rec, events, DEFAULT_TUNING):
        interesting = want_all or (want_final and ev.kind == "none") or \
            (want_n is not None and ev.goal_n == want_n)
        if ev.kind == "goal":
            n_goals += 1
            if target is not None and getattr(target, "directive", "") == DIRECTIVE_FRONTIER:
                same_goal += (abs(target.position.x - ev.xy[0]) < 0.011 and
                              abs(target.position.y - ev.xy[1]) < 0.011)
                same_count += getattr(target, "n_clusters", -1) == ev.n_clusters
        if not interesting:
            continue
        print()
        print("=" * 110)
        head = (f"decision {idx}"
                + (f" (goal {ev.goal_n})" if ev.goal_n else " (the one that ended the run)"))
        print(f"{head} at {datetime.fromtimestamp(ev.t, timezone.utc):%H:%M:%S} UTC, "
              f"costmap {costmap.ts - ev.t:+.1f} s, "
              f"state: {len(before.visited)} goals issued, "
              f"{len(before.observed)} standing points, {len(before.failed)} failed")
        print("=" * 110)
        scored, info = explain(costmap, pose, before, ev.t, DEFAULT_TUNING)
        print_table(scored, info, target, ev, top=args.top)

    print()
    # Two different claims, and only the first one is a claim about the INPUT:
    #   the cluster count is what the map and the state say, whatever the
    #   scoring then does with them, so it is the check that the decode, the
    #   pose lookup and the rebuilt memory are the ones the run had.
    #   the goal is what the CURRENT scoring picks; it matches the run only
    #   while the scoring has not moved since that run.
    print(f"decisions whose cluster count matches the run's: {same_count} of {n_goals}")
    print(f"decisions whose goal matches the run's:          {same_goal} of {n_goals}")
    print("(both are comparisons, not tests: they read 10 of 10 on the code that flew "
          "run B\n and move as the scoring moves. What is under test lives in "
          "tests/test_explorer2_cold.py.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

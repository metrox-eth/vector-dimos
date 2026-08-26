#!/usr/bin/env python3
"""Score and compare recorded VECTOR exploration runs.

Reads a VectorMemory SQLite recording (the ``explore*.db`` files the rover
writes on the Jetson), prints a text scorecard and writes ``<db>.score.json``.
Optionally folds in the counts of a dimOS ``main.jsonl`` run log.

Recording schema (read off the real recordings, not guessed):

    _streams(name TEXT PRIMARY KEY, config TEXT NOT NULL)
    <stream>(id INTEGER PK, ts REAL, value NUMERIC,
             pose_x, pose_y, pose_z, pose_qx, pose_qy, pose_qz, pose_qw,
             tags BLOB)
    <stream>_blob(id INTEGER PK, data BLOB)      -- LCM-encoded payload
    <stream>_rtree*                              -- spatial index, unused here

Streams written by the explore blueprint: ``odom`` (PoseStamped),
``lidar`` / ``camera_floor`` (PointCloud2), ``global_costmap``
(OccupancyGrid), ``tf`` (TFMessage).  The pose columns of the ``odom`` rows
already hold the robot pose, so the trajectory needs no payload decoding; the
costmap does, and is decoded here straight from its LCM bytes.

Dependencies: standard library only (sqlite3, struct, json, math).  numpy is
used for grid counting when it happens to be installed, never required, so the
tool runs under a bare system python3.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import re
import shutil
import sqlite3
import struct
import subprocess
import sys
import time

try:  # optional, only ever a speed-up
    import numpy as _np
except ImportError:  # pragma: no cover - depends on the interpreter used
    _np = None

# --- where the runs live -----------------------------------------------------

JETSON = "metrox@192.168.0.56"
REMOTE_RECORDINGS = ".local/state/vector/recordings"  # relative to the remote home
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LOCAL_RECORDINGS = os.path.join(REPO_ROOT, "recordings")

# --- metric parameters (physical units) --------------------------------------

JUMP_M = 0.30          # a single odom step longer than this is a relocalisation
                       # jump, not motion: excluded from path length
                       # (same rule as tools/traj_rec.py on the Jetson)
WINDOW_S = 1.0         # speed averaging window
MOVING_MPS = 0.03      # a window at or above this counts as "moving"
CHORD_M = 0.05         # trajectory is resampled into chords of this length
                       # before headings are computed: this is what keeps
                       # standstill odom noise from inventing headings
REVERSAL_DEG = 120.0   # course change above this is a reversal
REVERSAL_S = 3.0       # ... only if it happens in less than this

# --- dimOS log events --------------------------------------------------------
# Substring matched against the "event" field of each JSONL record.
#
# Two explorers write these logs and they do not speak the same dialect, so a
# table scored on the v1 patterns alone showed a v2 run as all zeros - no
# goals, no completion - which is exactly the wrong thing for an A/B:
#
#   v1  fast_explorer / the stock wavefront module (25-26/08)
#       "Published frontier goal: (x, y)" and "Exploration complete ..."
#   v2  vector_dimos.explorer2 (26/08 onwards)
#       "goal 7: (0.43, 5.83) 5.5 m away, 42 frontier cells, ..." and
#       "exploration complete: no reachable frontier left (10 targets, 26 ms)"
#
# The two completions are NOT the same event and must never be added up.
# v1's is the self-stop this rewrite exists to remove (three of them in one
# night with ten valid clusters on the map); v2's is the only way it can end -
# it says the map holds nothing reachable left to look at. So v1's keeps its
# own row, and v2's gets a row of its own: "clean termination".

EVENT_PATTERNS = [
    ("goals_published", "Published frontier goal"),
    ("goals_arrived", "Arrived"),
    ("no_path_found", "No path found"),
    ("slips", "SLIP #"),
    ("map_rollbacks", "rolled back"),
    ("exploration_complete", "Exploration complete"),
]

# explorer2's own lines. `goals_published` is deliberately the SAME counter as
# v1's: a run is one explorer or the other for its whole life (see
# explorer2.explorer_v2_enabled), so the two can never both fire in one log.
# ("no path found" is NOT here: the global planner logs that itself, in both
# dialects, and explorer2's "planner gave up on that goal" is the same refusal
# reported a second time - counting both would double every one of them.)
V2_EVENT_PATTERNS = [
    ("clean_termination", "exploration complete: no reachable frontier left"),
    ("goal_timeouts", "goal timeout after"),
    ("waits", "clusters are on a recently failed goal"),
    ("back_offs", "born cornered"),
]

# "goal 12: (-2.87, 0.68) 3.4 m away, ..." - the number is what tells it apart
# from every other line that starts with the word "goal".
V2_GOAL_RE = re.compile(r"^goal \d+: \(")
V2_LOGGER = "explorer2.py"

SCHEMA_VERSION = "vector_bench_run/1"


# =============================================================================
# recording access
# =============================================================================

def open_recording(path):
    """Open a recording read-only (never touch a run we are scoring)."""
    if not os.path.exists(path):
        raise SystemExit(f"no such recording: {path}")
    return sqlite3.connect(f"file:{os.path.abspath(path)}?mode=ro", uri=True)


def list_streams(conn):
    """Return {stream_name: config dict} from the _streams table."""
    out = {}
    try:
        rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    except sqlite3.Error:
        return out
    for name, config in rows:
        try:
            out[name] = json.loads(config)
        except (TypeError, ValueError):
            out[name] = {}
    return out


def find_stream(streams, payload_needle, fallback):
    """Find a stream by the payload class it records, e.g. "PoseStamped"."""
    for name, config in sorted(streams.items()):
        if payload_needle in str(config.get("payload_module", "")):
            return name
    return fallback if fallback in streams else None


def read_poses(conn, stream):
    """Return [(ts, x, y, yaw_rad)] ordered by time, NULL poses dropped."""
    rows = conn.execute(
        f'SELECT ts, pose_x, pose_y, pose_qx, pose_qy, pose_qz, pose_qw '
        f'FROM "{stream}" ORDER BY ts'
    ).fetchall()
    poses = []
    for ts, x, y, qx, qy, qz, qw in rows:
        if ts is None or x is None or y is None:
            continue
        qx, qy = qx or 0.0, qy or 0.0
        qz, qw = qz or 0.0, qw or 1.0
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        poses.append((float(ts), float(x), float(y), yaw))
    return poses


# =============================================================================
# trajectory metrics
# =============================================================================

def _cumulative_distance(poses):
    """Cumulative path length sampled at each pose timestamp.

    Returns (timestamps, cumulative_metres, jumps_rejected).  Steps longer than
    JUMP_M contribute 0 m: they are odometry teleports, not travel.
    """
    times = [poses[0][0]]
    cumulative = [0.0]
    jumps = 0
    for i in range(1, len(poses)):
        step = math.hypot(poses[i][1] - poses[i - 1][1],
                          poses[i][2] - poses[i - 1][2])
        if step > JUMP_M:
            jumps += 1
            step = 0.0
        times.append(poses[i][0])
        cumulative.append(cumulative[-1] + step)
    return times, cumulative, jumps


def _distance_at(times, cumulative, t):
    """Linearly interpolate the cumulative distance at an arbitrary time."""
    if t <= times[0]:
        return cumulative[0]
    if t >= times[-1]:
        return cumulative[-1]
    i = bisect.bisect_right(times, t) - 1
    if i >= len(times) - 1:
        return cumulative[-1]
    span = times[i + 1] - times[i]
    if span <= 0.0:
        return cumulative[i]
    frac = (t - times[i]) / span
    return cumulative[i] + frac * (cumulative[i + 1] - cumulative[i])


def _window_speeds(times, cumulative):
    """Mean speed over each whole WINDOW_S window, in m/s.

    Distance is interpolated at the window edges, so the result does not depend
    on where the odom samples happen to fall.
    """
    duration = times[-1] - times[0]
    n_windows = int(math.floor(duration / WINDOW_S + 1e-9))
    if n_windows < 1:
        # recording shorter than one window: report the whole thing as one
        if duration <= 0.0:
            return [], True
        return [(cumulative[-1] - cumulative[0]) / duration], True
    speeds = []
    for w in range(n_windows):
        a = times[0] + w * WINDOW_S
        b = a + WINDOW_S
        speeds.append((_distance_at(times, cumulative, b)
                       - _distance_at(times, cumulative, a)) / WINDOW_S)
    return speeds, False


def _chords(poses):
    """Resample the trajectory into chords of at least CHORD_M.

    Returns [(t_start, t_end, course_rad)].  Working on fixed-length chords
    rather than raw samples is what makes the course meaningful: a rover that
    only jitters in place never accumulates CHORD_M, so it produces no chord
    and therefore no heading at all.
    """
    chords = []
    if len(poses) < 2:
        return chords
    at, ax, ay = poses[0][0], poses[0][1], poses[0][2]
    for ts, x, y, _yaw in poses[1:]:
        dx, dy = x - ax, y - ay
        length = math.hypot(dx, dy)
        if length >= CHORD_M:
            if length <= JUMP_M:  # a teleport is not a heading
                chords.append((at, ts, math.atan2(dy, dx)))
            at, ax, ay = ts, x, y
    return chords


def _angle_delta_deg(a, b):
    """Signed smallest angle from a to b, in degrees."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return math.degrees(d)


def count_reversals(poses):
    """Course changes above REVERSAL_DEG happening in under REVERSAL_S."""
    chords = _chords(poses)
    reversals = 0
    for (a_start, a_end, a_course), (b_start, b_end, b_course) in zip(chords, chords[1:]):
        a_mid = 0.5 * (a_start + a_end)
        b_mid = 0.5 * (b_start + b_end)
        if (b_mid - a_mid) >= REVERSAL_S:
            continue
        if abs(_angle_delta_deg(a_course, b_course)) > REVERSAL_DEG:
            reversals += 1
    return reversals


def trajectory_metrics(poses):
    """All odometry-derived metrics, in SI units."""
    if len(poses) < 2:
        return {
            "n_poses": len(poses),
            "duration_s": 0.0,
            "path_length_m": 0.0,
            "jumps_rejected": 0,
            "mean_speed_mps": 0.0,
            "max_speed_mps": 0.0,
            "moving_fraction": 0.0,
            "n_windows": 0,
            "bbox_w_m": 0.0,
            "bbox_h_m": 0.0,
            "bbox_diag_m": 0.0,
            "bbox_area_m2": 0.0,
            "thrash_index": None,
            "reversals": 0,
        }

    times, cumulative, jumps = _cumulative_distance(poses)
    path_length = cumulative[-1]
    duration = times[-1] - times[0]

    speeds, partial = _window_speeds(times, cumulative)
    mean_speed = sum(speeds) / len(speeds) if speeds else 0.0
    max_speed = max(speeds) if speeds else 0.0
    moving = sum(1 for s in speeds if s >= MOVING_MPS)
    moving_fraction = moving / len(speeds) if speeds else 0.0

    xs = [p[1] for p in poses]
    ys = [p[2] for p in poses]
    width = max(xs) - min(xs)
    height = max(ys) - min(ys)
    diagonal = math.hypot(width, height)
    thrash = (path_length / diagonal) if diagonal > 1e-6 else None

    return {
        "n_poses": len(poses),
        "duration_s": duration,
        "path_length_m": path_length,
        "jumps_rejected": jumps,
        "mean_speed_mps": mean_speed,
        "max_speed_mps": max_speed,
        "moving_fraction": moving_fraction,
        "n_windows": 0 if partial else len(speeds),
        "bbox_w_m": width,
        "bbox_h_m": height,
        "bbox_diag_m": diagonal,
        "bbox_area_m2": width * height,
        "thrash_index": thrash,
        "reversals": count_reversals(poses),
    }


# =============================================================================
# coverage: nav_msgs/OccupancyGrid decoded straight from its LCM bytes
# =============================================================================
#
# Wire layout, from dimos_lcm.nav_msgs.OccupancyGrid._encode_one and the nested
# messages it delegates to (all big-endian):
#
#   8  fingerprint
#   4  data_length                       (int32)
#      header:   4 seq, 4+4 stamp sec/nsec,
#                4 frame_id length (includes the trailing NUL), frame_id bytes
#      info:     4+4 map_load_time, 4 resolution (float32),
#                4 width, 4 height,
#                24 origin position (3 doubles), 32 origin orientation
#   N  data_length signed bytes, row-major, -1 unknown / 0 free / 1..100 cost

GRID_HEADER_FIXED = 8 + 4 + 4 + 8 + 4     # up to and including frame_id length
GRID_INFO_BYTES = 8 + 4 + 4 + 4 + 24 + 32


def decode_occupancy_grid(blob):
    """Decode an LCM OccupancyGrid payload. Returns a dict of grid facts."""
    if len(blob) < GRID_HEADER_FIXED:
        raise ValueError("blob too short for an OccupancyGrid")
    (data_length,) = struct.unpack_from(">i", blob, 8)
    (frame_len,) = struct.unpack_from(">I", blob, 24)
    if frame_len < 1 or frame_len > 256:
        raise ValueError(f"implausible frame_id length {frame_len}")
    offset = GRID_HEADER_FIXED + frame_len
    frame_id = blob[GRID_HEADER_FIXED:offset - 1].decode("utf-8", "replace")

    resolution, width, height = struct.unpack_from(">fii", blob, offset + 8)
    origin_x, origin_y, _origin_z = struct.unpack_from(">3d", blob, offset + 20)

    cells_at = offset + GRID_INFO_BYTES
    cells = blob[cells_at:cells_at + data_length]
    if len(cells) != data_length:
        raise ValueError(f"truncated grid: {len(cells)} of {data_length} cells")
    if width * height != data_length:
        raise ValueError(f"grid {width}x{height} does not match {data_length} cells")

    if _np is not None:
        grid = _np.frombuffer(cells, dtype=_np.int8)
        unknown = int((grid == -1).sum())
        free = int((grid == 0).sum())
    else:
        unknown = cells.count(0xFF)   # -1 as an unsigned byte
        free = cells.count(0x00)

    seen = data_length - unknown
    return {
        "frame_id": frame_id,
        "resolution_m": float(resolution),
        "width": width,
        "height": height,
        "origin_x_m": origin_x,
        "origin_y_m": origin_y,
        "cells_total": data_length,
        "cells_seen": seen,
        "cells_free": free,
        "cells_occupied": seen - free,
    }


def coverage_metrics(conn, streams, traj):
    """Cells seen from the last costmap; bbox area as a labelled proxy if none."""
    stream = find_stream(streams, "OccupancyGrid", "global_costmap")
    if stream:
        try:
            row = conn.execute(
                f'SELECT b.data FROM "{stream}" g '
                f'JOIN "{stream}_blob" b ON b.id = g.id '
                f'ORDER BY g.ts DESC LIMIT 1'
            ).fetchone()
        except sqlite3.Error:
            row = None
        if row and row[0]:
            try:
                grid = decode_occupancy_grid(row[0])
            except (ValueError, struct.error) as exc:
                return {
                    "source": "bbox_proxy",
                    "note": f"costmap stream '{stream}' unreadable ({exc}); "
                            f"trajectory bbox area used as a proxy",
                    "area_m2": traj["bbox_area_m2"],
                }
            area = grid["cells_seen"] * grid["resolution_m"] ** 2
            return {
                "source": f"costmap:{stream}",
                "note": "",
                "resolution_m": grid["resolution_m"],
                "grid_w": grid["width"],
                "grid_h": grid["height"],
                "cells_total": grid["cells_total"],
                "cells_seen": grid["cells_seen"],
                "cells_free": grid["cells_free"],
                "cells_occupied": grid["cells_occupied"],
                "area_m2": area,
            }
    return {
        "source": "bbox_proxy",
        "note": "no costmap/grid stream in this recording; "
                "trajectory bbox area used as a proxy",
        "area_m2": traj["bbox_area_m2"],
    }


# =============================================================================
# dimOS run log
# =============================================================================

def parse_log(path):
    """Count the events of interest in a dimOS main.jsonl run log.

    Both explorer dialects are counted (see EVENT_PATTERNS): the goal counters
    add up, because one log only ever holds one of them, and the two
    completions keep separate rows because they mean opposite things.
    """
    counts = {key: 0 for key, _ in EVENT_PATTERNS}
    counts.update({key: 0 for key, _ in V2_EVENT_PATTERNS})
    lines = 0
    unparsed = 0
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for raw in handle:
            raw = raw.strip()
            if not raw:
                continue
            try:
                record = json.loads(raw)
            except ValueError:
                unparsed += 1
                continue
            lines += 1
            event = record.get("event")
            if not isinstance(event, str):
                continue
            for key, pattern in EVENT_PATTERNS:
                if pattern in event:
                    counts[key] += 1
            for key, pattern in V2_EVENT_PATTERNS:
                if pattern in event:
                    counts[key] += 1
            logger = record.get("logger")
            if V2_GOAL_RE.match(event) and (
                    not isinstance(logger, str) or logger.endswith(V2_LOGGER)):
                counts["goals_published"] += 1
    counts["lines_parsed"] = lines
    counts["lines_unparsed"] = unparsed
    return counts


# =============================================================================
# scoring
# =============================================================================

def score_recording(db_path, log_path=None):
    conn = open_recording(db_path)
    try:
        streams = list_streams(conn)
        odom = find_stream(streams, "PoseStamped", "odom")
        if odom is None:
            raise SystemExit(
                f"{db_path}: no pose stream found; streams present: "
                f"{sorted(streams) or 'none'}"
            )
        poses = read_poses(conn, odom)
        traj = trajectory_metrics(poses)
        coverage = coverage_metrics(conn, streams, traj)
    finally:
        conn.close()

    return {
        "schema": SCHEMA_VERSION,
        "db": os.path.abspath(db_path),
        "log": os.path.abspath(log_path) if log_path else None,
        "generated_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "pose_stream": odom,
        "streams": sorted(streams),
        "trajectory": traj,
        "coverage": coverage,
        "log_events": parse_log(log_path) if log_path else None,
    }


def _fmt(value, spec):
    if value is None:
        return "n/a"
    return format(value, spec)


def print_scorecard(score):
    traj = score["trajectory"]
    cov = score["coverage"]
    line = "=" * 72
    print(line)
    print("VECTOR run scorecard")
    print(f"  recording : {score['db']}")
    print(f"  log       : {score['log'] or '(none)'}")
    print(line)

    print(f"TRAJECTORY  (stream '{score['pose_stream']}', {traj['n_poses']} poses)")
    print(f"  duration                        {traj['duration_s']:9.1f} s"
          f"   ({traj['duration_s'] / 60.0:.1f} min)")
    print(f"  path length                     {traj['path_length_m']:9.2f} m")
    print(f"  odom jumps rejected (>{JUMP_M:.2f} m)  {traj['jumps_rejected']:9d}")
    print(f"  mean speed / {WINDOW_S:.0f} s window        {traj['mean_speed_mps']:9.3f} m/s"
          f"   ({traj['n_windows']} windows)")
    print(f"  max speed / {WINDOW_S:.0f} s window         {traj['max_speed_mps']:9.3f} m/s")
    print(f"  moving windows (>={MOVING_MPS:.2f} m/s)    {traj['moving_fraction'] * 100:9.1f} %")
    print(f"  bounding box                    {traj['bbox_w_m']:9.2f} x {traj['bbox_h_m']:.2f} m"
          f"   (diag {traj['bbox_diag_m']:.2f} m, area {traj['bbox_area_m2']:.2f} m2)")
    print(f"  thrash index (path/diag)        {_fmt(traj['thrash_index'], '9.2f')}")
    print(f"  reversals (>{REVERSAL_DEG:.0f} deg in <{REVERSAL_S:.0f} s)   {traj['reversals']:9d}")

    print("COVERAGE")
    print(f"  source                          {cov['source']}")
    if cov.get("note"):
        print(f"  note                            {cov['note']}")
    if "cells_seen" in cov:
        print(f"  grid                            {cov['grid_w']} x {cov['grid_h']} cells"
              f" @ {cov['resolution_m']:.3f} m")
        print(f"  cells seen                      {cov['cells_seen']:9d}"
              f" / {cov['cells_total']} ({cov['cells_free']} free,"
              f" {cov['cells_occupied']} occupied)")
    print(f"  area seen                       {cov['area_m2']:9.2f} m2")

    events = score["log_events"]
    if events is None:
        print("LOG EVENTS                        (no --log given)")
    else:
        print(f"LOG EVENTS  ({events['lines_parsed']} lines"
              f"{', %d unparsed' % events['lines_unparsed'] if events['lines_unparsed'] else ''})")
        for key, pattern in EVENT_PATTERNS:
            print(f"  {key:<30}  {events[key]:9d}   \"{pattern}\"")
        for key, pattern in V2_EVENT_PATTERNS:
            print(f"  {key:<30}  {events.get(key, 0):9d}   \"{pattern}\"")
    print(line)


# =============================================================================
# compare
# =============================================================================

COMPARE_ROWS = [
    ("duration", "s", ("trajectory", "duration_s"), "10.1f"),
    ("path length", "m", ("trajectory", "path_length_m"), "10.2f"),
    ("mean speed (1 s win)", "m/s", ("trajectory", "mean_speed_mps"), "10.3f"),
    ("max speed (1 s win)", "m/s", ("trajectory", "max_speed_mps"), "10.3f"),
    ("moving windows", "%", ("trajectory", "moving_fraction"), "10.1f"),
    ("bbox width", "m", ("trajectory", "bbox_w_m"), "10.2f"),
    ("bbox height", "m", ("trajectory", "bbox_h_m"), "10.2f"),
    ("bbox diagonal", "m", ("trajectory", "bbox_diag_m"), "10.2f"),
    ("bbox area", "m2", ("trajectory", "bbox_area_m2"), "10.2f"),
    ("thrash index", "", ("trajectory", "thrash_index"), "10.2f"),
    ("reversals", "n", ("trajectory", "reversals"), "10.0f"),
    ("odom jumps rejected", "n", ("trajectory", "jumps_rejected"), "10.0f"),
    ("poses recorded", "n", ("trajectory", "n_poses"), "10.0f"),
    ("cells seen", "n", ("coverage", "cells_seen"), "10.0f"),
    ("area seen", "m2", ("coverage", "area_m2"), "10.2f"),
    ("goals published", "n", ("log_events", "goals_published"), "10.0f"),
    ("goals arrived", "n", ("log_events", "goals_arrived"), "10.0f"),
    ("no path found", "n", ("log_events", "no_path_found"), "10.0f"),
    ("slips", "n", ("log_events", "slips"), "10.0f"),
    ("map rollbacks", "n", ("log_events", "map_rollbacks"), "10.0f"),
    ("exploration complete", "n", ("log_events", "exploration_complete"), "10.0f"),
    ("clean termination", "n", ("log_events", "clean_termination"), "10.0f"),
    ("goal timeouts", "n", ("log_events", "goal_timeouts"), "10.0f"),
    ("waits (not a stop)", "n", ("log_events", "waits"), "10.0f"),
    ("back-offs", "n", ("log_events", "back_offs"), "10.0f"),
]


def _dig(score, path):
    node = score
    for key in path:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    return node


COL = 20


def _short_label(label):
    """Shrink 'explore.20260825212935.db' to the run id that identifies it."""
    for prefix in ("explore.",):
        if label.startswith(prefix):
            label = label[len(prefix):]
    if label.endswith(".db"):
        label = label[:-3]
    return label if len(label) <= COL else label[-COL:]


def print_comparison(score_a, score_b, label_a, label_b):
    label_a, label_b = _short_label(label_a), _short_label(label_b)
    line = "-" * (26 + 5 + 2 * COL + 12 + 4)
    print(line)
    print(f"{'metric':<26} {'unit':<5} {label_a:>{COL}} {label_b:>{COL}} {'delta':>12}")
    print(line)
    for label, unit, path, spec in COMPARE_ROWS:
        a = _dig(score_a, path)
        b = _dig(score_b, path)
        if a is None and b is None:
            continue
        if unit == "%":
            a = a * 100.0 if a is not None else None
            b = b * 100.0 if b is not None else None
        if a is None or b is None:
            delta = "n/a"
        else:
            delta = format(b - a, "+" + spec).strip()
        cell_a = _fmt(a, spec).strip() if a is not None else "n/a"
        cell_b = _fmt(b, spec).strip() if b is not None else "n/a"
        print(f"{label:<26} {unit:<5} {cell_a:>{COL}} {cell_b:>{COL}} {delta:>12}")
    print(line)
    print(f"A = {score_a.get('db', label_a)}")
    print(f"B = {score_b.get('db', label_b)}")
    print(line)


# =============================================================================
# fetch
# =============================================================================

def ensure_gitignore():
    """Make sure the recordings directory stays out of git."""
    path = os.path.join(REPO_ROOT, ".gitignore")
    entry = "recordings/"
    existing = []
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as handle:
            existing = [ln.strip() for ln in handle]
    if entry in existing:
        return False
    with open(path, "a", encoding="utf-8") as handle:
        if existing and existing[-1] != "":
            handle.write("\n")
        handle.write(entry + "\n")
    return True


def cmd_fetch(name):
    if shutil.which("scp") is None:
        raise SystemExit("scp not found on this machine")
    os.makedirs(LOCAL_RECORDINGS, exist_ok=True)
    if ensure_gitignore():
        print("added 'recordings/' to .gitignore")
    remote = f"{JETSON}:{REMOTE_RECORDINGS}/{name}"
    local = os.path.join(LOCAL_RECORDINGS, os.path.basename(name))
    print(f"fetching {remote}")
    result = subprocess.run(["scp", "-p", remote, local])
    if result.returncode != 0:
        raise SystemExit(f"scp failed with code {result.returncode}")
    size = os.path.getsize(local)
    print(f"wrote {local} ({size / 1e6:.1f} MB)")
    return local


# =============================================================================
# CLI
# =============================================================================

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Score and compare recorded VECTOR exploration runs.")
    parser.add_argument("db", nargs="?", help="path to a recording (explore*.db)")
    parser.add_argument("--log", help="dimOS run log (main.jsonl) for the same run")
    parser.add_argument("--fetch", metavar="NAME.DB",
                        help=f"scp {JETSON}:{REMOTE_RECORDINGS}/NAME.DB "
                             f"into {LOCAL_RECORDINGS}/")
    parser.add_argument("--compare", nargs=2, metavar=("A.SCORE.JSON", "B.SCORE.JSON"),
                        help="print a side-by-side table of two scorecards")
    args = parser.parse_args(argv)

    if args.fetch:
        cmd_fetch(args.fetch)
        return 0

    if args.compare:
        path_a, path_b = args.compare
        with open(path_a, encoding="utf-8") as handle:
            score_a = json.load(handle)
        with open(path_b, encoding="utf-8") as handle:
            score_b = json.load(handle)
        print_comparison(score_a, score_b,
                         os.path.basename(path_a).replace(".score.json", ""),
                         os.path.basename(path_b).replace(".score.json", ""))
        return 0

    if not args.db:
        parser.error("give a recording to score, or use --fetch / --compare")

    score = score_recording(args.db, args.log)
    print_scorecard(score)
    out = args.db + ".score.json"
    with open(out, "w", encoding="utf-8") as handle:
        json.dump(score, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

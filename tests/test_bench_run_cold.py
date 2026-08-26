"""Cold tests for tools/bench_run.py - known input, known output, SI units.

No robot, no Jetson, no network: every fixture is built here.  Rule #2 applies
throughout - a check only counts if a value we chose going in comes back out in
physical units (metres, seconds, m/s), never merely "it ran and returned a
number of the right shape".

The trajectory fixtures are sampled every 0.10 m, i.e. deliberately coarser
than bench_run.CHORD_M (0.05 m), so each recorded sample is exactly one chord.
That removes any float-boundary ambiguity in the resampling and makes the
expected reversal counts exact rather than approximate.
"""
import json
import math
import sqlite3
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import bench_run  # noqa: E402

FAIL = 0


def check(label, ok, detail=""):
    global FAIL
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if not ok:
        FAIL = 1


def close(value, expected, tol):
    return value is not None and abs(value - expected) <= tol


# --- fixtures ---------------------------------------------------------------

# Real recording DDL, copied from an explore*.db written by VectorMemory.
STREAM_DDL = """
CREATE TABLE "{name}" (
  id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, value NUMERIC,
  pose_x REAL, pose_y REAL, pose_z REAL,
  pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
  tags BLOB DEFAULT (jsonb('{{}}')))
"""
BLOB_DDL = 'CREATE TABLE "{name}_blob" (id INTEGER PRIMARY KEY, data BLOB NOT NULL)'

ODOM_CONFIG = json.dumps(
    {"payload_module": "dimos.msgs.geometry_msgs.PoseStamped.PoseStamped",
     "codec_id": "lcm"})
COSTMAP_CONFIG = json.dumps(
    {"payload_module": "dimos.msgs.nav_msgs.OccupancyGrid.OccupancyGrid",
     "codec_id": "lcm"})

# First 110 bytes of a real global_costmap blob (run explore.20260825212935).
# Kept verbatim so the decoder is checked against bytes dimOS actually wrote,
# not only against a fixture we encoded ourselves.
REAL_GRID_HEADER = bytes.fromhex(
    "e7dfd179cdfc3b6500016e9e00000000"
    "6a8da6b40106773200000006776f726c"
    "64006a8da6b4010677323d4ccccd0000"
    "014e00000119c01cccccccccccccc01a"
    "999999999999000000000000000000000000000000000000"
    "00000000000000000000000000003ff0000000000000"
)
REAL_GRID_CELLS = 93854      # data_length in that header (334 x 281)
REAL_GRID_RES = 0.05         # m per cell
REAL_GRID_W, REAL_GRID_H = 334, 281
REAL_GRID_ORIGIN = (-7.2, -6.65)

T0 = 1787000000.0            # a realistic epoch base, as in the real recordings
STEP_M = 0.10                # sample spacing along the path
SPEED_MPS = 0.5              # constant speed of the synthetic runs
DT_S = STEP_M / SPEED_MPS    # 0.2 s between samples


def walk(start, legs):
    """Sample a polyline every STEP_M. legs = [(dx, dy, length_m), ...]."""
    x, y = start
    points = [(x, y)]
    for ux, uy, length in legs:
        for _ in range(int(round(length / STEP_M))):
            x += ux * STEP_M
            y += uy * STEP_M
            points.append((x, y))
    return points


def poses_of(points):
    """(ts, x, y, yaw) samples for a polyline walked at SPEED_MPS."""
    return [(T0 + i * DT_S, x, y, 0.0) for i, (x, y) in enumerate(points)]


def make_db(path, points, costmap_body=None):
    """Write a recording with the real schema holding the given trajectory."""
    conn = sqlite3.connect(str(path))
    conn.execute("CREATE TABLE _streams (name TEXT PRIMARY KEY, config TEXT NOT NULL)")
    conn.execute(STREAM_DDL.format(name="odom"))
    conn.execute(BLOB_DDL.format(name="odom"))
    conn.execute("INSERT INTO _streams VALUES ('odom', ?)", (ODOM_CONFIG,))
    for i, (x, y) in enumerate(points):
        conn.execute(
            'INSERT INTO "odom" (ts, pose_x, pose_y, pose_z, '
            'pose_qx, pose_qy, pose_qz, pose_qw) VALUES (?,?,?,?,?,?,?,?)',
            (T0 + i * DT_S, x, y, 0.37, 0.0, 0.0, 0.0, 1.0))
    if costmap_body is not None:
        conn.execute(STREAM_DDL.format(name="global_costmap"))
        conn.execute(BLOB_DDL.format(name="global_costmap"))
        conn.execute("INSERT INTO _streams VALUES ('global_costmap', ?)",
                     (COSTMAP_CONFIG,))
        conn.execute('INSERT INTO "global_costmap" (ts, pose_x, pose_y) VALUES (?,?,?)',
                     (T0 + 1.0, 0.0, 0.0))
        conn.execute('INSERT INTO "global_costmap_blob" (id, data) VALUES (1, ?)',
                     (REAL_GRID_HEADER + costmap_body,))
    conn.commit()
    conn.close()


# =============================================================================

print("A. square, 1 m side, constant 0.5 m/s (known trajectory -> known metres)")
square = walk((0.0, 0.0), [(1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0)])
check("41 samples (4 x 1 m at 0.10 m)", len(square) == 41, f"{len(square)}")
m = bench_run.trajectory_metrics(poses_of(square))
check("path length = 4.000 m", close(m["path_length_m"], 4.0, 0.04),
      f"{m['path_length_m']:.4f} m")
check("bbox = 1.00 x 1.00 m",
      close(m["bbox_w_m"], 1.0, 0.01) and close(m["bbox_h_m"], 1.0, 0.01),
      f"{m['bbox_w_m']:.3f} x {m['bbox_h_m']:.3f} m")
check("bbox diagonal = sqrt(2) m", close(m["bbox_diag_m"], math.sqrt(2.0), 0.014),
      f"{m['bbox_diag_m']:.4f} m")
check("thrash index = 4 / sqrt(2) = 2.828",
      close(m["thrash_index"], 4.0 / math.sqrt(2.0), 0.028), f"{m['thrash_index']:.4f}")
check("duration = 8.0 s (4 m at 0.5 m/s)", close(m["duration_s"], 8.0, 0.08),
      f"{m['duration_s']:.3f} s")
check("mean speed = 0.500 m/s", close(m["mean_speed_mps"], 0.5, 0.005),
      f"{m['mean_speed_mps']:.4f} m/s")
check("max speed = 0.500 m/s", close(m["max_speed_mps"], 0.5, 0.005),
      f"{m['max_speed_mps']:.4f} m/s")
check("8 full 1 s windows", m["n_windows"] == 8, f"{m['n_windows']}")
check("all windows moving", close(m["moving_fraction"], 1.0, 1e-9),
      f"{m['moving_fraction']:.2f}")
check("no odom jumps", m["jumps_rejected"] == 0, f"{m['jumps_rejected']}")
# The true expected count: a square turns 90 deg at each of its 4 corners, and
# 90 deg is NOT above the 120 deg reversal threshold. A clean square therefore
# scores ZERO reversals - four corners are four turns, not four reversals.
check("reversals = 0 (four 90 deg corners, threshold is 120 deg)",
      m["reversals"] == 0, f"{m['reversals']}")

print("B. shuttle, 3 x 1 m back and forth (the detector must actually fire)")
shuttle = walk((0.0, 0.0), [(1, 0, 1.0), (-1, 0, 1.0), (1, 0, 1.0)])
ms = bench_run.trajectory_metrics(poses_of(shuttle))
check("path length = 3.000 m", close(ms["path_length_m"], 3.0, 0.03),
      f"{ms['path_length_m']:.4f} m")
check("bbox = 1.00 x 0.00 m",
      close(ms["bbox_w_m"], 1.0, 0.01) and close(ms["bbox_h_m"], 0.0, 1e-9),
      f"{ms['bbox_w_m']:.3f} x {ms['bbox_h_m']:.3f} m")
check("thrash index = 3.00 (3 m over a 1 m diagonal)",
      close(ms["thrash_index"], 3.0, 0.03), f"{ms['thrash_index']:.4f}")
check("reversals = 2 (two 180 deg turnarounds)", ms["reversals"] == 2,
      f"{ms['reversals']}")

print("C. standstill with +/-5 mm odom noise (no invented reversals)")
import random  # noqa: E402
rng = random.Random(20260825)
jitter = [(T0 + i * 0.1,
           rng.uniform(-0.005, 0.005), rng.uniform(-0.005, 0.005), 0.0)
          for i in range(600)]           # 60 s at 10 Hz, exactly where it sits
mj = bench_run.trajectory_metrics(jitter)
check("noise really is present (path > 0)", mj["path_length_m"] > 0.1,
      f"{mj['path_length_m']:.3f} m")
check("bbox stays under 2 cm",
      mj["bbox_w_m"] < 0.02 and mj["bbox_h_m"] < 0.02,
      f"{mj['bbox_w_m']:.4f} x {mj['bbox_h_m']:.4f} m")
check("reversals = 0 (no 5 cm chord can form inside 1 cm of jitter)",
      mj["reversals"] == 0, f"{mj['reversals']}")
check("max 1 s speed under 0.10 m/s", mj["max_speed_mps"] < 0.10,
      f"{mj['max_speed_mps']:.4f} m/s")

print("D. odometry teleport is not travel (5 m jump rejected)")
jumped = square + [(5.0, 5.0), (5.1, 5.0)]
mt = bench_run.trajectory_metrics(poses_of(jumped))
check("1 jump rejected", mt["jumps_rejected"] == 1, f"{mt['jumps_rejected']}")
check("path = 4.10 m (square + the one real 0.10 m step)",
      close(mt["path_length_m"], 4.10, 0.041), f"{mt['path_length_m']:.4f} m")

print("E. OccupancyGrid decoded from real dimOS header bytes")
# Known body: 10 000 free cells, 5 000 occupied, the rest unknown.
body = (b"\x00" * 10000) + (b"\x64" * 5000) + (b"\xff" * (REAL_GRID_CELLS - 15000))
check("body is the length the real header announces", len(body) == REAL_GRID_CELLS,
      f"{len(body)}")
grid = bench_run.decode_occupancy_grid(REAL_GRID_HEADER + body)
check("frame_id = 'world'", grid["frame_id"] == "world", grid["frame_id"])
check(f"grid = {REAL_GRID_W} x {REAL_GRID_H} cells",
      grid["width"] == REAL_GRID_W and grid["height"] == REAL_GRID_H,
      f"{grid['width']} x {grid['height']}")
check("resolution = 0.050 m/cell", close(grid["resolution_m"], REAL_GRID_RES, 1e-6),
      f"{grid['resolution_m']:.4f} m")
check("origin = (-7.20, -6.65) m",
      close(grid["origin_x_m"], REAL_GRID_ORIGIN[0], 1e-6)
      and close(grid["origin_y_m"], REAL_GRID_ORIGIN[1], 1e-6),
      f"({grid['origin_x_m']:.3f}, {grid['origin_y_m']:.3f})")
check("cells seen = 15 000", grid["cells_seen"] == 15000, f"{grid['cells_seen']}")
check("10 000 free / 5 000 occupied",
      grid["cells_free"] == 10000 and grid["cells_occupied"] == 5000,
      f"{grid['cells_free']} / {grid['cells_occupied']}")

print("F. end to end on a recording with the real schema")
with tempfile.TemporaryDirectory(prefix="bench_run_cold_") as tmp:
    tmp = Path(tmp)
    with_map = tmp / "explore.withmap.db"
    make_db(with_map, square, costmap_body=body)
    score = bench_run.score_recording(str(with_map))
    check("pose stream found via _streams", score["pose_stream"] == "odom",
          str(score["pose_stream"]))
    check("41 poses read back", score["trajectory"]["n_poses"] == 41,
          str(score["trajectory"]["n_poses"]))
    check("path length = 4.000 m through the db",
          close(score["trajectory"]["path_length_m"], 4.0, 0.04),
          f"{score['trajectory']['path_length_m']:.4f} m")
    cov = score["coverage"]
    check("coverage source = costmap", cov["source"] == "costmap:global_costmap",
          cov["source"])
    check("cells seen = 15 000", cov["cells_seen"] == 15000, str(cov["cells_seen"]))
    # 15 000 cells at 0.05 m -> 15 000 * 0.0025 = 37.5 m2
    check("area seen = 37.50 m2", close(cov["area_m2"], 37.5, 0.001),
          f"{cov['area_m2']:.3f} m2")

    no_map = tmp / "explore.nomap.db"
    make_db(no_map, square, costmap_body=None)
    score2 = bench_run.score_recording(str(no_map))
    check("no costmap -> proxy, clearly labelled",
          score2["coverage"]["source"] == "bbox_proxy"
          and "proxy" in score2["coverage"]["note"],
          score2["coverage"]["source"])
    check("proxy area = bbox area = 1.00 m2",
          close(score2["coverage"]["area_m2"], 1.0, 0.02),
          f"{score2['coverage']['area_m2']:.3f} m2")

    print("G. run log parser (3 goals, 1 no-path, 2 slips)")
    log = tmp / "main.jsonl"
    records = [
        {"event": "Building the blueprint", "timestamp": "2026-08-25T14:49:32Z"},
        {"event": "Published frontier goal: (1.83, 2.91)"},
        {"event": "No path found to the goal."},
        {"event": "Published frontier goal: (0.10, -2.00)"},
        {"event": "SLIP #1: wheels 0.10 m, lidar 0.02 m in 1 s"},
        {"event": "slip: rolled back the last 2 s of map writes (34 batches)"},
        {"event": "Cancelling goal."},
        {"event": "Published frontier goal: (3.00, 1.00)"},
        {"event": "SLIP #2: wheels 0.12 m, lidar 0.01 m in 1 s"},
        {"event": "Arrived at goal."},
        {"event": "Exploration complete after 3 goals"},
        {"level": "info", "no_event_field": True},
    ]
    with open(log, "w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")
        handle.write("this line is not json\n")     # must be counted, not fatal

    events = bench_run.parse_log(str(log))
    check("goals published = 3", events["goals_published"] == 3,
          str(events["goals_published"]))
    check("no path found = 1", events["no_path_found"] == 1,
          str(events["no_path_found"]))
    check("slips = 2", events["slips"] == 2, str(events["slips"]))
    check("map rollbacks = 1", events["map_rollbacks"] == 1,
          str(events["map_rollbacks"]))
    check("goals arrived = 1", events["goals_arrived"] == 1,
          str(events["goals_arrived"]))
    check("exploration complete = 1", events["exploration_complete"] == 1,
          str(events["exploration_complete"]))
    check("12 json lines parsed", events["lines_parsed"] == 12,
          str(events["lines_parsed"]))
    check("1 malformed line survived", events["lines_unparsed"] == 1,
          str(events["lines_unparsed"]))

    # --- the explorer2 dialect: the same questions, different words ---------
    print("H. run log parser, explorer2 dialect (4 goals, 1 clean termination)")
    log2 = Path(tmp) / "v2.jsonl"
    records = [
        {"event": "Building the blueprint", "logger": "dimos/core/x.py"},
        {"event": "goal 1: (-2.87, 0.58) 3.2 m away, 11 frontier cells, 11 clusters,"
                  " chosen in 575 ms", "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "goal timeout after 45 s, re-deciding",
         "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "goal 2: (-1.22, 4.48) 3.6 m away, 122 frontier cells, 9 clusters,"
                  " chosen in 158 ms", "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "No path found to the goal.", "logger": "dimos/navigation/global_planner.py"},
        {"event": "planner gave up on that goal: excluded for 60 s",
         "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "3 of 11 clusters are on a recently failed goal: waiting 12.0 s, not stopping",
         "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "born cornered: 0.22 m2 of reachable floor and no frontier - one"
                  " 0.22 m back-off via the bump reflex",
         "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "goal 3: (2.28, 4.48) 6.4 m away, 280 frontier cells, 11 clusters,"
                  " chosen in 468 ms", "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "Arrived at goal.", "logger": "dimos/navigation/global_planner.py"},
        {"event": "goal 4: (0.43, 5.83) 5.5 m away, 42 frontier cells, 12 clusters,"
                  " chosen in 154 ms", "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        {"event": "exploration complete: no reachable frontier left (4 targets, 26 ms)",
         "logger": "/home/metrox/vector-dimos/vector_dimos/explorer2.py"},
        # a line that starts with the word "goal" and is NOT one
        {"event": "goal request published on world frame",
         "logger": "dimos/navigation/global_planner.py"},
    ]
    with open(log2, "w", encoding="utf-8") as handle:
        for record in records:
            handle.write(json.dumps(record) + "\n")

    v2 = bench_run.parse_log(str(log2))
    check("goals published = 4 (from explorer2's own lines)", v2["goals_published"] == 4,
          str(v2["goals_published"]))
    check("clean termination = 1", v2["clean_termination"] == 1, str(v2["clean_termination"]))
    check("v1's 'Exploration complete' stays 0: it is a different event",
          v2["exploration_complete"] == 0, str(v2["exploration_complete"]))
    check("no path found = 1, counted once and not twice", v2["no_path_found"] == 1,
          str(v2["no_path_found"]))
    check("goal timeouts = 1", v2["goal_timeouts"] == 1, str(v2["goal_timeouts"]))
    check("waits = 1 (a wait is not a stop)", v2["waits"] == 1, str(v2["waits"]))
    check("back-offs = 1", v2["back_offs"] == 1, str(v2["back_offs"]))
    check("goals arrived = 1", v2["goals_arrived"] == 1, str(v2["goals_arrived"]))

    # and the v1 fixture must not have grown a v2 event
    check("a v1 log reports no clean termination", events["clean_termination"] == 0,
          str(events["clean_termination"]))

sys.exit(FAIL)

"""Cold tests for tools/replay_decision.py - known log lines in, known state out.

Rule #2: a check counts only if a value chosen going in comes back in physical
units (metres, cells), never "it ran". Two things are pinned here, both of them
crashes the tool used to take on real runs:

  A. read_log routing   - explorer2 emits "goal timeout after 45 s having closed
                          0.10 m of 3.20 m: excluded ..." on the timeout that
                          made no progress. It starts with "goal " and carries a
                          colon, so the goal parser used to swallow it and try
                          int("timeout"). It is not a goal: it is the loop's own
                          note_failed, and it must come back as one.
  B. replay memory      - the failed-goal exclusion must be rebuilt with the
                          arguments explorer2._note_failed passes: the goal, the
                          point the rover STOOD at when it failed, and the
                          costmap it held (whose unknown count is the reopening
                          trigger). Called any other way it is a TypeError.

The log fixture carries explorer2's exact wording; the recording is a synthetic
free map with two unknown patches of known size, so both signatures are counts
we chose.

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_replay_decision_cold.py
"""
import json
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import replay_decision as rd  # noqa: E402
from vector_dimos.explorer2 import DEFAULT_TUNING  # noqa: E402

FREE, UNKNOWN = 0, -1
RES = 0.05

FAIL = 0


def check(label, ok, detail=""):
    global FAIL
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if not ok:
        FAIL = 1


# --- fixtures ---------------------------------------------------------------

LOGGER = "/home/dimos/vector-dimos/vector_dimos/explorer2.py"

# One run: a goal, a timeout that closed nothing (-> excluded), a second goal,
# a timeout that DID close ground (-> not a failure), a planner refusal, the end.
LINES = [
    ("2026-08-28T10:00:00.000000Z",
     "goal 1: (1.00, 2.00) 3.2 m away, 11 frontier cells, 5 clusters, chosen in 100 ms"),
    ("2026-08-28T10:00:45.000000Z",
     "goal timeout after 45 s having closed 0.10 m of 3.20 m: excluded until "
     "the map or the viewpoint changes"),
    ("2026-08-28T10:00:46.000000Z",
     "goal 2: (3.00, 1.00) 2.1 m away, 7 frontier cells, 4 clusters, chosen in 120 ms"),
    ("2026-08-28T10:01:31.000000Z",
     "goal timeout after 45 s, 1.20 m closer, re-deciding"),
    ("2026-08-28T10:01:35.000000Z",
     "planner gave up on that goal: excluded until the map or the viewpoint changes"),
    ("2026-08-28T10:01:36.000000Z",
     "exploration complete: no reachable frontier left (2 targets, 26 ms)"),
]

T_EXCLUDED = rd._epoch(LINES[1][0])     # the timeout that excluded goal 1
T_GAVE_UP = rd._epoch(LINES[4][0])      # the refusal that excluded goal 2

# Where the rover stood, in metres, at each stretch of the run.
STOOD_EARLY = (0.10, 0.20)
STOOD_AT_TIMEOUT = (0.75, 0.25)
STOOD_AT_REFUSAL = (2.50, 1.30)


def write_log(path: Path) -> None:
    with open(path, "w") as fh:
        # a line from another logger, and a line that is not json: both ignored
        fh.write("not json at all\n")
        fh.write(json.dumps({"logger": "/x/local_planner.py", "event": "goal 9: nonsense",
                             "timestamp": LINES[0][0]}) + "\n")
        for ts, event in LINES:
            fh.write(json.dumps({"logger": LOGGER, "event": event, "timestamp": ts}) + "\n")


def build_grid():
    """10 m x 10 m free map, origin (0, 0), with two unknown patches of known size."""
    grid = np.full((200, 200), FREE, dtype=np.int8)
    grid[40:45, 20:24] = UNKNOWN       # 20 cells, inside 0.6 m of (1.00, 2.00)
    grid[18:21, 58:62] = UNKNOWN       # 12 cells, inside 0.6 m of (3.00, 1.00)
    return grid


class FakeRecording:
    """costmap_at / pose_at, the only two things replay() asks a Recording for."""

    def __init__(self):
        self.grid = build_grid()

    def costmap_at(self, t):
        return rd._Grid(self.grid, RES, 0.0, 0.0, t)

    def pose_at(self, t):
        if t >= T_GAVE_UP:
            x, y = STOOD_AT_REFUSAL
        elif t >= T_EXCLUDED:
            x, y = STOOD_AT_TIMEOUT
        else:
            x, y = STOOD_EARLY
        return rd._Pose(x, y, 0.0, 1.0, t)


# --- A. read_log routing ----------------------------------------------------

print("A. read_log: an excluded timeout is a failure, not a goal")

with tempfile.TemporaryDirectory() as tmp:
    log = Path(tmp) / "run.jsonl"
    write_log(log)
    events = rd.read_log(str(log))

check("6 explorer2 lines in, 5 events out (the 're-deciding' timeout is not one)",
      len(events) == 5, str(len(events)))
check("kinds in order: goal, failed, goal, failed, none",
      [e.kind for e in events] == ["goal", "failed", "goal", "failed", "none"],
      str([e.kind for e in events]))
check("the excluded timeout is the failure at 10:00:45",
      events[1].kind == "failed" and events[1].t == T_EXCLUDED,
      events[1].text[:40])

g = events[0]
check("goal 1 parsed: n=1, (1.00, 2.00) m, 3.2 m, 11 cells, 5 clusters, 100 ms",
      (g.goal_n == 1 and g.xy == (1.00, 2.00) and g.path_cost_m == 3.2
       and g.info_cells == 11 and g.n_clusters == 5 and g.elapsed_ms == 100.0),
      f"{g.goal_n} {g.xy} {g.path_cost_m} {g.info_cells} {g.n_clusters} {g.elapsed_ms}")
check("goal 2 parsed: (3.00, 1.00) m, 2.1 m away",
      events[2].xy == (3.00, 1.00) and events[2].path_cost_m == 2.1,
      f"{events[2].xy} {events[2].path_cost_m}")


# --- B. replay rebuilds the exclusion the way the loop wrote it -------------

print("B. replay: note_failed carries the goal, the standing point and the map")

rec = FakeRecording()
states = [before for _, _, _, _, before, _ in rd.replay(rec, events, DEFAULT_TUNING)]

check("3 decisions replayed (2 goals + the end of the run)", len(states) == 3,
      str(len(states)))
check("decision 1 saw no exclusion yet", states[0].failed == [], str(states[0].failed))
check("decision 2 saw exactly the one exclusion the timeout wrote",
      len(states[1].failed) == 1, str(len(states[1].failed)))

if states[1].failed:
    x, y, sx, sy, sig = states[1].failed[0]
    check("excluded goal = (1.00, 2.00) m", (x, y) == (1.00, 2.00), f"({x}, {y})")
    check("stood at (0.75, 0.25) m - where the rover was at the TIMEOUT, "
          "not where it decided", (sx, sy) == STOOD_AT_TIMEOUT, f"({sx}, {sy})")
    check("unknown signature = 20 cells around that goal", sig == 20, str(sig))

if len(states) > 2 and len(states[2].failed) > 1:
    x, y, sx, sy, sig = states[2].failed[1]
    check("the planner refusal excludes goal 2 = (3.00, 1.00) m",
          (x, y) == (3.00, 1.00), f"({x}, {y})")
    check("stood at (2.50, 1.30) m at the refusal", (sx, sy) == STOOD_AT_REFUSAL,
          f"({sx}, {sy})")
    check("unknown signature = 12 cells around goal 2", sig == 12, str(sig))
else:
    check("the last decision carries both exclusions", False,
          str([s.failed for s in states]))

sys.exit(FAIL)

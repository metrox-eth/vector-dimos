"""Cold tests for tools/explore_sim.py - known input, known output, in m2.

No robot, no map file from the rig: the checkpoint is built here, so every
area printed by the harness is a number we chose going in.

The fixture is a 20x20 grid at 0.10 m = 4.00 m2 of grid, of which the "real
run" observed a 10x10 block = 1.00 m2 (0.10 m2 of it obstacle, 0.90 m2 free).
Those three numbers are far enough apart that the report can only quote the
right one.
"""
import contextlib
import io
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import explore_sim  # noqa: E402

FAIL = 0

RES = 0.10
N = 20
GRID_M2 = 4.00      # the whole 20x20 grid
OBSERVED_M2 = 1.00  # the 10x10 block the run saw
FREE_M2 = 0.90      # that block minus the obstacle strip


def check(label, ok, detail=""):
    global FAIL
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if not ok:
        FAIL = 1


def write_map(path):
    """A ScoredGrid checkpoint with a 1.00 m2 observed island in a 4.00 m2 grid."""
    lidar = np.zeros((N, N), dtype=np.int16)
    low = np.zeros((N, N), dtype=np.int16)
    seen = np.zeros((N, N), dtype=bool)
    seen[5:15, 5:15] = True                    # 100 cells = 1.00 m2 observed
    lidar[5, 5:15] = 10                        # 10 cells >= OCCUPIED_AT = 0.10 m2 wall
    np.savez(path, lidar=lidar, low=low, seen=seen, res=RES, ox=0.0, oy=0.0, n=N,
             pose_xy=np.array([1.0, 1.0]))
    return str(path)


with tempfile.TemporaryDirectory() as tmp:
    map_path = write_map(Path(tmp) / "synthetic.npz")

    print("closed arena (the default: unknown is a wall)")
    world = explore_sim.load_world(map_path, None)
    check(f"observed area = {OBSERVED_M2:.2f} m2",
          abs(world.observed_area_m2 - OBSERVED_M2) < 1e-9, f"{world.observed_area_m2:.2f}")
    check(f"free floor = {FREE_M2:.2f} m2",
          abs(world.free_area_m2 - FREE_M2) < 1e-9, f"{world.free_area_m2:.2f}")

    # The pre-fix expression, kept as the counter-example: once unknown_is_wall
    # has filled the map, no cell is UNKNOWN, so it measures the grid.
    stale = float((world.truth != explore_sim.UNKNOWN).sum()) * RES * RES
    check(f"'truth != UNKNOWN' is the whole {GRID_M2:.2f} m2 grid once the arena is closed",
          abs(stale - GRID_M2) < 1e-9, f"{stale:.2f}")
    check("so it is NOT the observed area", abs(stale - OBSERVED_M2) > 1.0,
          f"{stale:.2f} vs {OBSERVED_M2:.2f}")

    print("open arena (--unknown-open)")
    op = explore_sim.load_world(map_path, None, unknown_is_wall=False)
    check(f"observed area is the same {OBSERVED_M2:.2f} m2 either way",
          abs(op.observed_area_m2 - OBSERVED_M2) < 1e-9, f"{op.observed_area_m2:.2f}")
    check("unknown cells survive here", (op.truth == explore_sim.UNKNOWN).any())

    print("the report line")
    empty = explore_sim.Run("empty")
    summary = explore_sim.summarise(empty)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        explore_sim.print_report(summary, summary, empty, empty, world)
    line = next((ln for ln in out.getvalue().splitlines() if "ever observed" in ln), "")
    check(f"report quotes {OBSERVED_M2:.1f} m2 ever observed",
          f"{OBSERVED_M2:.1f} m2 ever observed" in line, line.strip())
    check(f"report does not quote the {GRID_M2:.1f} m2 grid",
          f"{GRID_M2:.1f} m2 ever observed" not in line, line.strip())

sys.exit(FAIL)

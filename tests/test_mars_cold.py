"""Cold bench for tools/mars (audit 2026-08-28): the raw-lidar stage draws what
the lidar actually saw, and the guard reads its camera mount from
lidar_odometry instead of keeping a copy.

Both scripts are loaded for real, with make_transport replaced by canned
streams (no rover, no bus). Groups:

  A. stage 1, the documented run - 12 s = 120 revolutions at 10 Hz: a cell hit
     4x per revolution reads '4' and a cell hit once reads '1'. The old view
     clamped the SUM to 99 BEFORE dividing by 120, so every cell floored to 0
     and the whole grid was dots - the tool accused the one stage that was fine.
  B. stage 1, 20 revolutions x 25 hits on one cell -> '#' (>= 10 per rev). The
     old view read 99 // 20 = '4'.
  C. the guard's mount IS lidar_odometry.CAMERA_XYZ_BASE (rear mast: -0.20 m,
     0.56 m up), not the dead front-bumper copy (+0.30 m, 0.80 m up).
  D. a depth wall read at 2.00 m from the camera -> the guard prints C = 1.80 m
     and ahead = 1.80 m in the ROVER frame. The copy printed 2.30 m: half a
     metre of phantom clearance in front of a rover that trusts this line.

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_mars_cold.py
"""

import importlib.util
import io
import sys
import time
from contextlib import redirect_stdout
from pathlib import Path

import numpy as np

import dimos.core.transport_factory as transport_factory

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from vector_dimos.lidar_odometry import CAMERA_XYZ_BASE  # noqa: E402

MARS = ROOT / "tools" / "mars"
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


class Cloud:
    """PointCloud2 stand-in: as_numpy() -> (points, ...) like the real one."""

    def __init__(self, pts):
        self.pts = np.asarray(pts, dtype=np.float32)

    def as_numpy(self):
        return (self.pts,)


class Frame:
    """Image stand-in: .data is the raw array, .to_opencv() the same."""

    def __init__(self, data):
        self.data = data

    def to_opencv(self):
        return self.data


class Info:
    def __init__(self, K):
        self.K = K


def load(script, feeds, argv, on_sleep=None):
    """Exec a tools/mars script with canned streams. Each subscribe is served
    its stream at subscribe time; on_sleep(subs) runs where the script blocks."""
    subs = {}

    class Fake:
        def __init__(self, name):
            self.name = name

        def subscribe(self, cb):
            subs[self.name] = cb
            if self.name in feeds:
                cb(feeds[self.name])
            return self

    real_make, real_argv, real_sleep = transport_factory.make_transport, sys.argv, time.sleep
    transport_factory.make_transport = lambda name, *a, **k: Fake(name)
    sys.argv = argv
    time.sleep = (lambda _s: on_sleep(subs)) if on_sleep else (lambda _s: None)
    spec = importlib.util.spec_from_file_location(f"mars_{Path(script).stem}", MARS / script)
    mod = importlib.util.module_from_spec(spec)
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            spec.loader.exec_module(mod)
    except SystemExit:                      # the scripts exit when a later stage is missing
        pass
    finally:
        transport_factory.make_transport, sys.argv, time.sleep = real_make, real_argv, real_sleep
    return mod, buf.getvalue()


def stage1(revs, cells):
    """Run stages.py over `revs` revolutions, each carrying `cells` = {(x, y): n
    points}. Returns the printed stage-1 grid as a list of row strings."""
    rev = Cloud([[x, y, 0.0] for (x, y), n in cells.items() for _ in range(n)])

    def feed(subs):
        for _ in range(revs):
            subs["pointcloud"](rev)

    _, out = load("stages.py", {}, ["stages.py"], on_sleep=feed)
    lines = out.splitlines()
    head = next(i for i, ln in enumerate(lines) if ln.startswith("--- 1. RAW LIDAR"))
    return [ln.split(" ", 1)[1] for ln in lines[head + 1:head + 31]]


print("tools/mars - the raw stage shows the leg, the guard knows where the camera is")

# --- A. the documented run: 12 s at 10 Hz = 120 revolutions -------------------
# A leg 0.95 m ahead, 5 cm to the right -> cell (row 5, col 15), 4 returns per
# revolution; a faint cell 45 cm ahead-left -> (10, 10), 1 return per revolution.
grid = stage1(120, {(0.95, -0.05): 4, (0.45, 0.45): 1})
check("120 revolutions, 4 hits/rev on the leg cell -> '4'", grid[5][15] == "4", f"got {grid[5][15]!r}")
check("... 1 hit/rev stays visible -> '1'", grid[10][10] == "1", f"got {grid[10][10]!r}")
check("... and the empty cells stay empty", grid[2][2] == "." and grid[20][25] == ".")
check("... the rover marker is still at the centre", grid[15][15] == "R", f"got {grid[15][15]!r}")

# --- B. the clamp used to corrupt the average even on a short run -------------
grid = stage1(20, {(0.95, -0.05): 25})
check("20 revolutions, 25 hits/rev -> '#' (>= 10), not the clamped '4'",
      grid[5][15] == "#", f"got {grid[5][15]!r}")

# --- C + D. the guard's camera mount ------------------------------------------
# Depth field: 5.00 m everywhere (so the frame is valid), a 2.00 m patch of 8
# sampled pixels straight ahead = a wall/leg at 2.00 m FROM THE CAMERA.
depth = np.full((480, 640), 5000, dtype=np.uint16)
depth[296:312, 320:328] = 2000
K = [386.6, 0, 320.6, 0, 386.0, 245.0, 0, 0, 1]
mod, out = load("sense.py",
                {"depth_image": Frame(depth), "camera_info": Info(K),
                 "pointcloud": Cloud([[2.5, 0.0, 0.0]])},
                ["sense.py", "bench", "0"])
check("mount x = CAMERA_XYZ_BASE[0] (rear mast, no local copy)",
      mod.CAM_X == CAMERA_XYZ_BASE[0], f"{mod.CAM_X} vs {CAMERA_XYZ_BASE[0]}")
check("mount height = CAMERA_XYZ_BASE[2]",
      mod.CAM_H == CAMERA_XYZ_BASE[2], f"{mod.CAM_H} vs {CAMERA_XYZ_BASE[2]}")
line = next((ln for ln in out.splitlines() if ln.startswith("guard ")), "")
fields = dict(kv.split("=") for kv in line.replace("guard ", "").split() if "=" in kv)
check("depth read 2.00 m -> guard C = 1.80 m in the rover frame",
      fields.get("C") == "1.80", f"got C={fields.get('C')} in {line!r}")
check("... so ahead = 1.80 m (the lidar sees 2.50 m)",
      fields.get("ahead") == "1.80", f"got ahead={fields.get('ahead')} in {line!r}")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

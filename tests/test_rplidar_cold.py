"""Cold bench: lidar polar->xyz maths (pure) + the retry loop with no sensor.

Three sections:
  A. polar_to_xy / scan_to_points - known measures -> known metres.
  B. the module's retry loop driven by a FAKE rplidar lib: sensor absent ->
     sensor appears -> sensor yanked mid-scan -> sensor back.
  C. the real rplidar lib against a port that does not exist, if installed.


Run with the robot's dimos stack STOPPED: a live stack on the same LCM bus
starves the bench module's pub/sub and section B reads zero clouds (24/08).
"""
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos import rplidar_c1 as rp
from vector_dimos.rplidar_c1 import (DEFAULT_BAUDRATE, RPLidarC1, polar_to_xy,
                                     scan_to_points)

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and bool(cond)


def close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def wait_for(predicate, timeout=6.0, tick=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(tick)
    return bool(predicate())


class LogSpy:
    """Records what the module logs (dimOS's logger is structlog, not stdlib)."""

    def __init__(self):
        self.lines = []

    def _record(self, msg, *args, **kwargs):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = _record

    def count(self, needle):
        return sum(needle in line for line in self.lines)


# --- A. polar -> xyz, known values ----------------------------------------
# The lib yields (quality, angle_deg in [0,360), distance_mm); we map with
# x = d*cos(theta), y = d*sin(theta), so 0 deg -> +X and 90 deg -> +Y.
print("A. polar_to_xy / scan_to_points (no lib, no sensor)")

x, y = polar_to_xy(90.0, 2000.0)
check(close(x, 0.0) and close(y, 2.0),   # body flipped 24/08: raw 90 deg (old right) = the NEW left
      f"90 deg (clockwise), 2000 mm -> (0.0, 2.0) m, the robot's right  (got ({x:.9f}, {y:.9f}))")
x, y = polar_to_xy(0.0, 1500.0)
check(close(x, -1.5) and close(y, 0.0), "0 deg raw, 1500 mm -> (-1.5, 0.0) m, the NEW tail")
x, y = polar_to_xy(180.0, 1000.0)
check(close(x, 1.0) and close(y, 0.0), "180 deg raw, 1000 mm -> (+1.0, 0.0) m, the bumper")
x, y = polar_to_xy(270.0, 500.0)
check(close(x, 0.0) and close(y, -0.5), "270 deg raw, 500 mm -> (0.0, -0.5) m, the NEW right")
x, y = polar_to_xy(45.0, 1414.213562)
check(close(x, -1.0) and close(y, 1.0), "45 deg raw, 1414.2136 mm -> (-1.0, +1.0) m")
x, y = polar_to_xy(30.0, 4000.0)
check(close(x, -3.464101615) and close(y, 2.0),
      "30 deg raw, 4000 mm -> (-3.4641016, +2.0) m")

# One canned revolution: 3 keepers, one weak return, one invalid (distance 0).
SCAN = [(15, 0.0, 1000.0),      # kept  -> ( 1.0,  0.0)
        (15, 90.0, 2000.0),     # kept  -> ( 0.0,  2.0)
        (5, 180.0, 3000.0),     # dropped: quality 5 < 10
        (20, 270.0, 500.0),     # kept  -> ( 0.0, -0.5)
        (18, 45.0, 0.0)]        # dropped: distance 0 = invalid measure
EXPECTED = [(-1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, -0.5, 0.0)]   # body flipped 24/08: raw 0 deg = new tail
EXPECTED_ALL = [(-1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (3.0, 0.0, 0.0), (0.0, -0.5, 0.0)]   # body FLIPPED 24/08: raw 0 deg = new tail   # + the weak (quality 5) return at 180 deg, kept by the default

points = scan_to_points(SCAN, min_quality=10)
check(len(points) == 3, f"min_quality=10 keeps 3 of 5 measures (got {len(points)})")
check(all(close(g, e) for gp_, ep in zip(points, EXPECTED)
          for g, e in zip(gp_, ep)),
      f"points = {[tuple(round(v, 3) for v in p) for p in points]} m")
check(all(p[2] == 0.0 for p in points), "every point is flat: z = 0.0")
# At min_quality=16 only the quality-20 measure survives: the quality-18 one
# is the invalid (distance 0) return, dropped on distance regardless.
check(len(scan_to_points(SCAN, min_quality=16)) == 1,
      "raising min_quality to 16 keeps only the quality-20 return")
check(scan_to_points([], 10) == [], "an empty scan yields no points")


# --- A2. the mast mask (measured 2026-08-23: 0.21 m over 350-10 deg) --------
from vector_dimos.rplidar_c1 import scan_to_points, MAST_MASK_DEG, MASK_RANGE_M
scan = [(40, 0.0, 210.0), (40, 355.0, 220.0),                        # the mast bar (0.225 m, +-6 deg): dropped
        (40, 37.0, 380.0),                                           # 37 deg, 0.38 m: under MIN_RANGE_M: dropped
        (40, 20.0, 450.0),                                           # 20 deg, 0.45 m: outside the bar, real: kept (the old +-45 deg wedge ate it)
        (40, 0.0, 1200.0), (40, 5.0, 800.0),                         # same wedge, far: kept
        (40, 90.0, 210.0), (40, 180.0, 350.0),                       # outside the wedge but on the rover (< 0.40 m): dropped
        (40, 90.0, 450.0), (40, 180.0, 600.0),                       # outside the wedge, real: kept
        (3, 60.0, 500.0), (40, 60.0, 0.0)]                           # weak / invalid: dropped
pts = scan_to_points(scan, min_quality=10)
check(len(pts) == 5, f"mast bar under {MASK_RANGE_M} m and anything under 0.40 m dropped, 5 real points kept ({len(pts)})")
check(any(close(x, -1.2) and close(y, 0.0) for x, y, _ in pts), "a far point inside the bar's bearing survives at 1.2 m toward the NEW tail")

# --- B. retry loop on a fake rplidar lib ----------------------------------
print("\nB. retry loop (fake lib, no sensor)")


class FakeLidarError(Exception):
    """Stand-in for rplidar.RPLidarException."""


class Bench:
    """What the test flips to move the sensor in and out."""

    def __init__(self):
        self.present = False        # is the port openable?
        self.unplugged = False      # yanked mid-scan?
        self.opens = []             # (port, baudrate) per attempt
        self.shutdowns = 0


bench = Bench()


class FakeLidar:
    def __init__(self, port, baudrate=115200, timeout=1, logger=None):
        bench.opens.append((port, baudrate))
        if not bench.present:
            raise FakeLidarError(
                "Failed to connect to the sensor due to: [Errno 2] "
                "could not open port %s" % port)

    def get_info(self):
        return {"model": 97, "firmware": (1, 0)}

    def iter_scans(self):
        while True:
            if bench.unplugged:
                raise FakeLidarError("device reports readiness to read but "
                                     "returned no data (port closed?)")
            yield list(SCAN)
            time.sleep(0.05)

    def _dead_port(self):
        bench.shutdowns += 1
        if bench.unplugged:  # a yanked USB dongle fails every close step
            raise FakeLidarError("port is gone")

    stop = stop_motor = disconnect = _dead_port


rp.RPLidarC1.lidar_class = FakeLidar   # the bench drives a fake device class
spy = LogSpy()
real_logger, rp.logger = rp.logger, spy

lidar_module = RPLidarC1(retry_period_s=0.5)
clouds = []
lidar_module.pointcloud.subscribe(clouds.append)
check(lidar_module.port == "/dev/ttyUSB0" and lidar_module.baudrate == 460800,
      "defaults: /dev/ttyUSB0 (CP2102 dongle) at 460800 baud")
check(DEFAULT_BAUDRATE == 460800, "DEFAULT_BAUDRATE is the C1 line rate")
lidar_module.start()

# 1. sensor absent at startup: retries, logs once, never dies.
check(wait_for(lambda: len(bench.opens) >= 3, 4.0),
      f"absent sensor -> keeps retrying ({len(bench.opens)} attempts)")
check(spy.count("RPLIDAR C1 unavailable") == 1,
      "the failure is logged ONCE, not once per retry")
check(lidar_module._thread.is_alive() and not clouds,
      "loop alive, nothing published while the sensor is missing")
check(all(open_args == ("/dev/ttyUSB0", 460800) for open_args in bench.opens),
      "every attempt opens the port at 460800 baud")

# 2. sensor appears: one flat cloud per revolution, known points.
bench.present = True
check(wait_for(lambda: len(clouds) >= 2, 4.0),
      f"sensor plugged in -> publishes clouds ({len(clouds)} so far)")
check(spy.count("RPLIDAR C1 up on /dev/ttyUSB0 @ 460800 baud") == 1,
      "logs the sensor coming up, with its port and baud rate")
xyz = clouds[0].as_numpy()[0]
# the module's default is min_quality=0 since 2026-08-23: every measure with a
# distance is kept (the weak return included), only the distance-0 one is dropped
check(xyz.shape == (4, 3), f"cloud carries the 4 measures that have a distance {xyz.shape}")
check(all(close(g, e) for row, exp in zip(xyz, EXPECTED_ALL)
          for g, e in zip(row, exp)),
      f"cloud points = {[tuple(round(float(v), 3) for v in r) for r in xyz]} m")
check(clouds[0].frame_id == "lidar_link", "frame_id survives ModuleConfig")

# 3. yanked mid-scan: logged, closing steps all fail, loop keeps going.
published_before = len(clouds)
shutdowns_before = bench.shutdowns
bench.unplugged = True
check(wait_for(lambda: spy.count("returned no data") >= 1, 3.0),
      "unplugged mid-scan -> the lib error is logged")
check(wait_for(lambda: bench.shutdowns > shutdowns_before, 2.0),
      "the close sequence runs even though the port is gone")
check(lidar_module._thread.is_alive(), "loop survives the unplug")
check(wait_for(lambda: len(bench.opens) > published_before, 3.0)
      and len(clouds) == published_before,
      "retrying, and no cloud published while unplugged")

# 4. plugged back in: publishing resumes.
resumed_at = len(clouds)
bench.unplugged = False
check(wait_for(lambda: len(clouds) > resumed_at, 4.0),
      "sensor back -> publishing resumes")

t0 = time.monotonic()
lidar_module.stop()
stop_s = time.monotonic() - t0
check(not lidar_module._thread.is_alive() and stop_s < 3.0,
      f"stop() joins the loop in {stop_s:.2f} s (retry wait is interruptible)")

rp.logger = real_logger
rp.RPLidarC1.lidar_class = None


# --- C. the real reader, against a port that does not exist ---------------
print("\nC. real C1 reader (pyserial) on a missing port")
rp.RPLidarC1.lidar_class = None
if True:
    spy = LogSpy()
    real_logger, rp.logger = rp.logger, spy
    absent = RPLidarC1(port="/dev/ttyUSB_vector_cold_test", retry_period_s=0.5)
    absent.start()
    check(wait_for(lambda: spy.count("RPLIDAR C1 unavailable") == 1, 4.0),
          "missing device -> the open error is caught and logged once")
    check(any("No such file or directory" in line for line in spy.lines),
          "the log names the real cause")
    check(absent._thread.is_alive(), "module still running with no sensor")
    t0 = time.monotonic()
    absent.stop()
    check(not absent._thread.is_alive() and time.monotonic() - t0 < 3.0,
          "clean stop with the sensor never present")
    rp.logger = real_logger

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

"""Gyro sign bench: does the D455F gyro agree with the lidar about turning?

The gyro axis mapping ("-y") was calibrated on the OLD front mast; the camera
moved to the rear mast and the sign was never re-validated - and a wrong-sign
rotation prior is worse than none. This bench measures instead of guessing:

  1. commands a slow in-place rotation, +30 deg then -30 deg (0.3 rad/s)
  2. integrates the gyro (mapped axis) over each move
  3. reads the lidar odometry yaw over the same window
  4. verdict: SAME sign and similar magnitude -> the mapping is right;
     OPPOSITE sign -> flip it; garbage -> the mapping axis is wrong.

Known input -> known output, in degrees (the functional-test rule).

Run on the Jetson WITH the stack up (needs imu + odom on the bus and the
motors armed), rover repositioned with room to turn:

    .venv/bin/python tools/gyro_sign_bench.py
"""
from __future__ import annotations

import math
import threading
import time

import lcm as lcmlib

from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu

TURN_RAD_S = 0.3
TURN_DEG = 30.0
GYRO_AXIS = "-y"          # the mapping under test (lidar_odometry.gyro_axis)


def _axis_value(msg: Imu) -> float:
    v = msg.angular_velocity
    val = {"x": v.x, "y": v.y, "z": v.z}[GYRO_AXIS[-1]]
    return -val if GYRO_AXIS.startswith("-") else val


class Bench:
    def __init__(self) -> None:
        self.lc = lcmlib.LCM()
        self.twist = make_transport("/cmd_vel", Twist)
        self.gyro_int = 0.0
        self.last_imu_ts: float | None = None
        self.yaw: float | None = None
        self.lock = threading.Lock()
        self.lc.subscribe("/imu#sensor_msgs.Imu", self._on_imu)
        self.lc.subscribe("/odom#geometry_msgs.PoseStamped", self._on_odom)
        self.running = True
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self) -> None:
        while self.running:
            self.lc.handle_timeout(100)

    def _on_imu(self, _ch: str, data: bytes) -> None:
        msg = Imu.lcm_decode(data)
        now = time.monotonic()
        with self.lock:
            if self.last_imu_ts is not None:
                self.gyro_int += _axis_value(msg) * (now - self.last_imu_ts)
            self.last_imu_ts = now

    def _on_odom(self, _ch: str, data: bytes) -> None:
        msg = PoseStamped.lcm_decode(data)
        with self.lock:
            self.yaw = msg.orientation.euler[2]

    def reset(self) -> None:
        with self.lock:
            self.gyro_int = 0.0
            self.last_imu_ts = None

    def snapshot(self) -> tuple[float, float | None]:
        with self.lock:
            return self.gyro_int, self.yaw

    def command(self, wz: float, seconds: float) -> None:
        msg = Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, wz])
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            self.twist.broadcast(None, msg)
            time.sleep(0.1)
        self.twist.broadcast(None, Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))


def main() -> int:
    b = Bench()
    time.sleep(1.5)
    _, yaw0 = b.snapshot()
    if yaw0 is None:
        print("KO: no odom on the bus - is the stack up?")
        return 1
    if b.last_imu_ts is None:
        print("KO: no imu on the bus - is the camera module up?")
        return 1

    results = []
    for direction, label in ((+1.0, f"+{TURN_DEG:.0f} deg (left)"),
                             (-1.0, f"-{TURN_DEG:.0f} deg (right)")):
        b.reset()
        _, yaw_before = b.snapshot()
        b.command(direction * TURN_RAD_S, math.radians(TURN_DEG) / TURN_RAD_S)
        time.sleep(1.0)
        gyro, yaw_after = b.snapshot()
        lidar = math.degrees(math.atan2(math.sin(yaw_after - yaw_before),
                                        math.cos(yaw_after - yaw_before)))
        gyro_deg = math.degrees(gyro)
        results.append((label, gyro_deg, lidar))
        print(f"{label}: gyro({GYRO_AXIS}) = {gyro_deg:+.1f} deg | lidar yaw = {lidar:+.1f} deg")

    ok = all(g * l > 0 and 0.5 < abs(g / l) < 2.0 for _, g, l in results if abs(l) > 5)
    flipped = all(g * l < 0 for _, g, l in results if abs(l) > 5)
    if ok:
        print(f"VERDICT: mapping '{GYRO_AXIS}' CORRECT - the gyro may serve as rotation prior")
    elif flipped:
        print(f"VERDICT: SIGN FLIPPED - use '{GYRO_AXIS[1:] if GYRO_AXIS.startswith('-') else '-' + GYRO_AXIS}'")
    else:
        print("VERDICT: axis mapping WRONG or gyro not delivering - check the other axes")
    b.running = False
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())

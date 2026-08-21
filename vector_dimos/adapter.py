"""VectorBaseAdapter - dimOS TwistBaseAdapter for the VECTOR mecanum platform.

Pure python on purpose: dimOS's TwistBaseAdapter is a structural Protocol
(runtime_checkable), so this file imports nothing from dimos and stays
cold-testable. The dimOS glue (adapter registry, blueprints) lives in
blueprints.py.

Virtual joint order (holonomic): [vx, vy, wz]. Odometry [x, y, theta] is
integrated from wheel feedback - LOW CONFIDENCE by design doctrine (mecanum
rollers slip; use it for sanity, never as the localization reference: the
D455F point cloud + RPLIDAR do that job).
"""
from __future__ import annotations

import math
import time
import threading

from .kinematics import (MecanumGeometry, forward, inverse, rads_to_rpm,
                         rpm_to_rads)
from .zlac8015d import Controller

FRONT_ID = 2   # L port = FL, R port = FR   (proven v1 topology)
BACK_ID = 1    # L port = BL, R port = BR
DEFAULT_PORT = "/dev/ttyTHS1"


class VectorBaseAdapter:
    """TwistBaseAdapter implementation for VECTOR (2x ZLAC8015D, RS485).

    Args:
        dof: must be 3 (holonomic).
        address: serial port of the RS485 bus (Waveshare hat).
        client: injectable modbus client (tests use MockModbusClient).
    """

    def __init__(self, dof: int = 3, address: str | None = None,
                 hardware_id: str = "base", client=None,
                 geometry: MecanumGeometry | None = None,
                 accel_ms: int = 1000, **_: object) -> None:
        if dof != 3:
            raise ValueError(f"VECTOR is holonomic: dof must be 3, got {dof}")
        self._port = address or DEFAULT_PORT
        self._client = client
        self._geometry = geometry or MecanumGeometry()
        self._accel_ms = accel_ms
        self._connected = False
        self._enabled = False
        self._lock = threading.RLock()
        self._front: Controller | None = None
        self._back: Controller | None = None
        # odometry integration state
        self._pose = [0.0, 0.0, 0.0]
        self._last_t: float | None = None

    # ── connection ─────────────────────────────────────────────────────
    def connect(self) -> bool:
        try:
            with self._lock:
                if self._client is None:
                    from pymodbus.client.sync import ModbusSerialClient
                    self._client = ModbusSerialClient(
                        method="rtu", port=self._port, baudrate=115200,
                        timeout=0.5)
                    self._client.connect()
                self._front = Controller(FRONT_ID, client=self._client)
                self._back = Controller(BACK_ID, client=self._client)
                self._connected = True
            return True
        except Exception:
            self._connected = False
            return False

    def disconnect(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self.write_stop()
                    self.write_enable(False)
                except Exception:
                    pass
                try:
                    self._client.close()
                except Exception:
                    pass
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return 3

    # ── wheel <-> twist plumbing ───────────────────────────────────────
    def _read_wheel_rads(self):
        """Feedback (FL, FR, BL, BR) in rad/s, left-port inversion undone."""
        fl_raw, fr_raw = self._front.get_rpm()
        bl_raw, br_raw = self._back.get_rpm()
        return (rpm_to_rads(-fl_raw), rpm_to_rads(fr_raw),
                rpm_to_rads(-bl_raw), rpm_to_rads(br_raw))

    def read_velocities(self) -> list[float]:
        with self._lock:
            vx, vy, wz = forward(*self._read_wheel_rads(), self._geometry)
        return [vx, vy, wz]

    def read_odometry(self) -> list[float] | None:
        """Integrated wheel odometry [x, y, theta] - low confidence."""
        with self._lock:
            vx, vy, wz = forward(*self._read_wheel_rads(), self._geometry)
            now = time.monotonic()
            if self._last_t is not None:
                dt = now - self._last_t
                x, y, th = self._pose
                x += (vx * math.cos(th) - vy * math.sin(th)) * dt
                y += (vx * math.sin(th) + vy * math.cos(th)) * dt
                th += wz * dt
                self._pose = [x, y, th]
            self._last_t = now
            return list(self._pose)

    def write_velocities(self, velocities: list[float]) -> bool:
        vx, vy, wz = velocities
        w_fl, w_fr, w_bl, w_br = inverse(vx, vy, wz, self._geometry)
        try:
            with self._lock:
                # left ports inverted (v1 convention: negative = forward)
                self._front.set_rpm(-rads_to_rpm(w_fl), rads_to_rpm(w_fr))
                self._back.set_rpm(-rads_to_rpm(w_bl), rads_to_rpm(w_br))
            return True
        except Exception:
            return False

    def write_stop(self) -> bool:
        try:
            with self._lock:
                self._front.set_rpm(0, 0)
                self._back.set_rpm(0, 0)
            return True
        except Exception:
            return False

    def write_enable(self, enable: bool) -> bool:
        try:
            with self._lock:
                for c in (self._front, self._back):
                    if enable:
                        c.set_mode_velocity()
                        c.set_accel_ms(self._accel_ms, self._accel_ms)
                        c.enable()
                    else:
                        c.disable()
                self._enabled = enable
            return True
        except Exception:
            return False

    def read_enabled(self) -> bool:
        return self._enabled

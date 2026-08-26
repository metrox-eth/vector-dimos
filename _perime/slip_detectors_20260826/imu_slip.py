"""Wheel-slip detection against the IMU: the witness that never lies.

The wheels-vs-lidar guard (stuck_guard.py) fails exactly when slip matters
most: while the wheels push, the wheel prior drags the lidar pose along, so
the judge agrees with the liar (23/08 18:45, map lost; 24/08 metrox: "les
roues tournent dans le vide pendant relativement longtemps"). The IMU sits
on the body and reports what the BODY does, wheels be damned.

Two independent tests, both cold-benched:
  * rotation, continuous: the wheels imply a yaw rate (base/wz in the
    coordinator joint state); the gyro measures the real one. Disagreement
    beyond GYRO_MISMATCH_RAD_S for CONFIRM_S -> slip. Works at any time,
    wheels in the air included.
  * translation, at onset: the wheel speed rose by ONSET_SPEED_MPS within
    ONSET_WINDOW_S but the integrated forward acceleration accounts for
    less than ONSET_ACCEL_FRACTION of it -> the wheels spun up, the body
    did not follow. (A body already sliding at constant wheel speed shows
    no acceleration at all - that steady case stays with the lidar guard.)

Publishes the same ``slip`` Bool the planner and the costmap already react
to (stop + back off 20 cm, freeze the wheel prior, roll the map back 2 s).
Expected reaction time ~0.3-0.5 s instead of ~1.2 s.

Silent inside a ``no_slip_reflex`` zone (26/08). Slipping on a ramp is normal
and transient; on the kitchen-workshop ramp the reflex fired twice mid-climb
(IMU SLIP #9/#10), cut the torque, and the rover slid back down "like ice".
That needs the rover's position, hence the ``odom`` stream, and it only counts
once the run has relocalized into the persistent frame the zones are drawn in.

The IMU frame is the D455 accel optical frame: x right, y down, z forward
(camera pitched 1.4 deg, ignored here). Body yaw rate = -gyro.y; body
forward acceleration = accel.z.
"""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

from vector_dimos import persistent_map

logger = setup_logger()

GYRO_MISMATCH_RAD_S = 0.25    # wheels say we turn this much faster than the gyro measures
CONFIRM_S = 0.2               # mismatch must hold this long
ONSET_WINDOW_S = 0.4
ONSET_SPEED_MPS = 0.12        # wheel speed rise that must be mirrored by the body
ONSET_ACCEL_FRACTION = 0.4    # body must account for at least this share of the rise
COOLDOWN_S = 2.0
GRAVITY = 9.81


class ImuSlipDetector(Module):
    imu: In[Imu]
    coordinator_joint_state: In[JointState]
    odom: In[PoseStamped]                   # where we are: a ramp is allowed to make the wheels slip
    reloc_frame: In[PoseStamped]            # lidar_odometry: the zones only count in the persistent frame
    slip: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gyro: deque[tuple[float, float]] = deque(maxlen=256)      # (t, body wz rad/s)
        self._accel: deque[tuple[float, float]] = deque(maxlen=256)     # (t, body forward accel m/s2)
        self._wheel_v: deque[tuple[float, float, float]] = deque(maxlen=128)   # (t, |v| m/s, wz rad/s)
        self._mismatch_since = 0.0
        self._last_trip = 0.0
        self.trips = 0
        self._xy: tuple[float, float] | None = None
        self._quiet = persistent_map.ZoneWatch(persistent_map.NO_SLIP_REFLEX)
        self._last_quiet_log = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("IMU slip detector up: gyro mismatch %.2f rad/s for %.1f s; onset %.2f m/s over %.1f s"
                    % (GYRO_MISMATCH_RAD_S, CONFIRM_S, ONSET_SPEED_MPS, ONSET_WINDOW_S))

    @rpc
    def stop(self) -> None:
        super().stop()

    async def handle_imu(self, msg: Imu) -> None:
        now = time.monotonic()
        # D455 accel optical frame -> body: yaw rate = -gyro.y, forward accel = accel.z
        self._gyro.append((now, -float(msg.angular_velocity.y)))
        self._accel.append((now, float(msg.linear_acceleration.z)))
        self._check(now)

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._xy = (float(msg.position.x), float(msg.position.y))

    async def handle_reloc_frame(self, msg: PoseStamped) -> None:
        if self._quiet.note_frame(str(getattr(msg, "frame_id", "") or "")):
            logger.info("IMU slip detector: persistent frame "
                        + ("live, no-slip-reflex zones now count" if self._quiet.persistent
                           else "lost, no-slip-reflex zones ignored"))

    async def handle_coordinator_joint_state(self, msg: JointState) -> None:
        now = time.monotonic()
        try:
            names = list(msg.name)
            iv, iw = names.index("base/vx"), names.index("base/wz")
            vx = float(msg.velocity[iv]); wz = float(msg.velocity[iw])
            iy = names.index("base/vy") if "base/vy" in names else None
            vy = float(msg.velocity[iy]) if iy is not None else 0.0
        except (ValueError, IndexError):
            return
        self._wheel_v.append((now, math.hypot(vx, vy), wz))

    # ---- the two tests ------------------------------------------------------
    def _rotation_mismatch(self, now: float) -> bool:
        if not self._gyro or not self._wheel_v:
            return False
        recent_g = [g for t, g in self._gyro if t >= now - CONFIRM_S]
        recent_w = [w for t, _, w in self._wheel_v if t >= now - CONFIRM_S]
        if not recent_g or not recent_w:
            return False
        gyro_wz = sum(recent_g) / len(recent_g)
        wheel_wz = sum(recent_w) / len(recent_w)
        return abs(wheel_wz) > 0.15 and abs(wheel_wz - gyro_wz) > GYRO_MISMATCH_RAD_S

    def _onset_mismatch(self, now: float) -> bool:
        w = [(t, v) for t, v, _ in self._wheel_v if t >= now - ONSET_WINDOW_S]
        if len(w) < 3:
            return False
        rise = w[-1][1] - w[0][1]
        if rise < ONSET_SPEED_MPS:
            return False
        a = [(t, x) for t, x in self._accel if t >= now - ONSET_WINDOW_S]
        if len(a) < 4:
            return False
        dv = 0.0
        for (t0, a0), (t1, a1) in zip(a, a[1:], strict=False):
            dv += 0.5 * (a0 + a1) * (t1 - t0)
        return abs(dv) < ONSET_ACCEL_FRACTION * rise

    def _check(self, now: float) -> None:
        if now - self._last_trip < COOLDOWN_S:
            return
        rotation = self._rotation_mismatch(now)
        onset = self._onset_mismatch(now)
        if not rotation and not onset:
            self._mismatch_since = 0.0
            return
        if self._mismatch_since == 0.0:
            self._mismatch_since = now
            return
        if now - self._mismatch_since < CONFIRM_S:
            return
        kind = "rotation" if rotation else "onset"
        zone = self._quiet.at(*self._xy) if self._xy else None
        if zone is not None:
            self._mismatch_since = 0.0
            if now - self._last_quiet_log >= 5.0:
                self._last_quiet_log = now
                logger.info(f"IMU slip ({kind}) inside '{zone}' - no slip published, "
                            "the wheels are meant to slip there")
            return
        self._last_trip = now
        self._mismatch_since = 0.0
        self.trips += 1
        logger.warning(f"IMU SLIP #{self.trips} ({kind}): wheels move, the body does not")
        self.slip.publish(Bool(data=True))

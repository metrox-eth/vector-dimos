"""Wheel odometry publisher for VECTOR - LOW CONFIDENCE by doctrine.

Mecanum rollers slip by construction (every strafe is a controlled skid),
so wheel odometry is never the localization reference on this platform:
the D455F point cloud is primary and the RPLIDAR anchors it. This module
exists as a sanity/backup signal only.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any

from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class VectorWheelOdometry(Module):
    """Publishes Odometry integrated from the base adapter's wheel feedback."""

    dedicated_worker = True

    odometry: Out[Odometry]

    def __init__(self, adapter=None, rate_hz: float = 20.0,
                 frame_id: str = "odom", child_frame_id: str = "base_link",
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.adapter = adapter
        self.rate_hz = rate_hz
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="vector-wheel-odom", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _loop(self) -> None:
        period = 1.0 / self.rate_hz
        while not self._stop_event.is_set():
            if self.adapter is not None and self.adapter.is_connected():
                pose3 = self.adapter.read_odometry()
                vx, vy, wz = self.adapter.read_velocities()
                if pose3 is not None:
                    x, y, th = pose3
                    self.odometry.publish(Odometry(
                        frame_id=self.frame_id,
                        child_frame_id=self.child_frame_id,
                        pose=Pose(position=Vector3(x=x, y=y, z=0.0),
                                  orientation=_yaw_to_quat(th)),
                        twist=Twist(linear=Vector3(x=vx, y=vy, z=0.0),
                                    angular=Vector3(x=0.0, y=0.0, z=wz))))
            time.sleep(period)


def _yaw_to_quat(yaw: float):
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0),
                      w=math.cos(yaw / 2.0))

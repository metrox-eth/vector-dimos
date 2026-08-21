"""PlayStation-style gamepad teleop for VECTOR - Twist out, pygame.joystick.

dimOS ships keyboard/phone/quest teleop but no gamepad; this module fills
the gap for VECTOR. Publishes into ``tele_cmd_vel`` (wire through dimOS's
MovementManager so navigation and teleop coexist), or remap to ``cmd_vel``
for direct drive.

Mapping (standard dual-stick):
    left stick Y  -> vx (forward)     left stick X  -> vy (strafe)
    right stick X -> wz (rotation)    R2 held       -> boost
"""
from __future__ import annotations

import threading
import time
from typing import Any

from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_LINEAR_SPEED = 0.6    # m/s at full stick
DEFAULT_ANGULAR_SPEED = 1.2   # rad/s at full stick
DEFAULT_DEADZONE = 0.12
DEFAULT_RATE_HZ = 50.0
DEFAULT_BOOST = 2.0


class GamepadTeleop(Module):
    """pygame.joystick teleop. Outputs Twist on tele_cmd_vel."""

    dedicated_worker = True

    tele_cmd_vel: Out[Twist]

    def __init__(self, linear_speed: float = DEFAULT_LINEAR_SPEED,
                 angular_speed: float = DEFAULT_ANGULAR_SPEED,
                 deadzone: float = DEFAULT_DEADZONE,
                 rate_hz: float = DEFAULT_RATE_HZ,
                 boost_multiplier: float = DEFAULT_BOOST,
                 joystick_index: int = 0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.deadzone = deadzone
        self.rate_hz = rate_hz
        self.boost_multiplier = boost_multiplier
        self.joystick_index = joystick_index
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def _dz(self, v: float) -> float:
        return 0.0 if abs(v) < self.deadzone else v

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="vector-gamepad", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _loop(self) -> None:
        import pygame

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() <= self.joystick_index:
            logger.error("No gamepad found at index %d", self.joystick_index)
            return
        pad = pygame.joystick.Joystick(self.joystick_index)
        pad.init()
        logger.info("Gamepad connected: %s", pad.get_name())
        period = 1.0 / self.rate_hz
        while not self._stop_event.is_set():
            pygame.event.pump()
            boost = self.boost_multiplier if pad.get_axis(5) > 0.0 else 1.0
            vx = -self._dz(pad.get_axis(1)) * self.linear_speed * boost
            vy = -self._dz(pad.get_axis(0)) * self.linear_speed * boost
            wz = -self._dz(pad.get_axis(3)) * self.angular_speed
            self.tele_cmd_vel.publish(Twist(
                linear=Vector3(x=vx, y=vy, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=wz)))
            time.sleep(period)
        self.tele_cmd_vel.publish(Twist(linear=Vector3(x=0, y=0, z=0),
                                        angular=Vector3(x=0, y=0, z=0)))

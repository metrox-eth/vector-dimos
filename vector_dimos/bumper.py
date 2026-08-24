"""Front bumper (4 micro-switches) + HC-SR04 sonar on the Jetson header.

The physical layer of metrox's bumper build (24/08: aluminium profile, red
RC-car springs, V-156 switches, TPU strikers, printed sonar housing).

Wiring: ONE contiguous 2x6 block on header pins 29-40 (metrox crimps a
single wide Dupont housing and populates only the used positions):
  29/31/33/35 switch NO contacts - 32 = software 3.3 V OUT feeding the four
  switch COMs (pressed reads HIGH; no runtime pull-ups on Jetson GPIO, the
  inputs rely on the default pull-downs - verify with a multimeter, add 10k
  to GND if a pin floats) - 36 sonar Trig - 37 sonar Echo (divider if 5 V)
  - 30/34/39 GND. The only wire outside the block: sonar Vcc to pin 2 (5 V)
  or 1/17 (3.3 V per metrox's bench test); a GPIO cannot source its 2-15 mA.

What the robot does in the room:
  * a switch closes -> STOP + back off 20 cm (the planner's ``bump`` reflex,
    no map rollback: the map was honest, the world was just invisible), and
    the contact point is written into the map's low layer - only the camera
    seeing bare floor there can erase it (regle 9: a contact is learned).
  * the sonar sees something under SONAR_MAP_MAX ahead -> the reading is
    written into the map as a low obstacle, so the planner slows and goes
    around BEFORE touching; under SONAR_STOP_M it counts as a contact.

Every pin is a parameter. Cold-benched with a fake GPIO backend.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

logger = setup_logger()

DEBOUNCE_S = 0.02
SONAR_HZ = 15.0
SONAR_STOP_M = 0.15          # closer than this = a contact
SONAR_MAP_MAX = 1.0          # readings under this go into the map
SONAR_MEDIAN = 3             # concordant echoes before believing one
SOUND_M_PER_S = 343.0
CONTACT_COOLDOWN_S = 1.0
PATCH_HALF_W = 0.10          # low-layer patch half-width written at a contact
PATCH_Z = (0.15, 0.30, 0.45, 0.60)


class GpioBackend:
    """Thin wrapper so the bench can fake the header."""

    def __init__(self) -> None:
        import Jetson.GPIO as GPIO  # type: ignore[import-not-found]
        self._g = GPIO
        GPIO.setmode(GPIO.BOARD)

    def setup_in(self, pin: int) -> None:
        self._g.setup(pin, self._g.IN)

    def setup_out(self, pin: int) -> None:
        self._g.setup(pin, self._g.OUT, initial=self._g.LOW)

    def read(self, pin: int) -> bool:
        return bool(self._g.input(pin))

    def write(self, pin: int, high: bool) -> None:
        self._g.output(pin, self._g.HIGH if high else self._g.LOW)

    def cleanup(self) -> None:
        self._g.cleanup()


class BumperSonar(Module):
    """Ins: odom (to place contacts in the world). Outs: ``bump`` (planner
    reflex), ``lidar`` (world-frame obstacle points into the costmap's low
    layer, same route as the stuck guard's patches)."""

    odom: In[PoseStamped]
    bump: Out[Bool]
    lidar: Out[PointCloud2]

    def __init__(self,
                 switch_pins: tuple[int, ...] = (29, 31, 33, 35),
                 switch_power_pin: int = 32,
                 switch_xy: tuple[tuple[float, float], ...] = ((0.30, 0.18), (0.30, 0.06), (0.30, -0.06), (0.30, -0.18)),
                 sonar_trig_pin: int = 36, sonar_echo_pin: int = 37,
                 sonar_xy: tuple[float, float] = (0.30, 0.0),
                 enabled: bool = True, world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.switch_pins, self.switch_xy = switch_pins, switch_xy
        self.switch_power_pin = switch_power_pin
        self.sonar_trig_pin, self.sonar_echo_pin, self.sonar_xy = sonar_trig_pin, sonar_echo_pin, sonar_xy
        self.enabled, self.world_frame = enabled, world_frame
        self._gpio = None
        self._pose = (0.0, 0.0, 0.0)
        self._pressed_since: dict[int, float] = {}
        self._last_contact = 0.0
        self._sonar_readings: list[float] = []
        self._running = False
        self.contacts = 0

    # ---- plumbing -----------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        if not self.enabled:
            logger.info("bumper/sonar disabled by config")
            return
        try:
            self._gpio = self._make_backend()
            for p in self.switch_pins:
                self._gpio.setup_in(p)
            self._gpio.setup_out(self.switch_power_pin)
            self._gpio.write(self.switch_power_pin, True)   # feeds the switch COMs
            self._gpio.setup_out(self.sonar_trig_pin)
            self._gpio.setup_in(self.sonar_echo_pin)
        except Exception:  # noqa: BLE001 - no header, no bumper; the robot still runs
            logger.exception("bumper/sonar: GPIO unavailable, running without")
            self._gpio = None
            return
        self._running = True
        threading.Thread(target=self._switch_loop, daemon=True).start()
        threading.Thread(target=self._sonar_loop, daemon=True).start()
        logger.info(f"bumper up: switches {self.switch_pins} (pressed=HIGH), sonar trig {self.sonar_trig_pin} echo {self.sonar_echo_pin}")

    def _make_backend(self):
        return GpioBackend()

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception:  # noqa: BLE001
                pass
        super().stop()

    async def handle_odom(self, msg: PoseStamped) -> None:
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._pose = (float(msg.position.x), float(msg.position.y), yaw)

    # ---- switches -----------------------------------------------------------
    def _switch_loop(self) -> None:
        while self._running:
            now = time.monotonic()
            for pin, xy in zip(self.switch_pins, self.switch_xy, strict=True):
                if self._gpio.read(pin):
                    since = self._pressed_since.setdefault(pin, now)
                    if now - since >= DEBOUNCE_S:
                        self._contact(xy, f"switch pin {pin}")
                else:
                    self._pressed_since.pop(pin, None)
            time.sleep(0.005)

    def _contact(self, body_xy: tuple[float, float], what: str) -> None:
        now = time.monotonic()
        if now - self._last_contact < CONTACT_COOLDOWN_S:
            return
        self._last_contact = now
        self.contacts += 1
        self._publish_patch(body_xy)
        self.bump.publish(Bool(data=True))
        logger.warning(f"BUMP #{self.contacts}: {what} at body ({body_xy[0]:+.2f}, {body_xy[1]:+.2f}) -> stop, back off, learned")

    def _publish_patch(self, body_xy: tuple[float, float]) -> None:
        x, y, yaw = self._pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * body_xy[0] - s * body_xy[1]
        wy = y + s * body_xy[0] + c * body_xy[1]
        px, py = -s, c                      # lateral unit vector
        pts = [(wx + t * px, wy + t * py, z)
               for t in np.linspace(-PATCH_HALF_W, PATCH_HALF_W, 5) for z in PATCH_Z]
        cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id=self.world_frame, timestamp=time.time())
        for _ in range(3):                  # costmap needs 2 hits to call it occupied
            self.lidar.publish(cloud)

    # ---- sonar --------------------------------------------------------------
    def _sonar_loop(self) -> None:
        period = 1.0 / SONAR_HZ
        while self._running:
            d = self._sonar_measure()
            if d is not None:
                self._sonar_readings.append(d)
                self._sonar_readings = self._sonar_readings[-SONAR_MEDIAN:]
                if len(self._sonar_readings) == SONAR_MEDIAN:
                    med = sorted(self._sonar_readings)[SONAR_MEDIAN // 2]
                    spread = max(self._sonar_readings) - min(self._sonar_readings)
                    if spread < 0.10:
                        self._sonar_report(med)
            time.sleep(period)

    def _sonar_measure(self) -> float | None:
        g = self._gpio
        g.write(self.sonar_trig_pin, True); time.sleep(10e-6); g.write(self.sonar_trig_pin, False)
        t0 = time.monotonic()
        while not g.read(self.sonar_echo_pin):
            if time.monotonic() - t0 > 0.03:
                return None
        rise = time.monotonic()
        while g.read(self.sonar_echo_pin):
            if time.monotonic() - rise > 0.03:
                return None
        return (time.monotonic() - rise) * SOUND_M_PER_S / 2.0

    def _sonar_report(self, distance: float) -> None:
        if distance < SONAR_STOP_M:
            self._contact((self.sonar_xy[0] + distance, self.sonar_xy[1]), f"sonar {distance:.2f} m")
        elif distance < SONAR_MAP_MAX:
            self._publish_patch((self.sonar_xy[0] + distance, self.sonar_xy[1]))

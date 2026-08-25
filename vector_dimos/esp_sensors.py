"""Contact switches + sonar via the ESP32-S3 bridge (USB serial).

Born 25/08/2026 after the TXB0108 verdict (docs/verdict_museau_20260825.md):
the Jetson 40-pin could not read these sensors; a 2$ ESP32 does it natively.
The ESP firmware (firmware/esp32_sonar/) prints:
    SW a b c d     on every debounced change + 500 ms heartbeat (1 = pressed)
    SONAR <m>      at 10 Hz (-1 = no echo)

This module feeds the SAME outputs the old GPIO BumperSonar fed, so the
costmap and the recovering planner wiring stay untouched, plus a rear line:
- ``bump``      (Bool) front corner contact  -> planner: stop, back off, replan
- ``bump_rear`` (Bool) rear corner contact   -> planner: stop, move forward, replan
- ``lidar``     (PointCloud2) world-frame obstacle patches into the costmap

Corner map (validated live 25/08 17h37): SW order = GPIO 1,2,3,4 =
avant-gauche, arriere-gauche, arriere-droit, avant-droit.
Sonar: front centre, usable range measured by metrox = 66 cm -> trust cap
0.55 m (regle du monde 17). Median of 3, spread < 0.10 m.
"""

from __future__ import annotations

import logging
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
from dimos_lcm.std_msgs import Bool

logger = logging.getLogger(__name__)

ESP_PORT = "/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_80b54ee325280000-if00"
ESP_BAUD = 115200

# SW index -> (nom, position corps (x, y), arriere ?)
CORNERS = (
    ("avant-gauche",  (0.30,  0.20), False),
    ("arriere-gauche", (-0.30, 0.20), True),
    ("arriere-droit", (-0.30, -0.20), True),
    ("avant-droit",   (0.30, -0.20), False),
)
SONAR_XY = (0.30, 0.0)          # centre du pare-chocs avant (a confirmer si demonte)
SONAR_MAX_TRUSTED = 0.55        # regle du monde 17 (66 cm mesures, marge)
SONAR_MEDIAN = 3
SONAR_SPREAD_MAX = 0.10
CONTACT_COOLDOWN_S = 1.0
PATCH_HALF_W = 0.10
PATCH_Z = (0.15, 0.30, 0.45, 0.60)


def parse_line(line: str):
    """('sw', (a,b,c,d)) | ('sonar', metres) | None — cold-testable."""
    parts = line.strip().split()
    if len(parts) == 5 and parts[0] == "SW":
        try:
            return ("sw", tuple(int(p) for p in parts[1:5]))
        except ValueError:
            return None
    if len(parts) == 2 and parts[0] == "SONAR":
        try:
            return ("sonar", float(parts[1]))
        except ValueError:
            return None
    return None


class SonarFilter:
    """Median of 3, spread gate, trust cap — cold-testable."""

    def __init__(self) -> None:
        self._readings: list[float] = []

    def feed(self, d: float) -> float | None:
        if d <= 0 or d > SONAR_MAX_TRUSTED:
            return None
        self._readings.append(d)
        self._readings = self._readings[-SONAR_MEDIAN:]
        if len(self._readings) < SONAR_MEDIAN:
            return None
        if max(self._readings) - min(self._readings) >= SONAR_SPREAD_MAX:
            return None
        return sorted(self._readings)[SONAR_MEDIAN // 2]


class EspSensors(Module):
    """Ins: odom (world placement). Outs: bump, bump_rear, lidar (patches)."""

    odom: In[PoseStamped]
    bump: Out[Bool]
    bump_rear: Out[Bool]
    lidar: Out[PointCloud2]

    def __init__(self, port: str = ESP_PORT, enabled: bool = True,
                 world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.port, self.enabled, self.world_frame = port, enabled, world_frame
        self._pose = (0.0, 0.0, 0.0)
        self._sw = (0, 0, 0, 0)
        self._last_contact = 0.0
        self._sonar = SonarFilter()
        self._last_sonar_patch = 0.0
        self._running = False
        self.contacts = 0

    # ---- plumbing -----------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        if not self.enabled:
            logger.info("esp_sensors disabled by config")
            return
        self._running = True
        threading.Thread(target=self._serial_loop, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    async def handle_odom(self, msg: PoseStamped) -> None:
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._pose = (float(msg.position.x), float(msg.position.y), yaw)

    # ---- serial -------------------------------------------------------------
    def _serial_loop(self) -> None:
        import serial
        while self._running:
            try:
                ser = serial.Serial(self.port, ESP_BAUD, timeout=2.0)
                logger.info("esp_sensors: liaison ESP ouverte (%s)", self.port)
                while self._running:
                    line = ser.readline().decode(errors="replace")
                    if not line:
                        continue
                    self._handle_line(line)
            except Exception as e:  # noqa: BLE001 - l'ESP peut etre debranche
                logger.warning("esp_sensors: liaison perdue (%s), retry dans 2 s", e)
                time.sleep(2.0)

    def _handle_line(self, line: str) -> None:
        parsed = parse_line(line)
        if parsed is None:
            return
        kind, value = parsed
        if kind == "sw":
            prev, self._sw = self._sw, value
            for i in range(4):
                if value[i] and not prev[i]:
                    name, xy, rear = CORNERS[i]
                    self._contact(name, xy, rear)
        else:
            med = self._sonar.feed(value)
            if med is not None and time.monotonic() - self._last_sonar_patch > 0.5:
                self._last_sonar_patch = time.monotonic()
                self._publish_patch((SONAR_XY[0] + med, SONAR_XY[1]))

    # ---- contacts -----------------------------------------------------------
    def _contact(self, name: str, body_xy: tuple[float, float], rear: bool) -> None:
        now = time.monotonic()
        if now - self._last_contact < CONTACT_COOLDOWN_S:
            return
        self._last_contact = now
        self.contacts += 1
        self._publish_patch(body_xy)
        (self.bump_rear if rear else self.bump).publish(Bool(data=True))
        logger.warning("BUMP #%d: %s (%s) -> stop, degage, replanifie",
                       self.contacts, name, "arriere" if rear else "avant")

    def _publish_patch(self, body_xy: tuple[float, float]) -> None:
        x, y, yaw = self._pose
        c, s = math.cos(yaw), math.sin(yaw)
        wx = x + c * body_xy[0] - s * body_xy[1]
        wy = y + s * body_xy[0] + c * body_xy[1]
        px, py = -s, c
        pts = [(wx + t * px, wy + t * py, z)
               for t in np.linspace(-PATCH_HALF_W, PATCH_HALF_W, 5) for z in PATCH_Z]
        cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32),
                                       frame_id=self.world_frame, timestamp=time.time())
        for _ in range(3):
            self.lidar.publish(cloud)

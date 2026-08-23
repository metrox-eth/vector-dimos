"""SLAMTEC RPLIDAR C1 -> flat PointCloud2 for dimOS.

dimOS has no 2D lidar path (no LaserScan message at all) - so instead of
inventing a message type, this module publishes each 360-degree scan as a
FLAT PointCloud2 (z=0). That feeds directly into dimOS's CostMapper
(pointcloud -> OccupancyGrid) and the 2D A* planner: the short path to
"the walls anchor the point clouds".

The C1 is hot-pluggable here: a missing port, a permission error or a mid-scan
unplug is logged once and retried every RETRY_PERIOD_S, so the module can be
started before the sensor is plugged in and survives it being pulled out.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The C1 comes with a CP2102 USB-UART dongle -> /dev/ttyUSB0 on the Jetson.
DEFAULT_PORT = "/dev/ttyUSB0"
# 460800 is the C1's line rate. The reader is our own (c1_serial.py, plain
# pyserial): rplidar-roboticia 0.9.5 answered "Descriptor length mismatch" on
# this unit (2026-08-23) while the raw SLAMTEC protocol worked first try.
DEFAULT_BAUDRATE = 460800
DEFAULT_FRAME = "lidar_link"
RETRY_PERIOD_S = 5.0


def polar_to_xy(angle_deg: float, distance_mm: float) -> tuple[float, float]:
    """One lidar measure -> (x, y) in metres in the sensor frame.

    The lib (rplidar-roboticia 0.9.5, ``iter_measures``) yields the heading
    angle in degrees over [0, 360) and the distance in millimetres. We use the
    plain math convention here, x = d*cos(theta), y = d*sin(theta), so
    0 deg -> +X and 90 deg -> +Y.

    Caveat to settle on the real robot: SLAMTEC's protocol describes that
    heading as increasing CLOCKWISE seen from above, while the robot frame is
    counter-clockwise (x forward, y left). Nothing in the lib compensates. If
    the scan of a known scene comes out mirrored once the C1 is mounted, the
    fix belongs here - negate the angle - and nowhere else.
    """
    # SLAMTEC's heading grows CLOCKWISE seen from above (C1 datasheet, fig.
    # 2-4, "left hand"); the robot frame is right-handed (x forward, y left),
    # so the sign of y is flipped here. Settled 2026-08-23 on the datasheet;
    # the first map of a known room is the check.
    theta = math.radians(angle_deg)
    distance_m = distance_mm / 1000.0
    return distance_m * math.cos(theta), -distance_m * math.sin(theta)


# The camera mast stands in the scan plane: only its horizontal bar, 50 mm
# wide at 225 mm from the lidar centre (metrox, 23/08) = +-6.3 deg. Masked
# with margin: +-12 deg, under 0.30 m. The +-45 deg / 0.50 m wedge shipped
# before blinded a 90 deg sector right where the rover hits things.
MAST_MASK_DEG: tuple[tuple[float, float], ...] = ((348.0, 12.0),)   # wraps past 360
MASK_RANGE_M = 0.30
# Anything closer than this in ANY direction is the rover itself (e-stop box,
# cables on the lid: 0.25 m returns mapped as obstacles that followed the
# rover around on 2026-08-23). The chassis is ~0.45 m long, the lidar is at
# its centre.
MIN_RANGE_M = 0.40


def _in_mask(angle_deg: float, distance_m: float,
             mask: tuple[tuple[float, float], ...], mask_range_m: float) -> bool:
    if distance_m >= mask_range_m:
        return False
    a = angle_deg % 360.0
    for start, end in mask:
        if (start <= a <= end) if start <= end else (a >= start or a <= end):
            return True
    return False


def scan_to_points(scan: list[tuple[float, float, float]],
                   min_quality: int,
                   mask: tuple[tuple[float, float], ...] = MAST_MASK_DEG,
                   mask_range_m: float = MASK_RANGE_M) -> list[tuple[float, float, float]]:
    """A 360-degree scan -> flat (x, y, 0.0) points in metres.

    ``scan`` is what ``iter_scans`` yields: (quality, angle_deg, distance_mm)
    measures. Weak returns (quality below min_quality), invalid ones (distance
    0) and the mast's own reflection (inside ``mask`` and closer than
    ``mask_range_m``) are dropped.
    """
    return [(*polar_to_xy(angle, distance), 0.0)
            for (quality, angle, distance) in scan
            if quality >= min_quality and distance > MIN_RANGE_M * 1000.0
            and not _in_mask(angle, distance / 1000.0, mask, mask_range_m)]


class RPLidarC1(Module):
    """RPLIDAR C1 driver module. Publishes flat pointclouds."""

    dedicated_worker = True

    pointcloud: Out[PointCloud2]

    def __init__(self, port: str = DEFAULT_PORT,
                 baudrate: int = DEFAULT_BAUDRATE,
                 frame_id: str = DEFAULT_FRAME,
                 min_quality: int = 0,   # 2026-08-23: at 10, 13.7 % of valid returns vanished - whole thin/dark objects (table legs) with no strong return at all
                 retry_period_s: float = RETRY_PERIOD_S,
                 **kwargs: Any) -> None:
        # frame_id is a dimOS ModuleConfig field and Module exposes it as a
        # read-only property, so it goes through the config, never onto self.
        super().__init__(frame_id=frame_id, **kwargs)
        self.port = port
        self.baudrate = baudrate
        self.min_quality = min_quality
        self.retry_period_s = retry_period_s
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # @rpc is not decoration for its own sake: Module.start/stop carry it,
    # and an override that drops it falls out of the class's rpcs table.
    # dimOS then proxies the call by pickling the module across the worker
    # pipe, which dies on our threading.Event ('cannot pickle _thread.lock').
    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker,
                                        name="vector-rplidar", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        super().stop()

    # Named _worker, NOT _loop: dimOS's Module keeps its asyncio event
    # loop in self._loop, which would shadow the method and make the
    # thread target the event loop object.
    # Injection point for the cold bench (a fake device class); None = C1Lidar.
    lidar_class: Any = None

    def _worker(self) -> None:
        from vector_dimos.c1_serial import C1Lidar
        RPLidar = self.lidar_class or C1Lidar

        last_error: str | None = None
        while not self._stop_event.is_set():
            lidar = None
            try:
                # The reader opens the serial port inside __init__: a missing
                # device or a permission error raises right here.
                lidar = RPLidar(self.port, baudrate=self.baudrate)
                logger.info("RPLIDAR C1 up on %s @ %d baud: %s",
                            self.port, self.baudrate, _describe(lidar))
                last_error = None
                self._scan(lidar)
            except Exception as exc:  # noqa: BLE001 - absent/unplugged sensor
                error = f"{type(exc).__name__}: {exc}"
                if error != last_error:  # log once per distinct cause
                    logger.warning(
                        "RPLIDAR C1 unavailable on %s (%s) - retrying every "
                        "%.0f s", self.port, error, self.retry_period_s)
                    last_error = error
            finally:
                _shutdown(lidar)
            self._stop_event.wait(self.retry_period_s)

    def _scan(self, lidar: Any) -> None:
        """Publish one flat cloud per revolution until stop or a lib error."""
        for scan in lidar.iter_scans():
            if self._stop_event.is_set():
                return
            points = scan_to_points(scan, self.min_quality)
            if points:
                self.pointcloud.publish(PointCloud2.from_numpy(
                    np.asarray(points, dtype=np.float32),
                    frame_id=self.frame_id, timestamp=time.time()))


def _describe(lidar: Any) -> str:
    """Device info for the log line - never worth failing a session over."""
    try:
        return str(lidar.get_info())
    except Exception as exc:  # noqa: BLE001
        return f"info unavailable ({type(exc).__name__})"


def _shutdown(lidar: Any) -> None:
    """Stop the scan, stop the motor, close the port. Each step best-effort.

    On an unplug every step raises (the port is gone); that is expected and
    must not mask the original error or block the retry.
    """
    if lidar is None:
        return
    for step in (lidar.stop, lidar.stop_motor, lidar.disconnect):
        try:
            step()
        except Exception:  # noqa: BLE001
            logger.debug("rplidar shutdown step %s failed",
                         getattr(step, "__name__", step), exc_info=True)

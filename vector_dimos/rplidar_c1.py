"""SLAMTEC RPLIDAR C1 -> flat PointCloud2 for dimOS.

dimOS has no 2D lidar path (no LaserScan message at all) - so instead of
inventing a message type, this module publishes each 360-degree scan as a
FLAT PointCloud2 (z=0). That feeds directly into dimOS's CostMapper
(pointcloud -> OccupancyGrid) and the 2D A* planner: the short path to
"the walls anchor the point clouds".
"""
from __future__ import annotations

import math
import threading
from typing import Any

import numpy as np

from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_FRAME = "lidar_link"


class RPLidarC1(Module):
    """RPLIDAR C1 driver module. Publishes flat pointclouds."""

    dedicated_worker = True

    pointcloud: Out[PointCloud2]

    def __init__(self, port: str = DEFAULT_PORT, frame_id: str = DEFAULT_FRAME,
                 min_quality: int = 10, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.port = port
        self.frame_id = frame_id
        self.min_quality = min_quality
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._loop,
                                        name="vector-rplidar", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        super().stop()

    def _loop(self) -> None:
        from rplidar import RPLidar

        lidar = RPLidar(self.port)
        logger.info("RPLIDAR C1 up on %s: %s", self.port, lidar.get_info())
        try:
            for scan in lidar.iter_scans():
                if self._stop_event.is_set():
                    break
                pts = [(d / 1000.0 * math.cos(math.radians(a)),
                        d / 1000.0 * math.sin(math.radians(a)), 0.0)
                       for (q, a, d) in scan
                       if q >= self.min_quality and d > 0]
                if pts:
                    self.pointcloud.publish(PointCloud2.from_numpy(
                        np.asarray(pts, dtype=np.float32),
                        frame_id=self.frame_id))
        finally:
            try:
                lidar.stop()
                lidar.disconnect()
            except Exception:
                pass

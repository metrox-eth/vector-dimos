"""Lidar odometry for VECTOR: KISS-ICP on the RPLIDAR C1 clouds.

Doctrine (project decision, 2026-08-23): the wheels are never the
localization reference on this platform. dimOS ships no odometry for a 2D
lidar or an RGB-D camera (its robots bring their own: the Go2's, Point-LIO on
a Mid-360), so this module supplies the pose: scan-to-map ICP (kiss-icp,
CPU, a few ms on a 350-point scan) in the `world` frame.

Streams
  pointcloud : In[PointCloud2]   one revolution in lidar_link (rplidar_c1.py)
  odom       : Out[PoseStamped]  lidar pose in `world`
  lidar      : Out[PointCloud2]  the same revolution re-expressed in `world` -
               what dimOS's VoxelGridMapper expects ("assumes input clouds are
               already in world frame")
  tf         : Out[TFMessage]    world->base_link (+ static base_link->lidar_link)

A planar scan leaves z / roll / pitch unobservable for a 3D ICP. The cloud is
thickened (copies at +-PLANE_THICKNESS_M) so those DOFs stay pinned, and the
published pose is projected onto SE(2) anyway.
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
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PLANE_THICKNESS_M = 0.05
LIDAR_HEIGHT_M = 0.30          # lidar_link above base_link (lid of the chassis)


def _yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class LidarOdometry(Module):
    pointcloud: In[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    def __init__(self, max_range_m: float = 12.0, min_range_m: float = 0.35,
                 voxel_size_m: float = 0.10, world_frame: str = "world",
                 base_frame: str = "base_link", lidar_frame: str = "lidar_link",
                 log_every_s: float = 2.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.max_range_m, self.min_range_m, self.voxel_size_m = max_range_m, min_range_m, voxel_size_m
        self.world_frame, self.base_frame, self.lidar_frame = world_frame, base_frame, lidar_frame
        self.log_every_s = log_every_s
        # dimOS pickles module instances into their worker process: no lock,
        # no native object may exist before start() (a Lock is not picklable).
        self._kiss: Any = None
        self._lock: Any = None
        self._n = 0
        self._last_log = 0.0
        self._last_ms = 0.0
        self.pose2d = (0.0, 0.0, 0.0)   # x, y, yaw of lidar_link in world

    # ── lifecycle ──────────────────────────────────────────────────────
    @rpc
    def start(self) -> None:
        super().start()
        from kiss_icp.config import KISSConfig
        from kiss_icp.kiss_icp import KissICP
        cfg = KISSConfig()
        cfg.data.max_range = self.max_range_m
        cfg.data.min_range = self.min_range_m
        cfg.data.deskew = False
        cfg.mapping.voxel_size = self.voxel_size_m
        self._kiss = KissICP(cfg)
        self._lock = threading.Lock()
        # the `pointcloud` In is bound to `async def handle_pointcloud` by dimOS (_auto_bind_handlers)
        logger.info(f"lidar odometry up (kiss-icp, voxel {self.voxel_size_m} m, "
                    f"range {self.min_range_m}-{self.max_range_m} m) -> {self.world_frame}")

    @rpc
    def stop(self) -> None:
        super().stop()

    # ── the work ───────────────────────────────────────────────────────
    async def handle_pointcloud(self, msg: PointCloud2) -> None:
        out = msg.as_numpy()
        pts = np.asarray(out[0] if isinstance(out, tuple) else out, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 30:
            return
        t0 = time.monotonic()
        thick = np.concatenate([pts, pts + [0, 0, PLANE_THICKNESS_M], pts - [0, 0, PLANE_THICKNESS_M]])
        with self._lock:
            self._kiss.register_frame(thick, np.zeros(len(thick)))
            pose = np.asarray(getattr(self._kiss, "last_pose", None)
                              if getattr(self._kiss, "last_pose", None) is not None
                              else self._kiss.poses[-1])
        R, t = pose[:3, :3], pose[:3, 3]
        yaw = math.atan2(R[1, 0], R[0, 0])
        x, y = float(t[0]), float(t[1])
        self.pose2d = (x, y, yaw)
        # SE(2) projection for everything we publish
        c, s = math.cos(yaw), math.sin(yaw)
        R2 = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        world_pts = (pts @ R2.T) + np.array([x, y, LIDAR_HEIGHT_M])
        ts = time.time()
        q = _yaw_quat(yaw)
        self.odom.publish(PoseStamped(ts, self.world_frame, position=Vector3(x, y, LIDAR_HEIGHT_M), orientation=q))
        self.lidar.publish(PointCloud2.from_numpy(world_pts.astype(np.float32),
                                                  frame_id=self.world_frame, timestamp=ts))
        self.tf.publish(TFMessage(
            Transform(translation=Vector3(x, y, 0.0), rotation=q,
                      frame_id=self.world_frame, child_frame_id=self.base_frame, ts=ts),
            Transform(translation=Vector3(0.0, 0.0, LIDAR_HEIGHT_M), rotation=Quaternion(0, 0, 0, 1),
                      frame_id=self.base_frame, child_frame_id=self.lidar_frame, ts=ts)))
        self._n += 1
        self._last_ms = (time.monotonic() - t0) * 1000.0
        if t0 - self._last_log >= self.log_every_s:
            self._last_log = t0
            logger.info(f"lidar odom #{self._n}: x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}deg "
                        f"({len(pts)} pts, {self._last_ms:.1f} ms)")

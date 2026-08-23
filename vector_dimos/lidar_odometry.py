"""Lidar odometry for VECTOR: KISS-ICP scan-to-map on the RPLIDAR C1, with a
motion PRIOR from the D455F gyro (rotation) and the wheels (translation).

Doctrine (2026-08-23): the wheels are never the localization reference. dimOS
ships no odometry for a 2D lidar or an RGB-D camera (its robots bring their
own), so this module supplies the pose.

Why a prior. Measured the same day with kiss-icp alone on 340-point planar
scans: translation matched the wheels to 5 mm on a straight line, but turns
were under-estimated by 20-35 % (+16 deg reported for +20 deg, -27 for -41)
even with indoor thresholds. ICP refines a guess; on a sparse 2D scan a bad
guess for a spin is not recovered. The gyro measures the spin directly, the
wheels give a decent translation guess on tiles; ICP corrects both against
the map and the published pose is ICP's, not the prior.

Streams
  pointcloud              : In[PointCloud2]  one revolution, lidar_link (rplidar_c1.py)
  imu                     : In[Imu]          D455F motion module (angular_velocity)
  coordinator_joint_state : In[JointState]   base/vx, base/vy, base/wz positions = wheel odom
  odom   : Out[PoseStamped]  lidar pose in `world`
  lidar  : Out[PointCloud2]  the revolution re-expressed in `world` (VoxelGridMapper input)
  tf     : Out[TFMessage]    world->base_link, base_link->lidar_link

A planar scan leaves z/roll/pitch unobservable for a 3D ICP: the cloud is
thickened (copies at +-PLANE_THICKNESS_M) and the pose is projected on SE(2).
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
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PLANE_THICKNESS_M = 0.05
LIDAR_HEIGHT_M = 0.30          # lidar_link above base_link (lid of the chassis)
# D455F on the mast: ~0.21 m ahead of the lidar (the mast sits at the lidar's
# 0.21 m return), 0.60 m up, level, looking forward. Optical frame x right,
# y down, z forward -> base: X = z, Y = -x, Z = -y.
CAMERA_XYZ_BASE = (0.21, 0.0, 0.60)
DEPTH_STRIDE = 8               # 640x480 -> 80x60 samples, 5 Hz: what the map needs, not more
DEPTH_EVERY = 3                # one depth frame in three (15 fps -> 5 Hz)
DEPTH_MAX_M = 4.0
OBSTACLE_Z_M = (0.05, 0.70)    # the rover's height band in world z: legs yes, table tops no


def _yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _se2(dx: float, dy: float, dyaw: float) -> np.ndarray:
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([[c, -s, 0.0, dx], [s, c, 0.0, dy], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


class LidarOdometry(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]
    coordinator_joint_state: In[JointState]
    depth_image: In[Image]          # RealSense depth (aligned to colour), DEPTH16 mm
    camera_info: In[CameraInfo]     # colour intrinsics (= aligned depth intrinsics)
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    tf: Out[TFMessage]

    def __init__(self, max_range_m: float = 12.0, min_range_m: float = 0.35,
                 voxel_size_m: float = 0.05, initial_threshold_m: float = 0.3,
                 gyro_axis: str = "-y", use_gyro_prior: bool = True,
                 use_wheel_prior: bool = True, world_frame: str = "world",
                 base_frame: str = "base_link", lidar_frame: str = "lidar_link",
                 log_every_s: float = 2.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.max_range_m, self.min_range_m, self.voxel_size_m = max_range_m, min_range_m, voxel_size_m
        self.initial_threshold_m = initial_threshold_m
        # which IMU axis carries the robot's yaw rate, with its sign. The D455F
        # motion module reports in the camera's optical frame (x right, y down,
        # z forward): a level camera looking forward gives yaw_rate = -gyro.y.
        self.gyro_axis = gyro_axis
        self.use_gyro_prior, self.use_wheel_prior = use_gyro_prior, use_wheel_prior
        self.world_frame, self.base_frame, self.lidar_frame = world_frame, base_frame, lidar_frame
        self.log_every_s = log_every_s
        # dimOS pickles module instances into their worker: no lock, no native
        # object before start().
        self._kiss: Any = None
        self._lock: Any = None
        self._n = 0
        self._last_log = 0.0
        self._last_ms = 0.0
        self.pose2d = (0.0, 0.0, 0.0)
        # gyro integration since the last scan (+ totals per axis for calibration)
        self._gyro_last_ts: float | None = None
        self._gyro_acc = 0.0
        self._gyro_seen = False
        self._gyro_totals = np.zeros(3)
        # wheel odometry: latest and the value at the last scan
        self._wheel: tuple[float, float, float] | None = None
        self._wheel_at_scan: tuple[float, float, float] | None = None
        self._prior_used = "cv"
        self._K: tuple[float, float, float, float] | None = None
        self._depth_n = 0
        self._depth_pts_last = 0

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
        cfg.mapping.max_points_per_voxel = 30
        # kiss-icp defaults are tuned for cars (2 m initial threshold)
        cfg.adaptive_threshold.initial_threshold = self.initial_threshold_m
        cfg.adaptive_threshold.min_motion_th = 0.02
        self._kiss = KissICP(cfg)
        self._lock = threading.Lock()
        logger.info(f"lidar odometry up (kiss-icp, voxel {self.voxel_size_m} m, range "
                    f"{self.min_range_m}-{self.max_range_m} m, priors: gyro={self.use_gyro_prior} "
                    f"axis {self.gyro_axis}, wheels={self.use_wheel_prior}) -> {self.world_frame}")

    @rpc
    def stop(self) -> None:
        super().stop()

    # ── priors ─────────────────────────────────────────────────────────
    async def handle_imu(self, msg: Imu) -> None:
        w = msg.angular_velocity
        ts = float(getattr(msg, "ts", 0.0) or time.time())
        if self._gyro_last_ts is not None:
            dt = ts - self._gyro_last_ts
            if 0.0 < dt < 0.1:
                v = np.array([w.x, w.y, w.z], dtype=float)
                self._gyro_totals += v * dt
                axis = {"x": 0, "y": 1, "z": 2}[self.gyro_axis[-1]]
                sign = -1.0 if self.gyro_axis.startswith("-") else 1.0
                self._gyro_acc += sign * v[axis] * dt
                self._gyro_seen = True
        self._gyro_last_ts = ts

    async def handle_coordinator_joint_state(self, msg: JointState) -> None:
        names = list(msg.name); pos = list(msg.position)
        try:
            ix, iy, ith = (names.index(n) for n in ("base/vx", "base/vy", "base/wz"))
        except ValueError:
            return
        self._wheel = (float(pos[ix]), float(pos[iy]), float(pos[ith]))

    async def handle_camera_info(self, msg: CameraInfo) -> None:
        K = list(msg.K or [])
        if len(K) == 9 and K[0] > 0:
            self._K = (float(K[0]), float(K[4]), float(K[2]), float(K[5]))

    async def handle_depth_image(self, msg: Image) -> None:
        """Sparse obstacle cloud from the depth image, in `world`, onto `lidar`.

        The 2D lidar misses thin black table legs (2026-08-23: the rover
        touched Vita's stage). The D455F sees them: every DEPTH_STRIDE-th pixel
        is back-projected, placed in the world with the current pose, cropped
        to the rover's height band, voxel-deduplicated and published on the
        same `lidar` channel the mapper consumes."""
        self._depth_n += 1
        if self._K is None or self._depth_n % DEPTH_EVERY:
            return
        d = np.asarray(msg.data)
        if d.ndim == 3:
            d = d[..., 0]
        fx, fy, cx, cy = self._K
        h, w = d.shape
        vs, us = np.mgrid[0:h:DEPTH_STRIDE, 0:w:DEPTH_STRIDE]
        z = d[vs, us].astype(np.float64) / 1000.0
        ok = (z > 0.15) & (z < DEPTH_MAX_M)
        if not ok.any():
            return
        z, us, vs = z[ok], us[ok], vs[ok]
        xo = (us - cx) * z / fx          # optical x (right)
        yo = (vs - cy) * z / fy          # optical y (down)
        # optical -> base (camera level, looking forward), then camera offset
        bx, by, bz = z + CAMERA_XYZ_BASE[0], -xo + CAMERA_XYZ_BASE[1], -yo + CAMERA_XYZ_BASE[2]
        band = (bz > OBSTACLE_Z_M[0]) & (bz < OBSTACLE_Z_M[1])
        if not band.any():
            return
        bx, by, bz = bx[band], by[band], bz[band]
        x, y, yaw = self.pose2d
        c, s_ = math.cos(yaw), math.sin(yaw)
        wx, wy = c * bx - s_ * by + x, s_ * bx + c * by + y
        pts = np.stack([wx, wy, bz], axis=1)
        # voxel-deduplicate at the map resolution
        keys = np.floor(pts / 0.05).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts = pts[idx].astype(np.float32)
        self._depth_pts_last = len(pts)
        self.lidar.publish(PointCloud2.from_numpy(pts, frame_id=self.world_frame, timestamp=time.time()))

    def _prior_delta(self) -> np.ndarray:
        """Motion since the last scan, in the previous lidar frame (SE(2) as 4x4)."""
        last_delta = np.asarray(self._kiss.last_delta)
        dx, dy = float(last_delta[0, 3]), float(last_delta[1, 3])
        dyaw = math.atan2(last_delta[1, 0], last_delta[0, 0])
        used = []
        if self.use_wheel_prior and self._wheel is not None and self._wheel_at_scan is not None:
            x0, y0, th0 = self._wheel_at_scan; x1, y1, th1 = self._wheel
            c, s = math.cos(-th0), math.sin(-th0)
            wx, wy = x1 - x0, y1 - y0
            dx, dy = c * wx - s * wy, s * wx + c * wy        # world delta -> body frame at the last scan
            dyaw = math.atan2(math.sin(th1 - th0), math.cos(th1 - th0))
            used.append("wheels")
        if self.use_gyro_prior and self._gyro_seen:
            dyaw = self._gyro_acc
            used.append("gyro")
        self._prior_used = "+".join(used) or "cv"
        return _se2(dx, dy, dyaw)

    # ── the work ───────────────────────────────────────────────────────
    async def handle_pointcloud(self, msg: PointCloud2) -> None:
        out = msg.as_numpy()
        pts = np.asarray(out[0] if isinstance(out, tuple) else out, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 30:
            return
        t0 = time.monotonic()
        thick = np.concatenate([pts, pts + [0, 0, PLANE_THICKNESS_M], pts - [0, 0, PLANE_THICKNESS_M]])
        with self._lock:
            k = self._kiss
            prior = self._prior_delta()
            frame = k.preprocessor.preprocess(thick, np.zeros(len(thick)), prior)
            source, frame_down = k.voxelize(frame)
            sigma = k.adaptive_threshold.get_threshold()
            initial_guess = np.asarray(k.last_pose) @ prior
            new_pose = k.registration.align_points_to_map(
                points=source, voxel_map=k.local_map, initial_guess=initial_guess,
                max_correspondance_distance=3 * sigma, kernel=sigma)
            k.adaptive_threshold.update_model_deviation(np.linalg.inv(initial_guess) @ new_pose)
            k.local_map.update(frame_down, new_pose)
            k.last_delta = np.linalg.inv(np.asarray(k.last_pose)) @ new_pose
            k.last_pose = new_pose
            # consume the priors
            self._gyro_acc, self._gyro_seen = 0.0, False
            self._wheel_at_scan = self._wheel
        pose = np.asarray(new_pose)
        R, t = pose[:3, :3], pose[:3, 3]
        yaw = math.atan2(R[1, 0], R[0, 0]); x, y = float(t[0]), float(t[1])
        self.pose2d = (x, y, yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        R2 = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        world_pts = (pts @ R2.T) + np.array([x, y, LIDAR_HEIGHT_M])
        ts = time.time(); q = _yaw_quat(yaw)
        self.odom.publish(PoseStamped(ts, self.world_frame, position=Vector3(x, y, LIDAR_HEIGHT_M), orientation=q))
        self.lidar.publish(PointCloud2.from_numpy(world_pts.astype(np.float32), frame_id=self.world_frame, timestamp=ts))
        self.tf.publish(TFMessage(
            Transform(translation=Vector3(x, y, 0.0), rotation=q, frame_id=self.world_frame, child_frame_id=self.base_frame, ts=ts),
            Transform(translation=Vector3(0.0, 0.0, LIDAR_HEIGHT_M), rotation=Quaternion(0, 0, 0, 1),
                      frame_id=self.base_frame, child_frame_id=self.lidar_frame, ts=ts)))
        self._n += 1
        self._last_ms = (time.monotonic() - t0) * 1000.0
        if t0 - self._last_log >= self.log_every_s:
            self._last_log = t0
            g = np.degrees(self._gyro_totals); w = self._wheel
            logger.info(f"lidar odom #{self._n}: x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}deg "
                        f"({len(pts)} pts, {self._last_ms:.1f} ms, prior {self._prior_used}, depth cloud "
                        f"{self._depth_pts_last} pts) | gyro integral "
                        f"x={g[0]:+.1f} y={g[1]:+.1f} z={g[2]:+.1f}deg | wheels "
                        + (f"x={w[0]:+.3f} y={w[1]:+.3f} th={math.degrees(w[2]):+.1f}deg" if w else "none"))

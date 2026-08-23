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
LIDAR_HEIGHT_M = 0.37          # lidar_link above base_link (metrox, 2026-08-23: 37 cm; centred in width, 3 cm behind the length centre)
# D455F on the mast at the front bumper: 0.30 m ahead of the lidar (rover 54 cm
# long, lidar 3 cm behind its centre), 0.80 m up (floor reads 0.80 m below the
# optical axis, flat with range; depth scale checked against the lidar), level. Optical frame x right,
# y down, z forward -> base: X = z, Y = -x, Z = -y.
CAMERA_XYZ_BASE = (0.30, 0.0, 0.57)   # depth origin above base_link: 0.57 m by floor-plane fit (RANSAC, 23/08); metrox's tape says 0.60 to the lens - the 0.80 shipped before put the floor 23 cm too high in the map
CAMERA_PITCH_RAD = math.radians(1.4)  # camera looks 1.4 deg DOWN (same fit); roll 0.1 deg ignored
DEPTH_STRIDE = 8               # 640x480 -> 80x60 samples, 5 Hz: what the map needs, not more
DEPTH_EVERY = 3                # one depth frame in three (15 fps -> 5 Hz)
DEPTH_MAX_M = 3.0               # beyond that the floor noise (1-2 % of range) leaks into the band
OBSTACLE_Z_M = (0.12, 0.70)    # the rover's height band in world z: legs yes, table tops no, floor noise no


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
    camera_floor: Out[PointCloud2]   # world-frame floor samples (z = 0) the depth camera saw bare: what lets costmap2d forget a low object
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
        self._pose_hist: list[tuple[float, float, float, float]] = []   # (wall ts, x, y, yaw), last ~3 s
        self._yaw_rate = 0.0
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
        ok = (z > 0.30) & (z < DEPTH_MAX_M)          # < 0.30 m = the rover's own front, not the world
        if not ok.any():
            return
        z, us, vs = z[ok], us[ok], vs[ok]
        xo = (us - cx) * z / fx          # optical x (right)
        yo = (vs - cy) * z / fy          # optical y (down)
        # optical -> base (camera level, looking forward), then camera offset
        # optical (x right, y down, z forward) -> base (x forward, y left, z up),
        # camera pitched down by CAMERA_PITCH_RAD: forward axis = (cos, 0, -sin),
        # down axis = (-sin, 0, -cos)
        cp, sp = math.cos(CAMERA_PITCH_RAD), math.sin(CAMERA_PITCH_RAD)
        bx = z * cp - yo * sp + CAMERA_XYZ_BASE[0]
        by = -xo + CAMERA_XYZ_BASE[1]
        bz = -z * sp - yo * cp + CAMERA_XYZ_BASE[2]
        # floor threshold grows with range (depth noise ~1-2 % of range): a 5 cm
        # chair base is an obstacle at 1 m, floor noise is not an obstacle at 3 m
        floor_z = 0.03 + 0.03 * np.clip(bx - 1.0, 0.0, None)
        band = (bz > floor_z) & (bz < OBSTACLE_Z_M[1])
        floor = (bz <= floor_z) & (bz > -0.10)
        if not band.any() and not floor.any():
            return
        fx, fy = bx[floor], by[floor]          # bare floor: published separately, z = 0 (costmap2d misses)
        bx, by, bz = bx[band], by[band], bz[band]
        # pose at the frame's capture time (the depth handler runs 50-200 ms late;
        # at 17 deg/s that smeared the camera layer by several degrees per frame)
        fts = float(getattr(msg, "ts", 0.0) or 0.0)
        if abs(self._yaw_rate) > math.radians(8.0):
            return                         # turning: the camera layer would smear, the lidar maps alone
        pose = self._pose_at(fts) if fts > 0 else self.pose2d
        x, y, yaw = pose
        c, s_ = math.cos(yaw), math.sin(yaw)
        if len(fx):
            fwx, fwy = c * fx - s_ * fy + x, s_ * fx + c * fy + y
            fpts = np.stack([fwx, fwy, np.zeros_like(fwx)], axis=1)
            fkeys = np.floor(fpts / 0.05).astype(np.int64)
            _, fidx = np.unique(fkeys, axis=0, return_index=True)
            self.camera_floor.publish(PointCloud2.from_numpy(fpts[fidx].astype(np.float32), frame_id=self.world_frame, timestamp=time.time()))
        if len(bx) == 0:
            return
        wx, wy = c * bx - s_ * by + x, s_ * bx + c * by + y
        pts = np.stack([wx, wy, bz], axis=1)
        # voxel-deduplicate at the map resolution
        keys = np.floor(pts / 0.05).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        pts = pts[idx].astype(np.float32)
        self._depth_pts_last = len(pts)
        self.lidar.publish(PointCloud2.from_numpy(pts, frame_id=self.world_frame, timestamp=time.time()))

    def _pose_at(self, ts: float) -> tuple[float, float, float]:
        """Pose interpolated at wall time ts from the recent history (else the latest)."""
        h = self._pose_hist
        if len(h) < 2 or ts <= h[0][0]:
            return self.pose2d if not h else (h[0][1], h[0][2], h[0][3])
        if ts >= h[-1][0]:
            return (h[-1][1], h[-1][2], h[-1][3])
        for a, b in zip(h, h[1:]):
            if a[0] <= ts <= b[0]:
                f = (ts - a[0]) / max(b[0] - a[0], 1e-6)
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                return (a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2]), a[3] + f * dyaw)
        return self.pose2d

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
        now_wall = time.time()
        if self._pose_hist:
            pt, _, _, pyaw = self._pose_hist[-1]
            dt = now_wall - pt
            if dt > 1e-3:
                self._yaw_rate = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)) / dt
        self._pose_hist.append((now_wall, x, y, yaw))
        self._pose_hist = [h for h in self._pose_hist if now_wall - h[0] < 3.0]
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

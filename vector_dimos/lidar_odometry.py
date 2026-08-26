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
  reloc_frame : Out[PoseStamped]  which frame this run lives in (costmap2d listens)
  tf     : Out[TFMessage]    world->base_link, base_link->lidar_link

A planar scan leaves z/roll/pitch unobservable for a 3D ICP: the cloud is
thickened (copies at +-PLANE_THICKNESS_M) and the pose is projected on SE(2).

Where `world` IS (2026-08-26). kiss-icp always starts at its own identity, so
until now every restart gave the flat a new arbitrary origin: the map could
not be kept, keep-out zones had no frame to be drawn in, and hand-carrying the
rover scan-matched the new room against a stale memory and offset the walls.
This module now holds an `_origin` on top of kiss-icp - the transform from its
frame into the frame the saved map lives in. At boot the first RELOC_REVS
revolutions are matched against the persistent map
(`vector_dimos.relocalize2d`); on acceptance the origin is set and the
continued map shares the saved frame, on rejection the run starts fresh
exactly as before, with the score numbers logged. The same search is re-run
when the body is moved without the wheels, and NOTHING is written to the map
while it runs.
"""
from __future__ import annotations

import math
import os
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

from vector_dimos import persistent_map
from vector_dimos.relocalize2d import MapField, relocalize

logger = setup_logger()

PLANE_THICKNESS_M = 0.05
LIDAR_HEIGHT_M = 0.37          # lidar_link above base_link (metrox, 2026-08-23: 37 cm; centred in width, 3 cm behind the length centre)
# D455F on the mast at the front bumper: 0.30 m ahead of the lidar (rover 54 cm
# long, lidar 3 cm behind its centre), 0.80 m up (floor reads 0.80 m below the
# optical axis, flat with range; depth scale checked against the lidar), level. Optical frame x right,
# y down, z forward -> base: X = z, Y = -x, Z = -y.
CAMERA_XYZ_BASE = (-0.20, 0.0, 0.56)  # camera moved to the REAR mast 24/08 (bumper build): 20 cm behind the lidar axis (metrox's tape), 0.56 m up, floor-plane fit; sees the floor from ~0.95 m ahead, its own body not at all
CAMERA_PITCH_RAD = math.radians(1.1)  # looks 1.1 deg DOWN (floor fit 24/08); roll -0.1 ignored
DEPTH_STRIDE = 8               # 640x480 -> 80x60 samples, 5 Hz: what the map needs, not more
DEPTH_EVERY = 3                # one depth frame in three (15 fps -> 5 Hz)
DEPTH_MAX_M = 3.0               # beyond that the floor noise (1-2 % of range) leaks into the band
OBSTACLE_MAX_M = 1.8            # 25/08 23h42: camera low-obstacle layer RE-ENABLED to 1.8 m. The 'marble
                                # ghosts' were the lidar layer rotated 180 deg (fixed in rplidar_c1.py):
                                # the camera was right all along. 1.8 m cap kept for far-range depth noise.
                                # WAS suspended 22h with this note: Even under
                                # 1.8 m the marble reflections re-walled the explorer while driving (566 low
                                # cells, 0.5 m2 reachable, 0 frontiers - 3rd walled run in a row). Low objects
                                # stay covered by sonar (<0.55 m ahead) + contact switches + footprint clearing.
                                # Re-enable by raising this once a reflection-proof filter exists and is BENCHED
                                # against the marble. Was: camera obstacles trusted to 1.8 m; on the polished marble the
                                # depth reflections past ~2 m armed phantom low cells (2872 cells, only
                                # 29% lidar-corroborated, ring-walled the explorer - measured 25/08 21:35).
                                # Floor misses keep the full DEPTH_MAX_M range: erasing stays cheap.
# --- relocalization: the map as a place the rover comes back to ----------
# Every restart used to birth an amnesiac map with a fresh arbitrary origin.
# When a persistent map exists, the first revolutions are matched against it
# and the odometry origin is set so the CONTINUED map shares the saved frame.
RELOC_REVS = 8                 # revolutions accumulated per attempt (~0.8 s at 10 Hz)
RELOC_RETRY_S = 5.0            # between two attempts
BOOT_GRACE_S = 120.0           # after a refused boot attempt, keep trying this long WHILE exploring
                               # (see the class docstring: a standing rover has one viewpoint)
CARRY_RESIDUAL_M = 0.35        # the body outran the wheels by this much in CARRY_WINDOW_S: it was carried
CARRY_WINDOW_S = 1.0
CARRY_COOLDOWN_S = 15.0
LOST_SIGMA_M = 1.0             # kiss-icp's adaptive threshold above this...
LOST_SIGMA_S = 1.5             # ...for this long = scan matching no longer converging
CURRENT_MAP_MAX_AGE_S = 300.0  # a checkpoint older than this is not "the current map" any more

OBSTACLE_Z_M = (0.12, 1.30)    # world z band the camera turns into floor obstacles. Upper bound 1.30, not 0.70 (metrox, 23/08): a table top is a BLOCK - the rover goes around tables like a human, never between the legs; a lamp head on a tripod counts too


def _yaw_quat(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _se2(dx: float, dy: float, dyaw: float) -> np.ndarray:
    c, s = math.cos(dyaw), math.sin(dyaw)
    return np.array([[c, -s, 0.0, dx], [s, c, 0.0, dy], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


def split_floor_and_obstacles(bx: np.ndarray, by: np.ndarray, bz: np.ndarray,
                              floor_z: np.ndarray, cell_m: float = 0.05) -> tuple[np.ndarray, np.ndarray]:
    """Masks (obstacle, floor) over base-frame depth points.

    Obstacle = above the floor threshold and below OBSTACLE_Z_M[1] (1.30 m:
    table tops included, so a table is a block the rover walks around).
    Floor = at floor level. Under a table the camera sees BOTH the top and
    the floor between the legs on the same ground cell; the obstacle wins,
    so floor samples on a cell that also holds an obstacle are dropped -
    otherwise hit and miss cancel and the table vanishes from the costmap.
    """
    obst = (bz > floor_z) & (bz < OBSTACLE_Z_M[1])
    floor = (bz <= floor_z) & (bz > -0.10)
    if obst.any() and floor.any():
        keys_o = set(map(tuple, np.floor(np.stack([bx[obst], by[obst]], 1) / cell_m).astype(np.int64)))
        fk = np.floor(np.stack([bx, by], 1) / cell_m).astype(np.int64)
        under = np.array([tuple(k) in keys_o for k in fk])
        floor = floor & ~under
    return obst, floor


class LidarOdometry(Module):
    pointcloud: In[PointCloud2]
    imu: In[Imu]
    coordinator_joint_state: In[JointState]
    depth_image: In[Image]          # RealSense depth (aligned to colour), DEPTH16 mm
    camera_info: In[CameraInfo]     # colour intrinsics (= aligned depth intrinsics)
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    camera_floor: Out[PointCloud2]   # world-frame floor samples (z = 0) the depth camera saw bare: what lets costmap2d forget a low object
    reloc_frame: Out[PoseStamped]    # which frame this run lives in, republished every revolution (costmap2d listens)
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
        # relocalization. `_origin` carries the kiss-icp frame into the frame
        # the map lives in: published pose = _origin (+) kiss pose. kiss-icp is
        # never told about it, so its own scan-to-map keeps working untouched.
        self._origin: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._frame = "fresh"           # "fresh" (own arbitrary origin) | "persistent" (the saved flat)
        # "idle" | "searching" (map writing FROZEN) | "retrying" (boot grace: keep
        # looking while the rover explores and the fresh map keeps building)
        self._reloc_state = "idle"
        self._boot_deadline = 0.0
        self._reloc_gen = 0             # a search started before the last reset is stale
        self._reloc_pts: list[np.ndarray] = []
        self._reloc_thread: Any = None
        self._reloc_result: Any = None
        self._reloc_reason = ""
        self._reloc_next = 0.0
        self._wheel_hist: list[tuple[float, float, float]] = []   # (wall ts, x, y), last ~3 s
        self._lost_since = 0.0
        self._carry_cooldown = 0.0
        self._cfg: Any = None

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
        self._cfg = cfg
        self._kiss = KissICP(cfg)
        self._lock = threading.Lock()
        logger.info(f"lidar odometry up (kiss-icp, voxel {self.voxel_size_m} m, range "
                    f"{self.min_range_m}-{self.max_range_m} m, priors: gyro={self.use_gyro_prior} "
                    f"axis {self.gyro_axis}, wheels={self.use_wheel_prior}) -> {self.world_frame}")
        if not persistent_map.enabled():
            logger.info("PERSISTENT_MAP=0: fresh frame, as before this existed")
        elif not persistent_map.map_exists():
            logger.info(f"no persistent map at {persistent_map.MAP_PATH} yet: fresh frame, "
                        "and this run will become the first one")
        else:
            self._begin_relocalization("boot", reset_kiss=False)

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
        now = time.time()
        self._wheel_hist.append((now, self._wheel[0], self._wheel[1]))
        self._wheel_hist = [w for w in self._wheel_hist if now - w[0] < 3.0]

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
        obst, floor = split_floor_and_obstacles(bx, by, bz, floor_z)
        if not obst.any() and not floor.any():
            return
        fx, fy = bx[floor], by[floor]          # bare floor: published separately, z = 0 (costmap2d misses)
        near = np.hypot(bx, by) < OBSTACLE_MAX_M   # see OBSTACLE_MAX_M: distant "low obstacles" are marble ghosts
        bx, by, bz = bx[obst & near], by[obst & near], bz[obst & near]
        # pose at the frame's capture time (the depth handler runs 50-200 ms late;
        # at 17 deg/s that smeared the camera layer by several degrees per frame)
        fts = float(getattr(msg, "ts", 0.0) or 0.0)
        if abs(self._yaw_rate) > math.radians(8.0) or self._searching:
            return                         # turning or relocalizing: the pose is not trusted
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

    # ── relocalization ─────────────────────────────────────────────────
    @property
    def _searching(self) -> bool:
        """Frozen: nothing goes into the map until we know where we are."""
        return self._reloc_state == "searching"

    def _to_map_frame(self, kiss: tuple[float, float, float]) -> tuple[float, float, float]:
        """kiss-icp's own frame -> the frame the map lives in."""
        ox, oy, oyaw = self._origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return (ox + c * kiss[0] - s * kiss[1],
                oy + s * kiss[0] + c * kiss[1],
                math.atan2(math.sin(oyaw + kiss[2]), math.cos(oyaw + kiss[2])))

    def _begin_relocalization(self, reason: str, reset_kiss: bool) -> None:
        """Freeze map writing and start looking for where we are.

        `reset_kiss` after a hand-carry: kiss-icp's local map is a memory of
        the place the rover was picked up FROM, and scan-matching a new room
        against it is exactly what smeared the walls. A fresh KissICP gives a
        clean frame to search in; the origin then carries it onto the map.
        """
        if reset_kiss and self._cfg is not None:
            from kiss_icp.kiss_icp import KissICP
            with self._lock:
                self._kiss = KissICP(self._cfg)
                self._wheel_at_scan = None
                self._gyro_acc, self._gyro_seen = 0.0, False
        self._reloc_gen += 1
        self._reloc_pts = []
        self._reloc_thread = None
        self._reloc_result = None
        self._reloc_reason = reason
        self._reloc_state = "searching"
        self._lost_since = 0.0
        logger.warning(f"relocalization ({reason}): map writing FROZEN, accumulating {RELOC_REVS} revolutions")

    def _reference_map(self) -> str | None:
        """What to match against: the saved flat at boot, the current map after
        a hand-carry (the freshest checkpoint on disk - same frame, at most one
        checkpoint period old), the saved flat as a fallback."""
        if self._reloc_reason == "boot":
            return persistent_map.MAP_PATH if persistent_map.map_exists() else None
        current = persistent_map.newest_checkpoint()
        if current is not None and time.time() - os.path.getmtime(current) < CURRENT_MAP_MAX_AGE_S:
            return current
        if self._frame == "persistent" and persistent_map.map_exists():
            return persistent_map.MAP_PATH
        return None

    def _accumulate(self, pts: np.ndarray, kiss: tuple[float, float, float]) -> None:
        """One revolution into the search batch, in the kiss-icp frame - the
        frame the search solves for."""
        c, s = math.cos(kiss[2]), math.sin(kiss[2])
        self._reloc_pts.append(np.stack([c * pts[:, 0] - s * pts[:, 1] + kiss[0],
                                         s * pts[:, 0] + c * pts[:, 1] + kiss[1]], axis=1))
        if len(self._reloc_pts) < RELOC_REVS or self._reloc_thread is not None:
            return
        if self._reloc_state == "retrying" and time.monotonic() > self._boot_deadline:
            self._reloc_state = "idle"
            self._reloc_pts = []
            logger.warning(f"relocalization: gave up after {BOOT_GRACE_S:.0f} s of trying - this run "
                           "keeps its own fresh frame, and the keep-out zones do NOT apply to it")
            return
        if time.monotonic() < self._reloc_next:
            self._reloc_pts = self._reloc_pts[-RELOC_REVS:]
            return
        path = self._reference_map()
        if path is None:
            self._reloc_pts = []
            self._reloc_next = time.monotonic() + RELOC_RETRY_S
            logger.warning("relocalization: no map to match against - map writing stays frozen "
                           "(the rover is somewhere nothing has been saved about)")
            return
        batch = np.concatenate(self._reloc_pts)
        self._reloc_pts = []
        self._reloc_thread = threading.Thread(target=self._search, args=(batch, path, self._reloc_gen),
                                              name="relocalize", daemon=True)
        self._reloc_thread.start()

    def _search(self, pts: np.ndarray, path: str, gen: int) -> None:
        """The search itself, off the lidar thread (a global pass costs about a
        second and a half; the pose must keep flowing meanwhile). The frame it
        solves for is fixed, so nothing races - as long as that frame is still
        the current one: a hand-carry started while a search was in flight
        resets kiss-icp, and `gen` is how the stale answer is dropped."""
        try:
            from vector_dimos.costmap2d import ScoredGrid
            field = MapField.from_grid(ScoredGrid.load(path))
            result = (relocalize(field, pts), path, gen)
        except Exception:  # noqa: BLE001
            logger.exception("relocalization failed")
            result = (None, path, gen)
        self._reloc_result = result

    def _collect_relocalization(self) -> None:
        """Apply a finished search. Rejection at boot = start fresh, exactly as
        before. Rejection after a hand-carry = stay frozen and try again: a map
        is worth more than a session."""
        if self._reloc_result is None:
            return
        match, path, gen = self._reloc_result
        self._reloc_result = None
        self._reloc_thread = None
        if gen != self._reloc_gen:
            logger.info("relocalization: dropping an answer computed for a frame we have left")
            return
        origin = os.path.basename(path)
        if match is not None and match.accepted:
            self._origin = (match.x, match.y, match.yaw)
            self._pose_hist = []
            self._wheel_hist = []
            self._carry_cooldown = time.monotonic() + CARRY_COOLDOWN_S
            was = self._reloc_state
            self._reloc_state = "idle"
            if self._reloc_reason == "boot":
                self._frame = "persistent"
            logger.info(f"RELOCALIZED against {origin} ({self._reloc_reason}): {match.as_log()} - "
                        f"the map continues in the {self._frame} frame"
                        + (", the fresh map built during the grace window is dropped"
                           if was == "retrying" else ", writing resumed"))
            return
        detail = match.as_log() if match is not None else "the search raised"
        self._reloc_next = time.monotonic() + RELOC_RETRY_S
        if self._reloc_reason != "boot":
            logger.warning(f"relocalization REJECTED against {origin}: {detail} - map writing stays "
                           f"FROZEN, retrying in {RELOC_RETRY_S:.0f} s")
            return
        if self._reloc_state == "searching":
            # A standing rover sees the flat from ONE spot; that is often not
            # enough to tell two places apart (measured 26/08: refused at boot
            # with margin 1.01, accepted with 1.46 after 90 s of driving, and
            # the boot candidate was the WRONG one). So: start mapping fresh
            # right away - the rover has to move for the answer to sharpen -
            # and keep asking for BOOT_GRACE_S while it does.
            self._reloc_state = "retrying"
            self._frame = "fresh"
            self._boot_deadline = time.monotonic() + BOOT_GRACE_S
            logger.warning(f"relocalization REJECTED against {origin}: {detail} - mapping fresh for "
                           f"now, and trying again for {BOOT_GRACE_S:.0f} s while the rover moves")
            return
        logger.info(f"relocalization still refused ({detail}) - retrying while exploring, "
                    f"{max(0.0, self._boot_deadline - time.monotonic()):.0f} s of grace left")

    def _carried(self, now_wall: float, x: float, y: float) -> bool:
        """Was the body moved without the wheels, or has scan matching given up?

        Two triggers, both cheap. The displacement residual is the honest one:
        over a second, the body cannot outrun the wheels by a third of a metre
        unless somebody picked the rover up. kiss-icp's adaptive threshold is
        the second: when it stays above a metre, the scans stopped matching the
        map at all. Either way the answer is the same - freeze, relocalize.
        """
        if self._reloc_state != "idle" or time.monotonic() < self._carry_cooldown:
            return False
        why = None
        sigma = float(self._kiss.adaptive_threshold.get_threshold())
        if sigma > LOST_SIGMA_M:
            self._lost_since = self._lost_since or time.monotonic()
            if time.monotonic() - self._lost_since > LOST_SIGMA_S:
                why = f"scan matching lost (threshold {sigma:.2f} m for {LOST_SIGMA_S:.1f} s)"
        else:
            self._lost_since = 0.0
        if why is None:
            old_pose = [h for h in self._pose_hist if now_wall - h[0] >= CARRY_WINDOW_S]
            old_wheel = [w for w in self._wheel_hist if now_wall - w[0] >= CARRY_WINDOW_S]
            if not old_pose or not old_wheel or not self._wheel_hist:
                return False
            d_body = math.hypot(x - old_pose[-1][1], y - old_pose[-1][2])
            d_wheel = math.hypot(self._wheel_hist[-1][1] - old_wheel[-1][1],
                                 self._wheel_hist[-1][2] - old_wheel[-1][2])
            if d_body - d_wheel > CARRY_RESIDUAL_M:
                why = (f"the body moved {d_body:.2f} m while the wheels rolled {d_wheel:.2f} m "
                       f"in {CARRY_WINDOW_S:.0f} s")
        if why is None:
            return False
        logger.warning(f"HAND-CARRY / lost: {why}")
        self._begin_relocalization("carried", reset_kiss=True)
        return True

    # ── the work ───────────────────────────────────────────────────────
    async def handle_pointcloud(self, msg: PointCloud2) -> None:
        out = msg.as_numpy()
        pts = np.asarray(out[0] if isinstance(out, tuple) else out, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 30:
            return
        t0 = time.monotonic()
        self._collect_relocalization()
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
        kiss = (float(t[0]), float(t[1]), math.atan2(R[1, 0], R[0, 0]))
        x, y, yaw = self._to_map_frame(kiss)
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
        if self._reloc_state != "idle":
            self._accumulate(pts, kiss)
        elif self._carried(now_wall, x, y):
            pass                            # _carried() opened a new search; this revolution is not written
        self.odom.publish(PoseStamped(ts, self.world_frame, position=Vector3(x, y, LIDAR_HEIGHT_M), orientation=q))
        self.reloc_frame.publish(PoseStamped(
            ts, f"reloc:{'searching' if self._searching else self._frame}",
            position=Vector3(self._origin[0], self._origin[1], 0.0), orientation=_yaw_quat(self._origin[2])))
        if not self._searching:
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

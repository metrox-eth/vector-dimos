"""A virtual bumper for a rover that has none yet.

When the wheels advance but the lidar-odometry pose does not (the rover is
pushing against something its sensors cannot see: a low box, a chair base,
glass), this module (1) publishes `stop_movement` so the planner cancels the
goal, and (2) injects a small patch of points 0.35 m ahead, at rover heights,
onto the `lidar` channel the mapper consumes - the collision becomes an
obstacle in the map, and the planner routes around it next time. The real
bumpers, when they exist, will do the same with a switch.

Measured need: 2026-08-23, the rover pushed the laser crate and later boxes
in front of the sofa for "a good while" with its wheels slipping.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_lcm.std_msgs import Bool  # the same class the planner / explorer / movement manager declare
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WINDOW_S = 1.0          # compare wheel vs lidar displacement over this window
MIN_WHEEL_M = 0.08      # below this the wheels did not really try to move
MAX_RATIO = 0.3         # lidar moved less than 30 % of what the wheels claim -> stuck
COOLDOWN_S = 4.0
OBSTACLE_AHEAD_M = 0.35
OBSTACLE_HALF_W = 0.25


class StuckGuard(Module):
    odom: In[PoseStamped]
    coordinator_joint_state: In[JointState]
    cmd_vel: In[Twist]                      # what the planner asks for
    lidar: Out[PointCloud2]
    slip: Out[Bool]                         # True on every trip: planner backs off 20 cm, odometry stops trusting the wheels, map stops writing

    def __init__(self, world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.world_frame = world_frame
        self._wheel: deque[tuple[float, float, float, float]] = deque(maxlen=400)
        self._lidar: deque[tuple[float, float, float, float]] = deque(maxlen=400)
        self._last_check = 0.0
        self._last_trip = 0.0
        self._last_debug = 0.0
        self.trips = 0
        self._cmd: deque[tuple[float, float]] = deque(maxlen=400)   # (t, |v| commanded)

    @rpc
    def start(self) -> None:
        super().start()
        logger.info("stuck guard up (virtual bumper): wheels vs lidar pose, window %.1f s" % WINDOW_S)

    @rpc
    def stop(self) -> None:
        super().stop()

    async def handle_odom(self, msg: PoseStamped) -> None:
        p = msg.position if hasattr(msg, "position") else msg.pose.position
        q = msg.orientation if hasattr(msg, "orientation") else msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self._lidar.append((time.monotonic(), float(p.x), float(p.y), yaw))

    async def handle_cmd_vel(self, msg: Twist) -> None:
        v = math.hypot(float(msg.linear.x), float(msg.linear.y))
        self._cmd.append((time.monotonic(), v if abs(float(msg.angular.z)) < 0.1 else 0.0))   # a spin is not a push

    async def handle_coordinator_joint_state(self, msg: JointState) -> None:
        names = list(msg.name); pos = list(msg.position)
        try:
            ix, iy, ith = (names.index(n) for n in ("base/vx", "base/vy", "base/wz"))
        except ValueError:
            return
        now = time.monotonic()
        self._wheel.append((now, float(pos[ix]), float(pos[iy]), float(pos[ith])))
        if now - self._last_check < 0.25:
            return
        self._last_check = now
        self._check(now)

    def _disp(self, buf: deque, now: float) -> tuple[float, float, float, float] | None:
        old = [s for s in buf if s[0] <= now - WINDOW_S]
        if not old or len(buf) < 2:
            return None
        a, b = old[-1], buf[-1]
        dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return (math.hypot(b[1] - a[1], b[2] - a[2]), dyaw, b[1], b[2])

    def _check(self, now: float) -> None:
        if now - self._last_trip < COOLDOWN_S:
            return
        w = self._disp(self._wheel, now); l = self._disp(self._lidar, now)
        if w is None or l is None:
            return
        dw, dl = w[0], l[0]
        cmd = [v for t, v in self._cmd if t >= now - WINDOW_S]
        vcmd = (sum(cmd) / len(cmd)) if cmd else 0.0          # mean commanded speed over the window
        if now - self._last_debug >= 2.0:
            self._last_debug = now
            logger.info(f"stuck guard: cmd {vcmd:.3f} m/s, wheels {dw:.3f} m, lidar {dl:.3f} m over {WINDOW_S:.0f} s")
        # stuck = the wheels turned for real and the world did not move. Only
        # that: the "commanded speed" path fired on every slow start (23/08).
        if not (dw >= MIN_WHEEL_M and dl < MAX_RATIO * dw):
            return
        # stuck: wheels claim dw metres, the world says dl
        self._last_trip = now; self.trips += 1
        lx, ly, lyaw = self._lidar[-1][1], self._lidar[-1][2], self._lidar[-1][3]
        # direction the wheels were pushing, in the world frame
        ws = [s for s in self._wheel if s[0] >= now - WINDOW_S]
        vx, vy = ws[-1][1] - ws[0][1], ws[-1][2] - ws[0][2]
        heading = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-3 else lyaw
        # the wheel odom frame and the lidar frame share the heading convention (same start)
        # but not necessarily the origin: use the lidar yaw if the wheel direction is ambiguous
        cx, cy = lx + OBSTACLE_AHEAD_M * math.cos(heading), ly + OBSTACLE_AHEAD_M * math.sin(heading)
        px, py = -math.sin(heading), math.cos(heading)
        pts = []
        for s in np.linspace(-OBSTACLE_HALF_W, OBSTACLE_HALF_W, 11):
            for z in (0.15, 0.30, 0.45, 0.60):
                pts.append((cx + s * px, cy + s * py, z))
        cloud = PointCloud2.from_numpy(np.asarray(pts, dtype=np.float32), frame_id=self.world_frame, timestamp=time.time())
        # No stop_movement: the frontier explorer reads it as "a human took over"
        # and ends the exploration for good (2026-08-23). The planner has its own
        # "Robot is stuck. Replanning" - with the obstacle now in the map, the
        # replanned path goes elsewhere.
        self.slip.publish(Bool(data=True))
        for _ in range(3):
            self.lidar.publish(cloud)
        logger.warning(f"SLIP #{self.trips}: wheels {dw:.2f} m, lidar {dl:.2f} m in {WINDOW_S:.0f} s -> "
                       f"slip reflex + virtual obstacle at ({cx:+.2f}, {cy:+.2f}), heading {math.degrees(heading):+.0f} deg")

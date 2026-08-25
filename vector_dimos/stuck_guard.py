"""A slip detector: the wheels turn, the world does not move.

When the wheels advance but the lidar-odometry pose does not (the rover is
pushing against something its sensors cannot see: a low box, a chair base,
glass), this module publishes ``slip`` — the recovering planner stops and
backs off, the odometry stops trusting the wheels, and the costmap rolls
back the writes the sliding pose just made.

It does NOT write the map (sensor doctrine, metrox 25/08): only the lidar
and the camera put obstacles down. A slip says "something is here that I
cannot see", which is a reason to back off, not a measurement — and the
virtual obstacles it used to inject landed at a pose that had, by
definition, just stopped being trustworthy.

Measured need: 2026-08-23, the rover pushed the laser crate and later boxes
in front of the sofa for "a good while" with its wheels slipping.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos_lcm.std_msgs import Bool  # the same class the planner / explorer / movement manager declare
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

WINDOW_S = 1.5          # compare wheel vs lidar displacement over this window. 1.0 s fired on
                        # every start: 25 kg of inertia + the 400 ms motor ramp + ~150 ms of lidar
                        # odom latency mean wheels honestly lead the body early on (50 false slips
                        # in 35 min on 25/08, each erasing 2 s of map = the sofa vanished).
MIN_WHEEL_M = 0.15      # the wheels must have REALLY rolled (half the window at 0.2 m/s)
MAX_RATIO = 0.25        # world moved less than a quarter of what the wheels claim -> stuck
COOLDOWN_S = 4.0


class StuckGuard(Module):
    odom: In[PoseStamped]
    coordinator_joint_state: In[JointState]
    cmd_vel: In[Twist]                      # what the planner asks for
    slip: Out[Bool]                         # True on every trip: planner backs off 20 cm, odometry stops trusting the wheels, map rolls back

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
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
        logger.info("stuck guard up (slip detector): wheels vs lidar pose, window %.1f s" % WINDOW_S)

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
        # No stop_movement: the frontier explorer reads it as "a human took over"
        # and ends the exploration for good (2026-08-23). The planner has its own
        # "Robot is stuck. Replanning", and the slip reflex backs the rover off
        # first so the replan starts from a position that is not glued to the
        # thing it was pushing.
        self.slip.publish(Bool(data=True))
        logger.warning(f"SLIP #{self.trips}: wheels {dw:.2f} m, lidar {dl:.2f} m in {WINDOW_S:.0f} s -> "
                       "slip reflex (stop, back off, replan); the map rolls back, it learns nothing here")

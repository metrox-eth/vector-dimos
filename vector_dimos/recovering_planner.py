"""ReplanningAStarPlanner that backs up before replanning when stuck.

VECTOR has no bumpers yet. When it drives into something the costmap missed
(a table leg, the sofa skirt), dimOS's planner notices it is not moving and
replans — from a position glued to the obstacle. The new path starts inside
the thing it hit, so it fails or pushes again. Backing up first along the
way it came (known free, it just drove through it) gives the planner room.

Field request (metrox, 23/08/2026): "il faudrait qu'il revienne 20-40 cm en
arrière s'il ne sait pas ce qu'il y a derrière lui".
"""

import threading
import time

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.core.stream import In
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos_lcm.std_msgs import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

BACKUP_DISTANCE_M = 0.20   # metrox: 'just enough to turn'
BACKUP_SPEED_MPS = 0.10
BACKUP_TIMEOUT_S = 5.0
BACKUP_PERIOD_S = 0.05  # base watchdog is 0.2 s; 10 Hz left gaps that stopped the wheels


class RecoveringGlobalPlanner(GlobalPlanner):
    """Stuck → reverse BACKUP_DISTANCE_M (odometry-measured) → replan."""

    backup_distance_m: float = BACKUP_DISTANCE_M
    backup_speed_mps: float = BACKUP_SPEED_MPS
    backup_timeout_s: float = BACKUP_TIMEOUT_S

    def _replan_path(self) -> None:
        # Runs in the planner's monitoring thread. Upstream asserts a goal
        # exists; the goal can vanish (explorer cancels on "no path") while we
        # spend 3 s backing up, and an assert there kills the thread for the
        # rest of the run (seen 23/08: no "Arrived"/"stuck" handled after it).
        try:
            if self._recovering:
                return                     # the slip reflex owns the wheels right now
            # The stuck detector reads position only: while the follower turns in
            # place (initial/final rotation) the position does not change and a
            # 2.5 s window reads "stuck" -> replan -> new rotation -> "stuck"...
            # (20 bursts in the kitchen, 23/08 19:35). Rotating is not stuck.
            if not self._in_stop_message and self._local_planner_state() in ("initial_rotation", "final_rotation"):
                return
            with self._lock:
                has_goal = self._current_goal is not None and self._current_odom is not None
            if not has_goal:
                logger.info("Replan requested without a goal; ignoring.")
                return
            # Back up only if we were actually driving a path for a while: a
            # fresh goal with "no path" also reads as "stuck" (the tracker's
            # window is already still) and that made the rover reverse every
            # 15 s in a dead end (seen 23/08 17:00, metrox: "il va a reculons").
            driving_for = time.perf_counter() - self._path_started_at
            if self._position_tracker.is_stuck() and driving_for >= self._stuck_time_window:
                self._back_up()
                with self._lock:
                    has_goal = self._current_goal is not None
                if not has_goal:
                    logger.info("Goal cancelled during back-up; not replanning.")
                    return
            super()._replan_path()
        except Exception:  # noqa: BLE001 - never let the monitor thread die
            logger.exception("Replan failed; planner monitor keeps running")

    _path_started_at: float = float("inf")
    _recovering: bool = False
    _in_stop_message: bool = False

    def _local_planner_state(self) -> str:
        with self._local_planner._lock:
            return str(self._local_planner._state)

    def _handle_stop_message(self, stop_message) -> None:  # type: ignore[override]
        # so _replan_path can tell "the follower stopped (obstacle/arrived)" from "the monitor thinks we are stuck"
        self._in_stop_message = True
        try:
            super()._handle_stop_message(stop_message)
        finally:
            self._in_stop_message = False

    def slip(self) -> bool:
        """Slip reflex (stuck_guard saw the wheels turn while the lidar pose
        stood still, within 1 s): stop, back off BACKUP_DISTANCE_M in our own
        thread, then ask the monitor for a replan. While the wheels push, the
        odometry slides with them and smears the map (23/08 18:45: 25 s of
        pushing, map lost) - the only cure is to stop within the second.
        Returns False if a recovery is already running."""
        if self._recovering:
            return False
        self._recovering = True
        threading.Thread(target=self._slip_recovery, daemon=True).start()
        return True

    def _slip_recovery(self) -> None:
        try:
            self._back_up()
            with self._lock:
                has_goal = self._current_goal is not None
            if has_goal:
                self._on_stopped_navigating("obstacle_found")
        except Exception:  # noqa: BLE001
            logger.exception("Slip recovery failed")
        finally:
            self._recovering = False
    # dimOS's default is 8 s / 0.4 m. "Stuck for eight seconds is already dead
    # for a robot doing 0.3 m/s" (metrox, 23/08): 2.5 s, same 5 cm/s floor.
    _stuck_time_window: float = 2.5
    _stuck_threshold: float = 0.12
    def _plan_path(self) -> None:
        super()._plan_path()
        if self._local_planner.get_state() != NavigationState.IDLE:
            self._path_started_at = time.perf_counter()
        else:
            self._path_started_at = float("inf")

    def _back_up(self) -> float:
        """Reverse until odometry says we moved backup_distance_m, or time out.

        Runs in the planner's monitoring thread; the local path follower is
        stopped first so it cannot fight the reverse command on cmd_vel.
        Returns the distance actually travelled (m).
        """
        self._local_planner.stop_planning()
        with self._lock:
            start = self._current_odom
        if start is None:
            return 0.0

        reverse = Twist(linear=Vector3(-self.backup_speed_mps, 0.0, 0.0))
        t0 = time.perf_counter()
        travelled = 0.0
        while time.perf_counter() - t0 < self.backup_timeout_s:
            with self._lock:
                odom = self._current_odom
            travelled = odom.position.distance(start.position)
            if travelled >= self.backup_distance_m:
                break
            self._local_planner.cmd_vel.on_next(reverse)
            time.sleep(BACKUP_PERIOD_S)
        self._local_planner.cmd_vel.on_next(Twist())
        self._position_tracker.reset_data()
        self._path_started_at = float("inf")
        logger.info(
            "Stuck: backed up before replanning",
            travelled_m=round(travelled, 3),
            seconds=round(time.perf_counter() - t0, 1),
        )
        return travelled


class RecoveringPlanner(ReplanningAStarPlanner):
    """Drop-in for ReplanningAStarPlanner with the back-up recovery and the
    slip reflex (``slip`` In, fed by stuck_guard.py)."""

    slip: In[Bool]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._planner = RecoveringGlobalPlanner(self._planner._global_config)

    async def handle_slip(self, msg: Bool) -> None:
        if getattr(msg, "data", False):
            self._planner.slip()

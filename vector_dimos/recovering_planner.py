"""ReplanningAStarPlanner that backs up before replanning when stuck.

VECTOR has no bumpers yet. When it drives into something the costmap missed
(a table leg, the sofa skirt), dimOS's planner notices it is not moving and
replans — from a position glued to the obstacle. The new path starts inside
the thing it hit, so it fails or pushes again. Backing up first along the
way it came (known free, it just drove through it) gives the planner room.

Field request (metrox, 23/08/2026): "il faudrait qu'il revienne 20-40 cm en
arrière s'il ne sait pas ce qu'il y a derrière lui".
"""

import time

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

BACKUP_DISTANCE_M = 0.25
BACKUP_SPEED_MPS = 0.10
BACKUP_TIMEOUT_S = 5.0
BACKUP_PERIOD_S = 0.1


class RecoveringGlobalPlanner(GlobalPlanner):
    """Stuck → reverse BACKUP_DISTANCE_M (odometry-measured) → replan."""

    backup_distance_m: float = BACKUP_DISTANCE_M
    backup_speed_mps: float = BACKUP_SPEED_MPS
    backup_timeout_s: float = BACKUP_TIMEOUT_S

    def _replan_path(self) -> None:
        if self._position_tracker.is_stuck():
            self._back_up()
        super()._replan_path()

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
        logger.info(
            "Stuck: backed up before replanning",
            travelled_m=round(travelled, 3),
            seconds=round(time.perf_counter() - t0, 1),
        )
        return travelled


class RecoveringPlanner(ReplanningAStarPlanner):
    """Drop-in for ReplanningAStarPlanner with the back-up recovery."""

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._planner = RecoveringGlobalPlanner(self._planner._global_config)

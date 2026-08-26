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

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.gradient import voronoi_gradient
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.core.stream import In
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos_lcm.std_msgs import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

BACKUP_DISTANCE_M = 0.20   # metrox: 'just enough to turn'
BACKUP_SPEED_MPS = 0.10
BACKUP_TIMEOUT_S = 5.0
BACKUP_PERIOD_S = 0.05  # base watchdog is 0.2 s; 10 Hz left gaps that stopped the wheels

# --- obstacle cost gradient -------------------------------------------------
#
# Measured 26/08 before writing any of this, because the obvious version of the
# story is wrong. dimOS does NOT plan on a flat costmap: global_planner.py:356
# builds a gradient (navigation_map.py:65 -> path_map.py:40-43) and its A*
# really does pay per-cell cost rather than only thresholding it
# (min_cost_astar.py:211-216; the C++ extension behaves identically). On the
# saved flat that field already keeps a path within 3 cm of the widest
# bottleneck any route could possibly have. Two things it still gets wrong:
#
#   1. It cannot tell a wide door from a narrow one. voronoi_gradient puts cost
#      0 on the medial axis of EVERY corridor, so a 0.65 m doorway and a 0.90 m
#      doorway both offer a zero-cost route and the tie falls to path length.
#      Measured: with dimOS's field alone the planner takes the 0.65 m corridor
#      when a 0.90 m one of the same length is right there. That is exactly
#      lesh's "I'd want some gradient from impassable things so that pathing
#      algo prefers to stay away from obstacles", and the shape of 4 of the 11
#      impacts in the 25/08 autopsy.
#   2. simple_inflate rounds its radius up to whole cells (inflation.py:29) and
#      dilates with a disc, so the nominal 0.275 m becomes "block everything
#      within 6 cells INCLUSIVE" = 0.30 m inclusive. The centre cell of a 0.55 m
#      doorway sits at exactly 0.30 m and dies by one cell. Measured: every gap
#      up to 0.62 m plans "no path", 0.65 m is the first that plans - while
#      VECTOR is 0.50 m wide and fits through all of them. That is the
#      "0.62 walled off every 60-70 cm gap" regression, and it lives in dimOS's
#      own default, not in anything we wrote.
#
# So: keep dimOS's ridge cost (it is already near-optimal and its zero-cost
# medial axis is what stops A*, which compares summed cell cost before length,
# from wandering off on infinite detours), take the inflation from a distance
# transform instead of a ceil'd dilation, and add one term dimOS has no way to
# know about - the turning circle.

ROBOT_WIDTH_M = 0.50        # nav_blueprints.py, RecoveringPlanner.blueprint
PIVOT_DIAMETER_M = 0.78     # measured pivot envelope, same blueprint
CONTROL_MARGIN_M = 0.05     # cross-track error we let the follower spend

# Lethal strictly below half a body plus that margin, so a cell sitting exactly
# ON the radius stays passable. That cell is the centre of a doorway the body
# fits through, and rounding it away - which is what a ceil'd dilation does - is
# what walled off every 60-70 cm gap in this flat.
LETHAL_CLEARANCE_M = ROBOT_WIDTH_M / 2 + CONTROL_MARGIN_M   # 0.30
# Below this a turn in place sweeps into the obstacle. It is also where the
# extra cost stops: everything the rover can pivot in must stay at the cost
# dimOS gave it, because A* minimises the SUM of cell costs, so a penalty that
# never reaches zero is just a penalty on length. Swept 26/08 - an influence
# radius past the turning circle made both clearance and path length worse.
PIVOT_CLEARANCE_M = PIVOT_DIAMETER_M / 2                    # 0.39
# Cost added at the lethal edge, on top of dimOS's 0-99 ridge cost, and how
# sharply it falls off across the band. Both swept on the real flat and on the
# synthetic corridors 26/08. The exponent matters more than the size: A*
# minimises the SUM of cell costs, so a penalty that is still noticeable in the
# middle of a pinch just makes the path shorter - it cuts the corner and LOSES
# clearance. The 4th power keeps the middle of the band almost free and puts
# the whole cost against the wall. Measured over the 5 replay pairs:
#
#     penalty  ramp   mean     worst-min  length   corridor chosen
#     0        -      0.507    0.316      31.28    narrow
#     100      ^1     0.506    0.292      31.34    wide
#     100      ^2     0.508    0.300      31.34    wide
#     100      ^4     0.508    0.316      31.28    wide     <- shipped
#     200      ^4     0.509    0.316      32.63    wide     (starts detouring)
#
# So 100 with a 4th-power ramp is the one setting that takes the wide door and
# gives up nothing: same worst case and same path length as dimOS shipped.
PIVOT_PENALTY = 100
PIVOT_RAMP_EXPONENT = 4
# Matches path_map.py:29 so the ridge cost keeps the shape dimOS tuned.
GRADIENT_DISTANCE_M = 1.5


def clearance_cost_map(
    binary: OccupancyGrid,
    lethal_m: float = LETHAL_CLEARANCE_M,
    pivot_m: float = PIVOT_CLEARANCE_M,
    penalty: int = PIVOT_PENALTY,
) -> OccupancyGrid:
    """Binary costmap -> the cost field A* searches.

    Three layers: lethal only where the body cannot fit, dimOS's Voronoi ridge
    cost outside it, and a penalty across the band VECTOR can drive through but
    not pivot in. The penalty ramps as the 4th power of how far into that band
    a cell is, so it stays near zero at the turning circle and only bites hard
    against the wall - see PIVOT_RAMP_EXPONENT for why the shape matters.
    """
    grid = binary.grid
    resolution = binary.resolution

    # metres from every cell to the nearest observed obstacle cell
    distance_m = ndimage.distance_transform_edt(grid < CostValues.OCCUPIED) * resolution
    # strictly inside the radius, with a tolerance so a cell landing exactly on
    # it survives float noise (6 cells x 0.05 is not exactly 0.30)
    too_close = distance_m + 1e-6 < lethal_m

    lethal = OccupancyGrid(
        grid=np.where(too_close, np.int8(CostValues.OCCUPIED), grid).astype(np.int8),
        resolution=resolution,
        origin=binary.origin,
        frame_id=binary.frame_id,
        ts=binary.ts,
    )
    base = voronoi_gradient(lethal, max_distance=GRADIENT_DISTANCE_M).grid.astype(np.int16)

    # unknown keeps its -1 (A* prices it itself, min_cost_astar.py:207-210) and
    # lethal keeps its 100; only genuinely passable cells take the penalty.
    passable = (base >= CostValues.FREE) & (base < CostValues.OCCUPIED)
    ramp = np.clip((pivot_m - distance_m) / (pivot_m - lethal_m), 0.0, 1.0)
    extra = (penalty * ramp**PIVOT_RAMP_EXPONENT).astype(np.int16)

    out = base.copy()
    out[passable] = np.clip(
        base[passable] + extra[passable], CostValues.FREE, CostValues.OCCUPIED - 1
    )

    return OccupancyGrid(
        grid=out.astype(np.int8),
        resolution=resolution,
        origin=binary.origin,
        frame_id=binary.frame_id,
        ts=binary.ts,
    )


class RecoveringGlobalPlanner(GlobalPlanner):
    """Stuck → reverse BACKUP_DISTANCE_M (odometry-measured) → replan."""

    backup_distance_m: float = BACKUP_DISTANCE_M
    backup_speed_mps: float = BACKUP_SPEED_MPS
    backup_timeout_s: float = BACKUP_TIMEOUT_S

    # dimOS PController enforces a 0.2 m/s floor (_min_linear_velocity) that
    # silently overrides any NERF_SPEED cap below 0.2: the 0.149 m/s
    # exploration cap came out as 0.2 on the wheels (seen 25/08 21:06).
    # VECTOR drives cleanly at 0.10 m/s on hard floor (adherence matrix
    # 101-111%, 25/08), so lower the floor and keep the cap real.
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._local_planner._controller._min_linear_velocity = 0.10
        self._local_planner._controller._min_angular_velocity = 0.10

    def _find_wide_path(self, goal: Vector3, robot_pos: Vector3) -> Path | None:
        """Plan on the clearance cost field instead of dimOS's fixed 1.1x
        inflation (global_planner.py:349-363).

        Same three steps as upstream - build a costmap, keep the cells under the
        robot passable, run A* - with our field in place of
        make_gradient_costmap(1.1). Upstream's single 1.1x try is what walled
        off every 60-70 cm doorway; our lethal radius is strictly smaller, so
        this only ever adds routes.
        """
        binary = self._navigation_map.binary_costmap
        costmap = clearance_cost_map(binary)
        self._clear_robot_footprint(costmap, binary, robot_pos)
        path = min_cost_astar(costmap, goal, robot_pos)
        if path and path.poses:
            return path
        return None

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
                self._back_up(trigger="stuck_detector")
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

    def slip(self, direction: float = -1.0, trigger: str = "slip") -> bool:
        """Slip reflex (stuck_guard saw the wheels turn while the lidar pose
        stood still, within 1 s): stop, back off BACKUP_DISTANCE_M in our own
        thread, then ask the monitor for a replan. While the wheels push, the
        odometry slides with them and smears the map (23/08 18:45: 25 s of
        pushing, map lost) - the only cure is to stop within the second.

        `trigger` says who asked (slip / bump / bump_rear) and is carried all
        the way into the back-off log line: until 26/08 a back-off looked the
        same in the run log whether a switch, the slip detector or the stuck
        detector caused it, and that alone made the contact switches look dead.

        Returns False if a recovery is already running."""
        if self._recovering:
            return False
        self._recovering = True
        threading.Thread(target=self._slip_recovery, args=(direction, trigger),
                         daemon=True).start()
        return True

    def _slip_recovery(self, direction: float = -1.0, trigger: str = "slip") -> None:
        try:
            self._back_up(direction, trigger)
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

    def _back_up(self, direction: float = -1.0, trigger: str = "stuck_detector") -> float:
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

        reverse = Twist(linear=Vector3(direction * self.backup_speed_mps, 0.0, 0.0))
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
            trigger=trigger,
            direction="forward" if direction > 0 else "reverse",
            travelled_m=round(travelled, 3),
            seconds=round(time.perf_counter() - t0, 1),
        )
        return travelled


class RecoveringPlanner(ReplanningAStarPlanner):
    """Drop-in for ReplanningAStarPlanner with the back-up recovery, the slip
    reflex (``slip`` In: stuck_guard + imu_slip - the map also rolls back) and
    the bump reflex (``bump`` In: the physical bumper/sonar - same stop and
    20 cm back-off, no rollback: the map was honest, the world was invisible)."""

    slip: In[Bool]
    bump: In[Bool]

    bump_rear: In[Bool]
    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._planner = RecoveringGlobalPlanner(self._planner._global_config)

    async def handle_slip(self, msg: Bool) -> None:
        if getattr(msg, "data", False):
            self._planner.slip(trigger="slip")

    async def handle_bump(self, msg: Bool) -> None:
        """Front contact: stop, back off 0.20 m, replan.

        The arrival is logged here, in the planner's own worker, whatever the
        reflex then decides: a bump that lands while a recovery is already
        running (acted=False) used to leave no trace at all on either side."""
        if getattr(msg, "data", False):
            acted = self._planner.slip(trigger="bump")
            logger.warning("BUMP received: front contact", reflex="back off 0.20 m",
                           acted=acted)

    async def handle_bump_rear(self, msg: Bool) -> None:
        """Rear contact: stop, move FORWARD 0.20 m, replan."""
        if getattr(msg, "data", False):
            acted = self._planner.slip(direction=+1.0, trigger="bump_rear")
            logger.warning("BUMP received: rear contact", reflex="forward 0.20 m",
                           acted=acted)

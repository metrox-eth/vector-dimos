"""ReplanningAStarPlanner that abandons a goal instead of fighting for it.

Doctrine (after the guard-layer autopsy): the detectors
tell the truth, the REFLEXES were the bug. The old blind 20 cm back-off on
every stuck/slip walked the rover backwards across the flat in 20 cm steps
until the rear wall (run 12h54: 38 blind reverses, 14 rear bumper hits, dead
on the wall). A recovery manoeuvre that creates the crash is worse than none.

What is left, and it is all of it:
  * stuck or slip -> STOP, abandon the goal (``cancel_goal`` publishes
    goal_reached=False; the explorer excludes that spot for 60 s and picks
    another frontier — the pivot toward it is the only motion that follows).
    No scripted motion of any kind.
  * a CONTACT (bumper switch, or the explorer's born-cornered back-off) is
    the one thing still allowed a scripted move: away from the contact,
    0.20 m, once. A second contact during that escape aborts it dead —
    contact has priority over everything, it is never ignored again
    (run 12h54: every rear bump during a slip reverse was dropped,
    ``acted=False``, and the rover kept grinding the wall).
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

ESCAPE_DISTANCE_M = 0.20   # just enough to turn
ESCAPE_SPEED_MPS = 0.10
ESCAPE_TIMEOUT_S = 5.0
ESCAPE_PERIOD_S = 0.05  # base watchdog is 0.2 s; 10 Hz left gaps that stopped the wheels

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
#      the upstream "I'd want some gradient from impassable things so that pathing
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
    """Stuck → stop, abandon the goal. Contact → one escape move, then abandon."""

    escape_distance_m: float = ESCAPE_DISTANCE_M
    escape_speed_mps: float = ESCAPE_SPEED_MPS
    escape_timeout_s: float = ESCAPE_TIMEOUT_S

    # The 26/08-morning session lowered the controller floors to 0.10 so the
    # 0.149 m/s exploration cap would be real. The evening run showed the cost:
    # wz flip-flopped at exactly the 0.10 floor, rotations never completed and
    # every goal died (11 issued, 7 abandoned). The 23-25/08 runs, on dimOS's
    # stock 0.2 floors, drove and reached goals - so the floors stay stock.
    # The cap being overridden below 0.2 is the price, measured and accepted.
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self._escape_abort = threading.Event()

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
        # exists; the goal can vanish (explorer cancels on "no path") while an
        # escape runs, and an assert there kills the thread for the rest of
        # the run (seen 23/08: no "Arrived"/"stuck" handled after it).
        try:
            if self._escaping:
                return                     # a contact escape owns the wheels right now
            # The stuck detector reads position only: while the follower turns in
            # place (initial/final rotation) the position does not change and a
            # 2.5 s window reads "stuck" -> replan -> new rotation -> "stuck"...
            # (20 bursts in the kitchen, 23/08 19:35). Rotating is not stuck.
            if not self._in_stop_message and self._local_planner_state() in ("initial_rotation", "final_rotation", "idle"):
                # "idle" added 26/08: while explorer2 WAITS out an exclusion the
                # follower commands nothing - stillness without intent is not
                # "stuck". And the tracker's window must RESTART here (evening
                # 26/08): a mecanum turns in place for seconds, the tracker
                # counted that stillness, and the first blink into
                # path_following fired a replan - new path, sometimes the other
                # way round an obstacle, new rotation, replan again: the 1 Hz
                # bang-bang wiggle that killed both evening flights. Stillness
                # while rotating never feeds the stuck verdict.
                self._position_tracker.reset_data()
                return
            with self._lock:
                has_goal = self._current_goal is not None and self._current_odom is not None
            if not has_goal:
                logger.info("Replan requested without a goal; ignoring.")
                return
            # Stuck counts only if we were actually driving a path for a while:
            # a fresh goal with "no path" also reads as "stuck" (the tracker's
            # window is already still) - seen 23/08 17:00.
            driving_for = time.perf_counter() - self._path_started_at
            if self._position_tracker.is_stuck() and driving_for >= self._stuck_time_window:
                self.abandon_goal(trigger="stuck_detector")
                return
            super()._replan_path()
        except Exception:  # noqa: BLE001 - never let the monitor thread die
            logger.exception("Replan failed; planner monitor keeps running")

    _path_started_at: float = float("inf")
    _escaping: bool = False
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

    def abandon_goal(self, trigger: str) -> None:
        """Stop, give the goal up, no recovery motion of any kind.

        ``cancel_goal`` publishes goal_reached=False; the explorer notes the
        failed spot (60 s exclusion) and picks another frontier. The blind
        20 cm reverse that used to live here is what walked the rover into
        the rear wall 38 times on 26/08 - it is gone on purpose."""
        logger.warning("Goal abandoned, no recovery motion", trigger=trigger)
        self._path_started_at = float("inf")
        self.cancel_goal()

    def escape(self, direction: float, trigger: str) -> bool:
        """Contact escape: the ONE scripted move left. Away from the contact,
        escape_distance_m, once, then the goal is abandoned.

        A second contact while an escape runs means both ends have touched
        something: abort the motion and stop dead rather than script another
        move (run 12h54: rear bumps during a reflex were dropped with
        acted=False and the rover kept grinding the wall).

        Returns False if this contact aborted a running escape instead."""
        if self._escaping:
            self._escape_abort.set()
            logger.warning("Contact during an escape: stopping dead", trigger=trigger)
            return False
        self._escaping = True
        self._escape_abort.clear()
        threading.Thread(target=self._escape_run, args=(direction, trigger),
                         daemon=True).start()
        return True

    _escape_abort: threading.Event

    def _escape_run(self, direction: float, trigger: str) -> None:
        try:
            self._local_planner.stop_planning()
            with self._lock:
                start = self._current_odom
            if start is None:
                return
            twist = Twist(linear=Vector3(direction * self.escape_speed_mps, 0.0, 0.0))
            t0 = time.perf_counter()
            travelled = 0.0
            while time.perf_counter() - t0 < self.escape_timeout_s:
                if self._escape_abort.is_set():
                    break
                with self._lock:
                    odom = self._current_odom
                travelled = odom.position.distance(start.position)
                if travelled >= self.escape_distance_m:
                    break
                self._local_planner.cmd_vel.on_next(twist)
                time.sleep(ESCAPE_PERIOD_S)
            self._local_planner.cmd_vel.on_next(Twist())
            self._position_tracker.reset_data()
            self._path_started_at = float("inf")
            logger.warning(
                "Contact escape",
                trigger=trigger,
                direction="forward" if direction > 0 else "reverse",
                travelled_m=round(travelled, 3),
                seconds=round(time.perf_counter() - t0, 1),
                aborted=self._escape_abort.is_set(),
            )
            self.cancel_goal()
        except Exception:  # noqa: BLE001
            logger.exception("Contact escape failed")
        finally:
            self._escaping = False

    # dimOS's default is 8 s / 0.4 m. "Stuck for eight seconds is already dead
    # for a robot doing 0.3 m/s": 2.5 s, same 5 cm/s floor.
    _stuck_time_window: float = 2.5
    _stuck_threshold: float = 0.12

    def _plan_path(self) -> None:
        super()._plan_path()
        if self._local_planner.get_state() != NavigationState.IDLE:
            self._path_started_at = time.perf_counter()
        else:
            self._path_started_at = float("inf")


class RecoveringPlanner(ReplanningAStarPlanner):
    """Drop-in for ReplanningAStarPlanner where stuck = goal abandoned (no
    motion) and a contact (``bump``/``bump_rear`` In: the physical switches,
    plus the explorer's born-cornered back-off on ``bump``) triggers the one
    scripted escape move away from the contact. The slip detectors and their
    reflex are gone - the contact switches carry that role now."""

    bump: In[Bool]
    bump_rear: In[Bool]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._planner = RecoveringGlobalPlanner(self._planner._global_config)

    async def handle_bump(self, msg: Bool) -> None:
        """Front contact: stop, escape 0.20 m in reverse, abandon the goal."""
        if getattr(msg, "data", False):
            acted = self._planner.escape(direction=-1.0, trigger="bump")
            logger.warning("BUMP received: front contact", reflex="escape reverse 0.20 m",
                           acted=acted)

    async def handle_bump_rear(self, msg: Bool) -> None:
        """Rear contact: stop, escape 0.20 m FORWARD, abandon the goal."""
        if getattr(msg, "data", False):
            acted = self._planner.escape(direction=+1.0, trigger="bump_rear")
            logger.warning("BUMP received: rear contact", reflex="escape forward 0.20 m",
                           acted=acted)

"""Cold bench for RecoveringGlobalPlanner after the 2026-08-26 guard rip.

Doctrine under test: stuck = goal abandoned, NO recovery motion of
any kind - the blind 20 cm reverse walked the rover into the rear wall 38
times in one run. The only scripted move left is the contact escape (bumper
switches), and a second contact aborts it dead instead of being dropped.

No robot. A fake local planner integrates the escape twists it receives into
the odometry the planner reads.
"""

import threading
import time

from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from vector_dimos.recovering_planner import RecoveringGlobalPlanner

FOLLOWER_SPEED_MPS = 0.30   # what the fake follower puts on cmd_vel once started


class FakeLocalPlanner:
    def __init__(self) -> None:
        self.cmd_vel: Subject = Subject()
        self.stopped = 0
        self.starts = 0
        self.events: list[str] = []
        self.sent: list[Twist] = []
        self._lock = threading.Lock()
        self._state = "path_following"

    def stop_planning(self) -> None:
        self.stopped += 1
        self.events.append("stop")

    def start_planning(self, path) -> None:
        # the real follower drives cmd_vel from here on; one forward twist is
        # enough to show up as a second writer on the escape's Subject
        self.starts += 1
        self.events.append("start")
        self.cmd_vel.on_next(Twist(linear=Vector3(FOLLOWER_SPEED_MPS, 0.0, 0.0)))

    def get_state(self):
        from dimos.navigation.base import NavigationState
        return NavigationState.IDLE


def make_planner(moves: bool):
    planner = RecoveringGlobalPlanner(GlobalConfig())
    fake = FakeLocalPlanner()
    planner._local_planner = fake
    planner._current_odom = PoseStamped(position=Vector3(2.0, 1.0, 0.0))
    last = [time.perf_counter()]

    def integrate(tw: Twist) -> None:
        fake.sent.append(tw)
        now = time.perf_counter()
        dt, last[0] = now - last[0], now
        if moves and tw.linear.x != 0.0:
            with planner._lock:
                p = planner._current_odom.position
                planner._current_odom = PoseStamped(
                    position=Vector3(p.x + tw.linear.x * dt, p.y, 0.0)
                )

    fake.cmd_vel.subscribe(integrate)
    return planner, fake


def straight_path(start: Vector3, goal: Vector3) -> Path:
    return Path(poses=[
        PoseStamped(position=Vector3(start.x + (goal.x - start.x) * f,
                                     start.y + (goal.y - start.y) * f, 0.0))
        for f in (0.0, 0.5, 1.0)
    ])


def plannable(planner) -> None:
    """No costmap on a cold bench: every goal gets a straight path."""
    planner._find_safe_goal = lambda goal: goal
    planner._find_wide_path = lambda goal, robot: straight_path(robot, goal)


def wait_until(predicate, timeout: float = 4.0) -> bool:
    t0 = time.perf_counter()
    while not predicate() and time.perf_counter() - t0 < timeout:
        time.sleep(0.02)
    return predicate()


def wait_escape_done(planner, timeout: float = 8.0) -> None:
    t0 = time.perf_counter()
    while planner._escaping and time.perf_counter() - t0 < timeout:
        time.sleep(0.02)
    assert not planner._escaping, "escape never finished"


def test_stuck_abandons_goal_without_any_motion() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0   # has been driving a path
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    planner._replan_path()
    assert fake.sent == [], "stuck must not command ANY motion"
    assert planner._current_goal is None, "the goal must be abandoned"
    assert failed == [False], "goal_reached=False is what makes the explorer exclude the spot"
    print("  stuck -> zero twists, goal abandoned, goal_reached=False published")


def test_stuck_needs_a_driven_path_and_ignores_rotation() -> None:
    calls = []
    planner, fake = make_planner(moves=True)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    RecoveringGlobalPlanner.__mro__[1]._replan_path = lambda self: calls.append("replan")  # type: ignore[method-assign]

    planner._path_started_at = float("inf")                 # fresh goal, no path driven yet
    planner._replan_path()
    assert calls == ["replan"] and planner._current_goal is not None, calls
    print("  stuck but not driving a path -> normal replan, goal kept")

    calls.clear()
    planner._path_started_at = time.perf_counter() - 30.0
    fake._state = "initial_rotation"                         # turning in place: position still, not stuck
    resets = []
    planner._position_tracker.reset_data = lambda: resets.append(1)  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == [] and planner._current_goal is not None, calls
    assert resets, "rotation must RESTART the stuck tracker window (the 1 Hz wiggle, 26/08 evening)"
    planner._in_stop_message = True                          # but a follower stop message is handled
    planner._replan_path()
    planner._in_stop_message = False
    assert planner._current_goal is None, "stuck while driving (stop message path) -> abandoned"
    fake._state = "path_following"
    print("  rotating in place -> ignored; follower stop message -> handled")


def test_escape_moves_away_then_abandons() -> None:
    planner, fake = make_planner(moves=True)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    t0 = time.perf_counter()
    assert planner.escape(direction=-1.0, trigger="bump") is True
    wait_escape_done(planner)
    dt = time.perf_counter() - t0
    assert fake.stopped >= 1, "local follower must be stopped before the escape"
    assert all(tw.linear.x <= 0.0 for tw in fake.sent), "front bump escapes in REVERSE"
    assert fake.sent[-1].linear.x == 0.0, "must end with a zero twist"
    assert planner._current_odom.position.x < 1.85, "must have moved ~0.20 m away"
    assert 1.5 < dt < 4.0, dt
    assert planner._current_goal is None and failed == [False], "escape ends by abandoning the goal"
    print(f"  bump -> escaped {2.0 - planner._current_odom.position.x:.2f} m in {dt:.1f} s, goal abandoned")


def test_escape_times_out_when_not_moving() -> None:
    planner, fake = make_planner(moves=False)
    planner.escape_timeout_s = 1.0
    assert planner.escape(direction=+1.0, trigger="bump_rear") is True
    wait_escape_done(planner)
    assert all(tw.linear.x >= 0.0 for tw in fake.sent), "rear bump escapes FORWARD"
    assert fake.sent[-1].linear.x == 0.0
    print("  blocked escape times out and ends with a zero twist")


def test_second_contact_aborts_the_escape() -> None:
    planner, fake = make_planner(moves=False)     # never reaches 0.20 m on its own
    planner.escape_timeout_s = 5.0
    t0 = time.perf_counter()
    assert planner.escape(direction=-1.0, trigger="bump") is True
    time.sleep(0.15)
    assert planner.escape(direction=+1.0, trigger="bump_rear") is False, \
        "a second contact must ABORT, not start a new scripted move"
    wait_escape_done(planner)
    dt = time.perf_counter() - t0
    assert dt < 2.0, f"abort must stop the escape early, took {dt:.1f} s"
    assert fake.sent[-1].linear.x == 0.0
    print(f"  second contact aborted the escape after {dt:.2f} s, wheels zeroed")


def test_stuck_is_silent_during_an_escape() -> None:
    planner, fake = make_planner(moves=False)
    planner.escape_timeout_s = 1.0
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    assert planner.escape(direction=-1.0, trigger="bump") is True
    planner._replan_path()          # the monitor fires mid-escape
    assert planner._escaping, "the escape must keep the wheels"
    wait_escape_done(planner)
    print("  monitor stuck check during an escape -> ignored")


def test_goal_during_an_escape_is_held_then_planned() -> None:
    """S1 (audit 28/08): the explorer wakes 3 s into a 5 s escape and sends a
    goal. Two writers on cmd_vel, then the escape cancelled the fresh goal and
    published goal_reached=False - the explorer excluded a frontier it never
    drove to. Same timeline here, 1 s escape."""
    planner, fake = make_planner(moves=False)      # wedged: the escape runs its full timeout
    plannable(planner)
    failed = []
    planner.goal_reached.subscribe(lambda b: failed.append(bool(b.data)))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))   # the goal being driven
    planner.escape_timeout_s = 1.0

    assert planner.escape(direction=-1.0, trigger="bump") is True
    time.sleep(0.3)                                # mid-escape, as the explorer wakes up
    frontier = PoseStamped(position=Vector3(6.0, 1.0, 0.0))
    planner.handle_goal_request(frontier)

    assert fake.starts == 0, "the follower must NOT be started while the escape owns the wheels"
    assert planner._held_goal is frontier, "the goal is held, not dropped"
    time.sleep(0.2)                                # let the escape keep writing
    assert [tw for tw in fake.sent if tw.linear.x > 0.0] == [], \
        "no forward follower twist may interleave with the reverse escape"

    wait_escape_done(planner)
    assert wait_until(lambda: fake.starts == 1), "the held goal must be planned once the escape ends"
    assert planner._current_goal is frontier, "the held goal becomes the current goal"
    assert planner._held_goal is None
    assert failed == [], "no goal_reached=False for a goal the rover never drove"
    assert fake.events.count("start") == 1 and fake.events[-1] == "start"
    print("  goal mid-escape -> zero follower twists during the escape, planned after, no false failure")


def test_goal_outside_an_escape_is_planned_at_once() -> None:
    planner, fake = make_planner(moves=False)
    plannable(planner)
    goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner.handle_goal_request(goal)
    assert fake.starts == 1 and planner._current_goal is goal, "unchanged outside an escape"
    assert getattr(planner, "_held_goal", None) is None   # getattr: this case also runs pre-fix
    assert [round(tw.linear.x, 2) for tw in fake.sent] == [FOLLOWER_SPEED_MPS], \
        "the follower is alone on cmd_vel"
    print("  goal outside an escape -> planned immediately, follower alone on cmd_vel")


def test_a_plan_racing_the_escape_start_never_leaves_the_follower_running() -> None:
    planner, fake = make_planner(moves=False)
    plannable(planner)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))

    planner._escaping = True                        # the escape took the wheels first
    planner._plan_path()
    assert fake.starts == 0, "a replan during an escape must not start the follower"
    assert planner._held_goal is planner._current_goal

    planner._escaping = False                       # ... and the other way round:
    planner._held_goal = None
    def contact_while_astar_runs(goal, robot):      # the bumper fires while A* runs
        planner._escaping = True
        return straight_path(robot, goal)
    planner._find_wide_path = contact_while_astar_runs
    planner._plan_path()
    assert fake.events[-1] == "stop", "a follower started under a contact must be stopped again"
    assert planner._held_goal is planner._current_goal
    planner._escaping = False
    print("  plan racing an escape -> follower never left running, goal held")


if __name__ == "__main__":
    for t in (test_stuck_abandons_goal_without_any_motion,
              test_stuck_needs_a_driven_path_and_ignores_rotation,
              test_escape_moves_away_then_abandons,
              test_escape_times_out_when_not_moving,
              test_second_contact_aborts_the_escape,
              test_stuck_is_silent_during_an_escape,
              test_goal_during_an_escape_is_held_then_planned,
              test_goal_outside_an_escape_is_planned_at_once,
              test_a_plan_racing_the_escape_start_never_leaves_the_follower_running):
        print(t.__name__); t()
    print("TEST PASSED")

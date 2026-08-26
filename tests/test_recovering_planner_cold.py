"""Cold bench for RecoveringGlobalPlanner after the 26/08 guard rip.

Doctrine under test (metrox): stuck = goal abandoned, NO recovery motion of
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
from vector_dimos.recovering_planner import RecoveringGlobalPlanner


class FakeLocalPlanner:
    def __init__(self) -> None:
        self.cmd_vel: Subject = Subject()
        self.stopped = 0
        self.sent: list[Twist] = []
        self._lock = threading.Lock()
        self._state = "path_following"

    def stop_planning(self) -> None:
        self.stopped += 1

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
    planner._replan_path()
    assert calls == [] and planner._current_goal is not None, calls
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


if __name__ == "__main__":
    for t in (test_stuck_abandons_goal_without_any_motion,
              test_stuck_needs_a_driven_path_and_ignores_rotation,
              test_escape_moves_away_then_abandons,
              test_escape_times_out_when_not_moving,
              test_second_contact_aborts_the_escape,
              test_stuck_is_silent_during_an_escape):
        print(t.__name__); t()
    print("TEST PASSED")

"""Cold bench for RecoveringGlobalPlanner: known odometry in -> known reverse out (0.20 m, metrox: just enough to turn).

No robot. A fake local planner integrates the reverse twists it receives into
the odometry the planner reads, so "back up 0.25 m at 0.10 m/s" must take
~2.5 s, stop, and leave the wheels with a zero twist.
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


def test_backs_up_measured_distance() -> None:
    planner, fake = make_planner(moves=True)
    t0 = time.perf_counter()
    travelled = planner._back_up()
    dt = time.perf_counter() - t0
    assert fake.stopped == 1, "local follower must be stopped before reversing"
    assert 0.20 <= travelled < 0.25, travelled
    assert 2.0 < dt < 3.5, dt
    assert all(tw.linear.x <= 0.0 for tw in fake.sent)
    assert fake.sent[-1].linear.x == 0.0, "must end with a zero twist"
    assert planner._current_odom.position.x < 1.85
    print(f"  backed up {travelled:.3f} m in {dt:.1f} s, {len(fake.sent)} twists")


def test_times_out_when_not_moving() -> None:
    planner, fake = make_planner(moves=False)
    planner.backup_timeout_s = 1.0
    t0 = time.perf_counter()
    travelled = planner._back_up()
    dt = time.perf_counter() - t0
    assert travelled == 0.0
    assert 0.9 < dt < 1.5, dt
    assert fake.sent[-1].linear.x == 0.0
    print(f"  timed out after {dt:.1f} s with zero twist")


def test_replan_backs_up_only_when_stuck() -> None:
    calls = []
    planner, fake = make_planner(moves=True)
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0   # has been driving a path
    planner._back_up = lambda: calls.append("backup") or 0.25  # type: ignore[method-assign]
    RecoveringGlobalPlanner.__mro__[1]._replan_path = lambda self: calls.append("replan")  # type: ignore[method-assign]
    planner._position_tracker.is_stuck = lambda: False  # type: ignore[method-assign]
    planner._replan_path()
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == ["replan", "backup", "replan"], calls
    print(f"  order: {calls}")
    calls.clear()
    planner._path_started_at = float("inf")                 # fresh goal, no path driven yet
    planner._replan_path()
    assert calls == ["replan"], calls
    print("  stuck but not driving a path -> no back-up")
    calls.clear()
    planner._path_started_at = time.perf_counter() - 30.0
    fake._state = "initial_rotation"                         # turning in place: position still, not stuck
    planner._replan_path()
    assert calls == [], calls
    planner._in_stop_message = True                          # but a follower stop message still replans
    planner._replan_path()
    planner._in_stop_message = False
    assert calls == ["backup", "replan"], calls
    fake._state = "path_following"
    print("  rotating in place -> ignored by the stuck detector; follower stop message -> handled")


def test_replan_without_goal_neither_backs_up_nor_dies() -> None:
    calls = []
    planner, fake = make_planner(moves=True)
    planner._current_goal = None
    planner._back_up = lambda: calls.append("backup") or 0.25  # type: ignore[method-assign]
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == [], calls

    def cancel_during_backup() -> float:
        calls.append("backup")
        planner._current_goal = None
        return 0.25

    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._path_started_at = time.perf_counter() - 30.0
    planner._back_up = cancel_during_backup  # type: ignore[method-assign]
    RecoveringGlobalPlanner.__mro__[1]._replan_path = lambda self: calls.append("replan")  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == ["backup"], calls

    def boom(self) -> None:
        raise AssertionError("upstream assert")

    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    planner._position_tracker.is_stuck = lambda: False  # type: ignore[method-assign]
    RecoveringGlobalPlanner.__mro__[1]._replan_path = boom  # type: ignore[method-assign]
    planner._replan_path()  # must not raise
    print("  no goal -> no backup; goal lost during backup -> no replan; upstream exception swallowed")


if __name__ == "__main__":
    for t in (test_backs_up_measured_distance, test_times_out_when_not_moving, test_replan_backs_up_only_when_stuck, test_replan_without_goal_neither_backs_up_nor_dies):
        print(t.__name__); t()
    print("TEST PASSED")

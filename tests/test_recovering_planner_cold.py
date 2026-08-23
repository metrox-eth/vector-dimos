"""Cold bench for RecoveringGlobalPlanner: known odometry in -> known reverse out.

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
    assert 0.25 <= travelled < 0.30, travelled
    assert 2.0 < dt < 3.5, dt
    assert all(tw.linear.x <= 0.0 for tw in fake.sent)
    assert fake.sent[-1].linear.x == 0.0, "must end with a zero twist"
    assert planner._current_odom.position.x < 1.80
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
    planner._back_up = lambda: calls.append("backup") or 0.25  # type: ignore[method-assign]
    RecoveringGlobalPlanner.__mro__[1]._replan_path = lambda self: calls.append("replan")  # type: ignore[method-assign]
    planner._position_tracker.is_stuck = lambda: False  # type: ignore[method-assign]
    planner._replan_path()
    planner._position_tracker.is_stuck = lambda: True  # type: ignore[method-assign]
    planner._replan_path()
    assert calls == ["replan", "backup", "replan"], calls
    print(f"  order: {calls}")


if __name__ == "__main__":
    for t in (test_backs_up_measured_distance, test_times_out_when_not_moving, test_replan_backs_up_only_when_stuck):
        print(t.__name__); t()
    print("TEST PASSED")

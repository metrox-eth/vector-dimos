"""Cold bench for the slip reflex chain (no robot):
  * stuck_guard: wheels 0.20 m/s, lidar still -> trips within ~1.2 s and publishes slip;
    honest motion and a slow start never trip
  * RecoveringGlobalPlanner.slip(): backs off 0.20 m (odometry-measured), ends with a
    zero twist, requests a replan; a second slip during the reflex is ignored
  * LidarOdometry prior: after a slip, the wheel prior is frozen to identity for
    SLIP_HOLD_S seconds (tested on the gate, not on kiss-icp)
"""

import asyncio
import time
from collections import deque

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos_lcm.std_msgs import Bool
from reactivex import Subject

from vector_dimos.recovering_planner import RecoveringGlobalPlanner
from vector_dimos.stuck_guard import StuckGuard
from dimos.core.global_config import GlobalConfig


class Probe:
    def __init__(self) -> None:
        self.msgs = []

    def publish(self, msg) -> None:
        self.msgs.append(msg)


def guard_scenario(wheel_speed: float, lidar_speed: float, seconds: float = 2.5):
    g = StuckGuard.__new__(StuckGuard)
    g._wheel = deque(maxlen=400); g._lidar = deque(maxlen=400); g._cmd = deque(maxlen=400)
    g._last_check = 0.0; g._last_trip = 0.0; g._last_debug = 0.0; g.trips = 0
    g.world_frame = "world"; g.lidar = Probe(); g.slip = Probe()
    t_trip = [None]

    async def feed() -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            t = time.monotonic() - t0
            await g.handle_odom(PoseStamped(position=Vector3(1.0 + lidar_speed * t, 2.0, 0.0)))
            g._wheel.append((time.monotonic(), 5.0 + wheel_speed * t, 5.0, 0.0))
            g._check(time.monotonic())
            if g.trips and t_trip[0] is None:
                t_trip[0] = t
            await asyncio.sleep(0.05)

    asyncio.run(feed())
    return g.trips, len(g.slip.msgs), t_trip[0]


def test_guard_trips_fast_and_publishes_slip() -> None:
    trips, slips, t = guard_scenario(0.20, 0.0)
    # detection needs one full comparison window: bound = WINDOW_S + a scheduler margin
    assert trips >= 1 and slips == trips and t is not None and t < 2.0, (trips, slips, t)
    print(f"  wheels 0.20 m/s, lidar still -> slip published after {t:.1f} s")
    assert guard_scenario(0.20, 0.19)[0] == 0
    assert guard_scenario(0.03, 0.01)[0] == 0
    print("  honest motion / slow start -> no slip")


class FakeLocalPlanner:
    def __init__(self) -> None:
        self.cmd_vel: Subject = Subject(); self.sent = []; self.stopped = 0

    def stop_planning(self) -> None:
        self.stopped += 1


def test_planner_slip_reflex_backs_off_20cm() -> None:
    planner = RecoveringGlobalPlanner(GlobalConfig())
    fake = FakeLocalPlanner(); planner._local_planner = fake
    planner._current_odom = PoseStamped(position=Vector3(2.0, 1.0, 0.0))
    planner._current_goal = PoseStamped(position=Vector3(4.0, 1.0, 0.0))
    last = [time.perf_counter()]

    def integrate(tw: Twist) -> None:
        fake.sent.append(tw); now = time.perf_counter(); dt, last[0] = now - last[0], now
        if tw.linear.x != 0.0:
            with planner._lock:
                p = planner._current_odom.position
                planner._current_odom = PoseStamped(position=Vector3(p.x + tw.linear.x * dt, p.y, 0.0))
    fake.cmd_vel.subscribe(integrate)
    assert planner.slip() is True and planner.slip() is False
    t0 = time.perf_counter()
    while planner._recovering and time.perf_counter() - t0 < 8:
        time.sleep(0.05)
    x = planner._current_odom.position.x
    assert 1.74 <= x <= 1.81, x
    assert fake.sent[-1].linear.x == 0.0 and fake.stopped == 1
    assert planner._replan_event.is_set() and planner._replan_reason == "obstacle_found"
    print(f"  slip -> stopped, backed off to x={x:.2f} (0.20 m), replan requested")


def test_odometry_prior_frozen_after_slip() -> None:
    from vector_dimos import lidar_odometry as lo
    class Stub:
        _slip_until = 0.0
    st = Stub()
    asyncio.run(lo.LidarOdometry.handle_slip(st, Bool(data=True)))
    assert lo.LidarOdometry.slipping.fget(st) is True
    st._slip_until = time.monotonic() - 0.1
    assert lo.LidarOdometry.slipping.fget(st) is False
    print(f"  slip -> wheels not believed / map not written for {lo.SLIP_HOLD_S:.0f} s, then back to normal")


if __name__ == "__main__":
    for t in (test_guard_trips_fast_and_publishes_slip, test_planner_slip_reflex_backs_off_20cm, test_odometry_prior_frozen_after_slip):
        print(t.__name__); t()
    print("TEST PASSED")

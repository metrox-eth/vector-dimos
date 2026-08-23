"""Cold bench for StuckGuard (no robot): the virtual bumper trips only when the
wheels turn for real (>= 0.08 m in 1 s) while the lidar pose stays put, and
only once that has held for 2 s. A slow start (wheels 3 cm, lidar 1 cm) and
honest motion (wheels = lidar) never trip - those were the false alarms that
made the rover reverse all afternoon on 23/08."""

import asyncio
import time
from collections import deque

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from vector_dimos.stuck_guard import StuckGuard, WINDOW_S


class Probe:
    def __init__(self) -> None:
        self.clouds = []

    def publish(self, msg) -> None:
        self.clouds.append(msg)


def run_scenario(wheel_speed: float, lidar_speed: float, seconds: float = 3.5) -> int:
    guard = StuckGuard.__new__(StuckGuard)
    guard._wheel = deque(maxlen=400); guard._lidar = deque(maxlen=400); guard._cmd = deque(maxlen=400)
    guard._last_check = 0.0; guard._last_trip = 0.0; guard._last_debug = 0.0; guard.trips = 0; guard._blocked_since = 0.0
    guard.world_frame = "world"
    guard.lidar = Probe()

    async def feed() -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            t = time.monotonic() - t0
            await guard.handle_odom(PoseStamped(position=Vector3(1.0 + lidar_speed * t, 2.0, 0.0)))
            guard._wheel.append((time.monotonic(), 5.0 + wheel_speed * t, 5.0, 0.0))
            guard._check(time.monotonic())
            await asyncio.sleep(0.05)

    asyncio.run(feed())
    return guard.trips


def test_wheels_turning_lidar_still_trips_after_2s() -> None:
    trips = run_scenario(wheel_speed=0.20, lidar_speed=0.0)
    assert trips >= 1, trips
    print(f"  wheels 0.20 m/s, lidar still for 3.5 s -> {trips} trip(s)")


def test_honest_motion_does_not_trip() -> None:
    assert run_scenario(wheel_speed=0.20, lidar_speed=0.19) == 0
    print("  wheels 0.20 m/s, lidar 0.19 m/s -> no trip")


def test_slow_start_does_not_trip() -> None:
    assert run_scenario(wheel_speed=0.03, lidar_speed=0.01) == 0
    print("  slow start (wheels 0.03 m/s, lidar 0.01 m/s) -> no trip")


def test_short_block_under_2s_does_not_trip() -> None:
    assert run_scenario(wheel_speed=0.20, lidar_speed=0.0, seconds=1.6) == 0
    print("  block lasting 1.6 s -> no trip yet")


if __name__ == "__main__":
    for t in (test_wheels_turning_lidar_still_trips_after_2s, test_honest_motion_does_not_trip,
              test_slow_start_does_not_trip, test_short_block_under_2s_does_not_trip):
        print(t.__name__); t()
    print("TEST PASSED")

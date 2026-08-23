"""Cold bench for StuckGuard's commanded-motion path (no robot).

Known input -> known output: a path-follower twist (vx 0.24 m/s with its
usual yaw correction wz 0.2) while the lidar pose does not move must trip
the virtual bumper; a pure spin (vx 0, wz 0.3) must not.
"""

import asyncio
import time

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from vector_dimos.stuck_guard import StuckGuard, WINDOW_S


class Probe:
    def __init__(self) -> None:
        self.clouds = []

    def publish(self, msg) -> None:
        self.clouds.append(msg)


def run_scenario(vx: float, wz: float) -> int:
    guard = StuckGuard.__new__(StuckGuard)
    StuckGuard.__init__.__wrapped__(guard) if hasattr(StuckGuard.__init__, "__wrapped__") else None
    # Bypass Module plumbing: only the buffers and the check are exercised.
    from collections import deque
    guard._wheel = deque(maxlen=400); guard._lidar = deque(maxlen=400); guard._cmd = deque(maxlen=400)
    guard._last_check = 0.0; guard._last_trip = 0.0; guard._last_debug = 0.0; guard.trips = 0
    guard.world_frame = "world"
    guard.lidar = Probe()
    guard.bump = Probe()

    async def feed() -> None:
        t_end = time.monotonic() + WINDOW_S + 0.6
        while time.monotonic() < t_end:
            await guard.handle_odom(PoseStamped(position=Vector3(1.0, 2.0, 0.0)))
            guard._wheel.append((time.monotonic(), 5.0, 5.0, 0.0))   # stalled wheels (wheel odom frozen)
            await guard.handle_cmd_vel(Twist(linear=Vector3(vx, 0.0, 0.0), angular=Vector3(0.0, 0.0, wz)))
            guard._check(time.monotonic())
            await asyncio.sleep(0.05)

    asyncio.run(feed())
    return guard.trips


def test_forward_with_yaw_correction_trips() -> None:
    trips = run_scenario(vx=0.24, wz=0.2)
    assert trips >= 1, trips
    print(f"  vx 0.24 + wz 0.2, lidar still -> {trips} trip(s)")


def test_pure_spin_does_not_trip() -> None:
    trips = run_scenario(vx=0.0, wz=0.3)
    assert trips == 0, trips
    print("  pure spin -> no trip")


if __name__ == "__main__":
    for t in (test_forward_with_yaw_correction_trips, test_pure_spin_does_not_trip):
        print(t.__name__); t()
    print("TEST PASSED")

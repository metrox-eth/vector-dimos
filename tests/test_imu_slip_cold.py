"""Cold bench for ImuSlipDetector - known streams in, known trips out.

Timing target: the detector must fire within ~0.5 s of a slip, against the
~1.2 s of the lidar guard (and the cases the lidar guard cannot see at all:
wheels in the air, rotation slip, dragged odometry).
"""

import asyncio
import time

from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from vector_dimos.imu_slip import ImuSlipDetector


class Probe:
    def __init__(self) -> None:
        self.msgs = []

    def publish(self, msg) -> None:
        self.msgs.append(msg)


def make() -> ImuSlipDetector:
    d = ImuSlipDetector.__new__(ImuSlipDetector)
    from collections import deque
    d._gyro = deque(maxlen=256); d._accel = deque(maxlen=256); d._wheel_v = deque(maxlen=128)
    d._mismatch_since = 0.0; d._last_trip = 0.0; d.trips = 0
    d.slip = Probe()
    return d


def joint(vx: float, wz: float) -> JointState:
    return JointState(name=["base/vx", "base/vy", "base/wz"], velocity=[vx, 0.0, wz])


def imu(body_wz: float, body_forward_accel: float) -> Imu:
    # body wz -> optical gyro.y = -wz ; body forward accel -> optical accel.z
    return Imu(angular_velocity=Vector3(0.0, -body_wz, 0.0),
               linear_acceleration=Vector3(0.0, 0.0, body_forward_accel))


def run(seconds: float, wheels, body):
    """wheels(t) -> (vx, wz); body(t) -> (wz, forward_accel). 100 Hz imu, 20 Hz wheels."""
    d = make()
    t_trip = [None]

    async def feed() -> None:
        t0 = time.monotonic()
        k = 0
        while time.monotonic() - t0 < seconds:
            t = time.monotonic() - t0
            await d.handle_imu(imu(*body(t)))
            if k % 5 == 0:
                await d.handle_coordinator_joint_state(joint(*wheels(t)))
            if d.trips and t_trip[0] is None:
                t_trip[0] = t
            k += 1
            await asyncio.sleep(0.01)

    asyncio.run(feed())
    return d.trips, t_trip[0]


def test_rotation_slip_detected_fast() -> None:
    # wheels claim a 0.5 rad/s spin, the body does not rotate (pushing a table corner)
    trips, t = run(2.0, wheels=lambda t: (0.0, 0.5), body=lambda t: (0.0, 0.0))
    assert trips >= 1 and t is not None and t < 0.8, (trips, t)
    print(f"  rotation slip -> detected in {t:.2f} s")


def test_honest_rotation_no_trip() -> None:
    trips, _ = run(1.5, wheels=lambda t: (0.0, 0.5), body=lambda t: (0.5, 0.0))
    assert trips == 0
    print("  honest rotation -> no trip")


def test_translation_onset_slip_detected() -> None:
    # wheels ramp 0 -> 0.25 m/s in 0.4 s; the body never accelerates (wheels in the air)
    trips, t = run(1.6, wheels=lambda t: (min(0.25, 0.6 * t), 0.0), body=lambda t: (0.0, 0.0))
    assert trips >= 1 and t is not None and t < 1.0, (trips, t)
    print(f"  spin-up with no body acceleration -> detected in {t:.2f} s")


def test_honest_acceleration_no_trip() -> None:
    # wheels ramp, body accelerates to match (0.6 m/s2 during the ramp)
    trips, _ = run(1.6, wheels=lambda t: (min(0.25, 0.6 * t), 0.0),
                   body=lambda t: (0.0, 0.6 if t < 0.42 else 0.0))
    assert trips == 0
    print("  honest spin-up -> no trip")


def test_constant_speed_stays_quiet() -> None:
    # steady rolling: constant wheel speed, zero accel - not this detector's case
    trips, _ = run(1.2, wheels=lambda t: (0.25, 0.0), body=lambda t: (0.0, 0.0))
    assert trips == 0
    print("  steady rolling (or steady slide) -> quiet here, stays with the lidar guard")


if __name__ == "__main__":
    for t in (test_rotation_slip_detected_fast, test_honest_rotation_no_trip,
              test_translation_onset_slip_detected, test_honest_acceleration_no_trip,
              test_constant_speed_stays_quiet):
        print(t.__name__); t()
    print("TEST PASSED")

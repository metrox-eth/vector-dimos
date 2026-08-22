"""Mecanum kinematics and the real VECTOR topology.

Standard X-configuration mecanum equations (45-degree rollers). SI units.
Wheel order everywhere in this package: FL, FR, BL, BR.

Bus topology (from the first robot code by Sam, proven on the chassis):
  - Controller ID 1 = BACK  (L port = BL, R port = BR)
  - Controller ID 2 = FRONT (L port = FL, R port = FR)
  - LEFT ports (FL, BL) are inverted: negative RPM = forward.

Wheel radius resolved 2026-08-21 (metrox): 0.105 m, the value the first robot
code (Sam's driver, R_Wheel) ran on this chassis. The built mecanum wheel is
NOMINALLY 8", which is not the same number - 8 in = 0.2032 m = 0.1016 m radius,
and 0.105 m is 8.27 in - so the nominal size is where 0.105 comes from, not a
confirmation of it. The old 0.0635 m was the bare 5" tire BEFORE the mecanum
was built around it - stale. Validate with an odometry roundtrip (drive a known
distance, compare /odom) before trusting odometry: radius scales it linearly.
"""
import math
from dataclasses import dataclass

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius_m: float = 0.105    # Sam's R_Wheel; nominal 8" would be 0.1016 - close, not equal. Confirm via odometry roundtrip.
    half_wheelbase_m: float = 0.15   # v1: wheelbase 0.3
    half_track_m: float = 0.20       # v1: track 0.4

    @property
    def k(self) -> float:
        return self.half_wheelbase_m + self.half_track_m


def inverse(vx: float, vy: float, wz: float, g: MecanumGeometry):
    """Body twist -> wheel angular velocities [rad/s] (FL, FR, BL, BR)."""
    r, k = g.wheel_radius_m, g.k
    return (
        (vx - vy - k * wz) / r,
        (vx + vy + k * wz) / r,
        (vx + vy - k * wz) / r,
        (vx - vy + k * wz) / r,
    )


def forward(w_fl: float, w_fr: float, w_bl: float, w_br: float,
            g: MecanumGeometry):
    """Wheel angular velocities [rad/s] -> body twist (vx, vy, wz)."""
    r, k = g.wheel_radius_m, g.k
    vx = r / 4.0 * (w_fl + w_fr + w_bl + w_br)
    vy = r / 4.0 * (-w_fl + w_fr + w_bl - w_br)
    wz = r / (4.0 * k) * (-w_fl + w_fr - w_bl + w_br)
    return vx, vy, wz


def rads_to_rpm(w: float) -> float:
    return w * RPM_PER_RAD_S


def rpm_to_rads(rpm: float) -> float:
    return rpm / RPM_PER_RAD_S

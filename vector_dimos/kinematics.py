"""Mecanum kinematics and the real VECTOR topology.

Standard X-configuration mecanum equations (45-degree rollers). SI units.
Wheel order everywhere in this package: FL, FR, BL, BR.

Bus topology (from the first robot code by Sam, proven on the chassis):
  - Controller ID 1 = BACK  (L port = BL, R port = BR)
  - Controller ID 2 = FRONT (L port = FL, R port = FR)
  - LEFT ports (FL, BL) are inverted: negative RPM = forward.

Wheel radius: 0.085 m - the built mecanum wheel MEASURED on the chassis
(metrox, 2026-08-22): 17 cm diameter, roller tips included. Two other numbers
exist in the first robot code and both are wrong for this wheel: 0.0635 m is
the bare 5" tire before the mecanum was built around it, and 0.105 m (R_Wheel)
was a dead constant that never fed the mecanum path. Until 2026-08-22 this
package shipped 0.105 and overestimated odometry by ~24 %. Validate with an
odometry roundtrip (drive a known distance, compare /odom) before trusting
odometry: radius scales it linearly.
"""
import math
from dataclasses import dataclass

RPM_PER_RAD_S = 60.0 / (2.0 * math.pi)


@dataclass(frozen=True)
class MecanumGeometry:
    wheel_radius_m: float = 0.085    # measured: 17 cm diameter (2026-08-22). Confirm via odometry roundtrip.
    half_wheelbase_m: float = 0.15   # measured 2026-08-23 (metrox, tape): axle to axle 30 cm
    half_track_m: float = 0.185      # measured 2026-08-23: 37 cm between the ground contact points (v1 said 0.40 - never measured)

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

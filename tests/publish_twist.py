#!/usr/bin/env python3
"""Publish a Twist on /cmd_vel and print the wheel RPM it should produce.

Live proof tool, not a cold test: it needs a coordinator already running
(`dimos run vector-dimos.base`). It publishes from a SEPARATE process, over
the same LCM transport the runtime uses, so what comes out the other end is
the real path: LCM -> ControlCoordinator.twist_command -> JointVelocityTask
-> VectorBaseAdapter.write_velocities -> per-wheel RPM on the MODBUS bus.

Run the base with VECTOR_MOCK_BUS=1 and the adapter logs the RPM it would
have written; compare it with the expectation this script prints:

    $ VECTOR_MOCK_BUS=1 dimos run vector-dimos.base --no-local-relay --daemon
    $ python tests/publish_twist.py --vx 0.37 --vy -0.21 --wz 0.83
    $ dimos log -n 20

Why it streams instead of sending one message: JointVelocityTask drops the
command to zero if it goes ~0.2 s without an update, so a single Twist buys
about 200 ms of motion. Anything driving this base has to keep publishing;
the default 20 Hz is comfortably inside that window.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.kinematics import MecanumGeometry, inverse, rads_to_rpm

DEFAULT_TOPIC = "/cmd_vel"


def expected_rpm(vx: float, vy: float, wz: float,
                 geometry: MecanumGeometry) -> tuple[int, int, int, int]:
    """(FL, FR, BL, BR) integer RPM the adapter should command for this twist."""
    return tuple(round(rads_to_rpm(w))            # type: ignore[return-value]
                 for w in inverse(vx, vy, wz, geometry))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--vx", type=float, default=0.37, help="m/s, forward")
    p.add_argument("--vy", type=float, default=-0.21, help="m/s, left")
    p.add_argument("--wz", type=float, default=0.83, help="rad/s, yaw")
    p.add_argument("--duration", type=float, default=1.0, help="seconds")
    p.add_argument("--rate", type=float, default=20.0, help="Hz (keep > 5)")
    p.add_argument("--topic", default=DEFAULT_TOPIC)
    args = p.parse_args()

    geometry = MecanumGeometry()
    fl, fr, bl, br = expected_rpm(args.vx, args.vy, args.wz, geometry)
    print(f"geometry: {geometry}")
    print(f"twist vx={args.vx:+.3f} vy={args.vy:+.3f} wz={args.wz:+.3f}")
    print(f"expected wheel RPM  FL={fl:+d} FR={fr:+d} BL={bl:+d} BR={br:+d}")
    # left ports are wired inverted (negative RPM = forward)
    print(f"expected bus raw    front(id 2) L/R=({-fl:+d}, {fr:+d})  "
          f"back(id 1) L/R=({-bl:+d}, {br:+d})")

    from dimos.core.transport_factory import make_transport
    from dimos.msgs.geometry_msgs.Twist import Twist

    msg = Twist(linear=[args.vx, args.vy, 0.0], angular=[0.0, 0.0, args.wz])
    transport = make_transport(args.topic, Twist)

    period = 1.0 / args.rate
    sent = 0
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        transport.broadcast(None, msg)
        sent += 1
        time.sleep(period)
    # Leave the base stopped rather than relying on the task timeout.
    transport.broadcast(None, Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))

    print(f"published {sent} Twist on {args.topic} over {args.duration:.1f}s "
          f"at {args.rate:.0f} Hz, then one zero Twist")
    print("now compare with: dimos log -n 20")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

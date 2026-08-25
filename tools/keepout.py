#!/usr/bin/env python3
"""Declare, once, the places the rover must treat differently.

A zone is an axis-aligned rectangle in the persistent frame - the frame the
saved map lives in, the one the rover relocalizes into at boot. Read the
corner coordinates straight off the Rerun map by hovering it.

    tools/keepout.py add toilettes 0.55 -9.95 2.65 -6.65
    tools/keepout.py add rampe -3.0 -8.2 -1.95 -7.75 --type no_slip_reflex \
        --note "slipping on a ramp is normal"
    tools/keepout.py list
    tools/keepout.py rm toilettes

Two types:

  forbidden       (the default) the cells become occupied in the costmap AFTER
                  every layer: no lidar ray, no camera floor sample and no
                  body_clear can erase them, and the planner never enters.
  no_slip_reflex  the place is allowed, but while the rover stands in it the
                  anti-slip reflexes (stuck_guard, ImuSlipDetector) stay
                  silent. For a ramp, where slipping is normal and cutting the
                  torque makes the rover slide back down.

The zones land in ~/.local/state/vector/keepout.json. The running stack picks
an edit up within about half a minute, so there is nothing to restart.

They only apply to a run that RELOCALIZED into the persistent frame. A run
with its own fresh origin ignores them - the coordinates would land somewhere
else in the flat - and says so in its log.

Standard library only: runs under the bare python3 as well as the venv.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vector_dimos import persistent_map


def _map_bounds() -> str:
    """The persistent map's extent, so a typo in a coordinate is visible."""
    if not persistent_map.map_exists():
        return (f"no persistent map yet ({persistent_map.MAP_PATH}) - zones can be declared, but "
                "they only take effect once a run has saved one")
    try:
        import numpy as np
        z = np.load(persistent_map.MAP_PATH)
        res, n, ox, oy = float(z["res"]), int(z["n"]), float(z["ox"]), float(z["oy"])
        return (f"persistent map: x {ox:+.1f} .. {ox + n * res:+.1f} m, "
                f"y {oy:+.1f} .. {oy + n * res:+.1f} m, {res} m cells")
    except Exception as exc:  # noqa: BLE001
        return f"persistent map unreadable: {exc}"


def cmd_list(_args: argparse.Namespace) -> int:
    zones = persistent_map.load_keepouts()
    print(_map_bounds())
    if not zones:
        print(f"no keep-out zone declared ({persistent_map.KEEPOUT_PATH})")
        return 0
    frame = persistent_map.keepout_frame()
    print(f"{len(zones)} zone(s) in {persistent_map.KEEPOUT_PATH}"
          + (f", drawn on {frame!r}" if frame else "") + ":")
    for z in zones:
        w, h = z["x1"] - z["x0"], z["y1"] - z["y0"]
        print(f"  {z['label']:<24} {z['type']:<15} x {z['x0']:+.2f} .. {z['x1']:+.2f}   "
              f"y {z['y0']:+.2f} .. {z['y1']:+.2f}   ({w:.2f} x {h:.2f} m)")
        if z["note"]:
            print(f"  {'':<24} {'':<15} {z['note']}")
    return 0


def cmd_add(args: argparse.Namespace) -> int:
    existing = persistent_map.load_keepouts()
    zones = [z for z in existing if z["label"] != args.label]
    replaced = len(zones) != len(existing)
    zones.append({"label": args.label, "type": args.type,
                  "x0": min(args.x0, args.x1), "y0": min(args.y0, args.y1),
                  "x1": max(args.x0, args.x1), "y1": max(args.y0, args.y1),
                  "note": args.note})
    persistent_map.save_keepouts(zones)
    print(f"{'replaced' if replaced else 'added'} {args.type} zone {args.label!r}: "
          f"x {min(args.x0, args.x1):+.2f} .. {max(args.x0, args.x1):+.2f}, "
          f"y {min(args.y0, args.y1):+.2f} .. {max(args.y0, args.y1):+.2f} m")
    print(_map_bounds())
    return 0


def cmd_rm(args: argparse.Namespace) -> int:
    zones = persistent_map.load_keepouts()
    kept = [z for z in zones if z["label"] != args.label]
    if len(kept) == len(zones):
        print(f"no zone named {args.label!r}", file=sys.stderr)
        return 1
    persistent_map.save_keepouts(kept)
    print(f"removed zone {args.label!r} ({len(kept)} left)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_add = sub.add_parser("add", help="declare a rectangle, in metres, in the persistent frame")
    p_add.add_argument("label")
    for name in ("x0", "y0", "x1", "y1"):
        p_add.add_argument(name, type=float)
    p_add.add_argument("--type", choices=persistent_map.ZONE_TYPES, default=persistent_map.FORBIDDEN,
                       help="forbidden (never enter) or no_slip_reflex (allowed, reflexes stay quiet)")
    p_add.add_argument("--note", default="", help="why, in your own words")
    p_add.set_defaults(func=cmd_add)

    sub.add_parser("list", help="show the declared zones").set_defaults(func=cmd_list)

    p_rm = sub.add_parser("rm", help="remove a zone by label")
    p_rm.add_argument("label")
    p_rm.set_defaults(func=cmd_rm)

    args = ap.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())

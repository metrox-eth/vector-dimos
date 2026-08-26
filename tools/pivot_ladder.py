"""Pivot ladder: does the BODY actually rotate when the wheels say it does?

The 26/08 autopsy left one physical question open: at the follower's capped
0.35 rad/s the wheels turned and the detectors screamed, but whether the body
truly failed to pivot on this floor is unsettled (the detectors are gone, and
every odometry witness was contaminated by the slip freeze). This tool answers
it with the one witness that needs no calibration: the lidar scans themselves.

For each commanded yaw rate it captures a scan, spins in place for --secs,
captures another scan, and measures how far the ROOM rotated around the sensor
(circular correlation of 1-degree range profiles - no wheels, no gyro, no
odometry anywhere in the loop). Wheel-integrated rotation is printed next to
it, so the table reads: commanded vs wheels-claimed vs body-real.

Run ON the Jetson, with the dimos stack STOPPED (the tool owns the RS485 bus
and the C1 serial port), rover on open floor, e-stop in hand:

    .venv/bin/python tools/pivot_ladder.py                 # 0.35 0.5 0.7 1.0 rad/s, 2 s each
    .venv/bin/python tools/pivot_ladder.py --wz 0.5 --secs 3
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.adapter import VectorBaseAdapter
from vector_dimos.c1_serial import C1Lidar
from vector_dimos.rplidar_c1 import DEFAULT_PORT as LIDAR_PORT

COMMAND_HZ = 20.0        # the drives' own watchdog zeroes after 1 s of silence
SETTLE_S = 0.7           # let the body stop before the "after" scan
MAX_WZ = 1.2             # rad/s - hard clamp, e-stop territory beyond
SHIFT_SEARCH_DEG = 90    # widest rotation one rung is expected to produce


def range_profile(scan: list[tuple[int, float, float]]) -> np.ndarray:
    """One revolution -> 360-bin profile of the nearest return per degree."""
    prof = np.full(360, np.nan)
    for quality, angle, distance_mm in scan:
        if quality < 10 or distance_mm <= 150.0:
            continue
        b = int(angle) % 360
        d = distance_mm / 1000.0
        if not (prof[b] <= d):          # NaN-safe min
            prof[b] = d
    # fill the gaps so the correlation is not dominated by missing bins
    idx = np.arange(360)
    good = ~np.isnan(prof)
    if good.sum() < 90:
        raise RuntimeError(f"scan too sparse for a verdict ({int(good.sum())} bins)")
    prof[~good] = np.interp(idx[~good], idx[good], prof[good], period=360)
    return prof


def room_shift_deg(before: np.ndarray, after: np.ndarray) -> int:
    """How many degrees the range profile rotated between the two scans."""
    b = before - before.mean()
    a = after - after.mean()
    best, best_v = 0, -np.inf
    for shift in range(-SHIFT_SEARCH_DEG, SHIFT_SEARCH_DEG + 1):
        v = float(np.dot(b, np.roll(a, shift)))
        if v > best_v:
            best_v, best = v, shift
    return best


def grab_profile(scans) -> np.ndarray:
    """Drain buffered revolutions, keep the freshest one."""
    scan = None
    for _ in range(3):
        scan = next(scans)
    return range_profile(scan)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--wz", type=float, nargs="+", default=[0.35, 0.5, 0.7, 1.0],
                    help="yaw rates to try, rad/s (default: 0.35 0.5 0.7 1.0)")
    ap.add_argument("--secs", type=float, default=2.0, help="spin time per rung (default 2.0)")
    args = ap.parse_args()

    rungs = [min(abs(w), MAX_WZ) for w in args.wz]

    lidar = C1Lidar(LIDAR_PORT)
    scans = lidar.iter_scans()
    adapter = VectorBaseAdapter(dof=3)
    if not adapter.connect():
        print("drives did not answer - is the dimos stack stopped?")
        return 1

    input("Zone dégagée, e-stop en main ? Entrée pour armer et démarrer... ")
    rows = []
    try:
        adapter.write_enable(True)
        for wz in rungs:
            before = grab_profile(scans)
            th0 = adapter.read_odometry()[2]
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.secs:
                adapter.write_velocities([0.0, 0.0, wz])
                time.sleep(1.0 / COMMAND_HZ)
            adapter.write_velocities([0.0, 0.0, 0.0])
            time.sleep(SETTLE_S)
            th1 = adapter.read_odometry()[2]
            after = grab_profile(scans)

            commanded = math.degrees(wz) * args.secs
            wheels = math.degrees(th1 - th0)
            body = -room_shift_deg(before, after)   # the room turns opposite to the body
            rows.append((wz, commanded, wheels, body))
            print(f"  wz {wz:.2f} rad/s x {args.secs:.1f} s : commande {commanded:+6.1f} deg | "
                  f"roues {wheels:+6.1f} deg | corps (lidar) {body:+4d} deg")
            time.sleep(1.0)
    finally:
        adapter.write_velocities([0.0, 0.0, 0.0])
        adapter.write_enable(False)
        adapter.disconnect()
        lidar.stop()
        lidar.disconnect()

    print("\n  wz      commande   roues    corps   corps/roues")
    for wz, commanded, wheels, body in rows:
        ratio = body / wheels if abs(wheels) > 2.0 else float("nan")
        print(f"  {wz:4.2f}  {commanded:+8.1f} {wheels:+8.1f} {body:+7d}   {ratio:5.2f}")
    print("\nLecture : corps/roues ~1.0 = le pivot marche a cette vitesse ; ~0 = les roues")
    print("patinent sans tourner le corps ; le signe du corps est mesure par le lidar seul.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

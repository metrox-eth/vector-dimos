"""Read-only probe of an RPLIDAR C1 on a serial port: info, health, a few turns.

    $ python tests/probe_rplidar.py /dev/ttyUSB0            # 460800 baud (C1)
    $ python tests/probe_rplidar.py /dev/ttyUSB0 --turns 10

Prints points/s, angular coverage, and the 36 sector minima (10 deg each) so a
fixed occlusion (a mast in the scan plane) shows up as a sector stuck at a
short, constant range. Stops the motor when done.
"""
import argparse
import math
import sys
import time

sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
from vector_dimos.c1_serial import C1Scanner

p = argparse.ArgumentParser()
p.add_argument("port")
p.add_argument("--baud", type=int, default=460800)
p.add_argument("--turns", type=int, default=5)
args = p.parse_args()

lidar = C1Scanner(args.port, baudrate=args.baud, timeout=3)
try:
    lidar.open()
    print("info:", lidar.info()); print("health:", lidar.health())
    sectors = [[] for _ in range(36)]
    n = 0; t0 = time.monotonic(); turns = 0
    for scan in lidar.scans(min_len=50):
        turns += 1
        for (_q, ang, dist) in scan:
            if dist > 0:
                sectors[int(ang // 10) % 36].append(dist); n += 1
        if turns >= args.turns:
            break
    dt = time.monotonic() - t0
    print(f"{turns} turns in {dt:.2f} s -> {turns/dt:.1f} Hz, {n/dt:.0f} points/s, "
          f"{sum(1 for s in sectors if s)}/36 sectors seen")
    print("sector minima [m] (10 deg each, 0 = lidar forward mark):")
    row = []
    for i, s in enumerate(sectors):
        row.append(f"{i*10:3d}:{min(s):.2f}" if s else f"{i*10:3d}: -- ")
        if len(row) == 6:
            print("   " + "  ".join(row)); row = []
except Exception as e:
    print("RPLIDAR error:", type(e).__name__, e); sys.exit(1)
finally:
    lidar.close()

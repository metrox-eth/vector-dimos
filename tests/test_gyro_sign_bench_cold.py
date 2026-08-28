#!/usr/bin/env python3
"""Cold bench for tools/gyro_sign_bench.verdict(): the rungs -> the verdict.

No rover, no bus: every rung is a fixture (label, gyro deg, lidar deg), degrees
in and a verdict + exit code out (Rule #2).

The bug this pins: the verdict used `all(... for ... if abs(lidar) > 5)`, and
all() over an EMPTY set is True - a run where the body never turned (e-stop,
wheels off the ground, drives not armed) printed "mapping CORRECT" and exited
0, shipping an unmeasured rotation prior into lidar_odometry.  A verdict now
needs at least one rung the lidar credits with a real turn.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import gyro_sign_bench as bench  # noqa: E402

OK = KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- fixtures, degrees ------------------------------------------------------

# the rover never turned: the twists went out, the lidar saw 0.4 deg of drift
STILL = [("+30 deg (left)", 29.5, 0.4), ("-30 deg (right)", -28.9, -0.3)]
AGREE = [("+30 deg (left)", 29.5, 30.2), ("-30 deg (right)", -28.9, -29.7)]
ONE_RUNG = [("+30 deg (left)", 29.5, 30.2), ("-30 deg (right)", -1.2, -0.4)]
FLIPPED = [("+30 deg (left)", -29.5, 30.2), ("-30 deg (right)", 28.9, -29.7)]
DEAD_GYRO = [("+30 deg (left)", 0.2, 30.2), ("-30 deg (right)", -0.1, -29.7)]
TOO_BIG = [("+30 deg (left)", 91.0, 30.2), ("-30 deg (right)", -89.0, -29.7)]

print("A. the rover never turned -> no verdict")
msg, code = bench.verdict(STILL)
check("0.4 deg of lidar yaw is not a turn: INCONCLUSIVE", "INCONCLUSIVE" in msg, msg)
check("and never 'CORRECT'", "CORRECT" not in msg)
check("exit code 3, not 0", code == 3, str(code))

msg, code = bench.verdict([])
check("no rung at all: INCONCLUSIVE, exit 3", "INCONCLUSIVE" in msg and code == 3, f"{code} {msg}")

# the pre-fix line, verbatim from HEAD:tools/gyro_sign_bench.py:119 - it is
# True on STILL because the generator is empty.  This is the bite.
prefix_ok = all(g * l > 0 and 0.5 < abs(g / l) < 2.0 for _, g, l in STILL if abs(l) > 5)
check("the pre-fix expression DID say CORRECT on the same rungs", prefix_ok is True)

print("B. the rover turned -> the verdict is measured")
msg, code = bench.verdict(AGREE)
check("29.5 deg gyro vs 30.2 deg lidar, both rungs: CORRECT, exit 0",
      "CORRECT" in msg and code == 0, f"{code} {msg}")

msg, code = bench.verdict(ONE_RUNG)
check("one real rung is enough: CORRECT, exit 0", "CORRECT" in msg and code == 0, f"{code} {msg}")
check("and it says how many rungs were measured", "1/2 rungs measured" in msg, msg)

msg, code = bench.verdict(FLIPPED)
check("-29.5 deg gyro vs +30.2 deg lidar: SIGN FLIPPED -> 'y', exit 2",
      "SIGN FLIPPED" in msg and "'y'" in msg and code == 2, f"{code} {msg}")

print("C. the body turned but the gyro did not")
msg, code = bench.verdict(DEAD_GYRO)
check("0.2 deg gyro over a 30 deg turn: WRONG, exit 2 (never CORRECT)",
      "WRONG" in msg and code == 2, f"{code} {msg}")
msg, code = bench.verdict(TOO_BIG)
check("91 deg gyro over a 30 deg turn (ratio 3): WRONG, exit 2",
      "WRONG" in msg and code == 2, f"{code} {msg}")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

#!/usr/bin/env python3
"""The ROBOT PROFILE the sim runs with: our rover, or the maintainer's Go2.

Every bench in this workspace so far simulated OUR rover: a 0.46 m wide body
rolling at 0.15 m/s and needing a 60 cm aisle. The floors it explored were all
recorded by a Unitree Go2, which is 0.31 m wide and walks. Two consequences,
and they decide everything:

  - two thirds of the recorded floor is closed to our body and open to theirs,
    so the "passable floor" the whole bench measures coverage against is a
    skeleton of the real floor (fleet report section 3.1, and its figures);
  - goal_timeout x speed is how far a walk gets before the loop pulls the robot
    off it. At 15 s (the UPSTREAM WavefrontConfig.goal_timeout) and 0.15 m/s
    that is 2.2 m, which is why the cand2 bench found config `shipped` mostly
    measuring the timeout. At the Go2's own recorded speed it is metres, not
    centimetres.

This module holds the two profiles as data and installs one of them onto
explore_sim's module constants. Every constant it touches is read by
explore_sim AT CALL TIME, so setting them before a world is built is enough;
the bench sets them in the worker too, the way it already does for
LIDAR_RANGE_M, so a spawn-started worker cannot silently run the wrong body.

NOTHING HERE IS UPSTREAM CODE and nothing here touches upstream code. These are
the harness' own simulation constants: the shape of the robot and how fast it
moves. dimOS' own constants (the loop's 0.25 m costmap inflation, the 15 s goal
timeout, the info-gain self-stop, the failed-goal radius) are NOT in this file
and are not changed by it.

WHERE THE GO2 NUMBERS COME FROM
-------------------------------
speed 0.60 m/s   Derived in `derive_speed.py` from the `odom` stream of dimOS'
                 OWN recordings, the same stores the maps were extracted from:
                 the median speed over the moving 0.5 s windows of the five HK
                 recordings pooled (18 733 windows, median 0.6029, p90 1.0387).
                 See speed_derivation.txt for the per-map table and the
                 sensitivity of the choice.
turn 0.63 rad/s  DERIVED, by the same rule as the speed. A window where the body
                 is turning (yaw rate >= 0.10 rad/s) while NOT translating
                 (speed < 0.05 m/s) is a turn IN PLACE, which is what
                 explore_sim's TURN_RATE models; the yaw rate of a walking curve
                 is not. Pooled over the five HK recordings: 1 492 pivot
                 windows, MEDIAN 0.6287 rad/s, p90 1.2968, max 3.55. The Go2
                 pivots about 26 % faster than the rover's 0.5 rad/s, which
                 matters because turning spends goal-timeout budget: a 180 deg
                 turn costs 5.0 s instead of 6.3 s. See speed_derivation.txt.
width 0.31 m     Unitree Go2 published standing size 700 x 310 x 400 mm
                 (docs.quadruped.de "Model Variants | Unitree GO2", and the
                 Unitree shop page; identical across Air / Pro / Edu).
length 0.70 m    same source. Used only for the pivot circle.

THE MARGINS ARE THE HARNESS' OWN, KEPT PROPORTIONAL
---------------------------------------------------
The rover profile is, in explore_sim's own words:

    ROBOT_WIDTH_M      = 0.50   recovering_planner's robot_width: body + 4 cm
    BODY_HALF_WIDTH_M  = 0.23   the real body, 0.46 / 2
    CONTROL_MARGIN_M   = 0.05   a control margin, not a body dimension
    LETHAL_CLEARANCE_M = 0.30   = ROBOT_WIDTH_M / 2 + CONTROL_MARGIN_M
    PIVOT_CLEARANCE_M  = 0.39   = half the body diagonal, hypot(0.625, 0.46)/2

The Go2 profile keeps every one of those RULES and only changes the body:

    ROBOT_WIDTH_M      = 0.35   = 0.31 + the same 4 cm planner margin
    BODY_HALF_WIDTH_M  = 0.155  = 0.31 / 2
    CONTROL_MARGIN_M   = 0.05   unchanged, it is a controller margin
    LETHAL_CLEARANCE_M = 0.225  = 0.35 / 2 + 0.05, the same formula
    PIVOT_CLEARANCE_M  = 0.383  = hypot(0.70, 0.31) / 2, the same rule

Note what that last line says: the Go2 is narrower but LONGER, so its pivot
circle is almost the rover's. Only the two width-driven numbers really move.

WHAT IS NOT SCALED, AND WHY - now with the measurement behind it
----------------------------------------------------------------
SCAN_EVERY_M stays 0.25 m, one simulated lidar revolution per 0.25 m of travel.
The objection this has to survive is real: raising the speed without touching a
per-DISTANCE scan rate would be an optimistic discovery bias if the robot could
not actually deliver a revolution inside every 0.25 m of walk. It can. Measured
in derive_speed.py from the recordings' own lidar frame timestamps against their
own odom, moving pairs only, pooled over the five HK recordings: the Go2 travels
a MEDIAN 0.0789 m and a p90 of 0.1457 m between two consecutive lidar frames,
its stream running at 7.7 Hz. So at 0.60 m/s there are about 3.2 real
revolutions inside every 0.25 m slice, and about 12.7 at 0.15 m/s: the simulated
one-per-0.25 m is the coarser of the two at BOTH speeds, and the information per
metre is limited by geometry, not by the sensor, at either speed.

Keeping the step in DISTANCE therefore isolates the robot profile from the
sensor model, and keeps map-building per metre travelled identical to every
earlier bench in this workspace, so a coverage number here and a coverage number
there mean the same thing. The honest caveat, stated in the report: the sim
under-samples the real Go2 lidar by about 3x per metre, which makes it
CONSERVATIVE about discovery, not optimistic.

ARRIVE_M (0.25 m), BUMP_BACKUP_M (0.20 m) and the harness stop conditions
(MAX_GOALS 300, MAX_PATH_M 600, MAX_SIM_S 6000, STALL_M 20) are unchanged.
MAX_PATH_M is the budget that binds under both profiles.
"""
from __future__ import annotations

import math

PROFILES = {
    # the profile every earlier bench in this workspace ran
    "rover": dict(
        SPEED_MPS=0.15,
        TURN_RATE=0.5,
        ROBOT_WIDTH_M=0.50,
        BODY_HALF_WIDTH_M=0.23,
        CONTROL_MARGIN_M=0.05,
        LETHAL_CLEARANCE_M=0.30,
        PIVOT_CLEARANCE_M=0.39,
        SCAN_EVERY_M=0.25,
    ),
    # THE FIDELITY PROFILE (sim_2830_fidelity). Identical to "go2" in every
    # dimension and margin; the ONE change is the speed, and it is a correction.
    #
    # The go2 bench set SPEED_MPS from the recordings (pooled median 0.6029 m/s
    # of moving windows). Those recordings are a HUMAN TELEOPERATING the robot.
    # The loop under test does not teleoperate: it publishes a goal and the
    # autonomous stack drives, and that stack has its own cap,
    #     dimos/navigation/replanning_a_star/local_planner.py:67
    #     _speed: float = 0.55
    # which is what the Go2 walks at when the wavefront selector is driving it
    # (GlobalPlanner builds its LocalPlanner with that value, local_planner.py:84-97;
    # the go2 "smart" blueprint overrides neither it nor global_config.nerf_speed).
    # 0.55 is therefore the speed of the system this bench is about, and the
    # recording-derived 0.60 is the speed of a different system (a human's thumb).
    #
    # It is also the CONSERVATIVE choice for the premise this bench rests on:
    # goal_timeout x speed is how far a walk gets before the loop pulls the robot
    # off it, and 15 s x 0.55 = 8.25 m against 9.00 m at 0.60.
    #
    # Everything else is the go2 profile unchanged, so a fidelity number and a
    # go2-bench number differ by the speed and the loop semantics and nothing else.
    "go2ctrl": dict(
        SPEED_MPS=0.55,
        TURN_RATE=0.63,
        ROBOT_WIDTH_M=0.35,
        BODY_HALF_WIDTH_M=0.155,
        CONTROL_MARGIN_M=0.05,
        LETHAL_CLEARANCE_M=0.225,
        PIVOT_CLEARANCE_M=round(math.hypot(0.70, 0.31) / 2.0, 4),
        SCAN_EVERY_M=0.25,
    ),
    # the maintainer's robot, at the speed a human drove it in the recordings
    "go2": dict(
        SPEED_MPS=0.60,
        TURN_RATE=0.63,
        ROBOT_WIDTH_M=0.35,
        BODY_HALF_WIDTH_M=0.155,
        CONTROL_MARGIN_M=0.05,
        LETHAL_CLEARANCE_M=0.225,
        PIVOT_CLEARANCE_M=round(math.hypot(0.70, 0.31) / 2.0, 4),
        SCAN_EVERY_M=0.25,
    ),
}

BODY = {
    "rover": {"width_m": 0.46, "length_m": 0.625,
              "source": "explore_sim.py l.111-112, the real rover with its bumper bars"},
    "go2": {"width_m": 0.31, "length_m": 0.70,
            "source": "Unitree Go2 published standing size 700 x 310 x 400 mm"},
    "go2ctrl": {"width_m": 0.31, "length_m": 0.70,
                "source": "Unitree Go2 published standing size 700 x 310 x 400 mm; "
                          "speed from dimos LocalPlanner._speed = 0.55 m/s"},
}


def apply(ES, name: str) -> dict:
    """Install a profile onto explore_sim's module constants. Returns it."""
    if name not in PROFILES:
        raise ValueError(f"unknown robot profile {name!r}, have {list(PROFILES)}")
    p = PROFILES[name]
    for k, v in p.items():
        if not hasattr(ES, k):
            raise AttributeError(f"explore_sim has no constant {k}; profile refused")
        setattr(ES, k, v)
    ES.ROBOT_PROFILE = name
    return dict(p)


def describe(name: str) -> str:
    p = PROFILES[name]
    b = BODY[name]
    return (f"profile {name}: body {b['width_m']:.3f} x {b['length_m']:.3f} m, "
            f"speed {p['SPEED_MPS']:.2f} m/s, turn {p['TURN_RATE']:.2f} rad/s, "
            f"lethal {p['LETHAL_CLEARANCE_M']:.3f} m, pivot {p['PIVOT_CLEARANCE_M']:.3f} m, "
            f"scan every {p['SCAN_EVERY_M']:.2f} m")

"""Cold bench for tools/garde_vitesse.py: the speed watchdog must measure the
body on the LOG's own timestamps and kill on ONE interval over the envelope.

Rule #2: known input -> known output in metres and seconds. The rover is not
needed - synthetic odom lines shaped exactly like the rover's main.jsonl are fed
straight to the guard (no tail, no processes, no sleeping). Groups:

  A. dating    - a known ISO stamp -> a known epoch; two lines 2 s apart -> 2.000 s
  B. runaway   - 1.000 m per 2 s line = 0.50 m/s -> breach on the FIRST interval
  C. envelope  - 0.600 m per 2 s line = 0.30 m/s -> measured 0.30, never a breach
  D. geometry  - dx 0.600 / dy 0.800 in 2 s = 1.000 m of chord -> 0.50 m/s (hypot)
  E. cadence   - the real field log recordings/courseB.jsonl: every interval is
                 measured now (the old 0.02 < dt < 2.0 window dropped the ones at
                 or above 2 s) and the crawl stays silent
  F. kill      - a breach publishes 'stop' through explore_ctl, on the run's bus,
                 and a sick bus does not take the watchdog down
  G. junk      - non-odom, undated and out-of-order lines invent no decision

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_garde_vitesse_cold.py
"""

import io
import json
import sys
from contextlib import redirect_stdout
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import explore_ctl  # noqa: E402
import garde_vitesse  # noqa: E402

T0 = 1787714110.954660          # 2026-08-26T03:15:10.954660Z, the first odom line of courseB
STEP_S = 2.0                    # LidarOdometry's log_every_s: the cadence the old window sat on
FIELD_LOG = ROOT / "recordings" / "courseB.jsonl"
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def iso(t):
    return datetime.fromtimestamp(t, timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def odom_line(n, t, x, y):
    """One line shaped like the rover's own: structlog jsonl, %+.3f positions."""
    return json.dumps({
        "event": f"lidar odom #{n}: x={x:+.3f} y={y:+.3f} yaw=+0.0deg (380 pts, 31.0 ms, "
                 "prior wheels, depth cloud 274 pts) | gyro integral x=+0.0 y=-1.1 z=-0.2deg "
                 "| wheels x=+0.000 y=+0.000 th=+0.0deg",
        "level": "info",
        "logger": "/home/metrox/vector-dimos/vector_dimos/lidar_odometry.py",
        "timestamp": iso(t),
        "func_name": "handle_pointcloud",
        "lineno": 635,
    })


def straight_run(speed, steps=5, step_s=STEP_S):
    """Lines for a body going straight at `speed` m/s, one line every `step_s`."""
    return [odom_line(20 * i + 1, T0 + i * step_s, i * speed * step_s, 0.0) for i in range(steps + 1)]


print("A. dating the lines")
sample = garde_vitesse.parse_odom(odom_line(1, T0, 0.0, 0.0))
check("a real-shaped odom line parses", sample is not None)
check(f"its stamp is {T0} s (2026-08-26T03:15:10.954660Z)", abs(sample[0] - T0) < 1e-6, f"{sample[0]:.6f}")
check("x, y come back in metres", sample[1:] == (0.0, 0.0), str(sample[1:]))
later = garde_vitesse.parse_odom(odom_line(21, T0 + STEP_S, 1.0, 0.0))
check("two lines 2 s apart -> 2.000 s of log time", abs((later[0] - sample[0]) - 2.0) < 1e-6,
      f"{later[0] - sample[0]:.6f} s")

print("B. runaway at 0.5 m/s (1.000 m per line)")
guard = garde_vitesse.SpeedGuard()
speeds, kills = [], []
for i, line in enumerate(straight_run(0.5)):
    v = guard.feed(line)
    if v is not None:
        speeds.append(v)
        if v > garde_vitesse.LIMIT_MS:
            kills.append(i)
check("first line closes no interval", len(speeds) == 5, f"{len(speeds)} intervals from 6 lines")
check("every interval measures 0.50 m/s", all(abs(v - 0.5) < 1e-9 for v in speeds),
      ", ".join(f"{v:.3f}" for v in speeds))
check("the SECOND line already breaches (1 interval = 1 kill, ~1 m of runaway)", kills[:1] == [1],
      f"breaches at lines {kills}")
check("no strike counter left: all 5 intervals breach", len(kills) == 5, str(kills))
guard = garde_vitesse.SpeedGuard()
first = [guard.breach(line) for line in straight_run(0.5, steps=1)]
check("breach() returns the measured 0.50 m/s on that first interval",
      first[0] is None and abs(first[1] - 0.5) < 1e-9, str(first))

print("C. inside the envelope at 0.3 m/s (0.600 m per line)")
guard = garde_vitesse.SpeedGuard()
verdicts = [(guard.feed(line), line) for line in straight_run(0.3)]
measured = [v for v, _ in verdicts if v is not None]
check("every interval measures 0.30 m/s", len(measured) == 5 and all(abs(v - 0.3) < 1e-9 for v in measured),
      ", ".join(f"{v:.3f}" for v in measured))
guard = garde_vitesse.SpeedGuard()
check("silence: breach() never fires under 0.35 m/s",
      all(guard.breach(line) is None for line in straight_run(0.3)))
guard = garde_vitesse.SpeedGuard()
check("0.34 m/s (0.680 m per line) stays silent too",
      all(guard.breach(line) is None for line in straight_run(0.34)))
guard = garde_vitesse.SpeedGuard()
check("0.36 m/s (0.720 m per line) kills",
      any(guard.breach(line) is not None for line in straight_run(0.36)))

print("D. geometry: the chord, not one axis")
guard = garde_vitesse.SpeedGuard()
guard.feed(odom_line(1, T0, 0.0, 0.0))
v = guard.feed(odom_line(21, T0 + STEP_S, 0.6, 0.8))   # 3-4-5: 1.000 m of chord
check("dx 0.600 m, dy 0.800 m in 2 s -> 0.50 m/s", abs(v - 0.5) < 1e-9, f"{v:.6f} m/s")
guard = garde_vitesse.SpeedGuard()
guard.feed(odom_line(1, T0, 5.0, -3.0))
v = guard.feed(odom_line(21, T0 + STEP_S, 5.0, -3.6))   # backwards on y, 0.600 m
check("0.600 m travelled backwards in 2 s -> 0.30 m/s (sign-free)", abs(v - 0.3) < 1e-9, f"{v:.6f} m/s")

print("E. the real field log (recordings/courseB.jsonl, a 0.149 m/s crawl)")
guard = garde_vitesse.SpeedGuard()
field, dts, prev = [], [], None
for raw in FIELD_LOG.read_text().splitlines():
    v = guard.feed(raw)
    s = garde_vitesse.parse_odom(raw)
    if s is not None:
        if prev is not None:
            dts.append(s[0] - prev)
        prev = s[0]
    if v is not None:
        field.append(v)
check("239 odom lines -> 238 measured intervals", len(field) == 238, f"{len(field)} intervals")
kept_by_old = sum(1 for dt in dts if 0.02 < dt < 2.0)
check("the old 0.02 < dt < 2.0 window kept only 55 of those 238 intervals",
      kept_by_old == 55, f"{kept_by_old} kept, {len(dts) - kept_by_old} dropped at >= 2 s")
check("the log's own cadence really straddles 2 s (min 1.699 s, max 3.071 s)",
      abs(min(dts) - 1.699) < 0.01 and abs(max(dts) - 3.071) < 0.01,
      f"min {min(dts):.3f} s, max {max(dts):.3f} s")
check("that flight peaks at 0.21 m/s and never trips the 0.35 m/s envelope",
      abs(max(field) - 0.2093) < 1e-3 and max(field) < garde_vitesse.LIMIT_MS,
      f"max {max(field):.4f} m/s")
guard = garde_vitesse.SpeedGuard()
check("no false kill replaying the whole flight",
      all(guard.breach(raw) is None for raw in FIELD_LOG.read_text().splitlines()))

print("F. the kill itself")
published = []
saved = explore_ctl.publish
try:
    explore_ctl.publish = lambda cmd: published.append(cmd) or f"/{cmd}_explore_cmd#std_msgs.Bool"
    out = io.StringIO()
    with redirect_stdout(out):
        garde_vitesse.stop_all()
    check("a breach publishes exactly one 'stop'", published == ["stop"], str(published))
    check("through explore_ctl, so it rides the run's transport (zenoh or lcm)",
          "stop_explore_cmd" in out.getvalue(), out.getvalue().strip())

    def sick(cmd):
        raise RuntimeError("bus down")

    explore_ctl.publish = sick
    out = io.StringIO()
    with redirect_stdout(out):
        garde_vitesse.stop_all()   # must not raise: the watchdog keeps watching
    check("a failing bus is logged, not fatal", "FAILED" in out.getvalue(), out.getvalue().strip())
finally:
    explore_ctl.publish = saved

print("G. junk lines invent nothing")
guard = garde_vitesse.SpeedGuard()
guard.feed(odom_line(1, T0, 0.0, 0.0))
check("a non-odom line -> None", guard.feed('{"event": "bump detected", "timestamp": "' + iso(T0 + 1) + '"}') is None)
check("an odom line with no timestamp -> None (no wall clock fallback)",
      guard.feed("lidar odom #7: x=+9.000 y=+9.000 yaw=+0.0deg") is None)
check("tail's '==> file <==' header -> None", guard.feed("==> /tmp/explore_launch.log <==\n") is None)
check("a line dated BEFORE the previous one -> None",
      guard.feed(odom_line(41, T0 - 5.0, 9.0, 9.0)) is None)
guard = garde_vitesse.SpeedGuard()
guard.feed(odom_line(1, T0, 0.0, 0.0))
check("a repeated line (same stamp, dt = 0) -> None, no division",
      guard.feed(odom_line(1, T0, 0.0, 0.0)) is None)
guard = garde_vitesse.SpeedGuard()
lines = straight_run(0.5)
lines.insert(3, '{"event": "costmap tick", "timestamp": "' + iso(T0 + 2.5) + '"}')
check("junk between odom lines does not break the chain",
      sum(1 for line in lines if guard.breach(line) is not None) == 5)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

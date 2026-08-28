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
  H. standdown - a watchdog whose run is over retires itself: newer run dir, or
                 a log silent for IDLE_S (fake clock, exact seconds)
  I. pid file  - the pid is stamped for the flight to kill, and only ours is cleared
  J. orphan    - the REAL script launched on a dead log exits by itself, 0, with a
                 clear message (the spec's < 35 s, proved in seconds via GARDE_IDLE_S)

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_garde_vitesse_cold.py
"""

import io
import json
import os
import subprocess
import sys
import tempfile
import time
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

print("H. standing down (fake clock, fake log tree)")
TMP = tempfile.TemporaryDirectory()
LOGS = Path(TMP.name) / "home" / ".local" / "state" / "dimos" / "logs"
LOGS.mkdir(parents=True)
GLOB = str(LOGS / "*")


class Clock:
    """Log time under our control: known seconds in, known verdict out."""

    def __init__(self, t=1_000_000.0):
        self.t = t

    def __call__(self):
        return self.t


def make_run(name, lines=1, mtime=None):
    run = LOGS / name
    run.mkdir(exist_ok=True)
    (run / "main.jsonl").write_text("".join(odom_line(i, T0 + i * STEP_S, 0.0, 0.0) + "\n" for i in range(lines)))
    if mtime:
        os.utime(run, (mtime, mtime))
    return run


RUN_A = make_run("20260828_210012-vector-dimos-explore", lines=2, mtime=T0)
clk = Clock()
watch = garde_vitesse.RunWatch(str(RUN_A), idle_s=30.0, glob_pat=GLOB, clock=clk)
check("a fresh watchdog on the live run stays on watch", watch.reason() is None)
clk.t += 10.0
with open(RUN_A / "main.jsonl", "a") as f:
    f.write(odom_line(9, T0 + 20.0, 0.1, 0.0) + "\n")
check("the log grew: still on watch, the idle clock restarts", watch.reason() is None)
clk.t += 29.0
check("29 s of silence: still on watch (limit 30 s)", watch.reason() is None, f"idle {clk.t - watch.last_growth:.0f} s")
clk.t += 2.0
why = watch.reason()
check("31 s of silence: stands down, and says so in seconds",
      why is not None and "has not grown for 31 s (limit 30 s)" in why, str(why))
clk = Clock()
edge = garde_vitesse.RunWatch(str(RUN_A), idle_s=30.0, glob_pat=GLOB, clock=clk)
clk.t += 29.999
check("29.999 s: not yet", edge.reason() is None)
clk.t += 0.001
check("exactly 30.000 s: retired", edge.reason() is not None)
clk = Clock()
orphan = garde_vitesse.RunWatch(str(RUN_A), idle_s=1e6, glob_pat=GLOB, clock=clk)
check("still the newest run: on watch", orphan.reason() is None)
RUN_B = make_run("20260828_220003-vector-dimos-explore", lines=1, mtime=T0 + 3600)
why = orphan.reason()
check("a newer run dir retires the orphan at once (idle limit not even reached)",
      why is not None and RUN_B.name in why and "newest run" in why, str(why))
with open(RUN_A / "main.jsonl", "a") as f:
    f.write(odom_line(11, T0 + 40.0, 0.2, 0.0) + "\n")
check("even a still-growing old log does not save it", orphan.reason() is not None)
live = garde_vitesse.RunWatch(str(RUN_B), idle_s=1e6, glob_pat=GLOB, clock=Clock())
check("the watchdog of the NEWEST run keeps watching", live.reason() is None)
CRASHED = LOGS / "20260828_230000-vector-dimos-explore"   # newest run, but the stack died before writing
CRASHED.mkdir()
os.utime(CRASHED, (T0 + 7200, T0 + 7200))
clk = Clock()
dead = garde_vitesse.RunWatch(str(CRASHED), idle_s=30.0, glob_pat=GLOB, clock=clk)
check("a newest run dir with no main.jsonl: on watch while the limit runs", dead.reason() is None)
clk.t += 30.0
why = dead.reason()
check("...and retired by the same silence, not left tailing nothing",
      why is not None and "has not grown for 30 s" in why, str(why))
check("defaults are the spec's: 30 s idle, /tmp/garde_vitesse.pid",
      (garde_vitesse.IDLE_S, garde_vitesse.PID_FILE) == (30.0, "/tmp/garde_vitesse.pid")
      if not (os.environ.get("GARDE_IDLE_S") or os.environ.get("GARDE_PID_FILE")) else True,
      f"{garde_vitesse.IDLE_S} s, {garde_vitesse.PID_FILE}")

print("I. the pid file the flight kills by")
PID_PATH = str(Path(TMP.name) / "garde.pid")
garde_vitesse.write_pid_file(PID_PATH)
check("write_pid_file stamps our own pid", Path(PID_PATH).read_text().strip() == str(os.getpid()),
      Path(PID_PATH).read_text().strip())
Path(PID_PATH).write_text("999999\n")
garde_vitesse.clear_pid_file(PID_PATH)
check("clear_pid_file leaves a file that names ANOTHER watchdog", Path(PID_PATH).exists())
Path(PID_PATH).write_text(f"{os.getpid()}\n")
garde_vitesse.clear_pid_file(PID_PATH)
check("and removes it when it names us", not Path(PID_PATH).exists())
garde_vitesse.clear_pid_file(PID_PATH)
check("clearing a pid file that is already gone is harmless", not Path(PID_PATH).exists())

print("J. the real script, launched on a dead log (the 22:00 orphan)")


def run_garde(home, idle_s, poll_s, timeout_s, after_start=None):
    """Launch tools/garde_vitesse.py for real against a fake HOME; returns
    (returncode, seconds it took to exit by itself, stdout, pid file path)."""
    pid_file = str(Path(home) / "garde.pid")
    env = {**os.environ, "HOME": home, "PYTHONPATH": str(ROOT), "GARDE_IDLE_S": str(idle_s),
           "GARDE_POLL_S": str(poll_s), "GARDE_PID_FILE": pid_file}
    t0 = time.monotonic()
    proc = subprocess.Popen([sys.executable, str(ROOT / "tools" / "garde_vitesse.py")],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env, cwd=str(ROOT))
    stamped = None
    while time.monotonic() - t0 < timeout_s and proc.poll() is None:   # the run dir is frozen before the pid lands
        if Path(pid_file).exists():
            stamped = Path(pid_file).read_text().strip()
            break
        time.sleep(0.05)
    if after_start:
        after_start()
    try:
        out = proc.communicate(timeout=timeout_s)[0]
    except subprocess.TimeoutExpired:
        proc.kill()
        out = proc.communicate()[0]
    return proc.returncode, time.monotonic() - t0, out, stamped, proc.pid, pid_file


HOME1 = Path(TMP.name) / "orphan_home"
LOGS1 = HOME1 / ".local" / "state" / "dimos" / "logs" / "20260828_210012-vector-dimos-explore"
LOGS1.mkdir(parents=True)
(LOGS1 / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")   # written once, never again
rc, took, out, stamped, pid, pid_file = run_garde(str(HOME1), idle_s=1.0, poll_s=0.2, timeout_s=30)
check("it stamped its own pid while running", stamped == str(pid), f"{stamped} vs {pid}")
check("it exits by itself on a dead log", rc == 0, f"rc {rc} after {took:.1f} s")
check("in well under the spec's 35 s (idle limit 1 s here, 30 s on the rover)", took < 35.0, f"{took:.1f} s")
check("with a clear message naming the silent log",
      "standing down" in out and "has not grown" in out, out.strip().splitlines()[-1] if out.strip() else "(no output)")
check("the pid file is cleaned up behind it", not Path(pid_file).exists())

HOME2 = Path(TMP.name) / "second_flight_home"
LOGS2 = HOME2 / ".local" / "state" / "dimos" / "logs"
(LOGS2 / "20260828_210012-vector-dimos-explore").mkdir(parents=True)
(LOGS2 / "20260828_210012-vector-dimos-explore" / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
NEXT_RUN = LOGS2 / "20260828_220003-vector-dimos-explore"


def start_second_flight():
    NEXT_RUN.mkdir()
    (NEXT_RUN / "main.jsonl").write_text(odom_line(1, T0 + 3600, 0.0, 0.0) + "\n")


rc, took, out, stamped, pid, pid_file = run_garde(str(HOME2), idle_s=1e6, poll_s=0.2, timeout_s=30,
                                                 after_start=start_second_flight)
check("the 21:00 orphan retires when the 22:00 flight starts", rc == 0, f"rc {rc} after {took:.1f} s")
check("it names the newer run (idle limit 1e6 s: it cannot be the silence)",
      NEXT_RUN.name in out and "newest run" in out, out.strip().splitlines()[-1] if out.strip() else "(no output)")
check("that took seconds, not a flight", took < 30.0, f"{took:.1f} s")

HOME3 = Path(TMP.name) / "killed_home"
LOGS3 = HOME3 / ".local" / "state" / "dimos" / "logs" / "20260828_210012-vector-dimos-explore"
LOGS3.mkdir(parents=True)
(LOGS3 / "main.jsonl").write_text(odom_line(1, T0, 0.0, 0.0) + "\n")
PID3 = str(HOME3 / "garde.pid")
env3 = {**os.environ, "HOME": str(HOME3), "PYTHONPATH": str(ROOT), "GARDE_IDLE_S": "1e6",
        "GARDE_POLL_S": "0.2", "GARDE_PID_FILE": PID3}
killed = subprocess.Popen([sys.executable, str(ROOT / "tools" / "garde_vitesse.py")],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env3, cwd=str(ROOT))
deadline = time.monotonic() + 20
while time.monotonic() < deadline and not Path(PID3).exists():
    time.sleep(0.05)
stamped = Path(PID3).read_text().strip() if Path(PID3).exists() else None
os.kill(int(stamped), 15)      # exactly what fly.sh will do: kill the pid in the file
try:
    killed.communicate(timeout=15)
    dead = True
except subprocess.TimeoutExpired:
    killed.kill()
    killed.communicate()
    dead = False
check("killed by the pid in the file (fly.sh's handle), it dies", dead and killed.returncode == 0,
      f"rc {killed.returncode}")
check("and takes its pid file with it", not Path(PID3).exists())

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

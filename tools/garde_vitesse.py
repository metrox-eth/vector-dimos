#!/usr/bin/env python3
"""Speed watchdog: tails newest dimos run log, computes speed from lidar odom
lines, and kills exploration if the body exceeds the envelope. It must be
running whenever the rover explores unattended.

Speed is measured between the TIMESTAMPS THE LOG CARRIES, never between the
moments the lines reach this process: LidarOdometry rate-limits `lidar odom #N`
(log_every_s = 2.0) and the interval jitters around it, so the old wall-clock
window (0.02 < dt < 2.0) threw away most samples - 183 of the 238 intervals in
recordings/courseB.jsonl - and could never strike in time. One interval above
the envelope is a kill; no cadence is assumed anywhere.

It guards ONE run: the run dir is frozen at startup and the pid written to
PID_FILE (fly.sh kills THAT pid, not whatever `pgrep garde_vitesse` finds).
A watchdog whose run is over stands down by itself - either a newer run dir
appeared, or the log it tails stopped growing for IDLE_S. Before this, an
orphan of the previous flight kept tailing a dead log while fly.sh, the panel
and the vigil all read its mere presence as 'armed' and the live flight
explored unguarded."""
import atexit, glob, math, os, re, signal, subprocess, sys, threading, time
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import explore_ctl  # the stop rides the run's transport, same module the flight uses

LIMIT_MS = 0.35        # hard envelope: cap 0.149 + margin; beyond = runaway
LOG_GLOB = "~/.local/state/dimos/logs/*"
IDLE_S = float(os.environ.get("GARDE_IDLE_S", "30"))   # log silent this long -> the run is over
POLL_S = float(os.environ.get("GARDE_POLL_S", "2"))    # how often the stand-down is re-checked
PID_FILE = os.environ.get("GARDE_PID_FILE", "/tmp/garde_vitesse.pid")
ODOM_RE = re.compile(r"lidar odom #(\d+): x=([+-][\d.]+) y=([+-][\d.]+)")
TS_RE = re.compile(r'"timestamp":\s*"([^"]+)"')

def run_dirs(glob_pat: str = LOG_GLOB) -> list:
    """The dimos run dirs, oldest mtime first."""
    return sorted([d for d in glob.glob(os.path.expanduser(glob_pat)) if os.path.isdir(d)], key=os.path.getmtime)

def newest_run(glob_pat: str = LOG_GLOB) -> str:
    runs = run_dirs(glob_pat)
    if not runs:
        sys.exit("no dimos run logs found")
    return runs[-1]

def log_of(run_dir: str) -> str:
    return os.path.join(run_dir, "main.jsonl")

def write_pid_file(path: str = PID_FILE) -> None:
    """Stamp our pid so the flight can kill THIS watchdog by pid."""
    with open(path, "w") as f:
        f.write(f"{os.getpid()}\n")
    atexit.register(clear_pid_file, path)

def clear_pid_file(path: str = PID_FILE) -> None:
    """Drop the pid file, but only while it still names us: a newer watchdog
    that overwrote it must keep its own."""
    try:
        if int(open(path).read().strip()) == os.getpid():
            os.remove(path)
    except (OSError, ValueError):
        pass

class RunWatch:
    """Stand-down decision for the ONE run this watchdog guards: a newer run dir
    means the live flight has its own watchdog and we are its orphan; a log that
    stopped growing for idle_s means the run we guard is over (or dead)."""

    def __init__(self, run_dir: str, idle_s: float = IDLE_S, glob_pat: str = LOG_GLOB, clock=time.time):
        self.run_dir = os.path.realpath(run_dir)
        self.log_path = log_of(run_dir)
        self.idle_s = idle_s
        self.glob_pat = glob_pat
        self.clock = clock
        self.size = self._size()
        self.last_growth = clock()

    def _size(self) -> int:
        try:
            return os.path.getsize(self.log_path)
        except OSError:
            return -1      # no log to grow: the idle limit will retire us

    def reason(self):
        """Why this watchdog must stand down, or None while it still guards the
        live run."""
        runs = run_dirs(self.glob_pat)
        if runs and os.path.realpath(runs[-1]) != self.run_dir:
            return (f"{os.path.basename(runs[-1])} is now the newest run, this process only tails "
                    f"{os.path.basename(self.run_dir)} - the live flight is guarded by its own watchdog")
        size, now = self._size(), self.clock()
        if size != self.size:
            self.size, self.last_growth = size, now
            return None
        idle = now - self.last_growth
        if idle >= self.idle_s:
            return (f"{self.log_path} has not grown for {idle:.0f} s (limit {self.idle_s:g} s) "
                    "- the run it guards is over")
        return None

def supervise(watch: "RunWatch", tail, poll_s: float = POLL_S) -> None:
    """Kill the tail - and with it the main loop - as soon as this watchdog is
    no longer the guard of a live run."""
    while tail.poll() is None:
        why = watch.reason()
        if why:
            print(f"[garde] standing down: {why}", flush=True)
            tail.terminate()
            return
        time.sleep(poll_s)

def parse_odom(line: str):
    """(t seconds, x metres, y metres) from an odom log line, or None when the
    line carries no odom sample or no timestamp to date it with."""
    m = ODOM_RE.search(line)
    ts = TS_RE.search(line)
    if not m or not ts:
        return None
    try:
        t = datetime.fromisoformat(ts.group(1).replace("Z", "+00:00")).timestamp()
    except ValueError:
        return None
    return t, float(m.group(2)), float(m.group(3))

class SpeedGuard:
    """Body speed over consecutive odom lines, dated by the log itself."""

    def __init__(self, limit: float = LIMIT_MS):
        self.limit = limit
        self.prev = None  # (t, x, y) of the last dated odom line

    def feed(self, line: str):
        """Speed in m/s over the interval this line closes, or None when the
        line closes no interval (not odom, undated, or not after the previous)."""
        sample = parse_odom(line)
        if sample is None:
            return None
        t, x, y = sample
        prev, self.prev = self.prev, sample
        if prev is None or t <= prev[0]:
            return None
        return math.hypot(x - prev[1], y - prev[2]) / (t - prev[0])

    def breach(self, line: str):
        """That speed when it exceeds the envelope, else None."""
        v = self.feed(line)
        return v if v is not None and v > self.limit else None

def stop_all() -> None:
    try:
        print(f"[garde] STOP EXPLORE published on {explore_ctl.publish('stop')}", flush=True)
    except Exception as e:  # a sick bus must not take the watchdog down with it
        print(f"[garde] STOP EXPLORE FAILED: {e}", flush=True)

def main() -> None:
    run_dir = newest_run()          # frozen: this watchdog guards THIS run, nothing else
    path = log_of(run_dir)
    write_pid_file()
    watch = RunWatch(run_dir)
    print(f"[garde] watching {path} (limit {LIMIT_MS} m/s, idle limit {IDLE_S:g} s, "
          f"pid {os.getpid()} -> {PID_FILE})", flush=True)
    guard = SpeedGuard()
    paths = [path] + (["/tmp/explore_launch.log"] if os.path.exists("/tmp/explore_launch.log") else [])
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # killed by pid: leave through atexit, pid file and all
    with subprocess.Popen(["tail", "-Fn0", *paths], stdout=subprocess.PIPE, text=True) as tail:
        threading.Thread(target=supervise, args=(watch, tail), daemon=True).start()
        try:
            for line in tail.stdout:
                if "BUMP" in line or "bump" in line:
                    print(f"[garde] {line.rstrip()}", flush=True)
                v = guard.breach(line)
                if v is not None:
                    print(f"[garde] {v:.2f} m/s over the {LIMIT_MS} m/s envelope", flush=True)
                    stop_all()
        finally:
            tail.terminate()   # or the context manager waits forever on a tail nobody killed
    print(f"[garde] stopped watching {path}", flush=True)

if __name__ == "__main__":
    main()

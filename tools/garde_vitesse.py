#!/usr/bin/env python3
"""Speed watchdog: tails newest dimos run log, computes speed from lidar odom
lines, and kills exploration if the body exceeds the envelope. It must be
running whenever the rover explores unattended.

Speed is measured between the TIMESTAMPS THE LOG CARRIES, never between the
moments the lines reach this process: LidarOdometry rate-limits `lidar odom #N`
(log_every_s = 2.0) and the interval jitters around it, so the old wall-clock
window (0.02 < dt < 2.0) threw away most samples - 183 of the 238 intervals in
recordings/courseB.jsonl - and could never strike in time. One interval above
the envelope is a kill; no cadence is assumed anywhere."""
import glob, math, os, re, subprocess, sys
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import explore_ctl  # the stop rides the run's transport, same module the flight uses

LIMIT_MS = 0.35        # hard envelope: cap 0.149 + margin; beyond = runaway
ODOM_RE = re.compile(r"lidar odom #(\d+): x=([+-][\d.]+) y=([+-][\d.]+)")
TS_RE = re.compile(r'"timestamp":\s*"([^"]+)"')

def newest_log() -> str:
    runs = sorted([d for d in glob.glob(os.path.expanduser("~/.local/state/dimos/logs/*")) if os.path.isdir(d)], key=os.path.getmtime)
    if not runs:
        sys.exit("no dimos run logs found")
    path = runs[-1]
    if os.path.isdir(path):
        path = os.path.join(path, "main.jsonl")
    return path

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
    path = newest_log()
    print(f"[garde] watching {path} (limit {LIMIT_MS} m/s)", flush=True)
    guard = SpeedGuard()
    paths = [path] + (["/tmp/explore_launch.log"] if os.path.exists("/tmp/explore_launch.log") else [])
    with subprocess.Popen(["tail", "-Fn0", *paths], stdout=subprocess.PIPE, text=True) as tail:
        for line in tail.stdout:
            if "BUMP" in line or "bump" in line:
                print(f"[garde] {line.rstrip()}", flush=True)
            v = guard.breach(line)
            if v is not None:
                print(f"[garde] {v:.2f} m/s over the {LIMIT_MS} m/s envelope", flush=True)
                stop_all()

if __name__ == "__main__":
    main()

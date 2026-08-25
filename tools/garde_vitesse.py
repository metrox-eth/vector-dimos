#!/usr/bin/env python3
"""Speed watchdog: tails newest dimos run log, computes speed from lidar odom
lines, and kills exploration if the body exceeds the envelope. Iris runs this
whenever the rover explores unattended (rule 19)."""
import glob, math, os, re, subprocess, sys, time

LIMIT_MS = 0.35        # hard envelope: cap 0.149 + margin; beyond = runaway
STRIKES_NEEDED = 3     # consecutive fast ticks before we pull the plug
ODOM_RE = re.compile(r"lidar odom #(\d+): x=([+-][\d.]+) y=([+-][\d.]+)")

def newest_log() -> str:
    runs = sorted([d for d in glob.glob(os.path.expanduser("~/.local/state/dimos/logs/*")) if os.path.isdir(d)], key=os.path.getmtime)
    if not runs:
        sys.exit("no dimos run logs found")
    path = runs[-1]
    if os.path.isdir(path):
        path = os.path.join(path, "main.jsonl")
    return path

def stop_all() -> None:
    subprocess.run([os.path.expanduser("~/vector-dimos/.venv/bin/python3"),
                    os.path.expanduser("~/vector-dimos/tools/explore_ctl.py"), "stop"])
    print("[garde] STOP EXPLORE published", flush=True)

def main() -> None:
    path = newest_log()
    print(f"[garde] watching {path} (limit {LIMIT_MS} m/s)", flush=True)
    strikes = 0
    prev = None  # (t, x, y)
    paths = [path] + (["/tmp/explore_launch.log"] if os.path.exists("/tmp/explore_launch.log") else [])
    with subprocess.Popen(["tail", "-Fn0", *paths], stdout=subprocess.PIPE, text=True) as tail:
        for line in tail.stdout:
            m = ODOM_RE.search(line)
            if "BUMP" in line or "bump" in line:
                print(f"[garde] {line.rstrip()}", flush=True)
            if not m:
                continue
            now = time.monotonic()
            x, y = float(m.group(2)), float(m.group(3))
            if prev is not None:
                dt = now - prev[0]
                if 0.02 < dt < 2.0:
                    v = math.hypot(x - prev[1], y - prev[2]) / dt
                    if v > LIMIT_MS:
                        strikes += 1
                        print(f"[garde] fast tick {v:.2f} m/s ({strikes}/{STRIKES_NEEDED})", flush=True)
                        if strikes >= STRIKES_NEEDED:
                            stop_all()
                            strikes = 0
                    else:
                        strikes = 0
            prev = (now, x, y)

if __name__ == "__main__":
    main()

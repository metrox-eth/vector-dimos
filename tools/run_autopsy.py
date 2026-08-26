"""Draw the FILM of a run: trajectory + every event, on the map it built.

Born 26/08 (owner: "tu n'arrives pas a voir ce que fait le robot"): reading a
log as event counters hides the behaviour. Three sessions read "35 stuck, 22
bumps" without seeing the story - the rover walking backwards across the flat
in 20 cm steps into the rear wall. One picture showed it in five seconds.

So: after EVERY run, draw the run BEFORE talking about it. Presumption of
stupidity - the picture answers "what did the robot do that was dumb?".

Run ON the Jetson (needs dimos for the costmap blobs + matplotlib):

    .venv/bin/python tools/run_autopsy.py                  # newest run, newest recording
    .venv/bin/python tools/run_autopsy.py --log ~/.local/state/dimos/logs/<run>/main.jsonl \
                                          --db ~/.local/state/vector/recordings/explore.db \
                                          --out /tmp/autopsy.png
"""
from __future__ import annotations

import argparse
import datetime
import glob
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

LOG_GLOB = "~/.local/state/dimos/logs/*-explore/main.jsonl"
DB_GLOB = "~/.local/state/vector/recordings/explore*.db"

# one entry per event family: (regex on the log line, marker, colour, legend label)
EVENTS = {
    "goal":    (re.compile(r"^goal \d+: \(([\d.\-]+), ([\d.\-]+)\)"), "*", "#2e7d32", "buts choisis"),
    "abandon": (re.compile(r"^Goal abandoned"), "o", "#c62828", "buts abandonnes (stuck)"),
    "escape":  (re.compile(r"^Contact escape"), "P", "#ef6c00", "escapes contact"),
    "bump":    (re.compile(r"BUMP received"), "v", "#111111", "chocs pare-chocs"),
    "arrived": (re.compile(r"^Arrived at goal"), "D", "#1b5e20", "buts atteints"),
    # legacy families, so a pre-26/08 log still draws its own disease
    "backup":  (re.compile(r"^Stuck: backed up"), "x", "#c62828", "reculs aveugles (legacy)"),
    "slip":    (re.compile(r"^(SLIP #|IMU SLIP #)"), "+", "#8e24aa", "slips (legacy)"),
}


def parse_ts(s: str) -> float:
    return datetime.datetime.fromisoformat(s.replace("Z", "+00:00")).timestamp()


def newest(pattern: str) -> str | None:
    hits = sorted(glob.glob(os.path.expanduser(pattern)), key=os.path.getmtime)
    return hits[-1] if hits else None


def read_log(path: str):
    """-> (events {family: [(t, x?, y?)]}, t0, t1). Goal lines carry their own xy."""
    events: dict[str, list] = {k: [] for k in EVENTS}
    t0 = t1 = None
    for line in open(path):
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        ts = e.get("timestamp")
        if not ts:
            continue
        t = parse_ts(ts)
        t0 = t if t0 is None else t0
        t1 = t
        msg = e.get("event", "")
        for fam, (rx, *_rest) in EVENTS.items():
            m = rx.search(msg) if fam == "bump" else rx.match(msg)
            if m:
                xy = (float(m.group(1)), float(m.group(2))) if fam == "goal" else None
                events[fam].append((t, xy))
    return events, t0, t1


def read_db(path: str, t0: float, t1: float):
    """-> (odom [(t,x,y)], occupied cell centres Nx2) for the run's time window."""
    import sqlite3

    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

    db = sqlite3.connect(path)
    odom = db.execute("SELECT ts, pose_x, pose_y FROM odom WHERE ts BETWEEN ? AND ? ORDER BY ts",
                      (t0 - 5, t1 + 5)).fetchall()
    occ = np.zeros((0, 2))
    row = db.execute("SELECT id FROM global_costmap WHERE ts <= ? ORDER BY ts DESC LIMIT 1",
                     (t1 + 5,)).fetchone()
    if row:
        blob = db.execute("SELECT data FROM global_costmap_blob WHERE id=?", (row[0],)).fetchone()[0]
        g = OccupancyGrid.lcm_decode(blob)
        ys, xs = np.nonzero(g.grid == 100)
        res = float(g.resolution)
        occ = np.stack([g.origin.position.x + (xs + 0.5) * res,
                        g.origin.position.y + (ys + 0.5) * res], axis=1)
    return odom, occ


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", default=None, help="main.jsonl of the run (default: newest)")
    ap.add_argument("--db", default=None, help="recording db (default: newest explore*.db)")
    ap.add_argument("--out", default=None, help="output PNG (default: next to the log)")
    args = ap.parse_args()

    log = args.log or newest(LOG_GLOB)
    db = args.db or newest(DB_GLOB)
    if not log or not db:
        print(f"missing input: log={log} db={db}")
        return 1
    out = args.out or os.path.join(os.path.dirname(log), "autopsy.png")

    events, t0, t1 = read_log(log)
    odom, occ = read_db(db, t0, t1)
    if len(odom) < 2:
        print(f"no odom rows in {db} for this run's window - wrong db?")
        return 1

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    ot = np.array([r[0] for r in odom]); ox = np.array([r[1] for r in odom]); oy = np.array([r[2] for r in odom])

    def pos_at(t):
        i = int(np.argmin(np.abs(ot - t)))
        return ox[i], oy[i]

    fig, ax = plt.subplots(figsize=(11, 9))
    fig.patch.set_facecolor("#fafaf8"); ax.set_facecolor("#fafaf8")
    if len(occ):
        ax.scatter(occ[:, 0], occ[:, 1], s=3, c="#555555", marker="s", linewidths=0)
    tnorm = (ot - ot[0]) / max(ot[-1] - ot[0], 1e-6)
    for i in range(len(ox) - 1):
        ax.plot(ox[i:i + 2], oy[i:i + 2], color=plt.cm.Blues(0.25 + 0.7 * tnorm[i]), lw=1.8, zorder=2)

    handles = [Line2D([], [], color="#7da7d9", lw=2, label="trajectoire (claire->foncee = temps)")]
    if len(occ):
        handles.append(Line2D([], [], marker="s", color="#555555", ls="none", ms=5, label="obstacles (carte)"))
    for fam, (_rx, mark, colour, label) in EVENTS.items():
        rows = events[fam]
        if not rows:
            continue
        for t, xy in rows:
            x, y = xy if xy else pos_at(t)
            ax.plot(x, y, marker=mark, ms=9, color=colour, mec="#fafaf8", mew=0.6, ls="none", zorder=6)
        handles.append(Line2D([], [], marker=mark, color=colour, ls="none", ms=9,
                              label=f"{label} ({len(rows)})"))

    ax.plot(ox[0], oy[0], "o", ms=11, color="#1d4f91", mec="white", mew=1.5, zorder=8)
    ax.annotate("DEPART", (ox[0], oy[0]), textcoords="offset points", xytext=(8, -14),
                fontsize=9, color="#1d4f91", weight="bold")
    ax.plot(ox[-1], oy[-1], "s", ms=11, color="#0d2b52", mec="white", mew=1.5, zorder=8)
    ax.annotate("FIN", (ox[-1], oy[-1]), textcoords="offset points", xytext=(8, 8),
                fontsize=9, color="#0d2b52", weight="bold")

    dist = float(np.hypot(np.diff(ox), np.diff(oy)).sum())
    mins = (t1 - t0) / 60.0
    n = {fam: len(rows) for fam, rows in events.items()}
    title = (f"{os.path.basename(os.path.dirname(log))} - {mins:.0f} min, {dist:.1f} m - "
             f"{n['goal']} buts, {n['arrived']} atteints, {n['abandon'] + n['backup']} stuck, "
             f"{n['bump']} chocs")
    ax.set_title(title, fontsize=11, color="#222222")
    ax.legend(handles=handles, loc="upper left", fontsize=9, framealpha=0.95)
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_aspect("equal"); ax.grid(True, color="#e0e0dc", lw=0.5)
    for sp in ax.spines.values():
        sp.set_color("#cccccc")
    plt.tight_layout()
    plt.savefig(out, dpi=130, facecolor="#fafaf8")

    print(f"LE FILM -> {out}")
    print(f"  {mins:.1f} min, {dist:.1f} m parcourus")
    print(f"  buts: {n['goal']} choisis, {n['arrived']} atteints, "
          f"{n['abandon']} abandonnes, {n['escape']} escapes contact, {n['bump']} chocs")
    if n["backup"] or n["slip"]:
        print(f"  (log d'avant le 26/08: {n['backup']} reculs aveugles, {n['slip']} slips)")
    per_min = (n["abandon"] + n["backup"] + n["bump"]) / max(mins, 0.1)
    print(f"  verdict: {per_min:.1f} evenements de blocage/choc par minute"
          + (" - REGARDE LA CARTE avant toute conclusion" if per_min > 1 else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Headline metrics across the lidar-range brackets.

    plot_brackets.py results_midstart_3m.json results_midstart_4m.json ...

One panel per metric, one pair of bars (stock, #2830) per range. Medians over
the paired runs of both configs, exactly the numbers make_outputs.py prints.
Writes brackets_midstart.png.
"""

from __future__ import annotations

import json
import os
import statistics
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_outputs as MO  # noqa: E402

ARM_COLOR = {"stock": "#d1495b", "pr2830": "#0f8b8d"}
ARM_LABEL = {"stock": "wavefront stock", "pr2830": "PR #2830"}

METRICS = [
    ("d_robot_to_goal_median_m", "median robot to goal (m)", "median"),
    ("share_goals_beyond_5m_pct", "share of goals beyond 5 m (%)", "median"),
    ("goal_jump_total_m", "goal to goal distance, total per run (m)", "median"),
    ("cross_map_swings", "cross-map swings, total over runs", "sum"),
    ("path_to_80pct_m", "path to 80 % of the ceiling (m)", "median"),
    ("path_m", "total path (m)", "median"),
]


def collect(path):
    with open(path) as fh:
        data = json.load(fh)
    rows = [r["summary"] for r in data["results"]]
    dead = MO.degenerate_starts(rows)
    live = [r for r in rows if (r["map"], r["start"], r["config"]) not in dead]
    return data["meta"].get("lidar_range_m", float("nan")), live


def value(rows, arm, key, how):
    v = [r[key] for r in rows if r["arm"] == arm and r.get(key) is not None
         and not (isinstance(r[key], float) and np.isnan(r[key]))]
    if not v:
        return float("nan")
    return float(sum(v)) if how == "sum" else float(statistics.median(v))


def main(argv=None):
    paths = argv or sys.argv[1:]
    if not paths:
        print("usage: plot_brackets.py results_*.json")
        return 1
    data = sorted((collect(p) for p in paths), key=lambda t: t[0])
    # Only the maps every bracket has, so the bars compare like with like: the
    # 4 m sweep covers both occupancy readings, the 3 m and 5 m brackets only
    # `bigoffice`, and mixing them would put a different denominator in each bar.
    common = set.intersection(*[{r["map"] for r in rows} for _, rows in data])
    data = [(rng, [r for r in rows if r["map"] in common]) for rng, rows in data]
    print(f"maps common to every bracket: {sorted(common)}")
    ranges = [d[0] for d in data]

    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.0))
    x = np.arange(len(ranges))
    width = 0.36
    for ax, (key, label, how) in zip(axes.ravel(), METRICS):
        for i, arm in enumerate(("stock", "pr2830")):
            vals = [value(rows, arm, key, how) for _, rows in data]
            bars = ax.bar(x + (i - 0.5) * width, vals, width,
                          color=ARM_COLOR[arm], alpha=0.9, label=ARM_LABEL[arm])
            ax.bar_label(bars, fmt="%.1f", fontsize=7, padding=1)
        ax.set_xticks(x)
        ax.set_xticklabels([f"{r:.0f} m" for r in ranges])
        ax.set_title(label, fontsize=9)
        ax.tick_params(labelsize=8)
        ax.grid(axis="y", alpha=0.25, lw=0.5)
        ax.set_axisbelow(True)
        ax.margins(y=0.18)                      # headroom for the bar labels
    axes[0, 0].legend(fontsize=8, loc="lower center", ncol=2, framealpha=0.9)
    axes[1, 0].set_xlabel("simulated lidar range", fontsize=8)
    fig.suptitle("Headline metrics against simulated lidar range, "
                 "middle-of-space starts, one map (bigoffice) and both configs pooled\n"
                 "medians over runs, except cross-map swings which is a total count",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    out = os.path.join(HERE, "brackets_midstart.png")
    fig.savefig(out, dpi=145)
    plt.close(fig)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())

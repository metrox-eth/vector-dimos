#!/usr/bin/env python3
"""Figures for the mid-start re-run. Same drawing as plot_bancs.py, English labels.

    plot_midstart.py results_midstart_4m.json

Writes:
  goal_sequences_midstart.png         bigoffice, config 'shipped', all 6 starts
  goal_sequences_midstart_hc.png      bigoffice_hc, same
  goal_sequences_midstart_scoring.png bigoffice, config 'scoring', all 6 starts
  coverage_vs_path_midstart.png       coverage against path, both configs
"""

from __future__ import annotations

import importlib.util
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)


def _import_explore_sim():
    if "explore_sim" in sys.modules:
        return sys.modules["explore_sim"]
    spec = importlib.util.spec_from_file_location(
        "explore_sim", "/home/openclaw/vector-dimos/tools/explore_sim.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()

MAP_FILE = {"bigoffice": "bigoffice.npz", "bigoffice_hc": "bigoffice_hc.npz"}
MAP_LABEL = {"bigoffice": "bigoffice (simple occupancy)",
             "bigoffice_hc": "bigoffice_hc (height-cost occupancy)"}

ARM_LABEL = {"stock": "wavefront stock (dimos @6fcc4e2)",
             "pr2830": "PR #2830 (info gain / A* path cost)"}
ARM_COLOR = {"stock": "#d1495b", "pr2830": "#0f8b8d"}

CMAP = ListedColormap(["#ffffff", "#e9e4da", "#3a3a3a"])   # unseen / free / wall


def load_world(map_name):
    return ES.load_world(os.path.join(SCRATCH, MAP_FILE[map_name]), None,
                         unknown_is_wall=True)


def background(ax, world):
    img = np.full(world.truth.shape, 0, dtype=np.uint8)
    img[world.observed & (world.truth != ES.OCCUPIED)] = 1
    img[world.truth == ES.OCCUPIED] = 2
    img[~world.observed] = 0
    h, w = world.truth.shape
    extent = (world.ox, world.ox + w * world.res, world.oy, world.oy + h * world.res)
    ax.imshow(img, origin="lower", extent=extent, cmap=CMAP, vmin=0, vmax=2,
              interpolation="nearest")
    ys, xs = np.nonzero(world.observed)
    pad = 0.6
    ax.set_xlim(world.ox + xs.min() * world.res - pad, world.ox + xs.max() * world.res + pad)
    ax.set_ylim(world.oy + ys.min() * world.res - pad, world.oy + ys.max() * world.res + pad)
    ax.set_aspect("equal")


def draw_run(ax, world, res, title, swing_m):
    background(ax, world)
    arm = res["summary"]["arm"]
    col = ARM_COLOR[arm]
    poses = np.array(res["poses"]) if res["poses"] else np.zeros((0, 2))
    if len(poses):
        ax.plot(poses[:, 0], poses[:, 1], "-", color=col, lw=1.4, alpha=0.85, zorder=3)
        ax.plot(poses[0, 0], poses[0, 1], "o", color="#111", ms=7, zorder=6)
    goals = res["goals"]
    # the jump the selector asked for: robot at issue time -> goal
    for g in goals:
        ax.annotate("", xy=(g["x"], g["y"]), xytext=(g["from_x"], g["from_y"]),
                    arrowprops=dict(arrowstyle="->", color=col, lw=0.9,
                                    alpha=0.55, shrinkA=0, shrinkB=0), zorder=4)
    # a cross-map swing: the segment from the previous goal to this one, when it
    # is longer than half the passable-floor bounding-box diagonal
    for a, b in zip(goals, goals[1:]):
        if b["d_prev_goal"] is not None and b["d_prev_goal"] > swing_m:
            ax.plot([a["x"], b["x"]], [a["y"], b["y"]], "-", color="#c77d00",
                    lw=2.6, alpha=0.85, zorder=5)
    for g in goals:
        ax.plot(g["x"], g["y"], "o", color="white", ms=11, zorder=7,
                markeredgecolor=col, markeredgewidth=1.3)
        ax.text(g["x"], g["y"], str(g["index"] + 1), ha="center", va="center",
                fontsize=6.5, color=col, zorder=8, fontweight="bold")
    ax.set_title(title, fontsize=8.5, loc="left")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#bbb")


def panel_title(res):
    s = res["summary"]
    return (f"{ARM_LABEL[s['arm']]}\n"
            f"{s['n_goals']} goals, median jump {s['d_robot_to_goal_median_m']:.2f} m, "
            f"max {s['d_robot_to_goal_max_m']:.2f} m\n"
            f"{s['path_m']:.1f} m driven, {s['coverage_pct']:.0f} % coverage\n"
            f"goal to goal {s['goal_jump_total_m']:.1f} m, "
            f"{s['cross_map_swings']} swing(s)")


def main(argv=None):
    a = argv or sys.argv[1:]
    path = a[0] if a else os.path.join(HERE, "results_midstart_4m.json")
    with open(path) as fh:
        data = json.load(fh)
    results = data["results"]
    lidar = data["meta"].get("lidar_range_m", float("nan"))

    def pick(m, st, cfg, arm):
        for r in results:
            s = r["summary"]
            if (s["map"], s["start"], s["config"], s["arm"]) == (m, st, cfg, arm):
                return r
        return None

    maps = sorted({r["summary"]["map"] for r in results})
    worlds = {m: load_world(m) for m in maps}

    # ---- goal sequences, one figure per (map, config) ----------------------
    outputs = [("bigoffice", "shipped", "goal_sequences_midstart.png"),
               ("bigoffice_hc", "shipped", "goal_sequences_midstart_hc.png"),
               ("bigoffice", "scoring", "goal_sequences_midstart_scoring.png"),
               ("bigoffice_hc", "scoring", "goal_sequences_midstart_hc_scoring.png")]
    blurb = {"shipped": "upstream code as-is (get_exploration_goal), 45 s goal timeout",
             "scoring": "scoring only: no self-stop, no goal timeout, house failed-goal filter"}
    for m, config, fname in outputs:
        if m not in maps:
            continue
        starts = sorted({r["summary"]["start"] for r in results
                         if r["summary"]["map"] == m},
                        key=lambda s: (s != "centre", s))
        swing = next((r["summary"]["swing_threshold_m"] for r in results
                      if r["summary"]["map"] == m), float("inf"))
        fig, axes = plt.subplots(len(starts), 2, figsize=(11.5, 4.3 * len(starts)))
        axes = np.atleast_2d(axes)
        for i, st in enumerate(starts):
            for j, arm in enumerate(("stock", "pr2830")):
                r = pick(m, st, config, arm)
                if r is None:
                    axes[i, j].axis("off")
                    continue
                draw_run(axes[i, j], worlds[m], r, panel_title(r), swing)
            axes[i, 0].set_ylabel(f"start '{st}'", fontsize=9, fontweight="bold")
        fig.suptitle(
            f"{MAP_LABEL.get(m, m)}: wavefront stock (left) vs PR #2830 (right)\n"
            f"middle-of-space starts, {lidar:.0f} m lidar, config '{config}': {blurb[config]}\n"
            "black dot = start, arrows = requested jump (robot to goal), "
            "numbers = goal order, orange segment = cross-map swing "
            f"(consecutive goals more than {swing:.1f} m apart)",
            fontsize=10)
        fig.tight_layout(rect=(0, 0, 1, 0.985))
        fig.savefig(os.path.join(HERE, fname), dpi=140)
        plt.close(fig)
        print(fname)

    # ---- coverage against path --------------------------------------------
    configs = ["shipped", "scoring"]
    fig, axes = plt.subplots(len(configs), len(maps),
                             figsize=(4.6 * len(maps), 3.8 * len(configs)), squeeze=False)
    for row, config in enumerate(configs):
        for k, m in enumerate(maps):
            ax = axes[row, k]
            for arm in ("stock", "pr2830"):
                first = True
                for r in results:
                    s = r["summary"]
                    if s["map"] != m or s["arm"] != arm or s["config"] != config:
                        continue
                    c = np.array(r["coverage_curve"])
                    if not len(c):
                        continue
                    ax.plot(c[:, 0], 100 * c[:, 1] / s["ceiling_m2"], color=ARM_COLOR[arm],
                            alpha=0.75, lw=1.3, label=ARM_LABEL[arm] if first else None)
                    first = False
            ax.set_title(f"{m}, config '{config}'", fontsize=9)
            ax.set_xlabel("path driven (m)", fontsize=8)
            if k == 0:
                ax.set_ylabel("coverage (% of visible ceiling)", fontsize=8)
            ax.grid(alpha=0.25, lw=0.5)
            ax.tick_params(labelsize=7)
    axes[0, 0].legend(fontsize=7, loc="lower right")
    fig.suptitle(f"Coverage against path, one line per start, {lidar:.0f} m lidar, "
                 "middle-of-space starts", fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(os.path.join(HERE, "coverage_vs_path_midstart.png"), dpi=145)
    plt.close(fig)
    print("coverage_vs_path_midstart.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())

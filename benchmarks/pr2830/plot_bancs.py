#!/usr/bin/env python3
"""Draw the A/B: stock wavefront vs PR #2830, same map, same start, same scale.

    plot_bancs.py results.json

Writes bancs.png (one row per map, stock left / PR right) and one
banc_<map>.png per map with every start pose, plus couverture.png.
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
    spec = importlib.util.spec_from_file_location(
        "explore_sim", "/home/openclaw/vector-dimos/tools/explore_sim.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()

MAP_FILE = {
    "map_20260823": "map_20260823.npz",
    "map_20260825": "map_20260825.npz",
    "map_20260827": "map_20260827.npz",
    "costmap_175224": "costmap_175224.npz",
    "costmap_175905": "costmap_175905.npz",
}

ARM_LABEL = {"stock": "wavefront stock (dimos @6fcc4e2)",
             "pr2830": "PR #2830 (info gain / A* path cost)"}
ARM_COLOR = {"stock": "#d1495b", "pr2830": "#0f8b8d"}

# free / occupied / never observed
CMAP = ListedColormap(["#ffffff", "#e9e4da", "#3a3a3a"])


def load_world(map_name):
    path = os.path.join(SCRATCH, MAP_FILE[map_name])
    return ES.load_world(path, None, unknown_is_wall=True)


def background(ax, world):
    img = np.full(world.truth.shape, 0, dtype=np.uint8)   # unobserved -> white
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


def draw_run(ax, world, res, title):
    background(ax, world)
    arm = res["summary"]["arm"]
    col = ARM_COLOR[arm]
    poses = np.array(res["poses"]) if res["poses"] else np.zeros((0, 2))
    if len(poses):
        ax.plot(poses[:, 0], poses[:, 1], "-", color=col, lw=1.4, alpha=0.85, zorder=3)
        ax.plot(poses[0, 0], poses[0, 1], "o", color="#111", ms=7, zorder=6)
    goals = res["goals"]
    for g in goals:
        # the jump the selector asked for: robot at issue time -> goal
        ax.annotate("", xy=(g["x"], g["y"]), xytext=(g["from_x"], g["from_y"]),
                    arrowprops=dict(arrowstyle="->", color=col, lw=0.9,
                                    alpha=0.55, shrinkA=0, shrinkB=0), zorder=4)
    for g in goals:
        ax.plot(g["x"], g["y"], "o", color="white", ms=11, zorder=7,
                markeredgecolor=col, markeredgewidth=1.3)
        ax.text(g["x"], g["y"], str(g["index"] + 1), ha="center", va="center",
                fontsize=6.5, color=col, zorder=8, fontweight="bold")
    ax.set_title(title, fontsize=8.5, loc="left")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#bbb")


def panel_title(res):
    s = res["summary"]
    dm = s["d_robot_to_goal_median_m"]
    return (f"{ARM_LABEL[s['arm']]}\n"
            f"{s['n_goals']} goals · med jump {dm:.2f} m · max {s['d_robot_to_goal_max_m']:.2f} m\n"
            f"{s['path_m']:.1f} m driven · {s['coverage_pct']:.0f} % coverage")


def main(argv=None):
    path = (argv or sys.argv[1:])[0] if (argv or sys.argv[1:]) else os.path.join(HERE, "results.json")
    with open(path) as fh:
        data = json.load(fh)
    results = data["results"]

    def pick(map_name, start, config, arm):
        for r in results:
            s = r["summary"]
            if (s["map"], s["start"], s["config"], s["arm"]) == (map_name, start, config, arm):
                return r
        return None

    maps = sorted({r["summary"]["map"] for r in results})
    worlds = {m: load_world(m) for m in maps}

    def best_start(m, config):
        """The start where the most driving happened, summed over BOTH arms.

        An arm-neutral rule: it cannot prefer one strategy, and it keeps the
        summary figure off the starts where the flat's geometry, not the
        scoring, decided the run (a nook the 0.46 m body cannot leave).
        """
        best, score = None, -1.0
        for st in sorted({r["summary"]["start"] for r in results
                          if r["summary"]["map"] == m}):
            a, b = pick(m, st, config, "stock"), pick(m, st, config, "pr2830")
            if a is None or b is None:
                continue
            s = a["summary"]["path_m"] + b["summary"]["path_m"]
            if s > score:
                best, score = st, s
        return best

    # ---- bancs.png / bancs_scoring.png : one row per map ------------------
    for config, fname, blurb in (
            ("shipped", "bancs.png",
             "upstream code as-is (get_exploration_goal), 45 s goal timeout"),
            ("scoring", "bancs_scoring.png",
             "scoring only: no self-stop, no goal timeout, our failed-goal filter")):
        rows = [(m, best_start(m, config)) for m in maps]
        rows = [(m, st) for m, st in rows if st]
        fig, axes = plt.subplots(len(rows), 2, figsize=(11.5, 4.6 * len(rows)))
        axes = np.atleast_2d(axes)
        for i, (m, st) in enumerate(rows):
            for j, arm in enumerate(("stock", "pr2830")):
                draw_run(axes[i, j], worlds[m], pick(m, st, config, arm),
                         panel_title(pick(m, st, config, arm)))
            axes[i, 0].set_ylabel(f"{m}\nstart '{st}'", fontsize=9, fontweight="bold")
        fig.suptitle("dimOS frontier exploration — wavefront stock vs PR #2830\n"
                     "real recorded maps · same world, same planner, same robot · "
                     f"config '{config}': {blurb}\n"
                     "black dot = start · arrows = requested jump (robot → goal) · "
                     "numbers = goal order",
                     fontsize=10, y=0.999)
        fig.tight_layout(rect=(0, 0, 1, 0.975))
        fig.savefig(os.path.join(HERE, fname), dpi=145)
        plt.close(fig)
        print(fname)

    # ---- one PNG per map, every start (config « shipped ») ----------------
    config = "shipped"
    for m in maps:
        starts = sorted({r["summary"]["start"] for r in results
                         if r["summary"]["map"] == m})
        fig, axes = plt.subplots(len(starts), 2, figsize=(11.5, 4.3 * len(starts)))
        axes = np.atleast_2d(axes)
        for i, st in enumerate(starts):
            for j, arm in enumerate(("stock", "pr2830")):
                r = pick(m, st, config, arm)
                if r is None:
                    axes[i, j].axis("off")
                    continue
                draw_run(axes[i, j], worlds[m], r, panel_title(r))
            axes[i, 0].set_ylabel(f"start '{st}'", fontsize=9, fontweight="bold")
        fig.suptitle(f"{m}: wavefront stock (left) vs PR #2830 (right), "
                     f"config '{config}', all start poses", fontsize=11)
        fig.tight_layout(rect=(0, 0, 1, 0.99))
        fig.savefig(os.path.join(HERE, f"banc_{m}.png"), dpi=140)
        plt.close(fig)
        print(f"banc_{m}.png")
    config = "scoring"

    # ---- coverage curves ---------------------------------------------------
    fig, axes = plt.subplots(1, len(maps), figsize=(4.2 * len(maps), 3.8), squeeze=False)
    for k, m in enumerate(maps):
        ax = axes[0, k]
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
        ax.set_title(m, fontsize=9)
        ax.set_xlabel("path driven (m)", fontsize=8)
        if k == 0:
            ax.set_ylabel("coverage (% of visible ceiling)", fontsize=8)
        ax.grid(alpha=0.25, lw=0.5)
        ax.tick_params(labelsize=7)
    axes[0, 0].legend(fontsize=7, loc="lower right")
    fig.suptitle("Coverage vs path, one line per start pose, "
                 "config 'scoring' (each arm pays the path its choices cost)",
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(os.path.join(HERE, "couverture.png"), dpi=145)
    plt.close(fig)
    print("couverture.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())

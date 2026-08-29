"""Figures for the fidelity runs: map + full trajectory + goal order, stock vs
stock+M4.3 side by side, faithful timing (T_sel=15 s, 0.55 m/s)."""
import json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

S = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(S, "out")
CMAP = ListedColormap(["#ffffff", "#e9e4da", "#3a3a3a", "#7fbf7f"])

def load_runs(fn):
    d = json.load(open(os.path.join(OUT, fn)))
    return d["results"] if isinstance(d, dict) and "results" in d else d

def world(map_name):
    z = np.load(os.path.join("/tmp/claude-1000/-home-openclaw-lerobot/c23c4729-e991-4dc8-b2d5-9c140c6a8780/scratchpad", f"{map_name}.npz"))
    return z

def draw(ax, z, run, title):
    lid = z["lidar"]; res = float(z["res"]); ox = float(z["ox"]); oy = float(z["oy"])
    seen = z["seen"]
    img = np.zeros_like(lid, dtype=np.int8)   # 0 = never observed (white)
    img[seen & (lid == 0)] = 1                # free (beige)
    img[lid == 3] = 2                         # occupied (dark grey)
    h, w = lid.shape
    extent = [ox, ox + w * res, oy, oy + h * res]
    ax.imshow(img, origin="lower", extent=extent, cmap=CMAP, vmin=0, vmax=3,
              interpolation="nearest")
    p = np.array(run["poses"])
    ax.plot(p[:, 0], p[:, 1], "-", color="#0f5fd8", lw=1.0, alpha=0.85)
    ax.plot(p[0, 0], p[0, 1], "o", color="#111", ms=7, zorder=5)
    goals = run["goals"]
    diag = 0.5 * math.hypot(w * res, h * res)
    prev = None
    for g in goals:
        x, y = g["x"], g["y"]
        ax.annotate(str(g["index"] + 1), (x, y), fontsize=7, color="#b3232e",
                    ha="center", va="center",
                    bbox=dict(boxstyle="circle,pad=0.15", fc="#fff", ec="#b3232e", lw=0.8))
        if prev is not None and math.hypot(x - prev[0], y - prev[1]) > diag * 0.5:
            ax.plot([prev[0], x], [prev[1], y], "-", color="#d98a00", lw=2.2, alpha=0.9)
        prev = (x, y)
    s = run["summary"]
    ax.set_title(f"{title}\n{s['n_goals']} goals, {s['path_m']:.0f} m driven, "
                 f"{s['coverage_pct']:.0f}% coverage", fontsize=9)
    ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])

def page(map_name, rng, config, out_png):
    stock = {r["summary"]["start"]: r for r in load_runs(f"fidA_{rng}_{config}_t15_{map_name}.json")}
    fix = {r["summary"]["start"]: r
           for r in load_runs(f"fidB_{rng}_{config}_t15_m43_{map_name}.json")
           if r["summary"]["arm"] != "stock"}
    z = world(map_name)
    starts = [s for s in ["centre","mid1","mid2","mid3","mid4","mid5"] if s in stock and s in fix]
    fig, axes = plt.subplots(len(starts), 2, figsize=(11, 5.2 * len(starts)))
    if len(starts) == 1: axes = np.array([axes])
    for i, st in enumerate(starts):
        draw(axes[i][0], z, stock[st], f"stock, start '{st}'")
        draw(axes[i][1], z, fix[st], f"stock + signed momentum, start '{st}'")
    fig.suptitle(f"{map_name}, {rng} lidar, config '{config}', faithful timing "
                 f"(T_sel 15 s, 0.55 m/s, old goal active during compute)", fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.98])
    fig.savefig(os.path.join(S, out_png), dpi=130)
    plt.close(fig)
    print(out_png)

page("hk_park", "12m", "shipped", "fid_hk_park_12m_shipped.png")
page("hk_elevator", "12m", "shipped", "fid_hk_elevator_12m_shipped.png")
page("hk_entrance", "4m", "scoring", "fid_hk_entrance_4m_scoring.png")

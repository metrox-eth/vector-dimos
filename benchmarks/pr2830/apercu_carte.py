#!/usr/bin/env python3
"""Draw the extracted map + the Go2's own trajectory, so the input is looked at
before it is used. Writes carte_bigoffice.png."""

from __future__ import annotations

import importlib.util
import os
import sqlite3
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from scipy import ndimage  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SCRATCH = os.path.dirname(HERE)
DB = os.path.join(SCRATCH, "go2_bigoffice_work.db")


def _import_explore_sim():
    spec = importlib.util.spec_from_file_location(
        "explore_sim", "/home/openclaw/vector-dimos/tools/explore_sim.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()
CMAP = ListedColormap(["#ffffff", "#e9e4da", "#3a3a3a", "#7fbf7f"])


def odom_xy():
    conn = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
    rows = conn.execute("select pose_x, pose_y from odom order by ts").fetchall()
    conn.close()
    return np.asarray(rows, dtype=float)


def main(npz=None, out=None):
    npz = npz or os.path.join(SCRATCH, "bigoffice.npz")
    out = out or os.path.join(HERE, "carte_bigoffice.png")
    world = ES.load_world(npz, None, unknown_is_wall=True)
    res = world.res
    ok = (world.truth == ES.FREE) & (world.clearance + 1e-6 >= ES.BODY_HALF_WIDTH_M)
    labels, n = ndimage.label(ok, structure=ES._EIGHT)
    sizes = ndimage.sum(ok, labels, index=np.arange(1, n + 1))
    body = labels == (int(np.argmax(sizes)) + 1)

    img = np.zeros(world.truth.shape, dtype=np.uint8)
    img[world.observed & (world.truth != ES.OCCUPIED)] = 1
    img[world.truth == ES.OCCUPIED] = 2
    img[~world.observed] = 0
    img[body] = 3

    h, w = world.truth.shape
    extent = (world.ox, world.ox + w * res, world.oy, world.oy + h * res)
    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.imshow(img, origin="lower", extent=extent, cmap=CMAP, vmin=0, vmax=3,
              interpolation="nearest")
    p = odom_xy()
    ax.plot(p[:, 0], p[:, 1], "-", color="#0f5fd8", lw=1.2, alpha=0.9,
            label="Go2 trajectory (odom, 5465 poses)")
    ax.plot(p[0, 0], p[0, 1], "o", color="#111", ms=7)
    ax.set_aspect("equal")
    ax.legend(fontsize=8, loc="lower left")
    ax.set_title(f"go2_bigoffice, extracted map ({w}x{h} @ {res} m = "
                 f"{w * res:.1f} x {h * res:.1f} m)\n"
                 f"white = never observed · beige = free · grey = occupied · "
                 f"green = connected floor reachable by a 0.46 m body "
                 f"({body.sum() * res * res:.0f} m2)", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print(f"-> {out}")


if __name__ == "__main__":
    main(*sys.argv[1:])

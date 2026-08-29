#!/usr/bin/env python3
"""Middle-of-space start poses, for the mid-start re-run of the #2830 bench.

The shipped bench started from `origin`, `pose` (where the real run began) and
farthest-point samples, which on this map put several starts at the far end of
the 20 m corridor. dimOS maintainer lesh: from a dead end you cannot see the
behaviour the PR targets, because there is only one way to go. Spawn in the
middle, where several corridors meet, and the choice of frontier actually
decides where the rover walks next.

So this module picks starts that are, by construction, in the middle:

  centre   unchanged from the shipped bench: the body-passable cell closest to
           the centroid of the largest body-passable region.
  mid1..5  cells that pass three filters and are then spread apart:
           1. GRAPH CENTRE. Approximate geodesic eccentricity inside the
              body-passable floor (max geodesic distance to a 1 m lattice of
              that floor); keep the lowest ECC_PCT % of it. A dead end has the
              highest eccentricity there is, so this is exactly the filter that
              removes the starts lesh objected to.
           2. CLEARANCE. Distance transform to the nearest ground-truth
              obstacle at least CLEAR_MIN_M, so the spawn is in the open, not
              wedged in a pinch.
           3. NOT A DEAD END. Cast N_DIRS rays; a direction is open if the body
              could drive PROBE_M along it. Count the CONTIGUOUS angular runs of
              open directions: a dead end gives 1, a corridor 2, a T 3, a
              crossing 4. Require at least MIN_RUNS.
           Survivors are then chosen by farthest-point sampling seeded on
           `centre`, so the five are spread over the central region instead of
           piling into one spot. The junction criterion enters as the TIE-BREAK
           of that sampling: among the candidates within TIE_BAND of the best
           spread, the one with the most ways out wins, then the most clearance.
           It is a preference and not a hard filter because on this particular
           map only 19 of the 74 central candidate cells have three ways out and
           they sit in two clusters - filtering on it hard would return five
           near-duplicate starts, which is not what was asked for.

Nothing here touches either arm or the simulator. It only chooses input poses,
and both arms get the identical set.
"""

from __future__ import annotations

import importlib.util
import math
import os
import sys

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

HERE = os.path.dirname(os.path.abspath(__file__))


def _import_explore_sim():
    """The ONE explore_sim instance of this process.

    It matters: the lidar range is a module constant read at call time, so two
    copies of the module would mean one of them silently still at 12 m.
    """
    if "explore_sim" in sys.modules:
        return sys.modules["explore_sim"]
    path = "/home/openclaw/vector-dimos/tools/explore_sim.py"
    spec = importlib.util.spec_from_file_location("explore_sim", path)
    assert spec and spec.loader
    m = importlib.util.module_from_spec(spec)
    sys.modules["explore_sim"] = m
    spec.loader.exec_module(m)
    return m


ES = _import_explore_sim()

ECC_PCT = 60.0        # keep the 60 % of the floor closest to the graph centre
CLEAR_MIN_M = 0.35    # 12 cm more than the 0.23 m body half-width
N_DIRS = 32
PROBE_M = 1.5
MIN_RUNS = 2          # at least two ways out, i.e. never a dead end
TIE_BAND = 0.9        # a candidate within 90 % of the best spread is a tie
N_MID = 5


def passable(world) -> np.ndarray:
    """Free ground the 0.46 m body fits on, largest connected piece.

    Identical rule to bench_2830._passable; duplicated here so the gate script
    can import this module without importing the bench.
    """
    ok = (world.truth == ES.FREE) & (world.clearance + 1e-6 >= ES.BODY_HALF_WIDTH_M)
    labels, n = ndimage.label(ok, structure=ES._EIGHT)
    if n == 0:
        return ok
    sizes = ndimage.sum(ok, labels, index=np.arange(1, n + 1))
    return labels == (int(np.argmax(sizes)) + 1)


def _graph(mask: np.ndarray):
    """8-connected graph over the True cells of `mask`, weights in cells."""
    ys, xs = np.nonzero(mask)
    n = int(ys.size)
    index = np.full(mask.shape, -1, dtype=np.int64)
    index[ys, xs] = np.arange(n)
    rows, cols, vals = [], [], []
    h, w = mask.shape
    for dy, dx, length in ES._NEIGHBOURS:
        y0, y1 = max(0, -dy), h - max(0, dy)
        x0, x1 = max(0, -dx), w - max(0, dx)
        a = index[y0:y1, x0:x1]
        b = index[y0 + dy:y1 + dy, x0 + dx:x1 + dx]
        both = (a >= 0) & (b >= 0)
        if not both.any():
            continue
        rows.append(a[both])
        cols.append(b[both])
        vals.append(np.full(int(both.sum()), length))
    graph = csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(n, n))
    return graph, ys, xs, index


def eccentricity(mask: np.ndarray, res: float, sample_m: float = 1.0) -> np.ndarray:
    """Approximate geodesic eccentricity, in metres, over the cells of `mask`.

    Exact eccentricity is a shortest path from every cell to every cell. This
    takes the max over a lattice of sources instead: the true eccentricity of a
    cell is attained at some extreme point of the region, and a 1 m lattice
    lands within 0.7 m of any such point, so the ranking this produces is the
    ranking of the exact quantity up to that margin. Only the RANKING is used.
    """
    graph, ys, xs, index = _graph(mask)
    step = max(1, int(round(sample_m / res)))
    src = [i for i, (y, x) in enumerate(zip(ys, xs)) if y % step == 0 and x % step == 0]
    if not src:
        src = [0]
    d = dijkstra(graph, directed=False, indices=src)      # (len(src), n) in cells
    d[~np.isfinite(d)] = 0.0
    ecc_nodes = d.max(axis=0) * res
    out = np.full(mask.shape, np.inf)
    out[ys, xs] = ecc_nodes
    return out


def open_runs(world, gy: int, gx: int, n_dirs: int = N_DIRS,
              probe_m: float = PROBE_M) -> int:
    """How many DISTINCT directions can the body leave this cell in?

    A direction is open when every cell along it, out to probe_m, is free
    ground with at least the body half-width of clearance. Contiguous open
    directions are one way out; the count of such circular runs is the number of
    ways out. Dead end 1, corridor 2, T 3, crossing 4.
    """
    res = world.res
    h, w = world.truth.shape
    steps = max(1, int(round(probe_m / res)))
    angles = np.linspace(0.0, 2 * math.pi, n_dirs, endpoint=False)
    r = (np.arange(1, steps + 1) * res)[None, :]
    cy = (gy + 0.5) * res
    cx = (gx + 0.5) * res
    yy = np.floor((cy + np.sin(angles)[:, None] * r) / res).astype(np.int64)
    xx = np.floor((cx + np.cos(angles)[:, None] * r) / res).astype(np.int64)
    inside = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
    yc, xc = np.clip(yy, 0, h - 1), np.clip(xx, 0, w - 1)
    ok = (inside & (world.truth[yc, xc] == ES.FREE)
          & (world.clearance[yc, xc] + 1e-6 >= ES.BODY_HALF_WIDTH_M))
    open_dir = ok.all(axis=1)
    if open_dir.all():
        return 1
    if not open_dir.any():
        return 0
    # circular runs of True
    idx = np.nonzero(~open_dir)[0][0]
    rolled = np.roll(open_dir, -idx)
    return int(np.sum(rolled[1:] & ~rolled[:-1])) + int(rolled[0])


def choose_mid_starts(world, n_mid: int = N_MID, verbose: bool = True):
    """centre + mid1..mid_n, plus the diagnostics that justify each pick."""
    body = passable(world)
    ys, xs = np.nonzero(body)
    if ys.size == 0:
        return {}, {}
    res = world.res

    # centre: unchanged from the shipped bench
    cy, cx = ys.mean(), xs.mean()
    i = int(np.argmin((ys - cy) ** 2 + (xs - cx) ** 2))
    centre_yx = (int(ys[i]), int(xs[i]))

    ecc = eccentricity(body, res)
    ecc_vals = ecc[ys, xs]
    cut = float(np.percentile(ecc_vals, ECC_PCT))
    central = body & (ecc <= cut)

    clear_ok = world.clearance + 1e-6 >= CLEAR_MIN_M
    cand = central & clear_ok
    cys, cxs = np.nonzero(cand)

    # ways out, on a 0.5 m lattice of the candidates (one ray cast per cell)
    step = max(1, int(round(0.5 / res)))
    lattice = [(int(y), int(x)) for y, x in zip(cys, cxs)
               if y % step == 0 and x % step == 0]
    runs = {yx: open_runs(world, *yx) for yx in lattice}
    keep = [yx for yx, r in runs.items() if r >= MIN_RUNS]
    min_runs_used = MIN_RUNS
    if len(keep) < n_mid:
        keep = list(runs)
        min_runs_used = 0
    n_junction = sum(1 for yx in keep if runs[yx] >= 3)

    # farthest-point sampling over the survivors, seeded on `centre`, with the
    # junction criterion as the tie-break inside the TIE_BAND
    chosen = [centre_yx]
    pool = list(keep)
    picks = []
    for _ in range(n_mid):
        if not pool:
            break
        arr = np.array(pool, dtype=float)
        d = np.full(len(pool), np.inf)
        for gy, gx in chosen:
            d = np.minimum(d, (arr[:, 0] - gy) ** 2 + (arr[:, 1] - gx) ** 2)
        tie = [i for i in range(len(pool)) if d[i] >= (TIE_BAND ** 2) * d.max()]
        tie.sort(key=lambda i: (-runs[pool[i]], -float(world.clearance[pool[i]])))
        yx = pool.pop(tie[0])
        chosen.append(yx)
        picks.append(yx)

    starts: dict[str, tuple[float, float]] = {"centre": world.world_xy(*centre_yx)}
    for k, yx in enumerate(picks, start=1):
        starts[f"mid{k}"] = world.world_xy(*yx)

    diag = {}
    for name, yx in [("centre", centre_yx)] + [(f"mid{k}", yx)
                                               for k, yx in enumerate(picks, start=1)]:
        diag[name] = {
            "cell": [int(yx[0]), int(yx[1])],
            "xy": list(world.world_xy(*yx)),
            "clearance_m": float(world.clearance[yx]),
            "open_directions": int(open_runs(world, *yx)),
            "eccentricity_m": float(ecc[yx]),
        }
    pts = np.array([d["xy"] for d in diag.values()])
    sep = min((math.hypot(*(pts[a] - pts[b]))
               for a in range(len(pts)) for b in range(a + 1, len(pts))),
              default=float("nan"))
    meta = {
        "ecc_min_m": float(ecc_vals.min()), "ecc_max_m": float(ecc_vals.max()),
        "ecc_cut_m": cut, "ecc_pct": ECC_PCT,
        "clear_min_m": CLEAR_MIN_M, "min_open_directions": min_runs_used,
        "n_candidate_cells": int(cand.sum()), "n_lattice": len(lattice),
        "n_survivors": len(keep), "n_survivors_with_3_ways_out": n_junction,
        "min_pairwise_separation_m": float(sep),
        "body_bbox_diagonal_m": body_bbox_diagonal_m(world),
        "starts": diag,
    }
    if verbose:
        print(f"  eccentricity over the passable floor: {ecc_vals.min():.1f} to "
              f"{ecc_vals.max():.1f} m, central cut at {cut:.1f} m ({ECC_PCT:.0f}th pct)")
        print(f"  candidates: {int(cand.sum())} cells central and clear, "
              f"{len(lattice)} on the 0.5 m lattice, {len(keep)} with at least "
              f"{min_runs_used} ways out ({n_junction} of them with 3 or more)")
        for name, d in diag.items():
            print(f"    {name:7s} ({d['xy'][0]:6.2f},{d['xy'][1]:6.2f})  "
                  f"clearance {d['clearance_m']:.2f} m  "
                  f"ways out {d['open_directions']}  "
                  f"eccentricity {d['eccentricity_m']:.1f} m")
        print(f"  closest pair of starts: {sep:.2f} m")
    return starts, meta


def body_bbox_diagonal_m(world) -> float:
    """Diagonal of the bounding box of the body-passable floor, in metres.

    This is the extent of the space the rover can actually be in, and it is what
    a "cross-map swing" is measured against.
    """
    body = passable(world)
    ys, xs = np.nonzero(body)
    if ys.size == 0:
        return 0.0
    bw = (xs.max() - xs.min() + 1) * world.res
    bh = (ys.max() - ys.min() + 1) * world.res
    return float(math.hypot(bw, bh))


def main() -> int:
    scratch = os.path.dirname(HERE)
    for name in ("bigoffice", "bigoffice_hc"):
        path = os.path.join(scratch, f"{name}.npz")
        if not os.path.exists(path):
            continue
        world = ES.load_world(path, None, unknown_is_wall=True)
        print(f"\n=== {name} ===  passable bbox diagonal "
              f"{body_bbox_diagonal_m(world):.1f} m")
        choose_mid_starts(world)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

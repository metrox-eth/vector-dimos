"""The map as an asset that survives the stack: files, generations, keep-outs.

Every restart used to birth an amnesiac map with a fresh arbitrary origin.
This module holds the small amount of state that makes the flat ONE place
across sessions:

  ~/.local/state/vector/persistent_map.npz          the map to come back to
  ~/.local/state/vector/persistent_map.<stamp>.npz  the previous generations
  ~/.local/state/vector/keepout.json                the zones the owner drew on the map
  ~/.local/state/vector/checkpoints/<run>/*.npz     what a live run writes (costmap2d)

The map file IS a costmap checkpoint: `ScoredGrid.save()` already writes
everything needed (both score layers, `seen`, resolution and origin), so
promoting a run into the persistent map is a file copy, not a conversion.

Switches (environment, read at each call so a test can flip them):
  PERSISTENT_MAP=0          turn the whole feature off: relocalization, the
                            persistent map, the keep-outs. The rover behaves
                            exactly as it did before this existed.
  PERSISTENT_MAP_REBASE=1   let a run that did NOT relocalize overwrite the
                            persistent map. Off by default, on purpose: a
                            fresh-frame map replacing a good one would move
                            the flat under the keep-out zones, which are
                            coordinates in the persistent frame.

Zones (keepout.json, written by the drawing UI `tools/zone_server.py`, by
`tools/keepout.py`, or by the owner's hand):

    {"frame": "<which map these coordinates belong to>",
     "zones": [{"label": ..., "type": ..., "x0":, "y0":, "x1":, "y1":, "note": ...},
               {"label": ..., "type": ..., "points": [[x, y], ...], "note": ...}]}

A zone has one of two shapes, and both are read everywhere a zone is read:

  rectangle  x0/y0/x1/y1, axis-aligned - what the CLI writes, and every zone
             written before 26/08.
  polygon    `points`, at least 3 vertices in metres, in the persistent frame -
             what the owner draws with the mouse. The house is tilted 5.75 deg
             in the map frame, so its keep-out edges are not axis-aligned and
             an enclosing rectangle either eats the corridor or leaks the
             corner. `points` wins if a zone somehow carries both shapes.

One type:

  forbidden       the cells become occupied in `occupancy()`, after every
                  layer: nothing erases them and the planner never enters.
                  (The toilets: a 3 cm step at the door.)

(`no_slip_reflex` zones died with the slip detectors on 26/08 - the contact
switches replaced them; an old zone file may still carry some, they are
ignored.)

It only applies to a run that relocalized into the persistent frame: in a
fresh-frame run the same coordinates point somewhere else in the flat.

Standard library only at import time (numpy is imported inside the one
function that needs it), so `tools/keepout.py` runs under a bare python3.
"""

from __future__ import annotations

import json
import math
import os
import re
import shutil
import time

STATE_DIR = os.path.expanduser("~/.local/state/vector")
CHECKPOINT_DIR = os.path.join(STATE_DIR, "checkpoints")
MAP_PATH = os.path.join(STATE_DIR, "persistent_map.npz")
KEEPOUT_PATH = os.path.join(STATE_DIR, "keepout.json")
GENERATIONS = 5          # previous maps kept, newest first: a bad session is undoable

FORBIDDEN = "forbidden"            # never enterable: forced occupied, after every layer
ZONE_TYPES = (FORBIDDEN,)
LEGACY_ZONE_TYPES = ("no_slip_reflex",)   # died with the slip detectors (26/08); old files may carry them
ZONE_EDGE_TOL_M = 0.025            # half a 5 cm cell: how far outside a polygon edge still counts
                                   # as inside, so a pose answers the same as the cell it stands on
                                   # (a rectangle already rounds outward to whole cells)


# --- switches -------------------------------------------------------------

def _flag(name: str, default: str) -> bool:
    return os.environ.get(name, default).strip().lower() not in ("0", "false", "no", "off", "")


def enabled() -> bool:
    """PERSISTENT_MAP=1|0 (default 1): the single switch for the whole feature."""
    return _flag("PERSISTENT_MAP", "1")


def rebase_allowed() -> bool:
    """PERSISTENT_MAP_REBASE=1: a fresh-frame run may replace the saved map."""
    return _flag("PERSISTENT_MAP_REBASE", "0")


# --- the persistent map ---------------------------------------------------

def map_exists() -> bool:
    return os.path.isfile(MAP_PATH)


def generations() -> list[str]:
    """Previous persistent maps, newest first."""
    d = os.path.dirname(MAP_PATH)
    if not os.path.isdir(d):
        return []
    base = os.path.basename(MAP_PATH)[: -len(".npz")]
    live = os.path.basename(MAP_PATH)
    out = [os.path.join(d, f) for f in os.listdir(d)
           if f.startswith(base + ".") and f.endswith(".npz") and f != live]
    return sorted(out, reverse=True)


def promote(checkpoint_path: str, keep: int = GENERATIONS) -> str:
    """Make `checkpoint_path` the persistent map, keeping the old ones.

    The current map is moved aside under its own timestamp before the new one
    lands, and the copy goes through a temporary file: a power cut mid-write
    leaves the previous generation intact.
    """
    os.makedirs(os.path.dirname(MAP_PATH), exist_ok=True)
    tmp = MAP_PATH + ".tmp"
    shutil.copyfile(checkpoint_path, tmp)
    if os.path.isfile(MAP_PATH):
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(os.path.getmtime(MAP_PATH)))
        aside = MAP_PATH[: -len(".npz")] + "." + stamp + ".npz"
        if not os.path.exists(aside):
            os.replace(MAP_PATH, aside)
    os.replace(tmp, MAP_PATH)
    for old in generations()[keep:]:
        os.remove(old)
    return MAP_PATH


def newest_checkpoint(run_dir: str | None = None) -> str | None:
    """The freshest costmap checkpoint on disk (of one run, or of any run).

    Used two ways: to promote a finished run, and to give a mid-run
    relocalization the current map -- at most one checkpoint period old, and
    already in the frame the run is writing.
    """
    if run_dir:
        dirs = [run_dir]
    elif os.path.isdir(CHECKPOINT_DIR):
        dirs = [os.path.join(CHECKPOINT_DIR, d) for d in sorted(os.listdir(CHECKPOINT_DIR))]
    else:
        dirs = []
    best, best_ts = None, -1.0
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for f in os.listdir(d):
            if not f.endswith(".npz"):
                continue
            p = os.path.join(d, f)
            ts = os.path.getmtime(p)
            if ts > best_ts:
                best, best_ts = p, ts
    return best


# --- keep-out zones -------------------------------------------------------

def load_keepouts(path: str = KEEPOUT_PATH) -> list[dict]:
    """The declared zones, normalised.

    A rectangle comes back as label, type, x0 <= x1, y0 <= y1, note - byte for
    byte what it was before polygons existed. A polygon comes back as label,
    type, points, note: no bounding box is stored or invented, so nothing can
    go stale against the vertices, and a consumer that wants the extent asks
    `zone_bounds()`.

    Accepts the file's `{"frame": ..., "zones": [...]}` shape and a bare list.
    A zone with no `type` is `forbidden` - the safe reading of an unlabelled
    "do not go there". A missing file is simply no zone; a broken one raises,
    because silently driving into the toilets is worse than a stack trace.
    """
    if not os.path.isfile(path):
        return []
    with open(path) as fh:
        doc = json.load(fh)
    zones = doc.get("zones", []) if isinstance(doc, dict) else doc
    out = []
    for z in zones:
        kind = str(z.get("type", FORBIDDEN))
        if kind in LEGACY_ZONE_TYPES:
            continue                       # a dead concept is dropped, not a crash
        if kind not in ZONE_TYPES:
            raise ValueError(f"keep-out {z.get('label')!r}: unknown type {kind!r}, "
                             f"expected one of {ZONE_TYPES}")
        label = str(z["label"])
        pts = z.get("points")
        if pts is not None:
            out.append({"label": label, "type": kind,
                        "points": _clean_points(pts, label),
                        "note": str(z.get("note", ""))})
            continue
        out.append({"label": label, "type": kind,
                    "x0": min(float(z["x0"]), float(z["x1"])),
                    "y0": min(float(z["y0"]), float(z["y1"])),
                    "x1": max(float(z["x0"]), float(z["x1"])),
                    "y1": max(float(z["y0"]), float(z["y1"])),
                    "note": str(z.get("note", ""))})
    return out


def keepout_frame(path: str = KEEPOUT_PATH) -> str:
    """Which map the zone coordinates were drawn on - free-text, for the log."""
    if not os.path.isfile(path):
        return ""
    with open(path) as fh:
        doc = json.load(fh)
    return str(doc.get("frame", "")) if isinstance(doc, dict) else ""


def save_keepouts(zones: list[dict], path: str = KEEPOUT_PATH, frame: str | None = None,
                  backup: bool = False) -> None:
    """Write the zones back, keeping the `frame` note the owner wrote.

    Atomic, and in that order on purpose: the whole document is serialised
    first (a zone that will not serialise raises here, before anything on disk
    has moved), then the previous file is copied aside as `<path>.bak` if
    `backup`, then the new one lands with a single `os.replace`. A reader never
    sees a half-written zone file, and a refused write leaves the live one
    exactly as it was. The UI saves with `backup=True`: the owner redraws his
    house with the mouse, and the version before the last save stays one `cp`
    away.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc = {"frame": frame if frame is not None else keepout_frame(path), "zones": zones}
    body = _dumps(doc)
    if backup and os.path.isfile(path):
        shutil.copyfile(path, path + ".bak")
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as fh:
            fh.write(body)
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


# `json.dumps(indent=2)` puts every number of every vertex on its own line: a
# 40-corner house drawn with the mouse becomes 120 lines nobody can read. One
# vertex per line instead. The pattern needs the newlines, so it can only ever
# match a real two-number array - a JSON string cannot contain one.
_POINT_RE = re.compile(r"\[\n\s+(-?\d+(?:\.\d+)?),\n\s+(-?\d+(?:\.\d+)?)\n\s+\]")


def _dumps(doc: dict) -> str:
    return _POINT_RE.sub(r"[\1, \2]", json.dumps(doc, indent=2)) + "\n"


def zones_of(zones: list[dict], kind: str) -> list[dict]:
    return [z for z in zones if z.get("type", FORBIDDEN) == kind]


# --- the two zone shapes ---------------------------------------------------

def _clean_points(pts, label: str = "?") -> list[list[float]]:
    """A `points` list into plain floats, or a ValueError saying which zone."""
    try:
        out = [[float(p[0]), float(p[1])] for p in pts]
    except (TypeError, ValueError, IndexError, KeyError) as exc:
        raise ValueError(f"keep-out {label!r}: points must be [[x, y], ...] in metres ({exc})") from exc
    if len(out) < 3:
        raise ValueError(f"keep-out {label!r}: a polygon needs at least 3 points, got {len(out)}")
    for x, y in out:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"keep-out {label!r}: a vertex is not a finite number")
    return out


def zone_points(z: dict) -> list[list[float]] | None:
    """The zone's vertices in metres, or None if it is a rectangle."""
    pts = z.get("points")
    return _clean_points(pts, str(z.get("label", "?"))) if pts is not None else None


def zone_bounds(z: dict) -> tuple[float, float, float, float]:
    """(x0, y0, x1, y1) of the zone's extent, whatever its shape.

    For a polygon this is the bounding box - what to draw a viewport around,
    never what to forbid: the whole point of a polygon is that its box is
    wrong (metrox's house sits 5.75 deg off the map axes).
    """
    pts = zone_points(z)
    if pts is None:
        return (float(z["x0"]), float(z["y0"]), float(z["x1"]), float(z["y1"]))
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), min(ys), max(xs), max(ys))


def polygon_area(points) -> float:
    """Shoelace, in square metres. Sign dropped: winding order is the owner's
    mouse, not a decision."""
    pts = _clean_points(points)
    a = 0.0
    for i, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2.0


def _in_polygon(pts: list[list[float]], x: float, y: float) -> bool:
    """Even-odd ray casting: a horizontal ray from (x, y) to -inf."""
    inside = False
    for i, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(i + 1) % len(pts)]
        if (y1 > y) != (y2 > y) and x > x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _dist_to_outline(pts: list[list[float]], x: float, y: float) -> float:
    """Metres from the point to the nearest edge of the polygon."""
    best = float("inf")
    for i, (x1, y1) in enumerate(pts):
        x2, y2 = pts[(i + 1) % len(pts)]
        dx, dy = x2 - x1, y2 - y1
        d2 = dx * dx + dy * dy
        t = 0.0 if d2 == 0.0 else max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / d2))
        best = min(best, math.hypot(x - (x1 + t * dx), y - (y1 + t * dy)))
    return best


def point_in_zone(z: dict, x: float, y: float, tol: float = ZONE_EDGE_TOL_M) -> bool:
    """Is (x, y) inside this zone? The one containment test, both shapes.

    A rectangle includes its border (`<=`). A polygon includes its border too,
    within `tol`: even-odd counting is exact and therefore asymmetric - a point
    exactly on the left edge is in, the same point on the right edge is out -
    and a guard that answers differently on two sides of the same ramp is a
    guard nobody can reason about. Half a cell also makes the answer agree with
    the map, whose cells round outward.
    """
    pts = zone_points(z)
    if pts is None:
        return float(z["x0"]) <= x <= float(z["x1"]) and float(z["y0"]) <= y <= float(z["y1"])
    return _in_polygon(pts, x, y) or (tol > 0.0 and _dist_to_outline(pts, x, y) <= tol)


def zone_at(zones: list[dict], x: float, y: float, kind: str) -> str | None:
    """The label of the `kind` zone the point falls in, or None."""
    for z in zones_of(zones, kind):
        if point_in_zone(z, x, y):
            return str(z["label"])
    return None


def polygon_mask(points, res: float, ox: float, oy: float, n):
    """Bool mask of the cells a polygon claims. Numpy only.

    `n` is the grid size: an int for the square map grid, or (rows, cols) for a
    published crop, which is not square.

    Two passes, and the second is not decoration:

      * even-odd ray casting on the cell CENTRES, vectorised over the polygon's
        bounding box (one pass per edge, no per-cell python);
      * every cell the OUTLINE crosses, sampled every half cell.

    The outline pass is what makes a polygon behave like a rectangle: the
    rectangle mask rounds outward (floor of both corners, both inclusive), so a
    1.00 m rectangle claims 1.05 m of cells and a zone thinner than one cell
    still claims one. Without it a hand-drawn 3 cm doorway strip could land
    between two cell centres and forbid nothing at all.
    """
    import numpy as np

    pts = _clean_points(points)
    rows, cols = (n, n) if isinstance(n, int) else (int(n[0]), int(n[1]))
    mask = np.zeros((rows, cols), dtype=bool)
    xs_p = [p[0] for p in pts]
    ys_p = [p[1] for p in pts]
    gx0 = max(0, int((min(xs_p) - ox) // res))
    gx1 = min(cols - 1, int((max(xs_p) - ox) // res))
    gy0 = max(0, int((min(ys_p) - oy) // res))
    gy1 = min(rows - 1, int((max(ys_p) - oy) // res))
    if gx1 < gx0 or gy1 < gy0:
        return mask                      # the whole polygon falls off this grid
    xs = ox + (np.arange(gx0, gx1 + 1) + 0.5) * res
    ys = oy + (np.arange(gy0, gy1 + 1) + 0.5) * res
    gridx, gridy = np.meshgrid(xs, ys)
    sub = np.zeros(gridx.shape, dtype=bool)
    for i, (x1, y1) in enumerate(pts):   # pass 1: the inside, by even-odd counting
        x2, y2 = pts[(i + 1) % len(pts)]
        if y1 == y2:                     # a horizontal edge crosses no horizontal ray
            continue
        crosses = (y1 > gridy) != (y2 > gridy)
        xint = x1 + (gridy - y1) * (x2 - x1) / (y2 - y1)
        sub ^= crosses & (gridx > xint)
    for i, (x1, y1) in enumerate(pts):   # pass 2: the outline (never XORed - it is an OR)
        x2, y2 = pts[(i + 1) % len(pts)]
        steps = int(math.hypot(x2 - x1, y2 - y1) / (res / 2)) + 2
        t = np.linspace(0.0, 1.0, steps)
        ex = ((x1 + (x2 - x1) * t - ox) // res).astype(np.int64)
        ey = ((y1 + (y2 - y1) * t - oy) // res).astype(np.int64)
        ok = (ex >= gx0) & (ex <= gx1) & (ey >= gy0) & (ey <= gy1)
        sub[ey[ok] - gy0, ex[ok] - gx0] = True
    mask[gy0:gy1 + 1, gx0:gx1 + 1] = sub
    return mask


def keepout_mask(zones: list[dict], res: float, ox: float, oy: float, n: int,
                 kind: str = FORBIDDEN):
    """Bool mask of the cells covered by the `kind` zones, on an n x n grid.

    A zone is never allowed to be empty: a rectangle thinner than one cell
    still claims the cell it falls in, so a badly typed 2 cm zone protects
    something rather than nothing. A polygon claims its cell centres plus its
    outline, for the same reason (`polygon_mask`).
    """
    import numpy as np

    mask = np.zeros((n, n), dtype=bool)
    for z in zones_of(zones, kind):
        pts = zone_points(z)
        if pts is not None:
            mask |= polygon_mask(pts, res, ox, oy, n)
            continue
        x0 = int((z["x0"] - ox) // res)
        x1 = int((z["x1"] - ox) // res)
        y0 = int((z["y0"] - oy) // res)
        y1 = int((z["y1"] - oy) // res)
        x0, x1 = max(0, min(x0, n - 1)), max(0, min(x1, n - 1))
        y0, y1 = max(0, min(y0, n - 1)), max(0, min(y1, n - 1))
        mask[y0:y1 + 1, x0:x1 + 1] = True
    return mask


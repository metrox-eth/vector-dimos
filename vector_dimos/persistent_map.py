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

Zones (keepout.json, written by `tools/keepout.py` or by the owner's hand):

    {"frame": "<which map these coordinates belong to>",
     "zones": [{"label": ..., "type": ..., "x0":, "y0":, "x1":, "y1":, "note": ...}]}

Two types, because "the rover must not go there" and "the rover's reflexes are
wrong there" are different problems:

  forbidden       the cells become occupied in `occupancy()`, after every
                  layer: nothing erases them and the planner never enters.
                  (The toilets: a 3 cm step at the door.)
  no_slip_reflex  the place IS allowed, but while the rover stands in it the
                  anti-slip reflexes stay silent. Slipping on a ramp is normal
                  and transient; on 26/08 the reflex cut the torque mid-climb
                  and the rover slid back down "like ice" (IMU SLIP #9/#10).

Both only apply to a run that relocalized into the persistent frame: in a
fresh-frame run the same coordinates point somewhere else in the flat.

Standard library only at import time (numpy is imported inside the one
function that needs it), so `tools/keepout.py` runs under a bare python3.
"""

from __future__ import annotations

import json
import os
import shutil
import time

STATE_DIR = os.path.expanduser("~/.local/state/vector")
CHECKPOINT_DIR = os.path.join(STATE_DIR, "checkpoints")
MAP_PATH = os.path.join(STATE_DIR, "persistent_map.npz")
KEEPOUT_PATH = os.path.join(STATE_DIR, "keepout.json")
GENERATIONS = 5          # previous maps kept, newest first: a bad session is undoable

FORBIDDEN = "forbidden"            # never enterable: forced occupied, after every layer
NO_SLIP_REFLEX = "no_slip_reflex"  # allowed, but the anti-slip reflexes stay quiet inside
ZONE_TYPES = (FORBIDDEN, NO_SLIP_REFLEX)
ZONE_RELOAD_S = 30.0               # how often a live module re-reads the file


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
    """The declared zones, normalised: label, type, x0 <= x1, y0 <= y1, note.

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
        if kind not in ZONE_TYPES:
            raise ValueError(f"keep-out {z.get('label')!r}: unknown type {kind!r}, "
                             f"expected one of {ZONE_TYPES}")
        out.append({"label": str(z["label"]), "type": kind,
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


def save_keepouts(zones: list[dict], path: str = KEEPOUT_PATH, frame: str | None = None) -> None:
    """Write the zones back, keeping the `frame` note the owner wrote."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    doc = {"frame": frame if frame is not None else keepout_frame(path), "zones": zones}
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        json.dump(doc, fh, indent=2)
        fh.write("\n")
    os.replace(tmp, path)


def zones_of(zones: list[dict], kind: str) -> list[dict]:
    return [z for z in zones if z.get("type", FORBIDDEN) == kind]


def zone_at(zones: list[dict], x: float, y: float, kind: str) -> str | None:
    """The label of the `kind` zone the point falls in, or None."""
    for z in zones_of(zones, kind):
        if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]:
            return str(z["label"])
    return None


def keepout_mask(zones: list[dict], res: float, ox: float, oy: float, n: int,
                 kind: str = FORBIDDEN):
    """Bool mask of the cells covered by the `kind` zones, on an n x n grid.

    A zone is never allowed to be empty: a rectangle thinner than one cell
    still claims the cell it falls in, so a badly typed 2 cm zone protects
    something rather than nothing.
    """
    import numpy as np

    mask = np.zeros((n, n), dtype=bool)
    for z in zones_of(zones, kind):
        x0 = int((z["x0"] - ox) // res)
        x1 = int((z["x1"] - ox) // res)
        y0 = int((z["y0"] - oy) // res)
        y1 = int((z["y1"] - oy) // res)
        x0, x1 = max(0, min(x0, n - 1)), max(0, min(x1, n - 1))
        y0, y1 = max(0, min(y0, n - 1)), max(0, min(y1, n - 1))
        mask[y0:y1 + 1, x0:x1 + 1] = True
    return mask


class ZoneWatch:
    """Is the rover standing in a zone of this type, right now?

    Shared by the two anti-slip guards, which have no map and no checkpoint
    tick of their own. It re-reads the file every ZONE_RELOAD_S, so a zone
    declared with `tools/keepout.py` takes effect without restarting a stack
    that must not be restarted.

    It answers None until the run has relocalized into the persistent frame:
    in a fresh-frame run these coordinates point somewhere else in the flat,
    and a reflex silenced over the wrong square metre is a rover falling down
    the ramp.
    """

    def __init__(self, kind: str, path: str = KEEPOUT_PATH) -> None:
        self.kind = kind
        self.path = path
        self.persistent = False
        self._zones: list[dict] = []
        self._next_read = 0.0

    def note_frame(self, frame_id: str) -> bool:
        """Feed it lidar_odometry's `reloc_frame` messages. True on a change."""
        was = self.persistent
        state = str(frame_id or "").removeprefix("reloc:")
        if state in ("persistent", "fresh"):
            self.persistent = state == "persistent"
        return self.persistent != was

    def zones(self) -> list[dict]:
        now = time.monotonic()
        if now >= self._next_read:
            self._next_read = now + ZONE_RELOAD_S
            try:
                self._zones = zones_of(load_keepouts(self.path), self.kind)
            except Exception:  # noqa: BLE001 - a broken file must not kill a reflex
                pass
        return self._zones

    def at(self, x: float, y: float) -> str | None:
        if not self.persistent:
            return None
        for z in self.zones():
            if z["x0"] <= x <= z["x1"] and z["y0"] <= y <= z["y1"]:
                return str(z["label"])
        return None

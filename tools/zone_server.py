#!/usr/bin/env python3
"""Draw the rover's keep-out zones with the mouse, on the map it actually uses.

    http://<rover>:8902

Before this, a zone was four numbers read off the Rerun map by hovering it, and
every zone was an axis-aligned rectangle. The test flat sits 5.75 deg off the
map axes, so the fences drawn that way either ate a corridor or leaked a corner,
and each attempt cost a round of type-and-look. Requirement (2026-08-26): a UI
where the limits can be drawn directly.

So: the persistent map is rendered as a PNG, the browser draws it on a canvas,
and a click puts down a polygon vertex in world metres. What comes out is the
same `keepout.json` `tools/keepout.py` writes - polygons instead of rectangles,
read by exactly the same code (`persistent_map.keepout_mask`, the two slip
guards, the Rerun overlay).

What it touches: `~/.local/state/vector/persistent_map.npz` (read only) and
`~/.local/state/vector/keepout.json` (read, and written atomically with the
previous version kept as `keepout.json.bak`). It opens no serial port, talks to
no motor, and knows nothing about the dimOS stack - which picks a saved edit up
within about half a minute, without a restart.

Endpoints:
    GET  /           the page (tools/zone_ui.html)
    GET  /map.png    the persistent map: grey floor, red obstacles, dark unknown
    GET  /map_meta   res, ox, oy, n and the pixel <-> metre formulas
    GET  /zones      the current keepout.json
    POST /zones      validate, back up, write atomically

Standard library plus numpy (to read the .npz): no new dependency on the
Jetson, no build step, no CDN.

Run:  .venv/bin/python tools/zone_server.py
      (installed as vector-zones.service - see tools/vector-zones.service)
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from vector_dimos import persistent_map  # noqa: E402

PORT = 8902
UI_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "zone_ui.html")

# costmap2d's OCCUPIED_AT and FREE_FLOOR, on purpose not imported: costmap2d
# pulls the whole dimOS runtime in, and this server must start even when the
# stack cannot. Kept in sync by tests/test_zones_cold.py, which imports both.
OCCUPIED_AT = 2
FREE_FLOOR = -3

COLOR_UNKNOWN = (34, 34, 38)      # never observed
COLOR_FLOOR = (168, 168, 164)     # seen, but not cleared over and over
COLOR_CLEAR = (222, 222, 218)     # seen and cleared down to the floor: certainly drivable
COLOR_OCCUPIED = (210, 45, 45)    # an obstacle the rover believes in

MAX_BODY = 2_000_000              # a POST bigger than this is not a hand-drawn flat
MAX_ZONES = 200
MAX_POINTS = 2000
MAX_LABEL = 60
MAX_NOTE = 300
OUT_OF_MAP_M = 20.0               # how far past the map's edge a vertex may still land

_lock = threading.Lock()
_png_cache: dict = {"mtime": None, "png": b"", "meta": {}}


# --- the map as a picture ---------------------------------------------------

def _png_bytes(rgb: np.ndarray) -> bytes:
    """An (h, w, 3) uint8 array as a PNG. zlib and struct, nothing else."""
    h, w, _ = rgb.shape
    rows = np.zeros((h, w * 3 + 1), dtype=np.uint8)
    rows[:, 1:] = rgb.reshape(h, w * 3)          # filter byte 0 = None, per row
    raw = zlib.compress(rows.tobytes(), 6)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", raw)
            + chunk(b"IEND", b""))


def render_map(path: str = None) -> tuple[bytes, dict]:
    """The persistent map as (PNG bytes, metadata).

    Same reading as `ScoredGrid.occupancy()`: score = max(lidar, low), occupied
    at OCCUPIED_AT, free where `seen`, unknown everywhere else. The image is
    flipped vertically, so row 0 is the TOP of the map (the highest y) - the
    formulas in the metadata are the contract the page converts with, and the
    cold bench checks them both ways.
    """
    path = path or persistent_map.MAP_PATH
    z = np.load(path)
    lidar, low, seen = z["lidar"], z["low"], z["seen"]
    res, n = float(z["res"]), int(z["n"])
    ox, oy = float(z["ox"]), float(z["oy"])
    score = np.maximum(lidar, low)

    rgb = np.empty(score.shape + (3,), dtype=np.uint8)
    rgb[:] = COLOR_UNKNOWN
    rgb[seen] = COLOR_FLOOR
    rgb[seen & (score <= FREE_FLOOR)] = COLOR_CLEAR
    rgb[score >= OCCUPIED_AT] = COLOR_OCCUPIED

    meta = {
        "res": res, "ox": ox, "oy": oy, "n": n,
        "width": int(score.shape[1]), "height": int(score.shape[0]),
        "x_min": ox, "x_max": ox + score.shape[1] * res,
        "y_min": oy, "y_max": oy + score.shape[0] * res,
        "seen_cells": int(seen.sum()),
        "occupied_cells": int((score >= OCCUPIED_AT).sum()),
        "map_path": path,
        "map_mtime": os.path.getmtime(path),
        "saved": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(os.path.getmtime(path))),
        # px -> metres and back. Row 0 of the PNG is the highest y.
        "x_of_px": "x = ox + (px + 0.5) * res",
        "y_of_py": "y = oy + (height - py - 0.5) * res",
        "px_of_x": "px = (x - ox) / res",
        "py_of_y": "py = height - (y - oy) / res",
    }
    return _png_bytes(np.flipud(rgb)), meta


def map_png() -> tuple[bytes, dict]:
    """The rendered map, re-rendered only when the file on disk changes."""
    with _lock:
        if not os.path.isfile(persistent_map.MAP_PATH):
            raise FileNotFoundError(persistent_map.MAP_PATH)
        mtime = os.path.getmtime(persistent_map.MAP_PATH)
        if _png_cache["mtime"] != mtime:
            png, meta = render_map()
            _png_cache.update(mtime=mtime, png=png, meta=meta)
        return _png_cache["png"], _png_cache["meta"]


# --- what the page is allowed to save ---------------------------------------

TYPE_LABELS = {persistent_map.FORBIDDEN: "forbidden"}


class Refused(ValueError):
    """A zone the server will not write, with the reason in plain words."""


def _finite(v, what: str) -> float:
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise Refused(f"{what}: not a number.") from None
    if not math.isfinite(f):
        raise Refused(f"{what}: not a finite number.")
    return f


def validate_zones(doc, bounds: tuple[float, float, float, float] | None = None) -> list[dict]:
    """The posted document into zones ready to save, or a Refused.

    Deliberately strict: this file makes cells lethal and silences the slip
    reflexes, and the only thing standing between it and a typo is right here.
    """
    zones = doc.get("zones") if isinstance(doc, dict) else doc
    if not isinstance(zones, list):
        raise Refused("The message carries no zone list.")
    if len(zones) > MAX_ZONES:
        raise Refused(f"Too many zones ({len(zones)}, maximum {MAX_ZONES}).")
    out: list[dict] = []
    seen_labels: set[str] = set()
    for raw in zones:
        if not isinstance(raw, dict):
            raise Refused("A zone is not an object.")
        label = str(raw.get("label", "")).strip()
        if not label:
            raise Refused("A zone has no name.")
        if len(label) > MAX_LABEL:
            raise Refused(f"Name {label[:20]!r}... is too long (maximum {MAX_LABEL} characters).")
        if label in seen_labels:
            raise Refused(f"Two zones share the name {label!r}.")
        seen_labels.add(label)
        kind = str(raw.get("type", persistent_map.FORBIDDEN))
        if kind not in persistent_map.ZONE_TYPES:
            raise Refused(f"Zone {label!r}: unknown type {kind!r} "
                          f"(expected: {' or '.join(persistent_map.ZONE_TYPES)}).")
        note = str(raw.get("note", ""))[:MAX_NOTE]
        pts_raw = raw.get("points")
        if pts_raw is not None:
            if not isinstance(pts_raw, list):
                raise Refused(f"Zone {label!r}: 'points' must be a list [[x, y], ...].")
            if len(pts_raw) < 3:
                raise Refused(f"Zone {label!r}: a polygon needs at least 3 points "
                              f"(it has {len(pts_raw)}).")
            if len(pts_raw) > MAX_POINTS:
                raise Refused(f"Zone {label!r}: too many points ({len(pts_raw)}).")
            pts = []
            for i, p in enumerate(pts_raw):
                if not isinstance(p, (list, tuple)) or len(p) != 2:
                    raise Refused(f"Zone {label!r} : le point {i + 1} n'est pas [x, y].")
                pts.append([_finite(p[0], f"Zone {label!r}, point {i + 1}, x"),
                            _finite(p[1], f"Zone {label!r}, point {i + 1}, y")])
            zone = {"label": label, "type": kind, "points": pts, "note": note}
        else:
            xs = [_finite(raw.get(k), f"Zone {label!r}, {k}") for k in ("x0", "y0", "x1", "y1")]
            zone = {"label": label, "type": kind,
                    "x0": min(xs[0], xs[2]), "y0": min(xs[1], xs[3]),
                    "x1": max(xs[0], xs[2]), "y1": max(xs[1], xs[3]), "note": note}
        if bounds is not None:
            x0, y0, x1, y1 = persistent_map.zone_bounds(zone)
            bx0, by0, bx1, by1 = bounds
            if (x0 < bx0 - OUT_OF_MAP_M or x1 > bx1 + OUT_OF_MAP_M
                    or y0 < by0 - OUT_OF_MAP_M or y1 > by1 + OUT_OF_MAP_M):
                raise Refused(f"Zone {label!r}: it lies far outside the map "
                              f"(x {x0:+.1f} .. {x1:+.1f}, y {y0:+.1f} .. {y1:+.1f} m).")
        out.append(zone)
    return out


def zones_doc() -> dict:
    """What GET /zones answers: the file, normalised, plus where it lives."""
    path = persistent_map.KEEPOUT_PATH
    return {
        "path": path,
        "frame": persistent_map.keepout_frame(),
        "zones": persistent_map.load_keepouts(),
        "mtime": os.path.getmtime(path) if os.path.isfile(path) else None,
        "types": list(persistent_map.ZONE_TYPES),
        "type_labels": TYPE_LABELS,
    }


def save_zones(zones: list[dict]) -> dict:
    """Back the file up, write it atomically, and read it back to answer."""
    persistent_map.save_keepouts(zones, backup=True)
    doc = zones_doc()
    doc["backup"] = persistent_map.KEEPOUT_PATH + ".bak"
    return doc


# --- HTTP -------------------------------------------------------------------

class Handler(BaseHTTPRequestHandler):
    server_version = "vector-zones/1.0"
    protocol_version = "HTTP/1.1"

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code: int, payload: dict) -> None:
        self._send(code, json.dumps(payload).encode("utf-8"), "application/json; charset=utf-8")

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        try:
            if path in ("/", "/index.html"):
                with open(UI_PATH, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/map.png":
                png, _ = map_png()
                self._send(200, png, "image/png")
            elif path == "/map_meta":
                _, meta = map_png()
                self._json(200, meta)
            elif path == "/zones":
                self._json(200, zones_doc())
            else:
                self._json(404, {"error": f"nothing at this address: {path}"})
        except FileNotFoundError as exc:
            self._json(503, {"error": f"map not found: {exc}. The rover has not saved "
                                      "a persistent map yet."})
        except Exception as exc:  # noqa: BLE001 - a browser reload must never kill the server
            self.log_error("GET %s failed: %r", path, exc)
            self._json(500, {"error": f"{type(exc).__name__}: {exc}"})

    do_HEAD = do_GET

    def do_POST(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path != "/zones":
            self._json(404, {"error": f"nothing at this address: {path}"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > MAX_BODY:
                raise Refused(f"Empty or oversized message ({length} bytes).")
            doc = json.loads(self.rfile.read(length).decode("utf-8"))
            bounds = None
            try:
                _, meta = map_png()
                bounds = (meta["x_min"], meta["y_min"], meta["x_max"], meta["y_max"])
            except Exception:  # noqa: BLE001 - no map yet is not a reason to refuse a zone
                pass
            zones = validate_zones(doc, bounds)
            answer = save_zones(zones)
            print(f"[{time.strftime('%H:%M:%S')}] {len(zones)} zone(s) written to "
                  f"{persistent_map.KEEPOUT_PATH}: "
                  + ", ".join(f"{z['label']}({len(z['points'])} pts)" if "points" in z
                              else f"{z['label']}(rect)" for z in zones), flush=True)
            self._json(200, answer)
        except Refused as exc:
            self._json(400, {"error": str(exc)})
        except json.JSONDecodeError as exc:
            self._json(400, {"error": f"JSON illisible : {exc}"})
        except Exception as exc:  # noqa: BLE001
            self.log_error("POST %s failed: %r", path, exc)
            self._json(500, {"error": f"{type(exc).__name__}: {exc}"})

    def log_message(self, *args) -> None:
        pass                                  # quiet: journalctl carries the writes only


def main() -> int:
    if not os.path.isfile(UI_PATH):
        print(f"missing page: {UI_PATH}", file=sys.stderr)
        return 1
    print(f"VECTOR zone UI on 0.0.0.0:{PORT}  (map {persistent_map.MAP_PATH}, "
          f"zones {persistent_map.KEEPOUT_PATH})", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

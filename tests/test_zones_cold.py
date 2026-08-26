"""Cold bench for polygon keep-out zones: the shape the owner draws with a mouse.

Rule #2: a known input gives a known output, in physical units. Groups:

  A. rasterizer   - a rectangle written as 4 points claims EXACTLY the cells the
                    legacy rectangle claims; an L claims exactly the union of the
                    two rectangles it is made of, and its notch stays free; a
                    tilted house does not forbid the corners of its bounding box
  B. containment  - the pose test on a tilted ramp: inside, 20 cm outside, and
                    the point that a bounding-box zone would have caught wrongly
  C. the map      - a polygon zone is 100 after every layer and survives 30 lidar
                    rays, a camera floor sample and body_clear, exactly like a
                    rectangle (needs dimOS: run this on the Jetson)
  D. the file     - legacy rectangles come back byte for byte, polygons come back
                    identically, and a save is atomic with the previous file kept
  E. the server   - the map PNG decoded back to known pixels at known metres, and
                    what the UI is not allowed to save

A, B, D and E need numpy only. C needs dimOS.

Run:  .venv/bin/python3 tests/test_zones_cold.py
"""

import json
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from vector_dimos import persistent_map  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
import zone_server  # noqa: E402

try:
    from vector_dimos.costmap2d import FREE_FLOOR, OCCUPIED_AT, ScoredGrid
    HAVE_DIMOS = True
except Exception:  # noqa: BLE001 - no dimOS on a laptop
    HAVE_DIMOS = False

FORBIDDEN = persistent_map.FORBIDDEN
NO_SLIP = "no_slip_reflex"   # the LEGACY type: died with the slip detectors 26/08, old files may carry it
RES = 0.05
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def rect(label, x0, y0, x1, y1, kind=FORBIDDEN):
    return {"label": label, "type": kind, "x0": x0, "y0": y0, "x1": x1, "y1": y1, "note": ""}


def poly(label, points, kind=FORBIDDEN, note=""):
    return {"label": label, "type": kind, "points": [list(p) for p in points], "note": note}


def tilted(cx, cy, half_w, half_h, deg):
    """A rectangle turned by `deg` about (cx, cy) - metrox's house is 5.75 deg
    off the map axes, which is the whole reason polygons exist."""
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return [[cx + c * x - s * y, cy + s * x + c * y]
            for x, y in ((-half_w, -half_h), (half_w, -half_h), (half_w, half_h), (-half_w, half_h))]


def perimeter(points):
    return sum(math.hypot(points[(i + 1) % len(points)][0] - p[0],
                          points[(i + 1) % len(points)][1] - p[1])
               for i, p in enumerate(points))


# --- A. the rasterizer ------------------------------------------------------
#
# Coordinates sit on half-cells (x.x25 / x.x75) on purpose: nothing lands on a
# cell boundary, so "which cell is this metre in" has one answer and the
# comparisons below are exact, not approximate.

print("A. rasterizer: cells claimed, in metres (res 5 cm, grid 4 x 4 m at the origin)")
N = 80
OX = OY = 0.0

r = rect("r", 1.025, 0.025, 2.025, 1.025)
p = poly("p", [[1.025, 0.025], [2.025, 0.025], [2.025, 1.025], [1.025, 1.025]])
m_rect = persistent_map.keepout_mask([r], RES, OX, OY, N)
m_poly = persistent_map.keepout_mask([p], RES, OX, OY, N)
check("a 1.00 x 1.00 m rectangle -> 21 x 21 cells", int(m_rect.sum()) == 441, f"{int(m_rect.sum())}")
check("the same rectangle written as 4 points -> the SAME cells, exactly",
      np.array_equal(m_rect, m_poly), f"{int(m_poly.sum())} cells")

A = rect("a", 0.025, 0.025, 2.025, 1.025)
B = rect("b", 0.025, 1.025, 1.025, 2.025)
L = poly("l", [[0.025, 0.025], [2.025, 0.025], [2.025, 1.025],
               [1.025, 1.025], [1.025, 2.025], [0.025, 2.025]])
m_union = (persistent_map.keepout_mask([A], RES, OX, OY, N)
           | persistent_map.keepout_mask([B], RES, OX, OY, N))
m_L = persistent_map.keepout_mask([L], RES, OX, OY, N)
check("an L-shape -> exactly the union of the two rectangles it is made of",
      np.array_equal(m_union, m_L), f"{int(m_L.sum())} cells")
notch = (int((1.5 - OY) / RES), int((1.5 - OX) / RES))
check("the notch of the L (1.50, 1.50) is NOT forbidden", not bool(m_L[notch]))
check("the leg of the L (0.50, 1.50) IS forbidden", bool(m_L[int(1.5 / RES), int(0.5 / RES)]))
check("point_in_zone agrees with the cells, in the notch and in the leg",
      not persistent_map.point_in_zone(L, 1.5, 1.5) and persistent_map.point_in_zone(L, 0.5, 1.5))

T = poly("t", [[0.025, 0.025], [2.025, 0.025], [0.025, 2.025]])
tri = persistent_map.keepout_mask([T], RES, OX, OY, N)
area_true = persistent_map.polygon_area(T["points"])
area_cells = int(tri.sum()) * RES * RES
check("a triangle of 2.00 m2 -> 2.00 m2 of cells, rounded outward",
      area_true == 2.0 and area_true <= area_cells <= area_true + perimeter(T["points"]) * RES,
      f"{area_cells:.3f} m2 of cells for {area_true:.2f} m2 of triangle")

H = poly("maison", tilted(1.0, 1.0, 0.8, 0.4, 5.75))
house = persistent_map.keepout_mask([H], RES, OX, OY, N)
hx0, hy0, hx1, hy1 = persistent_map.zone_bounds(H)
area_true = persistent_map.polygon_area(H["points"])
area_cells = int(house.sum()) * RES * RES
check("a house tilted 5.75 deg -> its own area of cells, not its bounding box",
      area_true <= area_cells <= area_true + perimeter(H["points"]) * RES,
      f"{area_cells:.3f} m2 of cells, {area_true:.2f} m2 of house, "
      f"{(hx1 - hx0) * (hy1 - hy0):.2f} m2 of bounding box")
corner = (hx0 + 0.02, hy0 + 0.02)
check("a corner of the bounding box is INSIDE the box and OUTSIDE the zone",
      hx0 <= corner[0] <= hx1 and hy0 <= corner[1] <= hy1
      and not persistent_map.point_in_zone(H, *corner)
      and not bool(house[int(corner[1] / RES), int(corner[0] / RES)]),
      f"({corner[0]:+.3f}, {corner[1]:+.3f}), "
      f"{persistent_map._dist_to_outline(H['points'], *corner) * 100:.0f} cm off the wall")
check("the middle of the tilted house is forbidden",
      persistent_map.point_in_zone(H, 1.0, 1.0) and bool(house[int(1.0 / RES), int(1.0 / RES)]))

thin = poly("seuil", [[1.0, 1.0], [1.03, 1.0], [1.03, 2.0], [1.0, 2.0]])   # a 3 cm doorway strip
m_thin = persistent_map.keepout_mask([thin], RES, OX, OY, N)
check("a strip 3 cm wide (thinner than a cell) still forbids a full metre of doorway",
      int(m_thin.sum()) >= 20, f"{int(m_thin.sum())} cells")
away = poly("ailleurs", [[50.0, 50.0], [51.0, 50.0], [51.0, 51.0]])
check("a zone entirely off this grid -> no cell, no crash",
      int(persistent_map.keepout_mask([away], RES, OX, OY, N).sum()) == 0)
check("a legacy no_slip_reflex polygon is not a keep-out mask at all",
      int(persistent_map.keepout_mask([poly("q", H["points"], NO_SLIP)],
                                      RES, OX, OY, N).sum()) == 0)
try:
    persistent_map.keepout_mask([poly("deux", [[0, 0], [1, 1]])], RES, OX, OY, N)
    check("a 2-point polygon raises rather than forbidding nothing", False)
except ValueError as exc:
    check("a 2-point polygon raises rather than forbidding nothing", True, str(exc))

# --- B. containment on a tilted ramp ---------------------------------------

print("B. containment: the ramp, 1.05 x 0.45 m, turned 5.75 deg, centred (-2.50, -8.00)")
RAMP = poly("rampe-cuisine-atelier", tilted(-2.5, -8.0, 0.525, 0.225, 5.75), FORBIDDEN, "26/08")
rx0, ry0, rx1, ry1 = persistent_map.zone_bounds(RAMP)
check("its middle is on the ramp", persistent_map.point_in_zone(RAMP, -2.5, -8.0))
check("20 cm past its long edge is not", not persistent_map.point_in_zone(RAMP, -2.5, -8.0 - 0.225 - 0.20))
trap = (rx0 + 0.03, ry1 - 0.03)     # inside the bounding box, outside the tilted ramp
check("the trap a rectangle zone falls into: in the box, off the ramp",
      rx0 <= trap[0] <= rx1 and ry0 <= trap[1] <= ry1
      and not persistent_map.point_in_zone(RAMP, *trap),
      f"({trap[0]:+.3f}, {trap[1]:+.3f})")
edge = RAMP["points"][0]
check("a pose ON the edge counts as inside (half a cell of tolerance)",
      persistent_map.point_in_zone(RAMP, edge[0], edge[1]))
check("2 cm outside the edge still counts, 20 cm does not",
      persistent_map.point_in_zone(RAMP, -2.5, -8.0 - 0.225 - 0.02)
      and not persistent_map.point_in_zone(RAMP, -2.5, -8.0 - 0.225 - 0.20))
check("zone_at reads the label off a polygon",
      persistent_map.zone_at([RAMP], -2.5, -8.0, FORBIDDEN) == RAMP["label"])
check("zone_at ignores it for another kind",
      persistent_map.zone_at([RAMP], -2.5, -8.0, NO_SLIP) is None)

d = tempfile.mkdtemp()
ZONE_FILE = os.path.join(d, "keepout.json")
TOILETS = poly("toilettes", tilted(1.6, -8.3, 1.05, 1.65, 5.75), FORBIDDEN, "3 cm step")
persistent_map.save_keepouts([TOILETS, RAMP], ZONE_FILE, frame="cold bench")
try:
    import json as _json
    doc = _json.load(open(ZONE_FILE))
    doc["zones"].append({"label": "vieille-rampe", "type": NO_SLIP,
                         "x0": -3.0, "y0": -8.2, "x1": -1.95, "y1": -7.75})
    with open(ZONE_FILE, "w") as fh:
        _json.dump(doc, fh)
    back_legacy = persistent_map.load_keepouts(ZONE_FILE)
    check("a legacy no_slip_reflex zone in the file is DROPPED on load, not a crash",
          [z["label"] for z in back_legacy] == ["toilettes", "rampe-cuisine-atelier"],
          f"{[z['label'] for z in back_legacy]}")

    # --- C. the map obeys a polygon --------------------------------------------

    print("C. the costmap: a polygon zone is the last word (needs dimOS)")
    if not HAVE_DIMOS:
        print("  -- skipped, no dimOS here: run this on the Jetson")
    else:
        SQUARE = poly("carre-penche", tilted(1.5, 0.0, 0.5, 0.5, 5.75))
        g = ScoredGrid(span_m=6.0)
        cells = g.set_keepouts([SQUARE])
        true_cells = persistent_map.polygon_area(SQUARE["points"]) / (RES * RES)
        check("a 1.00 x 1.00 m tilted square -> about 400 cells, never fewer",
              true_cells <= cells <= true_cells + perimeter(SQUARE["points"]) / RES,
              f"{cells} cells for {true_cells:.0f} cells of square")
        check("inside the polygon -> occupied", g.value_at(1.5, 0.0) == 100)
        bx0, by0, bx1, by1 = persistent_map.zone_bounds(SQUARE)
        check("a corner of its bounding box -> still free (a rectangle would have taken it)",
              g.value_at(bx0 + 0.03, by0 + 0.03) != 100)
        check("outside -> untouched (unknown)", g.value_at(0.0, 0.0) == -1)
        for _ in range(30):
            g.lidar_revolution(np.array([[2.6, 0.0]]), (0.0, 0.0))   # rays straight through it
        check("30 lidar rays through it -> still occupied", g.value_at(1.5, 0.0) == 100)
        g.camera_floor(np.array([[1.5, 0.0]]))
        check("the camera seeing bare floor there -> still occupied", g.value_at(1.5, 0.0) == 100)
        g.body_clear((1.5, 0.0, 0.0))
        check("body_clear ON the zone -> still occupied", g.value_at(1.5, 0.0) == 100)
        gx, gy = g.cell(np.array([1.5]), np.array([0.0]))
        check("the layers underneath really were cleared (nothing hidden)",
              g.lidar[gy[0], gx[0]] < OCCUPIED_AT, f"score {g.lidar[gy[0], gx[0]]}")
        check("dropping the zones frees the cell again",
              g.set_keepouts([]) == 0 and g.value_at(1.5, 0.0) != 100)
        check("a stale legacy no_slip_reflex polygon leaves the map alone",
              g.set_keepouts([poly("q", SQUARE["points"], NO_SLIP)]) == 0)
        check("a rectangle and a polygon mix in one file",
              g.set_keepouts([SQUARE, rect("r", -2.0, -2.0, -1.0, -1.0)]) > cells)

finally:
    shutil.rmtree(d, ignore_errors=True)

# --- D. the file ------------------------------------------------------------

print("D. keepout.json: both shapes, and a save that cannot half-happen")
d = tempfile.mkdtemp()
try:
    kp = os.path.join(d, "keepout.json")
    legacy = {"label": "rampe", "type": FORBIDDEN, "x0": 1.0, "y0": -0.5, "x1": 2.0, "y1": 0.5,
              "note": "ramp"}
    persistent_map.save_keepouts([dict(legacy)], kp, frame="carte_saine")
    check("a legacy rectangle comes back exactly as before polygons existed",
          persistent_map.load_keepouts(kp) == [legacy], f"{persistent_map.load_keepouts(kp)}")
    check("and it carries no 'points' key",
          "points" not in persistent_map.load_keepouts(kp)[0])

    drawn = poly("maison", tilted(1.0, 1.0, 0.8, 0.4, 5.75), FORBIDDEN, "dessinee a la souris")
    persistent_map.save_keepouts([dict(legacy), drawn], kp, frame="carte_saine")
    back = persistent_map.load_keepouts(kp)
    check("a rectangle and a polygon live in the same file", len(back) == 2)
    check("the polygon comes back vertex for vertex", back[1] == drawn, f"{back[1]}")
    check("a second round trip changes nothing",
          json.loads(open(kp).read())["zones"][1]["points"] == drawn["points"])
    check("the file stays readable by hand: one vertex per line",
          sum(1 for line in open(kp) if line.strip().startswith("[")) == len(drawn["points"]),
          [line.strip() for line in open(kp) if line.strip().startswith("[")][0])
    check("the frame note survives", persistent_map.keepout_frame(kp) == "carte_saine")

    both = {"label": "les-deux", "type": FORBIDDEN, "points": [[0, 0], [1, 0], [1, 1]],
            "x0": -9, "y0": -9, "x1": 9, "y1": 9}
    with open(kp, "w") as fh:
        json.dump({"zones": [both]}, fh)
    z = persistent_map.load_keepouts(kp)[0]
    check("a zone carrying both shapes is read as the polygon, not the box",
          "x0" not in z and len(z["points"]) == 3)

    with open(kp, "w") as fh:
        json.dump({"zones": [{"label": "trop-court", "points": [[0, 0], [1, 1]]}]}, fh)
    try:
        persistent_map.load_keepouts(kp)
        check("a 2-point polygon in the file raises rather than being ignored", False)
    except ValueError as exc:
        check("a 2-point polygon in the file raises rather than being ignored", True, str(exc))

    print("D'. the write itself")
    kp = os.path.join(d, "atomic.json")
    persistent_map.save_keepouts([rect("un", 0, 0, 1, 1)], kp, frame="v1")
    first = open(kp).read()
    persistent_map.save_keepouts([rect("deux", 0, 0, 2, 2)], kp, frame="v2", backup=True)
    check("the previous file is kept as keepout.json.bak", os.path.isfile(kp + ".bak"))
    check("and the .bak is the previous version, byte for byte",
          open(kp + ".bak").read() == first)
    check("the live file is the new version",
          persistent_map.load_keepouts(kp)[0]["label"] == "deux")
    check("no .tmp is left behind", not os.path.exists(kp + ".tmp"))

    good = open(kp).read()
    try:
        persistent_map.save_keepouts([{"label": "boum", "x0": object()}], kp, backup=True)
        check("a zone that will not serialise raises", False)
    except TypeError as exc:
        check("a zone that will not serialise raises", True, type(exc).__name__)
    check("...and the live file is untouched by the failed write", open(kp).read() == good)
    check("...and no .tmp is left behind", not os.path.exists(kp + ".tmp"))
finally:
    shutil.rmtree(d, ignore_errors=True)

# --- E. the server ----------------------------------------------------------

print("E. the zone server: the map picture, and what it refuses to save")
d = tempfile.mkdtemp()
try:
    n, res, ox, oy = 40, RES, -1.0, -1.0
    lidar = np.zeros((n, n), np.int8)
    low = np.zeros((n, n), np.int8)
    seen = np.zeros((n, n), bool)
    # cell CENTRES, so "which cell is this metre in" has exactly one answer
    WALL_M, FLOOR_M, NEVER_M = (0.325, 0.525), (-0.675, -0.475), (0.925, -0.925)
    wall = (int((WALL_M[1] - oy) / res), int((WALL_M[0] - ox) / res))
    lidar[wall] = 4
    seen[wall] = True
    floor = (int((FLOOR_M[1] - oy) / res), int((FLOOR_M[0] - ox) / res))
    seen[floor] = True
    lidar[floor] = low[floor] = zone_server.FREE_FLOOR   # cleared on both layers
    mp = os.path.join(d, "map.npz")
    np.savez(mp, lidar=lidar, low=low, seen=seen, last_hit_xy=np.zeros((n, n, 2), np.float32),
             res=res, ox=ox, oy=oy, n=n, pose_xy=np.zeros(2), ts=time.time())
    png, meta = zone_server.render_map(mp)
    check("the PNG says it is a PNG", png[:8] == b"\x89PNG\r\n\x1a\n")
    import struct as _struct
    w_px, h_px = _struct.unpack(">II", png[16:24])
    check("one pixel per 5 cm cell -> 40 x 40", (w_px, h_px) == (40, 40), f"{w_px} x {h_px}")
    check("the metadata gives the extent in metres",
          (meta["res"], meta["ox"], meta["oy"], meta["n"]) == (0.05, -1.0, -1.0, 40)
          and meta["x_max"] == 1.0 and meta["y_max"] == 1.0)

    import zlib as _zlib
    idat = png[png.index(b"IDAT") + 4:png.rindex(b"IEND") - 8]   # ...IDAT data, crc, len, IEND
    raw = _zlib.decompress(idat)
    row_len = w_px * 3 + 1
    pixels = np.frombuffer(raw, np.uint8).reshape(h_px, row_len)[:, 1:].reshape(h_px, w_px, 3)

    def pixel_at(x, y):
        """The documented formulas, applied literally."""
        px = int((x - meta["ox"]) / meta["res"])
        py = int(meta["height"] - (y - meta["oy"]) / meta["res"])
        return tuple(int(v) for v in pixels[py, px])

    check(f"the obstacle at ({WALL_M[0]:+.3f}, {WALL_M[1]:+.3f}) m is red in the picture",
          pixel_at(*WALL_M) == zone_server.COLOR_OCCUPIED, f"{pixel_at(*WALL_M)}")
    check(f"the cleared floor at ({FLOOR_M[0]:+.3f}, {FLOOR_M[1]:+.3f}) m is light grey",
          pixel_at(*FLOOR_M) == zone_server.COLOR_CLEAR, f"{pixel_at(*FLOOR_M)}")
    check("a cell nobody ever saw is dark", pixel_at(*NEVER_M) == zone_server.COLOR_UNKNOWN,
          f"{pixel_at(*NEVER_M)}")
    check("the picture is not upside down: +0.5 m is in the upper half, -0.5 m in the lower",
          int(meta["height"] - (0.525 - meta["oy"]) / meta["res"]) < meta["height"] / 2
          < int(meta["height"] - (-0.475 - meta["oy"]) / meta["res"]))
    check("one pixel is one cell: a neighbour cell is untouched",
          pixel_at(WALL_M[0] + res, WALL_M[1]) == zone_server.COLOR_UNKNOWN)
    if HAVE_DIMOS:
        check("the server reads the map exactly as the costmap does",
              zone_server.OCCUPIED_AT == OCCUPIED_AT and zone_server.FREE_FLOOR == FREE_FLOOR)

    print("E'. what the UI is not allowed to save")
    bounds = (meta["x_min"], meta["y_min"], meta["x_max"], meta["y_max"])
    ok_doc = {"zones": [{"label": "maison", "type": "forbidden",
                         "points": [[0.1, 0.1], [0.5, 0.1], [0.5, 0.5]], "note": "a la souris"}]}
    saved = zone_server.validate_zones(ok_doc, bounds)
    check("a drawn triangle is accepted and normalised",
          saved == [{"label": "maison", "type": FORBIDDEN,
                     "points": [[0.1, 0.1], [0.5, 0.1], [0.5, 0.5]], "note": "a la souris"}],
          f"{saved}")

    def refused(what, doc):
        try:
            zone_server.validate_zones(doc, bounds)
            check(what, False, "accepted!")
        except zone_server.Refused as exc:
            check(what, True, str(exc))

    refused("a polygon with 2 points",
            {"zones": [{"label": "z", "points": [[0, 0], [1, 1]]}]})
    refused("a zone with no name", {"zones": [{"label": "  ", "points": [[0, 0], [1, 0], [1, 1]]}]})
    refused("two zones with the same name",
            {"zones": [{"label": "z", "points": [[0, 0], [1, 0], [1, 1]]},
                       {"label": "z", "points": [[0, 0], [1, 0], [1, 1]]}]})
    refused("an unknown type",
            {"zones": [{"label": "z", "type": "wat", "points": [[0, 0], [1, 0], [1, 1]]}]})
    refused("a vertex that is not a number",
            {"zones": [{"label": "z", "points": [[0, "gauche"], [1, 0], [1, 1]]}]})
    refused("a zone 100 m off the map",
            {"zones": [{"label": "z", "points": [[100, 100], [101, 100], [101, 101]]}]})
    refused("something that is not a list of zones", {"zones": "toutes"})
    check("a legacy rectangle still passes through untouched",
          zone_server.validate_zones({"zones": [{"label": "r", "x0": 0.9, "y0": 0.9,
                                                 "x1": -0.9, "y1": -0.9}]}, bounds)
          == [{"label": "r", "type": FORBIDDEN, "x0": -0.9, "y0": -0.9, "x1": 0.9, "y1": 0.9,
               "note": ""}])
finally:
    shutil.rmtree(d, ignore_errors=True)

# --- F. the Rerun overlay ---------------------------------------------------

print("F. the Rerun overlay: a polygon drawn as the cells the rover really obeys")
if not HAVE_DIMOS:
    print("  -- skipped, no dimOS here: run this on the Jetson")
else:
    from vector_dimos import nav_blueprints

    d = tempfile.mkdtemp()
    saved_path = persistent_map.KEEPOUT_PATH
    real_load = persistent_map.load_keepouts
    try:
        kp = os.path.join(d, "keepout.json")
        HOUSE = poly("maison", tilted(1.0, 1.0, 0.8, 0.4, 5.75), FORBIDDEN)
        persistent_map.save_keepouts([HOUSE], kp, frame="cold bench")
        # nav_blueprints calls load_keepouts() with no argument, and that
        # default path was bound at import: point both at the bench file.
        persistent_map.KEEPOUT_PATH = kp
        real_load = persistent_map.load_keepouts
        persistent_map.load_keepouts = lambda path=kp: real_load(path)
        nav_blueprints._FORBIDDEN_ZONES["mtime"] = None
        nav_blueprints._ZONE_RASTER["key"] = None

        crop_ox, crop_oy, h, w = 0.0, 0.0, 60, 60          # a 3 x 3 m published crop
        zone_cells = persistent_map.polygon_mask(HOUSE["points"], RES, crop_ox, crop_oy, (h, w))
        cells = np.full((h, w), -1, dtype=np.int8)
        cells[zone_cells] = 100                            # the map applied the zone
        mask, boxes = nav_blueprints._forbidden_zones(cells, RES, crop_ox, crop_oy)
        check("a polygon in force -> an overlay, not a refusal", mask is not None)
        check("the overlay covers exactly the cells the map forced",
              mask is not None and np.array_equal(mask, zone_cells),
              f"{0 if mask is None else int(mask.sum())} cells for {int(zone_cells.sum())} forced")
        centers, half_sizes, labels = boxes
        slabs = sum(2 * hs[0] * 2 * hs[1] for hs in half_sizes)
        check("it is drawn as one slab per row of cells, covering the same area",
              abs(slabs - int(zone_cells.sum()) * RES * RES) < 1e-6,
              f"{len(centers)} slabs, {slabs:.3f} m2")
        check("exactly one slab carries the label", labels.count("maison") == 1)

        nav_blueprints._ZONE_RASTER["key"] = None
        cells[zone_cells] = -1                             # a run that never applied it
        mask, boxes = nav_blueprints._forbidden_zones(cells, RES, crop_ox, crop_oy)
        check("a polygon NOT in force -> nothing drawn, no orange rule the rover ignores",
              mask is None and boxes is None)
    finally:
        persistent_map.KEEPOUT_PATH = saved_path
        persistent_map.load_keepouts = real_load
        nav_blueprints._FORBIDDEN_ZONES["mtime"] = None
        nav_blueprints._ZONE_RASTER["key"] = None
        shutil.rmtree(d, ignore_errors=True)


print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

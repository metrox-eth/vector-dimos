"""Cold bench for the persistent map: relocalization, keep-out zones, generations.

Rule #2: a known input must give a known output, in physical units. Groups:

  A. distance_field   - a wall at a known place, distances read in metres
  B. relocalize       - a scan taken at a KNOWN pose (1.20, -0.70, 35 deg) in a
                        synthetic flat must come back within 5 cm and 2 deg
  C. rejection        - a scan from an ALIEN room must be refused, and say why
  D. forbidden zones  - forced to 100 after every layer; body_clear, a lidar
                        ray fails to erase them
  E. no_slip_reflex   - the ramp: the two slip guards stay silent inside it and
                        trip on the very same numbers outside it
  F. files            - keepout.json round trip, persistent map generations
  G. the state machine - origin arithmetic in metres, and what a boot rejection,
                        a late acceptance and a hand-carry each do to the frame

A, B, C and E need numpy only. D needs dimOS (ScoredGrid), so it runs on the
Jetson.

Run:  .venv/bin/python3 tests/test_relocalization_cold.py
"""

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
from vector_dimos.relocalize2d import (  # noqa: E402
    MapField, distance_field, relocalize,
)

try:
    from vector_dimos.costmap2d import OCCUPIED_AT, ScoredGrid
    HAVE_DIMOS = True
except Exception:  # noqa: BLE001 - no dimOS on a laptop
    HAVE_DIMOS = False

FORBIDDEN = persistent_map.FORBIDDEN
NO_SLIP = "no_slip_reflex"   # the LEGACY type: died with the slip detectors 26/08

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


# --- a synthetic flat, and a lidar that looks at it -------------------------

def flat_walls():
    """An L-shaped flat, 8 x 7 m, with a pillar. Asymmetric on purpose: a plain
    rectangle has a perfect 180 deg twin and no relocalizer can pick between
    them - which is exactly what the margin criterion is there to say."""
    walls = [((-4, -3), (4, -3)), ((4, -3), (4, 1)), ((4, 1), (1, 1)),
             ((1, 1), (1, 4)), ((1, 4), (-4, 4)), ((-4, 4), (-4, -3))]
    px, py, half = 2.0, -1.5, 0.2
    walls += [((px - half, py - half), (px + half, py - half)),
              ((px + half, py - half), (px + half, py + half)),
              ((px + half, py + half), (px - half, py + half)),
              ((px - half, py + half), (px - half, py - half))]
    return walls


def alien_walls():
    """A narrow corridor: nothing in the flat looks like this."""
    return [((-1, -5), (1, -5)), ((1, -5), (1, 6)), ((1, 6), (-1, 6)), ((-1, 6), (-1, -5))]


def revolution(walls, origin, n_rays=400, max_r=12.0, min_r=0.15):
    """One lidar revolution from `origin`, in world coordinates."""
    ox, oy = origin
    hits = []
    for a in np.linspace(-math.pi, math.pi, n_rays, endpoint=False):
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for (x1, y1), (x2, y2) in walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > min_r and 0.0 <= u <= 1.0 and (best is None or t < best):
                best = t
        if best is not None and best < max_r:
            hits.append((ox + dx * best, oy + dy * best))
    return np.array(hits)


def in_scan_frame(pts_world, pose):
    """World points seen from `pose`, expressed in the rover's own frame - what
    lidar odometry accumulates before it knows where it is."""
    c, s = math.cos(-pose[2]), math.sin(-pose[2])
    d = pts_world - np.array([pose[0], pose[1]])
    return np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)


def rasterize(walls, span=24.0, res=RES, inside=(0.0, 0.0)):
    """Walls into an occupancy pair (occupied, observed-free), like a saved map."""
    from collections import deque
    n = int(round(span / res))
    ox, oy = -span / 2, -span / 2
    occ = np.zeros((n, n), bool)
    for (x1, y1), (x2, y2) in walls:
        length = math.hypot(x2 - x1, y2 - y1)
        for t in np.linspace(0, 1, int(length / (res / 2)) + 2):
            gx = int((x1 + (x2 - x1) * t - ox) / res)
            gy = int((y1 + (y2 - y1) * t - oy) / res)
            if 0 <= gx < n and 0 <= gy < n:
                occ[gy, gx] = True
    free = np.zeros((n, n), bool)
    sx, sy = int((inside[0] - ox) / res), int((inside[1] - oy) / res)
    q = deque([(sy, sx)])
    free[sy, sx] = True
    while q:
        y, x = q.popleft()
        for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
            if 0 <= nx < n and 0 <= ny < n and not free[ny, nx] and not occ[ny, nx]:
                free[ny, nx] = True
                q.append((ny, nx))
    return occ, free, ox, oy


# --- A. the distance field --------------------------------------------------

print("A. distance_field (metres to the nearest occupied cell)")
occ = np.zeros((40, 40), bool)
occ[20, 20] = True
d = distance_field(occ, RES, radius_m=0.40)
check("on the wall -> 0 m", d[20, 20] == 0.0, f"{d[20, 20]}")
check("one cell away -> 0.05 m", abs(d[20, 21] - 0.05) < 1e-6, f"{d[20, 21]:.4f}")
check("four cells away -> 0.20 m", abs(d[20, 24] - 0.20) < 1e-6, f"{d[20, 24]:.4f}")
check("diagonal 3,4 cells -> 0.25 m", abs(d[23, 24] - 0.25) < 1e-6, f"{d[23, 24]:.4f}")
check("beyond the radius -> saturated at 0.40 m", d[0, 0] == 0.40, f"{d[0, 0]}")

# --- B. a scan at a known pose ---------------------------------------------

print("B. relocalize a scan taken at a KNOWN pose (1.20, -0.70, +35.0 deg)")
TRUE = (1.20, -0.70, math.radians(35.0))
occ, free, ox, oy = rasterize(flat_walls())
t0 = time.perf_counter()
field = MapField(occ, free, RES, ox, oy)
build_ms = (time.perf_counter() - t0) * 1000
scan = in_scan_frame(revolution(flat_walls(), (TRUE[0], TRUE[1])), TRUE)
t0 = time.perf_counter()
m = relocalize(field, scan)
search_s = time.perf_counter() - t0
err_xy = math.hypot(m.x - TRUE[0], m.y - TRUE[1])
err_yaw = abs(math.degrees(math.atan2(math.sin(m.yaw - TRUE[2]), math.cos(m.yaw - TRUE[2]))))
print(f"     {m.as_log()}")
check("accepted", m.accepted, m.reason)
check("position within 5 cm", err_xy < 0.05, f"{err_xy * 100:.1f} cm")
check("heading within 2 deg", err_yaw < 2.0, f"{err_yaw:.2f} deg")
check("walls overlap under 5 cm", m.median_dist_m < 0.05, f"{m.median_dist_m * 100:.1f} cm median")
check("the winner beats the rival basin", m.margin >= 1.25, f"margin {m.margin:.2f}")
check("field build + global search under 5 s", build_ms / 1000 + search_s < 5.0,
      f"{build_ms:.0f} ms build + {search_s:.2f} s search")

print("B'. the same scan from three other known poses")
for truth in ((-2.50, 2.00, math.radians(-120.0)), (3.00, -2.00, math.radians(0.0)),
              (0.00, 3.00, math.radians(175.0))):
    sc = in_scan_frame(revolution(flat_walls(), (truth[0], truth[1])), truth)
    r = relocalize(field, sc)
    e_xy = math.hypot(r.x - truth[0], r.y - truth[1])
    e_yaw = abs(math.degrees(math.atan2(math.sin(r.yaw - truth[2]), math.cos(r.yaw - truth[2]))))
    check(f"({truth[0]:+.2f}, {truth[1]:+.2f}, {math.degrees(truth[2]):+.0f} deg) found",
          r.accepted and e_xy < 0.05 and e_yaw < 2.0,
          f"{e_xy * 100:.1f} cm, {e_yaw:.2f} deg, score {r.score:.3f}, margin {r.margin:.2f}")

# --- C. an alien room must be refused --------------------------------------

print("C. a scan from an ALIEN room must be REJECTED")
alien = revolution(alien_walls(), (0.0, 1.0)) - np.array([0.0, 1.0])
m_alien = relocalize(field, alien)
print(f"     {m_alien.as_log()}")
check("rejected", not m_alien.accepted, m_alien.reason)
check("its score is well below the winner's", m_alien.score < m.score - 0.25,
      f"{m_alien.score:.3f} vs {m.score:.3f}")

print("C'. too few points -> no verdict, never a guess")
m_thin = relocalize(field, scan[:20])
check("20 points -> rejected", not m_thin.accepted, m_thin.reason)

# --- D. keep-out zones ------------------------------------------------------

print("D. forbidden zones (need dimOS: ScoredGrid)")
ZONE = [{"label": "toilettes", "type": FORBIDDEN, "x0": 1.0, "y0": -0.5, "x1": 2.0, "y1": 0.5},
        {"label": "rampe", "type": NO_SLIP, "x0": -2.0, "y0": -0.5, "x1": -1.0, "y1": 0.5}]
if not HAVE_DIMOS:
    print("  -- skipped, no dimOS here: run this on the Jetson")
else:
    g = ScoredGrid(span_m=6.0)
    cells = g.set_keepouts(ZONE)
    check("a 1.0 x 1.0 m forbidden zone at 0.05 m -> 21 x 21 cells", cells == 441, f"{cells} cells")
    check("the legacy no_slip_reflex zone is NOT a wall in the map", g.value_at(-1.5, 0.0) == -1,
          f"value {g.value_at(-1.5, 0.0)}")
    check("inside the zone -> occupied", g.value_at(1.5, 0.0) == 100)
    check("outside the zone -> untouched (unknown)", g.value_at(0.0, 0.0) == -1)
    for _ in range(30):
        g.lidar_revolution(np.array([[2.6, 0.0]]), (0.0, 0.0))   # rays straight through the zone
    check("30 lidar rays through it -> still occupied", g.value_at(1.5, 0.0) == 100)
    g.camera_floor(np.array([[1.5, 0.0]]))
    check("the camera seeing bare floor there -> still occupied", g.value_at(1.5, 0.0) == 100)
    g.body_clear((1.5, 0.0, 0.0))
    check("body_clear ON the zone -> still occupied", g.value_at(1.5, 0.0) == 100)
    gx, gy = g.cell(np.array([1.5]), np.array([0.0]))
    check("the layers underneath really were cleared (nothing hidden)",
          g.lidar[gy[0], gx[0]] < OCCUPIED_AT, f"score {g.lidar[gy[0], gx[0]]}")
    check("dropping the zones frees the cell again", (g.set_keepouts([]) == 0
                                                      and g.value_at(1.5, 0.0) != 100))
    check("a file with only no_slip_reflex zones leaves the map alone",
          g.set_keepouts([ZONE[1]]) == 0)

    print("D'. a saved map is loadable, keep-outs land on the right cells")
    d = tempfile.mkdtemp()
    try:
        g2 = ScoredGrid(span_m=6.0)
        for i in range(3):
            g2.lidar_revolution(np.array([[1.0, 0.0]]), (0.0, 0.15 * i))
        path = os.path.join(d, "map.npz")
        g2.save(path, (0.0, 0.0))
        back = ScoredGrid.load(path)
        check("reloaded map keeps its obstacle", back.value_at(1.0, 0.0) == 100)
        back.lidar_revolution(np.array([[2.0, 0.0]]), (0.0, 0.0))   # a loaded map must be writable
        wgx, wgy = back.cell(np.array([2.0]), np.array([0.0]))
        check("a loaded map can be continued (still writable)", back.lidar[wgy[0], wgx[0]] > 0)
        back.set_keepouts(ZONE)
        check("keep-out applies to a loaded map", back.value_at(1.5, 0.0) == 100)
    finally:
        shutil.rmtree(d, ignore_errors=True)

# --- E. legacy zones ---------------------------------------------------------

print("E. legacy no_slip_reflex zones: dropped on load (the slip detectors died 26/08)")
RAMP = {"label": "rampe-cuisine-atelier", "type": NO_SLIP,
        "x0": -3.0, "y0": -8.2, "x1": -1.95, "y1": -7.75, "note": "26/08"}
d = tempfile.mkdtemp()
zone_file = os.path.join(d, "keepout.json")
persistent_map.save_keepouts([{"label": "toilettes", "type": FORBIDDEN,
                               "x0": 0.55, "y0": -9.95, "x1": 2.65, "y1": -6.65, "note": ""}],
                             zone_file, frame="cold bench")
try:
    import json as _json
    doc = _json.load(open(zone_file))
    doc["zones"].append(RAMP)
    with open(zone_file, "w") as fh:
        _json.dump(doc, fh)
    back = persistent_map.load_keepouts(zone_file)
    check("a file carrying a legacy ramp zone loads without it, without crashing",
          [z["label"] for z in back] == ["toilettes"], f"{[z['label'] for z in back]}")
    doc["zones"].append({"label": "x", "type": "carrement_inconnu",
                         "x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 1.0})
    with open(zone_file, "w") as fh:
        _json.dump(doc, fh)
    try:
        persistent_map.load_keepouts(zone_file)
        check("a truly unknown type still raises", False)
    except ValueError as exc:
        check("a truly unknown type still raises", True, str(exc))
finally:
    shutil.rmtree(d, ignore_errors=True)

# --- F. the files -----------------------------------------------------------

print("F. keepout.json and the persistent map generations")
d = tempfile.mkdtemp()
try:
    kp = os.path.join(d, "keepout.json")
    persistent_map.save_keepouts(
        [{"label": "rampe", "type": FORBIDDEN, "x0": 2.0, "y0": 0.5, "x1": 1.0, "y1": -0.5,
          "note": "ramp"}], kp, frame="carte_saine")
    back = persistent_map.load_keepouts(kp)
    check("corners are normalised on the way back", back == [
        {"label": "rampe", "type": FORBIDDEN, "x0": 1.0, "y0": -0.5, "x1": 2.0, "y1": 0.5,
         "note": "ramp"}], f"{back}")
    check("the frame note survives", persistent_map.keepout_frame(kp) == "carte_saine")
    with open(kp, "w") as fh:
        fh.write('{"zones": [{"label": "x", "type": "wat", "x0": 0, "y0": 0, "x1": 1, "y1": 1}]}')
    try:
        persistent_map.load_keepouts(kp)
        check("an unknown zone type raises rather than being ignored", False)
    except ValueError as exc:
        check("an unknown zone type raises rather than being ignored", True, str(exc))
    with open(kp, "w") as fh:
        fh.write('[{"label": "old", "x0": 0, "y0": 0, "x1": 1, "y1": 1}]')
    old_schema = persistent_map.load_keepouts(kp)
    check("a bare list still reads, and defaults to forbidden",
          old_schema[0]["type"] == FORBIDDEN)
    check("a missing file is no zone, not a crash",
          persistent_map.load_keepouts(os.path.join(d, "nope.json")) == [])

    forbidden = [{"label": "z", "type": FORBIDDEN, "x0": 1.0, "y0": -0.5,
                  "x1": 2.0, "y1": 0.5, "note": ""}]
    mask = persistent_map.keepout_mask(forbidden, RES, -1.0, -1.0, 80)
    check("mask covers 21 x 21 cells at the right place", int(mask.sum()) == 441
          and mask[int((0.0 + 1.0) / RES), int((1.5 + 1.0) / RES)], f"{int(mask.sum())} cells")
    stale_legacy = [dict(back[0], type=NO_SLIP)]     # an old in-memory list, pre-26/08
    check("keepout_mask ignores a stale legacy no_slip_reflex zone",
          int(persistent_map.keepout_mask(stale_legacy, RES, -1.0, -1.0, 80).sum()) == 0)

    saved = persistent_map.MAP_PATH
    persistent_map.MAP_PATH = os.path.join(d, "persistent_map.npz")
    try:
        for gen in range(4):
            src = os.path.join(d, f"ck{gen}.npz")
            np.savez(src, lidar=np.zeros((4, 4), np.int8), gen=gen)
            persistent_map.promote(src, keep=2)
            os.utime(persistent_map.MAP_PATH, (time.time() - 100 * (3 - gen),) * 2)
        check("the newest promotion is the live map",
              int(np.load(persistent_map.MAP_PATH)["gen"]) == 3)
        gens = persistent_map.generations()
        check("4 promotions, keep=2 -> exactly 2 older generations", len(gens) == 2,
              f"{[os.path.basename(g) for g in gens]}")
        check("the live map is never one of its own generations",
              persistent_map.MAP_PATH not in gens)
        check("the previous map is recoverable", int(np.load(gens[0])["gen"]) == 2,
              os.path.basename(gens[0]))
    finally:
        persistent_map.MAP_PATH = saved

    os.environ.pop("PERSISTENT_MAP", None)
    check("PERSISTENT_MAP defaults to on", persistent_map.enabled())
    os.environ["PERSISTENT_MAP"] = "0"
    check("PERSISTENT_MAP=0 turns everything off", not persistent_map.enabled())
    os.environ["PERSISTENT_MAP"] = "1"
    check("rebasing the saved flat is off by default", not persistent_map.rebase_allowed())
finally:
    shutil.rmtree(d, ignore_errors=True)


# --- G. the frame state machine --------------------------------------------

print("G. lidar odometry: the origin, and what each verdict does to the frame")
if not HAVE_DIMOS:
    print("  -- skipped, no dimOS here: run this on the Jetson")
else:
    from vector_dimos import lidar_odometry as LO
    from vector_dimos.relocalize2d import Match

    def verdict(x=0.0, y=0.0, yaw=0.0, accepted=True, score=0.9, margin=1.4):
        return Match(x, y, yaw, score, margin, score / margin, (9.0, 9.0, 0.0),
                     0.0, 1.0, 400, 0.1, accepted,
                     "ACCEPTED" if accepted else "REJECTED: margin too low")

    class Threshold:
        def __init__(self, v=0.2):
            self.v = v

        def get_threshold(self):
            return self.v

    class Kiss:
        def __init__(self):
            self.adaptive_threshold = Threshold()

    def odometry():
        o = LO.LidarOdometry.__new__(LO.LidarOdometry)
        o._origin = (0.0, 0.0, 0.0); o._frame = "fresh"; o._reloc_state = "idle"
        o._reloc_pts = []; o._reloc_thread = None; o._reloc_result = None
        o._reloc_reason = ""; o._reloc_next = 0.0; o._boot_deadline = 0.0; o._reloc_gen = 0
        o._pose_hist = []; o._wheel_hist = []; o._lost_since = 0.0
        o._carry_cooldown = 0.0; o._cfg = None; o._kiss = Kiss()
        o._gyro_acc = 0.0; o._gyro_seen = False
        return o

    # the origin is a transform, in metres and degrees
    o = odometry()
    check("no origin -> the kiss pose is the map pose",
          o._to_map_frame((1.5, -0.5, math.radians(20.0))) == (1.5, -0.5, math.radians(20.0)))
    o._origin = (2.0, -1.0, math.radians(90.0))
    x, y, yaw = o._to_map_frame((1.0, 0.0, 0.0))
    check("origin (2, -1, +90 deg) + kiss (1, 0, 0) -> map (2, 0, +90 deg)",
          abs(x - 2.0) < 1e-9 and abs(y - 0.0) < 1e-9 and abs(math.degrees(yaw) - 90.0) < 1e-9,
          f"({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.1f} deg)")
    x, y, yaw = o._to_map_frame((0.0, 2.0, math.radians(-90.0)))
    check("origin (2, -1, +90 deg) + kiss (0, 2, -90 deg) -> map (0, -1, 0 deg)",
          abs(x - 0.0) < 1e-9 and abs(y + 1.0) < 1e-9 and abs(math.degrees(yaw)) < 1e-9,
          f"({x:+.3f}, {y:+.3f}, {math.degrees(yaw):+.1f} deg)")

    # boot: accepted straight away
    o = odometry(); o._begin_relocalization("boot", reset_kiss=False)
    check("boot search -> map writing frozen", o._searching)
    o._reloc_result = (verdict(1.2, -0.7, math.radians(35.0)), persistent_map.MAP_PATH, o._reloc_gen)
    o._collect_relocalization()
    check("boot accepted -> persistent frame, origin set, writing resumed",
          o._frame == "persistent" and not o._searching and o._reloc_state == "idle"
          and abs(o._origin[0] - 1.2) < 1e-9, f"origin {o._origin}")

    # boot: refused -> map fresh NOW, keep trying while the rover moves
    o = odometry(); o._begin_relocalization("boot", reset_kiss=False)
    o._reloc_result = (verdict(accepted=False, margin=1.01), persistent_map.MAP_PATH, o._reloc_gen)
    o._collect_relocalization()
    check("boot refused -> fresh frame, unfrozen, still trying",
          o._frame == "fresh" and not o._searching and o._reloc_state == "retrying")
    check("the grace window is open", o._boot_deadline > time.monotonic())
    o._reloc_result = (verdict(-3.9, -9.0, math.radians(-160.0)), persistent_map.MAP_PATH, o._reloc_gen)
    o._collect_relocalization()
    check("accepted later in the grace window -> the frame swaps to persistent",
          o._frame == "persistent" and o._reloc_state == "idle"
          and abs(o._origin[1] + 9.0) < 1e-9, f"origin {o._origin}")

    # boot: refused until the grace runs out
    o = odometry(); o._begin_relocalization("boot", reset_kiss=False)
    o._reloc_result = (verdict(accepted=False), persistent_map.MAP_PATH, o._reloc_gen)
    o._collect_relocalization()
    o._boot_deadline = time.monotonic() - 1.0
    o._reloc_pts = [np.zeros((10, 2))] * LO.RELOC_REVS
    o._accumulate(np.zeros((10, 2)), (0.0, 0.0, 0.0))
    check("grace expired -> gives up, keeps the fresh frame",
          o._reloc_state == "idle" and o._frame == "fresh")

    # a hand-carry that cannot be resolved must NOT unfreeze
    o = odometry(); o._frame = "persistent"
    o._begin_relocalization("carried", reset_kiss=False)
    o._reloc_result = (verdict(accepted=False), "/tmp/current.npz", o._reloc_gen)
    o._collect_relocalization()
    check("hand-carry refused -> stays FROZEN, never corrupts the map",
          o._searching and o._frame == "persistent")
    o._reloc_result = (verdict(3.0, 4.0, 0.0), "/tmp/current.npz", o._reloc_gen)
    o._collect_relocalization()
    check("hand-carry resolved -> unfrozen, still the persistent frame",
          not o._searching and o._frame == "persistent" and o._origin[:2] == (3.0, 4.0))

    # an answer computed for a frame we have since left must be dropped
    o = odometry(); o._frame = "persistent"
    o._begin_relocalization("carried", reset_kiss=False)
    stale = o._reloc_gen
    o._begin_relocalization("carried", reset_kiss=False)      # picked up again mid-search
    o._reloc_result = (verdict(99.0, 99.0, 0.0), "/tmp/current.npz", stale)
    o._collect_relocalization()
    check("a stale answer is dropped, the origin is untouched",
          o._origin == (0.0, 0.0, 0.0) and o._searching, f"origin {o._origin}")

    # the hand-carry trigger, in metres
    def carried(body_m, wheel_m, sigma=0.2):
        o = odometry(); o._kiss.adaptive_threshold.v = sigma
        now = time.time()
        o._pose_hist = [(now - 1.5, 0.0, 0.0, 0.0), (now, body_m, 0.0, 0.0)]
        o._wheel_hist = [(now - 1.5, 0.0, 0.0), (now, wheel_m, 0.0)]
        return o._carried(now, body_m, 0.0)

    check("body 0.60 m, wheels 0.00 m in 1 s -> carried", carried(0.60, 0.0))
    check("body 0.60 m, wheels 0.55 m (honest driving) -> not carried",
          not carried(0.60, 0.55))
    check("body 0.20 m, wheels 0.00 m (a nudge) -> not carried", not carried(0.20, 0.0))
    check("wheels spinning, body still (a slip, not a carry) -> not carried",
          not carried(0.0, 0.60))
    o = odometry(); o._kiss.adaptive_threshold.v = 2.0
    now = time.time(); o._pose_hist = [(now, 0.0, 0.0, 0.0)]
    o._lost_since = time.monotonic() - LO.LOST_SIGMA_S - 1.0
    check("scan matching threshold stuck above 1 m -> relocalize", o._carried(now, 0.0, 0.0))


print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

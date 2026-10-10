"""Cold bench for vector_dimos.robot_profile: one body per robot, VECTOR unchanged.

Rule #2: a known input gives a known output, in physical units. Groups:

  A. VECTOR      - the profile carries today's constants, value for value
                   (0.37 m, 2.75 deg, mast 348..12 deg under 0.30 m, body box
                   0.33 x 0.25, body clear 0.31 x 0.23, width 0.50, pivot 0.78
                   -> lethal 0.30 m, pivot clearance 0.39 m)
  B. sources     - the same numbers read out of rplidar_c1 / lidar_odometry /
                   costmap2d / recovering_planner source (ast, no import): the
                   profile and the module constants cannot drift apart
  C. MOSS        - the placeholders, ROBOT_PROFILE selection at call time, the
                   env overrides (MOSS_ROBOT_WIDTH_M=0.40 -> lethal 0.25 m),
                   a malformed value keeps the placeholder
  D. the lidar   - rplidar_c1 under ROBOT_PROFILE=moss: a return at 0.30 m
                   dead ahead that VECTOR drops is kept, one inside MOSS's body
                   box is dropped, polar_to_xy(90 deg, 1000 mm) = (0, -1.0) m;
                   the CP2102N glob comes after VECTOR's two sticks
  E. the map     - costmap2d tags lidar returns at the profile's height (0.20 m
                   on MOSS, 0.37 m on VECTOR), body_clear clears the profile's
                   footprint, the planner's clearances follow the profile

A-C need nothing. D and E import the modules: with dimOS when it is there
(the Jetson), else behind a stand-in for every dimOS name (the rig) - the
functions under test are pure numpy and never touch dimOS objects.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_robot_profile_cold.py
"""
import ast
import asyncio
import importlib.abc
import importlib.machinery
import math
import os
import sys
import tempfile
import threading
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from vector_dimos import robot_profile as rp  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def close(a, b, tol=1e-9):
    return abs(a - b) <= tol


for var in [k for k in os.environ if k.startswith("MOSS_")] + ["ROBOT_PROFILE"]:
    os.environ.pop(var, None)

# ── A. VECTOR = today's constants ─────────────────────────────────────────
print("A. VECTOR profile = the historical constants")
V = rp.VECTOR
check("lidar height 0.37 m", V.lidar_height_m == 0.37, f"{V.lidar_height_m}")
check("lidar yaw offset 2.75 deg", V.lidar_yaw_offset_deg == 2.75)
check("lidar axis = base_link origin (0, 0)", V.lidar_xy_base_m == (0.0, 0.0))
check("scan min range 0.15 m", V.scan_min_range_m == 0.15)
check("mast mask ((348, 12),) deg under 0.30 m",
      V.scan_mask_sectors_deg == ((348.0, 12.0),) and V.scan_mask_range_m == 0.30)
check("body box 0.33 x 0.25 m (half extents)", V.body_box_m == (0.33, 0.25))
check("body clear 0.31 x 0.23 m (half extents)", V.body_clear_m == (0.31, 0.23))
check("width 0.50 m, pivot 0.78 m, margin 0.05 m",
      (V.robot_width_m, V.pivot_diameter_m, V.control_margin_m) == (0.50, 0.78, 0.05))
check("camera (0.14, 0, 0.32) m, 6.3 deg down",
      V.camera_xyz_base_m == (0.14, 0.0, 0.32) and V.camera_pitch_deg == 6.3)
check("speed ceiling 0.35 m/s (garde_vitesse LIMIT_MS)", V.max_speed_mps == 0.35)
d = rp.derived(V)
check("derived lethal clearance 0.30 m", close(d.lethal_clearance_m, 0.30), f"{d.lethal_clearance_m!r}")
check("derived pivot clearance 0.39 m", close(d.pivot_clearance_m, 0.39), f"{d.pivot_clearance_m!r}")
check("derived == recovering_planner's float expressions, bit for bit",
      d.lethal_clearance_m == 0.50 / 2 + 0.05 and d.pivot_clearance_m == 0.78 / 2)
check("ROBOT_PROFILE unset -> VECTOR", rp.current() is V)
check("VECTOR freezes the map when exploration stops (today's costmap)", V.freeze_map_on_stop is True)
try:
    V.lidar_height_m = 1.0   # type: ignore[misc]
    check("assigning a field raises", False, "no exception")
except Exception as exc:  # dataclasses.FrozenInstanceError
    check("assigning a field raises", True, type(exc).__name__)


# ── B. the module constants, read out of the sources ─────────────────────
print("B. module constants (source, no import) = the VECTOR profile")


def constants(rel):
    """Module-level NAME = literal (or math.radians(literal)) assignments."""
    tree = ast.parse((ROOT / rel).read_text())
    out = {}
    for node in tree.body:
        targets, value = [], None
        if isinstance(node, ast.Assign):
            targets, value = node.targets, node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None:
            targets, value = [node.target], node.value
        for t in targets:
            if not isinstance(t, ast.Name):
                continue
            try:
                out[t.id] = ast.literal_eval(value)
            except ValueError:
                if (isinstance(value, ast.Call) and getattr(value.func, "attr", "") == "radians"
                        and len(value.args) == 1):
                    out[t.id] = ("radians", ast.literal_eval(value.args[0]))
    return out


rc = constants("vector_dimos/rplidar_c1.py")
lo = constants("vector_dimos/lidar_odometry.py")
cm = constants("vector_dimos/costmap2d.py")
pl = constants("vector_dimos/recovering_planner.py")
check("rplidar_c1.LIDAR_YAW_OFFSET_DEG", rc.get("LIDAR_YAW_OFFSET_DEG") == V.lidar_yaw_offset_deg, f"{rc.get('LIDAR_YAW_OFFSET_DEG')}")
check("rplidar_c1.MAST_MASK_DEG", rc.get("MAST_MASK_DEG") == V.scan_mask_sectors_deg, f"{rc.get('MAST_MASK_DEG')}")
check("rplidar_c1.MASK_RANGE_M", rc.get("MASK_RANGE_M") == V.scan_mask_range_m)
check("rplidar_c1.MIN_RANGE_M", rc.get("MIN_RANGE_M") == V.scan_min_range_m)
check("lidar_odometry.LIDAR_HEIGHT_M", lo.get("LIDAR_HEIGHT_M") == V.lidar_height_m)
check("lidar_odometry.CAMERA_XYZ_BASE", lo.get("CAMERA_XYZ_BASE") == V.camera_xyz_base_m)
check("lidar_odometry.CAMERA_PITCH_RAD = radians(6.3)",
      lo.get("CAMERA_PITCH_RAD") == ("radians", V.camera_pitch_deg), f"{lo.get('CAMERA_PITCH_RAD')}")
check("costmap2d.LIDAR_Z_M = the lidar height", cm.get("LIDAR_Z_M") == V.lidar_height_m)
check("recovering_planner ROBOT_WIDTH_M / PIVOT_DIAMETER_M / CONTROL_MARGIN_M",
      (pl.get("ROBOT_WIDTH_M"), pl.get("PIVOT_DIAMETER_M"), pl.get("CONTROL_MARGIN_M"))
      == (V.robot_width_m, V.pivot_diameter_m, V.control_margin_m))
src = (ROOT / "vector_dimos/rplidar_c1.py").read_text()
check("scan_to_points no longer hard-codes the body box (0.33 / 0.25)",
      "> 0.33 or" not in src and "p.body_box_m" in src)
src = (ROOT / "vector_dimos/costmap2d.py").read_text()
check("body_clear no longer hard-codes 0.31 / 0.23", "arange(-0.31" not in src and "body_clear_m" in src)


# ── C. MOSS placeholders and overrides ───────────────────────────────────
print("C. MOSS profile: placeholders, selection at call time, env overrides")
M = rp.MOSS
check("lidar 0.20 m up, yaw 0 deg, axis (0, 0)",
      (M.lidar_height_m, M.lidar_yaw_offset_deg, M.lidar_xy_base_m) == (0.20, 0.0, (0.0, 0.0)))
check("min range 0.15 m, no mask sector, mask range 0.30 m",
      (M.scan_min_range_m, M.scan_mask_sectors_deg, M.scan_mask_range_m) == (0.15, (), 0.30))
check("body box 0.22 x 0.18, body clear 0.20 x 0.16",
      M.body_box_m == (0.22, 0.18) and M.body_clear_m == (0.20, 0.16))
check("width 0.34, pivot 0.50, margin 0.05, track 0.266 m",
      (M.robot_width_m, M.pivot_diameter_m, M.control_margin_m, M.track_m) == (0.34, 0.50, 0.05, 0.266))
check("camera (0.156, 0, 0.075) m, pitch 0, ceiling 0.30 m/s",
      M.camera_xyz_base_m == (0.156, 0.0, 0.075) and M.camera_pitch_deg == 0.0 and M.max_speed_mps == 0.30)
dm = rp.derived(M)
check("MOSS lethal 0.22 m, pivot 0.25 m", close(dm.lethal_clearance_m, 0.22) and close(dm.pivot_clearance_m, 0.25),
      f"{dm.lethal_clearance_m:.4f} / {dm.pivot_clearance_m:.4f}")
os.environ["ROBOT_PROFILE"] = "moss"
check("ROBOT_PROFILE=moss -> MOSS (read now, not at import)", rp.current() == M)
check("MOSS keeps writing the map after exploration stops (freeze_map_on_stop False)",
      M.freeze_map_on_stop is False and rp.current().freeze_map_on_stop is False)
os.environ["ROBOT_PROFILE"] = " MOSS "
check("case and spaces tolerated", rp.current().name == "moss")
os.environ["ROBOT_PROFILE"] = "rover9"
check("unknown name -> VECTOR", rp.current() is V)
os.environ["ROBOT_PROFILE"] = "moss"
os.environ["MOSS_ROBOT_WIDTH_M"] = "0.40"
check("MOSS_ROBOT_WIDTH_M=0.40 -> lethal 0.25 m", close(rp.derived().lethal_clearance_m, 0.25),
      f"{rp.derived().lethal_clearance_m:.4f}")
check("... and it never leaks into VECTOR", rp.derived(V).lethal_clearance_m == 0.50 / 2 + 0.05)
os.environ["MOSS_BODY_BOX_M"] = "0.25 0.20"
os.environ["MOSS_SCAN_MASK_DEG"] = "350 10, 170 190"
os.environ["MOSS_CAMERA_XYZ_M"] = "0.16,0,0.08"
p = rp.current()
check("MOSS_BODY_BOX_M='0.25 0.20' -> (0.25, 0.20)", p.body_box_m == (0.25, 0.20))
check("MOSS_SCAN_MASK_DEG='350 10, 170 190' -> two sectors", p.scan_mask_sectors_deg == ((350.0, 10.0), (170.0, 190.0)))
check("MOSS_CAMERA_XYZ_M='0.16,0,0.08' -> (0.16, 0.0, 0.08)", p.camera_xyz_base_m == (0.16, 0.0, 0.08))
os.environ["MOSS_LIDAR_HEIGHT_M"] = "twenty"
os.environ["MOSS_BODY_CLEAR_M"] = "0.3"
p = rp.current()
check("MOSS_LIDAR_HEIGHT_M=twenty -> placeholder 0.20 kept", p.lidar_height_m == 0.20)
check("MOSS_BODY_CLEAR_M with one number -> placeholder kept", p.body_clear_m == (0.20, 0.16))
os.environ["MOSS_LIDAR_HEIGHT_M"] = "0.215"
check("MOSS_LIDAR_HEIGHT_M=0.215 -> 0.215 m", rp.current().lidar_height_m == 0.215)
for var in [k for k in os.environ if k.startswith("MOSS_")]:
    os.environ.pop(var, None)
check("env cleared -> the plain MOSS placeholders again", rp.current() == M)
check("state dir default ~/.local/state/moss, recordings/moss.db under it",
      rp.moss_recording_path({}) == Path.home() / ".local/state/moss/recordings/moss.db",
      str(rp.moss_recording_path({})))
os.environ.pop("ROBOT_PROFILE", None)


# ── D/E need the modules: real dimOS, or a stand-in for every dimOS name ──

class _Stub:
    """Any dimOS class / function / decorator: constructible, subscriptable,
    subclassable; never called by the functions under test."""

    def __init__(self, *args, **kwargs):
        pass

    def __class_getitem__(cls, item):
        return cls

    def __call__(self, *args, **kwargs):
        return args[0] if len(args) == 1 and callable(args[0]) else self

    def __getattr__(self, name):
        return lambda *a, **k: None


class _StubModule(types.ModuleType):
    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        cls = type(name, (_Stub,), {})
        setattr(self, name, cls)
        return cls


class _StubFinder(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def __init__(self, roots):
        self.roots = roots

    def find_spec(self, fullname, path, target=None):
        if fullname.split(".")[0] in self.roots:
            return importlib.machinery.ModuleSpec(fullname, self, is_package=True)
        return None

    def create_module(self, spec):
        mod = _StubModule(spec.name)
        mod.__path__ = []
        return mod

    def exec_module(self, module):
        pass


def _install_stubs():
    """Stand-ins only for what does not import here; returns what was stubbed."""
    stubbed = []
    for root, probe in (("dimos", "dimos.core.module"), ("dimos_lcm", "dimos_lcm.std_msgs")):
        try:
            __import__(probe)
        except Exception:  # noqa: BLE001 - absent, or present without its deps
            for name in [n for n in sys.modules if n == root or n.startswith(root + ".")]:
                del sys.modules[name]
            stubbed.append(root)
    if stubbed:
        sys.meta_path.insert(0, _StubFinder(set(stubbed)))
    return stubbed


stubbed = _install_stubs()
if stubbed:
    print(f"  (stand-ins for {', '.join(stubbed)}: not importable here)")

print("D. rplidar_c1 under ROBOT_PROFILE=moss")
from vector_dimos import rplidar_c1 as rpl  # noqa: E402

os.environ["ROBOT_PROFILE"] = "moss"
x, y = rpl.polar_to_xy(90.0, 1000.0)
check("polar_to_xy(90 deg, 1000 mm), MOSS yaw 0 -> (0, -1.0) m (SLAMTEC clockwise: y negated)",
      abs(x) < 1e-6 and abs(y + 1.0) < 1e-6, f"({x:.9f}, {y:.9f})")
x, y = rpl.polar_to_xy(0.0, 2000.0)
check("polar_to_xy(0 deg, 2000 mm), MOSS -> (2.0, 0) m: no 2.75 deg mounting term", close(x, 2.0) and close(y, 0.0, 1e-9))
scan = [(15, 0.0, 300.0),     # 0.30 m dead ahead
        (15, 0.0, 250.0),     # 0.25 m dead ahead: inside VECTOR's mast mask
        (15, 0.0, 200.0),     # 0.20 m dead ahead: inside MOSS's body box (0.22)
        (15, 90.0, 100.0),    # 0.10 m: under the 0.15 m floor everywhere
        (15, 180.0, 1500.0)]  # 1.5 m behind: a wall
pts_m = rpl.scan_to_points(scan, 0)
ahead = sorted(round(p[0], 3) for p in pts_m if abs(p[1]) < 1e-6 and p[0] > 0)
check("MOSS keeps the 0.30 m and 0.25 m returns ahead", ahead == [0.25, 0.3], f"kept ahead {ahead}")
check("MOSS drops the 0.20 m return (inside its 0.22 x 0.18 body box) and the 0.10 m one",
      len(pts_m) == 3, f"{len(pts_m)} points: {[(round(a, 3), round(b, 3)) for a, b, _ in pts_m]}")
check("the 1.5 m wall behind -> (-1.5, 0) m", any(close(p[0], -1.5, 1e-6) and abs(p[1]) < 1e-6 for p in pts_m))
os.environ.pop("ROBOT_PROFILE", None)
pts_v = rpl.scan_to_points(scan, 0)
check("VECTOR, same scan: only the wall survives (mast mask + 0.33 x 0.25 body box)",
      len(pts_v) == 1 and pts_v[0][0] < -1.4, f"{len(pts_v)} point(s)")
x, y = rpl.polar_to_xy(87.25, 2000.0)
check("VECTOR unchanged: 87.25 deg raw (theta 90), 2000 mm -> (0, -2.0) m",
      abs(x) < 1e-6 and abs(y + 2.0) < 1e-6)
check("explicit profile argument wins over the env",
      len(rpl.scan_to_points(scan, 0, profile=rp.MOSS)) == 3)
check("explicit mask still wins over the profile (VECTOR mask on MOSS: drops the 0.25 m return)",
      len(rpl.scan_to_points(scan, 0, mask=((348.0, 12.0),), mask_range_m=0.30, profile=rp.MOSS)) == 2)

saved = (rpl._KNOWN_STICKS, rpl._CP2102N_GLOB)
with tempfile.TemporaryDirectory() as d:
    vec = (os.path.join(d, "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"),
           os.path.join(d, "usb-FIREPHX_USB_SER_FX2348N-if00"))
    rpl._KNOWN_STICKS = vec
    rpl._CP2102N_GLOB = os.path.join(d, "usb-Silicon_Labs_CP2102N_*-port0")
    check("nothing plugged -> VECTOR's first stick (unchanged fallback)", rpl._resolve_port() == vec[0])
    moss = os.path.join(d, "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_62c3-if00-port0")
    open(moss, "w").close()
    check("a CP2102N alone -> taken", rpl._resolve_port() == moss)
    open(vec[1], "w").close()
    check("VECTOR's FIREPHX present too -> VECTOR's stick first", rpl._resolve_port() == vec[1])
    rpl._KNOWN_STICKS, rpl._CP2102N_GLOB = saved
check("the CP2102N glob never matches VECTOR's CP2102 (no N) stick",
      "CP2102N_" in rpl._CP2102N_GLOB and not rpl._CP2102N_GLOB.endswith("CP2102_*"))


print("E. costmap2d / recovering_planner / lidar_odometry read the profile")
try:
    from vector_dimos import costmap2d as cmod
    have_cm = True
except Exception as exc:  # noqa: BLE001
    have_cm = False
    print(f"  -- skipped (costmap2d does not import: {type(exc).__name__}: {exc})")
if have_cm:
    cloud = np.array([[1.0, 0.0, np.float32(0.37)], [2.0, 0.0, np.float32(0.20)],
                      [3.0, 0.0, 0.25]], dtype=np.float32)
    check("VECTOR: the z=0.37 point (float32) is a lidar return, z=0.20 is not",
          list(cmod.lidar_returns(cloud)) == [True, False, False])
    os.environ["ROBOT_PROFILE"] = "moss"
    check("MOSS: the z=0.20 point (float32) is the lidar return, 0.37 is a camera point",
          list(cmod.lidar_returns(cloud)) == [False, True, False])

    def cleared(hx=None, hy=None):
        """Cells body_clear marks at pose (0, 0, 0) on a 4 m grid; with hx, hy
        given, the historical formula with those literals instead (the code
        before the profile: arange(-0.31, 0.31 + 1e-9, res / 2) ...)."""
        g = cmod.ScoredGrid(resolution=0.05, span_m=4.0)
        if hx is None:
            g.body_clear((0.0, 0.0, 0.0))
            return g.seen
        lx = np.arange(-hx, hx + 1e-9, g.res / 2)
        ly = np.arange(-hy, hy + 1e-9, g.res / 2)
        bx, by = np.meshgrid(lx, ly)
        gx, gy = g.cell(bx.ravel(), by.ravel())
        seen = np.zeros_like(g.seen)
        seen[gy, gx] = True
        return seen

    moss_cells = cleared()
    os.environ.pop("ROBOT_PROFILE", None)
    vec_cells = cleared()
    check("VECTOR body_clear = the historical 0.31 x 0.23 footprint, cell for cell",
          np.array_equal(vec_cells, cleared(0.31, 0.23)), f"{int(vec_cells.sum())} cells")
    check("MOSS body_clear = a 0.20 x 0.16 footprint, cell for cell",
          np.array_equal(moss_cells, cleared(0.20, 0.16)), f"{int(moss_cells.sum())} cells")
    check("MOSS clears less floor than VECTOR (smaller body)",
          0 < moss_cells.sum() < vec_cells.sum(), f"{int(moss_cells.sum())} < {int(vec_cells.sum())}")

    def _pose(x, y):
        return types.SimpleNamespace(position=types.SimpleNamespace(x=x, y=y, z=0.0),
                                     orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    def after_stop(handler, open_first=True):
        """A VectorCostMap (fields as __init__ lays them, no dimOS runtime):
        boot pose (0, 0), optionally the explore order, then `handler`
        (Bool True), then the rover at (1.0, 0). Returns (map, cells cleared
        after the stop)."""
        vc = cmod.VectorCostMap.__new__(cmod.VectorCostMap)
        vc._frozen, vc._mission_frozen, vc._mission_over = False, True, False
        vc._frame, vc._grid = "fresh", cmod.ScoredGrid(resolution=0.05, span_m=4.0)
        vc._pose_xy, vc._pose_yaw, vc._last_clear, vc._boot_xy = None, 0.0, None, None
        vc._t0 = 0.0
        asyncio.run(vc.handle_odom(_pose(0.0, 0.0)))
        if open_first:
            asyncio.run(vc.handle_explore_cmd(types.SimpleNamespace(data=True)))
        asyncio.run(getattr(vc, handler)(types.SimpleNamespace(data=True)))
        before = int(vc._grid.seen.sum())
        asyncio.run(vc.handle_odom(_pose(1.0, 0.0)))
        return vc, int(vc._grid.seen.sum()) - before

    def footprint_at_1m():
        g1 = cmod.ScoredGrid(resolution=0.05, span_m=4.0)
        g1.body_clear((1.0, 0.0, 0.0))
        return int(g1.seen.sum())

    vec_fp1 = footprint_at_1m()
    os.environ["ROBOT_PROFILE"] = "moss"
    moss_fp1 = footprint_at_1m()
    vc, n = after_stop("handle_stop_explore_cmd")
    check("MOSS: stop_explore_cmd after explore_cmd -> mission NOT over, map still open",
          vc._mission_over is False and vc._write_frozen() is False)
    check("MOSS: the rover at (1.0, 0) after the stop -> body_clear writes its 0.20 x 0.16 footprint",
          n == moss_fp1 and moss_fp1 < vec_fp1, f"{n} cells written (MOSS footprint {moss_fp1}, VECTOR {vec_fp1})")
    vc, n = after_stop("handle_explore_done")
    check("MOSS: explore_done -> map still open, footprint written", vc._write_frozen() is False and n > 0, f"{n} cells")
    vc, n = after_stop("handle_stop_explore_cmd", open_first=False)
    check("MOSS: a stop BEFORE the map opened -> the 5 cm gate still opens it (1.0 m move writes)",
          vc._mission_over is False and vc._write_frozen() is False and n > 0, f"{n} cells")
    os.environ.pop("ROBOT_PROFILE", None)
    vc, n = after_stop("handle_stop_explore_cmd")
    check("VECTOR unchanged: stop_explore_cmd -> mission over, map frozen, 0 cells written after",
          vc._mission_over is True and vc._write_frozen() is True and n == 0, f"{n} cells")
    vc, n = after_stop("handle_explore_done")
    check("VECTOR unchanged: explore_done -> frozen, 0 cells", vc._mission_over is True and n == 0)
    asyncio.run(vc.handle_explore_cmd(types.SimpleNamespace(data=True)))
    check("VECTOR unchanged: a later explore_cmd does not reopen a finished mission", vc._write_frozen() is True)

try:
    from vector_dimos import recovering_planner as rpm
    have_pl = True
except Exception as exc:  # noqa: BLE001
    have_pl = False
    print(f"  -- skipped (recovering_planner does not import: {type(exc).__name__}: {exc})")
if have_pl:
    check("VECTOR clearances() == (LETHAL_CLEARANCE_M, PIVOT_CLEARANCE_M) exactly",
          rpm.clearances() == (rpm.LETHAL_CLEARANCE_M, rpm.PIVOT_CLEARANCE_M), f"{rpm.clearances()}")
    os.environ["ROBOT_PROFILE"] = "moss"
    lth, pv = rpm.clearances()
    check("MOSS clearances() = (0.22, 0.25) m", close(lth, 0.22) and close(pv, 0.25), f"({lth:.4f}, {pv:.4f})")
    os.environ["MOSS_ROBOT_WIDTH_M"] = "0.40"
    check("MOSS_ROBOT_WIDTH_M=0.40 -> planner lethal 0.25 m", close(rpm.clearances()[0], 0.25))
    os.environ.pop("MOSS_ROBOT_WIDTH_M", None)
    os.environ.pop("ROBOT_PROFILE", None)
    check("module constants untouched (0.30 / 0.39)",
          close(rpm.LETHAL_CLEARANCE_M, 0.30) and close(rpm.PIVOT_CLEARANCE_M, 0.39))

    # clearance_cost_map itself, as _find_wide_path calls it (no radii): one
    # obstacle cell on a 41 x 41 grid at 0.05 m, cells read along +x. The
    # dimOS grid types and the Voronoi field get minimal stand-ins (the
    # Voronoi term is not under test: a zero field keeps lethal = 100 exact).
    class _Grid:
        def __init__(self, grid, resolution, origin=None, frame_id="world", ts=0.0):
            self.grid, self.resolution, self.origin, self.frame_id, self.ts = grid, resolution, origin, frame_id, ts

    saved_pl = (rpm.OccupancyGrid, rpm.CostValues, rpm.voronoi_gradient)
    rpm.OccupancyGrid = _Grid
    rpm.CostValues = types.SimpleNamespace(FREE=0, OCCUPIED=100, UNKNOWN=-1)
    rpm.voronoi_gradient = lambda g, max_distance: _Grid(g.grid.copy(), g.resolution)
    try:
        raw = np.zeros((41, 41), dtype=np.int8)
        raw[20, 20] = 100
        binary = _Grid(raw, 0.05)

        def row(**kw):
            return [int(v) for v in rpm.clearance_cost_map(binary, **kw).grid[20, 24:27]]  # 0.20, 0.25, 0.30 m

        os.environ["ROBOT_PROFILE"] = "moss"
        r = row()
        check("MOSS clearance_cost_map(): 0.20 m lethal, 0.25 m passable (lethal 0.22 m)",
              r[0] == 100 and r[1] < 100, f"costs at 0.20/0.25/0.30 m: {r}")
        r = row(lethal_m=0.30, pivot_m=0.39)
        check("MOSS with explicit 0.30/0.39 radii -> 0.25 m lethal (explicit wins)", r[1] == 100, f"{r}")
        os.environ.pop("ROBOT_PROFILE", None)
        r = row()
        check("VECTOR clearance_cost_map(): 0.25 m lethal, 0.30 m passable (lethal 0.30 m)",
              r[1] == 100 and r[2] < 100, f"{r}")
    finally:
        rpm.OccupancyGrid, rpm.CostValues, rpm.voronoi_gradient = saved_pl
        os.environ.pop("ROBOT_PROFILE", None)

    # the soft STOP (stop_movement) vs a running contact escape
    class _Odom:
        position = types.SimpleNamespace(distance=lambda other: 0.0)   # never moves: only abort/timeout end it

    sent = []
    g = rpm.RecoveringGlobalPlanner.__new__(rpm.RecoveringGlobalPlanner)
    g._lock = threading.Lock()
    g._escape_abort = threading.Event()
    g._escaping, g._escape_id, g._held_goal = False, 0, None
    g._current_odom = _Odom()
    g._local_planner = types.SimpleNamespace(stop_planning=lambda: None,
                                             cmd_vel=types.SimpleNamespace(on_next=sent.append))
    g._position_tracker = types.SimpleNamespace(reset_data=lambda: None)
    g.escape_timeout_s = 3.0
    cancels, planned = [], []
    g.cancel_goal = lambda *a, **k: cancels.append(1)
    g.handle_goal_request = planned.append
    module = types.SimpleNamespace(_planner=g)

    def wait_idle(limit_s):
        t0 = time.monotonic()
        while g._escaping and time.monotonic() - t0 < limit_s:
            time.sleep(0.01)
        return time.monotonic() - t0

    check("bump -> escape starts", g.escape(-1.0, trigger="bump") is True)
    time.sleep(0.25)
    rpm.RecoveringGlobalPlanner.handle_goal_request(g, "goal-A")     # held: the escape owns the wheels
    n_before = len(sent)
    check("0.25 s into the escape: reversing (twists sent), a goal held",
          g._escaping and n_before >= 3 and g._held_goal == "goal-A", f"{n_before} twists")
    rpm.RecoveringPlanner._on_stop_movement(module, types.SimpleNamespace(data=True))
    dt = wait_idle(1.0)
    time.sleep(0.2)
    check("stop_movement -> the escape ends within 0.3 s (not its 3 s timeout)", not g._escaping and dt < 0.3,
          f"{dt:.2f} s")
    check("... at most one more reverse twist + the zero twist after the STOP, none 0.2 s later",
          len(sent) - n_before <= 2, f"{len(sent) - n_before} twists after STOP")
    check("... the held goal is dropped, never planned; the goal is cancelled",
          planned == [] and g._held_goal is None and len(cancels) >= 1, f"planned {planned}")
    sent.clear()
    check("a new contact after the STOP still escapes (abort flag cleared)", g.escape(-1.0, trigger="bump") is True)
    time.sleep(0.2)
    check("... and reverses", g._escaping and len(sent) >= 2, f"{len(sent)} twists")
    check("VECTOR contact semantics kept: a second contact during it aborts (returns False)",
          g.escape(+1.0, trigger="bump_rear") is False)
    check("... and the escape stops", wait_idle(1.0) < 0.3 and not g._escaping)

try:
    from vector_dimos import lidar_odometry as lom
    have_lo = True
except Exception as exc:  # noqa: BLE001
    have_lo = False
    print(f"  -- skipped (lidar_odometry does not import: {type(exc).__name__}: {exc})")
if have_lo:
    check("lidar_odometry._profile(): VECTOR at 0.37 m when unset",
          lom._profile().lidar_height_m == lom.LIDAR_HEIGHT_M == 0.37)
    os.environ["ROBOT_PROFILE"] = "moss"
    check("lidar_odometry._profile(): MOSS at 0.20 m, constant LIDAR_HEIGHT_M still 0.37",
          lom._profile().lidar_height_m == 0.20 and lom.LIDAR_HEIGHT_M == 0.37)
    check("camera pitch: VECTOR radians(6.3) == CAMERA_PITCH_RAD bit for bit",
          math.radians(rp.VECTOR.camera_pitch_deg) == lom.CAMERA_PITCH_RAD)
    os.environ.pop("ROBOT_PROFILE", None)

    # handle_depth_image on one pixel: depth 1.000 m at the principal point
    # (optical (0, 0, 1)), rover at (0, 0, 0). Base point = (cos p + cx, 0,
    # -sin p + cz): MOSS (pitch 0, cam 0.156/0/0.075) -> (1.156, 0, 0.075);
    # VECTOR (6.3 deg, 0.14/0/0.32) -> (1.1340, 0, 0.2103).
    def depth_point():
        me = types.SimpleNamespace(_depth_n=lom.DEPTH_EVERY - 1, _K=(600.0, 600.0, 0.0, 0.0),
                                   _searching=False, pose2d=(0.0, 0.0, 0.0), _pose_hist=[],
                                   camera_floor=types.SimpleNamespace(publish=lambda m: None),
                                   world_frame="world", _depth_pts_last=0, _pending_cam_pts=None)
        msg = types.SimpleNamespace(data=np.array([[1000]], dtype=np.uint16), ts=0.0)
        saved_pc = lom.PointCloud2
        lom.PointCloud2 = types.SimpleNamespace(from_numpy=lambda arr, **k: arr)
        try:
            asyncio.run(lom.LidarOdometry.handle_depth_image(me, msg))
        finally:
            lom.PointCloud2 = saved_pc
        return None if me._pending_cam_pts is None else [float(v) for v in me._pending_cam_pts[0][0]]

    os.environ["ROBOT_PROFILE"] = "moss"
    pt = depth_point()
    check("MOSS depth projection: 1 m at the principal point -> (1.156, 0, 0.075) m",
          pt is not None and all(abs(a - b) < 1e-5 for a, b in zip(pt, (1.156, 0.0, 0.075))), f"{pt}")
    os.environ.pop("ROBOT_PROFILE", None)
    pt = depth_point()
    vp = math.radians(6.3)
    want = (math.cos(vp) + 0.14, 0.0, -math.sin(vp) + 0.32)
    check("VECTOR depth projection unchanged -> (1.1340, 0, 0.2103) m",
          pt is not None and all(abs(a - b) < 1e-5 for a, b in zip(pt, want)), f"{pt}")
    # the lidar lift (world cloud z and base_link->lidar_link tf) sits inside
    # handle_pointcloud next to kiss-icp, too heavy for a cold call: pin the
    # source instead - both use sites read the ONE profile value
    seg = ast.get_source_segment((ROOT / "vector_dimos/lidar_odometry.py").read_text(),
                                 next(n for n in ast.walk(ast.parse((ROOT / "vector_dimos/lidar_odometry.py").read_text()))
                                      if isinstance(n, ast.AsyncFunctionDef) and n.name == "handle_pointcloud"))
    check("handle_pointcloud: lidar_h = _profile().lidar_height_m feeds world_pts z AND the lidar_link tf",
          "lidar_h = _profile().lidar_height_m" in seg and "np.array([x, y, lidar_h])" in seg
          and "Vector3(0.0, 0.0, lidar_h)" in seg)

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

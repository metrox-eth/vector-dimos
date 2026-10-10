"""Cold bench for tools/moss_cockpit.py: a real server on a free port, known in -> known out.

Rule #2: every check feeds a value we know and reads back a value in physical
units (metres, radians, volts, %, pixel colours, bytes on a named channel).
Groups:

  A. pure helpers   - battery gauge (3S: 10.2 V -> 0 %, 11.4 V -> 50 %, 12.6 V -> 100 %),
                      quaternion -> yaw, grid -> PNG pixel colours, the map_meta
                      pixel <-> metre formulas, even downsampling
  B. demo server    - /, /static, /state.json schema, /map.png (demo size, wall =
                      WHITE, floor = free colour), /map_meta roundtrip, /scan.json
                      (<= 1500 points, every point on a wall), /camera.mjpg framing
                      and its 5 fps cap, POST /cmd/goal|stop|episode|stack, path
                      traversal refused, bad bodies refused
  C. decoding       - messages built with dimos_lcm and fed to a live cockpit's bus
                      cache: OccupancyGrid -> exact PNG pixels, PointCloud2 -> scan
                      points (and the /pointcloud fallback), PoseStamped yaw 90 deg,
                      telemetry String -> battery %, Path, explore/goal state, a JPEG
                      image passed through byte for byte
  D. commands       - POST /cmd/* in live mode publish the known bytes on the known
                      channels (fake LCM handle); explore_ctl's generic publisher and
                      its unchanged start/stop
  E. stack + files  - run registry (dead pid skipped), main.jsonl tail, preflight.json,
                      the launcher run non-blocking on a stub script, refusal when a
                      stack runs
  G. review fixes   - launcher env without EXPLORE/MOCK/MOSS_MOCK_BUS, Stop cancels a
                      launch in its gates (SIGTERM, SIGKILL after the grace), an orphan
                      episode closed when the stack stops, a new run_id empties the bus
                      cache, a viewer that stops reading frees its camera slot
  F. hygiene        - no dimos/serial/torch import, no LAN address in the new code

C and D need dimos_lcm (pure python: on the rig, PYTHONPATH can point at a copy
of the Jetson's site-packages). Nothing here needs dimOS, lcm or hardware.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_moss_cockpit_cold.py
"""

import http.client
import json
import math
import os
import re
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import datetime as _dt
import signal
import socket
import urllib.error
import urllib.request
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import numpy as np  # noqa: E402

import moss_cockpit as mc  # noqa: E402

try:
    import dimos_lcm  # noqa: F401
    from dimos_lcm.geometry_msgs import Point, Pose, PoseStamped, Quaternion, Twist, Vector3
    from dimos_lcm.nav_msgs import MapMetaData, OccupancyGrid, Path as LcmPath
    from dimos_lcm.sensor_msgs import Image, PointCloud2, PointField
    from dimos_lcm.std_msgs import Bool, Header, String, Time
    HAVE_LCM_MSGS = True

    # Fresh nested objects every time: the generated constructors share their
    # default instances, so a default-built message mutated here would leak into
    # every later one (the cockpit's own zero twist included).
    def hdr(frame="", sec=0, nsec=0):
        return Header(seq=0, stamp=Time(sec=sec, nsec=nsec), frame_id=frame)

    def pose_of(x, y, qz=0.0, qw=1.0):
        return Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=qz, w=qw))
except ImportError:
    HAVE_LCM_MSGS = False

OK = 0
KO = 0
_TMPDIR = tempfile.TemporaryDirectory(prefix="moss_cockpit_bench_")   # state dir, run registry, page, logs
TMP = _TMPDIR.name
os.environ["MOSS_STATE_DIR"] = os.path.join(TMP, "state")
os.environ["XDG_STATE_HOME"] = os.path.join(TMP, "xdg")     # an empty dimOS run registry
os.environ.pop("DIMOS_TRANSPORT", None)
os.environ["TRANSPORT"] = "lcm"

WHITE = (0xEA, 0xF0, 0xEF)
FREE = (0x2E, 0x45, 0x56)
UNKNOWN = (0x3D, 0x57, 0x6A)
COST_MAX = (0x8F, 0xA6, 0xB5)


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def cost_colour(v):
    """The free -> COST_MAX ramp for cost v in 1..99, computed independently of the LUT."""
    return tuple(int(round(f + (t - f) * v / 99.0)) for f, t in zip(FREE, COST_MAX))


def png_decode(data: bytes) -> np.ndarray:
    """(h, w, 3) uint8 from an 8-bit RGB PNG whose rows all use filter 0 (what we write)."""
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "PNG signature"
    pos, idat = 8, b""
    w = h = None
    while pos < len(data):
        n = struct.unpack(">I", data[pos:pos + 4])[0]
        tag, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + n]
        crc = struct.unpack(">I", data[pos + 8 + n:pos + 12 + n])[0]
        assert crc == zlib.crc32(tag + body) & 0xFFFFFFFF, f"CRC of {tag!r}"
        if tag == b"IHDR":
            w, h, depth, ctype = struct.unpack(">IIBB", body[:10])
            assert (depth, ctype) == (8, 2), "8-bit RGB"
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    rows = np.frombuffer(zlib.decompress(idat), np.uint8).reshape(h, w * 3 + 1)
    assert (rows[:, 0] == 0).all(), "filter byte 0 on every row"
    return rows[:, 1:].reshape(h, w, 3)


class Client:
    def __init__(self, port):
        self.port = port
        self.base = f"http://127.0.0.1:{port}"

    def get(self, path):
        try:
            with urllib.request.urlopen(self.base + path, timeout=10) as r:
                return r.status, r.headers, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.headers, e.read()

    def json(self, path):
        code, _, body = self.get(path)
        return code, json.loads(body)

    def post(self, path, payload, raw=None, header=True):
        data = raw if raw is not None else json.dumps(payload).encode()
        hdrs = {"Content-Type": "application/json"}
        if header:
            hdrs["X-Requested-With"] = "moss-cockpit"     # what the page's post() sends
        req = urllib.request.Request(self.base + path, data=data, method="POST", headers=hdrs)
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def raw_get(self, path):
        """No client-side path normalisation (urllib keeps '..' too, this is explicit)."""
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=10)
        conn.putrequest("GET", path, skip_accept_encoding=True)
        conn.endheaders()
        r = conn.getresponse()
        body = r.read()
        conn.close()
        return r.status, body

    def mjpeg_frames(self, n, timeout=8.0):
        """Read n multipart parts; returns (content_type, [(t_arrival, jpeg)])."""
        conn = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
        conn.request("GET", "/camera.mjpg")
        r = conn.getresponse()
        ctype = r.getheader("Content-Type")
        if r.status != 200:
            body = r.read()
            conn.close()
            return r.status, ctype, body
        frames = []
        fp = r.fp
        while len(frames) < n:
            line = fp.readline()
            if not line:
                break
            if line.strip() != b"--frame":
                continue
            headers = {}
            while True:
                h = fp.readline().strip()
                if not h:
                    break
                k, v = h.split(b":", 1)
                headers[k.strip().lower()] = v.strip()
            jpeg = fp.read(int(headers[b"content-length"]))
            frames.append((time.monotonic(), jpeg))
        conn.close()
        return 200, ctype, frames


def serve(**kw):
    server = mc.make_server("127.0.0.1", 0, listen=False, **kw)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server, Client(server.server_address[1])


# ---------------------------------------------------------------------------
print("A. pure helpers")
check("11.4 V -> 50 %", mc.battery_pct(11.4) == 50, str(mc.battery_pct(11.4)))
check("10.2 V -> 0 %, 12.6 V -> 100 %", (mc.battery_pct(10.2), mc.battery_pct(12.6)) == (0, 100))
check("clamped: 9.0 V -> 0 %, 13.1 V -> 100 %", (mc.battery_pct(9.0), mc.battery_pct(13.1)) == (0, 100))
s45 = math.sin(math.pi / 4)
check("quaternion (0,0,sin45,cos45) -> yaw 1.5708 rad",
      abs(mc.yaw_of(0, 0, s45, s45) - math.pi / 2) < 1e-9, f"{mc.yaw_of(0, 0, s45, s45):.6f}")
check("quaternion (0,0,1,0) -> yaw +-pi", abs(abs(mc.yaw_of(0, 0, 1, 0)) - math.pi) < 1e-9)
cells = np.array([[-1, 0, 100], [50, 0, -1]], dtype=np.int8)    # row 0 = min y
png, meta = mc.render_grid(cells, 0.5, 1.0, 2.0, stamp=7.5, frame="world")
img = png_decode(png)
check("3x2 grid -> 3x2 PNG", img.shape == (2, 3, 3), str(img.shape))
expect = [[cost_colour(50), FREE, UNKNOWN], [UNKNOWN, FREE, WHITE]]   # PNG row 0 = max y
got = [[tuple(int(c) for c in img[r, k]) for k in range(3)] for r in range(2)]
check("pixels flipped (row 0 = max y): cost 50, free, unknown / unknown, free, WHITE",
      got == expect, str(got))
check("cost 99 -> #8FA6B5, cost 1 ~ free",
      tuple(mc.MAP_LUT[99]) == COST_MAX and tuple(mc.MAP_LUT[1]) == cost_colour(1))
check("meta: res 0.5, origin (1, 2), extent x 1..2.5 y 2..3, stamp, frame",
      (meta["res"], meta["ox"], meta["oy"], meta["x_max"], meta["y_max"], meta["stamp"], meta["frame"])
      == (0.5, 1.0, 2.0, 2.5, 3.0, 7.5, "world"), json.dumps(meta)[:120])
x_of_px = lambda px, m: m["ox"] + (px + 0.5) * m["res"]               # noqa: E731
y_of_py = lambda py, m: m["oy"] + (m["height"] - py - 0.5) * m["res"]  # noqa: E731
px_of_x = lambda x, m: (x - m["ox"]) / m["res"]                        # noqa: E731
py_of_y = lambda y, m: m["height"] - (y - m["oy"]) / m["res"]          # noqa: E731
check("formulas: lethal cell (col 2, grid row 0) centre = (2.25 m, 2.25 m) -> PNG pixel (2, 1)",
      (x_of_px(2, meta), y_of_py(1, meta)) == (2.25, 2.25)
      and tuple(int(c) for c in img[1, 2]) == WHITE)
pts = np.column_stack((np.arange(4000, dtype=np.float32), np.zeros(4000, np.float32)))
ds = mc.downsample(pts)
check("4000 points -> 1500, first and last kept, evenly spaced",
      len(ds) == 1500 and ds[0, 0] == 0 and ds[-1, 0] == 3999
      and np.ptp(np.diff(ds[:, 0])) <= 1.0, f"{len(ds)} pts")
base = mc.base_view({"fw": "qtpy-mdd3a-v3", "mode": 2, "bus_mV": 11400, "ina_ok": True,
                     "ticks": [10, 12], "target_pct": [40, 40], "output_pct": [38, 39]}, 0.25)
check("telemetry 11400 mV, mode 2 -> 11.4 V, 50 %, 'drive', driver None",
      (base["battery_v"], base["battery_pct"], base["mode_name"], base["driver"]) == (11.4, 50, "drive", None),
      json.dumps(base)[:120])
check("ina_ok False -> no battery % (the voltage is not measured)",
      mc.base_view({"bus_mV": 0, "ina_ok": False}, 0.1)["battery_pct"] is None)
check("NaN -> null in JSON (a cancelled dimOS goal is NaN)",
      json.dumps(mc.json_safe({"x": float("nan"), "y": [np.float32(1.5)]})) == '{"x": null, "y": [1.5]}')

# ---------------------------------------------------------------------------
print("B. demo server")
page_dir = os.path.join(TMP, "page")
os.makedirs(os.path.join(page_dir, "fonts"))
Path(page_dir, "index.html").write_text("<!doctype html><title>MOSS cockpit</title>bench page")
Path(page_dir, "tokens.css").write_text(":root{--moss-mint:#6CF8C7}")
Path(page_dir, "fonts", "x.ttf").write_bytes(b"\x00\x01\x00\x00")
demo, cli = serve(demo=True, page_dir=page_dir)
code, headers, body = cli.get("/")
check("GET / -> the page's index.html", code == 200 and b"bench page" in body
      and headers["Content-Type"].startswith("text/html") and headers["Cache-Control"] == "no-store")
code, headers, body = cli.get("/static/tokens.css")
check("GET /static/tokens.css -> text/css", code == 200 and headers["Content-Type"].startswith("text/css")
      and b"6CF8C7" in body)
code, headers, _ = cli.get("/static/fonts/x.ttf")
check("GET /static/fonts/x.ttf -> font/ttf", code == 200 and headers["Content-Type"] == "font/ttf")
for bad in ("/static/../moss_cockpit.py", "/static/%2e%2e/moss_cockpit.py", "/static//etc/passwd",
            "/static/..%2f..%2fREADME.md"):
    code, body = cli.raw_get(bad)
    check(f"path traversal {bad} -> 404", code == 404 and b"import" not in body, str(code))
code, _, _ = cli.get("/nothing")
check("unknown GET -> 404", code == 404)

code, st = cli.json("/state.json")
need = {"ts", "mode", "stack", "pose", "topics", "base", "nav", "recording", "system"}
check("state.json top-level keys", code == 200 and need <= set(st), str(sorted(set(st))))
check("mode demo, stack running moss-explore", st["mode"] == "demo" and st["stack"]["running"]
      and st["stack"]["blueprint"] == "moss-explore")
check("topics carry age_s + hz for odom/global_costmap/lidar/path/cmd_vel/base_telemetry/color_image",
      all(k in st["topics"] and {"age_s", "hz"} <= set(st["topics"][k])
          for k in ("odom", "global_costmap", "lidar", "path", "cmd_vel", "base_telemetry", "color_image")))
b = st["base"]
check("base: battery_v / battery_pct consistent with the 3S gauge",
      b and abs(b["battery_pct"] - mc.battery_pct(b["battery_v"])) <= 1 and 10.9 <= b["battery_v"] <= 11.6,
      f"{b and b['battery_v']} V {b and b['battery_pct']} %")
check("nav keys", {"exploring", "explore_done", "goal", "goal_reached", "last_cmd_vel"} <= set(st["nav"]))
check("recording keys", {"db", "exists", "size_mb", "episode"} <= set(st["recording"]))
sysm = st["system"]
check("system: real cpu/ram/disks/load/uptime", all(k in sysm for k in (
    "cpu_percent", "cpu_per_core", "ram_used_gb", "ram_total_gb", "ram_percent", "temps", "disks",
    "load_1m", "uptime_s")) and sysm["ram_total_gb"] > 0 and isinstance(sysm["disks"], list),
    f"ram {sysm.get('ram_total_gb')} GB, {len(sysm.get('disks', []))} disks")
p0 = st["pose"]
check("demo pose starts on the loop's first corner (-4.25, -4.25)",
      abs(p0["y"] + 4.25) < 0.05 and -4.3 < p0["x"] < -3.5, f"{p0}")

code, headers, png = cli.get("/map.png")
img = png_decode(png)
check("/map.png: valid PNG, demo size 240x240", code == 200 and img.shape == (240, 240, 3)
      and headers["Content-Type"] == "image/png", str(img.shape))
code, meta = cli.json("/map_meta")
check("/map_meta: res 0.05, origin (-6, -6), 240x240, stamp + frame",
      (meta["res"], meta["ox"], meta["oy"], meta["width"], meta["height"], meta["frame"])
      == (0.05, -6.0, -6.0, 240, 240, "world") and meta["stamp"] > 0)
rt = px_of_x(x_of_px(10, meta), meta)     # x_of_px = the pixel's centre; px_of_x is continuous
check("px_of_x(x_of_px(10)) == 10 (pixel index = floor)", math.floor(rt) == 10, f"{rt}")
check("py_of_y(y_of_py(10)) == 10 (pixel index = floor)", math.floor(py_of_y(y_of_py(10, meta), meta)) == 10)


def pixel_at(x, y):
    return tuple(int(c) for c in img[int(py_of_y(y, meta)), int(px_of_x(x, meta))])


check("wall at (-5.5, -4.25) -> WHITE", pixel_at(-5.5, -4.25) == WHITE, str(pixel_at(-5.5, -4.25)))
check("floor at (-4.25, -4.25) (1.15 m from any wall) -> free colour", pixel_at(-4.25, -4.25) == FREE,
      str(pixel_at(-4.25, -4.25)))
check("outside the walls (-5.9, 5.9) -> unknown", pixel_at(-5.9, 5.9) == UNKNOWN)
check("next to the wall (-5.35, -4.25) -> a cost colour between free and #8FA6B5",
      pixel_at(-5.35, -4.25) not in (FREE, WHITE, UNKNOWN))

code, scan = cli.json("/scan.json")
pts = scan["points"]
truth = demo.cockpit.world.truth


def on_wall(x, y):
    c = int(math.floor((x + 6.0) / 0.05))
    r = int(math.floor((y + 6.0) / 0.05))
    return 0 <= c < 240 and 0 <= r < 240 and truth[r, c] == 100


check("/scan.json: world frame, 1..1500 points", scan["frame"] == "world" and 0 < len(pts) <= 1500,
      f"{len(pts)} pts")
check("every scan point lies on a wall cell of the demo map", all(on_wall(x, y) for x, y in pts))
wall_below = [p for p in pts if abs(p[0] - p0["x"]) < 0.2 and p[1] < p0["y"]]
check("the ray straight down from the pose hits the outer wall at y ~ -5.4",
      bool(wall_below) and abs(max(p[1] for p in wall_below) + 5.4) < 0.06,
      f"{max(p[1] for p in wall_below) if wall_below else None}")

code, ctype, frames = cli.mjpeg_frames(4)
check("/camera.mjpg: multipart/x-mixed-replace; boundary=frame",
      code == 200 and ctype == "multipart/x-mixed-replace; boundary=frame", str(ctype))
check("first part is a JPEG (SOI ff d8 ... EOI ff d9)",
      bool(frames) and frames[0][1][:2] == b"\xff\xd8" and frames[0][1][-2:] == b"\xff\xd9")
if len(frames) == 4:
    span = frames[-1][0] - frames[0][0]
    check("4 frames take >= 0.55 s (5 fps cap)", span >= 0.55, f"{span:.2f} s for 3 intervals")
else:
    check("4 frames received", False, f"{len(frames)}")


def clients_drop_to_zero(cockpit, within_s=2.0):
    deadline = time.monotonic() + within_s
    while cockpit.camera_clients and time.monotonic() < deadline:
        time.sleep(0.05)
    return cockpit.camera_clients == 0


check("viewer gone -> camera client count back to 0", clients_drop_to_zero(demo.cockpit),
      str(demo.cockpit.camera_clients))

code, ans = cli.post("/cmd/goal", {"x": 1.0, "y": -4.25}, header=False)
_, st = cli.json("/state.json")
check("#16 POST /cmd/goal without 'X-Requested-With: moss-cockpit' -> 403, demo goal untouched",
      code == 403 and ans["ok"] is False and "X-Requested-With" in ans["error"] and st["nav"]["goal"] is None
      and st["nav"]["exploring"] is True, f"{code} {json.dumps(ans)[:100]}")
code, ans = cli.post("/cmd/goal", {"x": 1.0, "y": -4.25})
check("POST /cmd/goal (1.0, -4.25) -> ok, sent goal_request",
      code == 200 and ans["ok"] and ans["sent"] == "/goal_request#geometry_msgs.PoseStamped", json.dumps(ans))
_, st = cli.json("/state.json")
check("state.nav.goal = (1.0, -4.25), goal_reached null, exploring stopped",
      st["nav"]["goal"] == {"x": 1.0, "y": -4.25} and st["nav"]["goal_reached"] is None
      and st["nav"]["exploring"] is False, json.dumps(st["nav"]))
x_before = st["pose"]["x"]
time.sleep(1.0)
_, st = cli.json("/state.json")
dx = st["pose"]["x"] - x_before
check("the demo pose drives toward the goal at ~0.2 m/s", 0.1 < dx < 0.35, f"dx {dx:+.3f} m in ~1 s")
code, path = cli.json("/path.json")
check("/path.json: pose -> goal", code == 200 and path["points"] and path["points"][-1] == [1.0, -4.25],
      json.dumps(path)[:100])
for body, why in (({"x": "a", "y": 0}, "x not a number"), ({"x": 1e6, "y": 0}, "beyond 500 m"),
                  ({"y": 0}, "x missing")):
    code, ans = cli.post("/cmd/goal", body)
    check(f"goal refused ({why}) -> 400", code == 400 and ans["ok"] is False, ans.get("error", ""))
code, ans = cli.post("/cmd/goal", None, raw=b"{not json")
check("unreadable JSON body -> 400", code == 400 and ans["ok"] is False)
code, ans = cli.post("/cmd/fly", {})
check("unknown command -> 404", code == 404 and ans["ok"] is False)
code, ans = cli.post("/cmd/stop", {})
check("POST /cmd/stop -> ok, three topics named", code == 200 and ans["ok"]
      and len([ans["sent"]] + ans["also"]) == 3, json.dumps(ans))
_, st = cli.json("/state.json")
time.sleep(0.4)
_, st2 = cli.json("/state.json")
check("after the soft stop the demo pose holds still",
      abs(st2["pose"]["x"] - st["pose"]["x"]) < 1e-6 and st2["nav"]["goal"] is not None)
code, ans = cli.post("/cmd/episode", {"action": "start", "name": "ep3"})
_, st = cli.json("/state.json")
check("episode start ep3 -> state.recording.episode.name", code == 200 and ans["ok"]
      and st["recording"]["episode"]["name"] == "ep3", json.dumps(st["recording"]["episode"]))
code, ans = cli.post("/cmd/episode", {"action": "start", "name": "../x"})
check("episode name with a slash -> 400", code == 400)
cli.post("/cmd/episode", {"action": "stop"})
_, st = cli.json("/state.json")
check("episode stop -> null", st["recording"]["episode"] is None)
code, ans = cli.post("/cmd/stack", {"action": "map"})
check("demo: stack map while running -> refused", code == 200 and ans["ok"] is False, ans.get("error", ""))
cli.post("/cmd/stack", {"action": "stop"})
_, st = cli.json("/state.json")
check("demo: stack stop -> running false, pose null", st["stack"] == {"running": False} and st["pose"] is None)
code, ans = cli.post("/cmd/stack", {"action": "map"})
_, st = cli.json("/state.json")
check("demo: stack map -> moss-nav running", ans["ok"] and st["stack"]["blueprint"] == "moss-nav")
code, lg = cli.json("/log.json?n=2")
check("/log.json?n=2 -> 2 records {ts, level, event, logger}", code == 200 and len(lg["lines"]) == 2
      and set(lg["lines"][0]) == {"ts", "level", "event", "logger"})
code, pf = cli.json("/preflight.json")
check("/preflight.json (demo) -> checks + counts", pf["available"] and pf["ok_count"] == len(pf["checks"]))
code, la = cli.json("/launch.json")
check("/launch.json -> running flag + lines", code == 200 and "running" in la and isinstance(la["lines"], list))
demo.shutdown()

page_none = os.path.join(TMP, "no_page")
os.makedirs(page_none)
srv, c2 = serve(demo=True, page_dir=page_none)
code, _, body = c2.get("/")
check("no index.html -> one-line placeholder page, 200", code == 200 and b"not installed" in body)
srv.shutdown()

# ---------------------------------------------------------------------------
print("C. decoding (dimos_lcm messages fed to a live cockpit)")
live = None
if not HAVE_LCM_MSGS or not mc.HAVE_MSGS:
    print("  -- skipped, no dimos_lcm here: run with the Jetson site-packages on PYTHONPATH, or on the Jetson")
else:
    RUNS = Path(os.environ["XDG_STATE_HOME"], "dimos", "runs")

    def register(run_id, blueprint, pid=None):
        """A live dimOS registry entry (our own pid: alive), or none with run_id None."""
        RUNS.mkdir(parents=True, exist_ok=True)
        f = RUNS / "bench.json"
        if run_id is None:
            f.unlink(missing_ok=True)
            return
        f.write_text(json.dumps({"run_id": run_id, "pid": pid or os.getpid(), "blueprint": blueprint,
                                 "started_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                                 "log_dir": "/nowhere"}))

    real_run_pid = mc._dimos_run_pid
    mc._dimos_run_pid = lambda: None          # this rig's own processes never count as a stack
    register("run-a", "vector-dimos.moss-explore")
    live, lcli = serve(demo=False, page_dir=page_dir)
    bus = live.cockpit.bus
    code, st = lcli.json("/state.json")
    check("live, empty bus: pose null, base null, topics never seen", code == 200 and st["mode"] == "live"
          and st["pose"] is None and st["base"] is None and st["topics"]["odom"]["age_s"] is None)
    code, _, _ = lcli.get("/map.png")
    check("no costmap yet -> /map.png 404", code == 404)

    g = OccupancyGrid(data_length=6, header=hdr("world", 1760000000, 500000000),
                      info=MapMetaData(map_load_time=Time(), resolution=0.5, width=3, height=2,
                                       origin=pose_of(1.0, 2.0)),
                      data=[-1, 0, 100, 50, 0, -1])
    bus.ingest(mc.CHANNELS["global_costmap"], g.lcm_encode())
    code, _, png = lcli.get("/map.png")
    img = png_decode(png)
    got = [[tuple(int(c) for c in img[r, k]) for k in range(3)] for r in range(2)]
    check("OccupancyGrid 3x2 res 0.5 origin (1,2) -> PNG 3x2", img.shape == (2, 3, 3))
    check("exact pixels: [cost50, free, unknown] / [unknown, free, WHITE]", got == expect, str(got))
    code, meta = lcli.json("/map_meta")
    check("map_meta from the message: res 0.5, ox 1, oy 2, stamp 1760000000.5, frame world",
          (meta["res"], meta["ox"], meta["oy"], meta["stamp"], meta["frame"])
          == (0.5, 1.0, 2.0, 1760000000.5, "world"), json.dumps(meta)[:120])

    def cloud(points, frame):
        m = PointCloud2(header=hdr(frame))
        m.height, m.width = 1, len(points)
        fields = []
        for i, name in enumerate(("x", "y", "z", "intensity")):
            f = PointField()
            f.name, f.offset, f.datatype, f.count = name, 4 * i, 7, 1
            fields.append(f)
        m.fields, m.fields_length = fields, 4
        m.point_step, m.row_step, m.is_bigendian, m.is_dense = 16, 16 * len(points), False, True
        m.data = np.array([[x, y, z, 0.0] for x, y, z in points], dtype="<f4").tobytes()
        m.data_length = len(m.data)
        return m.lcm_encode()

    bus.ingest(mc.CHANNELS["pointcloud"], cloud([(0.5, 0.25, 0.0)], "lidar_link"))
    code, scan = lcli.json("/scan.json")
    check("/lidar never seen -> /pointcloud fallback, lidar frame, (0.5, 0.25)",
          scan["frame"] == "lidar_link" and scan["source"] == "pointcloud" and scan["points"] == [[0.5, 0.25]],
          json.dumps(scan)[:120])
    bus.ingest(mc.CHANNELS["lidar"], cloud([(1.0, 2.0, 0.37), (3.0, -4.0, 0.37)], "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    check("PointCloud2 with 2 points on /lidar -> [[1, 2], [3, -4]] world",
          scan["frame"] == "world" and scan["points"] == [[1.0, 2.0], [3.0, -4.0]], json.dumps(scan)[:120])
    big = [(math.cos(a), math.sin(a), 0.0) for a in np.linspace(0, 2 * math.pi, 3000)]
    bus.ingest(mc.CHANNELS["lidar"], cloud(big, "world"))
    time.sleep(mc.SCAN_MIN_INTERVAL_S + 0.05)
    code, scan = lcli.json("/scan.json")
    radii = [math.hypot(x, y) for x, y in scan["points"]]
    check("3000-point ring of radius 1 m -> 1500 points, all at 1 m",
          len(scan["points"]) == 1500 and max(abs(r - 1.0) for r in radii) < 2e-3, f"{len(scan['points'])}")

    pose = PoseStamped(header=hdr("world"), pose=pose_of(1.25, -0.5, qz=s45, qw=s45))
    for _ in range(3):
        bus.ingest(mc.CHANNELS["odom"], pose.lcm_encode())
        time.sleep(0.1)
    tele = {"ts": time.time(), "fw": "qtpy-mdd3a-v3", "driver": None, "mode": 0, "reason": "boot",
            "bus_mV": 11400, "ina_ok": True, "ticks": [5, 6], "target_pct": [0, 0], "output_pct": [0, 0],
            "ramp_ms": [5, 5], "firmware_age_s": 0.2, "mock": False}
    bus.ingest(mc.CHANNELS["base_telemetry"], String(data=json.dumps(tele)).lcm_encode())
    code, st = lcli.json("/state.json")
    check("PoseStamped (1.25, -0.5), quaternion yaw 90 deg -> pose yaw 1.5708",
          st["pose"]["x"] == 1.25 and st["pose"]["y"] == -0.5 and abs(st["pose"]["yaw"] - 1.5708) < 1e-4,
          json.dumps(st["pose"]))
    check("odom 3 messages 0.1 s apart -> ~10 Hz", 7.0 <= st["topics"]["odom"]["hz"] <= 11.0,
          f"{st['topics']['odom']['hz']} Hz")
    b = st["base"]
    check("String telemetry 11400 mV -> battery 11.4 V, 50 %, mode off, fw, driver null",
          (b["battery_v"], b["battery_pct"], b["mode_name"], b["fw"], b["driver"])
          == (11.4, 50, "off", "qtpy-mdd3a-v3", None), json.dumps(b)[:160])

    poses = [PoseStamped(header=hdr("world"), pose=pose_of(x, y))
             for x, y in ((0.0, 0.0), (0.5, 0.25), (1.0, 1.0))]
    path = LcmPath(poses_length=3, header=hdr("world"), poses=poses)
    bus.ingest(mc.CHANNELS["path"], path.lcm_encode())
    code, pj = lcli.json("/path.json")
    check("Path of 3 poses -> [[0,0],[0.5,0.25],[1,1]]", pj["points"] == [[0.0, 0.0], [0.5, 0.25], [1.0, 1.0]],
          json.dumps(pj))

    t0 = 100.0
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0)
    code, st = lcli.json("/state.json")
    check("explore_cmd True -> exploring", st["nav"]["exploring"] is True)
    bus.ingest(mc.CHANNELS["stop_explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 1)
    code, st = lcli.json("/state.json")
    check("then stop_explore_cmd -> not exploring", st["nav"]["exploring"] is False)
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 2)
    bus.ingest(mc.CHANNELS["explore_done"], Bool(data=True).lcm_encode(), mono=t0 + 3)
    code, st = lcli.json("/state.json")
    check("start again, then explore_done -> not exploring, explore_done true",
          st["nav"]["exploring"] is False and st["nav"]["explore_done"] is True, json.dumps(st["nav"]))
    goal = PoseStamped(header=hdr("world"), pose=pose_of(2.0, 3.5))
    bus.ingest(mc.CHANNELS["goal_request"], goal.lcm_encode(), mono=t0 + 4)
    bus.ingest(mc.CHANNELS["goal_reached"], Bool(data=True).lcm_encode(), mono=t0 + 3.5)
    code, st = lcli.json("/state.json")
    check("goal_request (2, 3.5) -> nav.goal; an older goal_reached does not count",
          st["nav"]["goal"] == {"x": 2.0, "y": 3.5} and st["nav"]["goal_reached"] is None)
    bus.ingest(mc.CHANNELS["goal_reached"], Bool(data=True).lcm_encode(), mono=t0 + 5)
    code, st = lcli.json("/state.json")
    check("goal_reached after it -> true", st["nav"]["goal_reached"] is True)
    nan_goal = PoseStamped(header=hdr("world"), pose=pose_of(float("nan"), float("nan")))
    bus.ingest(mc.CHANNELS["goal_request"], nan_goal.lcm_encode(), mono=t0 + 6)
    code, st = lcli.json("/state.json")
    check("a NaN goal (dimOS cancel) -> goal null, still valid JSON", code == 200 and st["nav"]["goal"] is None)
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 7)
    exp_a = lcli.json("/state.json")[1]["nav"]["exploring"]
    bus.ingest(mc.CHANNELS["stop_movement"], Bool(data=True).lcm_encode(), mono=t0 + 8)
    exp_b = lcli.json("/state.json")[1]["nav"]["exploring"]
    bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 9)
    exp_c = lcli.json("/state.json")[1]["nav"]["exploring"]
    check("#11 explore_cmd -> exploring; a newer stop_movement (pad touch) -> not; a newer explore_cmd -> again",
          (exp_a, exp_b, exp_c) == (True, False, True), str((exp_a, exp_b, exp_c)))
    tw = Twist(linear=Vector3(x=0.15, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.3))
    bus.ingest(mc.CHANNELS["cmd_vel"], tw.lcm_encode())
    code, st = lcli.json("/state.json")
    check("cmd_vel (0.15 m/s, -0.3 rad/s) -> last_cmd_vel", (st["nav"]["last_cmd_vel"]["vx"],
          st["nav"]["last_cmd_vel"]["wz"]) == (0.15, -0.3))
    marker = {"action": "start", "name": "bus-ep", "ts": 1760000001.0}
    bus.ingest(mc.CHANNELS["episode_marker"], String(data=json.dumps(marker)).lcm_encode())
    code, st = lcli.json("/state.json")
    check("an episode_marker from anyone -> state.recording.episode",
          st["recording"]["episode"] == {"name": "bus-ep", "started_at": 1760000001.0})

    def feed_image(enc, data, w, h):
        im = Image(header=hdr("camera"))
        im.width, im.height, im.encoding = w, h, enc
        im.step = 0 if enc == "jpeg" else w * 3
        im.data, im.data_length = data, len(data)

        def pump():
            for _ in range(30):
                bus.ingest(mc.IMAGE_CHANNEL, im.lcm_encode())
                time.sleep(0.05)
        threading.Thread(target=pump, daemon=True).start()

    feed_image("jpeg", mc.DEMO_JPEG, 320, 240)
    code, ctype, frames = lcli.mjpeg_frames(2)
    check("JPEG on the bus (JpegLcmTransport) -> MJPEG part byte for byte",
          code == 200 and frames and frames[0][1] == mc.DEMO_JPEG, f"{len(frames)} frames")
    time.sleep(0.3)
    try:
        import cv2  # noqa: F401
        have_enc = True
    except ImportError:
        try:
            import turbojpeg  # noqa: F401
            have_enc = True
        except ImportError:
            have_enc = False
    if have_enc:
        raw = np.zeros((8, 8, 3), np.uint8)
        raw[:, :, 0] = 200
        feed_image("rgb8", raw.tobytes(), 8, 8)
        code, ctype, frames = lcli.mjpeg_frames(1)
        ok_jpeg = code == 200 and frames and frames[0][1][:2] == b"\xff\xd8"
        detail = ""
        if ok_jpeg:
            import cv2
            dec = cv2.imdecode(np.frombuffer(frames[0][1], np.uint8), cv2.IMREAD_COLOR)
            detail = f"decoded BGR {dec[4, 4].tolist()}"
            ok_jpeg = abs(int(dec[4, 4, 2]) - 200) <= 8 and int(dec[4, 4, 0]) <= 8   # red stays red
        check("raw rgb8 8x8 red frame -> encoded JPEG, still red", ok_jpeg, detail)
    else:
        print("  -- skipped, no cv2/turbojpeg here: raw-frame encoding")
    time.sleep(1.6)   # the pump is over: the camera is silent, nothing left to write
    check("viewers gone while the camera is silent -> 0 clients (camera unsubscribed)",
          clients_drop_to_zero(live.cockpit), str(live.cockpit.camera_clients))

# ---------------------------------------------------------------------------
print("D. commands on the bus (fake LCM handle)")
TRUE_BYTES = b"\x1e\xbe\xf0jP\x95\x0e>\x01"   # std_msgs.Bool fingerprint + data=True
if live is None:
    print("  -- skipped, no dimos_lcm here: run with the Jetson site-packages on PYTHONPATH, or on the Jetson")
else:
    sent = []

    class FakeLCM:
        def publish(self, channel, payload):
            sent.append((channel, bytes(payload)))

    fake_mod = types.SimpleNamespace(LCM=FakeLCM)
    real_lcm = mc.lcm
    real_sys_lcm = sys.modules.get("lcm")
    mc.lcm = fake_mod
    sys.modules["lcm"] = fake_mod            # explore_ctl imports it when no handle is passed
    live.cockpit._pub_lcm = None
    try:
        import explore_ctl
        check("explore_ctl.topic unchanged: start/stop",
              (explore_ctl.topic("start"), explore_ctl.topic("stop"))
              == ("/explore_cmd#std_msgs.Bool", "/stop_explore_cmd#std_msgs.Bool"))
        used = explore_ctl.publish("stop")
        check("explore_ctl.publish('stop') -> Bool(True) bytes on /stop_explore_cmd",
              used == "/stop_explore_cmd#std_msgs.Bool" and sent == [(used, TRUE_BYTES)], str(sent))
        os.environ["TRANSPORT"] = "zenoh"
        check("generic channel on zenoh -> dimos/goal_request/geometry_msgs.PoseStamped",
              explore_ctl.channel("goal_request", "geometry_msgs.PoseStamped")
              == "dimos/goal_request/geometry_msgs.PoseStamped")
        os.environ["TRANSPORT"] = "lcm"
        sent.clear()

        code, ans = lcli.post("/cmd/explore", {"action": "start"})
        check("POST explore start -> /explore_cmd Bool(True)", ans["ok"]
              and sent == [("/explore_cmd#std_msgs.Bool", TRUE_BYTES)], str(sent))
        sent.clear()
        code, ans = lcli.post("/cmd/explore", {"action": "start"}, header=False)
        check("#16 POST explore start without the cockpit header -> 403, nothing published",
              code == 403 and ans["ok"] is False and sent == [], f"{code} {sent}")
        queued = []
        real_later = mc._later
        mc._later = lambda delay, fn: queued.append((delay, fn))
        GOAL_CH = "/goal_request#geometry_msgs.PoseStamped"
        STOP_CH = "/stop_explore_cmd#std_msgs.Bool"
        # not exploring (the bench bus says explore_cmd at t0+9, stop_explore_cmd at t0+10)
        bus.ingest(mc.CHANNELS["stop_explore_cmd"], Bool(data=True).lcm_encode(), mono=t0 + 10)
        sent.clear()
        code, ans = lcli.post("/cmd/goal", {"x": 1.5, "y": 0.5})
        g0 = PoseStamped.lcm_decode(sent[0][1]) if sent else None
        check("#2/#13 goal while NOT exploring -> goal_request (1.5, 0.5) at once, no stop_explore_cmd, nothing queued",
              [c for c, _ in sent] == [GOAL_CH] and queued == [] and ans.get("queued_s") is None
              and (g0.pose.position.x, g0.pose.position.y) == (1.5, 0.5), f"{[c for c, _ in sent]} {ans}")
        # exploring: stop now, goal 3.0 s later
        bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode())
        sent.clear()
        code, ans = lcli.post("/cmd/goal", {"x": 2.5, "y": -1.0})
        chans = [c for c, _ in sent]
        check("#3/#14 goal while exploring -> stop_explore_cmd now, answer queued_s 3.0, goal not yet sent",
              code == 200 and ans["ok"] and chans == [STOP_CH] and ans["sent"] == STOP_CH
              and ans["queued_s"] == 3.0 and ans["queued"] == GOAL_CH, f"{chans} {ans}")
        check("#14 the goal is handed to a timer of 3.0 s", [d for d, _ in queued] == [3.0], str(queued))
        sent.clear()
        if queued:
            queued.pop()[1]()                     # the 3 s are up
        chans = [c for c, _ in sent]
        check("#14 when the timer fires -> goal_request only", chans == [GOAL_CH], str(chans))
        g = PoseStamped.lcm_decode(sent[-1][1]) if sent else None
        check("goal bytes decode to (2.5, -1.0, 0) frame world, orientation (0,0,0,1), stamped now",
              g is not None and (g.pose.position.x, g.pose.position.y, g.pose.position.z) == (2.5, -1.0, 0.0)
              and g.header.frame_id == "world"
              and (g.pose.orientation.x, g.pose.orientation.y, g.pose.orientation.z, g.pose.orientation.w)
              == (0.0, 0.0, 0.0, 1.0) and abs(g.header.stamp.sec - time.time()) < 5)
        sent.clear()
        lcli.post("/cmd/goal", {"x": 4.0, "y": 4.0})          # still exploring: queued again
        lcli.post("/cmd/stop", {})                             # the operator stops inside the 3 s
        sent.clear()
        if queued:
            queued.pop()[1]()
        check("#14 a STOP inside the 3 s drops the queued goal: nothing published when the timer fires",
              sent == [], str(sent))
        mc._later = real_later
        register(None, None)                      # the stop check below runs with no stack at all
        sent.clear()
        polluted = Twist()                       # what a careless caller does to the shared default
        polluted.linear.x = 9.9
        code, ans = lcli.post("/cmd/stop", {})
        polluted.linear.x = 0.0
        chans = [c for c, _ in sent]
        check("POST stop, no stack -> stop_explore, stop_movement, tele_cmd_vel, cmd_vel",
              chans == ["/stop_explore_cmd#std_msgs.Bool", "/stop_movement#std_msgs.Bool",
                        "/tele_cmd_vel#geometry_msgs.Twist", "/cmd_vel#geometry_msgs.Twist"], str(chans))
        zero = Twist.lcm_decode(sent[2][1]) if len(sent) > 2 else None
        check("the twist is all zeros (even with dimos_lcm's shared default Vector3 dirtied)", zero is not None and (zero.linear.x, zero.linear.y, zero.linear.z,
              zero.angular.x, zero.angular.y, zero.angular.z) == (0.0,) * 6)
        check("stop_movement payload = Bool(True)", len(sent) > 1 and sent[1][1] == TRUE_BYTES)
        register("run-a", "vector-dimos.moss-explore")
        sent.clear()
        code, ans = lcli.post("/cmd/episode", {"action": "start", "name": "ep7"})
        m = json.loads(String.lcm_decode(sent[0][1]).data) if sent else {}
        check("POST episode start ep7 -> /episode_marker String {action, name, ts}",
              sent and sent[0][0] == "/episode_marker#std_msgs.String" and m.get("action") == "start"
              and m.get("name") == "ep7" and abs(m.get("ts", 0) - time.time()) < 5, json.dumps(m))
        _, st = lcli.json("/state.json")
        check("state.recording.episode = ep7", (st["recording"]["episode"] or {}).get("name") == "ep7")
        sent.clear()
        code, ans = lcli.post("/cmd/explore", {"action": "go"})
        check("explore with a bad action -> 400, nothing published", code == 400 and sent == [])
        # VECTOR's explore stack on the same computer and bus: hands off
        register("run-v", "vector-dimos.explore")
        sent.clear()
        c1, a1 = lcli.post("/cmd/explore", {"action": "start"})
        c2, a2 = lcli.post("/cmd/goal", {"x": 1.0, "y": 1.0})
        check("#12 VECTOR stack (blueprint 'explore') -> explore start and goal refused 409, nothing published",
              (c1, c2) == (409, 409) and not a1["ok"] and not a2["ok"] and sent == [], f"{c1} {c2} {sent}")
        stub_ok = Path(TMP, "stub_never.sh")
        stub_ok.write_text("echo launched > " + str(Path(TMP, "launched.txt")) + "\n")
        live.cockpit.launcher.script = str(stub_ok)           # never the real launcher, even on a KO
        c3_, a3 = lcli.post("/cmd/stack", {"action": "explore"})
        time.sleep(0.3)
        check("#12 VECTOR stack -> stack launch refused 409 (not MOSS), the launcher never ran",
              c3_ == 409 and not a3["ok"] and "not MOSS" in a3["error"]
              and not Path(TMP, "launched.txt").exists(), f"{c3_} {a3}")
        sent.clear()
        c4, a4 = lcli.post("/cmd/stop", {})
        check("#12 VECTOR stack -> the soft STOP is still allowed (published)", c4 == 200 and a4["ok"]
              and "/stop_movement#std_msgs.Bool" in [c for c, _ in sent], str([c for c, _ in sent]))
        register(None, None)
    finally:
        mc._dimos_run_pid = real_run_pid
        mc.lcm = real_lcm
        if real_sys_lcm is None:
            sys.modules.pop("lcm", None)
        else:
            sys.modules["lcm"] = real_sys_lcm

# ---------------------------------------------------------------------------
print("E. stack, logs, flight check, launcher")
runs = Path(os.environ["XDG_STATE_HOME"], "dimos", "runs")
runs.mkdir(parents=True, exist_ok=True)
logdir = Path(TMP, "logs", "run1")
logdir.mkdir(parents=True)
dead = subprocess.Popen(["true"])
dead.wait()
started = time.time() - 120.0
iso = _dt.datetime.fromtimestamp(started, _dt.timezone.utc).isoformat()
Path(runs, "old.json").write_text(json.dumps({"run_id": "old", "pid": dead.pid, "blueprint": "vector-dimos.moss-explore",
                                              "started_at": iso, "log_dir": "/nowhere"}))
check("registry with only a dead pid -> no stack", mc.stack_status() == {"running": False})
Path(runs, "live.json").write_text(json.dumps({"run_id": "20261011-120000-vector-dimos-moss-nav", "pid": os.getpid(),
                                               "blueprint": "vector-dimos.moss-nav", "started_at": iso,
                                               "log_dir": str(logdir)}))
s = mc.stack_status()
check("live pid -> running, blueprint moss-nav, uptime 120 s",
      s["running"] and s["blueprint"] == "moss-nav" and s["pid"] == os.getpid()
      and abs(s["uptime_s"] - 120.0) < 2.0, json.dumps(s))
recs = [{"timestamp": f"2026-10-11T12:00:0{i}Z", "level": "info", "event": f"line {i}", "logger": "x.py",
         "func_name": "f"} for i in range(3)]
Path(logdir, "main.jsonl").write_text("\n".join(json.dumps(r) for r in recs) + "\nnot json\n")
check("main.jsonl tail n=2 (after a non-JSON last line) -> the last two records, four keys",
      mc.log_tail(str(logdir), 2) == [{"ts": "2026-10-11T12:00:01Z", "level": "info", "event": "line 1",
                                       "logger": "x.py"}, {"ts": "2026-10-11T12:00:02Z", "level": "info",
                                                           "event": "line 2", "logger": "x.py"}],
      json.dumps(mc.log_tail(str(logdir), 2))[:160])
check("no preflight.json -> available false", mc.preflight_doc() == {"available": False})
os.makedirs(os.environ["MOSS_STATE_DIR"], exist_ok=True)
Path(os.environ["MOSS_STATE_DIR"], "preflight.json").write_text(json.dumps(
    {"ts": 1.0, "checks": [{"ok": True, "label": "stack", "detail": "none", "section": "stack"}],
     "ok_count": 1, "ko_count": 0, "summary": "1/1 OK - ready to fly"}))
pf = mc.preflight_doc()
check("saved preflight.json -> served with available true", pf["available"] and pf["summary"] == "1/1 OK - ready to fly")

stub = Path(TMP, "fly_stub.sh")
stub.write_text("echo \"stub $1 start\"\nsleep 1\necho \"stub $1 end\"\n")
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None
srv, c3 = serve(demo=False, page_dir=page_dir, fly_script=str(stub))
code, st = c3.json("/state.json")
check("state.json stack from the registry", st["stack"]["running"] and st["stack"]["blueprint"] == "moss-nav")
code, lg = c3.json("/log.json?n=1")
check("/log.json?n=1 -> the stack's last record", lg["lines"] == [{"ts": "2026-10-11T12:00:02Z", "level": "info",
                                                                   "event": "line 2", "logger": "x.py"}])
code, ans = c3.post("/cmd/stack", {"action": "explore"})
check("stack explore while a stack runs -> refused", ans["ok"] is False and "already running" in ans["error"])
Path(runs, "live.json").unlink()
t = time.monotonic()
code, ans = c3.post("/cmd/stack", {"action": "check"})
elapsed = time.monotonic() - t
check("stack check -> ok at once (non-blocking: the stub sleeps 1 s)", ans["ok"] and elapsed < 0.5,
      f"{elapsed:.2f} s")
code, la = c3.json("/launch.json")
check("/launch.json while it runs -> running true", la["running"] is True and la["action"] == "check")
code, ans = c3.post("/cmd/stack", {"action": "map"})
check("a second launch while the first runs -> refused", ans["ok"] is False)
time.sleep(1.5)
code, la = c3.json("/launch.json")
check("after it ends -> running false, rc 0, output in last_launch.log",
      la["running"] is False and la["returncode"] == 0 and "stub check end" in la["lines"]
      and os.path.isfile(os.path.join(os.environ["MOSS_STATE_DIR"], "last_launch.log")), json.dumps(la)[:200])
code, ans = c3.post("/cmd/stack", {"action": "dance"})
check("stack with an unknown action -> 400", code == 400)
code, pf = c3.json("/preflight.json")
check("/preflight.json (live) -> the saved document", pf.get("ok_count") == 1)
srv.shutdown()
mc._dimos_run_pid = real_pid
if live is not None:
    live.shutdown()

# ---------------------------------------------------------------------------
print("G. review fixes: launcher environment and cancel, run change, orphan episode, dead viewers")
for f in runs.glob("*.json"):
    f.unlink()
real_pid = mc._dimos_run_pid
mc._dimos_run_pid = lambda: None

# #1: the per-run switches of a CLI run never reach a page launch
envstub = Path(TMP, "env_stub.sh")
envstub.write_text('echo "env EXPLORE=${EXPLORE--} MOCK=${MOCK--} MOSS_MOCK_BUS=${MOSS_MOCK_BUS--} '
                   'STATE=${MOSS_STATE_DIR--}"\n')
saved = {k: os.environ.get(k) for k in ("EXPLORE", "MOCK", "MOSS_MOCK_BUS")}
os.environ.update(EXPLORE="1", MOCK="1", MOSS_MOCK_BUS="1")
la = mc.Launcher(str(envstub))
ans = la.start("explore")
for k, v in saved.items():
    if v is None:
        os.environ.pop(k, None)
    else:
        os.environ[k] = v
la._procs[-1][2].wait(5)
lines = la.status()["lines"]
want = f"env EXPLORE=- MOCK=- MOSS_MOCK_BUS=- STATE={os.environ['MOSS_STATE_DIR']}"
check("#1 cockpit env has EXPLORE=1 MOCK=1 MOSS_MOCK_BUS=1 -> the launched script sees none of them, "
      "MOSS_STATE_DIR still passed", ans["ok"] and want in lines, str(lines[-2:]))

# #9/#15: Stop during a launch ends the launch first
def fly_stub(trap):
    p = Path(TMP, f"fly_cancel_{int(trap)}.sh")
    ignore_term = "trap '' TERM; " if trap else ""
    p.write_text(f'case "$1" in\n  map) {ignore_term}'
                 f'sleep 30 & echo $! > {TMP}/sleep_{int(trap)}.pid; wait; echo "map reached the stack";;\n'
                 '  stop) echo "stub stop ran";;\nesac\n')
    return p


def pid_dead(pid, within_s=2.0):
    deadline = time.monotonic() + within_s
    while time.monotonic() < deadline:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        time.sleep(0.05)
    return False


for trap, sig, grace in ((False, "SIGTERM", None), (True, "SIGKILL", 0.5)):
    real_grace = mc.LAUNCH_KILL_GRACE_S
    if grace:
        mc.LAUNCH_KILL_GRACE_S = grace
    la = mc.Launcher(str(fly_stub(trap)))
    la.start("map")
    pidfile = Path(TMP, f"sleep_{int(trap)}.pid")
    deadline = time.monotonic() + 3
    while not (pidfile.exists() and pidfile.read_text().strip()) and time.monotonic() < deadline:
        time.sleep(0.05)
    map_proc = la._procs[-1][2]
    t = time.monotonic()
    ans = la.start("stop")
    took = time.monotonic() - t
    la._procs[-1][2].wait(5)
    lines = la.status()["lines"]
    rc = map_proc.poll()
    mc.LAUNCH_KILL_GRACE_S = real_grace
    want_rc = -signal.SIGTERM if sig == "SIGTERM" else -signal.SIGKILL
    check(f"#9/#15 Stop while 'map' is in its gates{' (TERM ignored)' if trap else ''} -> map ended by {sig} "
          f"(rc {want_rc}), its child sleep gone, then 'stop' ran",
          ans["ok"] and rc == want_rc and pid_dead(int(pidfile.read_text()))
          and any(f"cancelled by stop: {sig}" in ln for ln in lines) and "stub stop ran" in lines
          and "map reached the stack" not in lines, f"rc {rc}, {took:.2f} s, {lines[-3:]}")

# #18: an episode open when the stack stops is closed by the cockpit
register_g = Path(runs, "g.json")


def reg(run_id, blueprint="vector-dimos.moss-explore"):
    if run_id is None:
        register_g.unlink(missing_ok=True)
    else:
        register_g.write_text(json.dumps({"run_id": run_id, "pid": os.getpid(), "blueprint": blueprint,
                                          "started_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
                                          "log_dir": "/nowhere"}))


lc = mc.LiveCockpit(str(envstub))
reg("run-1")
lc._episode({"action": "start", "name": "ep9"})
ep_on = lc.state()["recording"]["episode"]
reg(None)
rec = lc.state()["recording"]
check("#18 episode ep9 open, stack stops -> episode null, recording.closed names ep9 and says why",
      (ep_on or {}).get("name") == "ep9" and rec["episode"] is None and (rec["closed"] or {}).get("name") == "ep9"
      and "stack stopped" in rec["closed"]["reason"], json.dumps(rec)[:200])
lc._episode({"action": "start", "name": "ep10"})
check("#18 the next episode start clears recording.closed", lc.episode_closed is None)

# #19: a new run_id drops the previous run's messages
if HAVE_LCM_MSGS and mc.HAVE_MSGS:
    lc = mc.LiveCockpit(str(envstub))
    reg("run-1")
    lc.state()
    g = OccupancyGrid(data_length=6, header=hdr("world", 1760000000, 0),
                      info=MapMetaData(map_load_time=Time(), resolution=0.5, width=3, height=2,
                                       origin=pose_of(1.0, 2.0)), data=[-1, 0, 100, 50, 0, -1])
    lc.bus.ingest(mc.CHANNELS["global_costmap"], g.lcm_encode())
    lc.bus.ingest(mc.CHANNELS["explore_cmd"], Bool(data=True).lcm_encode())
    lc.bus.ingest(mc.CHANNELS["goal_request"],
                  PoseStamped(header=hdr("world"), pose=pose_of(2.0, 3.5)).lcm_encode())
    a = (lc.map() is not None, lc.state()["nav"]["exploring"], lc.state()["nav"]["goal"])
    reg(None)
    b = lc.map() is not None
    reg("run-2")
    nav = lc.state()["nav"]
    c = (lc.map() is None, nav["exploring"], nav["goal"], lc.state()["topics"]["global_costmap"]["age_s"])
    check("#19 run-1: costmap, exploring, goal (2, 3.5); stack stopped: map kept; run-2: no map, not exploring, "
          "no goal, costmap never seen", a == (True, True, {"x": 2.0, "y": 3.5}) and b
          and c == (True, False, None, None), f"{a} {b} {c}")
else:
    print("  -- skipped #19, no dimos_lcm here")
reg(None)
mc._dimos_run_pid = real_pid

# #42: a viewer that stops reading frees its camera slot after the write timeout
real_wto = mc.MJPEG_WRITE_TIMEOUT_S
mc.MJPEG_WRITE_TIMEOUT_S = 0.5
srv, c5 = serve(demo=True, page_dir=page_dir)
big = b"\xff\xd8" + bytes(2_000_000) + b"\xff\xd9"
srv.cockpit.camera_frame = lambda: (time.monotonic(), big)
sock = socket.socket()
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
sock.connect(("127.0.0.1", srv.server_address[1]))
sock.sendall(b"GET /camera.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")       # then never reads again
seen = False
deadline = time.monotonic() + 3
while time.monotonic() < deadline and not seen:
    seen = srv.cockpit.camera_clients == 1
    time.sleep(0.02)
t = time.monotonic()
freed = clients_drop_to_zero(srv.cockpit, within_s=10.0)
check("#42 a viewer that stops reading (socket still open) -> slot freed by the write timeout",
      seen and freed, f"seen {seen}, freed {freed} after {time.monotonic() - t:.1f} s")
sock.close()
srv.shutdown()
mc.MJPEG_WRITE_TIMEOUT_S = real_wto

# ---------------------------------------------------------------------------
print("F. hygiene")
src = (ROOT / "tools" / "moss_cockpit.py").read_text()
ctl = (ROOT / "tools" / "explore_ctl.py").read_text()
check("no dimOS / torch / open3d import", not re.search(r"^\s*(import|from)\s+(dimos|torch|open3d)\b", src, re.M))
check("no serial port: no pyserial import, no /dev/tty", not re.search(r"^\s*(import|from)\s+serial\b", src, re.M)
      and "/dev/tty" not in src)
lan = re.compile(r"\b(192\.168|10\.\d{1,3}|172\.(1[6-9]|2\d|3[01]))\.\d{1,3}\.\d{1,3}\b")
check("no LAN address literal in moss_cockpit.py / explore_ctl.py", not lan.search(src) and not lan.search(ctl))
check("no non-ASCII text in moss_cockpit.py (English only)", all(ord(ch) < 128 for ch in src))
check("'dimos' never imported by this bench", "dimos" not in sys.modules)

_TMPDIR.cleanup()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

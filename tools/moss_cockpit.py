#!/usr/bin/env python3
"""MOSS cockpit: one small web server on the rover, one page for a whole run.

    http://<rover>:8904/            the page (tools/moss_cockpit/index.html)
    .venv/bin/python tools/moss_cockpit.py            live: listens to the stack's LCM bus
    python3 tools/moss_cockpit.py --demo --port 8905  synthetic rover, no bus, no dimOS

Why a new process instead of a dimOS web module: the dimOS viewers on the Nano
either need a secure context (the relay cockpit), git LFS (the command center)
or cost a core with no client connected (inflate + distance transform on every
costmap). This server imports lcm, dimos_lcm and numpy only - never dimos,
torch or open3d - and does nothing until a page asks: LCM callbacks keep the
latest bytes and a timestamp, decoding happens on request and is cached by
that timestamp (map at most 2 Hz, scan at most 4 Hz). The camera channel is
subscribed only while an MJPEG client is watching.

What it touches: the LCM bus (explicit channels, listen; publish only on a POST
/cmd), ~/.local/state/dimos/runs/*.json and the running stack's main.jsonl
(read), $MOSS_STATE_DIR (default ~/.local/state/moss: preflight.json and the
recording, read; last_launch.log, written by the launcher it starts). It NEVER
opens a serial port: battery and arming state come from /base_telemetry, which
the stack's coordinator publishes.

Endpoints (all Cache-Control: no-store; a failing request never kills the server):
    GET  /  /static/<file>      the page and its assets (tools/moss_cockpit/)
    GET  /state.json            stack, pose, topic rates, base, nav, recording, system
    GET  /map.png  /map_meta    latest /global_costmap as a PNG + the pixel <-> metre contract
    GET  /scan.json             latest lidar points, world frame (fallback: lidar frame)
    GET  /path.json             planner path
    GET  /log.json?n=40         tail of the running stack's main.jsonl
    GET  /preflight.json        the last saved flight check
    GET  /launch.json           tail of the last launcher output
    GET  /camera.mjpg           nose camera (RealSense colour), multipart JPEG, at most 5 fps
    GET  /camera.mjpg?cam=wrist the arm's camera, same framing (cam=nose is the default)
    POST /cmd/explore|goal|stop|episode|stack   publish on the bus / run tools/moss-fly.sh
                                (header 'X-Requested-With: moss-cockpit' required, else 403;
                                explore and goal only on a moss-* stack, else 409)

Standard library + numpy (+ lcm and dimos_lcm in live mode). No build step,
no CDN: the field has no internet.
"""

from __future__ import annotations

import argparse
import base64
import datetime as _dt
import glob
import json
import math
import os
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import zlib
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_DIR = os.path.dirname(TOOLS_DIR)
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)      # explore_ctl, the shared out-of-stack publisher

try:
    import lcm  # the bus; absent on a laptop: live mode says so, --demo does not need it
except ImportError:
    lcm = None

try:
    from dimos_lcm.geometry_msgs import Point as LcmPoint
    from dimos_lcm.geometry_msgs import Pose as LcmPose
    from dimos_lcm.geometry_msgs import PoseStamped as LcmPoseStamped
    from dimos_lcm.geometry_msgs import Quaternion as LcmQuaternion
    from dimos_lcm.geometry_msgs import Twist as LcmTwist
    from dimos_lcm.geometry_msgs import Vector3 as LcmVector3
    from dimos_lcm.nav_msgs import OccupancyGrid as LcmOccupancyGrid
    from dimos_lcm.nav_msgs import Path as LcmPath
    from dimos_lcm.sensor_msgs import Image as LcmImage
    from dimos_lcm.sensor_msgs import PointCloud2 as LcmPointCloud2
    from dimos_lcm.std_msgs import Bool as LcmBool
    from dimos_lcm.std_msgs import Header as LcmHeader
    from dimos_lcm.std_msgs import String as LcmString
    from dimos_lcm.std_msgs import Time as LcmTime
    HAVE_MSGS = True
except Exception:  # noqa: BLE001 - pure-python codecs, but not installed everywhere
    HAVE_MSGS = False

DEFAULT_PORT = 8904
DEFAULT_BIND = "0.0.0.0"
PAGE_DIR = os.path.join(TOOLS_DIR, "moss_cockpit")
FLY_SCRIPT = os.path.join(TOOLS_DIR, "moss-fly.sh")

# LCM channels as the stack names them: "/<stream>#<dimOS msg_name>". Spelled
# out instead of derived from dimos_lcm classes, whose msg_name is not always
# the one dimOS uses (dimos_lcm's PoseStamped says just "PoseStamped").
CHANNELS = {
    "global_costmap": "/global_costmap#nav_msgs.OccupancyGrid",
    "odom": "/odom#geometry_msgs.PoseStamped",
    "lidar": "/lidar#sensor_msgs.PointCloud2",            # world frame, from LidarOdometry
    "pointcloud": "/pointcloud#sensor_msgs.PointCloud2",  # lidar frame, straight from the C1 driver
    "path": "/path#nav_msgs.Path",
    "goal_request": "/goal_request#geometry_msgs.PoseStamped",
    "goal_reached": "/goal_reached#std_msgs.Bool",
    "explore_done": "/explore_done#std_msgs.Bool",
    "explore_cmd": "/explore_cmd#std_msgs.Bool",
    "stop_explore_cmd": "/stop_explore_cmd#std_msgs.Bool",
    "stop_movement": "/stop_movement#std_msgs.Bool",       # MovementManager on a pad touch: ends exploration
    "cmd_vel": "/cmd_vel#geometry_msgs.Twist",
    "base_telemetry": "/base_telemetry#std_msgs.String",
    "episode_marker": "/episode_marker#std_msgs.String",
}
CAMERAS = {                     # cockpit name -> (bus topic, LCM channel); subscribed only while watched
    "nose": ("color_image", "/color_image#sensor_msgs.Image"),
    "wrist": ("wrist_image", "/wrist_image#sensor_msgs.Image"),
}
IMAGE_NAME, IMAGE_CHANNEL = CAMERAS["nose"]
STATE_TOPICS = ("odom", "global_costmap", "lidar", "pointcloud", "path", "cmd_vel",
                "base_telemetry") + tuple(name for name, _ in CAMERAS.values())

MAP_MIN_INTERVAL_S = 0.5      # decode + PNG the costmap at most 2 Hz, whoever asks
SCAN_MIN_INTERVAL_S = 0.25    # scan at most 4 Hz
SCAN_FALLBACK_S = 5.0         # /lidar silent this long -> show /pointcloud (lidar frame)
MAX_SCAN_POINTS = 1500
HZ_WINDOW_S = 5.0
MJPEG_PERIOD_S = 0.2          # 5 fps: enough to see where the nose points, cheap on the Nano
MAX_CAMERA_CLIENTS = 4
FIRST_FRAME_WAIT_S = 2.0
LCM_HANDLE_MS = 250           # the listener loop also (un)subscribes the camera between handles
SYSTEM_CACHE_S = 1.0
MAX_BODY = 65536
GOAL_LIMIT_M = 500.0          # MovementManager ignores clicks beyond 500 m: refuse them here, loudly
LAUNCH_TAIL = 60
LAUNCH_KILL_GRACE_S = 3.0     # 'stop' during a launch: SIGTERM its process group, SIGKILL after this
LAUNCH_ENV_DROP = ("EXPLORE", "MOCK", "MOSS_MOCK_BUS")   # per-run switches of a CLI run: never inherited
GOAL_AFTER_STOP_S = 3.0       # the explorer's own 'stay here' goal lands ~2 s after its stop: ours goes after
MJPEG_WRITE_TIMEOUT_S = 5.0   # a viewer that vanished without FIN/RST frees its slot after this
CMD_HEADER = ("X-Requested-With", "moss-cockpit")   # every POST /cmd/* carries it: a cross-site form cannot

# 3S pack: 12.6 V full, the BMS cuts at 10.2 V (linear, clamped; a voltage
# gauge, not a coulomb counter).
BATTERY_EMPTY_V = 10.2
BATTERY_FULL_V = 12.6
MODE_NAMES = {0: "off", 1: "ready", 2: "drive", 3: "test"}

# Map colours, UI-only (the brand has no occupancy palette): unknown = STUDIO,
# free = a darker studio, cost 1..99 ramps from free to a light slate, lethal =
# brand WHITE, so obstacles read first on the dark page.
COLOR_UNKNOWN = (0x3D, 0x57, 0x6A)
COLOR_FREE = (0x2E, 0x45, 0x56)
COLOR_COST_MAX = (0x8F, 0xA6, 0xB5)
COLOR_OCCUPIED = (0xEA, 0xF0, 0xEF)

_ISO = "%Y-%m-%dT%H:%M:%S"


def state_dir() -> str:
    """MOSS's own state dir (recordings, preflight.json, last_launch.log)."""
    return os.path.expanduser(os.environ.get("MOSS_STATE_DIR") or "~/.local/state/moss")


def runs_dir() -> str:
    """dimOS's run registry, resolved the way dimos.constants does on Linux."""
    base = os.environ.get("XDG_STATE_HOME") or os.path.expanduser("~/.local/state")
    return os.path.join(base, "dimos", "runs")


# --- pictures ----------------------------------------------------------------

def png_bytes(rgb: np.ndarray, level: int = 3) -> bytes:
    """An (h, w, 3) uint8 array as a PNG. zlib and struct (tools/zone_server.py's
    _png_bytes; level 3 instead of 6: ~2x faster on a 480x480 costmap, a few % bigger)."""
    h, w, _ = rgb.shape
    rows = np.zeros((h, w * 3 + 1), dtype=np.uint8)
    rows[:, 1:] = rgb.reshape(h, w * 3)          # filter byte 0 = None, per row
    raw = zlib.compress(rows.tobytes(), level)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    return (b"\x89PNG\r\n\x1a\n"
            + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", raw)
            + chunk(b"IEND", b""))


def _build_lut() -> np.ndarray:
    """256-entry colour table indexed by the int8 cell value seen as uint8."""
    lut = np.empty((256, 3), dtype=np.uint8)
    lut[:] = COLOR_UNKNOWN                       # -128..-1 (as 128..255): unknown
    lut[0] = COLOR_FREE
    free = np.array(COLOR_FREE, dtype=float)
    top = np.array(COLOR_COST_MAX, dtype=float)
    for v in range(1, 100):
        lut[v] = np.rint(free + (top - free) * (v / 99.0)).astype(np.uint8)
    lut[100:128] = COLOR_OCCUPIED                # 100 lethal (and anything above)
    return lut


MAP_LUT = _build_lut()


def map_meta(width: int, height: int, res: float, ox: float, oy: float,
             stamp: float, frame: str) -> dict:
    """tools/zone_server.py's /map_meta contract, plus the costmap stamp and frame."""
    return {
        "res": res, "ox": ox, "oy": oy,
        "width": int(width), "height": int(height),
        "x_min": ox, "x_max": ox + width * res,
        "y_min": oy, "y_max": oy + height * res,
        "stamp": stamp, "frame": frame,
        # px -> metres and back. Row 0 of the PNG is the highest y.
        "x_of_px": "x = ox + (px + 0.5) * res",
        "y_of_py": "y = oy + (height - py - 0.5) * res",
        "px_of_x": "px = (x - ox) / res",
        "py_of_y": "py = height - (y - oy) / res",
    }


def render_grid(cells: np.ndarray, res: float, ox: float, oy: float,
                stamp: float = 0.0, frame: str = "world") -> tuple[bytes, dict]:
    """An OccupancyGrid's cells (rows = y from min y up, int8 -1/0/1..99/100) as (PNG, meta).
    Flipped so PNG row 0 is the highest y, like zone_server."""
    grid = np.ascontiguousarray(cells, dtype=np.int8)
    h, w = grid.shape
    rgb = MAP_LUT[grid.view(np.uint8)]
    return png_bytes(np.flipud(rgb)), map_meta(w, h, float(res), float(ox), float(oy), stamp, frame)


# --- message decoding (bytes off the bus -> plain values) -----------------------

def _stamp(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nsec) * 1e-9


def yaw_of(qx: float, qy: float, qz: float, qw: float) -> float:
    """Heading (rad) of a quaternion, z-up."""
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def decode_costmap(raw: bytes) -> tuple[bytes, dict]:
    m = LcmOccupancyGrid.lcm_decode(raw)
    w, h, n = int(m.info.width), int(m.info.height), int(m.data_length)
    # the int8 cells are the message's last field: view them straight off the
    # raw bytes instead of rebuilding a 230k-element tuple
    cells = np.frombuffer(raw, dtype=np.int8, count=n, offset=len(raw) - n)
    if n != w * h:
        raise ValueError(f"costmap {w}x{h} carries {n} cells")
    o = m.info.origin.position
    return render_grid(cells.reshape(h, w), float(m.info.resolution), float(o.x), float(o.y),
                       _stamp(m.header), m.header.frame_id or "world")


def decode_pose(raw: bytes) -> dict:
    m = LcmPoseStamped.lcm_decode(raw)
    p, q = m.pose.position, m.pose.orientation
    return {"x": float(p.x), "y": float(p.y), "z": float(p.z),
            "yaw": yaw_of(q.x, q.y, q.z, q.w), "stamp": _stamp(m.header),
            "frame": m.header.frame_id}


def cloud_xy(raw: bytes) -> tuple[str, float, np.ndarray]:
    """(frame, stamp, Nx2 float32 x/y) of a PointCloud2, NaN/inf rows dropped.
    Field offsets read from the message (dimOS writes x,y,z float32 at 0/4/8 with a
    16-byte step, more when per-point extras are attached)."""
    m = LcmPointCloud2.lcm_decode(raw)
    n = int(m.width) * int(m.height)
    step = int(m.point_step)
    offs = {f.name: int(f.offset) for f in m.fields}
    if n == 0 or step < 8:
        return m.header.frame_id, _stamp(m.header), np.zeros((0, 2), np.float32)
    end = ">" if m.is_bigendian else "<"
    dt = np.dtype({"names": ["x", "y"], "formats": [end + "f4", end + "f4"],
                   "offsets": [offs.get("x", 0), offs.get("y", 4)], "itemsize": step})
    data = m.data if isinstance(m.data, (bytes, bytearray)) else bytes(m.data)
    arr = np.frombuffer(data, dtype=dt, count=min(n, len(data) // step))
    xy = np.column_stack((arr["x"], arr["y"])).astype(np.float32)
    return m.header.frame_id, _stamp(m.header), xy[np.isfinite(xy).all(axis=1)]


def downsample(points: np.ndarray, limit: int = MAX_SCAN_POINTS) -> np.ndarray:
    """At most `limit` rows, taken evenly across the array (keeps the angular spread)."""
    if len(points) <= limit:
        return points
    return points[np.linspace(0, len(points) - 1, limit).astype(int)]


def decode_path(raw: bytes) -> list[list[float]]:
    m = LcmPath.lcm_decode(raw)
    return [[round(float(p.pose.position.x), 3), round(float(p.pose.position.y), 3)] for p in m.poses]


def decode_twist(raw: bytes) -> tuple[float, float]:
    m = LcmTwist.lcm_decode(raw)
    return float(m.linear.x), float(m.angular.z)


def decode_bool(raw: bytes) -> bool:
    return bool(LcmBool.lcm_decode(raw).data)


def decode_string(raw: bytes) -> str:
    return str(LcmString.lcm_decode(raw).data)


# --- messages to publish ----------------------------------------------------------
# Every nested field is built fresh: the generated dimos_lcm constructors take
# their defaults as shared instances (`pose=Pose()` evaluated once), so mutating
# a default-built message would silently change every later one in the process.

def goal_message(x: float, y: float, now: float | None = None):
    """A navigation goal the way dimOS's own web module sends a click: frame world,
    z 0, identity orientation, stamped now."""
    now = time.time() if now is None else now
    header = LcmHeader(seq=0, stamp=LcmTime(sec=int(now), nsec=int((now % 1.0) * 1e9)), frame_id="world")
    pose = LcmPose(position=LcmPoint(x=float(x), y=float(y), z=0.0),
                   orientation=LcmQuaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    return LcmPoseStamped(header=header, pose=pose)


def zero_twist():
    return LcmTwist(linear=LcmVector3(x=0.0, y=0.0, z=0.0), angular=LcmVector3(x=0.0, y=0.0, z=0.0))


# --- the base, from its telemetry JSON ------------------------------------------

def battery_pct(volts: float | None) -> int | None:
    """3S linear gauge: 10.2 V -> 0 %, 12.6 V -> 100 %, clamped."""
    if volts is None:
        return None
    frac = (volts - BATTERY_EMPTY_V) / (BATTERY_FULL_V - BATTERY_EMPTY_V)
    return int(round(100.0 * min(1.0, max(0.0, frac))))


def base_view(t: dict, age_s: float | None) -> dict:
    """The /base_telemetry JSON (VectorControlCoordinator, every 0.5 s) as the page shows it."""
    mv = t.get("bus_mV")
    volts = round(mv / 1000.0, 2) if isinstance(mv, (int, float)) else None
    mode = t.get("mode")
    return {
        "fw": t.get("fw"), "driver": t.get("driver"),
        "mode": mode, "mode_name": MODE_NAMES.get(mode, "unknown") if mode is not None else None,
        "reason": t.get("reason"),
        "bus_mV": mv, "battery_v": volts,
        "battery_pct": battery_pct(volts) if t.get("ina_ok", True) else None,
        "ina_ok": t.get("ina_ok"),
        "ticks": t.get("ticks"), "target_pct": t.get("target_pct"), "output_pct": t.get("output_pct"),
        "ramp_ms": t.get("ramp_ms"),
        "firmware_age_s": t.get("firmware_age_s"), "mock": t.get("mock"),
        "age_s": None if age_s is None else round(age_s, 2),
    }


# --- the machine (copied from tools/stats_server.py: stdlib, sysfs, /proc) -------

_prev_cpu = None
_prev_cores = None
_gpu_samples: list = []
_sys_lock = threading.Lock()
_sys_cache = [0.0, None]
GPU_LOAD_PATHS = ("/sys/devices/platform/bus@0/17000000.gpu/load", "/sys/devices/gpu.0/load")


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:  # noqa: BLE001 - sysfs raises odd errors on absent sensors
        return None


def cpu_percent():
    """CPU usage since the previous call (first call: 0.2 s sample)."""
    global _prev_cpu

    def sample():
        vals = [int(x) for x in _read("/proc/stat").splitlines()[0].split()[1:]]
        return vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)

    if _prev_cpu is None:
        _prev_cpu = sample()
        time.sleep(0.2)
    idle0, total0 = _prev_cpu
    idle1, total1 = sample()
    _prev_cpu = (idle1, total1)
    dt = total1 - total0
    return round(100.0 * (1 - (idle1 - idle0) / dt), 1) if dt > 0 else 0.0


def cpu_per_core():
    """Per-core usage since the previous call: one saturated worker reads differently
    from a spread load."""
    global _prev_cores

    def sample():
        out = []
        for line in _read("/proc/stat").splitlines()[1:]:
            if not line.startswith("cpu"):
                break
            vals = [int(x) for x in line.split()[1:]]
            out.append((vals[3] + (vals[4] if len(vals) > 4 else 0), sum(vals)))
        return out

    if _prev_cores is None:
        _prev_cores = sample()
        time.sleep(0.2)
    prev, cur = _prev_cores, sample()
    _prev_cores = cur
    res = []
    for (i0, t0), (i1, t1) in zip(prev, cur):
        dt = t1 - t0
        res.append(round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0)
    return res


def meminfo():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        k, v = line.split(":", 1)
        info[k] = int(v.strip().split()[0])  # kB
    total = info["MemTotal"]
    used = total - info.get("MemAvailable", info.get("MemFree", 0))
    return {"ram_used_gb": round(used / 1024**2, 1), "ram_total_gb": round(total / 1024**2, 1),
            "ram_percent": round(100.0 * used / total, 1)}


def _gpu_read_once():
    """Orin GPU load; sysfs gives tenths of a percent."""
    for path in GPU_LOAD_PATHS:
        raw = _read(path)
        if raw is not None:
            try:
                return int(raw) / 10.0
            except ValueError:
                pass
    return None


def _gpu_sampler():
    """10 Hz, 3 s average: the mapper works in 4 ms CUDA bursts that a single
    read misses (stats_server samples at 20 Hz; half is plenty for a 1 Hz page)."""
    while True:
        v = _gpu_read_once()
        now = time.time()
        if v is not None:
            _gpu_samples.append((now, v))
            while _gpu_samples and now - _gpu_samples[0][0] > 3.0:
                _gpu_samples.pop(0)
        time.sleep(0.1)


def gpu_percent():
    if not _gpu_samples:
        return _gpu_read_once()
    return round(sum(v for _, v in _gpu_samples) / len(_gpu_samples), 1)


def temps():
    zones = {}
    for z in glob.glob("/sys/class/thermal/thermal_zone*"):
        name = _read(os.path.join(z, "type")) or "?"
        raw = _read(os.path.join(z, "temp"))
        if raw is None:
            continue
        try:
            t = int(raw) / 1000.0
        except ValueError:
            continue
        if t <= 0:
            continue
        zones[name.replace("-thermal", "").replace("_thermal", "")] = round(t, 1)
    return zones


def disks():
    seen, out = set(), []
    for line in (_read("/proc/mounts") or "").splitlines():
        dev, mnt, fs = line.split()[:3]
        if fs not in ("ext4", "vfat", "xfs", "btrfs", "f2fs") or mnt in seen or mnt.startswith("/boot"):
            continue
        seen.add(mnt)
        try:
            st = os.statvfs(mnt)
        except OSError:
            continue
        total = st.f_blocks * st.f_frsize
        free = st.f_bavail * st.f_frsize
        if total == 0:
            continue
        out.append({"mountpoint": mnt, "fstype": fs,
                    "used_gb": round((total - free) / 1024**3, 1), "total_gb": round(total / 1024**3, 1),
                    "free_gb": round(free / 1024**3, 1), "percent": round(100.0 * (total - free) / total, 1)})
    return sorted(out, key=lambda d: d["mountpoint"])


def system_metrics() -> dict:
    """The machine, cached 1 s: several pages polling at 1 Hz cost one /proc pass."""
    with _sys_lock:
        stamp, cached = _sys_cache
        if cached is not None and time.monotonic() - stamp < SYSTEM_CACHE_S:
            return cached
        out = {"cpu_percent": cpu_percent(), "cpu_per_core": cpu_per_core(), "gpu_percent": gpu_percent()}
        try:
            out.update(meminfo())
        except Exception:  # noqa: BLE001
            pass
        out["temps"] = temps()
        out["disks"] = disks()
        out["load_1m"] = round(os.getloadavg()[0], 2)
        out["uptime_s"] = int(float((_read("/proc/uptime") or "0 0").split()[0]))
        _sys_cache[:] = [time.monotonic(), out]
        return out


# --- the stack, without asking dimOS -----------------------------------------------

def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True             # exists, not ours to signal
    except (OSError, TypeError, ValueError):
        return False


def _parse_iso(text: str) -> float | None:
    try:
        return _dt.datetime.fromisoformat(str(text).replace("Z", "+00:00")).timestamp()
    except (TypeError, ValueError):
        return None


def stack_status(now: float | None = None) -> dict:
    """The live run from dimOS's registry (dead pids skipped, never `dimos status`:
    every dimOS CLI call writes a log file and it would also prune the registry)."""
    now = time.time() if now is None else now
    best = None
    for path in glob.glob(os.path.join(runs_dir(), "*.json")):
        try:
            with open(path) as fh:
                entry = json.load(fh)
            pid = int(entry.get("pid"))
        except Exception:  # noqa: BLE001 - half-written or foreign file
            continue
        if not _pid_alive(pid):
            continue
        started = _parse_iso(entry.get("started_at"))
        if best is None or (started or 0) > (best[0] or 0):
            best = (started, pid, entry)
    if best is None:
        return {"running": False}
    started, pid, entry = best
    blueprint = str(entry.get("blueprint", ""))
    return {"running": True, "run_id": entry.get("run_id"),
            "blueprint": blueprint.rsplit(".", 1)[-1], "pid": pid,
            "started_at": entry.get("started_at"),
            "uptime_s": round(now - started, 1) if started else None,
            "log_dir": entry.get("log_dir")}


def _dimos_run_pid() -> int | None:
    """A `dimos run` process even without a registry entry (a foreground run, a
    registry swept by someone's `dimos status`). TOCTOU-safe like stats_server."""
    try:
        pids = os.listdir("/proc")
    except OSError:
        return None
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                argv = fh.read().split(b"\0")
        except OSError:
            continue
        if any(a.endswith(b"bin/dimos") for a in argv) and b"run" in argv:
            return int(pid)
    return None


def log_tail(log_dir: str | None, n: int) -> list[dict]:
    """The last n records of <log_dir>/main.jsonl as {ts, level, event, logger}."""
    if not log_dir:
        return []
    path = os.path.join(log_dir, "main.jsonl")
    try:
        with open(path, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - 262144))
            chunk = fh.read()
    except OSError:
        return []
    out = []
    for line in reversed(chunk.splitlines()):     # newest first, until n records parse
        if len(out) >= n:
            break
        try:
            rec = json.loads(line)
        except ValueError:
            continue                              # a half-written last line, a stray print
        if isinstance(rec, dict):
            out.append({"ts": rec.get("timestamp"), "level": rec.get("level"),
                        "event": rec.get("event"), "logger": rec.get("logger")})
    return out[::-1]


def preflight_doc() -> dict:
    """The last flight check moss-fly.sh saved (moss_preflight.py --json)."""
    path = os.path.join(state_dir(), "preflight.json")
    try:
        with open(path) as fh:
            doc = json.load(fh)
    except FileNotFoundError:
        return {"available": False}
    except (OSError, ValueError) as exc:
        return {"available": False, "error": f"{path}: {exc}"}
    if not isinstance(doc, dict):
        return {"available": False, "error": f"{path}: not a JSON object"}
    doc["available"] = True
    doc["mtime"] = os.path.getmtime(path)
    return doc


def recording_view(episode: dict | None, closed: dict | None = None) -> dict:
    """`closed`: the episode the cockpit closed itself because the stack stopped under it."""
    db = os.path.join(state_dir(), "recordings", "moss.db")
    size = 0
    for p in (db, db + "-wal", db + "-shm"):
        try:
            size += os.path.getsize(p)
        except OSError:
            pass
    return {"db": db, "exists": os.path.isfile(db), "size_mb": round(size / 1e6, 1), "episode": episode,
            "closed": closed}


_EPISODE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9 ._-]{0,59}$")


def episode_name(raw) -> str:
    """A safe episode name: letters, digits, space . _ -, 60 max; default ep-HHMMSS."""
    name = str(raw or "").strip()
    if not name:
        return time.strftime("ep-%H%M%S")
    if not _EPISODE_NAME.match(name):
        raise ValueError("episode name: letters, digits, space . _ - only, 60 characters max")
    return name


def _finite_xy(body: dict) -> tuple[float, float]:
    try:
        x, y = float(body.get("x")), float(body.get("y"))
    except (TypeError, ValueError):
        raise ValueError("goal needs numbers x and y (metres, world frame)") from None
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError("goal x/y must be finite")
    if abs(x) > GOAL_LIMIT_M or abs(y) > GOAL_LIMIT_M:
        raise ValueError(f"goal beyond {GOAL_LIMIT_M:.0f} m: the movement manager would drop it")
    return x, y


def _later(delay_s: float, fn) -> threading.Timer:
    """Run fn once, delay_s from now, on a daemon thread (the bench swaps this out)."""
    def run():
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - a timer thread has nobody to raise to
            print(f"moss_cockpit: deferred command failed: {exc!r}", file=sys.stderr, flush=True)
    timer = threading.Timer(delay_s, run)
    timer.daemon = True
    timer.start()
    return timer


# --- the launcher, as a child process -----------------------------------------------

class Launcher:
    """Runs tools/moss-fly.sh <action> without waiting for it; output appended to
    $MOSS_STATE_DIR/last_launch.log (a `stop` may run while a `map` is still in
    its gates, so both write to the same file, each under a header line)."""

    def __init__(self, script: str = FLY_SCRIPT):
        self.script = script
        self._lock = threading.Lock()
        self._procs: list[tuple[str, float, subprocess.Popen]] = []
        self.last: dict = {}

    def log_path(self) -> str:
        return os.path.join(state_dir(), "last_launch.log")

    def _running(self) -> list:
        return [(a, t, p) for a, t, p in self._procs if p.poll() is None]

    def _cancel(self, fh) -> None:
        """Stop pressed during a launch: end every launcher still in its gates first,
        or it would bring the stack up after the operator's stop. Each one leads its
        own process group (start_new_session), so its preflight child goes too."""
        for action, _, proc in self._running():
            if action == "stop":
                continue
            how = "SIGTERM"
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=LAUNCH_KILL_GRACE_S)
                except subprocess.TimeoutExpired:
                    how = "SIGKILL"
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=LAUNCH_KILL_GRACE_S)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                pass
            fh.write(f"=== moss-fly.sh {action} (pid {proc.pid}) cancelled by stop: {how} ===\n".encode())

    def start(self, action: str) -> dict:
        with self._lock:
            busy = self._running()
            if busy and action != "stop":
                return {"ok": False, "error": f"moss-fly.sh {busy[-1][0]} is still running"}
            if not os.path.isfile(self.script):
                return {"ok": False, "error": f"launcher missing: {self.script}"}
            path = self.log_path()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            # a cockpit started by `EXPLORE=1 moss-fly.sh explore` must not arm the next page launch
            env = {k: v for k, v in os.environ.items() if k not in LAUNCH_ENV_DROP}
            with open(path, "ab" if busy else "wb") as fh:
                if action == "stop":
                    self._cancel(fh)
                fh.write(f"=== moss-fly.sh {action}  {time.strftime(_ISO)} ===\n".encode())
                fh.flush()
                proc = subprocess.Popen(["bash", self.script, action], cwd=REPO_DIR, env=env,
                                        stdin=subprocess.DEVNULL, stdout=fh, stderr=subprocess.STDOUT,
                                        start_new_session=True)   # its own process group: Stop can end it
            self._procs = self._running() + [(action, time.time(), proc)]
            self.last = {"action": action, "started_at": time.time(), "pid": proc.pid}
            return {"ok": True, "sent": f"moss-fly.sh {action}", "pid": proc.pid}

    def status(self, lines: bool = True) -> dict:
        with self._lock:
            running = self._running()
            last = self._procs[-1] if self._procs else None
            rc = last[2].poll() if last else None
        head = {"running": bool(running), "action": last[0] if last else None,
                "started_at": last[1] if last else None, "returncode": rc}
        if not lines:
            return head
        try:
            with open(self.log_path(), "rb") as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - 65536))
                tail = fh.read().decode("utf-8", "replace").splitlines()[-LAUNCH_TAIL:]
        except OSError:
            tail = []
        return dict(head, lines=tail)


# --- bus bookkeeping ---------------------------------------------------------------

class _Latest:
    __slots__ = ("raw", "mono", "arrivals")

    def __init__(self):
        self.raw = None
        self.mono = None
        self.arrivals = deque(maxlen=64)


class BusCache:
    """Latest bytes + arrival times per stream. Callbacks only store; decoding is
    the request's job, cached by the arrival stamp."""

    def __init__(self):
        self._lock = threading.Lock()
        self._topics = {name: _Latest() for name in list(CHANNELS) + [n for n, _ in CAMERAS.values()]}
        self._by_channel = {ch: name for name, ch in CHANNELS.items()}
        for name, ch in CAMERAS.values():
            self._by_channel[ch] = name
        self._decoded: dict = {}

    def ingest(self, channel: str, data: bytes, mono: float | None = None) -> None:
        name = self._by_channel.get(channel)
        if name is None:
            return
        mono = time.monotonic() if mono is None else mono
        with self._lock:
            t = self._topics[name]
            t.raw, t.mono = data, mono
            t.arrivals.append(mono)

    def clear(self) -> None:
        """Forget every message: a new run must not show the previous run's map, scan or goal."""
        with self._lock:
            self._topics = {name: _Latest() for name in self._topics}
            self._decoded = {}

    def latest(self, name: str):
        with self._lock:
            t = self._topics[name]
            return t.raw, t.mono

    def stats(self, name: str, now: float) -> dict:
        with self._lock:
            t = self._topics[name]
            recent = [a for a in t.arrivals if now - a <= HZ_WINDOW_S]
            mono = t.mono
        hz = (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) >= 2 and recent[-1] > recent[0] else 0.0
        return {"age_s": None if mono is None else round(now - mono, 2), "hz": round(hz, 1)}

    def decoded(self, name: str, fn):
        """fn(raw) for the latest message of `name`, computed once per arrival.
        Returns (value, mono) or (None, None); a decode error caches None."""
        raw, mono = self.latest(name)
        if raw is None:
            return None, None
        hit = self._decoded.get(name)
        if hit is not None and hit[0] == mono:
            return hit[1], mono
        try:
            value = fn(raw)
        except Exception:  # noqa: BLE001 - one bad message must not poison the page
            value = None
        self._decoded[name] = (mono, value)
        return value, mono


# --- the cockpit: shared part ---------------------------------------------------------

class CameraUnavailable(RuntimeError):
    """The camera stream cannot be served (and why, for a 503)."""


class Cockpit:
    mode = "live"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        self.launcher = Launcher(fly_script)
        self.episode: dict | None = None
        self.episode_closed: dict | None = None   # closed by the cockpit when the stack stopped under it
        self._ep_lock = threading.Lock()
        self._cam_lock = threading.Lock()
        self._cam_clients = {cam: 0 for cam in CAMERAS}
        self.started = time.time()

    @property
    def camera_clients(self) -> int:
        """Viewers on all cameras together."""
        return sum(self._cam_clients.values())

    def camera_viewers(self, cam: str) -> int:
        return self._cam_clients.get(cam, 0)

    def camera_state(self) -> dict:
        doc = {"clients": self.camera_clients, "subscribed": self.camera_clients > 0}
        for cam in CAMERAS:
            n = self._cam_clients[cam]
            doc[cam] = {"clients": n, "subscribed": n > 0}
        return doc

    # episode bookkeeping, shared by both modes
    def _episode(self, body: dict) -> dict:
        action = str(body.get("action", ""))
        if action not in ("start", "stop"):
            raise ValueError('episode needs "action": "start" or "stop"')
        with self._ep_lock:
            if action == "start":
                name = episode_name(body.get("name"))
                self.episode = {"name": name, "started_at": time.time()}
                self.episode_closed = None
            else:
                name = (self.episode or {}).get("name") or episode_name(body.get("name"))
                self.episode = None
        return {"action": action, "name": name, "ts": time.time()}

    def camera_open(self, cam: str = "nose") -> None:
        if cam not in CAMERAS:
            raise CameraUnavailable(f"no camera called {cam!r} (nose or wrist)")
        with self._cam_lock:
            if self._cam_clients[cam] >= MAX_CAMERA_CLIENTS:
                raise CameraUnavailable(f"{MAX_CAMERA_CLIENTS} viewers already on the {cam} camera")
            self._cam_clients[cam] += 1

    def camera_close(self, cam: str = "nose") -> None:
        with self._cam_lock:
            if cam in self._cam_clients:
                self._cam_clients[cam] = max(0, self._cam_clients[cam] - 1)

    def launch(self) -> dict:
        return self.launcher.status()


# --- live: the stack's bus ----------------------------------------------------------

class LiveCockpit(Cockpit):
    mode = "live"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        super().__init__(fly_script)
        self.bus = BusCache()
        self.bus_error: str | None = None if lcm is not None else "python module lcm not importable"
        if not HAVE_MSGS:
            self.bus_error = "python module dimos_lcm not importable"
        self._pub_lock = threading.Lock()
        self._pub_lcm = None
        self._map_cache = (None, None, 0.0)     # (mono, (png, meta), decoded_at)
        self._scan_cache = (None, None, 0.0)
        self._cam_cache = {cam: (None, None) for cam in CAMERAS}   # cam -> (mono, jpeg)
        self._encoder = None
        self._ep_seen = None
        self._run_id = None                     # the run the bus cache belongs to
        self._goal_lock = threading.Lock()
        self._goal_gen = 0                      # a stop, an explore or a newer goal cancels a queued goal

    # the listener ------------------------------------------------------------
    def start_listener(self) -> None:
        if lcm is None:
            return
        threading.Thread(target=self._listen, name="lcm-listener", daemon=True).start()

    def _listen(self) -> None:
        """One LCM handle, one thread (stats_server._lcm_listener), explicit channels
        only; the camera is subscribed while someone watches it."""
        while True:
            try:
                lc = lcm.LCM()
                for ch in CHANNELS.values():
                    lc.subscribe(ch, self._on_message)
                image_subs = {cam: None for cam in CAMERAS}
                self.bus_error = None
                while True:
                    for cam, (_, channel) in CAMERAS.items():
                        want = self._cam_clients[cam] > 0
                        if want and image_subs[cam] is None:
                            image_subs[cam] = lc.subscribe(channel, self._on_message)
                        elif not want and image_subs[cam] is not None:
                            lc.unsubscribe(image_subs[cam])
                            image_subs[cam] = None
                    lc.handle_timeout(LCM_HANDLE_MS)
            except Exception as exc:  # noqa: BLE001 - multicast not up yet, etc.: retry
                self.bus_error = f"{type(exc).__name__}: {exc}"
                time.sleep(3.0)

    def _on_message(self, channel: str, data: bytes) -> None:
        self.bus.ingest(channel, data)

    # reads -------------------------------------------------------------------
    def _pose(self, now: float) -> dict | None:
        pose, mono = self.bus.decoded("odom", decode_pose)
        if pose is None:
            return None
        return {"x": round(pose["x"], 3), "y": round(pose["y"], 3), "yaw": round(pose["yaw"], 4),
                "age_s": round(now - mono, 2)}

    def _base(self, now: float) -> dict | None:
        t, mono = self.bus.decoded("base_telemetry", lambda raw: json.loads(decode_string(raw)))
        if not isinstance(t, dict):
            return None
        return base_view(t, now - mono)

    def _bool(self, name: str):
        value, mono = self.bus.decoded(name, decode_bool)
        return value, mono

    def _nav(self, now: float, stack: dict | None = None) -> dict:
        start, t_start = self._bool("explore_cmd")
        _, t_stop = self._bool("stop_explore_cmd")
        done, t_done = self._bool("explore_done")
        # a pad touch on moss-explore: MovementManager publishes stop_movement and the
        # explorer stops on it (WavefrontFrontierExplorer._on_stop_movement)
        halt, t_halt = self._bool("stop_movement")
        t_start = t_start if start else None
        exploring = t_start is not None and t_start > max(t_stop or -1.0, (t_done if done else None) or -1.0,
                                                          (t_halt if halt else None) or -1.0)
        # moss-nav has no explorer: an explore_cmd there only opens map writing (the costmap
        # mission gate), nothing drives. Only an explore stack can be exploring.
        if stack is not None and stack.get("running") and "explore" not in str(stack.get("blueprint") or ""):
            exploring = False
        explore_done = bool(done) and (t_start is None or (t_done or 0) > t_start)
        goal_pose, t_goal = self.bus.decoded("goal_request", decode_pose)
        goal = None
        if goal_pose is not None and math.isfinite(goal_pose["x"]) and math.isfinite(goal_pose["y"]):
            goal = {"x": round(goal_pose["x"], 3), "y": round(goal_pose["y"], 3)}
        reached, t_reached = self._bool("goal_reached")
        goal_reached = reached if (t_reached is not None and (t_goal is None or t_reached > t_goal)) else None
        tw, t_tw = self.bus.decoded("cmd_vel", decode_twist)
        last_cmd = None if tw is None else {"vx": round(tw[0], 3), "wz": round(tw[1], 3),
                                            "age_s": round(now - t_tw, 2)}
        return {"exploring": exploring, "explore_done": explore_done, "goal": goal,
                "goal_reached": goal_reached, "last_cmd_vel": last_cmd}

    def _episode_from_bus(self) -> None:
        """Markers published by anyone (another page, a script) move the episode too."""
        marker, mono = self.bus.decoded("episode_marker", lambda raw: json.loads(decode_string(raw)))
        if not isinstance(marker, dict) or mono == self._ep_seen:
            return
        self._ep_seen = mono
        with self._ep_lock:
            if marker.get("action") == "start":
                self.episode = {"name": marker.get("name"), "started_at": marker.get("ts")}
                self.episode_closed = None
            elif marker.get("action") == "stop":
                self.episode = None

    def _sync_run(self, stack: dict) -> None:
        """A new run_id in the registry: drop the previous run's messages (its costmap
        is in another odometry frame). A stop alone keeps them: the last map stays readable."""
        run_id = stack.get("run_id") if stack.get("running") else None
        if run_id is None or run_id == self._run_id:
            return
        if self._run_id is not None:
            self.bus.clear()
            self._map_cache = (None, None, 0.0)
            self._scan_cache = (None, None, 0.0)
            self._cam_cache = (None, None)
        self._run_id = run_id

    def _close_orphan_episode(self) -> None:
        """The stack stopped with an episode open: nothing records it any more, close it here."""
        with self._ep_lock:
            if self.episode is None:
                return
            self.episode_closed = dict(self.episode, ts=time.time(),
                                       reason="the stack stopped: the episode was closed")
            self.episode = None

    def _moss_stack(self) -> dict | None:
        """The running stack if it is a MOSS one. VECTOR's stack may run on the same
        computer and the same bus: it never takes a goal or an explore_cmd from this page."""
        st = stack_status()
        return st if st.get("running") and str(st.get("blueprint") or "").startswith("moss") else None

    def state(self) -> dict:
        now = time.monotonic()
        stack = stack_status()
        self._sync_run(stack)
        self._episode_from_bus()
        if not stack["running"]:
            pid = _dimos_run_pid()
            if pid:
                stack = {"running": True, "run_id": None, "blueprint": None, "pid": pid,
                         "started_at": None, "uptime_s": None, "log_dir": None}
        if not stack["running"]:
            self._close_orphan_episode()
        try:
            import explore_ctl
            transport = explore_ctl.transport()
        except Exception:  # noqa: BLE001
            transport = (os.environ.get("TRANSPORT") or "lcm").lower()
        return {
            "ts": time.time(), "mode": self.mode,
            "stack": stack,
            "pose": self._pose(now),
            "topics": {name: self.bus.stats(name, now) for name in STATE_TOPICS},
            "base": self._base(now),
            "nav": self._nav(now, stack),
            "recording": recording_view(self.episode, self.episode_closed),
            "system": system_metrics(),
            "bus": {"transport": transport, "listening": lcm is not None and HAVE_MSGS,
                    "error": self.bus_error},
            "camera": self.camera_state(),
            "launch": self.launcher.status(lines=False),
        }

    def map(self):
        self._sync_run(stack_status())
        raw, mono = self.bus.latest("global_costmap")
        if raw is None:
            return None
        cached_mono, cached, at = self._map_cache
        if cached is not None and (cached_mono == mono or time.monotonic() - at < MAP_MIN_INTERVAL_S):
            return cached
        result = decode_costmap(raw)
        self._map_cache = (mono, result, time.monotonic())
        return result

    def scan(self) -> dict | None:
        self._sync_run(stack_status())
        now = time.monotonic()
        raw, mono = self.bus.latest("lidar")
        name = "lidar"
        if raw is None or now - mono > SCAN_FALLBACK_S:
            fb_raw, fb_mono = self.bus.latest("pointcloud")
            if fb_raw is not None:
                raw, mono, name = fb_raw, fb_mono, "pointcloud"
        if raw is None:
            return None
        cached_key, cached, at = self._scan_cache
        if cached is not None and (cached_key == (name, mono) or now - at < SCAN_MIN_INTERVAL_S):
            return cached
        frame, stamp, xy = cloud_xy(raw)
        pts = np.round(downsample(xy).astype(float), 3).tolist()
        doc = {"frame": frame or ("world" if name == "lidar" else "lidar_link"), "stamp": stamp,
               "source": name, "age_s": round(now - mono, 2), "points": pts}
        self._scan_cache = ((name, mono), doc, now)
        return doc

    def path(self) -> dict:
        self._sync_run(stack_status())
        pts, mono = self.bus.decoded("path", decode_path)
        return {"points": pts or [], "age_s": None if mono is None else round(time.monotonic() - mono, 2)}

    def log(self, n: int) -> list[dict]:
        return log_tail(stack_status().get("log_dir"), n)

    def preflight(self) -> dict:
        return preflight_doc()

    # camera ------------------------------------------------------------------
    def _encode(self, arr: np.ndarray, encoding: str) -> bytes:
        if self._encoder is None:
            try:
                from turbojpeg import TJPF_BGR, TJPF_GRAY, TJPF_RGB, TurboJPEG
                tj = TurboJPEG()
                fmts = {"rgb8": TJPF_RGB, "bgr8": TJPF_BGR, "mono8": TJPF_GRAY}

                def enc(a, e):
                    return tj.encode(a, quality=70, pixel_format=fmts[e])
                self._encoder = ("turbojpeg", enc)
            except Exception:  # noqa: BLE001
                try:
                    import cv2

                    def enc(a, e):
                        a = a[..., ::-1] if e == "rgb8" else a
                        ok, buf = cv2.imencode(".jpg", np.ascontiguousarray(a), [cv2.IMWRITE_JPEG_QUALITY, 70])
                        if not ok:
                            raise CameraUnavailable("cv2 could not encode the frame")
                        return buf.tobytes()
                    self._encoder = ("cv2", enc)
                except Exception:  # noqa: BLE001
                    self._encoder = ("none", None)
        name, enc = self._encoder
        if enc is None:
            raise CameraUnavailable("raw camera frames and no JPEG encoder here (turbojpeg or cv2); "
                                    "put the stack's color_image on JpegLcmTransport")
        return enc(arr, encoding)

    def camera_frame(self, cam: str = "nose"):
        """(key, jpeg) of the latest frame of that camera, or None before the first one."""
        topic = CAMERAS[cam][0]
        raw, mono = self.bus.latest(topic)
        if raw is None:
            return None
        cached = self._cam_cache[cam]
        if cached[0] == mono:
            return mono, cached[1]
        m = LcmImage.lcm_decode(raw)
        enc = str(m.encoding).lower()
        data = m.data if isinstance(m.data, (bytes, bytearray)) else bytes(m.data)
        if enc == "jpeg":
            jpeg = bytes(data)        # JpegLcmTransport: the bus already carries the JPEG
        else:
            chans = {"rgb8": 3, "bgr8": 3, "mono8": 1, "rgba8": 4, "bgra8": 4}.get(enc)
            if chans is None:
                raise CameraUnavailable(f"camera encoding {m.encoding!r} not handled")
            arr = np.frombuffer(data, np.uint8)[: m.height * m.width * chans]
            arr = arr.reshape(m.height, m.width, chans) if chans > 1 else arr.reshape(m.height, m.width)
            if chans == 4:
                arr, enc = arr[..., :3], ("rgb8" if enc == "rgba8" else "bgr8")
            jpeg = self._encode(arr, enc)
        self._cam_cache[cam] = (mono, jpeg)
        return mono, jpeg

    # commands ----------------------------------------------------------------
    def _publisher(self):
        """One LCM handle for every command publish (None: explore_ctl opens its own,
        or the run is on zenoh)."""
        if lcm is None:
            return None
        with self._pub_lock:
            if self._pub_lcm is None:
                self._pub_lcm = lcm.LCM()
            return self._pub_lcm

    def command(self, name: str, body: dict) -> dict:
        if name == "stack":
            return self._stack(body)
        try:
            import explore_ctl                   # needs dimos_lcm: imported where it is used
            from dimos_lcm.std_msgs import Bool, String
        except ImportError as exc:
            return {"ok": False, "error": f"cannot publish on the bus here: {exc}", "_code": 503}
        bus = self._publisher()

        def pub(stream, msg, msg_name=None):
            with self._pub_lock:
                return explore_ctl.publish_msg(stream, msg, msg_name=msg_name, bus=bus)

        def not_moss():
            st = stack_status()
            what = f"the running stack is {st.get('blueprint')!r}" if st.get("running") else "no stack is running"
            return {"ok": False, "error": f"refused: {what}, not a MOSS stack", "_code": 409}

        if name == "explore":
            action = body.get("action")
            if action not in ("start", "stop"):
                raise ValueError('explore needs "action": "start" or "stop"')
            if self._moss_stack() is None:
                return not_moss()
            with self._goal_lock:
                self._goal_gen += 1
                return {"ok": True, "sent": pub(explore_ctl.CHANNELS[action], Bool(data=True))}
        if name == "goal":
            x, y = _finite_xy(body)
            st = self._moss_stack()
            if st is None:
                return not_moss()
            exploring = self._nav(time.monotonic(), st)["exploring"]
            with self._goal_lock:
                self._goal_gen += 1
                gen = self._goal_gen
                if not exploring:
                    sent = pub("goal_request", goal_message(x, y), msg_name="geometry_msgs.PoseStamped")
                    return {"ok": True, "sent": sent, "goal": {"x": x, "y": y}}
                # Explorer2 publishes its own goal_request: stop it first. Its stop ends with a
                # 'stay here' goal about 2 s later, which would replace ours: ours goes after it.
                first = pub(explore_ctl.CHANNELS["stop"], Bool(data=True))

            def deferred():
                with self._goal_lock:
                    if gen != self._goal_gen or self._moss_stack() is None:
                        return                    # a stop, an explore, a newer goal or no stack: drop it
                    pub("goal_request", goal_message(x, y), msg_name="geometry_msgs.PoseStamped")
            _later(GOAL_AFTER_STOP_S, deferred)
            return {"ok": True, "sent": first,
                    "queued": explore_ctl.channel("goal_request", "geometry_msgs.PoseStamped"),
                    "queued_s": GOAL_AFTER_STOP_S, "goal": {"x": x, "y": y}}
        if name == "stop":
            with self._goal_lock:
                self._goal_gen += 1               # a goal still queued behind an explore stop is dropped
            # soft stop: the firmware deadman (300 ms) and `moss-fly.sh stop` are the hard ones
            a = pub(explore_ctl.CHANNELS["stop"], Bool(data=True))
            b = pub("stop_movement", Bool(data=True))
            c = pub("tele_cmd_vel", zero_twist())
            also = [a, b]
            if stack_status().get("blueprint") != "moss-explore":
                # moss-nav has no MovementManager: the pad's tele_cmd_vel is remapped
                # onto cmd_vel and nobody listens on /tele_cmd_vel, so zero cmd_vel itself
                also.append(c)
                c = pub("cmd_vel", zero_twist())
            return {"ok": True, "sent": c, "also": also}
        if name == "episode":
            marker = self._episode(body)
            sent = pub("episode_marker", String(data=json.dumps(marker)))
            return {"ok": True, "sent": sent, "episode": self.episode}
        return {"ok": False, "error": f"unknown command {name!r}", "_code": 404}

    def _stack(self, body: dict) -> dict:
        action = body.get("action")
        if action not in ("map", "explore", "stop", "check"):
            raise ValueError('stack needs "action": "map", "explore", "stop" or "check"')
        if action in ("map", "explore"):
            st = stack_status()
            if st["running"] and not str(st.get("blueprint") or "").startswith("moss"):
                return {"ok": False, "error": f"a dimOS stack is already running ({st.get('blueprint')!r}, "
                                              "not MOSS): the MOSS cockpit leaves it alone", "_code": 409}
            if st["running"] or _dimos_run_pid():
                return {"ok": False, "error": "a dimOS stack is already running: stop it first"}
        return self.launcher.start(action)


# --- demo: a synthetic rover, for the page without a robot ------------------------------

# 320x240 JPEG (made once with cv2; studio-to-navy gradient, a mint square, "DEMO CAMERA")
_DEMO_JPEG_B64 = (
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wBDAA0JCgsKCA0LCgsODg0PEyAVExISEyccHhcgLikxMC4pLSwzOko+MzZGNywtQFdB"
    "RkxOUlNSMj5aYVpQYEpRUk//2wBDAQ4ODhMREyYVFSZPNS01T09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09PT09P"
    "T09PT09PT09PT09PT0//wAARCADwAUADASIAAhEBAxEB/8QAHwAAAQUBAQEBAQEAAAAAAAAAAAECAwQFBgcICQoL/8QAtRAA"
    "AgEDAwIEAwUFBAQAAAF9AQIDAAQRBRIhMUEGE1FhByJxFDKBkaEII0KxwRVS0fAkM2JyggkKFhcYGRolJicoKSo0NTY3ODk6"
    "Q0RFRkdISUpTVFVWV1hZWmNkZWZnaGlqc3R1dnd4eXqDhIWGh4iJipKTlJWWl5iZmqKjpKWmp6ipqrKztLW2t7i5usLDxMXG"
    "x8jJytLT1NXW19jZ2uHi4+Tl5ufo6erx8vP09fb3+Pn6/8QAHwEAAwEBAQEBAQEBAQAAAAAAAAECAwQFBgcICQoL/8QAtREA"
    "AgECBAQDBAcFBAQAAQJ3AAECAxEEBSExBhJBUQdhcRMiMoEIFEKRobHBCSMzUvAVYnLRChYkNOEl8RcYGRomJygpKjU2Nzg5"
    "OkNERUZHSElKU1RVVldYWVpjZGVmZ2hpanN0dXZ3eHl6goOEhYaHiImKkpOUlZaXmJmaoqOkpaanqKmqsrO0tba3uLm6wsPE"
    "xcbHyMnK0tPU1dbX2Nna4uPk5ebn6Onq8vP09fb3+Pn6/9oADAMBAAIRAxEAPwDBopM0ZroOcWikzRmgBaKTNGaAFopM0ZoA"
    "WikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikzRmgBaKTNGaAFopM0ZoAWikz"
    "RmgBaKTNGaAFopM0ZoAWikzRmgBM0ZpuaM0ASxxSy58uN3x12qTikkjkjOJI2Qn+8MV13geRo9O1ORcbkAYfUBqk0DX5tau2"
    "07U4YZI5EJGFxyOx/DNTcqxx0cUsufLjd8ddqk4pxtrgAkwSgDqdhrtvDVu9lLrdvbDLRSYiB78Nt/pVS6v/ABYtrK09nGsQ"
    "Ql22rwMcnrRzBY5ExyCMSFGCHo2OD+NIqs7BUUsx6ADJr0DTLSG/8G21nOwXzlKoT2bJI/lVTwfpDWU0l3fLsmLtDCp9RncR"
    "+R/I0uYOU4pgysVYEEdQe1PaGZYxI0UgQ9GKnB/Guz0uxiuvGWpzzIri3YFQezHof0NQ6V4pur/XFtZ44jazsUCbeQOcfWnz"
    "BY47NKAWICgknoBWj4ks47DXLiCFQsWQyqOwIzit7QPL0nwpcawsSPcMSFZh0GQoH0zzRfQVjkpYZYcebE6Z6blIpgySAAST"
    "0Arq7PxVDeW09tr8QkjcfKY0/wA/gap+DrJbnWvPbmG1UyEn16D/AB/Ci/cdjCkjkjOJEZCezDFAjkMZkCMUHBbHA/Guv8RS"
    "Ra54dTU7dfmtpWUjuFzj/wCJNWvCMcE3heWK5x5UkrI2ffA/rRzaBy6nCDJIABJPQCn+VL5nl+W+/wDu7Tn8q6zw9oDWetTz"
    "X2BHauEiY9HZvun9R+JFXJP+SiR/9cP/AGU0cwcpxX2W5/595v8Avg01YJmZlWKQleoCniu5v7vxQl9MtnaI1uHPlsQOR+dR"
    "+F57uQ6zNOuLsONygfxANxj6ilzBynEyRSx/6yN0/wB5SKZmvQdBudV1Lz4NbsgLcpwZIimT6YPWuU0WwjvfEq28fzW8cpcn"
    "1RTx+fA/GmpBYzHiljAMkboD03AjNMzXfa1LFrukalFAMy2EuV99o5P4/MPwrz/NCdxNWHZozTc0ZqhDs0ZpuaM0AOzRmm5o"
    "zQA7NGabmjNADs0ZpuaM0AOzRmm5ozQA7NGabmjNADs0ZpuaM0AJmjNNzRmkM7TwMhm0/U4lIDOFUZ9w1S6LoX9gXjX+p3ts"
    "qxodoVj3+oFcNmjNKw7nfeGJ21B9bmhbY00mYyeNuQ2P6VWn0DxK0Egl1dHjKncpnfBGOR0ris0ZpWC52dxO9v4E06eFtskc"
    "ysp9wWpdO1+XWvFGnBohDHEH+QNnLFDzXF5ozT5Qudzp2owWXjLUobiRUS4bCsem4dB+ppdO8MHTdYGoT3UH2OJi6fNg98Z7"
    "cVwuaXJxjPApWC5qeIr6PUNbuLiE5iJCocdQBjNb3h6W31TwzPorzpFPk7A3GRkMD781xmaM07aBc7K38NWWl201z4gmicAf"
    "u0jcjP8AIk+1T6O1povhVru+jcreuQUT7xU5AHUdsn8a4csT1OaTNFguegaHfaHfLcaVYWs1us6MWWQ8Nxg45POP5VTtontP"
    "BGpwucSQzspx6grXF5ozS5QudUniS41S70m0kjEYS4jMrBs+YQRzjtWrIf8Ai4sf/XD/ANlNcBmjNPlC56BqGj+Ip7+eW11U"
    "Rwu5KJ5zjaPTAFM8NxXFgNbjnlD3ERDM4OcttJzzXBZozS5QudzoWpzeINNvNMvbgi5ZSUkGFJH4eh/MGmeF4F0XTtQ1O/Uq"
    "Y2MWB1+U8gfU8fhXE5ozRyhc73QdX0E6h9nsLOeCS5+UtIcq3U4PzGuR1mz/ALP1a5tf4Uf5f908j9DVDNGaaVgbuOzRmm5o"
    "zTEOzRmm5ozQA7NGabmjNADs0ZpuaM0AOzRmm5ozQA7NGabmjNADs0ZpuaM0AOzRmm5ozQAlFJRQAtFJRQAtFJRQAtFJRQAt"
    "FJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQA3"
    "NGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRm"
    "kzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBtFJmjN"
    "IYtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtF"
    "JmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNACZozTaKAHZozTaKAHZozTaKAHZozTaKAHZozTaKAHZozT"
    "aKAHZozTaKAHZozTaKAHZozWhaaFqd5bJcW1tvifO1vMUZwcdz7VN/wjOs/8+f8A5FT/ABpXRDqwWjkjJzRmtb/hGdZ/58//"
    "ACKn+NH/AAjOs/8APn/5FT/Gi6F7an/MvvMnNGa1v+EZ1n/nz/8AIqf40f8ACM6z/wA+f/kVP8aLoPbU/wCZfeZOaM1rf8Iz"
    "rP8Az5/+RU/xpk3h3VoIXmltNscalmPmIcAcnvRdB7Wn/MvvMzNGabRTNB2aM02igB2aM02igB2aM02igB2aM02igB2aM02i"
    "gB2aM02igB2aM02igBKKSigYtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQAtFJRQB6R4U/5Fy0/4H/6G1a9Y/hP/kW7T/gf"
    "/obVsVi9zwq38SXqwooopGYUUUUAFU9X/wCQNff9e8n/AKCauVT1j/kDX3/XvJ/6CaaKh8SPLaKSitj3xaKSigBaKSigBaKS"
    "igBaKSigBaKSigBaKSigBaKSigBuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjN"
    "JmjNAHpXhL/kW7T/AIH/AOhtWxWN4S/5Fq0/4H/6G1bNYvc8Kt/El6sKKKKRmFFFFABVPWP+QLff9e8n/oJq5VPWf+QLff8A"
    "XvJ/6CaaKh8SPKs0ZpM0ZrY98XNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAG5ozSZ"
    "ozSAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAFzRmkzRmgBc0ZpM0ZoAXNGaTNGaAPTPCP8AyLVp/wAD/wDQ2rZrG8If"
    "8izaf8D/APQ2rZrJ7nhVv4kvVhRRRSMwooooAKpaz/yBb/8A69pP/QTV2qWs/wDIEv8A/r2k/wDQTTRUPiR5RmjNJmjNanvi"
    "5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAuaM0maM0ALmjNJmjNAC5ozSZozQAmaM1e/tG1/6Ath/33P/APHKP7Rtf+gL"
    "Yf8Afc//AMcoGUc0Zq9/aNr/ANAWw/77n/8AjlH9o2v/AEBbD/vuf/45QBRzRmr39o2v/QFsP++5/wD45R/aNr/0BbD/AL7n"
    "/wDjlAFHNGavf2ja/wDQFsP++5//AI5R/aNr/wBAWw/77n/+OUAUc0Zq9/aNr/0BbD/vuf8A+OUf2ja/9AWw/wC+5/8A45QB"
    "RzRmr39o2v8A0BbD/vuf/wCOUf2ja/8AQFsP++5//jlAFHNGavf2ja/9AWw/77n/APjlH9o2v/QFsP8Avuf/AOOUAUc0Zq9/"
    "aNr/ANAWw/77n/8AjlH9o2v/AEBbD/vuf/45QB33hD/kWbT/AIH/AOhtW1XnFp4vvLK2S3tLOyihTO1AHOMnJ5LE9TU3/Cc6"
    "p/zws/8Avhv/AIqocWeXUwlSU213PQaK8+/4TnVP+eFn/wB8N/8AFUf8Jzqn/PCz/wC+G/8AiqXKyPqdU9Borz7/AITnVP8A"
    "nhZ/98N/8VR/wnOqf88LP/vhv/iqOVh9TqnoNUta/wCQJf8A/XtJ/wCgmuL/AOE51T/nhZ/98N/8VTJvGmoTwvDNa2bxyKVd"
    "drjIPBHDUcrHHB1U0znc0Zq9/aNr/wBAWw/77n/+OUf2ja/9AWw/77n/APjlaHrFHNGavf2ja/8AQFsP++5//jlH9o2v/QFs"
    "P++5/wD45QBRzRmr39o2v/QFsP8Avuf/AOOUf2ja/wDQFsP++5//AI5QBRzRmr39o2v/AEBbD/vuf/45R/aNr/0BbD/vuf8A"
    "+OUAUc0Zq9/aNr/0BbD/AL7n/wDjlH9o2v8A0BbD/vuf/wCOUAUc0Zq9/aNr/wBAWw/77n/+OUf2ja/9AWw/77n/APjlAFHN"
    "Gavf2ja/9AWw/wC+5/8A45R/aNr/ANAWw/77n/8AjlAFHNGavf2ja/8AQFsP++5//jlH9o2v/QFsP++5/wD45QBnZozTc0Zo"
    "AdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRm"
    "gB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgBM0ZpKKB"
    "i5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5oz"
    "SUUALmjNJRQAuaM0lFAC5ozSUUALmjNJRQAuaM0lFAC5ozSUUANopM0ZpALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNA"
    "C0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0UmaM0ALRSZozQAtFJmjNAC0Um"
    "aM0ALRSZozQAmaM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzR"
    "mgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03NGaAHZozTc0ZoAdmjNNzRmgB2aM03N"
    "GaAHZozTc0ZoA//Z"
)
DEMO_JPEG = base64.b64decode("".join(_DEMO_JPEG_B64))


class DemoWorld:
    """A 12 x 12 m square corridor around a central block, mapped as the pose drives
    its loop at 0.2 m/s. Lethal walls, a cost ramp near them, unknown outside."""

    RES = 0.05
    N = 240
    OX = OY = -6.0
    SPEED = 0.2
    LOOP = ((-4.25, -4.25), (4.25, -4.25), (4.25, 4.25), (-4.25, 4.25))
    SEEN_RADIUS_M = 3.0

    def __init__(self, now: float):
        c = self.OX + (np.arange(self.N) + 0.5) * self.RES
        X, Y = np.meshgrid(c, c)                        # rows = y, from min y up
        ax, ay = np.abs(X), np.abs(Y)
        inside = (ax < 5.6) & (ay < 5.6)
        outer = inside & ((ax >= 5.4) | (ay >= 5.4))
        block = (ax <= 3.0) & (ay <= 3.0)
        block_wall = block & ((ax >= 2.85) | (ay >= 2.85))
        bin_box = (X >= -5.25) & (X <= -4.95) & (Y >= 0.3) & (Y <= 0.7)   # off the loop line
        truth = np.full((self.N, self.N), -1, dtype=np.int8)
        free = inside & ~outer & ~block
        truth[free] = 0
        d_outer = 5.4 - np.maximum(ax, ay)
        d_block = np.hypot(np.maximum(ax - 3.0, 0), np.maximum(ay - 3.0, 0))
        d_bin = np.hypot(np.maximum(np.abs(X + 5.1) - 0.15, 0), np.maximum(np.abs(Y - 0.5) - 0.2, 0))
        d = np.minimum(np.minimum(d_outer, d_block), d_bin)
        ramp = free & (d < 0.4)
        truth[ramp] = np.clip(np.rint(99 * (1 - d[ramp] / 0.4)), 1, 99).astype(np.int8)
        truth[outer | block_wall | bin_box] = 100
        self.X, self.Y, self.truth = X, Y, truth
        self.seen = np.zeros_like(truth, dtype=bool)
        self.x, self.y, self.yaw = self.LOOP[0][0], self.LOOP[0][1], 0.0
        self.corner = 1
        self.goal = None
        self.goal_active = False
        self.goal_reached = None
        self.hold_until = 0.0
        self.moving = False
        self.travelled = 0.0
        self.last = now
        self.map_stamp = now
        self._reveal(now)

    def _reveal(self, now: float) -> None:
        self._revealed_at = (self.x, self.y)
        near = np.hypot(self.X - self.x, self.Y - self.y) <= self.SEEN_RADIUS_M
        if not (near & ~self.seen).any():
            return
        self.seen |= near
        self.map_stamp = now

    def cells(self) -> np.ndarray:
        return np.where(self.seen, self.truth, np.int8(-1)).astype(np.int8)

    def target(self):
        return self.goal if self.goal_active else self.LOOP[self.corner]

    def step(self, now: float, may_patrol: bool) -> None:
        dt = min(max(now - self.last, 0.0), 1.0)
        self.last = now
        if not self.goal_active and (not may_patrol or now < self.hold_until):
            self.moving = False
            return
        tx, ty = self.target()
        dist = math.hypot(tx - self.x, ty - self.y)
        stride = self.SPEED * dt
        if dist > 1e-9:
            self.yaw = math.atan2(ty - self.y, tx - self.x)
        if dist <= stride:
            self.x, self.y = tx, ty
            if self.goal_active:
                self.goal_active, self.goal_reached = False, True
                self.hold_until = now + 5.0
                self.corner = min(range(4), key=lambda i: math.hypot(self.LOOP[i][0] - tx, self.LOOP[i][1] - ty))
            else:
                self.corner = (self.corner + 1) % 4
        else:
            self.x += stride * (tx - self.x) / dist
            self.y += stride * (ty - self.y) / dist
        self.travelled += min(dist, stride)
        self.moving = stride > 0
        if math.hypot(self.x - self._revealed_at[0], self.y - self._revealed_at[1]) > 0.25:
            self._reveal(now)

    def scan(self, max_range_m: float = 8.0) -> np.ndarray:
        """360 rays at 1 deg, ray-marched on the true walls (what a lidar would see)."""
        a = np.radians(np.arange(0.0, 360.0, 1.0))
        r = np.arange(1, int(max_range_m / self.RES)) * self.RES
        px = self.x + np.cos(a)[:, None] * r
        py = self.y + np.sin(a)[:, None] * r
        col = np.floor((px - self.OX) / self.RES).astype(int)
        row = np.floor((py - self.OY) / self.RES).astype(int)
        inb = (col >= 0) & (col < self.N) & (row >= 0) & (row < self.N)
        occ = np.zeros(px.shape, dtype=bool)
        occ[inb] = self.truth[row[inb], col[inb]] == 100
        hit = occ.any(axis=1)
        first = occ.argmax(axis=1)
        idx = np.nonzero(hit)[0]
        return np.column_stack((px[idx, first[idx]], py[idx, first[idx]])).astype(np.float32)


class DemoCockpit(Cockpit):
    """Same endpoints, synthetic sources, never touches LCM or the launcher; the
    machine metrics are real."""

    mode = "demo"

    def __init__(self, fly_script: str = FLY_SCRIPT):
        super().__init__(fly_script)
        now = time.time()
        self._lock = threading.Lock()
        self.world = DemoWorld(now)
        self.stack = {"running": True, "blueprint": "moss-explore", "started": now}
        self.exploring = True
        self.explore_done = False
        self.halted = False
        self.stack_stopped_at = None
        self._map_cache = (None, None)
        self._log: deque = deque(maxlen=200)
        self._launch_lines: deque = deque(maxlen=LAUNCH_TAIL)
        self._launch_action = None
        self._preflight_ts = now
        self._say("info", "demo cockpit: synthetic MOSS, nothing on the bus")
        self._say("info", "RPLIDAR C1 up (demo)")
        self._say("info", "lidar odometry up (demo)")

    def _say(self, level: str, event: str) -> None:
        self._log.append({"ts": time.strftime(_ISO), "level": level, "event": event, "logger": "demo"})

    def _tick(self) -> float:
        now = time.time()
        with self._lock:
            if not self.stack["running"]:            # no stack, no motion, not even toward a goal
                self.world.last, self.world.moving = now, False
                return now
            may = not self.halted and (self.exploring or self.stack["blueprint"] == "moss-nav")
            self.world.step(now, may)
        return now

    def state(self) -> dict:
        now = self._tick()
        w = self.world
        running = self.stack["running"]
        up = now - self.stack["started"]
        stack = ({"running": True, "run_id": time.strftime("%Y%m%d-%H%M%S-demo-", time.localtime(self.stack["started"]))
                  + self.stack["blueprint"], "blueprint": self.stack["blueprint"], "pid": os.getpid(),
                  "started_at": _dt.datetime.fromtimestamp(self.stack["started"], _dt.timezone.utc).isoformat(),
                  "uptime_s": round(up, 1), "log_dir": None} if running else {"running": False})
        silent = 0.0 if running else now - (self.stack_stopped_at or now)
        nominal = {"odom": 10.0, "global_costmap": 1.0, "lidar": 10.0, "pointcloud": 10.0, "path": 1.0,
                   "cmd_vel": 10.0, "base_telemetry": 2.0}
        for topic, _ in CAMERAS.values():
            nominal[topic] = 5.0
        topics = {k: {"age_s": round(silent + 1.0 / hz / 2, 2), "hz": hz if running else 0.0}
                  for k, hz in nominal.items()}
        volts = max(11.0, 11.6 - 0.0005 * (now - self.started))
        drive = w.moving and running
        pct = 40 if drive else 0
        telemetry = {"fw": "qtpy-mdd3a-v3", "driver": "mdd3a", "mode": 2 if drive else 0,
                     "reason": "drive" if drive else "boot", "bus_mV": int(volts * 1000), "ina_ok": True,
                     "ticks": [int(w.travelled * 1000), int(w.travelled * 1000)],
                     "target_pct": [pct, pct], "output_pct": [pct, pct], "ramp_ms": [5, 5],
                     "firmware_age_s": 0.1, "mock": True}
        return {
            "ts": now, "mode": self.mode, "stack": stack,
            "pose": ({"x": round(w.x, 3), "y": round(w.y, 3), "yaw": round(w.yaw, 4), "age_s": 0.05}
                     if running else None),
            "topics": topics,
            "base": base_view(telemetry, 0.3) if running else None,
            "nav": {"exploring": self.exploring and running, "explore_done": self.explore_done,
                    "goal": None if w.goal is None else {"x": round(w.goal[0], 3), "y": round(w.goal[1], 3)},
                    "goal_reached": w.goal_reached,
                    "last_cmd_vel": {"vx": w.SPEED if drive else 0.0, "wz": 0.0, "age_s": 0.1}},
            "recording": {"db": os.path.join(state_dir(), "recordings", "moss.db") + " (demo)",
                          "exists": True, "size_mb": round(0.5 + 0.01 * up, 1), "episode": self.episode},
            "system": system_metrics(),
            "bus": {"transport": "demo", "listening": False, "error": None},
            "camera": self.camera_state(),
            "launch": {"running": False, "action": self._launch_action},
        }

    def map(self):
        self._tick()
        with self._lock:
            stamp = self.world.map_stamp
            if self._map_cache[0] != stamp:
                self._map_cache = (stamp, render_grid(self.world.cells(), DemoWorld.RES, DemoWorld.OX,
                                                      DemoWorld.OY, stamp, "world"))
            return self._map_cache[1]

    def scan(self) -> dict | None:
        now = self._tick()
        if not self.stack["running"]:
            return None
        with self._lock:
            pts = downsample(self.world.scan())
        return {"frame": "world", "stamp": now, "source": "demo", "age_s": 0.05,
                "points": np.round(pts.astype(float), 3).tolist()}

    def path(self) -> dict:
        self._tick()
        w = self.world
        if not self.stack["running"] or not (w.goal_active or w.moving):
            return {"points": [], "age_s": 0.5}
        tx, ty = w.target()
        return {"points": [[round(w.x, 3), round(w.y, 3)], [round(tx, 3), round(ty, 3)]], "age_s": 0.5}

    def log(self, n: int) -> list[dict]:
        return list(self._log)[-n:]

    def preflight(self) -> dict:
        checks = [("stack", "stack: none", ""), ("ports", "QT Py present", "by-id"),
                  ("ports", "C1 lidar present", "by-id"), ("firmware", "telemetry", "qtpy-mdd3a-v3, mode 0 (off)"),
                  ("firmware", "battery", "11.62 V (INA219)"), ("lidar", "C1 spins", "1480 pts/s, 36/36 sectors"),
                  ("camera", "RealSense D455 on USB 3", "demo"), ("camera", "wrist camera", "demo"),
                  ("gamepad", "gamepad js0", "present"), ("system", "free space", "79 GB"),
                  ("system", "memory available", "3.1 GB"), ("software", "entry points", "moss-nav, moss-explore")]
        rows = [{"ok": True, "label": label, "detail": detail, "section": section}
                for section, label, detail in checks]
        return {"available": True, "demo": True, "ts": self._preflight_ts, "checks": rows,
                "ok_count": len(rows), "ko_count": 0, "summary": f"{len(rows)}/{len(rows)} OK - ready to fly"}

    def launch(self) -> dict:
        return {"running": False, "action": self._launch_action, "started_at": None, "returncode": 0,
                "lines": list(self._launch_lines)}

    def camera_frame(self, cam: str = "nose"):
        # a new key every period, so the stream really runs at 5 fps; both demo cameras
        # show the demo frame
        return int(time.time() / MJPEG_PERIOD_S), DEMO_JPEG

    def command(self, name: str, body: dict) -> dict:
        self._tick()
        with self._lock:
            w = self.world
            if name == "explore":
                action = body.get("action")
                if action not in ("start", "stop"):
                    raise ValueError('explore needs "action": "start" or "stop"')
                self.exploring = action == "start"
                self.halted = False if self.exploring else self.halted
                self._say("info", f"demo: exploration {'started' if self.exploring else 'stopped'}")
                return {"ok": True, "sent": "/explore_cmd#std_msgs.Bool" if self.exploring
                        else "/stop_explore_cmd#std_msgs.Bool"}
            if name == "goal":
                x, y = _finite_xy(body)
                self.exploring = False
                self.halted = False
                w.goal, w.goal_active, w.goal_reached = (x, y), True, None
                self._say("info", f"demo: goal ({x:+.2f}, {y:+.2f})")
                return {"ok": True, "sent": "/goal_request#geometry_msgs.PoseStamped",
                        "also": ["/stop_explore_cmd#std_msgs.Bool"], "goal": {"x": x, "y": y}}
            if name == "stop":
                self.exploring, self.halted, w.goal_active = False, True, False
                self._say("warning", "demo: soft stop")
                return {"ok": True, "sent": "/tele_cmd_vel#geometry_msgs.Twist",
                        "also": ["/stop_explore_cmd#std_msgs.Bool", "/stop_movement#std_msgs.Bool"]}
            if name == "episode":
                marker = self._episode(body)
                self._say("info", f"demo: episode {marker['action']} {marker['name']}")
                return {"ok": True, "sent": "/episode_marker#std_msgs.String", "episode": self.episode}
            if name == "stack":
                action = body.get("action")
                if action not in ("map", "explore", "stop", "check"):
                    raise ValueError('stack needs "action": "map", "explore", "stop" or "check"')
                now = time.time()
                self._launch_action = action
                self._launch_lines.append(f"=== moss-fly.sh {action}  {time.strftime(_ISO)} === (demo)")
                if action in ("map", "explore"):
                    if self.stack["running"]:
                        return {"ok": False, "error": "a dimOS stack is already running: stop it first"}
                    self.stack = {"running": True, "started": now,
                                  "blueprint": "moss-nav" if action == "map" else "moss-explore"}
                    self.exploring, self.halted = False, False
                elif action == "stop":
                    self.stack = dict(self.stack, running=False)
                    self.stack_stopped_at = now
                    self.exploring = False
                else:
                    self._preflight_ts = now
                self._say("info", f"demo: moss-fly.sh {action}")
                return {"ok": True, "sent": f"moss-fly.sh {action}"}
        return {"ok": False, "error": f"unknown command {name!r}", "_code": 404}


# --- HTTP ---------------------------------------------------------------------------

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
    ".js": "text/javascript; charset=utf-8", ".json": "application/json; charset=utf-8",
    ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg", ".jpeg": "image/jpeg",
    ".ttf": "font/ttf", ".otf": "font/otf", ".woff": "font/woff", ".woff2": "font/woff2",
    ".txt": "text/plain; charset=utf-8", ".ico": "image/x-icon",
}
PLACEHOLDER_PAGE = (b"<!doctype html><meta charset=utf-8><title>MOSS cockpit</title>"
                    b"<p>MOSS cockpit server is up; the page (tools/moss_cockpit/index.html) is not installed."
                    b" Try <a href=/state.json>/state.json</a>.</p>")


def json_safe(obj):
    """NaN/inf -> null (JSON.parse rejects NaN; a NaN goal is how dimOS cancels one),
    numpy scalars -> Python numbers."""
    if isinstance(obj, dict):
        return {str(k): json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    if isinstance(obj, np.generic):
        obj = obj.item()
    if isinstance(obj, float) and not math.isfinite(obj):
        return None
    return obj


class Handler(BaseHTTPRequestHandler):
    server_version = "moss-cockpit/1.0"
    protocol_version = "HTTP/1.1"

    @property
    def cockpit(self) -> Cockpit:
        return self.server.cockpit

    def _send(self, code: int, body: bytes, ctype: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def _json(self, code: int, payload) -> None:
        self._send(code, json.dumps(json_safe(payload), allow_nan=False).encode("utf-8"),
                   "application/json; charset=utf-8")

    def _static(self, rel: str) -> None:
        root = os.path.realpath(self.server.page_dir)
        full = os.path.realpath(os.path.join(root, unquote(rel)))
        if not full.startswith(root + os.sep) or not os.path.isfile(full):
            self._json(404, {"error": "nothing at this address"})
            return
        ctype = CONTENT_TYPES.get(os.path.splitext(full)[1].lower(), "application/octet-stream")
        with open(full, "rb") as fh:
            self._send(200, fh.read(), ctype)

    def do_GET(self) -> None:  # noqa: N802
        url = urlsplit(self.path)
        path = url.path
        try:
            if path in ("/", "/index.html"):
                index = os.path.join(self.server.page_dir, "index.html")
                if os.path.isfile(index):
                    with open(index, "rb") as fh:
                        self._send(200, fh.read(), "text/html; charset=utf-8")
                else:
                    self._send(200, PLACEHOLDER_PAGE, "text/html; charset=utf-8")
            elif path.startswith("/static/"):
                self._static(path[len("/static/"):])
            elif path == "/state.json":
                self._json(200, self.cockpit.state())
            elif path in ("/map.png", "/map_meta"):
                got = self.cockpit.map()
                if got is None:
                    self._json(404, {"error": "no costmap yet: is a mapping stack running?"})
                elif path == "/map.png":
                    self._send(200, got[0], "image/png")
                else:
                    self._json(200, got[1])
            elif path == "/scan.json":
                doc = self.cockpit.scan()
                self._json(200, doc if doc is not None else {"frame": None, "stamp": None, "points": []})
            elif path == "/path.json":
                self._json(200, self.cockpit.path())
            elif path == "/log.json":
                try:
                    n = int(parse_qs(url.query).get("n", ["40"])[0])
                except ValueError:
                    n = 40
                self._json(200, {"lines": self.cockpit.log(max(1, min(n, 500)))})
            elif path == "/preflight.json":
                self._json(200, self.cockpit.preflight())
            elif path == "/launch.json":
                self._json(200, self.cockpit.launch())
            elif path == "/camera.mjpg":
                cam = parse_qs(url.query).get("cam", ["nose"])[0]
                if cam not in CAMERAS:
                    self._json(404, {"error": f"no camera called {cam!r}: " + " or ".join(CAMERAS)})
                else:
                    self._mjpeg(cam)
            else:
                self._json(404, {"error": f"nothing at this address: {path}"})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:  # noqa: BLE001 - a page reload must never kill the server
            self.log_error("GET %s failed: %r", path, exc)
            try:
                self._json(500, {"error": f"{type(exc).__name__}: {exc}"})
            except OSError:
                pass

    do_HEAD = do_GET

    def _peer_gone(self) -> bool:
        """The viewer closed its socket. Checked while no frame is due: with a silent
        camera nothing is written, so a write error would never tell us, and the
        camera would stay subscribed for a viewer long gone."""
        try:
            readable, _, _ = select.select([self.connection], [], [], 0)
            return bool(readable) and self.connection.recv(1, socket.MSG_PEEK) == b""
        except (OSError, ValueError):
            return True

    def _mjpeg(self, cam: str = "nose") -> None:
        c = self.cockpit
        try:
            c.camera_open(cam)
        except CameraUnavailable as exc:
            self._json(503, {"error": str(exc)})
            return
        try:
            # a viewer that vanished without FIN/RST (phone off the WiFi, laptop asleep) blocks
            # the write once the buffers fill: time out and free its slot instead of waiting for TCP
            self.connection.settimeout(MJPEG_WRITE_TIMEOUT_S)
            frame = None
            deadline = time.monotonic() + FIRST_FRAME_WAIT_S
            try:
                while frame is None and time.monotonic() < deadline and not self.server.stopping:
                    frame = c.camera_frame(cam)
                    if frame is None:
                        time.sleep(0.05)
            except CameraUnavailable as exc:
                self._json(503, {"error": str(exc)})
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.close_connection = True
            if self.command == "HEAD":
                return
            sent_key, next_at = None, 0.0
            while not self.server.stopping:
                if frame is None or frame[0] == sent_key or time.monotonic() < next_at:
                    if self._peer_gone():
                        return
                    time.sleep(0.03)
                    try:
                        frame = c.camera_frame(cam)
                    except CameraUnavailable:
                        return
                    continue
                key, jpeg = frame
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                 + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                self.wfile.flush()
                sent_key, next_at = key, time.monotonic() + MJPEG_PERIOD_S
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass                                  # the viewer left
        finally:
            c.camera_close(cam)

    def do_POST(self) -> None:  # noqa: N802
        path = urlsplit(self.path).path
        if not path.startswith("/cmd/"):
            self._json(404, {"ok": False, "error": f"nothing at this address: {path}"})
            return
        name = path[len("/cmd/"):]
        if self.headers.get(CMD_HEADER[0]) != CMD_HEADER[1]:
            # a cross-site page cannot set this header without a CORS preflight, which this
            # server never grants (no OPTIONS): only the cockpit page and deliberate scripts reach here
            self.close_connection = True          # the body is not read: do not reuse the connection
            self._json(403, {"ok": False, "error": f"refused: POST /cmd/* needs the header "
                                                   f"'{CMD_HEADER[0]}: {CMD_HEADER[1]}'"})
            return
        try:
            length = int(self.headers.get("Content-Length") or "0")
            if length < 0 or length > MAX_BODY:
                raise ValueError(f"body of {length} bytes refused")
            raw = self.rfile.read(length) if length else b""
            body = json.loads(raw.decode("utf-8")) if raw.strip() else {}
            if not isinstance(body, dict):
                raise ValueError("the body must be a JSON object")
            answer = self.cockpit.command(name, body)
            code = answer.pop("_code", 200)
            self._json(code, answer)
        except ValueError as exc:                 # includes a JSON decode error
            self._json(400, {"ok": False, "error": str(exc)})
        except Exception as exc:  # noqa: BLE001
            self.log_error("POST %s failed: %r", path, exc)
            self._json(500, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})

    def log_message(self, *args) -> None:
        pass                                      # quiet: errors still go through log_error


class CockpitServer(ThreadingHTTPServer):
    daemon_threads = True                         # an open MJPEG stream must not block exit
    allow_reuse_address = True

    def __init__(self, addr, cockpit: Cockpit, page_dir: str = PAGE_DIR):
        super().__init__(addr, Handler)
        self.cockpit = cockpit
        self.page_dir = page_dir
        self.stopping = False

    def shutdown(self) -> None:
        self.stopping = True
        super().shutdown()


def make_server(bind: str = DEFAULT_BIND, port: int = DEFAULT_PORT, demo: bool = False,
                page_dir: str = PAGE_DIR, fly_script: str = FLY_SCRIPT, listen: bool = True) -> CockpitServer:
    """The server, not yet serving (port 0 = any free port: the bench uses that)."""
    cockpit = DemoCockpit(fly_script) if demo else LiveCockpit(fly_script)
    if not demo and listen:
        cockpit.start_listener()
    if any(os.path.exists(p) for p in GPU_LOAD_PATHS):
        threading.Thread(target=_gpu_sampler, name="gpu-sampler", daemon=True).start()
    return CockpitServer((bind, port), cockpit, page_dir)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="MOSS cockpit web server")
    ap.add_argument("--port", type=int, default=int(os.environ.get("MOSS_COCKPIT_PORT", DEFAULT_PORT)))
    ap.add_argument("--bind", default=DEFAULT_BIND)
    ap.add_argument("--demo", action="store_true", help="synthetic rover: no bus, no stack, no launcher")
    args = ap.parse_args(argv)
    server = make_server(args.bind, args.port, demo=args.demo)
    cockpit = server.cockpit
    if not args.demo and isinstance(cockpit, LiveCockpit) and cockpit.bus_error:
        print(f"warning: {cockpit.bus_error} - the page will show no live data", flush=True)
    print(f"MOSS cockpit ({cockpit.mode}) on http://{args.bind}:{server.server_address[1]}/", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

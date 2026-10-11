"""Cold bench for the MOSS blueprints: devices, state dir, telemetry, composition.

Rule #2: a known input gives a known output. Groups:

  A. devices      - moss_ports() on a fake /dev/serial/by-id: the QT Py and the
                    C1 resolved by glob, the sentinel when one is missing (never
                    another device, never /dev/ttyUSB0), MOSS_PORT /
                    MOSS_LIDAR_PORT win
  B. the import   - importing moss_blueprints sets ROBOT_PROFILE=moss and
                    PERSISTENT_MAP=0 by setdefault: an explicit value wins
  C. state dir    - recordings at ~/.local/state/moss/recordings/moss.db,
                    MOSS_STATE_DIR moves it, always absolute
  D. telemetry    - telemetry_payload(adapter_like, now) -> the known document
                    (driver None when the build does not send it, the age in
                    seconds, the mock flag), None for an adapter without
                    telemetry (VECTOR); the real adapter on the fake firmware
                    -> 11.6 V, fw qtpy-mdd3a-v3, JSON round trip
  E. the camera   - the base_link -> camera_link mount from the profile
                    (pitch -> quaternion, VECTOR's 1.1 deg mount as reference)
  F. entry points - pyproject declares moss-nav / moss-explore on the lazy
                    builders, the VECTOR entries are unchanged
  G. blueprints   - (dimOS) moss-nav and moss-explore build: module names,
                    remaps, JPEG camera transport, recorder path, explorer
                    clearances from the MOSS profile, no gamepad -> cmd_vel
                    remap in explore; switches MOSS_CAMERA=0 / MOSS_RECORD=0;
                    MossMemoryConfig defaults; base_telemetry on the coordinator

A-F need nothing. G needs dimOS: run this on the Jetson.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_moss_blueprints_cold.py
"""
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

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


CLEARED = [k for k in os.environ if k.startswith("MOSS_")] + ["ROBOT_PROFILE", "PERSISTENT_MAP"]
for var in CLEARED:
    os.environ.pop(var, None)

from vector_dimos import robot_profile as rp  # noqa: E402
from vector_dimos import moss_blueprints as mb  # noqa: E402

QTPY = "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_f4:12:fa:8d:a7:58-if00"
C1 = "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_62c3406e536eef119af7e3c2c169b110-if00-port0"

# ── A. devices ────────────────────────────────────────────────────────────
print("A. moss_ports() on a fake /dev/serial/by-id")
saved_root = rp.BYID_ROOT
byid = tempfile.mkdtemp(prefix="byid_")
rp.BYID_ROOT = byid
ports = mb.moss_ports()
check("empty by-id -> both sentinels",
      ports == {"base": "/dev/serial/by-id/MISSING-QT-PY", "lidar": "/dev/serial/by-id/MISSING-C1"}, f"{ports}")
open(os.path.join(byid, C1), "w").close()
ports = mb.moss_ports()
check("C1 only -> lidar resolved, base still the sentinel (the CP2102N is never the base)",
      ports["lidar"] == os.path.join(byid, C1) and ports["base"] == rp.MOSS_QTPY_MISSING, f"{ports}")
open(os.path.join(byid, QTPY), "w").close()
ports = mb.moss_ports()
check("both plugged -> both by-id paths",
      ports == {"base": os.path.join(byid, QTPY), "lidar": os.path.join(byid, C1)},
      f"base {os.path.basename(ports['base'])}, lidar {os.path.basename(ports['lidar'])}")
os.environ["MOSS_PORT"] = "/dev/ttyACM7"
os.environ["MOSS_LIDAR_PORT"] = "/dev/ttyUSB7"
check("MOSS_PORT / MOSS_LIDAR_PORT win over the glob",
      mb.moss_ports() == {"base": "/dev/ttyACM7", "lidar": "/dev/ttyUSB7"})
os.environ.pop("MOSS_PORT")
os.environ.pop("MOSS_LIDAR_PORT")
check("explicit env dict is honoured (no os.environ read)",
      mb.moss_ports({"MOSS_PORT": "/x", "MOSS_LIDAR_PORT": "/y"}) == {"base": "/x", "lidar": "/y"})
check("no resolution ever lands on /dev/ttyUSB0",
      all("ttyUSB0" not in v for v in mb.moss_ports().values()))


def refused(env):
    try:
        mb.moss_ports(env)
    except ValueError as exc:
        return str(exc)
    return None


c1_path = os.path.join(byid, C1)
msg = refused({"MOSS_PORT": c1_path})
check("MOSS_PORT = the C1's by-id path (lidar from the glob) -> ValueError naming the device",
      msg is not None and "same device" in msg and C1 in msg, f"{msg}")
alias = os.path.join(byid, "alias-to-c1")
os.symlink(c1_path, alias)
check("MOSS_PORT = a symlink to the C1, MOSS_LIDAR_PORT = the C1 -> refused (realpath compare)",
      refused({"MOSS_PORT": alias, "MOSS_LIDAR_PORT": c1_path}) is not None)
check("the plugged QT Py + C1 -> no refusal", refused({}) is None)

# ── B. import side effects ───────────────────────────────────────────────
print("B. importing moss_blueprints: ROBOT_PROFILE=moss, PERSISTENT_MAP=0 (setdefault)")
check("ROBOT_PROFILE set to moss by the import", os.environ.get("ROBOT_PROFILE") == "moss")
check("PERSISTENT_MAP set to 0 by the import (no relocalization against VECTOR's flat)",
      os.environ.get("PERSISTENT_MAP") == "0")
check("so the active profile is MOSS", rp.current().name == "moss")
probe = ("import os; import vector_dimos.moss_blueprints; "
         "print(os.environ['ROBOT_PROFILE'], os.environ['PERSISTENT_MAP'])")
env = {k: v for k, v in os.environ.items() if k not in CLEARED}
env.update({"ROBOT_PROFILE": "vector", "PERSISTENT_MAP": "1", "PYTHONPATH": str(ROOT),
            "PYTHONDONTWRITEBYTECODE": "1"})
out = subprocess.run([sys.executable, "-c", probe], env=env, capture_output=True, text=True, timeout=60)
check("an explicit ROBOT_PROFILE / PERSISTENT_MAP wins (setdefault, not overwrite)",
      out.stdout.split() == ["vector", "1"], (out.stdout + out.stderr).strip()[-200:])
try:
    mb.no_such_blueprint
    check("unknown attribute raises AttributeError", False)
except AttributeError:
    check("unknown attribute raises AttributeError", True)

# ── C. state dir ─────────────────────────────────────────────────────────
print("C. the recording path")
home_db = Path.home() / ".local/state/moss/recordings/moss.db"
check("default ~/.local/state/moss/recordings/moss.db", rp.moss_recording_path() == home_db, str(rp.moss_recording_path()))
os.environ["MOSS_STATE_DIR"] = "/data/moss_state"
check("MOSS_STATE_DIR=/data/moss_state -> /data/moss_state/recordings/moss.db",
      rp.moss_recording_path() == Path("/data/moss_state/recordings/moss.db"))
os.environ["MOSS_STATE_DIR"] = "~/elsewhere"
check("MOSS_STATE_DIR=~/elsewhere -> expanded", rp.moss_state_dir() == Path.home() / "elsewhere")
os.environ["MOSS_STATE_DIR"] = "relative/state"
check("a relative MOSS_STATE_DIR is made absolute (never site-packages)",
      rp.moss_recording_path().is_absolute(), str(rp.moss_recording_path()))
os.environ.pop("MOSS_STATE_DIR")

# ── D. telemetry payload ─────────────────────────────────────────────────
print("D. telemetry_payload(adapter_like, now)")
from vector_dimos.moss_adapter import MossBaseAdapter, MossGeometry, telemetry_payload  # noqa: E402
from vector_dimos.moss_mock import FakeMossFirmware  # noqa: E402


class FakeAdapter:
    def __init__(self, telemetry, age, mock=False):
        self.telemetry, self.telemetry_age_s, self.mock_bus = telemetry, age, mock


v3_line = {"ms": 1234, "fw": "qtpy-mdd3a-v3", "mode": 2, "reason": "drive",
           "target_pct": [40, -40], "output_pct": [35, -35], "ramp_ms": [5, 5],
           "ticks": [160585, 158263], "invalid_edges": [0, 0], "ina_ok": True,
           "bus_mV": 11208, "shunt_uV": 0, "current_mA": None}
doc = telemetry_payload(FakeAdapter(v3_line, 0.3), 1760000000.0)
expected = {"ts": 1760000000.0, "fw": "qtpy-mdd3a-v3", "driver": None, "mode": 2, "reason": "drive",
            "bus_mV": 11208, "ina_ok": True, "ticks": [160585, 158263], "target_pct": [40, -40],
            "output_pct": [35, -35], "ramp_ms": [5, 5], "firmware_age_s": 0.3, "mock": False}
check("v3 line, age 0.3 s, now 1760000000 -> the known document", doc == expected,
      "" if doc == expected else f"{doc}")
check("exactly the spec's keys, in order", list(doc) == ["ts", "fw", "driver", "mode", "reason", "bus_mV", "ina_ok",
                                                          "ticks", "target_pct", "output_pct", "ramp_ms",
                                                          "firmware_age_s", "mock"])
mdd10a = dict(v3_line, driver="mdd10a")
check("an MDD10A build's driver key is carried", telemetry_payload(FakeAdapter(mdd10a, 0.0), 0.0)["driver"] == "mdd10a")
check("no telemetry yet -> None (nothing published)", telemetry_payload(FakeAdapter(None, None), 0.0) is None)
check("an adapter without the attribute (VECTOR's) -> None", telemetry_payload(object(), 0.0) is None)
check("no adapter at all -> None", telemetry_payload(None, 0.0) is None)
check("age unknown -> firmware_age_s None", telemetry_payload(FakeAdapter(v3_line, None), 0.0)["firmware_age_s"] is None)

fw = FakeMossFirmware()
factory = lambda: fw  # noqa: E731
factory.is_mock = True
a = MossBaseAdapter(port_factory=factory, geometry=MossGeometry())
check("real adapter on the fake firmware connects", a.connect())
time.sleep(0.12)
t0 = time.time()
doc = telemetry_payload(a, t0)
check("-> fw qtpy-mdd3a-v3, 11600 mV (11.6 V), ina_ok, mode 0, mock True",
      doc is not None and doc["fw"] == "qtpy-mdd3a-v3" and doc["bus_mV"] == 11600 and doc["ina_ok"] is True
      and doc["mode"] == 0 and doc["mock"] is True and doc["driver"] is None, f"{doc}")
check("firmware_age_s ~0.12 s after the connect probe", doc is not None and 0.05 <= doc["firmware_age_s"] < 0.5,
      f"{doc and doc['firmware_age_s']}")
check("JSON round trip keeps every value", json.loads(json.dumps(doc)) == doc)
a.disconnect()

# ── E. camera mount ──────────────────────────────────────────────────────
print("E. camera_mount_xyzq(profile)")
m = mb.camera_mount_xyzq(rp.MOSS)
check("MOSS: (0.156, 0, 0.075) m, identity rotation (pitch 0)", m == (0.156, 0.0, 0.075, 0.0, 0.0, 0.0, 1.0), f"{m}")
from dataclasses import replace  # noqa: E402
m = mb.camera_mount_xyzq(replace(rp.MOSS, camera_pitch_deg=1.1))
check("1.1 deg nose down -> qy 0.00959916, qw 0.99995393 (VECTOR's camera_mount numbers)",
      close(m[4], 0.00959916, 1e-8) and close(m[6], 0.99995393, 1e-8), f"qy {m[4]:.8f} qw {m[6]:.8f}")
check("unit quaternion", close(math.hypot(m[4], m[6]), 1.0, 1e-12))

# ── F. entry points ──────────────────────────────────────────────────────
# ── H. wrist camera index ────────────────────────────────────────────────
print("H. wrist_camera_index(env, by_id_dir)")
v4l = tempfile.mkdtemp(prefix="v4l_")
check("empty by-id dir, no MOSS_WRIST_DEV -> None (no wrist camera)", mb.wrist_camera_index({}, v4l) is None)
os.symlink("/dev/video6", os.path.join(v4l, "usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0"))
os.symlink("/dev/video7", os.path.join(v4l, "usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index1"))
check("KD-USB by-id video-index0 -> /dev/video6 -> 6 (index1 ignored)", mb.wrist_camera_index({}, v4l) == 6)
check("MOSS_WRIST_DEV=3 -> 3", mb.wrist_camera_index({"MOSS_WRIST_DEV": "3"}, v4l) == 3)
check("MOSS_WRIST_DEV=/dev/video9 -> 9", mb.wrist_camera_index({"MOSS_WRIST_DEV": "/dev/video9"}, v4l) == 9)
check("MOSS_WRIST_DEV=/dev/ttyUSB0 (not a video device) -> None", mb.wrist_camera_index({"MOSS_WRIST_DEV": "/dev/ttyUSB0"}, v4l) is None)
mb.V4L_BY_ID = v4l          # section G builds the blueprints against this fake by-id
print()

print("F. pyproject entry points")
text = (ROOT / "pyproject.toml").read_text()
section = text.split('[project.entry-points."dimos.blueprints"]', 1)[1].split("\n[", 1)[0]
eps = dict(re.findall(r'^([a-z0-9-]+)\s*=\s*"([^"]+)"', section, re.M))
check("moss-nav -> vector_dimos.moss_blueprints:moss_nav_blueprint",
      eps.get("moss-nav") == "vector_dimos.moss_blueprints:moss_nav_blueprint")
check("moss-explore -> vector_dimos.moss_blueprints:moss_explore_blueprint",
      eps.get("moss-explore") == "vector_dimos.moss_blueprints:moss_explore_blueprint")
check("both targets have a lazy builder", set(mb._BUILDERS) == {"moss_nav_blueprint", "moss_explore_blueprint"})
vector_eps = {"base": "vector_dimos.blueprints:base_blueprint", "explore": "vector_dimos.nav_blueprints:explore_blueprint",
              "nav": "vector_dimos.nav_blueprints:nav_blueprint", "gamepad": "vector_dimos.blueprints:gamepad_blueprint",
              "cockpit": "vector_dimos.blueprints:cockpit_blueprint",
              "moss-gamepad": "vector_dimos.blueprints:moss_gamepad_blueprint"}
check("VECTOR's entries unchanged", all(eps.get(k) == v for k, v in vector_eps.items()))
check("names match dimOS's ^[a-z0-9]+(-[a-z0-9]+)*$", all(re.match(r"^[a-z0-9]+(-[a-z0-9]+)*$", k) for k in eps))

# ── G. blueprints (dimOS) ────────────────────────────────────────────────
print("G. moss-nav / moss-explore blueprints")
try:
    import dimos.core.coordination.blueprints  # noqa: F401
    HAVE_DIMOS = True
except Exception:  # noqa: BLE001 - no dimOS on the rig
    HAVE_DIMOS = False
if not HAVE_DIMOS:
    print("  -- skipped, no dimOS here: run this on the Jetson")
else:
    from dimos.core.coordination.blueprints import TransportSpec
    from dimos.core.transport import JpegLcmTransport
    from dimos.msgs.sensor_msgs.Image import Image
    from vector_dimos.blueprints import VectorControlCoordinator
    from vector_dimos.gamepad import GamepadTeleop
    from vector_dimos.moss_memory import MossMemoryConfig, MossRecorder

    def names(bp):
        return {b.module.__name__ for b in bp.active_blueprints}

    def atom(bp, cls_name):
        return next(b for b in bp.active_blueprints if b.module.__name__ == cls_name)

    nav = mb.build_moss_nav()
    n = names(nav)
    want = {"VectorControlCoordinator", "GamepadTeleop", "RPLidarC1", "LidarOdometry",
            "VectorCostMap", "RealSenseCamera", "MossRecorder"}
    check("moss-nav modules", want <= n, f"{sorted(n)}")
    check("moss-nav has no planner, no explorer, no MovementManager, no viewer",
          not ({"RecoveringPlanner", "Explorer2", "MovementManager", "WebsocketVisModule"} & n))
    coord = nav._instance_key(VectorControlCoordinator)
    pad = nav._instance_key(GamepadTeleop)
    check("coordinator instance name ControlCoordinator", coord == "ControlCoordinator")
    check("remap twist_command -> cmd_vel", nav.remapping_map.get((coord, "twist_command")) == "cmd_vel")
    check("remap pad tele_cmd_vel -> cmd_vel (direct drive)", nav.remapping_map.get((pad, "tele_cmd_vel")) == "cmd_vel")
    spec = nav.transport_map.get(("color_image", Image))
    check("color_image crosses the bus as JPEG on /color_image",
          isinstance(spec, TransportSpec) and spec.cls is JpegLcmTransport and spec.args[0] == "/color_image")
    lid = atom(nav, "RPLidarC1").kwargs
    check("lidar on the CP2102N by-id at 460800, frame lidar_link",
          lid.get("port") == os.path.join(byid, C1) and lid.get("baudrate") == 460800 and lid.get("frame_id") == "lidar_link")
    hw = atom(nav, "VectorControlCoordinator").kwargs["hardware"][0]
    check("base: adapter 'moss' on the QT Py by-id", hw.adapter_type == "moss" and hw.address == os.path.join(byid, QTPY))
    check("odometry without the gyro prior", atom(nav, "LidarOdometry").kwargs.get("use_gyro_prior") is False)
    cam = atom(nav, "RealSenseCamera").kwargs
    check("RealSense 640x480 @ 5 fps, colour only",
          (cam.get("width"), cam.get("height"), cam.get("fps"), cam.get("enable_depth"),
           cam.get("enable_pointcloud"), cam.get("enable_imu")) == (640, 480, 5, False, False, False))
    t = cam["base_transform"]
    check("camera mount from the profile: (0.156, 0, 0.075) m",
          close(t.translation.x, 0.156) and close(t.translation.z, 0.075))
    check("recorder at the MOSS state dir", atom(nav, "MossRecorder").kwargs.get("db_path") == str(home_db))
    check("wrist camera module WristCamera in moss-nav (fake by-id -> /dev/video6)", "WristCamera" in n, f"{sorted(n)}")
    wc = atom(nav, "WristCamera").kwargs
    check("wrist: Webcam hardware factory on index 6, 640x480 @ 5 fps, frame prefix 'wrist', no TF, 5 Hz barrier",
          getattr(wc.get("hardware"), "keywords", {}).get("camera_index") == 6
          and getattr(wc.get("hardware"), "keywords", {}).get("width") == 640
          and getattr(wc.get("hardware"), "keywords", {}).get("fps") == 5.0
          and getattr(wc.get("hardware"), "keywords", {}).get("frame_id_prefix") == "wrist"
          and wc.get("transform") is None and wc.get("frequency") == 5.0, f"{wc}")
    wk = mb.wrist_instance(nav)
    check("wrist instance name found (dimOS lowercases it)", wk == "wristcamera", f"{wk}")
    check("wrist outputs remapped: color_image -> wrist_image, camera_info -> wrist_camera_info, tf -> wrist_tf",
          nav.remapping_map.get((wk, "color_image")) == "wrist_image"
          and nav.remapping_map.get((wk, "camera_info")) == "wrist_camera_info"
          and nav.remapping_map.get((wk, "tf")) == "wrist_tf", f"{dict(nav.remapping_map)}")
    wspec = nav.transport_map.get(("wrist_image", Image))
    check("wrist_image crosses the bus as JPEG on /wrist_image",
          isinstance(wspec, TransportSpec) and wspec.cls is JpegLcmTransport and wspec.args[0] == "/wrist_image")
    check("the RealSense keeps its own names (no remap on RealSenseCamera)",
          not any(k[0] == "RealSenseCamera" for k in nav.remapping_map))

    exp = mb.build_moss_explore()
    e = names(exp)
    check("moss-explore adds RecoveringPlanner, MovementManager, Explorer2",
          {"RecoveringPlanner", "MovementManager", "Explorer2"} <= e and want <= e, f"{sorted(e)}")
    pad_e = exp._instance_key(GamepadTeleop)
    check("moss-explore: NO pad -> cmd_vel remap (MovementManager arbitrates)",
          (pad_e, "tele_cmd_vel") not in exp.remapping_map)
    check("moss-explore: twist_command -> cmd_vel", exp.remapping_map.get(("ControlCoordinator", "twist_command")) == "cmd_vel")
    ex = atom(exp, "Explorer2").kwargs
    check("explorer clearances = derived(MOSS): 0.22 / 0.25 m",
          close(ex.get("lethal_clearance_m"), 0.22) and close(ex.get("pivot_clearance_m"), 0.25), f"{ex}")
    pl = atom(exp, "RecoveringPlanner").kwargs
    check("planner footprint = MOSS width 0.34, pivot 0.50",
          pl.get("robot_width") == 0.34 and pl.get("robot_rotation_diameter") == 0.50)

    os.environ["MOSS_CAMERA"] = "0"
    os.environ["MOSS_RECORD"] = "0"
    bare = mb.build_moss_nav()
    check("MOSS_CAMERA=0 MOSS_RECORD=0 -> no RealSense, no recorder, no JPEG transport",
          not ({"RealSenseCamera", "MossRecorder"} & names(bare)) and ("color_image", Image) not in bare.transport_map)
    os.environ["MOSS_WRIST"] = "0"
    bare2 = mb.build_moss_nav()
    check("MOSS_WRIST=0 -> no WristCamera, no wrist remap, no wrist transport",
          "WristCamera" not in names(bare2) and mb.wrist_instance(bare2) is None
          and ("wrist_image", Image) not in bare2.transport_map)
    os.environ.pop("MOSS_WRIST")
    mb.V4L_BY_ID = tempfile.mkdtemp(prefix="v4l_empty_")
    nowrist = mb.build_moss_nav()
    check("wrist camera absent from by-id -> module skipped, blueprint still builds",
          "WristCamera" not in names(nowrist) and ("wrist_image", Image) not in nowrist.transport_map)
    mb.V4L_BY_ID = v4l
    os.environ["MOSS_RERUN"] = "1"
    check("MOSS_RERUN=1 -> the stock viewers ride along", "WebsocketVisModule" in names(mb.build_moss_nav()))
    for k in ("MOSS_CAMERA", "MOSS_RECORD", "MOSS_RERUN"):
        os.environ.pop(k, None)
    empty = tempfile.mkdtemp(prefix="byid_empty_")
    rp.BYID_ROOT = empty
    missing = mb.build_moss_nav()
    shutil.rmtree(empty, ignore_errors=True)
    check("devices absent -> the sentinels reach the modules (fail loudly, no fallback)",
          atom(missing, "RPLidarC1").kwargs["port"] == rp.MOSS_LIDAR_MISSING
          and atom(missing, "VectorControlCoordinator").kwargs["hardware"][0].address == rp.MOSS_QTPY_MISSING)

    cfg = MossMemoryConfig()
    check("MossMemoryConfig default db_path ~/.local/state/moss/recordings/moss.db", Path(cfg.db_path) == home_db, str(cfg.db_path))
    check("poseless: global_costmap, cmd_vel, base_telemetry, episode_marker; record_tf False",
          cfg.poseless_streams == ["global_costmap", "cmd_vel", "base_telemetry", "episode_marker"] and cfg.record_tf is False)
    os.environ["MOSS_STATE_DIR"] = "/data/moss_state"
    check("MOSS_STATE_DIR read when the config is built", Path(MossMemoryConfig().db_path) == Path("/data/moss_state/recordings/moss.db"))
    os.environ.pop("MOSS_STATE_DIR")
    ins = set(MossRecorder.__annotations__) | set(MossRecorder.__mro__[1].__annotations__)
    check("MossRecorder records odom, global_costmap, lidar, color_image, cmd_vel, base_telemetry, episode_marker",
          {"odom", "global_costmap", "lidar", "color_image", "cmd_vel", "base_telemetry", "episode_marker"} <= ins)
    check("VectorControlCoordinator has the base_telemetry Out",
          "base_telemetry" in VectorControlCoordinator.__annotations__)
    check("its start/stop overrides keep @rpc (else dimOS pickles the module to call them)",
          {"start", "stop"} <= set(VectorControlCoordinator.rpcs))

rp.BYID_ROOT = saved_root
shutil.rmtree(byid, ignore_errors=True)

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

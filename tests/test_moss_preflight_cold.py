#!/usr/bin/env python3
"""Cold bench for tools/moss_preflight.py: the MOSS flight check, every probe
stubbed, so what is proven is the VERDICT each world gets.

Rule #2: known input -> known output in physical units (battery mV -> volts,
lidar points/s, GB free). No robot, no serial port, no dimOS: every probe of
the module is replaced by a counting fake, so the bench also proves which
devices the check did NOT open. Groups:

  A. all green      - exit 0, "N/N OK - ready to fly", 11612 mV -> 11.61 V,
                      the QT Py opened once at 115200 and not one byte written,
                      the lidar probed once on the CP2102N path
  B. QT Py missing  - KO on the board, firmware skipped, the port never opened
  C. stack running  - registry entry with a live pid (and an unregistered
                      `bin/dimos run`): firmware and lidar SKIPPED "held by the
                      stack", zero opens; a dead registry pid is only stale
  D. battery        - 10500 mV -> KO with the BMS sentence; 10100 -> under the
                      cut; 10800 -> OK 10.80 V; ina_ok false -> INA219 not seen
  E. gamepad        - js0 / 2563:0526 / 057e:2009 (the Nintendo sentence) / none
  F. --json         - stdout is ONE JSON document; counts match the checks
  G. --quick / --no-spin - which stubs are never called
  H. firmware and port faults - held port, armed firmware, mute firmware,
                      RealSense on USB 2, MOSS_PORT pointing at the lidar
  I. software and env - PERSISTENT_MAP 0/unset/1, missing entry point, the
                      VECTOR profile exported by mistake, MOSS_MOCK_BUS=1
  J. --save         - the file holds the same document (what the cockpit reads)
  K. real probes on this machine - /proc fd scan finds a child holding a file,
                      sysfs USB ids are vvvv:pppp, our own cmdline is listed

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_moss_preflight_cold.py
"""

import contextlib
import fnmatch
import io
import json
import os
import subprocess
import sys
import tempfile
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import moss_preflight as mp  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


TMP = tempfile.TemporaryDirectory()
TMPD = Path(TMP.name)
QTPY = "/dev/serial/by-id/usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00"
C1 = "/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0000-if00-port0"
WRIST = "/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._KD-USB_Cameras_KD-USB_Cameras-video-index0"
TELEMETRY = {"ms": 1000, "fw": "qtpy-mdd3a-v3", "mode": 0, "reason": "boot",
             "target_pct": [0, 0], "output_pct": [0, 0], "ramp_ms": [5, 5],
             "ticks": [160585, 158263], "invalid_edges": [0, 0], "ina_ok": True,
             "bus_mV": 11612, "shunt_uV": 0, "current_mA": None}
D455 = {"name": "Intel RealSense D455F", "serial": "000000000001", "firmware": "5.17.0.10", "usb": "3.2"}


class FakeSerial:
    def __init__(self, world, port, baud):
        self.world, self.port, self.baud = world, port, baud
        self.chunks = list(world["serial_chunks"])
        world["opens"].append((port, baud))

    def reset_input_buffer(self):
        pass

    def read(self, n):
        if self.chunks:
            return self.chunks.pop(0)
        time.sleep(0.005)
        return b""

    def write(self, data):
        self.world["written"].append(data)
        return len(data)

    def close(self):
        self.world["closed"] += 1


def default_world():
    return {
        "files": {QTPY, C1, WRIST, "/dev/input/js0"},
        "runs": [],                 # (run_id, pid)
        "alive": set(),
        "cmdlines": [(1, "/sbin/init")],
        "holders": {},
        "telemetry": dict(TELEMETRY),
        "serial_chunks": None,      # None = built from "telemetry"
        "usb": {"239a:8119", "10c4:ea60", "8086:0b5c", "2c7f:4a60"},
        "free_gb": 79.0,
        "mem_kb": 6_100_000,
        "rmem": "212992",
        "lo_flags": 0x9,
        "lidar": {"info": {"model": 65, "serial": "ABCD"}, "health": ("Good", 0),
                  "turns": 3, "points_per_s": 1500.0, "hz": 10.0, "sectors": 36},
        "lidar_exc": None,
        "realsense": [dict(D455)],
        "imports_missing": set(),
        "entry_points": {"moss-nav", "moss-explore", "moss-gamepad", "moss-base", "explore"},
        "profile": "moss",
        "env": {"PERSISTENT_MAP": "0"},
        # counters
        "opens": [], "written": [], "closed": 0, "lidar_calls": [], "holder_calls": [],
    }


ENV_KEYS = ("MOSS_PORT", "MOSS_LIDAR_PORT", "MOSS_MOCK_BUS", "MOSS_RS_SERIAL", "MOSS_CAMERA",
            "PERSISTENT_MAP", "ROBOT_PROFILE", "MOSS_STATE_DIR")


def run(argv=(), **over):
    w = default_world()
    w.update(over)
    if w["serial_chunks"] is None:
        line = json.dumps(w["telemetry"]).encode() + b"\n"
        w["serial_chunks"] = [b"MOSS V0.4 firmware banner\n" + line[:40], line[40:] + line, line]
    runs_dir = TMPD / f"runs_{len(list(TMPD.glob('runs_*')))}"
    runs_dir.mkdir()
    for run_id, pid in w["runs"]:
        (runs_dir / f"{run_id}.json").write_text(json.dumps(
            {"run_id": run_id, "pid": pid, "blueprint": "vector-dimos.moss-nav",
             "started_at": "2026-10-11T00:00:00+00:00", "log_dir": "/x"}))
    state = TMPD / f"state_{runs_dir.name}"

    mp._ls = lambda pat: sorted(f for f in w["files"] if fnmatch.fnmatch(f, pat))
    mp._exists = lambda p: p in w["files"]
    mp._runs_dir = lambda: runs_dir
    mp._pid_alive = lambda pid: pid in w["alive"]
    mp._proc_cmdlines = lambda: list(w["cmdlines"])
    mp._port_holders = lambda dev: (w["holder_calls"].append(dev), w["holders"].get(dev, []))[1]
    mp._open_serial = lambda port, baud: FakeSerial(w, port, baud)
    mp._usb_devices = lambda: set(w["usb"])
    mp._statvfs = lambda path: types.SimpleNamespace(f_bavail=int(w["free_gb"] * 1e9 / 4096), f_frsize=4096)
    mp._meminfo = lambda: {"MemTotal": 7_600_000, "MemAvailable": w["mem_kb"]}
    mp._sysctl = lambda name: w["rmem"]
    mp._lo_flags = lambda: w["lo_flags"]

    def lidar(port):
        w["lidar_calls"].append(port)
        if w["lidar_exc"]:
            raise w["lidar_exc"]
        return dict(w["lidar"])
    mp._lidar_probe = lidar
    mp._realsense_probe = lambda: [dict(d) for d in w["realsense"]]

    def imp(name):
        if name in w["imports_missing"]:
            raise ModuleNotFoundError(f"No module named '{name}'")
        return types.SimpleNamespace(__version__="9.9")
    mp._import = imp
    mp._entry_points = lambda group="dimos.blueprints": set(w["entry_points"])
    mp._robot_profile = lambda: (types.SimpleNamespace(name=w["profile"], robot_width_m=0.34,
                                                       pivot_diameter_m=0.5, lidar_height_m=0.2),
                                 {"lethal_clearance_m": 0.22, "pivot_clearance_m": 0.25})
    mp.LISTEN_S = 0.05

    saved = {k: os.environ.get(k) for k in ENV_KEYS}
    for k in ENV_KEYS:
        os.environ.pop(k, None)
    os.environ["MOSS_STATE_DIR"] = str(state)
    os.environ.update(w["env"])
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            rc = mp.main(list(argv))
    finally:
        for k, v in saved.items():
            if v is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = v
    w["state"] = state
    return rc, buf.getvalue(), list(mp.RESULTS), w


def find(results, label):
    return [r for r in results if r["label"] == label]


def line_of(out, label):
    for ln in out.splitlines():
        if f" {label}" in ln and (ln.startswith("  OK") or ln.startswith("  KO") or ln.startswith("  --")):
            return ln.strip()
    return ""


def last_line(out):
    lines = [ln for ln in out.strip().splitlines() if ln.strip()]
    return lines[-1] if lines else ""


HEADERS = ("ports (by-id)", "firmware (QT Py, listen only)", "lidar (RPLidar C1, ~5 s spin)",
           "camera (RealSense, enumeration only)", "wrist camera (UVC)", "gamepad", "system", "software", "state dir")

print("A. all green")
rc, out, res, w = run()
counted = [r for r in res if not r.get("skipped")]
check("exit 0", rc == 0, f"rc {rc}")
check("summary N/N OK - ready to fly", last_line(out) == f"{len(counted)}/{len(counted)} OK - ready to fly", last_line(out))
check("no KO line at all", "  KO " not in out)
check("battery 11612 mV -> 11.61 V", line_of(out, "battery") == "OK  battery - 11.61 V", line_of(out, "battery"))
check("firmware version read from telemetry", "qtpy-mdd3a-v3" in line_of(out, "firmware version"))
check("firmware disarmed: mode 0", "mode 0 (off)" in line_of(out, "firmware disarmed"))
check("ticks L 160585 R 158263", "L 160585 R 158263" in line_of(out, "encoder ticks"))
check("driver absent -> informational OK with the build note",
      line_of(out, "motor driver").startswith("OK") and "absent on builds before 10/10 16h" in out)
check("telemetry parsed across chunk boundaries (3 lines, banner ignored)",
      "3 line(s)" in line_of(out, "firmware telemetry"), line_of(out, "firmware telemetry"))
check("QT Py opened ONCE, at 115200, on the by-id path", w["opens"] == [(QTPY, 115200)], str(w["opens"]))
check("not one byte written to the firmware", w["written"] == [], str(w["written"]))
check("port closed after listening", w["closed"] == 1)
check("lidar probed once on the CP2102N path", w["lidar_calls"] == [C1], str(w["lidar_calls"]))
check("lidar 1500 points/s, 36/36 sectors", "1500 points/s, 36/36 sectors" in line_of(out, "lidar scan"))
check("RealSense on USB 3.2 with serial and firmware",
      "serial 000000000001 fw 5.17.0.10 USB 3.2" in line_of(out, "RealSense D455"))
check("disk 79.0 GB free", "79.0 GB free" in line_of(out, "disk for recordings"), line_of(out, "disk for recordings"))
check("RAM 6.1 GB", "6.1 GB" in line_of(out, "RAM available"))
check("rmem 212992 is a warning, never a KO",
      line_of(out, "socket buffers").startswith("OK") and "warning" in line_of(out, "socket buffers"))
check("lo without MULTICAST: OK with the dimos-fixes-it note",
      line_of(out, "loopback multicast") == "OK  loopback multicast - off - dimos will fix with sudo on first run")
check("the state dir and recordings/ were created", (w["state"] / "recordings").is_dir())
check("sections in the spec order",
      [r["section"].split(" ")[0] for r in res][:1] == ["stack"]
      and [out.find("\n" + h + "\n") for h in HEADERS] == sorted(out.find("\n" + h + "\n") for h in HEADERS)
      and -1 not in [out.find("\n" + h + "\n") for h in HEADERS])
rc, out, res, w = run(lo_flags=0x1009)
check("lo flags 0x1009 -> multicast on", line_of(out, "loopback multicast") == "OK  loopback multicast - on")

print("B. QT Py missing")
rc, out, res, w = run(files={C1, WRIST, "/dev/input/js0"})
check("exit 1", rc == 1, f"rc {rc}")
check("KO on the motor board with the fix", line_of(out, "motor board (QT Py)").startswith("KO")
      and "not on USB" in line_of(out, "motor board (QT Py)"))
check("summary names it", "KO: motor board (QT Py)" in last_line(out), last_line(out))
check("firmware skipped, the serial port never opened",
      w["opens"] == [] and "skipped: no free QT Py port" in line_of(out, "firmware telemetry"))
check("the lidar still checked on its own port", w["lidar_calls"] == [C1])

print("C. a stack already running")
STACK_CMD = "/home/x/vector-dimos/.venv/bin/python3 /home/x/vector-dimos/.venv/bin/dimos --rerun-open none run vector-dimos.moss-nav --daemon"
rc, out, res, w = run(runs=[("20261011-040000-vector-dimos-moss-nav", 4242)], alive={4242},
                      cmdlines=[(1, "/sbin/init"), (4242, STACK_CMD)])
check("stack verdict OK with run id and pid",
      line_of(out, "stack") == "OK  stack - 20261011-040000-vector-dimos-moss-nav pid 4242", line_of(out, "stack"))
fw, li = find(res, "firmware telemetry"), find(res, "lidar scan")
check("firmware SKIPPED: held by the stack",
      len(fw) == 1 and fw[0].get("skipped") and fw[0]["detail"].startswith("skipped: held by the stack"),
      fw[0]["detail"] if fw else "none")
check("lidar SKIPPED: held by the stack",
      len(li) == 1 and li[0].get("skipped") and li[0]["detail"].startswith("skipped: held by the stack"))
check("zero serial opens, zero lidar spins", w["opens"] == [] and w["lidar_calls"] == [])
check("no port-holder scan either (the stack holds them by design)", w["holder_calls"] == [])
check("exit 0 (a stack is not a hardware fault) and skips counted apart",
      rc == 0 and "(2 skipped)" in last_line(out), last_line(out))
rc, out, res, w = run(cmdlines=[(1, "/sbin/init"), (777, "/home/x/vector-dimos/.venv/bin/dimos --rerun-open none run vector-dimos.moss-nav --daemon")])
check("unregistered `bin/dimos run` process -> probes skipped too",
      "unregistered dimos run pid 777" in line_of(out, "stack") and w["opens"] == [] and w["lidar_calls"] == [],
      line_of(out, "stack"))
rc, out, res, w = run(cmdlines=[(1, "/sbin/init"), (778, "/home/x/vector-dimos/.venv/bin/python3 /home/x/vector-dimos/.venv/bin/dimos log"),
                                (779, "python3 tools/moss_cockpit.py --port 8904")])
check("`dimos log` and the cockpit are not a stack", line_of(out, "stack") == "OK  stack - none"
      and len(w["opens"]) == 1, line_of(out, "stack"))
rc, out, res, w = run(runs=[("20261010-155115-vector-dimos-moss-gamepad", 999)], alive=set())
check("dead registry pid = stale, not a stack: probes run",
      "none (1 stale registry entry)" in line_of(out, "stack") and len(w["opens"]) == 1 and w["lidar_calls"] == [C1],
      line_of(out, "stack"))

rc, out, res, w = run(runs=[("20261010-155115-vector-dimos-moss-gamepad", 818205)], alive={818205},
                      cmdlines=[(1, "/sbin/init"), (818205, "-bash")])
fw = find(res, "firmware telemetry")
check("registry pid alive but reused by a shell = stale, not a stack: battery read, lidar spun",
      "none (1 stale registry entry)" in line_of(out, "stack") and len(w["opens"]) == 1
      and w["lidar_calls"] == [C1] and len(fw) == 1 and not fw[0].get("skipped")
      and "11.61 V" in line_of(out, "battery"), line_of(out, "stack"))

print("D. battery (3S pack, INA219 voltage)")
for mv, want_ok, needle in ((10500, False, "10.50 V - under 10.8 V: the BMS cuts at 10.2 V under load"),
                            (10100, False, "10.10 V - under the BMS cut (10.2 V)"),
                            (10800, True, "10.80 V"),
                            (12600, True, "12.60 V")):
    rc, out, res, w = run(telemetry=dict(TELEMETRY, bus_mV=mv))
    ln = line_of(out, "battery")
    check(f"{mv} mV -> {'OK' if want_ok else 'KO'} '{needle}'",
          ln.startswith("OK" if want_ok else "KO") and needle in ln and rc == (0 if want_ok else 1), ln)
rc, out, res, w = run(telemetry=dict(TELEMETRY, ina_ok=False, bus_mV=None))
check("ina_ok false -> KO INA219 not seen", line_of(out, "battery").startswith("KO") and "INA219 not seen" in out)

print("E. gamepad diagnosis")
files_nopad = {QTPY, C1, WRIST}
rc, out, res, w = run(files=files_nopad, usb={"057e:2009", "239a:8119"})
check("057e:2009 -> the Nintendo sentence, as a warning (nothing refuses a missing pad)",
      "pad enumerated as a Nintendo Pro Controller (057e:2009) without a driver: switch the pad to its "
      "Android/X-input mode and replug" in line_of(out, "gamepad") and line_of(out, "gamepad").startswith("OK")
      and "WARN" in line_of(out, "gamepad"),
      line_of(out, "gamepad"))
rc, out, res, w = run(files=files_nopad, usb={"2563:0526"})
check("2563:0526 -> receiver visible, no js0", "receiver visible (2563:0526), no js0: replug or reboot" in line_of(out, "gamepad"))
rc, out, res, w = run(files=files_nopad, usb=set())
check("nothing -> no pad, still a warning", "no pad" in line_of(out, "gamepad") and line_of(out, "gamepad").startswith("OK"))
rc, out, res, w = run(files=files_nopad, usb=set(), env={"MOSS_REQUIRE_PAD": "1"})
check("MOSS_REQUIRE_PAD=1 -> no pad is a KO and the check fails", line_of(out, "gamepad").startswith("KO") and rc == 1)
rc, out, res, w = run(usb={"057e:2009"})
check("js0 present wins over any USB id", line_of(out, "gamepad").startswith("OK"))

print("F. --json")
rc, out, res, w = run(["--json"], telemetry=dict(TELEMETRY, bus_mV=10500))
try:
    doc = json.loads(out)
    parsed = True
except ValueError as exc:
    doc, parsed = {}, False
    print("   ", exc, out[:200])
check("stdout is exactly one JSON document", parsed)
checks = doc.get("checks", [])
counted = [c for c in checks if not c.get("skipped")]
check("keys ts, checks, ok_count, ko_count, summary",
      all(k in doc for k in ("ts", "checks", "ok_count", "ko_count", "summary")))
check("each check has ok, label, detail, section",
      bool(checks) and all(set(("ok", "label", "detail", "section")) <= set(c) for c in checks))
check("ok_count / ko_count match the checks",
      doc.get("ok_count") == sum(1 for c in counted if c["ok"]) and doc.get("ko_count") == sum(1 for c in counted if not c["ok"])
      and doc.get("ko_count") == 1, f"{doc.get('ok_count')} / {doc.get('ko_count')}")
check("summary carries the KO label", doc.get("summary", "").endswith("KO: battery"), doc.get("summary"))
check("exit 1 with a KO in JSON mode too", rc == 1)
check("ts is now", abs(doc.get("ts", 0) - time.time()) < 60)

print("G. --quick and --no-spin")
rc, out, res, w = run(["--quick"])
check("--quick: the serial stub is never called", w["opens"] == [])
check("--quick: the lidar stub is never called", w["lidar_calls"] == [])
check("--quick: both reported skipped, exit 0",
      rc == 0 and "skipped: --quick" in line_of(out, "firmware telemetry") and "skipped" in line_of(out, "lidar scan"))
rc, out, res, w = run(["--no-spin"])
check("--no-spin: the firmware IS listened to (battery judged)", len(w["opens"]) == 1 and "11.61 V" in out)
check("--no-spin: the lidar never spins", w["lidar_calls"] == [])

print("H. firmware and port faults")
rc, out, res, w = run(holders={QTPY: [3131]})
check("QT Py held by pid 3131 -> KO naming it", "held by pid 3131" in line_of(out, "motor board (QT Py) free")
      and rc == 1, line_of(out, "motor board (QT Py) free"))
check("... and the firmware is never opened against it", w["opens"] == [])
rc, out, res, w = run(telemetry=dict(TELEMETRY, mode=2, reason="drive"))
check("mode 2 -> KO firmware disarmed with the reason",
      line_of(out, "firmware disarmed") == "KO  firmware disarmed - mode 2 (drive) - something else is driving it",
      line_of(out, "firmware disarmed"))
rc, out, res, w = run(serial_chunks=[b"", b"garbage\n"])
check("mute firmware -> KO, no further firmware verdict",
      line_of(out, "firmware telemetry").startswith("KO") and "mute" in out and not find(res, "battery"))
rc, out, res, w = run(telemetry=dict(TELEMETRY, driver="mdd10a"))
check("driver reported -> shown", line_of(out, "motor driver") == "OK  motor driver - mdd10a")
rc, out, res, w = run(realsense=[dict(D455, usb="2.1")])
check("RealSense on USB 2.1 -> KO 5 fps max", "on USB 2: 5 fps max" in line_of(out, "RealSense D455")
      and line_of(out, "RealSense D455").startswith("KO"))
rc, out, res, w = run(realsense=[], env={"PERSISTENT_MAP": "0", "MOSS_CAMERA": "0"})
check("no RealSense but MOSS_CAMERA=0 -> OK, said", line_of(out, "RealSense D455").startswith("OK")
      and "MOSS_CAMERA=0" in out)
rc, out, res, w = run(env={"PERSISTENT_MAP": "0", "MOSS_PORT": C1})
check("MOSS_PORT pointing at the lidar -> KO, and the lidar is never written as a motor port",
      line_of(out, "motor port is not the lidar").startswith("KO") and w["opens"] == [], line_of(out, "motor port is not the lidar"))
rc, out, res, w = run(lidar=dict(default_world()["lidar"], sectors=31, points_per_s=1400.0))
check("31/36 sectors -> KO lidar scan", line_of(out, "lidar scan").startswith("KO") and "31/36" in out)
rc, out, res, w = run(lidar_exc=RuntimeError("no descriptor for a550"))
check("lidar mute -> KO with the exception", "no descriptor for a550" in line_of(out, "lidar answers")
      and line_of(out, "lidar answers").startswith("KO"))
rc, out, res, w = run(files={QTPY, C1, "/dev/input/js0"})
check("wrist camera absent -> OK with a warning (nothing uses it yet)",
      line_of(out, "wrist camera").startswith("OK") and "WARN" in line_of(out, "wrist camera"))
rc, out, res, w = run(free_gb=3.2, mem_kb=1_900_000)
check("3.2 GB free -> KO disk; 1.9 GB RAM -> KO", "3.2 GB free" in line_of(out, "disk for recordings")
      and line_of(out, "disk for recordings").startswith("KO") and line_of(out, "RAM available").startswith("KO"))

print("I. software and env")
rc, out, res, w = run(env={})
check("PERSISTENT_MAP unset -> OK with the warning",
      line_of(out, "PERSISTENT_MAP") == "OK  PERSISTENT_MAP - warning: unset - defaults to 1 in code; moss-fly.sh exports 0")
rc, out, res, w = run(env={"PERSISTENT_MAP": "1"})
check("PERSISTENT_MAP=1 -> KO", line_of(out, "PERSISTENT_MAP").startswith("KO") and rc == 1)
rc, out, res, w = run(entry_points={"moss-gamepad", "moss-base"})
check("moss-nav / moss-explore missing -> KO with the reinstall line",
      "blueprint moss-nav - missing - reinstall: pip install -e . --no-deps" in out
      and line_of(out, "blueprint moss-explore").startswith("KO") and line_of(out, "blueprint moss-gamepad").startswith("OK"))
rc, out, res, w = run(imports_missing={"kiss_icp"})
check("kiss_icp missing -> KO import kiss_icp", line_of(out, "import kiss_icp").startswith("KO"))
rc, out, res, w = run(profile="vector", env={"PERSISTENT_MAP": "0", "ROBOT_PROFILE": "vector"})
check("ROBOT_PROFILE=vector -> KO robot profile", line_of(out, "robot profile").startswith("KO")
      and "ROBOT_PROFILE=vector" in out)
rc, out, res, w = run(files={C1, WRIST, "/dev/input/js0"}, env={"PERSISTENT_MAP": "0", "MOSS_MOCK_BUS": "1"})
check("MOSS_MOCK_BUS=1: no QT Py needed, firmware skipped, exit 0",
      rc == 0 and "MOSS_MOCK_BUS=1" in line_of(out, "motor board (QT Py)") and w["opens"] == [], last_line(out))

print("J. --save (the document the cockpit serves)")
target = TMPD / "saved" / "preflight.json"
rc, out, res, w = run(["--save", str(target)])
try:
    saved = json.loads(target.read_text())
except (OSError, ValueError):
    saved = {}
check("file written, human lines still on stdout", bool(saved) and "ready to fly" in last_line(out))
check("same checks and counts as the run", saved.get("ok_count") == len([r for r in res if not r.get("skipped")])
      and saved.get("ko_count") == 0 and len(saved.get("checks", [])) == len(res))
check("no .tmp left behind", not (target.parent / "preflight.json.tmp").exists())

print("K. the real probes, on this machine (no device opened)")
import importlib  # noqa: E402
real = importlib.reload(mp)
held = TMPD / "held_port"
held.write_text("x")
child = subprocess.Popen([sys.executable, "-c",
                          f"import time; f = open({str(held)!r}); print('ready', flush=True); time.sleep(20)"],
                         stdout=subprocess.PIPE, text=True)
child.stdout.readline()
try:
    holders = real._port_holders(str(held))
    check("/proc fd scan finds the child holding the file", child.pid in holders, f"{holders} vs {child.pid}")
    check("... and never reports this process", os.getpid() not in holders)
finally:
    child.kill()
    child.wait()
check("nobody holds it once the child is gone", real._port_holders(str(held)) == [])
ids = real._usb_devices()
check("sysfs USB ids look like vvvv:pppp", all(len(i) == 9 and i[4] == ":" for i in ids), f"{len(ids)} devices")
check("our own cmdline is in the /proc scan", any(pid == os.getpid() and "test_moss_preflight_cold" in cmd
                                                  for pid, cmd in real._proc_cmdlines()))
check("lo flags read as an int (or None off Linux)", real._lo_flags() is None or isinstance(real._lo_flags(), int))
check("nothing ran at import (RESULTS empty after reload)", real.RESULTS == [])
# a registry entry naming a LIVE pid that is not dimos (pid reuse after a reboot)
reused = subprocess.Popen([sys.executable, "-c", "import time; print('ready', flush=True); time.sleep(20)"],
                          stdout=subprocess.PIPE, text=True)
reused.stdout.readline()
try:
    real_runs = TMPD / "real_runs"
    real_runs.mkdir()
    (real_runs / "20261010-155115-vector-dimos-moss-gamepad.json").write_text(json.dumps(
        {"run_id": "20261010-155115-vector-dimos-moss-gamepad", "pid": reused.pid}))
    real._runs_dir = lambda: real_runs
    ctx = {}
    with contextlib.redirect_stdout(io.StringIO()) as sbuf:
        real.check_stack(ctx)
    check("real /proc: a registry pid that is alive but not `dimos run` is stale, not a stack",
          real._pid_alive(reused.pid) and f"pid {reused.pid}" not in (ctx.get("stack") or "")
          and "1 stale registry entry" in sbuf.getvalue(),
          sbuf.getvalue().strip())
finally:
    reused.kill()
    reused.wait()

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

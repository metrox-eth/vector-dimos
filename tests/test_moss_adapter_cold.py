"""Cold bench: MossBaseAdapter on the fake MOSS firmware. Known in -> known out.

No hardware. dimOS only for section 0's gamepad clamp (skipped without it).
The fake runs the firmware's own state machine (Control.h, qtpy-mdd3a-v3:
arm / drive / stop / ramp, 300 ms drive deadman, 10 s arm expiry, +-100 %),
so what is proven here is the line protocol the Jetson speaks to the QT Py
over USB CDC, and the adapter's deadman on top of it.

Rule #2: known input -> known output in percent, metres, radians, seconds.

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_moss_adapter_cold.py
"""
import math
import os
import re
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.moss_adapter import (FIRMWARE_MAX_PCT, KEEPALIVE_S, STALE_S,
                                       MossBaseAdapter, MossGeometry,
                                       teleop_envelope, teleop_speeds, tracks_to_twist,
                                       twist_to_tracks)
from vector_dimos.moss_mock import DRIVE, OFF, READY, FakeMossFirmware

OK = KO = 0


def check(cond, label):
    global OK, KO
    if cond:
        OK += 1
    else:
        KO += 1
    print(("  OK  " if cond else "  KO  ") + label)


def settle(s=0.06):
    time.sleep(s)


# ── 0. gamepad scale: full stick = the base's limit, not VECTOR's 0.6 m/s ──
print("0. teleop_speeds / teleop_envelope (track 0.30 m, 0.45 m/s at 100 %, turn gain 2)")
os.environ.pop("MOSS_TURN_GAIN", None)
G100 = MossGeometry(track_m=0.30, mps_at_limit=0.45, max_pct=100)
_lin, _ang = teleop_speeds(G100)
check(abs(_lin - 0.45) < 1e-9, "full stick forward -> 0.45 m/s (= 100/100)")
check(abs(_ang - 6.0) < 1e-9, "full stick sideways -> 6.0 rad/s (gain 2 x the 3.0 rad/s spin)")
check(twist_to_tracks(_lin, 0.0, G100) == (100, 100), "that twist IS 100/100")
check(twist_to_tracks(0.0, _ang / 2, G100) == (-100, 100), "half turn stick alone = full spin -100/+100")
check(teleop_speeds(MossGeometry(track_m=0.0, mps_at_limit=0.45, max_pct=100)) == (0.45, 0.0), "track 0 -> no spin, no crash")
env = teleop_envelope(G100)
check(all(abs(env[k] - v) < 1e-9 for k, v in {"linear": 0.45, "angular": 6.0, "lever": 0.15, "rim": 1.35}.items()), f"envelope: {env}")
try:
    from vector_dimos.gamepad import TwistEnvelope, clamp_twist   # gamepad.py imports dimOS
    HAVE_DIMOS = True
except ImportError:
    HAVE_DIMOS = False
if HAVE_DIMOS:
    E = TwistEnvelope(**env)
    def tracks(vx, wz):
        c = clamp_twist(vx, 0.0, wz, E); return twist_to_tracks(c[0], c[2], G100)
    check(tracks(0.45, 0.0) == (100, 100), "full forward through the gamepad clamp -> 100/100")
    check(tracks(0.0, 6.0) == (-100, 100), "full turn alone -> full spin -100/+100")
    check(tracks(0.45, 3.0) == (0, 100), "full forward + HALF turn -> inner track stops (0/100)")
    check(tracks(0.45, 6.0) == (-100, 100), "full forward + FULL turn -> inner track full reverse (-100/100): the tank pivot")
    check(tracks(0.45, 1.92) == (36, 100), f"full forward + turn just past the deadzone -> inner slows (36/100): {tracks(0.45, 1.92)}")
    os.environ["MOSS_TURN_GAIN"] = "1"
    _l1, _a1 = teleop_speeds(G100); E1 = TwistEnvelope(**teleop_envelope(G100))
    _c = clamp_twist(0.45, 0.0, _a1, E1)
    check(abs(_a1 - 3.0) < 1e-9 and twist_to_tracks(_c[0], _c[2], G100) == (0, 100), "MOSS_TURN_GAIN=1 -> full forward + full turn only stops the inner track (0/100)")
    os.environ.pop("MOSS_TURN_GAIN", None)
    check(clamp_twist(9.0, 0.0, 0.0) == (0.45, 0.0, 0.0), "no envelope given -> VECTOR's default ceiling (unchanged)")
else:
    print("  -- skipped, no dimOS here: run this on the Jetson")


# ── 1. pure mapping: twist -> track percent ─────────────────────────────
G = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=None)
print("1. twist -> tracks (track 0.30 m, 0.15 m/s at 25 %)")
check(twist_to_tracks(0.15, 0.0, G) == (25, 25), "vx=0.15 -> L=25 R=25 (the limit)")
check(twist_to_tracks(0.06, 0.0, G) == (10, 10), "vx=0.06 -> 10/10")
check(twist_to_tracks(-0.06, 0.0, G) == (-10, -10), "vx=-0.06 -> -10/-10 (reverse)")
check(twist_to_tracks(0.0, 1.0, G) == (-25, 25), "wz=+1 rad/s -> L=-25 R=+25 (spin left)")
check(twist_to_tracks(0.30, 0.0, G) == (25, 25), "vx=0.30 (2x the limit) -> clamped 25/25")
check(twist_to_tracks(0.001, 0.0, G) == (0, 0), "vx=0.001 rounds to 0 -> a stop")
GF = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=None, min_pct=12)
check(twist_to_tracks(0.06, 0.0, GF) == (12, 12), "floor 12: vx=0.06 (10 %) -> raised to 12/12")
check(twist_to_tracks(-0.06, 0.0, GF) == (-12, -12), "floor 12: vx=-0.06 -> -12/-12 (sign kept)")
check(twist_to_tracks(0.0, 0.3, GF) == (-12, 12), "floor 12: tiny spin (-5/+5) -> -12/+12")
check(twist_to_tracks(0.001, 0.0, GF) == (0, 0), "floor 12: a twist that rounds to 0 is STILL a stop")
check(twist_to_tracks(0.15, 0.0, GF) == (25, 25), "floor 12: the limit is untouched")
check(twist_to_tracks(0.09, 0.0, GF) == (15, 15), "floor 12: 15 % above the floor passes as is")
check(twist_to_tracks(0.06, 0.0, G) == (10, 10), "floor 0 (default): 10/10 unchanged")
check(twist_to_tracks(0.06, 0.0, MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, min_pct=99)) == (25, 25),
      "a floor above the limit is clamped to the limit")
check(twist_to_tracks(0.15, 0.0, MossGeometry(mps_at_limit=0.15, max_pct=10)) == (10, 10),
      "max_pct=10 -> the same full-scale twist lands on 10/10")
vx, vy, wz = tracks_to_twist(-0.15, 0.15, G)
check(abs(vx) < 1e-9 and abs(wz - 1.0) < 1e-9 and vy == 0.0,
      "tracks (-0.15, +0.15) m/s -> vx=0 wz=+1.0 (inverse of the above)")

# ── 2. the adapter drives the fake firmware ────────────────────────────
print("2. adapter on the fake firmware: connect, arm, drive, stop")
fw = FakeMossFirmware()
factory = lambda: fw
factory.is_mock = True
a = MossBaseAdapter(dof=3, port_factory=factory, geometry=G)
check(a.connect() is True, "connect() probes with 'status' and gets telemetry")
check(fw.commands[:1] == ["status"], f"first line on the wire is 'status': {fw.commands[:1]}")
check(a.get_dof() == 3, "get_dof() == 3 (vy ignored at the tracks)")
check(a.read_odometry() is None, "no ticks_per_m -> read_odometry() is None (honest)")
check(a.write_velocities([0.15, 0.0, 0.0]) is False and "drive 25 25" not in fw.commands,
      "drive before enable is refused and never reaches the wire")
check(a.write_enable(True) and a.read_enabled() and fw.mode == READY,
      "write_enable(True) = 'arm' -> firmware READY")
check(a.write_velocities([0.15, 0.4, 0.0]) is True,
      "write_velocities(vx=0.15, vy=0.4 ignored) accepted")
check(fw.commands[-2:] == ["arm", "drive 25 25"] and fw.mode == DRIVE and (fw.left, fw.right) == (25, 25),
      f"-> 'arm' then 'drive 25 25' on the wire, nothing else, firmware DRIVE 25/25: {fw.commands[-2:]}")
check(a.read_velocities() == [0.15, 0.0, 0.0],
      "read_velocities() = the commanded twist (open loop, no encoder calibration)")
n_before = fw.commands.count("drive 25 25")
settle(KEEPALIVE_S * 3.5)
n_after = fw.commands.count("drive 25 25")
check(n_after - n_before >= 2,
      f"keepalive re-sends the drive every {KEEPALIVE_S*1000:.0f} ms: +{n_after - n_before} in 350 ms")
check(a.write_stop() is True and fw.commands[-1] == "stop" and fw.mode == OFF,
      "write_stop() = 'stop' -> firmware OFF")
check(a.read_velocities() == [0.0, 0.0, 0.0], "after stop, commanded velocity is zero")

# ── 3. the adapter deadman: a commander that goes quiet ────────────────
print("3. deadman: no fresh twist -> one 'stop', tracks held")
check(a.write_velocities([0.06, 0.0, 0.0]) and fw.mode == DRIVE, "drive 10/10 again")
settle(STALE_S + 0.25)
tail = fw.commands[-3:]
check(fw.commands[-1] == "stop" and fw.mode == OFF,
      f"after {STALE_S}s without a write: 'stop' sent, firmware OFF (tail {tail})")
n_stop = fw.commands.count("stop")
settle(KEEPALIVE_S * 3)
check(fw.commands.count("stop") == n_stop, "the stop is sent ONCE, not hammered")
check(a.target_pct == (0, 0), "target zeroed")

# ── 4. arm expiry in the firmware, seen through telemetry ──────────────
print("4. the firmware disarms itself (10 s idle) -> 'ERR not_armed' -> re-arm, retry")
check(a.write_enable(True) and fw.mode == READY, "armed again")
fw.advance(10_001)          # firmware: READY for 10 s -> OFF 'arm_expired', telemetry queued
settle(0.1)
check(fw.mode == OFF and fw.reason == "arm_expired", f"firmware OFF ({fw.reason})")
check(a.firmware_mode == OFF, "adapter read mode OFF from telemetry")
before = len(fw.commands)
check(a.write_velocities([0.06, 0.0, 0.0]) is True, "a new drive is accepted")
check(fw.commands[before:] == ["drive 10 10", "arm", "drive 10 10"] and fw.mode == DRIVE,
      f"-> refused, 'arm', resent (never an 'arm' on a guess): {fw.commands[before:]}")

# ── 5. 'ERR not_armed' without telemetry -> re-arm and retry once ───────
print("5. firmware stopped silently (no telemetry) -> the same path, no guessing")
fw._stop("timeout")         # no telemetry line emitted: the adapter cannot know
a.write_stop()              # adapter-side reset to a clean known state
check(a.write_enable(True) and fw.mode == READY, "armed")
fw._stop("timeout")
before = len(fw.commands)
check(a.write_velocities([0.06, 0.0, 0.0]) is True, "drive accepted after the retry")
check(fw.commands[before:] == ["drive 10 10", "arm", "drive 10 10"] and fw.mode == DRIVE,
      f"-> refused, 'arm', resent: {fw.commands[before:]}")
a.write_stop()

# ── 6. odometry from ticks when calibrated ─────────────────────────────
print("6. ticks_per_m set -> velocity and pose from encoder ticks")
G2 = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=1000.0)
fw2 = FakeMossFirmware()
f2 = lambda: fw2
f2.is_mock = True
b = MossBaseAdapter(dof=3, port_factory=f2, geometry=G2)
check(b.connect(), "connect (calibrated geometry)")
settle()
fw2.ticks = [100, 100]
fw2.emit_telemetry()
settle()
x, y, th = b.read_odometry()
check(abs(x - 0.10) < 1e-6 and abs(y) < 1e-6 and abs(th) < 1e-6,
      f"+100 ticks both sides (1000/m) -> x=+0.100 m, y=0, th=0: {x:.3f} {y:.3f} {th:.3f}")
fw2.ticks = [50, 150]
fw2.emit_telemetry()
settle()
x, y, th = b.read_odometry()
check(abs(x - 0.10) < 1e-6 and abs(th - (0.10 / 0.30)) < 1e-6,
      f"then -50/+50 ticks -> x unchanged, th=+0.333 rad (0.1 m over 0.30 m track): th={th:.3f}")
vx, vy, wz = b.read_velocities()
check(abs(vx) < 1e-9 and wz > 0, f"read_velocities() from ticks: vx=0, wz>0 ({wz:+.2f})")
b.disconnect()
check(fw2.commands[-1] == "stop" and not b.is_connected(), "disconnect() sends 'stop'")

# ── 7. geometry from env: bounds and mistyped values ───────────────────
print("7. MossGeometry.from_env")
import os
os.environ.update({"MOSS_MAX_PCT": "140", "MOSS_TICKS_PER_M": "pouet", "MOSS_TRACK_M": "0.28"})
g = MossGeometry.from_env()
check(g.max_pct == FIRMWARE_MAX_PCT, f"MOSS_MAX_PCT=140 clamped to the firmware's {FIRMWARE_MAX_PCT}")
check(g.ticks_per_m is None, "MOSS_TICKS_PER_M=pouet -> None (no odometry, not a random scale)")
check(g.track_m == 0.28, "MOSS_TRACK_M=0.28 taken")
for k in ("MOSS_MAX_PCT", "MOSS_TICKS_PER_M", "MOSS_TRACK_M"):
    os.environ.pop(k, None)
os.environ["MOSS_MIN_PCT"] = "40"
check(MossGeometry.from_env().min_pct == 40, "MOSS_MIN_PCT=40 -> min_pct 40")
os.environ["MOSS_MIN_PCT"] = "150"
check(MossGeometry.from_env().min_pct == MossGeometry.from_env().max_pct, "MOSS_MIN_PCT above the ceiling -> clamped to max_pct")
os.environ["MOSS_MIN_PCT"] = "-5"
check(MossGeometry.from_env().min_pct == 0, "MOSS_MIN_PCT negative -> 0")
os.environ.pop("MOSS_MIN_PCT", None)
check(MossGeometry.from_env().min_pct == 0, "MOSS_MIN_PCT unset -> 0 (floor off, unchanged behaviour)")

# ── 8. the fake is the firmware: its own refusals ──────────────────────
print("8. fake firmware fidelity (Control.h)")
fw3 = FakeMossFirmware()
fw3.write(b"drive 10 10\n")
check(fw3.sent[-1] == "ERR not_armed" and fw3.mode == OFF, "drive before arm -> ERR not_armed")
fw3.write(b"arm\n")
fw3.write(b"drive 26 0\n")
check(fw3.sent[-1] == "OK" and fw3.mode == DRIVE and (fw3.left, fw3.right) == (26, 0),
      "26 % is valid since v3 (limit 100) -> OK, DRIVE 26/0")
fw3.write(b"drive 101 0\n")
check(fw3.sent[-1] == "ERR invalid_command" and fw3.mode == OFF and fw3.reason == "invalid",
      "101 % -> ERR invalid_command, OFF 'invalid'")
fw3.write(b"arm\ndrive -100 100\n")
check(fw3.sent[-1] == "OK" and (fw3.left, fw3.right) == (-100, 100), "arm + drive -100 100 (the full pivot) -> OK")
fw3.write(b"stop\n")      # 'arm' while DRIVE is arm_while_moving (a stop), so stop first
fw3.write(b"arm\ndrive 5 -5\n")
check(fw3.mode == DRIVE and (fw3.left, fw3.right) == (5, -5), "arm + drive 5 -5 -> DRIVE 5/-5")
fw3.advance(301)
check(fw3.mode == OFF and fw3.reason == "timeout", "301 ms without a drive -> OFF 'timeout'")
a.disconnect()

# ── 9. v3 firmware surface: ramp command and telemetry keys ────────────
print("9. fake firmware v3: 'ramp UP DOWN' and the telemetry keys")
import json
fw4 = FakeMossFirmware()
check("limit 100%" in fw4.sent[0] and "qtpy-mdd3a-v3" in fw4.sent[0], f"banner: {fw4.sent[0]!r}")
fw4.write(b"status\n")
tel = json.loads(fw4.sent[-1])
check(tel.get("fw") == "qtpy-mdd3a-v3" and tel.get("ramp_ms") == [30, 15],
      f"boot telemetry: fw qtpy-mdd3a-v3, ramp 30/15 ms per % ({tel.get('fw')}, {tel.get('ramp_ms')})")
check(tel.get("ina_ok") is True and tel.get("bus_mV") == 11600 and tel.get("shunt_uV") == 0
      and tel.get("current_mA") is None, "battery: ina_ok, 11600 mV (11.6 V), shunt 0, current null")
check("driver" not in tel, "no 'driver' key (only the MDD10A build sends it)")
fw4.write(b"ramp 5 5\n")
check(fw4.sent[-1] == "OK" and fw4.ramp_ms == [5, 5], "ramp 5 5 -> OK, 5/5 ms per %")
fw4.write(b"arm\ndrive 40 40\nramp 10 500\n")
check(fw4.sent[-1] == "OK" and fw4.mode == DRIVE and fw4.ramp_ms == [10, 500],
      "ramp 10 500 while driving -> OK, still DRIVE (a ramp change never stops)")
fw4.emit_telemetry()
tel = json.loads(fw4.sent[-1])
check(tel["ramp_ms"] == [10, 500] and tel["target_pct"] == [40, 40] and tel["output_pct"] == [40, 40],
      "telemetry reports ramp 10/500; output_pct == target_pct (ramp not simulated)")
for bad in ("ramp 0 5", "ramp 5 501", "ramp x 5"):
    fw4.write(b"arm\n")
    fw4.write((bad + "\n").encode())
    check(fw4.sent[-1] == "ERR invalid_command" and fw4.mode == OFF and fw4.ramp_ms == [10, 500],
          f"'{bad}' -> ERR invalid_command, OFF, ramp unchanged")

# ── 10. adapter: telemetry age, default port, geometry default ────────
print("10. adapter telemetry_age_s, default_port (QT Py by-id, never ttyUSB0), track 0.266")
import tempfile
from vector_dimos import robot_profile as rp
from vector_dimos.moss_adapter import default_port
fw5 = FakeMossFirmware()
f5 = lambda: fw5
f5.is_mock = True
c = MossBaseAdapter(dof=3, port_factory=f5, geometry=G)
check(c.telemetry_age_s is None, "before connect: telemetry_age_s is None")
check(c.connect(), "connect")
age0 = c.telemetry_age_s
settle(0.25)
age1 = c.telemetry_age_s
check(age0 is not None and 0.0 <= age0 < 0.2 and 0.2 <= age1 - age0 < 0.5,
      f"age right after connect {age0:.3f} s, +0.25 s later {age1:.3f} s")
fw5.emit_telemetry()
settle()
check(c.telemetry_age_s < 0.2, f"a new line resets the age ({c.telemetry_age_s:.3f} s)")
check(c.telemetry.get("bus_mV") == 11600 and c.telemetry is not c.telemetry, "telemetry is a copy of the last line")
c.disconnect()
saved_root, saved_port = rp.BYID_ROOT, os.environ.pop("MOSS_PORT", None)
with tempfile.TemporaryDirectory() as d:
    rp.BYID_ROOT = d
    check(default_port() == "/dev/serial/by-id/MISSING-QT-PY",
          f"no QT Py, MOSS_PORT unset -> the sentinel: {default_port()}")
    lidar = os.path.join(d, "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_x-if00-port0")
    open(lidar, "w").close()
    check(default_port() == "/dev/serial/by-id/MISSING-QT-PY", "a CP2102N alone is never taken for the base")
    qt = os.path.join(d, "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_f4:12:fa:8d:a7:58-if00")
    open(qt, "w").close()
    check(default_port() == qt, f"QT Py present -> its by-id path: {os.path.basename(default_port())}")
    check(MossBaseAdapter(port_factory=f5)._port == qt, "an adapter built without address takes it")
    os.environ["MOSS_PORT"] = "/dev/ttyACM9"
    check(default_port() == "/dev/ttyACM9", "MOSS_PORT wins")
    os.environ.pop("MOSS_PORT", None)
rp.BYID_ROOT = saved_root
if saved_port is not None:
    os.environ["MOSS_PORT"] = saved_port
check("ttyUSB0" not in default_port(), "the default is never /dev/ttyUSB0 (the lidar on MOSS)")
os.environ.pop("MOSS_TRACK_M", None)
check(MossGeometry().track_m == 0.266 and MossGeometry.from_env().track_m == 0.266,
      "MossGeometry track default 0.266 m (robot_profile.MOSS, CAD placeholder)")

# ── 11. the mock bus on the wall clock (MOSS_MOCK_BUS runs) ──────────────
print("11. MOSS_MOCK_BUS: live 200 ms telemetry and a firmware deadman that fires")
fw6 = FakeMossFirmware(auto_time=True)
fw6.write(b"arm\ndrive 10 10\n")
check(fw6.mode == DRIVE, "auto-time fake: arm + drive 10 10 -> DRIVE")
t_end = time.monotonic() + 0.45
while time.monotonic() < t_end:
    fw6.readline()
check(fw6.mode == OFF and fw6.reason == "timeout", f"no refresh for 0.45 s -> OFF '{fw6.reason}' (300 ms deadman)")
n_tel = sum(1 for line in fw6.sent if line.startswith("{"))
check(n_tel >= 2, f"telemetry every 200 ms while polled: {n_tel} lines in 0.45 s")
os.environ["MOSS_MOCK_BUS"] = "1"
m = MossBaseAdapter(dof=3, geometry=G)
check(m.connect() and m.mock_bus, "MOSS_MOCK_BUS=1 -> the adapter connects to the fake, mock_bus True")
settle(0.6)
check(m.telemetry_age_s is not None and m.telemetry_age_s < 0.35,
      f"telemetry stays fresh on the mock bus: age {m.telemetry_age_s:.3f} s after 0.6 s")
check(m.write_enable(True) and m.write_velocities([0.06, 0.0, 0.0]), "arm + drive through the mock bus")
settle(0.35)
check(m.telemetry.get("mode") == DRIVE and m.telemetry.get("target_pct") == [10, 10],
      f"keepalive holds DRIVE 10/10 past the 300 ms firmware deadman: {m.telemetry.get('mode')} {m.telemetry.get('target_pct')}")
m.disconnect()
os.environ.pop("MOSS_MOCK_BUS", None)

# ── 12. public-repo hygiene: the adapter source carries no operator quote ──
print("12. moss_adapter.py: no French operator quote (public repo)")
_src = (Path(__file__).resolve().parent.parent / "vector_dimos" / "moss_adapter.py").read_text()
# the quote that was there: French words and a French clock stamp ("15h49")
_banned = ["la sauce", "marche arriere", "s'arrete", "au-dela", "seuil", "elle peut"]
_hits = [w for w in _banned if w in _src.lower()] + re.findall(r"\b\d{1,2}h\d{2}\b", _src)
check(_hits == [], f"no French operator quote, no 'NNhNN' clock stamp in the source: hits {_hits}")

print()
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

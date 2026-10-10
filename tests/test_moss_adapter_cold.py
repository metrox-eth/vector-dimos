"""Cold bench: MossBaseAdapter on the fake MOSS firmware. Known in -> known out.

No hardware, no dimOS import. The fake runs the firmware's own state machine
(Control.h: arm / drive / stop, 300 ms drive deadman, 10 s arm expiry, +-25 %),
so what is proven here is the line protocol the Jetson will speak to the
ESP32 over the USB-UART, and the adapter's deadman on top of it.

Usage: python3 tests/test_moss_adapter_cold.py   (exit 0 = green)
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.moss_adapter import (FIRMWARE_MAX_PCT, KEEPALIVE_S, STALE_S,
                                       MossBaseAdapter, MossGeometry,
                                       teleop_speeds, tracks_to_twist,
                                       twist_to_tracks)
from vector_dimos.moss_mock import DRIVE, OFF, READY, FakeMossFirmware

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and cond


def settle(s=0.06):
    time.sleep(s)


# ── 0. gamepad scale: full stick = the base's limit, not VECTOR's 0.6 m/s ──
print("0. teleop_speeds (track 0.30 m, 0.15 m/s at 25 %)")
_lin, _ang = teleop_speeds(MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25))
check(abs(_lin - 0.15) < 1e-9, "full stick forward -> 0.15 m/s (= 25/25)")
check(abs(_ang - 1.0) < 1e-9, "full stick sideways -> 1.0 rad/s (= -25/+25 spin)")
check(twist_to_tracks(_lin, 0.0, MossGeometry(0.30, 0.15, 25)) == (25, 25), "and that twist IS 25/25")
check(twist_to_tracks(0.0, _ang, MossGeometry(0.30, 0.15, 25)) == (-25, 25), "and that spin IS -25/+25")
check(teleop_speeds(MossGeometry(track_m=0.0, mps_at_limit=0.15, max_pct=25)) == (0.15, 0.0), "track 0 -> no spin, no crash")


# ── 1. pure mapping: twist -> track percent ─────────────────────────────
G = MossGeometry(track_m=0.30, mps_at_limit=0.15, max_pct=25, ticks_per_m=None)
print("1. twist -> tracks (track 0.30 m, 0.15 m/s at 25 %)")
check(twist_to_tracks(0.15, 0.0, G) == (25, 25), "vx=0.15 -> L=25 R=25 (the limit)")
check(twist_to_tracks(0.06, 0.0, G) == (10, 10), "vx=0.06 -> 10/10")
check(twist_to_tracks(-0.06, 0.0, G) == (-10, -10), "vx=-0.06 -> -10/-10 (reverse)")
check(twist_to_tracks(0.0, 1.0, G) == (-25, 25), "wz=+1 rad/s -> L=-25 R=+25 (spin left)")
check(twist_to_tracks(0.30, 0.0, G) == (25, 25), "vx=0.30 (2x the limit) -> clamped 25/25")
check(twist_to_tracks(0.001, 0.0, G) == (0, 0), "vx=0.001 rounds to 0 -> a stop")
check(twist_to_tracks(0.15, 0.0, MossGeometry(max_pct=10)) == (10, 10),
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
os.environ.update({"MOSS_MAX_PCT": "40", "MOSS_TICKS_PER_M": "pouet", "MOSS_TRACK_M": "0.28"})
g = MossGeometry.from_env()
check(g.max_pct == FIRMWARE_MAX_PCT, f"MOSS_MAX_PCT=40 clamped to the firmware's {FIRMWARE_MAX_PCT}")
check(g.ticks_per_m is None, "MOSS_TICKS_PER_M=pouet -> None (no odometry, not a random scale)")
check(g.track_m == 0.28, "MOSS_TRACK_M=0.28 taken")
for k in ("MOSS_MAX_PCT", "MOSS_TICKS_PER_M", "MOSS_TRACK_M"):
    os.environ.pop(k, None)

# ── 8. the fake is the firmware: its own refusals ──────────────────────
print("8. fake firmware fidelity (Control.h)")
fw3 = FakeMossFirmware()
fw3.write(b"drive 10 10\n")
check(fw3.sent[-1] == "ERR not_armed" and fw3.mode == OFF, "drive before arm -> ERR not_armed")
fw3.write(b"arm\n")
fw3.write(b"drive 26 0\n")
check(fw3.sent[-1] == "ERR invalid_command" and fw3.mode == OFF, "26 % -> ERR invalid_command, OFF")
fw3.write(b"arm\ndrive 5 -5\n")
check(fw3.mode == DRIVE and (fw3.left, fw3.right) == (5, -5), "arm + drive 5 -5 -> DRIVE 5/-5")
fw3.advance(301)
check(fw3.mode == OFF and fw3.reason == "timeout", "301 ms without a drive -> OFF 'timeout'")
a.disconnect()

print()
print("GREEN: MOSS base adapter cold bench" if ok else "RED: see KO lines above")
sys.exit(0 if ok else 1)

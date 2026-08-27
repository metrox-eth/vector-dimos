"""Cold tests for the ESP32 muzzle and the sonar brake it drives.

Rule #2: a known input must give a known output, in physical units (metres,
m/s). Groups:
  A. parse_line   - real ESP serial lines
  B. SonarFilter  - median of 3, spread gate, 0.55 m trust cap
  C. CORNERS      - the switch map validated live on 2026-08-25
  D'. brake_forward  - metres in, forward m/s out (vector_dimos.adapter)
  D''. sonar_publication - what goes on the sonar_range stream, and when
  D'''. the same brake through the adapter, in wheel RPM on a mock bus

The ESP module writes nothing into the map any more (sensor doctrine, 2026-08-25),
so there is no patch geometry left to test here.

Run:  .venv/bin/python3 tests/test_esp_sensors_cold.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.esp_sensors import (  # noqa: E402
    CORNERS, SONAR_CLEAR_M, SONAR_MAX_TRUSTED, SonarFilter, parse_line,
    sonar_publication,
)

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


print("A. parse_line (real ESP lines)")
check("SW nominal", parse_line("SW 0 1 0 0") == ("sw", (0, 1, 0, 0)))
check("SONAR nominal", parse_line("SONAR 0.263") == ("sonar", 0.263))
check("SONAR -1", parse_line("SONAR -1") == ("sonar", -1.0))
check("noise -> None", parse_line("MUSEAU-ESP v2: boot") is None)
check("corrupt SW -> None", parse_line("SW 0 x 0 0") is None)

print("B. sonar filter (median of 3, cap 0.55, spread 0.10)")
f = SonarFilter()
check("1 reading -> None", f.feed(0.30) is None)
check("2 readings -> None", f.feed(0.31) is None)
med = f.feed(0.29)
check("3 stable readings -> median 0.30 m", med is not None and abs(med - 0.30) < 1e-9, f"{med}")
f2 = SonarFilter()
f2.feed(0.30); f2.feed(0.31)
check("past the 0.55 m cap -> rejected", f2.feed(0.80) is None)
f3 = SonarFilter()
f3.feed(0.20); f3.feed(0.45)
check("spread >= 0.10 m -> None", f3.feed(0.30) is None)
f4 = SonarFilter()
f4.feed(-1.0)
check("-1 (no echo) ignored", len(f4._readings) == 0)

print("C. corners (map validated 25/08)")
names = [c[0] for c in CORNERS]
check("GPIO order 1-4", names == ["avant-gauche", "arriere-gauche", "arriere-droit", "avant-droit"])
check("front = positive x", CORNERS[0][1][0] > 0 and CORNERS[3][1][0] > 0)
check("rear corners flagged rear", CORNERS[1][2] and CORNERS[2][2] and not CORNERS[0][2])

print("D'. brake_forward (metres in -> forward m/s out)")
from vector_dimos.adapter import (  # noqa: E402
    SONAR_CREEP_MPS, SONAR_SLOW_M, SONAR_STOP_M, SonarBrake, brake_forward,
)

check("thresholds are the doctrine's", SONAR_STOP_M == 0.30 and SONAR_SLOW_M == 0.55
      and SONAR_CREEP_MPS == 0.05, f"{SONAR_STOP_M}/{SONAR_SLOW_M}/{SONAR_CREEP_MPS}")

b = SonarBrake()
v = brake_forward(0.2, 0.60, 0.0, b)
check("0.60 m ahead -> 0.20 m/s (nothing in range)", v == 0.2, f"{v}")
v = brake_forward(0.2, 0.42, 0.0, b)
check("0.42 m ahead -> 0.05 m/s (creep)", v == 0.05, f"{v}")
v = brake_forward(0.2, 0.25, 0.0, b)
check("0.25 m ahead -> 0.00 m/s (stop)", v == 0.0, f"{v}")
v = brake_forward(-0.2, 0.25, 0.0, b)
check("reverse at 0.25 m -> -0.20 m/s untouched", v == -0.2, f"{v}")
v = brake_forward(0.0, 0.25, 0.0, b)
check("pure rotation at 0.25 m -> vx stays 0.00 (wz never clamped)", v == 0.0, f"{v}")
v = brake_forward(0.2, 0.25, 2.0, b)
check("same 0.25 m but 2.0 s old -> 0.20 m/s (stale = no brake)", v == 0.2, f"{v}")
v = brake_forward(0.2, None, 0.0, b)
check("no reading at all -> 0.20 m/s (the sonar is an aid)", v == 0.2, f"{v}")

h = SonarBrake()
v0 = brake_forward(0.2, 0.28, 0.0, h)
v1 = brake_forward(0.2, 0.32, 0.0, h)
v2 = brake_forward(0.2, 0.36, 0.0, h)
check("hysteresis: 0.28 m engages the stop -> 0.00 m/s", v0 == 0.0, f"{v0}")
check("hysteresis: 0.32 m stays stopped (5 cm margin) -> 0.00 m/s", v1 == 0.0, f"{v1}")
check("hysteresis: 0.36 m releases the stop -> 0.05 m/s (still in the slow band)",
      v2 == 0.05, f"{v2}")

v = brake_forward(0.2, SONAR_CLEAR_M, 0.0, SonarBrake())
check(f"the {SONAR_CLEAR_M} m clear heartbeat releases -> 0.20 m/s", v == 0.2, f"{v}")

# the documented 3-argument form: same latch, no explicit state to pass
check("3-arg form, 0.25 m -> 0.00 m/s", brake_forward(0.2, 0.25, 0.0) == 0.0)
check("3-arg form, clear -> 0.20 m/s", brake_forward(0.2, SONAR_CLEAR_M, 0.0) == 0.2)

print("D''. sonar_publication (what reaches the sonar_range stream)")
p = sonar_publication(0.42, 0.0, 0.25)
check("fresh median, 0.25 s since last -> publish 0.42 m", p == 0.42, f"{p}")
from vector_dimos.esp_sensors import SONAR_ENABLED
check("le sonar est DESACTIVE dans la stack (decision 26/08: coussinet)", SONAR_ENABLED is False)
p = sonar_publication(0.42, 0.0, 0.10)
check("same median 0.10 s later -> None (5 Hz cap)", p is None, f"{p}")
p = sonar_publication(None, 1.2, 1.0)
check(f"clear for 1.2 s -> publish {SONAR_CLEAR_M} m", p == SONAR_CLEAR_M, f"{p}")
p = sonar_publication(None, 0.5, 5.0)
check("clear for only 0.5 s -> None", p is None, f"{p}")
p = sonar_publication(None, 3.0, 0.5)
check("clear, but heartbeat sent 0.5 s ago -> None (1 Hz)", p is None, f"{p}")
check("the clear value is past the trust cap", SONAR_CLEAR_M > SONAR_MAX_TRUSTED)

print("D'''. the brake reaches the wheels (mock MODBUS bus, RPM out)")
from vector_dimos.adapter import VectorBaseAdapter  # noqa: E402
from vector_dimos.kinematics import MecanumGeometry  # noqa: E402
from vector_dimos.mock import MockModbusClient  # noqa: E402
from vector_dimos.zlac8015d import L_CMD_RPM, _to_i16  # noqa: E402


def wheel_rpm(adapter, bus, twist):
    """The per-wheel RPM one twist actually puts on the bus."""
    seen = len(bus.writes)
    assert adapter.write_velocities(list(twist)), "write_velocities refused"
    return [(unit, tuple(_to_i16(v) for v in vals))
            for (unit, addr, vals) in bus.writes[seen:] if addr == L_CMD_RPM]


def fresh():
    bus = MockModbusClient()
    a = VectorBaseAdapter(dof=3, client=bus, geometry=MecanumGeometry())
    assert a.connect() and a.write_enable(True)
    return a, bus


braked, bbus = fresh()
free, fbus = fresh()
full = wheel_rpm(free, fbus, (0.2, 0.0, 0.0))
creep = wheel_rpm(free, fbus, (0.05, 0.0, 0.0))
check("no sonar: 0.20 m/s -> 22 RPM per wheel", full == [(2, (22, -22)), (1, (22, -22))], f"{full}")
check("no sonar: 0.05 m/s -> 6 RPM per wheel", creep == [(2, (6, -6)), (1, (6, -6))], f"{creep}")

braked.note_sonar_range(0.42)   # decided 2026-08-26: the sonar INFORMS, it never brakes
got = wheel_rpm(braked, bbus, (0.2, 0.0, 0.0))
check("sonar 0.42 m: 0.20 m/s asked -> UNBRAKED 22 RPM (info only)", got == full, f"{got}")
braked.note_sonar_range(0.25)
got = wheel_rpm(braked, bbus, (0.2, 0.0, 0.0))
check("sonar 0.25 m: 0.20 m/s asked -> UNBRAKED 22 RPM (the evening of 26/08: a sonar stuck at 0.08 m clamped every drive for two hours)", got == full, f"{got}")
got = wheel_rpm(braked, bbus, (-0.2, 0.0, 0.0))
check("sonar 0.25 m: reverse -> the unbraked reverse RPM (-22)",
      got == wheel_rpm(free, fbus, (-0.2, 0.0, 0.0)), f"{got}")
got = wheel_rpm(braked, bbus, (0.0, 0.0, 0.5))
check("sonar 0.25 m: spin 0.5 rad/s -> the unbraked spin RPM",
      got == wheel_rpm(free, fbus, (0.0, 0.0, 0.5)), f"{got}")

print(f"{OK} OK, {KO} KO")
sys.exit(1 if KO else 0)


print("E. bar flutter filter (a corner must HOLD 0.10 s to be a contact)")
import time as _t

from vector_dimos.esp_sensors import BUMP_HOLD_S, EspSensors


class _Sink:
    def __init__(self):
        self.n = 0
    def publish(self, _msg):
        self.n += 1


def _bare():
    e = EspSensors.__new__(EspSensors)
    e._sw = (0, 0, 0, 0)
    e._sonar = None
    e._clear_since = None
    e._last_contact = -10.0
    e._last_sonar_publish = 0.0
    e.contacts = 0
    e.bump = _Sink(); e.bump_rear = _Sink()
    return e


e = _bare()
e._handle_line("SW 1 0 0 0")          # bar flutter: pressed...
e._handle_line("SW 0 0 0 0")          # ...released 20 ms later
_t.sleep(BUMP_HOLD_S + 0.08)
check("flutter (press+release inside the hold) publishes NOTHING", e.bump.n == 0 and e.contacts == 0)

e2 = _bare()
e2._handle_line("SW 1 0 0 0")         # real collision: pressed and HELD
_t.sleep(BUMP_HOLD_S + 0.08)
check("held contact publishes ONE bump after the hold", e2.bump.n == 1 and e2.contacts == 1)

e3 = _bare()
e3._handle_line("SW 0 0 0 1")         # rear corner held
_t.sleep(BUMP_HOLD_S + 0.08)
check("rear corner goes out on bump_rear", e3.bump_rear.n == 1 and e3.bump.n == 0)

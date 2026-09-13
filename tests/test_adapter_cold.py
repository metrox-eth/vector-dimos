"""Cold bench: full adapter on a mocked MODBUS bus. Known in -> known out."""
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.adapter import (BACK_ID, DECEL_MS_FLOOR, DECEL_MS_MAX,
                                  FRONT_ID, VectorBaseAdapter,
                                  resolve_decel_ms)
from vector_dimos.kinematics import MecanumGeometry, inverse, rads_to_rpm
from vector_dimos.mock import MockModbusClient
from vector_dimos.zlac8015d import (COMM_OFFLINE_TIME, CONTROL_REG, ENABLE,
                                    L_ACL_TIME, L_CMD_RPM, L_DCL_TIME, _to_i16)

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and cond


# The geometry the package ships: r = 0.085 m, k = 0.15 + 0.185 = 0.335 m (measured 23/08).
G = MecanumGeometry()
check((G.wheel_radius_m, G.half_wheelbase_m, G.half_track_m)
      == (0.085, 0.15, 0.185),
      f"bench runs on the real default geometry: r={G.wheel_radius_m} m, "
      f"half wheelbase={G.half_wheelbase_m} m, half track={G.half_track_m} m")
bus = MockModbusClient()
a = VectorBaseAdapter(dof=3, client=bus, geometry=G)
check(a.connect() is True, "connect() on mock bus")
check(a.get_dof() == 3, "get_dof() == 3")
check(a.write_enable(True) and a.read_enabled(), "enable sequence")
# The enable sequence writes the accel/decel ramp to BOTH drives, and the
# ramp the package ships is 400 ms (field-tuned on this chassis by the first
# robot code: 500 -> 1000 -> 400). Known value in -> known register out.
ramps = {(u, addr, tuple(vals)) for (u, addr, vals) in bus.writes
         if addr in (L_ACL_TIME, L_DCL_TIME)}
check(ramps == {(FRONT_ID, L_ACL_TIME, (400, 400)), (FRONT_ID, L_DCL_TIME, (400, 400)),
                (BACK_ID, L_ACL_TIME, (400, 400)), (BACK_ID, L_DCL_TIME, (400, 400))},
      f"accel/decel ramp 400 ms written to both drives: {sorted(ramps)}")

# ── 2026-09-13: the DECELERATION ramp is its own number now ────────────────
# metrox 30/08: "inertie teleop excessive : stick lache -> le rover glisse
# encore ~1,5 m". The drives have had separate registers all along (ZLAC8015D
# RS485 manual v1.04: 0x2080/0x2081 accel L/R, 0x2082/0x2083 decel L/R, U16,
# 0-32767 ms); the adapter was writing the same value to all four.
# The check just above is the one that matters most here: DEFAULT UNCHANGED,
# 400/400 on both drives, nobody's flight moves unless they ask.
print("\n  the deceleration ramp (0x2082-0x2083), new 2026-09-13")

# resolve_decel_ms: pure, known value in -> known value out, in milliseconds.
check(resolve_decel_ms(400) == 400,
      "no argument, no env -> decel = accel = 400 ms (the 12/09 behaviour)")
check(resolve_decel_ms(400, 150) == 150,
      "decel_ms=150 -> 150 ms (the value proposed for the workshop trial)")
check(resolve_decel_ms(400, None, {"VECTOR_DECEL_MS": "150"}) == 150,
      "VECTOR_DECEL_MS=150 -> 150 ms (the operator's only lever in a flight: "
      "dimOS builds this adapter from its registry with none of our kwargs)")
check(resolve_decel_ms(400, 150, {"VECTOR_DECEL_MS": "900"}) == 150,
      "an explicit argument beats the environment (150 wins over 900)")
check(resolve_decel_ms(400, None, {"VECTOR_DECEL_MS": "  "}) == 400,
      "an empty VECTOR_DECEL_MS is not an answer -> decel = accel = 400 ms")
check(resolve_decel_ms(400, None, {"VECTOR_DECEL_MS": "pouet"}) == 400,
      "a mistyped VECTOR_DECEL_MS gives YESTERDAY'S ramp (400 ms), never a "
      "random one")
# The low bound is a safety bound on 25 kg, not a style: a ramp under
# DECEL_MS_FLOOR ms can slide the rollers or pitch the chassis forward
# (0.45 m/s stopped in 0.100 s = 4.5 m/s2, against a pitch-over estimate of
# ~6.7 m/s2). Out of range is CLAMPED, never obeyed.
check(resolve_decel_ms(400, 10) == DECEL_MS_FLOOR,
      f"10 ms is refused and clamped to the {DECEL_MS_FLOOR} ms floor "
      f"(25 kg: sliding/pitching)")
check(resolve_decel_ms(400, 0) == DECEL_MS_FLOOR,
      f"0 ms (= 'stop instantly') clamped to {DECEL_MS_FLOOR} ms too")
check(resolve_decel_ms(400, 99999) == DECEL_MS_MAX,
      f"99999 ms clamped to the U16 range of the manual ({DECEL_MS_MAX} ms)")
check((DECEL_MS_FLOOR, DECEL_MS_MAX) == (100, 32767),
      f"bounds are the documented ones: floor {DECEL_MS_FLOOR} ms (safety), "
      f"ceiling {DECEL_MS_MAX} ms (ZLAC8015D manual, U16 0-32767)")

# ...and the number actually reaches the right registers, on the mock bus.
# accel 400 -> 0x2080/0x2081, decel 150 -> 0x2082/0x2083, on BOTH drives.
bus2 = MockModbusClient()
a2 = VectorBaseAdapter(dof=3, client=bus2, geometry=G, decel_ms=150)
check((a2.accel_ms, a2.decel_ms) == (400, 150),
      f"adapter built with decel_ms=150 -> accel {a2.accel_ms} ms, "
      f"decel {a2.decel_ms} ms")
check(a2.connect() and a2.write_enable(True), "enable sequence with a split ramp")
split = {(u, addr, tuple(vals)) for (u, addr, vals) in bus2.writes
         if addr in (L_ACL_TIME, L_DCL_TIME)}
check(split == {(FRONT_ID, L_ACL_TIME, (400, 400)), (FRONT_ID, L_DCL_TIME, (150, 150)),
                (BACK_ID, L_ACL_TIME, (400, 400)), (BACK_ID, L_DCL_TIME, (150, 150))},
      f"400 ms into 0x2080/0x2081 and 150 ms into 0x2082/0x2083, both drives: "
      f"{sorted(split)}")
a2.disconnect()

# the same through the environment - what metrox will actually type
os.environ["VECTOR_DECEL_MS"] = "150"
try:
    bus3 = MockModbusClient()
    a3 = VectorBaseAdapter(dof=3, client=bus3, geometry=G)
    check(a3.connect() and a3.write_enable(True), "enable sequence with VECTOR_DECEL_MS=150")
    env_ramps = {(u, addr, tuple(vals)) for (u, addr, vals) in bus3.writes
                 if addr in (L_ACL_TIME, L_DCL_TIME)}
    check(env_ramps == split,
          f"VECTOR_DECEL_MS=150 writes exactly the same registers as the "
          f"argument: {sorted(env_ramps)}")
    a3.disconnect()
finally:
    del os.environ["VECTOR_DECEL_MS"]

# and with nothing set, the ramp is symmetrical again - the knob is OFF by
# default, which is the property that lets metrox fly one change at a time.
bus4 = MockModbusClient()
a4 = VectorBaseAdapter(dof=3, client=bus4, geometry=G)
check((a4.accel_ms, a4.decel_ms) == (400, 400),
      f"no argument, no env -> {a4.accel_ms}/{a4.decel_ms} ms, symmetrical as before")
a4.disconnect()
# ...the drive-side watchdog (0x2000) is armed at 1000 ms on both drives
# (measured on blocks: wheels at rest < 1.9 s after a SIGKILLed runtime)...
wd = {(u, tuple(vals)) for (u, addr, vals) in bus.writes if addr == COMM_OFFLINE_TIME}
check(wd == {(FRONT_ID, (1000,)), (BACK_ID, (1000,))},
      f"comm-offline watchdog 1000 ms written to both drives: {sorted(wd)}")
# ...and a zero RPM target reaches each drive BEFORE its enable bit, so a
# target left behind by a dirty death is never re-armed.
for unit in (FRONT_ID, BACK_ID):
    seq = [(addr, tuple(vals)) for (u, addr, vals) in bus.writes if u == unit]
    i_zero = seq.index((L_CMD_RPM, (0, 0)))
    i_en = seq.index((CONTROL_REG, (ENABLE,)))
    check(i_zero < i_en,
          f"unit {unit}: zero target written (#{i_zero}) before enable (#{i_en})")
bus.writes.clear()   # the enable sequence's own zero target must not be mistaken for a command

# Pure forward 0.5 m/s: every wheel turns at 0.5 / 0.085 = 5.882 rad/s, i.e.
# +56.17 RPM on all four. The LEFT ports are wired inverted, so the bus must
# see front L/R = (-56, +56) and back L/R = (-56, +56).
FWD = 0.5
exp_fwd = [rads_to_rpm(w) for w in inverse(FWD, 0.0, 0.0, G)]
print(f"      forward {FWD} m/s -> wheel RPM FL/FR/BL/BR = "
      + "/".join(f"{v:+.2f}" for v in exp_fwd))
check(a.write_velocities([FWD, 0.0, 0.0]), "write_velocities accepts twist")
cmds = {(u, tuple(_to_i16(v) for v in vals))
        for (u, addr, vals) in bus.writes if addr == L_CMD_RPM}
front = next(v for (u, v) in cmds if u == FRONT_ID)
back = next(v for (u, v) in cmds if u == BACK_ID)
check(abs(front[0] - (-exp_fwd[0])) <= 1 and abs(front[1] - exp_fwd[1]) <= 1,
      f"front controller L(FL) inverted, R(FR) direct: {front} vs "
      f"({-exp_fwd[0]:+.2f}, {exp_fwd[1]:+.2f}) RPM")
check(abs(back[0] - (-exp_fwd[2])) <= 1 and abs(back[1] - exp_fwd[3]) <= 1,
      f"back controller L(BL) inverted, R(BR) direct: {back} vs "
      f"({-exp_fwd[2]:+.2f}, {exp_fwd[3]:+.2f}) RPM")

# feedback roundtrip: mock echoes commands -> read_velocities returns the twist
v = a.read_velocities()
check(all(abs(x - y) < 0.02 for x, y in zip(v, [FWD, 0.0, 0.0])),
      f"read_velocities roundtrip -> {[round(x, 3) for x in v]}")

# Strafe left 0.4 m/s: the mecanum diagonal. FL/BR roll backward at -44.94 RPM,
# FR/BL forward at +44.94; after the left-port inversion the bus sees
# front L/R = (+45, +45) and back L/R = (-45, -45).
STRAFE = 0.4
exp_str = [rads_to_rpm(w) for w in inverse(0.0, STRAFE, 0.0, G)]
print(f"      strafe {STRAFE} m/s -> wheel RPM FL/FR/BL/BR = "
      + "/".join(f"{v:+.2f}" for v in exp_str))
bus.writes.clear()
a.write_velocities([0.0, STRAFE, 0.0])
raw = {u: tuple(_to_i16(x) for x in vals)
       for (u, addr, vals) in bus.writes if addr == L_CMD_RPM}
check(abs(raw[FRONT_ID][0] - (-exp_str[0])) <= 1
      and abs(raw[FRONT_ID][1] - exp_str[1]) <= 1
      and abs(raw[BACK_ID][0] - (-exp_str[2])) <= 1
      and abs(raw[BACK_ID][1] - exp_str[3]) <= 1,
      f"strafe pattern raw: front {raw[FRONT_ID]} vs "
      f"({-exp_str[0]:+.2f}, {exp_str[1]:+.2f}), back {raw[BACK_ID]} vs "
      f"({-exp_str[2]:+.2f}, {exp_str[3]:+.2f}) RPM")

check(a.write_stop(), "write_stop")
check(a.read_velocities() == [0.0, 0.0, 0.0], "stopped -> zero feedback")
odo = a.read_odometry()
check(isinstance(odo, list) and len(odo) == 3, f"odometry shape {odo}")

# structural Protocol check when dimos is importable (optional)
try:
    from dimos.hardware.drive_trains.spec import TwistBaseAdapter
    check(isinstance(a, TwistBaseAdapter), "isinstance(adapter, TwistBaseAdapter)")
except ImportError:
    print("  ..  dimos not installed here - Protocol check skipped")

a.disconnect()
check(not a.is_connected(), "disconnect")

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

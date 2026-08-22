"""Cold bench: full adapter on a mocked MODBUS bus. Known in -> known out."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.adapter import VectorBaseAdapter, FRONT_ID, BACK_ID
from vector_dimos.kinematics import MecanumGeometry, inverse, rads_to_rpm
from vector_dimos.mock import MockModbusClient
from vector_dimos.zlac8015d import (CONTROL_REG, ENABLE, L_ACL_TIME, L_CMD_RPM,
                                    L_DCL_TIME, _to_i16)

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and cond


# The geometry the package ships: r = 0.085 m, k = 0.15 + 0.20 = 0.35 m.
G = MecanumGeometry()
check((G.wheel_radius_m, G.half_wheelbase_m, G.half_track_m)
      == (0.085, 0.15, 0.20),
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

"""Cold bench: full adapter on a mocked MODBUS bus. Known in -> known out."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.adapter import VectorBaseAdapter, FRONT_ID, BACK_ID
from vector_dimos.kinematics import MecanumGeometry, rads_to_rpm
from vector_dimos.mock import MockModbusClient
from vector_dimos.zlac8015d import L_CMD_RPM, _to_i16

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and cond


G = MecanumGeometry(wheel_radius_m=0.0635, half_wheelbase_m=0.15, half_track_m=0.20)
bus = MockModbusClient()
a = VectorBaseAdapter(dof=3, client=bus, geometry=G)
check(a.connect() is True, "connect() on mock bus")
check(a.get_dof() == 3, "get_dof() == 3")
check(a.write_enable(True) and a.read_enabled(), "enable sequence")

# pure forward 0.5 m/s: all wheels same magnitude; LEFT ports written NEGATIVE
check(a.write_velocities([0.5, 0.0, 0.0]), "write_velocities accepts twist")
expected_rpm = rads_to_rpm(0.5 / G.wheel_radius_m)
cmds = {(u, tuple(_to_i16(v) for v in vals))
        for (u, addr, vals) in bus.writes if addr == L_CMD_RPM}
front = next(v for (u, v) in cmds if u == FRONT_ID)
back = next(v for (u, v) in cmds if u == BACK_ID)
check(front[0] < 0 < front[1] and abs(abs(front[0]) - expected_rpm) < 1,
      f"front controller: L(FL) inverted {front}, |rpm|~{expected_rpm:.0f}")
check(back[0] < 0 < back[1], f"back controller: L(BL) inverted {back}")

# feedback roundtrip: mock echoes commands -> read_velocities returns the twist
v = a.read_velocities()
check(all(abs(x - y) < 0.02 for x, y in zip(v, [0.5, 0.0, 0.0])),
      f"read_velocities roundtrip -> {[round(x, 3) for x in v]}")

# strafe left: FL negative rolling (inverted port -> written POSITIVE raw)
bus.writes.clear()
a.write_velocities([0.0, 0.4, 0.0])
raw = {u: tuple(_to_i16(x) for x in vals)
       for (u, addr, vals) in bus.writes if addr == L_CMD_RPM}
check(raw[FRONT_ID][0] > 0 and raw[FRONT_ID][1] > 0
      and raw[BACK_ID][0] < 0 and raw[BACK_ID][1] < 0,
      f"strafe pattern raw: front {raw[FRONT_ID]}, back {raw[BACK_ID]}")

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

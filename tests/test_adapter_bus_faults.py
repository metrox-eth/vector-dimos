"""Cold bench: what the adapter does when the RS485 bus misbehaves.

The runtime contract being checked: dimos/control/hardware_interface.py
ConnectedTwistBase.read_state() calls read_velocities() then read_odometry()
every control tick. TickLoop._read_all_hardware does catch what that throws,
so an exception is not fatal - it drops the base joints from the tick
snapshot and logs an error at the tick rate. Serving last-known values keeps
the snapshot complete and the log readable.

  a. a bus that answers MODBUS errors -> connect() returns False, naming the
     silent drive (dimOS then refuses to start instead of driving blind);
  b. a bus that dies after connect -> reads keep returning the last known
     values, never raise, the integrated pose FREEZES instead of
     dead-reckoning, the failure counter tracks and recovers, and the
     warning is rate-limited instead of flooding the log;
  c. the two reads of one tick cost ONE bus round-trip per controller, and a
     silent controller is not followed by a second (timeout-priced) read;
  d. VECTOR_MOCK_BUS=1 -> no client needed, and a known twist produces the
     per-wheel RPM the kinematics say it should (rule #2: known in, known
     out, in physical units);
  e. the same, driven by the real dimOS ConnectedTwistBase (skipped when
     dimos is not importable);
  f. a ONE-SIDED outage: the axle that still answers keeps taking new
     commands while the silent one holds its last one. This test pins that
     behaviour rather than blessing it - see the adapter docstring.
  g. a REFUSED enable is loud: dimOS ignores write_enable()'s return value,
     so the failing step and the controller have to be in the log; and a
     drive that enables with a fault flag set gets a warning carrying the
     register value. Logging only - nothing stops the robot.
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos import adapter as adapter_mod
from vector_dimos.adapter import (VectorBaseAdapter, BACK_ID, FRONT_ID,
                                  FEEDBACK_MAX_AGE_S)
from vector_dimos.kinematics import MecanumGeometry, inverse, rads_to_rpm
from vector_dimos.mock import MockModbusClient
from vector_dimos.zlac8015d import (CONTROL_REG, L_CMD_RPM, L_FAULT,
                                    L_FB_RPM, _to_i16)

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and bool(cond)


class RecordingLogger:
    """Wraps the logger the adapter resolved, and keeps the lines."""

    def __init__(self, inner):
        self.inner = inner
        self.lines = []          # (level, message)

    def _emit(self, level, msg):
        self.lines.append((level, msg))
        getattr(self.inner, level)(msg)

    def info(self, msg):
        self._emit("info", msg)

    def warning(self, msg):
        self._emit("warning", msg)

    def error(self, msg):
        self._emit("error", msg)

    def since(self, mark, level=None):
        return [m for lvl, m in self.lines[mark:] if level in (None, lvl)]


LOG = RecordingLogger(adapter_mod._log())
adapter_mod._LOGGER = LOG
print(f"logger backend: {type(LOG.inner).__module__}.{type(LOG.inner).__name__}")


class _ErrorResult:
    """What pymodbus hands back when the drive answers an exception code."""

    def __init__(self):
        self.registers = []

    def isError(self):
        return True


class FlakyBus(MockModbusClient):
    """Mock bus with a kill switch on reads (writes keep working)."""

    def __init__(self, fail_reads=False):
        super().__init__()
        self.fail_reads = fail_reads
        self.closed = False

    def read_holding_registers(self, addr, count, unit=0):
        if self.fail_reads:
            self.reads.append((unit, addr, count))
            return _ErrorResult()
        return super().read_holding_registers(addr, count, unit=unit)

    def close(self):
        self.closed = True


class RaisingBus(FlakyBus):
    """Same, but the client raises instead of answering (port yanked)."""

    def read_holding_registers(self, addr, count, unit=0):
        if self.fail_reads:
            self.reads.append((unit, addr, count))
            raise OSError("[Errno 5] Input/output error: /dev/ttyTHS1")
        return MockModbusClient.read_holding_registers(self, addr, count,
                                                       unit=unit)


print("\n(a) bus answers MODBUS errors at connect")
mark = len(LOG.lines)
bus_a = FlakyBus(fail_reads=True)
a = VectorBaseAdapter(dof=3, client=bus_a)
check(a.connect() is False, "connect() returns False when a drive is silent")
check(a.is_connected() is False, "adapter stays disconnected")
check(bus_a.closed is True, "client was closed before giving up")
check(len(bus_a.reads) >= 1 and bus_a.reads[0][1] == L_FB_RPM,
      f"probe was read-only on the feedback register {bus_a.reads[:1]}")
errors = LOG.since(mark, "error")
check(errors == ["ZLAC8015D id 2 (front) did not answer on "
                 "/dev/ttyTHS1 @115200"],
      f"error names the drive, the port and the baudrate: {errors}")

# the real thing: pymodbus 2.5 RETURNS a ModbusIOException object on a
# timeout (transaction.py:219), it does not raise - that is exactly what
# /dev/ttyTHS1 answers today, with no drives wired.
try:
    from pymodbus.exceptions import ModbusIOException
except ImportError:
    print("  ..  pymodbus not installed here - real timeout object skipped")
else:
    class TimeoutBus(FlakyBus):
        def read_holding_registers(self, addr, count, unit=0):
            if self.fail_reads:
                self.reads.append((unit, addr, count))
                return ModbusIOException(
                    "No Response received from the remote unit", 0x03)
            return MockModbusClient.read_holding_registers(self, addr, count,
                                                           unit=unit)

    t = VectorBaseAdapter(dof=3, client=TimeoutBus(fail_reads=True))
    check(t.connect() is False,
          "a real pymodbus timeout object also fails connect()")

print("\n(b) bus dies after connect: reads degrade, never raise")
G = MecanumGeometry()
bus_b = FlakyBus()
b = VectorBaseAdapter(dof=3, client=bus_b, geometry=G)
check(b.connect() is True, "connect() on a healthy bus")
check(b.write_velocities([0.30, 0.0, 0.0]), "command 0.30 m/s forward")
time.sleep(FEEDBACK_MAX_AGE_S * 2)
good = b.read_velocities()
check(abs(good[0] - 0.30) < 0.02 and b.read_failure_count == 0,
      f"healthy feedback {[round(x, 3) for x in good]}, 0 failures")

# healthy bus: the pose does integrate (arm _last_t, then move)
b.read_odometry()
time.sleep(0.05)
moving = b.read_odometry()
check(moving[0] > 0.005,
      f"healthy bus: pose integrates -> x={moving[0]:.4f} m at 0.30 m/s")

mark = len(LOG.lines)
bus_b.fail_reads = True
seen = []
poses = []
for i in range(3):
    time.sleep(FEEDBACK_MAX_AGE_S * 2)
    seen.append(b.read_velocities())
    odo = b.read_odometry()
    poses.append(odo)
    check(isinstance(odo, list) and len(odo) == 3,
          f"read_odometry survives failure #{i + 1} -> "
          f"{[round(x, 4) for x in odo]}")
# the whole point: last-known VELOCITY is served, last-known POSE is frozen.
# integrating a stale 0.30 m/s would add ~0.3 m per second of outage to /odom.
check(all(pose == moving for pose in poses),
      f"pose frozen on a dead bus: {[round(pose[0], 4) for pose in poses]} "
      f"all equal to {moving[0]:.4f} m")
check(all(v == good for v in seen),
      f"read_velocities keeps serving the last known "
      f"{[round(x, 3) for x in good]}")
check(b.read_failure_count == 3,
      f"failure counter counts the 3 failed polls (got {b.read_failure_count})")
warns = LOG.since(mark, "warning")
check(len(warns) == 1, f"3 failures inside {adapter_mod.READ_WARN_PERIOD_S}s -> "
                       f"1 warning, not a flood ({len(warns)})")

# the rate limit must re-fire once its period is over (shortened here)
period = adapter_mod.READ_WARN_PERIOD_S
adapter_mod.READ_WARN_PERIOD_S = 0.05
mark = len(LOG.lines)
time.sleep(0.06)
b.read_velocities()
adapter_mod.READ_WARN_PERIOD_S = period
check(len(LOG.since(mark, "warning")) == 1,
      "a still-dead bus warns again once the rate-limit period is over")

mark = len(LOG.lines)
bus_b.fail_reads = False
time.sleep(FEEDBACK_MAX_AGE_S * 2)
back_up = b.read_velocities()
check(b.read_failure_count == 0, "recovery resets the failure counter")
check(abs(back_up[0] - 0.30) < 0.02,
      f"feedback is live again {[round(x, 3) for x in back_up]}")
time.sleep(0.05)
resumed = b.read_odometry()
check(resumed[0] > moving[0] + 0.005,
      f"pose integrates again after recovery: {resumed[0]:.4f} m "
      f"> {moving[0]:.4f} m")
infos = LOG.since(mark, "info")
check(infos == ["VECTOR base: wheel feedback recovered after 4 failures"],
      f"exactly one recovery line: {infos}")

# same story when the client raises instead of answering an error
bus_r = RaisingBus()
r = VectorBaseAdapter(dof=3, client=bus_r, geometry=G)
r.connect()
bus_r.fail_reads = True
time.sleep(FEEDBACK_MAX_AGE_S * 2)
try:
    v = r.read_velocities()
    o = r.read_odometry()
    raised = False
except Exception as exc:                      # noqa: BLE001 - that is the test
    raised, v, o = True, None, None
    print(f"      raised: {exc!r}")
check(not raised and v == [0.0, 0.0, 0.0] and o is not None,
      "a raising client is absorbed: last known served, no exception")
check(r.read_failure_count == 1,
      "one failed poll counted (both reads, one poll)")

print("\n(c) one bus round-trip per control tick")
bus_c = MockModbusClient()
c = VectorBaseAdapter(dof=3, client=bus_c, geometry=G)
c.connect()
time.sleep(FEEDBACK_MAX_AGE_S * 2)
bus_c.reads.clear()
c.read_velocities()
c.read_odometry()          # same tick, must reuse the cache
units = [u for (u, addr, _n) in bus_c.reads if addr == L_FB_RPM]
check(len(bus_c.reads) == 2 and sorted(units) == sorted([FRONT_ID, BACK_ID]),
      f"read_velocities + read_odometry = 1 read per controller {bus_c.reads}")
time.sleep(FEEDBACK_MAX_AGE_S * 2)
c.read_velocities()
check(len(bus_c.reads) == 4,
      f"after the cache expires the bus is polled again ({len(bus_c.reads)})")

# on the real bus an unanswered read costs the full serial timeout (~0.5 s),
# so a silent front controller must not be followed by a back read whose
# answer would be discarded anyway.
bus_sc = FlakyBus()
sc = VectorBaseAdapter(dof=3, client=bus_sc, geometry=G)
sc.connect()
bus_sc.fail_reads = True
time.sleep(FEEDBACK_MAX_AGE_S * 2)
bus_sc.reads.clear()
sc.read_velocities()
sc.read_odometry()
check(len(bus_sc.reads) == 1 and bus_sc.reads[0][0] == FRONT_ID,
      f"a silent front is not followed by a back read {bus_sc.reads}")

print("\n(d) VECTOR_MOCK_BUS=1: no client, real geometry, known RPM")
os.environ["VECTOR_MOCK_BUS"] = "1"
mark = len(LOG.lines)
m = VectorBaseAdapter(dof=3)          # no client injected
check(m.connect() is True, "connect() with no client and no hardware")
check(m.mock_bus is True and isinstance(m.client, MockModbusClient),
      "the mock bus was substituted")
loud = [line for line in LOG.since(mark) if "MOCK BUS" in line]
check(loud == ["VECTOR base: MOCK BUS (VECTOR_MOCK_BUS set) - "
               "no motors will move"],
      f"exactly one loud mock-bus line: {loud}")
check(m.write_enable(True) and m.read_enabled(), "enable sequence on mock bus")

VX, VY, WZ = 0.37, -0.21, 0.83
w_fl, w_fr, w_bl, w_br = inverse(VX, VY, WZ, MecanumGeometry())
exp = [rads_to_rpm(w) for w in (w_fl, w_fr, w_bl, w_br)]
print(f"      expected wheel RPM from kinematics (r={G.wheel_radius_m}, k={G.k}): "
      f"FL={exp[0]:.2f} FR={exp[1]:.2f} BL={exp[2]:.2f} BR={exp[3]:.2f}")
m.client.writes.clear()
mark = len(LOG.lines)
check(m.write_velocities([VX, VY, WZ]), "write_velocities on mock bus")
raw = {u: tuple(_to_i16(v) for v in vals)
       for (u, addr, vals) in m.client.writes if addr == L_CMD_RPM}
print(f"      bus raw: front(id {FRONT_ID}) L/R={raw[FRONT_ID]}, "
      f"back(id {BACK_ID}) L/R={raw[BACK_ID]}")
# left ports (FL, BL) are inverted on this chassis: negative RPM = forward
check(abs(raw[FRONT_ID][0] - (-exp[0])) <= 1,
      f"front L port = -FL: {raw[FRONT_ID][0]} vs {-exp[0]:.2f} RPM")
check(abs(raw[FRONT_ID][1] - exp[1]) <= 1,
      f"front R port = FR: {raw[FRONT_ID][1]} vs {exp[1]:.2f} RPM")
check(abs(raw[BACK_ID][0] - (-exp[2])) <= 1,
      f"back L port = -BL: {raw[BACK_ID][0]} vs {-exp[2]:.2f} RPM")
check(abs(raw[BACK_ID][1] - exp[3]) <= 1,
      f"back R port = BR: {raw[BACK_ID][1]} vs {exp[3]:.2f} RPM")

# the mock-mode command line: once per change, and only on change
m.write_velocities([VX, VY, WZ])
m.write_velocities([0.0, 0.0, 0.0])
cmd_lines = [line for line in LOG.since(mark) if "MOCK: twist" in line]
check(len(cmd_lines) == 2,
      f"3 writes, 2 distinct commands -> 2 log lines ({len(cmd_lines)})")
check("FL=+33 FR=+51 BL=-15 BR=+98" in cmd_lines[0],
      f"the logged RPM is the commanded RPM: {cmd_lines[0]}")

m.write_velocities([VX, VY, WZ])
time.sleep(FEEDBACK_MAX_AGE_S * 2)
twist = m.read_velocities()
check(all(abs(x - y) < 0.02 for x, y in zip(twist, [VX, VY, WZ])),
      f"mock echo roundtrip -> {[round(x, 3) for x in twist]} "
      f"(commanded {[VX, VY, WZ]})")
m.disconnect()

print("\n(e) same adapter driven by the real dimOS runtime wrapper")
try:
    from dimos.control.components import (HardwareComponent, HardwareType,
                                          make_twist_base_joints)
    from dimos.control.hardware_interface import ConnectedTwistBase
except ImportError:
    print("  ..  dimos not installed here - runtime check skipped")
else:
    joints = make_twist_base_joints("base")
    comp = HardwareComponent(hardware_id="base", hardware_type=HardwareType.BASE,
                             joints=joints, adapter_type="vector")
    twist_cmd = dict(zip(joints, (VX, VY, WZ)))

    # e1: mock bus - exactly what `dimos run vector-dimos.base` will do
    rt = VectorBaseAdapter(dof=3)
    check(rt.connect() is True, "mock-bus adapter connects under the runtime")
    hw = ConnectedTwistBase(rt, comp)
    rt.client.writes.clear()
    check(hw.write_command(twist_cmd, None) is True,
          "ConnectedTwistBase.write_command(twist)")
    raw_rt = {u: tuple(_to_i16(v) for v in vals)
              for (u, addr, vals) in rt.client.writes if addr == L_CMD_RPM}
    check(raw_rt == raw,
          f"runtime produces the same per-wheel RPM as the direct call {raw_rt}")
    time.sleep(FEEDBACK_MAX_AGE_S * 2)
    rt.client.reads.clear()
    vel = [hw.read_state()[j].velocity for j in joints]
    check(len(rt.client.reads) == 2,
          f"one read_state() tick = 2 bus reads {rt.client.reads}")
    check(all(abs(x - y) < 0.02 for x, y in zip(vel, [VX, VY, WZ])),
          f"read_state velocities {[round(x, 3) for x in vel]} "
          f"(commanded {[VX, VY, WZ]})")
    rt.disconnect()

    # e2: the bus dies under the runtime - read_state() has no try/except
    dead_bus = RaisingBus()
    rt2 = VectorBaseAdapter(dof=3, client=dead_bus, geometry=G)
    rt2.connect()
    hw2 = ConnectedTwistBase(rt2, comp)
    hw2.write_command(twist_cmd, None)
    time.sleep(FEEDBACK_MAX_AGE_S * 2)
    live = [hw2.read_state()[j].velocity for j in joints]
    dead_bus.fail_reads = True
    time.sleep(FEEDBACK_MAX_AGE_S * 2)
    try:
        dead = [hw2.read_state()[j].velocity for j in joints]
        raised_rt = False
    except Exception as exc:                  # noqa: BLE001 - that is the test
        raised_rt, dead = True, None
        print(f"      raised: {exc!r}")
    check(not raised_rt and dead == live,
          f"read_state() on a dead bus: no exception, last known "
          f"{[round(x, 3) for x in dead] if dead else dead}")
    check(rt2.read_failure_count == 1,
          f"one poll per tick even while failing ({rt2.read_failure_count})")

os.environ.pop("VECTOR_MOCK_BUS")

print("\n(f) one-sided outage: the axle that answers keeps taking commands")


class OneSidedBus(MockModbusClient):
    """Writes to one unit are refused; reads and the other unit are fine."""

    def __init__(self, dead_unit):
        super().__init__()
        self.dead_unit = dead_unit

    def write_registers(self, addr, values, unit=0):
        if unit == self.dead_unit:
            self.writes.append((unit, addr, list(values)))
            return _ErrorResult()
        return super().write_registers(addr, values, unit=unit)


bus_f = OneSidedBus(dead_unit=FRONT_ID)
f = VectorBaseAdapter(dof=3, client=bus_f, geometry=G)
check(f.connect() is True, "connect() still succeeds (reads answer, writes do not)")
bus_f.writes.clear()
check(f.write_velocities([VX, VY, WZ]) is False,
      "write_velocities() reports False when one controller refuses")
raw_f = {u: tuple(_to_i16(v) for v in vals)
         for (u, addr, vals) in bus_f.writes if addr == L_CMD_RPM}
exp_back = (round(-exp[2]), round(exp[3]))
print(f"      front(id {FRONT_ID}) refused, back(id {BACK_ID}) got "
      f"L/R={raw_f.get(BACK_ID)}, geometry says {exp_back}")
check(raw_f.get(BACK_ID) == exp_back,
      f"the healthy back axle STILL executes the new twist {raw_f.get(BACK_ID)} "
      f"= {exp_back} RPM - pinned, not endorsed (see adapter docstring)")
f.disconnect()

print("\n(g) a refused enable, and a drive that enables with a fault set")


class RefusesEnableBus(MockModbusClient):
    """One controller refuses the 0x200E enable write; the rest is fine."""

    def __init__(self, dead_unit):
        super().__init__()
        self.dead_unit = dead_unit

    def write_register(self, addr, value, unit=0):
        if unit == self.dead_unit and addr == CONTROL_REG:
            self.writes.append((unit, addr, [value]))
            return _ErrorResult()
        return super().write_register(addr, value, unit=unit)


bus_g = RefusesEnableBus(dead_unit=FRONT_ID)
g = VectorBaseAdapter(dof=3, client=bus_g, geometry=G)
check(g.connect() is True, "connect() succeeds (the drives answer reads)")
mark = len(LOG.lines)
check(g.write_enable(True) is False,
      "write_enable() returns False when a controller refuses 0x200E")
check(g.read_enabled() is False, "the adapter does not claim to be enabled")
errors = LOG.since(mark, "error")
check(errors == ["ZLAC8015D id 2 (front) enable sequence refused at: "
                 "enable (0x200E)"],
      f"exactly one error line, naming the controller and the step: {errors}")
infos = LOG.since(mark, "info")
check(infos == ["ZLAC8015D id 1 (back) enabled, faults L/R=0/0"],
      f"the controller that DID enable reports its fault registers: {infos}")
g.disconnect()

bus_h = MockModbusClient()
# 0x20A5 = the L-channel fault register: a set bit = a latched fault.
bus_h.regs[(FRONT_ID, L_FAULT)] = 0x0002
h = VectorBaseAdapter(dof=3, client=bus_h, geometry=G)
check(h.connect() is True, "connect() on a bus whose front drive has a fault")
mark = len(LOG.lines)
check(h.write_enable(True) is True,
      "the enable writes are accepted, so write_enable() is True")
warns = LOG.since(mark, "warning")
check(warns == ["ZLAC8015D id 2 (front) enabled WITH FAULTS L/R=0x0002/0x0000 "
                "- see the ZLAC8015D manual for the bit meanings"],
      f"one warning, carrying the register value: {warns}")
check(LOG.since(mark, "info") == ["ZLAC8015D id 1 (back) enabled, "
                                  "faults L/R=0/0"],
      "the healthy controller still logs its clean fault read")
h.disconnect()

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

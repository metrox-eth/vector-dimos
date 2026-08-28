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
  f. ARMING IS A TRANSACTION over the pair: a healthy bus arms both, and no
     injected failure may ever leave ONE axle armed. A refused prerequisite
     (one lost frame on the 0x2000 watchdog or the 0x200D mode write) arms
     nobody, a refused ENABLE on the front never reaches the back, and a
     refused ENABLE on the back rolls the front back to zero + disable.
     dimOS ignores write_enable()'s False, so the hardware state we leave
     behind is the whole protection.
     A drive that RAISES counts as a drive that refused, in all three phases
     and on the disarm: zlac8015d swallows every serial exception today, and
     that promise - made in another file - was the only thing stopping a raise
     from flying out of _arm_both into write_enable's outer except and
     returning False with the front axle armed and nothing to roll it back
     (review of 3ea2a00, 2026-08-28).
  g. THE FAULT REGISTERS DECIDE, and they are read BEFORE the ENABLE word.
     Non-zero on either drive, or a fault register that does not answer, and
     nobody is armed (metrox, 28/08 18:37). "enabled WITH FAULTS" used to be
     a warning after the fact, with write_enable returning True.
  h. A REFUSED COMMAND LATCHES THE WHOLE BASE. The two set_rpm calls go out
     back to back, so the healthy axle has already taken the new twist by
     the time the other one refuses - which is how a one-sided RS485 outage
     became a pirouette. The refusal now latches the base, zeroes and
     disables both controllers, and refuses every later non-zero command
     WITHOUT touching the bus; a stop always goes through, and only a
     complete write_enable(True) lets commands flow again.

  Sections f, g and h used to pin the opposite behaviour ("pinned, not
  endorsed"). The external audit of 2026-08-28 called both of them P0 and
  metrox chose the policy; these are the tests of that policy.
  i. the same tick, against a bus that takes REAL TIME to answer. Every
     other mock here answers in microseconds, so (b), (c) and (e2) count
     one poll per tick even with the cache stamped BEFORE the poll - the
     stamp is still fresh when the second read of the tick arrives. With
     the 0.5 s serial timeout on the line a poll outlives its own 15 ms
     window, and only a timestamp taken AFTER the poll keeps the tick at
     one round-trip per controller.
"""
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos import adapter as adapter_mod
from vector_dimos.adapter import (VectorBaseAdapter, BACK_ID, FRONT_ID,
                                  FEEDBACK_MAX_AGE_S, SERIAL_TIMEOUT_S)
from vector_dimos.kinematics import MecanumGeometry, inverse, rads_to_rpm
from vector_dimos.mock import MockModbusClient
from vector_dimos.zlac8015d import (COMM_OFFLINE_TIME, CONTROL_REG, DISABLE,
                                    ENABLE, L_CMD_RPM, L_FAULT, L_FB_RPM,
                                    OPR_MODE, R_FAULT, VEL_CONTROL, _to_i16)

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
from vector_dimos.adapter import DEFAULT_PORT
errors = LOG.since(mark, "error")
check(errors == [f"ZLAC8015D id 2 (front) did not answer on {DEFAULT_PORT} @115200"],
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
# values recomputed for the 24/08 body flip + the 0.085 m wheel (the old
# literal predated BODY_FLIPPED and had been red since then)
check("FL=-96 FR=+13 BL=-49 BR=-34" in cmd_lines[0],
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

class PolicyBus(MockModbusClient):
    """A bus with a per-(unit, register) kill switch, and a call log to read.

    `refuse` is the set of (unit, addr) pairs whose WRITES are answered with
    a MODBUS error: the frame goes out, the drive does not take it - one
    lost or CRC-broken frame on the RS485 run next to the two motor drives,
    or a pigtail that came loose. The written value is deliberately NOT
    stored, which is exactly the state the adapter has to assume.

    `silent_faults` is the set of units whose fault registers never answer.

    The mutation is live: a test arms on a healthy bus, then breaks it, which
    is the only way to reach the runtime latch (a bus broken from the start
    never gets past the arming transaction).
    """

    def __init__(self):
        super().__init__()
        self.refuse = set()
        self.silent_faults = set()

    def write_register(self, addr, value, unit=0):
        if (unit, addr) in self.refuse:
            self.writes.append((unit, addr, [value]))
            return _ErrorResult()
        return super().write_register(addr, value, unit=unit)

    def write_registers(self, addr, values, unit=0):
        if (unit, addr) in self.refuse:
            self.writes.append((unit, addr, list(values)))
            return _ErrorResult()
        return super().write_registers(addr, values, unit=unit)

    def read_holding_registers(self, addr, count, unit=0):
        if addr == L_FAULT and unit in self.silent_faults:
            self.reads.append((unit, addr, count))
            return _ErrorResult()
        return super().read_holding_registers(addr, count, unit=unit)

    # ── what the policy is asserted on ─────────────────────────────────
    def control_words(self):
        """Every 0x200E write attempt, in bus order: [(unit, value), ...]."""
        return [(u, v[0]) for (u, a, v) in self.writes if a == CONTROL_REG]

    def rpm_cmds(self, unit=None):
        """Every 0x2088 write attempt, signed: [(unit, (L, R)), ...]."""
        return [(u, tuple(_to_i16(x) for x in v)) for (u, a, v) in self.writes
                if a == L_CMD_RPM and unit in (None, u)]

    def armed(self):
        """Units whose control word actually READS as ENABLE right now."""
        return sorted(u for u in (FRONT_ID, BACK_ID)
                      if self.regs.get((u, CONTROL_REG)) == ENABLE)


def policy_adapter(bus, label):
    """A connected adapter on `bus`. connect() is read-only, so it survives
    any write kill switch."""
    a = VectorBaseAdapter(dof=3, client=bus, geometry=G)
    check(a.connect() is True, f"connect() on {label}")
    return a


print("\n(f) arming is a transaction: never ONE armed axle")

# f1 - the healthy baseline, in the drives' own units: the watchdog at
# COMM_OFFLINE_MS ms, the mode at VEL_CONTROL, and the ENABLE word written
# LAST per drive (after the watchdog and the zero target). Everything below
# is a deviation from this line.
bus_f1 = PolicyBus()
f1 = policy_adapter(bus_f1, "a healthy bus")
check(f1.write_enable(True) is True, "healthy bus: write_enable(True) is True")
check(f1.read_enabled() is True, "and the adapter says it is enabled")
wd = {u: bus_f1.regs.get((u, COMM_OFFLINE_TIME)) for u in (FRONT_ID, BACK_ID)}
mode = {u: bus_f1.regs.get((u, OPR_MODE)) for u in (FRONT_ID, BACK_ID)}
print(f"      healthy enable -> watchdog {wd} ms, mode {mode}, "
      f"control words {bus_f1.control_words()}")
check(all(v == adapter_mod.COMM_OFFLINE_MS for v in wd.values()),
      f"0x2000 watchdog = {adapter_mod.COMM_OFFLINE_MS} ms on both drives {wd}")
check(all(v == VEL_CONTROL for v in mode.values()),
      f"0x200D mode = {VEL_CONTROL} (velocity) on both drives {mode}")
check(bus_f1.armed() == sorted([FRONT_ID, BACK_ID]),
      f"BOTH drives armed {bus_f1.armed()}")
front_seq = [addr for (u, addr, _v) in bus_f1.writes if u == FRONT_ID]
check(front_seq[-1] == CONTROL_REG and COMM_OFFLINE_TIME in front_seq[:-1],
      f"ENABLE is the LAST write of that drive, after the watchdog: "
      f"{[hex(a) for a in front_seq]}")
# the transaction order itself: nobody is armed while the other is still
# being prepared, so no ENABLE may precede the last preparation write.
first_enable = next(idx for idx, (u, a, _v) in enumerate(bus_f1.writes)
                    if a == CONTROL_REG)
last_prep = max(idx for idx, (u, a, _v) in enumerate(bus_f1.writes)
                if a != CONTROL_REG)
check(first_enable > last_prep,
      f"both drives are fully prepared before the FIRST enable "
      f"(first 0x200E at #{first_enable}, last prep write at #{last_prep})")
f1.disconnect()

# f2 - one dropped response on a PREREQUISITE of the FRONT drive only.
# This is the audited P0: the old loop logged the front's abstention, hit
# `continue`, and armed the back anyway.
for dead_addr, what, step in (
        (COMM_OFFLINE_TIME, "the comm-offline watchdog 0x2000",
         "comm-offline watchdog (0x2000)"),
        (OPR_MODE, "the velocity mode 0x200D", "velocity mode (0x200D)"),
        (L_CMD_RPM, "the zero target 0x2088",
         "zero target (0x2088) before enable")):
    print(f"    -- the FRONT drive drops the response on {what} --")
    bus_f2 = PolicyBus()
    bus_f2.refuse.add((FRONT_ID, dead_addr))
    f2 = policy_adapter(bus_f2, "a bus whose front drops one frame")
    mark = len(LOG.lines)
    check(f2.write_enable(True) is False,
          "write_enable() returns False (dimOS coordinator.py:259 ignores it)")
    check(f2.read_enabled() is False, "the adapter does not claim to be enabled")
    print(f"      0x200E write attempts: {bus_f2.control_words()}")
    check(bus_f2.control_words() == [],
          f"0x200E was NEVER written, to EITHER drive - the back is not armed "
          f"behind the front's refusal {bus_f2.control_words()}")
    check(bus_f2.armed() == [], f"no drive is armed {bus_f2.armed()}")
    errors = LOG.since(mark, "error")
    check(errors == [f"ZLAC8015D id {FRONT_ID} (front) NOT armed, ENABLE "
                     f"(0x200E) withheld - refused at: {step}"],
          f"the log names the drive and the refused step: {errors}")
    f2.disconnect()

# ... and when the frame is lost for BOTH drives, both are named in one go
print("    -- both drives drop the response on the watchdog 0x2000 --")
bus_f3 = PolicyBus()
bus_f3.refuse.update({(FRONT_ID, COMM_OFFLINE_TIME),
                      (BACK_ID, COMM_OFFLINE_TIME)})
f3 = policy_adapter(bus_f3, "a bus where the watchdog write is lost on both")
mark = len(LOG.lines)
check(f3.write_enable(True) is False, "write_enable() returns False")
check(bus_f3.control_words() == [], "no ENABLE frame at all")
check(len(LOG.since(mark, "error")) == 2,
      f"both controllers are named in the same attempt: "
      f"{LOG.since(mark, 'error')}")
f3.disconnect()

# f4 - the FRONT refuses the ENABLE word itself: the back must never see one.
print("    -- the FRONT drive refuses the ENABLE word 0x200E --")
bus_f4 = PolicyBus()
bus_f4.refuse.add((FRONT_ID, CONTROL_REG))
f4 = policy_adapter(bus_f4, "a bus whose front refuses 0x200E")
mark = len(LOG.lines)
check(f4.write_enable(True) is False, "write_enable() is False")
check(f4.read_enabled() is False, "the adapter does not claim to be enabled")
print(f"      0x200E write attempts: {bus_f4.control_words()}")
check(bus_f4.control_words() == [(FRONT_ID, ENABLE)],
      f"the front's refused ENABLE is the ONLY control-word frame - the back "
      f"never gets one {bus_f4.control_words()}")
check(bus_f4.armed() == [], f"no drive is armed {bus_f4.armed()}")
check(LOG.since(mark, "error") == [f"ZLAC8015D id {FRONT_ID} (front) enable "
                                   "sequence refused at: enable (0x200E)"],
      f"one error, naming the drive and the step: {LOG.since(mark, 'error')}")
f4.disconnect()

# f5 - the BACK refuses the ENABLE word, so the FRONT is already armed:
# rollback, and the rollback is zero-then-disable (a drive disarmed with a
# live RPM target would run it the moment anything re-arms it).
print("    -- the BACK drive refuses the ENABLE word: rollback of the front --")
bus_f5 = PolicyBus()
bus_f5.refuse.add((BACK_ID, CONTROL_REG))
f5 = policy_adapter(bus_f5, "a bus whose back refuses 0x200E")
mark = len(LOG.lines)
check(f5.write_enable(True) is False, "write_enable() is False")
check(f5.read_enabled() is False, "the adapter does not claim to be enabled")
print(f"      0x200E write attempts: {bus_f5.control_words()}")
check(bus_f5.control_words() == [(FRONT_ID, ENABLE), (BACK_ID, ENABLE),
                                 (FRONT_ID, DISABLE)],
      f"the front was armed, the back refused, the front was DISABLED again "
      f"{bus_f5.control_words()}")
check(bus_f5.armed() == [],
      f"nothing stays under torque: no drive reads as armed {bus_f5.armed()}")
front_rpm = bus_f5.rpm_cmds(FRONT_ID)
check(front_rpm[-1] == (FRONT_ID, (0, 0)),
      f"and the front got a ZERO target before its disable {front_rpm[-1]}")
rollback = [m for m in LOG.since(mark, "error") if "rolling it back" in m]
check(len(rollback) == 1, f"the rollback is loud: {rollback}")
f5.disconnect()

# f6-f10 - the same transaction, against a drive that RAISES instead of
# answering False. The raise has to be injected at the CONTROLLER, not on the
# bus: zlac8015d's _write_register / _write_registers / _read swallow every
# serial exception, and that promise - made in another file - was the only
# reason an escaping raise never left an axle armed (review of 3ea2a00).
print("    -- a controller that RAISES instead of returning False --")


class RaisingController:
    """A ZLAC8015D whose `where` method raises; everything else is the real one."""

    def __init__(self, inner, where="enable"):
        self._inner = inner
        self._where = where

    def __getattr__(self, name):
        if name == self._where:
            def boom(*_a, **_kw):
                raise OSError("[Errno 5] Input/output error: /dev/ttyUSB0")
            return boom
        return getattr(self._inner, name)


# f6 - THE audited shape: the front is armed, then the back's enable RAISES.
# Before this guard the raise flew out of _arm_both into write_enable's outer
# except: False returned, front axle still under torque, nothing rolled back.
bus_f6 = PolicyBus()
f6 = policy_adapter(bus_f6, "a healthy bus whose back drive will raise")
f6._back = RaisingController(f6._back)
mark = len(LOG.lines)
check(f6.write_enable(True) is False, "the back RAISED: write_enable() is False")
check(f6.read_enabled() is False, "the adapter does not claim to be enabled")
print(f"      0x200E write attempts: {bus_f6.control_words()}")
check(bus_f6.control_words() == [(FRONT_ID, ENABLE), (FRONT_ID, DISABLE)],
      f"the front was armed, the back raised, the front was DISABLED again - "
      f"a raise rolls back exactly like a refusal {bus_f6.control_words()}")
check(bus_f6.armed() == [], f"NO axle stays under torque {bus_f6.armed()}")
check(bus_f6.rpm_cmds(FRONT_ID)[-1] == (FRONT_ID, (0, 0)),
      f"and the front got a ZERO target before its disable "
      f"{bus_f6.rpm_cmds(FRONT_ID)[-1]}")
check(len([m for m in LOG.since(mark, "error") if "rolling it back" in m]) == 1,
      f"the rollback is as loud as on a refusal: {LOG.since(mark, 'error')}")
f6.disconnect()

# f7 - the FRONT's enable raises: the back must never see an ENABLE word.
bus_f7 = PolicyBus()
f7 = policy_adapter(bus_f7, "a healthy bus whose front drive will raise")
f7._front = RaisingController(f7._front)
check(f7.write_enable(True) is False, "write_enable() is False")
check(bus_f7.control_words() == [],
      f"not one 0x200E frame: the back is not armed behind a front that raised "
      f"{bus_f7.control_words()}")
check(bus_f7.armed() == [] and f7.read_enabled() is False, "nothing armed, nothing claimed")
f7.disconnect()

# f8 - phase 1 (prepare): nothing is armed there, so a raise could not leave an
# axle under torque - but it DID skip the other drive's preparation, the log
# line naming the culprit, and the `_enabled = False` a failed transaction owes.
bus_f8 = PolicyBus()
f8 = policy_adapter(bus_f8, "a bus whose front raises on the 0x2000 watchdog")
f8._front = RaisingController(f8._front, where="set_comm_offline_ms")
mark = len(LOG.lines)
check(f8.write_enable(True) is False, "a raise in phase 1 is a refusal, not an escape")
check(f8.read_enabled() is False, "the adapter does not claim to be enabled")
check(bus_f8.control_words() == [] and bus_f8.armed() == [],
      f"0x200E withheld from BOTH {bus_f8.control_words()}")
check(LOG.since(mark, "error") == [f"ZLAC8015D id {FRONT_ID} (front) NOT armed, "
                                   "ENABLE (0x200E) withheld - refused at: "
                                   "comm-offline watchdog (0x2000)"],
      f"the log still names the drive and the step: {LOG.since(mark, 'error')}")
check(bus_f8.regs.get((BACK_ID, COMM_OFFLINE_TIME)) == adapter_mod.COMM_OFFLINE_MS,
      "and the BACK was still prepared - a raise on one side never skips the other")
f8.disconnect()

# f9 - phase 2: a fault register that RAISES is the same non-answer as one that
# does not reply. A drive we cannot question is a drive we do not arm.
bus_f9 = PolicyBus()
f9 = policy_adapter(bus_f9, "a bus whose front raises on 0x20A5")
f9._front = RaisingController(f9._front, where="get_faults")
mark = len(LOG.lines)
check(f9.write_enable(True) is False, "write_enable() is False")
check(bus_f9.control_words() == [] and bus_f9.armed() == [], "nobody armed")
check(any("did not answer" in m for m in LOG.since(mark, "error")),
      f"read as unreadable, and said so: {LOG.since(mark, 'error')}")
f9.disconnect()

# f10 - disarming: a raise on one side must not skip the other's disable. The
# axle we can still reach is the one that matters.
bus_f10 = PolicyBus()
f10 = policy_adapter(bus_f10, "a healthy bus, armed, then the front raises on disable")
check(f10.write_enable(True) is True, "armed on a healthy bus")
f10._front = RaisingController(f10._front, where="disable")
mark = len(LOG.lines)
check(f10.write_enable(False) is False, "a raise on disarm is a refusal too")
check(bus_f10.armed() == [FRONT_ID],
      f"the BACK was still asked to disarm and did {bus_f10.armed()}")
check(f10.read_enabled() is True,
      "and the adapter still says enabled - the front may well be under torque")
check(any(f"id {FRONT_ID} (front) refused disable" in m for m in LOG.since(mark, "error")),
      f"the drive out of reach is named: {LOG.since(mark, 'error')}")
f10.disconnect()

print("\n(g) the fault registers decide, BEFORE the ENABLE word")

# 0x20A5/0x20A6 = the L/R fault registers: a set bit is a latched fault.
for unit, side, reg, label in ((FRONT_ID, "front", L_FAULT, "L"),
                               (BACK_ID, "back", R_FAULT, "R")):
    print(f"    -- the {side} drive has a latched fault on its {label} channel --")
    bus_g = PolicyBus()
    bus_g.regs[(unit, reg)] = 0x0002
    g = policy_adapter(bus_g, f"a bus whose {side} drive has a fault")
    mark = len(LOG.lines)
    check(g.write_enable(True) is False,
          "write_enable() REFUSES to arm through a latched fault")
    check(g.read_enabled() is False, "the adapter does not claim to be enabled")
    check(bus_g.control_words() == [],
          f"no ENABLE frame on the bus at all {bus_g.control_words()}")
    lr = ("0x0002/0x0000" if reg == L_FAULT else "0x0000/0x0002")
    check(LOG.since(mark, "error") == [
              f"ZLAC8015D id {unit} ({side}) NOT armed: FAULTS L/R={lr} - see "
              "the ZLAC8015D manual for the bit meanings, then clear them"],
          f"the error carries the register value: {LOG.since(mark, 'error')}")
    g.disconnect()

print("    -- the front drive's fault registers do not answer --")
bus_g3 = PolicyBus()
bus_g3.silent_faults.add(FRONT_ID)
g3 = policy_adapter(bus_g3, "a bus whose front fault registers are silent")
mark = len(LOG.lines)
check(g3.write_enable(True) is False,
      "an UNREADABLE fault register is not a pass: write_enable() is False")
check(g3.read_enabled() is False, "the adapter does not claim to be enabled")
check(bus_g3.control_words() == [],
      f"no ENABLE frame on the bus at all {bus_g3.control_words()}")
check(LOG.since(mark, "error") == [
          f"ZLAC8015D id {FRONT_ID} (front) NOT armed: its fault registers "
          "(0x20A5/0x20A6) did not answer - a drive we cannot question is a "
          "drive we do not arm"],
      f"and the log says why: {LOG.since(mark, 'error')}")
g3.disconnect()

print("\n(h) a refused command latches the WHOLE base until an explicit rearm")

bus_h = PolicyBus()
h = policy_adapter(bus_h, "a healthy bus (the outage starts after arming)")
check(h.write_enable(True) is True, "armed on the healthy bus")
check(h.fault_reason is None, "no fault latched yet")

# the front pigtail comes loose: its RPM commands are refused from now on
bus_h.refuse.add((FRONT_ID, L_CMD_RPM))
bus_h.writes.clear()
mark = len(LOG.lines)
check(h.write_velocities([VX, VY, WZ]) is False,
      "write_velocities() reports False when one controller refuses")
print(f"      after the refusal: control words {bus_h.control_words()}, "
      f"RPM {bus_h.rpm_cmds()}")
check(h.fault_reason is not None,
      f"the base is LATCHED in fault: {h.fault_reason!r}")
check(h.read_enabled() is False,
      "and it no longer claims to be enabled (both drives were told to disarm)")
# containment: the axle that DOES answer is the one physically driving, and
# it must not be left executing the twist the other one refused.
check(bus_h.rpm_cmds(BACK_ID)[-1] == (BACK_ID, (0, 0)),
      f"the healthy BACK axle was zeroed {bus_h.rpm_cmds(BACK_ID)}")
check(bus_h.regs.get((BACK_ID, CONTROL_REG)) == DISABLE,
      "and disabled (0x200E = DISABLE)")
check((FRONT_ID, DISABLE) in bus_h.control_words(),
      f"the silent front was told to disable too - best effort, it is out of "
      f"software's reach {bus_h.control_words()}")
check(bus_h.armed() == [], f"no drive reads as armed {bus_h.armed()}")

# the point of the latch: the NEXT non-zero command never reaches the bus
bus_h.writes.clear()
bus_h.reads.clear()
check(h.write_velocities([VX, VY, WZ]) is False,
      "the next non-zero command is REFUSED")
check(bus_h.writes == [] and bus_h.reads == [],
      f"and it never touched the bus: {bus_h.writes} / {bus_h.reads}")
check(any("refusing wheel RPM" in m for m in LOG.since(mark, "warning")),
      "the refusal is logged")

# a stop is never refused: it is the one command a broken base still wants out
bus_h.writes.clear()
h.write_stop()
check(bus_h.rpm_cmds(BACK_ID) == [(BACK_ID, (0, 0))],
      f"write_stop() still reaches the bus while latched {bus_h.rpm_cmds()}")
bus_h.writes.clear()
h.write_velocities([0.0, 0.0, 0.0])
check(bus_h.rpm_cmds(BACK_ID) == [(BACK_ID, (0, 0))],
      f"a ZERO twist is not refused either, it goes out {bus_h.rpm_cmds()} "
      f"(it still returns False here: the front is genuinely gone)")

# the throttle: a 100 Hz coordinator driving into the refusal must not flood
mark = len(LOG.lines)
for _ in range(50):
    h.write_velocities([VX, VY, WZ])
check(len(LOG.since(mark, "warning")) == 0,
      f"50 more refusals inside {adapter_mod.FAULT_WARN_PERIOD_S}s add no "
      f"warning ({len(LOG.since(mark, 'warning'))})")

# a rearm attempt on a bus that is still broken must NOT clear the latch
check(h.write_enable(True) is False,
      "write_enable(True) fails while the front is still refusing frames")
check(h.fault_reason is not None, "so the latch survives the failed rearm")
bus_h.writes.clear()
check(h.write_velocities([VX, VY, WZ]) is False,
      "and commands are still refused")
check(bus_h.writes == [], "still no bus I/O")

# only a COMPLETE transaction rearms
bus_h.refuse.clear()
mark = len(LOG.lines)
check(h.write_enable(True) is True, "the bus heals: write_enable(True) is True")
check(h.fault_reason is None, "the latch is cleared - THAT is the rearm")
check(h.read_enabled() is True, "and the adapter is enabled again")
check(any("rearmed after" in m for m in LOG.since(mark, "info")),
      f"the rearm is logged: {[m for m in LOG.since(mark, 'info')]}")
bus_h.writes.clear()
check(h.write_velocities([VX, VY, WZ]) is True,
      "commands flow again")
exp_back = (round(-exp[2]), round(exp[3]))
check(bus_h.rpm_cmds(BACK_ID) == [(BACK_ID, exp_back)],
      f"with the geometry's RPM back on the bus {bus_h.rpm_cmds(BACK_ID)} "
      f"= {exp_back}")
h.disconnect()

print("    -- the serial layer RAISES instead of answering --")


class RaisingWriteBus(PolicyBus):
    """The port is yanked: write_registers raises instead of answering.

    Controller._write_registers swallows this today, so the adapter's own
    except is belt and braces - and the belt has to latch too, or a torn-out
    USB dongle would leave the base commandable.
    """

    def __init__(self):
        super().__init__()
        self.explode = False

    def write_registers(self, addr, values, unit=0):
        if self.explode and addr == L_CMD_RPM:
            raise OSError("[Errno 5] Input/output error: /dev/ttyUSB0")
        return super().write_registers(addr, values, unit=unit)


bus_hr = RaisingWriteBus()
hr = policy_adapter(bus_hr, "a bus that will raise")
check(hr.write_enable(True) is True, "armed on the healthy bus")
bus_hr.explode = True
check(hr.write_velocities([VX, VY, WZ]) is False,
      "a raising serial layer is absorbed: False, no exception")
check(hr.fault_reason is not None,
      f"and it latches the base too: {hr.fault_reason!r}")
bus_hr.explode = False
bus_hr.writes.clear()
check(hr.write_velocities([VX, VY, WZ]) is False,
      "the next non-zero command is refused even though the bus is fine again")
check(bus_hr.writes == [],
      "a healed bus does not un-latch by itself - only write_enable(True) does")
# on a bus that answers again, "a stop always passes" is visible in the
# return value too, latch or no latch
bus_hr.writes.clear()
check(hr.write_velocities([0.0, 0.0, 0.0]) is True,
      "a ZERO twist passes through the latch and succeeds")
check(hr.write_stop() is True, "and so does write_stop()")
check(hr.fault_reason is not None, "neither of them clears the latch")
check(hr.write_enable(False) is True,
      "write_enable(False) is never blocked by the latch either")
check(hr.fault_reason is not None, "and disarming does not clear it")
hr.disconnect()

print("\n(i) a bus that takes real time to answer: one poll per tick")


class SlowBus(MockModbusClient):
    """Reads cost wall-clock time; `silent_unit` never answers.

    `armed` keeps connect() instantaneous - the latency is the outage,
    not the bring-up. A silent drive costs SERIAL_TIMEOUT_S: pymodbus 2.5
    waits the whole response timeout before handing back its
    ModbusIOException, so the poll is 33x older than FEEDBACK_MAX_AGE_S
    when it returns.
    """

    def __init__(self, latency_s, silent_unit=None):
        super().__init__()
        self.latency_s = latency_s
        self.silent_unit = silent_unit
        self.armed = False

    def read_holding_registers(self, addr, count, unit=0):
        if not self.armed:
            return super().read_holding_registers(addr, count, unit=unit)
        time.sleep(self.latency_s)
        if unit == self.silent_unit:
            self.reads.append((unit, addr, count))
            return _ErrorResult()
        return super().read_holding_registers(addr, count, unit=unit)


# i1: the front pigtail comes loose - its reads time out
bus_l = SlowBus(latency_s=SERIAL_TIMEOUT_S, silent_unit=FRONT_ID)
lat = VectorBaseAdapter(dof=3, client=bus_l, geometry=G)
check(lat.connect() is True, "connect() while both drives still answer")
bus_l.armed = True
time.sleep(FEEDBACK_MAX_AGE_S * 2)
bus_l.reads.clear()
t0 = time.monotonic()
lat.read_velocities()      # what ConnectedTwistBase.read_state() does:
lat.read_odometry()        # these two, back to back, once per tick
tick_s = time.monotonic() - t0
polls = [r for r in bus_l.reads if r[1] == L_FB_RPM]
print(f"      timeout {SERIAL_TIMEOUT_S:.3f} s vs cache window "
      f"{FEEDBACK_MAX_AGE_S:.3f} s -> one tick: {len(polls)} poll(s), "
      f"{tick_s:.3f} s wall")
check(len(polls) == 1, f"exactly one bus poll for the whole tick {polls}")
check(lat.read_failure_count == 1,
      f"one failed poll counted, not two ({lat.read_failure_count})")
check(tick_s < SERIAL_TIMEOUT_S * 1.5,
      f"the tick pays ONE timeout: {tick_s:.3f} s < "
      f"{SERIAL_TIMEOUT_S * 1.5:.3f} s (two would be "
      f"{SERIAL_TIMEOUT_S * 2:.3f} s)")

# the cache still EXPIRES: a poll timestamped late must not freeze the rate
time.sleep(FEEDBACK_MAX_AGE_S * 2)
lat.read_velocities()
check(lat.read_failure_count == 2,
      f"the next tick does poll again once the window is over "
      f"({lat.read_failure_count})")
lat.disconnect()

# i2: nobody is silent, the bus is just slow (a long RS485 run, 115200 bd).
# Both drives answer, and one tick must still cost one read per controller.
bus_s = SlowBus(latency_s=0.05)
slow = VectorBaseAdapter(dof=3, client=bus_s, geometry=G)
check(slow.connect() is True, "connect() on the slow-but-healthy bus")
check(slow.write_velocities([0.30, 0.0, 0.0]), "command 0.30 m/s forward")
bus_s.armed = True
bus_s.reads.clear()
t0 = time.monotonic()
twist = slow.read_velocities()
slow.read_odometry()
tick_s = time.monotonic() - t0
print(f"      slow healthy bus ({bus_s.latency_s:.3f} s per read): one tick "
      f"= {len(bus_s.reads)} reads, {tick_s:.3f} s wall, "
      f"vx={twist[0]:.3f} m/s")
check(len(bus_s.reads) == 2,
      f"one read per controller, not two {bus_s.reads}")
check(abs(twist[0] - 0.30) < 0.02,
      f"and the feedback is the commanded 0.30 m/s ({twist[0]:.3f})")
check(slow.read_failure_count == 0, "no failure on a healthy bus")
slow.disconnect()

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

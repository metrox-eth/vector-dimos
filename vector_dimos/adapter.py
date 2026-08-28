"""VectorBaseAdapter - dimOS TwistBaseAdapter for the VECTOR mecanum platform.

Pure python on purpose: dimOS's TwistBaseAdapter is a structural Protocol
(runtime_checkable), so this file imports nothing from dimos at import time
and stays cold-testable. dimOS is used opportunistically for logging only.
The dimOS glue (adapter registry, blueprints) lives in blueprints.py.

Virtual joint order (holonomic): [vx, vy, wz]. Odometry [x, y, theta] is
integrated from wheel feedback - LOW CONFIDENCE by design doctrine (mecanum
rollers slip; use it for sanity, never as the localization reference: the
D455F point cloud + RPLIDAR do that job).

Two field-survival properties matter here, both dictated by the runtime:

  * reads never raise. dimos/control/hardware_interface.py's
    ConnectedTwistBase.read_state() calls read_velocities() then
    read_odometry() on every control tick. TickLoop._read_all_hardware
    does wrap that call in try/except, so an exception does not kill the
    coordinator - it drops the base joints from that tick's snapshot and
    logs "Failed to read base" at the tick rate (100 Hz by default). A
    failed read therefore serves the last known feedback instead (zeros
    before the first success) and is logged rate-limited: the snapshot
    stays complete and the log stays readable. The one caller with no
    guard is the get_joint_positions RPC (dimos/control/coordinator.py),
    where raising would fail that single reply. No automatic e-stop is
    wired to read failures.

  * at most one bus poll per FEEDBACK_MAX_AGE_S (15 ms). Those two calls
    share a short feedback cache, so a control tick costs one read per
    controller, not two. The window is longer than the 10 ms tick period,
    so an idle tick can also reuse the previous tick's feedback; a tick
    that writes a command drops the cache and does poll. The window is
    counted from the END of the poll, so a tick against a drive that
    times out pays SERIAL_TIMEOUT_S once, not twice.

connect() is deliberately strict: it probes both drives read-only and
returns False if either is silent, which makes dimOS refuse to start with
"Failed to connect to vector adapter" rather than run blind.

Arming is TRANSACTIONAL and it is the only thing that clears a fault:
write_enable(True) prepares both controllers, reads both fault registers,
and only then writes the two ENABLE words - front, then back, rolling the
front back if the back refuses. Either both axles end up armed or none
does. Before 2026-08-28 the loop prepared-and-armed one controller at a
time, so a refusal on the front still armed the back while _enabled stayed
False: 27 kg under torque that the software believed disarmed.

What a partial bus failure does, and the one thing it deliberately does NOT:

  * it does not dead-reckon. A failed poll freezes the integrated pose
    instead of integrating the last known twist, which would advance
    /odom by ~0.3 m per second of outage at 0.3 m/s with no measurement
    behind it. read_velocities() still serves the last known twist.

  * it DOES stop the healthy axle now. The two writes go out back to back
    and only one of them can fail, so by the time we know, the other axle
    has already taken the new command: a one-sided RS485 outage used to
    turn a straight line into a pirouette, with the silent axle holding
    its last command. A refused write therefore latches the whole base in
    fault, zeroes and disables BOTH controllers best effort, and every
    later non-zero command is refused without touching the bus until an
    explicit rearm. This was a chassis decision, not a default; metrox
    made it on 2026-08-28 18:37 after the external audit. Nothing in
    software can reach a drive that does not answer - that one is stopped
    by its own 0x2000 watchdog, and only because we stop talking to it.
    The real guarantee is a hardware enable line common to both drives,
    which this chassis does not have yet.

This is also the last thing the twist passes through before it becomes wheel
RPM, which is why the sonar proximity brake lives here (see brake_forward and
the SONAR_* constants): the ESP32 sonar is a brake, not a mapper. It clamps
the FORWARD component only - reverse and rotation are never touched, because
the sonar looks forward and backing away from what it sees must always be
allowed. No reading, or a reading older than SONAR_MAX_AGE_S, means no brake
at all: the sonar is an aid, and losing it must not immobilise the rover.

SERIAL_TIMEOUT_S is what a silent drive costs: pymodbus 2.5 waits the whole
response timeout before handing back its ModbusIOException, so one silent
controller adds that much to every read and every write of a control tick.
0.5 s is the conservative bring-up value, not a measured one - time a real
answering drive on the bus and pass a smaller timeout_s.

Set VECTOR_MOCK_BUS=1 to drive the whole real dimOS pipeline against
vector_dimos.mock.MockModbusClient on a machine with no motors wired.
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time

from .kinematics import (MecanumGeometry, forward, inverse, rads_to_rpm,
                         rpm_to_rads)
from .zlac8015d import Controller

FRONT_ID = 2   # L port = FL, R port = FR   (proven v1 topology)
BACK_ID = 1    # L port = BL, R port = BR
DEFAULT_PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG01OSGH-if00-port0"
BAUDRATE = 115200

MOCK_BUS_ENV = "VECTOR_MOCK_BUS"
SERIAL_TIMEOUT_S = 0.5       # pymodbus response timeout; a silent drive costs it
# The two reads of one tick must share one poll, so this has to exceed how long
# a poll takes on the real bus (unmeasured; the timestamp is taken before it).
# It also exceeds the 10 ms tick period, so an IDLE tick can reuse the previous
# tick's feedback - fewer polls, never more. As soon as a command is written the
# cache is dropped, so a tick that drives does poll.
FEEDBACK_MAX_AGE_S = 0.015
READ_WARN_PERIOD_S = 5.0     # rate limit for "read failed" warnings
# Same idea on the other side: a latched base refuses a command per control
# tick (100 Hz), and the log has to stay readable while someone drives into
# the refusal. The first refusal is always logged.
FAULT_WARN_PERIOD_S = 5.0
# Real bus: the two sides of the proof in `dimos log`. A changed RPM command is
# logged at most once per CMD_LOG_PERIOD_S (a zero command always); the
# encoder feedback is logged once per FEEDBACK_LOG_PERIOD_S while any wheel
# turns, plus the line where they all read zero again.
CMD_LOG_PERIOD_S = 0.5
FEEDBACK_LOG_PERIOD_S = 0.5
FEEDBACK_MOVING_RPM = 0.5    # |feedback| below this reads as "not turning"
# Drive-side watchdog (ZLAC8015D 0x2000, written at enable time). Measured on
# blocks 2026-08-22: with 1000 the wheels were at rest < 1.9 s after every
# dimOS process was SIGKILLed mid-motion (22 RPM), no fault raised, drives
# still enabled, next enable normal. The drives ship with 0 (= off): a dead
# runtime then leaves the wheels turning until the power is cut. The 100 Hz
# tick loop talks to the bus far more often than this, so it never trips.
COMM_OFFLINE_MS = 1000

# Sonar proximity brake (vector_dimos.esp_sensors publishes `sonar_range` in
# metres; blueprints.VectorControlCoordinator forwards it to note_sonar_range).
SONAR_STOP_M = 0.30          # under this, no forward motion at all
SONAR_SLOW_M = 0.55          # under this, forward motion creeps (the sonar's trust cap)
SONAR_CREEP_MPS = 0.05       # what "slow" means: enough to close on a target, not to ram it
SONAR_RELEASE_MARGIN_M = 0.05  # a level releases 5 cm further out than it engaged (no chatter)
SONAR_MAX_AGE_S = 1.5        # older than this = no data = no brake

_TRUTHY = {"1", "true", "yes", "on"}
_LOGGER = None


class SonarBrake:
    """The brake's latch: which clamp is currently engaged.

    Three levels (FREE / CREEP / STOP). A level engages at its threshold and
    releases SONAR_RELEASE_MARGIN_M further out, so a reading sitting on a
    threshold does not toggle the clamp on every 5 Hz sample.
    """

    FREE, CREEP, STOP = 0, 1, 2

    def __init__(self) -> None:
        self.level = self.FREE

    def update(self, sonar_m: float | None, age_s: float) -> int:
        """New level for this reading. Stale or missing data releases."""
        if sonar_m is None or age_s > SONAR_MAX_AGE_S:
            self.level = self.FREE
            return self.level
        margin = SONAR_RELEASE_MARGIN_M
        stop_at = SONAR_STOP_M + (margin if self.level == self.STOP else 0.0)
        slow_at = SONAR_SLOW_M + (margin if self.level >= self.CREEP else 0.0)
        if sonar_m < stop_at:
            self.level = self.STOP
        elif sonar_m < slow_at:
            self.level = self.CREEP
        else:
            self.level = self.FREE
        return self.level


_BRAKE = SonarBrake()


def brake_forward(vx: float, sonar_m: float | None, age_s: float,
                  brake: SonarBrake | None = None) -> float:
    """Forward speed the sonar allows, in m/s. Pure apart from the latch.

    Args:
        vx: commanded body-forward speed (m/s). Negative = reverse.
        sonar_m: last front distance in metres, or None if nothing was ever
            received. esp_sensors publishes 9.9 as its "clear" heartbeat.
        age_s: age of that reading in seconds.
        brake: the latch to use (hysteresis state). Defaults to the module
            one, which is what the cold bench exercises.

    No reading, or a reading older than SONAR_MAX_AGE_S, returns vx unchanged.
    That is a deliberate choice: the sonar is an aid, not a dependency, and a
    dead ESP32 must not be able to immobilise the rover - the contact switches
    and the lidar are still there. Reverse and rotation are never clamped: the
    sonar looks forward, and backing out of a corner must always work.
    """
    level = (_BRAKE if brake is None else brake).update(sonar_m, age_s)
    if vx <= 0.0 or level == SonarBrake.FREE:
        return vx
    if level == SonarBrake.STOP:
        return 0.0
    return min(vx, SONAR_CREEP_MPS)


def _log():
    """dimOS logger when dimos is importable, stdlib otherwise.

    Resolved on first use so that importing this module never imports dimos.
    """
    global _LOGGER
    if _LOGGER is None:
        try:
            from dimos.utils.logging_config import setup_logger
            _LOGGER = setup_logger()
        except Exception:
            lg = logging.getLogger("vector_dimos.adapter")
            if not lg.handlers and not logging.getLogger().handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(
                    logging.Formatter("%(levelname)s %(name)s: %(message)s"))
                lg.addHandler(handler)
            lg.setLevel(logging.INFO)
            _LOGGER = lg
    return _LOGGER


def _env_truthy(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in _TRUTHY


class VectorBaseAdapter:
    """TwistBaseAdapter implementation for VECTOR (2x ZLAC8015D, RS485).

    Args:
        dof: must be 3 (holonomic).
        address: serial port of the RS485 bus (Waveshare USB dongle).
        client: injectable modbus client (tests use MockModbusClient). An
            injected client is assumed already open: connect() probes it but
            never calls its connect(). It IS closed like an owned one - a
            failed probe and disconnect() both call close() on it - only the
            reference is kept (an owned client is dropped instead).
        timeout_s: pymodbus response timeout on the bus we own. It is the
            per-transaction cost of a silent drive - see the module
            docstring before lowering it.
        accel_ms: acceleration = deceleration ramp written to both drives
            at enable time. 400 ms is the value the first robot code
            converged on in the field (500 -> 1000 -> 400 over its commit
            history; no reason recorded).
        comm_offline_ms: the drives' own watchdog (0x2000), written at
            enable time; 0 turns it off. See COMM_OFFLINE_MS for what was
            measured. It is the only thing that stops the wheels when the
            runtime dies without running disconnect().
    """

    def __init__(self, dof: int = 3, address: str | None = None,
                 hardware_id: str = "base", client=None,
                 geometry: MecanumGeometry | None = None,
                 accel_ms: int = 400, timeout_s: float = SERIAL_TIMEOUT_S,
                 comm_offline_ms: int = COMM_OFFLINE_MS,
                 **_: object) -> None:
        if dof != 3:
            raise ValueError(f"VECTOR is holonomic: dof must be 3, got {dof}")
        self._port = address or DEFAULT_PORT
        self._timeout_s = timeout_s
        self._client = client
        self._owns_client = client is None
        self._geometry = geometry or MecanumGeometry()
        self._accel_ms = accel_ms
        self._comm_offline_ms = comm_offline_ms
        self._connected = False
        self._enabled = False
        # runtime fault latch: (reason, monotonic timestamp) or None. Set by a
        # refused write, cleared ONLY by a complete write_enable(True).
        self._faulted: tuple[str, float] | None = None
        self._refused_cmds = 0
        self._last_fault_warn_t = 0.0
        self._mock = False
        self._lock = threading.RLock()
        self._front: Controller | None = None
        self._back: Controller | None = None
        # wheel feedback cache: (FL, FR, BL, BR) rad/s + last poll timestamp
        self._feedback = (0.0, 0.0, 0.0, 0.0)
        self._feedback_t: float | None = None
        self._read_failures = 0
        self._last_read_warn_t = 0.0
        self._last_logged_cmd: tuple[int, int, int, int] | None = None
        self._last_cmd_log_t = 0.0
        self._last_fb_log_t = 0.0
        self._fb_was_moving = False
        # odometry integration state
        self._pose = [0.0, 0.0, 0.0]
        self._last_t: float | None = None
        # sonar proximity brake: last reading (m), when it arrived, and the latch
        self._sonar_m: float | None = None
        self._sonar_t: float | None = None
        self._brake = SonarBrake()

    # ── introspection ──────────────────────────────────────────────────
    @property
    def client(self):
        """The MODBUS client actually in use (mock bus included)."""
        return self._client

    @property
    def mock_bus(self) -> bool:
        """True when running on the fake bus (VECTOR_MOCK_BUS)."""
        return self._mock

    @property
    def read_failure_count(self) -> int:
        """Consecutive failed wheel-feedback reads (0 = healthy)."""
        return self._read_failures

    @property
    def fault_reason(self) -> str | None:
        """Why the base is latched in fault, or None when it is not.

        Read-only: nothing outside can clear the latch, and nothing should
        want to - a successful write_enable(True) is the only rearm.
        """
        return None if self._faulted is None else self._faulted[0]

    # ── sonar proximity brake ──────────────────────────────────────────
    def note_sonar_range(self, distance_m: float) -> None:
        """Feed the front sonar distance (metres) to the forward brake.

        Called from the coordinator's `sonar_range` handler (blueprints.py),
        which runs in this process. Anything at or past SONAR_SLOW_M - the
        9.9 m "clear" heartbeat included - simply releases the brake.
        """
        with self._lock:
            self._sonar_m = float(distance_m)
            self._sonar_t = time.monotonic()

    def _brake_vx(self, vx: float) -> float:
        """The sonar INFORMS, it no longer brakes (design decision).

        The brake was supposed to die with the guard rip on 2026-08-26 and did
        not; it then spent the whole evening clamping every forward command to
        zero on a sonar stuck at 0.08 m. The safety brake was explicitly
        removed: the sonar reading is an indication, not a safety device. The
        contact switches are the safety. The latch still tracks the level so the log
        keeps saying what the sonar WOULD have done - information, no action.
        """
        age = (SONAR_MAX_AGE_S + 1.0 if self._sonar_t is None
               else time.monotonic() - self._sonar_t)
        before = self._brake.level
        brake_forward(vx, self._sonar_m, age, self._brake)   # latch only: keeps the log honest
        if self._brake.level != before:
            if self._brake.level == SonarBrake.FREE:
                _log().info("sonar info: front clear again")
            else:
                _log().info(f"sonar info: something {self._sonar_m:.2f} m ahead (NO braking - switches are the safety)")
        return vx

    # ── connection ─────────────────────────────────────────────────────
    def connect(self) -> bool:
        with self._lock:
            if self._connected:
                return True
            if self._client is None and not self._open_client():
                return False
            self._front = Controller(FRONT_ID, client=self._client)
            self._back = Controller(BACK_ID, client=self._client)
            if not self._probe():
                self._front = self._back = None
                self._close_client()
                return False
            self._read_failures = 0
            self._last_logged_cmd = None
            self._connected = True
            return True

    def _open_client(self) -> bool:
        """Build the bus client we own. False on failure (logged)."""
        if _env_truthy(MOCK_BUS_ENV):
            from .mock import MockModbusClient
            self._client = MockModbusClient()
            self._mock = True
            _log().info(f"VECTOR base: MOCK BUS ({MOCK_BUS_ENV} set) - "
                        "no motors will move")
            return True
        try:
            from pymodbus.client.sync import ModbusSerialClient
            self._client = ModbusSerialClient(
                method="rtu", port=self._port, baudrate=BAUDRATE,
                timeout=self._timeout_s)
            opened = self._client.connect()
        except Exception as exc:
            _log().error(f"VECTOR base: cannot open RS485 bus on "
                         f"{self._port} @{BAUDRATE}: {exc!r}")
            self._client = None
            return False
        if not opened:
            _log().error(f"VECTOR base: serial client refused to open "
                         f"{self._port} @{BAUDRATE} (port busy, missing, or "
                         "no permission)")
            self._client = None
            return False
        return True

    def _probe(self) -> bool:
        """Read-only check that both drives answer. False if any is silent."""
        answers = {}
        for ctrl, side in ((self._front, "front"), (self._back, "back")):
            rpm = ctrl.get_rpm()
            if rpm is None:
                _log().error(f"ZLAC8015D id {ctrl.unit} ({side}) did not "
                             f"answer on {self._port} @{BAUDRATE}")
                return False
            answers[side] = rpm
        fl_raw, fr_raw = answers["front"]
        bl_raw, br_raw = answers["back"]
        self._feedback = (rpm_to_rads(-fl_raw), rpm_to_rads(fr_raw),
                          rpm_to_rads(-bl_raw), rpm_to_rads(br_raw))
        self._feedback_t = time.monotonic()
        return True

    def _close_client(self) -> None:
        try:
            if self._client is not None:
                self._client.close()
        except Exception:
            pass
        if self._owns_client:
            self._client = None
            self._mock = False

    def disconnect(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self.write_stop()
                    self.write_enable(False)
                except Exception:
                    pass
                self._close_client()
            self._front = self._back = None
            self._connected = False

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return 3

    # ── wheel feedback ─────────────────────────────────────────────────
    def _wheel_rads(self) -> tuple[float, float, float, float]:
        """Feedback (FL, FR, BL, BR) in rad/s, left-port inversion undone.

        Cached: read_state() asks for velocities then odometry back to back,
        and the bus is polled at most once per FEEDBACK_MAX_AGE_S. The
        timestamp is taken when the poll RETURNS, not when it starts: a
        poll that spent SERIAL_TIMEOUT_S (0.5 s) waiting on a silent drive
        is 33x older than the 15 ms window by the time it comes back, so
        stamping it on entry would make the second read of the same tick
        miss the cache and pay a second full timeout. A command write
        drops the cache, since the feedback it holds predates the command.
        A failed read serves the last known values and never raises.

        The second controller is not polled when the first one is silent:
        a partial answer is discarded anyway, and on the real bus every
        unanswered read costs the full serial timeout.
        """
        now = time.monotonic()
        if (self._feedback_t is not None
                and now - self._feedback_t < FEEDBACK_MAX_AGE_S):
            return self._feedback
        try:
            front = self._front.get_rpm() if self._front is not None else None
            if front is None:
                self._note_read_failure("front")
                return self._feedback
            back = self._back.get_rpm() if self._back is not None else None
            if back is None:
                self._note_read_failure("back")
                return self._feedback
            self._note_read_success()
            fl_raw, fr_raw = front
            bl_raw, br_raw = back
            self._feedback = (rpm_to_rads(-fl_raw), rpm_to_rads(fr_raw),
                              rpm_to_rads(-bl_raw), rpm_to_rads(br_raw))
            self._log_feedback(-fl_raw, fr_raw, -bl_raw, br_raw)
            return self._feedback
        finally:
            # every exit path, including a raising client: the window opens
            # when the bus is done with us
            self._feedback_t = time.monotonic()

    def _log_feedback(self, fl: float, fr: float, bl: float, br: float) -> None:
        """Encoder side of the proof, real bus only: wheel RPM as measured
        by the drives (FL, FR, BL, BR, left-port inversion undone), one line
        per FEEDBACK_LOG_PERIOD_S while any wheel turns and one more when
        they all read zero again. Read it next to the command line."""
        if self._mock:
            return
        moving = any(abs(v) >= FEEDBACK_MOVING_RPM for v in (fl, fr, bl, br))
        now = time.monotonic()
        if moving and now - self._last_fb_log_t < FEEDBACK_LOG_PERIOD_S:
            return
        if not moving and not self._fb_was_moving:
            return
        self._fb_was_moving = moving
        self._last_fb_log_t = now
        x, y, th = self._pose
        _log().info(f"VECTOR base feedback: wheel RPM FL={fl:+.1f} FR={fr:+.1f} "
                    f"BL={bl:+.1f} BR={br:+.1f} | odom x={x:+.3f} y={y:+.3f} "
                    f"th={math.degrees(th):+.1f}deg")

    def _note_read_failure(self, silent: str) -> None:
        """`silent` is the first controller that did not answer this poll."""
        self._read_failures += 1
        now = time.monotonic()
        if (self._read_failures == 1
                or now - self._last_read_warn_t >= READ_WARN_PERIOD_S):
            self._last_read_warn_t = now
            _log().warning(
                f"VECTOR base: no wheel feedback from {silent} "
                f"on {self._port} ({self._read_failures} consecutive) - "
                "serving last known velocities")

    def _note_read_success(self) -> None:
        if self._read_failures:
            _log().info(f"VECTOR base: wheel feedback recovered after "
                        f"{self._read_failures} failures")
            self._read_failures = 0

    def read_velocities(self) -> list[float]:
        with self._lock:
            vx, vy, wz = forward(*self._wheel_rads(), self._geometry)
        return [vx, vy, wz]

    def read_odometry(self) -> list[float] | None:
        """Integrated wheel odometry [x, y, theta] - low confidence.

        Frozen while the bus is failing. read_velocities() serves the last
        known twist so the control loop keeps a plausible velocity, but
        integrating that twist would move the pose with no measurement
        behind it (0.3 m per second of outage at 0.3 m/s).
        """
        with self._lock:
            vx, vy, wz = forward(*self._wheel_rads(), self._geometry)
            now = time.monotonic()
            if self._last_t is not None and self._read_failures == 0:
                dt = now - self._last_t
                x, y, th = self._pose
                x += (vx * math.cos(th) - vy * math.sin(th)) * dt
                y += (vx * math.sin(th) + vy * math.cos(th)) * dt
                th += wz * dt
                self._pose = [x, y, th]
            self._last_t = now
            return list(self._pose)

    # ── commands ───────────────────────────────────────────────────────
    def write_velocities(self, velocities: list[float]) -> bool:
        """Command a body twist. False if either controller refused.

        A refused write is a fault of the WHOLE base, not of one axle. The
        two set_rpm calls go out back to back, so when the second one fails
        the first has already been applied: leaving it there is how a
        one-sided RS485 outage becomes a pirouette. So a refusal latches the
        base, tries zero + disable on both controllers, and every later
        non-zero command is refused right here, without touching the bus,
        until a complete write_enable(True) rearms it. A zero command is
        never refused: stopping must always be allowed to reach the bus.

        The forward component passes the sonar brake first: this is the last
        place a twist can still be slowed before it is wheel RPM, so nothing
        upstream can drive into the sofa by ignoring it.
        """
        with self._lock:
            if self._front is None or self._back is None:
                return False
            try:
                vx, vy, wz = velocities
                vx = self._brake_vx(vx)
                w_fl, w_fr, w_bl, w_br = inverse(vx, vy, wz, self._geometry)
                # left ports inverted (v1 convention: negative = forward)
                raw = (round(-rads_to_rpm(w_fl)), round(rads_to_rpm(w_fr)),
                       round(-rads_to_rpm(w_bl)), round(rads_to_rpm(w_br)))
            except Exception:
                # a malformed twist is a caller bug, not a bus fault: refuse
                # it, but do not latch the base on it.
                return False
            # The gate is the RPM that would reach the drives, not the twist:
            # a twist too small to round to a single RPM IS a stop.
            if self._faulted is not None and raw != (0, 0, 0, 0):
                self._note_refused(raw)
                return False
            try:
                ok_front = self._front.set_rpm(raw[0], raw[1])
                ok_back = self._back.set_rpm(raw[2], raw[3])
            except Exception as exc:
                # Controller._write_registers swallows everything today, so
                # this is belt and braces - and the belt has to latch too.
                self._latch_fault(f"the serial layer raised on an RPM "
                                  f"command (0x2088): {exc!r}")
                return False
            if not (ok_front and ok_back):
                silent = "front" if not ok_front else "back"
                both = "" if ok_front or ok_back else " (neither answered)"
                self._latch_fault(f"the {silent} controller refused an RPM "
                                  f"command (0x2088){both}")
                return False
            self._feedback_t = None   # cached feedback predates it
            self._log_cmd((vx, vy, wz), raw)
            return True

    # ── runtime fault latch ────────────────────────────────────────────
    def _latch_fault(self, reason: str) -> None:
        """Latch the base in fault and try to leave both axles at rest.

        Called with the lock held. Containment is best effort by nature:
        the controller that just refused a frame is probably the one that
        will refuse the zero and the disable too. It is still attempted on
        BOTH - the point is the axle that DOES answer, which is the one
        physically driving. _enabled goes False because we asked both
        drives to disarm; the drive we cannot reach stops on its own
        0x2000 watchdog once we stop commanding it (COMM_OFFLINE_MS).

        Containment runs ONCE, on the transition. The stops that are still
        allowed through afterwards can fail against the same dead drive,
        and re-running the whole zero + disable on each of them would pour
        disable frames onto the bus at the tick rate for nothing: the base
        is already latched and already told both drives to stand down.
        """
        if self._faulted is not None:
            self._enabled = False
            return
        self._faulted = (reason, time.monotonic())
        self._enabled = False
        _log().error(f"VECTOR base FAULTED: {reason} - zero + disable on "
                     "both controllers, non-zero commands refused until "
                     "a successful write_enable(True)")
        for c, side in ((self._front, "front"), (self._back, "back")):
            if c is not None:
                self._stand_down(c, side)

    def _stand_down(self, controller, side: str) -> None:
        """Zero the target, then drop the ENABLE bit. Each step guarded.

        The order matters: a drive that takes the zero and then refuses the
        disable is stopped anyway, while a drive disarmed with a live RPM
        target would run it the moment anything re-arms it.
        """
        steps = (("zero target (0x2088)", lambda: controller.set_rpm(0, 0)),
                 ("disable (0x200E)", controller.disable))
        for what, call in steps:
            try:
                done = call()
            except Exception:
                done = False
            if not done:
                _log().error(f"ZLAC8015D id {controller.unit} ({side}) did not "
                             f"take {what} - it is out of software's reach")

    def _note_refused(self, raw) -> None:
        """A non-zero command arrived on a latched base. No bus, one log."""
        self._refused_cmds += 1
        now = time.monotonic()
        if (self._refused_cmds == 1
                or now - self._last_fault_warn_t >= FAULT_WARN_PERIOD_S):
            self._last_fault_warn_t = now
            reason, since = self._faulted
            _log().warning(
                f"VECTOR base FAULTED {now - since:.1f}s ago ({reason}): "
                f"refusing wheel RPM {raw} without touching the bus "
                f"({self._refused_cmds} refused so far) - "
                "write_enable(True) is the rearm")

    def write_stop(self) -> bool:
        try:
            with self._lock:
                if self._front is None or self._back is None:
                    return False
                ok_front = self._front.set_rpm(0, 0)
                ok_back = self._back.set_rpm(0, 0)
                if ok_front and ok_back:
                    self._feedback_t = None   # cached feedback predates it
                    self._log_cmd((0.0, 0.0, 0.0), (0, 0, 0, 0))
                return ok_front and ok_back
        except Exception:
            return False

    def _log_cmd(self, twist, raw) -> None:
        """Command side of the proof: one info line per change of the RPM
        command. Mock bus: every change (this is how the twist -> per-wheel
        RPM chain is proven through the real dimOS pipeline with no motors,
        pinned by the benches). Real bus: at most one line per
        CMD_LOG_PERIOD_S, except a zero command, which is always logged -
        the stop transitions are the lines worth having when something
        went wrong. Read it back in `dimos log`, next to the feedback line.
        """
        if raw == self._last_logged_cmd:
            return
        now = time.monotonic()
        zero = raw == (0, 0, 0, 0)
        if (not self._mock and not zero
                and now - self._last_cmd_log_t < CMD_LOG_PERIOD_S):
            return
        self._last_logged_cmd = raw
        self._last_cmd_log_t = now
        head = "VECTOR base MOCK:" if self._mock else "VECTOR base:"
        vx, vy, wz = twist
        _log().info(
            f"{head} twist vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} "
            f"-> wheel RPM FL={-raw[0]:+d} FR={raw[1]:+d} "
            f"BL={-raw[2]:+d} BR={raw[3]:+d} "
            f"(bus raw front L/R={raw[0]:+d}/{raw[1]:+d} "
            f"back L/R={raw[2]:+d}/{raw[3]:+d})")

    def write_enable(self, enable: bool) -> bool:
        """Arm (or disarm) both drives. False if any step was refused.

        Arming is a TRANSACTION over the pair: either both axles come out
        armed, or none does. It used to be a loop that prepared and armed
        one controller before touching the other, which had two ways to
        leave a single axle under torque behind a False return - a front
        that refused let the loop `continue` and arm the back, and a back
        that refused left the front armed with nothing to undo it. dimOS
        ignores this return value (coordinator.py), so the state we leave
        the hardware in is the only thing that protects: 27 kg on four
        mecanum wheels must never be armed by an activation that failed.

        Three phases, in this order:

          1. prepare BOTH, arm NOBODY: velocity mode, ramps, comm-offline
             watchdog, zero target. Arming without 0x2000 leaves a drive
             that keeps turning when the runtime dies; arming outside
             velocity mode makes 0x2088 mean something else entirely; and a
             drive keeps its last RPM target across a dirty death of
             whoever commanded it, so the zero has to land first. Both
             controllers are prepared even when the first one refuses - the
             writes cannot arm anything, and the log then says whether one
             side or the whole bus is broken. One refusal anywhere and
             nobody gets the ENABLE word.
          2. read the fault registers (0x20A5/0x20A6) on BOTH, BEFORE any
             enable. Non-zero, or unreadable, and we stay disarmed (metrox,
             28/08 18:37: a fault used to be a warning AFTER arming, and
             write_enable still returned True). A drive we cannot question
             is a drive we do not arm.
          3. enable front, then back. If the back refuses, the front is
             rolled back immediately (zero + disable) - best effort, but it
             is the axle that answered a second ago.

        A complete transaction is also the explicit rearm: it, and only it,
        clears the runtime fault latch set by a refused command.

        In all three phases a RAISE is a refusal (`_took`), never an escape.
        The `except` below is the last net, and reaching it from inside phase 3
        would return False with the front axle armed and nothing left to roll
        it back - the exact defect this transaction exists to kill. It is
        unreachable today only because Controller._write_registers swallows
        everything, which is a promise made in another file.
        """
        try:
            with self._lock:
                if self._front is None or self._back is None:
                    return False
                if not enable:
                    return self._disarm_both()
                return self._arm_both()
        except Exception:
            return False

    def _sides(self):
        return ((self._front, "front"), (self._back, "back"))

    @staticmethod
    def _took(step) -> bool:
        """One drive write. A RAISE COUNTS AS A REFUSAL, exactly like a False.

        Controller._write_register(s) and _read swallow every serial exception
        today, so nothing below raises in practice - and that is the ONLY thing
        that made the transaction safe (review of 3ea2a00, 28/08). It is a
        property of zlac8015d.py, not of this file: a driver rewrite, a
        pymodbus that throws on a closed socket, a `unit` the client rejects,
        and the guarantee is gone. A raise crossing _arm_both lands in
        write_enable's outer `except`, which returns False having rolled back
        NOTHING - the audit's own defect, an armed front axle hiding behind a
        False. Same guard `_stand_down` already puts on the rollback steps.
        """
        try:
            return bool(step())
        except Exception:  # noqa: BLE001 - a drive that raises is a drive that refused
            return False

    def _disarm_both(self) -> bool:
        """Drop the ENABLE bit on both. Never blocked by the fault latch.

        Both are asked even when the first one refuses: a raise on the front
        used to skip the back's disable entirely, leaving the axle we could
        still reach under torque.
        """
        ok = True
        for c, side in self._sides():
            if not self._took(c.disable):
                ok = False
                _log().error(f"ZLAC8015D id {c.unit} ({side}) refused "
                             "disable (0x200E)")
        if ok:
            self._enabled = False
        return ok

    def _arm_both(self) -> bool:
        """The transaction. True only when both axles are armed."""
        # ── phase 1: prepare both, arm nobody ──────────────────────────
        prepared = True
        for c, side in self._sides():
            failed = self._prepare(c)
            if failed:
                prepared = False
                _log().error(
                    f"ZLAC8015D id {c.unit} ({side}) NOT armed, ENABLE "
                    f"(0x200E) withheld - refused at: {', '.join(failed)}")
        if not prepared:
            # Nobody was armed: there is nothing to roll back, and the fault
            # registers are not worth a serial timeout on a bus this sick.
            self._enabled = False
            return False

        # ── phase 2: the faults decide, before the ENABLE word ─────────
        if not self._faults_clear():
            self._enabled = False
            return False

        # ── phase 3: arm, front then back, rolling back on refusal ─────
        armed = []
        for c, side in self._sides():
            if not self._took(c.enable):
                _log().error(f"ZLAC8015D id {c.unit} ({side}) enable "
                             "sequence refused at: enable (0x200E)")
                for done, done_side in armed:
                    _log().error(f"ZLAC8015D id {done.unit} ({done_side}) was "
                                 "already armed - rolling it back (zero + "
                                 "disable), no axle stays under torque")
                    self._stand_down(done, done_side)
                self._enabled = False
                return False
            armed.append((c, side))

        self._enabled = True
        self._clear_fault_latch()
        return True

    def _prepare(self, controller) -> list[str]:
        """Everything but the ENABLE word. Returns the refused steps.

        Nothing is armed here, so an escaping raise could not have left an axle
        under torque - but it DID skip the other controller's preparation, the
        per-step log naming the culprit, and the `self._enabled = False` that
        follows a failed transaction, leaving the adapter claiming an armament
        it no longer has. Every step is a refusal-or-raise (`_took`).
        """
        failed = []
        if not self._took(controller.set_mode_velocity):
            failed.append("velocity mode (0x200D)")
        if not self._took(lambda: controller.set_accel_ms(self._accel_ms, self._accel_ms)):
            failed.append("accel/decel ramp (0x2080-0x2083)")
        if not self._took(lambda: controller.set_comm_offline_ms(self._comm_offline_ms)):
            failed.append("comm-offline watchdog (0x2000)")
        if not self._took(lambda: controller.set_rpm(0, 0)):
            failed.append("zero target (0x2088) before enable")
        return failed

    def _faults_clear(self) -> bool:
        """Both fault registers read, and both read clean. Nothing armed yet.

        A latched fault is the drive telling us why the last flight ended;
        arming through it was how "enabled WITH FAULTS" became a warning
        nobody read. Unreadable counts as faulted: the answer we did not get
        is not the answer we wanted - and a read that RAISES is the same
        non-answer, not an exit from the transaction. Both controllers are
        asked even when the first one is dirty, so the log names every bad
        drive in one go.
        """
        clear = True
        for c, side in self._sides():
            try:
                faults = c.get_faults()
            except Exception:  # noqa: BLE001 - a drive we cannot question is a drive we do not arm
                faults = None
            if faults is None:
                clear = False
                _log().error(
                    f"ZLAC8015D id {c.unit} ({side}) NOT armed: its fault "
                    "registers (0x20A5/0x20A6) did not answer - a drive we "
                    "cannot question is a drive we do not arm")
                continue
            l_fault, r_fault = faults
            if l_fault or r_fault:
                clear = False
                _log().error(
                    f"ZLAC8015D id {c.unit} ({side}) NOT armed: FAULTS "
                    f"L/R=0x{l_fault:04X}/0x{r_fault:04X} - see the ZLAC8015D "
                    "manual for the bit meanings, then clear them")
            else:
                _log().info(f"ZLAC8015D id {c.unit} ({side}) faults L/R=0/0, "
                            "clear to arm")
        return clear

    def _clear_fault_latch(self) -> None:
        """The rearm. Only a complete _arm_both() gets here."""
        if self._faulted is None:
            return
        reason, since = self._faulted
        self._faulted = None
        self._refused_cmds = 0
        self._last_fault_warn_t = 0.0
        _log().info(f"VECTOR base rearmed after {time.monotonic() - since:.1f}s "
                    f"in fault ({reason}) - commands flow again")

    def read_enabled(self) -> bool:
        return self._enabled

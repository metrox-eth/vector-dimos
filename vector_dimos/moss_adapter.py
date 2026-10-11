"""MOSS tracked base for dimOS: a TwistBaseAdapter over the QT Py motor firmware.

MOSS drives two geared DC motors through a Cytron MDD3A dual driver from an
Adafruit QT Py ESP32-S3 running `firmware/MOSS_QTPY_MDD3A` (moss repo, fw
"qtpy-mdd3a-v3"; an MDD10A build adds a "driver" key to the telemetry). The
firmware speaks a line protocol on the QT Py's native USB CDC port (115200
nominal; the host must hold DTR high to receive anything):

    arm                 -> "OK"      READY for 10 s, then it disarms itself
    drive L R           -> "OK"      L, R in percent, -100..+100 (firmware limit)
                                     must be REFRESHED within 300 ms or the
                                     firmware stops ("timeout") - a deadman
    stop                -> "OK"
    ramp UP DOWN        -> "OK"      ms per 1 % rise / fall, 1..500 each (RAM only)
    status              -> one telemetry JSON line
    (unsolicited)          one telemetry JSON line every 200 ms:
        {"ms":..,"fw":"qtpy-mdd3a-v3","mode":0-3,"reason":"..","target_pct":[l,r],
         "output_pct":[l,r],"ramp_ms":[rise,fall],"ticks":[l,r],"invalid_edges":[l,r],
         "ina_ok":..,"bus_mV":..,"shunt_uV":..,"current_mA":null}
    errors:                "ERR not_armed" | "ERR invalid_command" | "ERR stopped_busy"

Modes: 0 OFF, 1 READY (armed, idle), 2 DRIVE, 3 TEST.

What this adapter does with it (the VECTOR stack on the Jetson drives the
MOSS tracks):

- write_enable(True)  = "arm"; write_enable(False) = "stop".
- write_velocities([vx, vy, wz]) maps the body twist to left/right track
  speeds (differential drive: vy is IGNORED, a tracked base cannot strafe),
  then to percent of the firmware limit, and sends "drive L R". The firmware
  is open loop (PWM percent): the metres-per-second scale is a CALIBRATION
  number (MossGeometry.mps_at_limit), measured on the floor, not derived.
- a keepalive thread re-sends the last non-zero drive every KEEPALIVE_S
  while the last command is fresher than STALE_S; past that it sends "stop"
  ONCE and holds. So a dead commander stops the tracks in <= STALE_S here
  and, if this process dies too, in 300 ms in the firmware. Nothing here
  ever prolongs a command nobody is refreshing.
- the firmware disarms itself 10 s after "arm" with no drive, and after any
  invalid line. The adapter does NOT guess that from telemetry (a line up to
  200 ms old can say OFF while the firmware is already in DRIVE, and an "arm"
  sent in DRIVE is "arm_while_moving": a STOP). The firmware is the source of
  truth: a drive answered "ERR not_armed" re-arms and retries once.
- odometry: the firmware counts quadrature ticks; with MossGeometry
  .ticks_per_m set (measured), read_velocities()/read_odometry() come from
  tick deltas. Without it (today), read_velocities() returns the COMMANDED
  twist (open loop, low confidence) and read_odometry() returns None -
  an honest None beats a pose made of guesses.
- telemetry: the last firmware JSON line is kept (`telemetry`, a copy, and
  `telemetry_age_s`); the coordinator republishes it on `base_telemetry`
  (blueprints.VectorControlCoordinator, payload = telemetry_payload below).

dof: reported as 3 so the stock dimOS twist joints (vx, vy, wz) and the
VECTOR gamepad blueprint work unchanged; vy is dropped at the tracks.

Calibration (env, all optional):
    MOSS_PORT           serial port (default: the QT Py by-id path,
                        /dev/serial/by-id/*QT_Py*; when absent the sentinel
                        /dev/serial/by-id/MISSING-QT-PY, which fails loudly -
                        never /dev/ttyUSB0, the lidar on MOSS)
    MOSS_TRACK_M        distance between the two track centrelines, metres
                        (default robot_profile.MOSS.track_m, 0.266 CAD, placeholder)
    MOSS_MPS_AT_LIMIT   floor speed in m/s at MOSS_MAX_PCT, measured
    MOSS_MAX_PCT        percent ceiling, <= the firmware's 100
    MOSS_MIN_PCT        static-friction floor in percent (0 = off): a non-zero track command below it is raised to it, sign kept
    MOSS_TICKS_PER_M    encoder ticks per metre of track travel, measured
    MOSS_RAMP_MS        "UP DOWN" sent as 'ramp UP DOWN' after connect
    MOSS_MOCK_BUS       any truthy value: drive the fake firmware (no motors)
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from dataclasses import dataclass

from vector_dimos import robot_profile

BAUDRATE = 115200


def default_port() -> str:
    """MOSS_PORT, else the QT Py by-id path, else a sentinel that cannot open
    (robot_profile.moss_qtpy_port). Resolved when an adapter is built, so a
    board plugged after import is still found."""
    return robot_profile.moss_qtpy_port()


DEFAULT_PORT = default_port()   # import-time snapshot, kept for callers that read it
FIRMWARE_MAX_PCT = 100         # Control.h MAX_PERCENT: the firmware refuses more (25 until 10/10, the bench limit)
FIRMWARE_COMMAND_TIMEOUT_S = 0.300   # Control.h COMMAND_TIMEOUT_MS
FIRMWARE_ARM_TIMEOUT_S = 10.0        # Control.h ARM_TIMEOUT_MS
KEEPALIVE_S = 0.100            # re-send period, 3x inside the firmware deadman
STALE_S = 0.500                # no fresh write for this long -> "stop", once
REPLY_TIMEOUT_S = 0.150        # the firmware answers within a loop tick (ms)
CONNECT_PROBE_S = 1.000        # the first "status" of a process: reader thread start + first parse on a Jetson take longer than a reply tick
TELEMETRY_FRESH_S = 1.0        # mode read from telemetry older than this = unknown
MOCK_BUS_ENV = "MOSS_MOCK_BUS"

MODE_OFF, MODE_READY, MODE_DRIVE, MODE_TEST = 0, 1, 2, 3


_LOGGER = None


def _log():
    """dimOS's structlog logger when dimOS is there (that is what `dimos log`
    shows - a plain logging.getLogger() has no handler under the daemon and
    every line of this adapter was lost: first MOCK run 10/10), the stdlib
    with a handler otherwise. Resolved on first use so that importing this
    module never imports dimos (the cold bench)."""
    global _LOGGER
    if _LOGGER is None:
        try:
            from dimos.utils.logging_config import setup_logger
            _LOGGER = setup_logger()
        except Exception:
            lg = logging.getLogger("vector_dimos.moss")
            if not lg.handlers and not logging.getLogger().handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(
                    logging.Formatter("%(levelname)s %(name)s: %(message)s"))
                lg.addHandler(handler)
            lg.setLevel(logging.INFO)
            _LOGGER = lg
    return _LOGGER


def _env_truthy(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() not in ("", "0", "false", "no", "off")


def _env_float(name: str, default):
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError:
        _log().warning(f"MOSS base: {name}={raw!r} is not a number - using {default}")
        return default


@dataclass
class MossGeometry:
    """Everything the twist <-> tracks mapping needs. PLACEHOLDERS until
    measured on the rover (the moss build log still lists 'measure ticks per
    revolution' as open). Each field has an env override so a flight can be
    re-scaled without a code change."""
    track_m: float = robot_profile.MOSS.track_m   # PLACEHOLDER 0.266 (CAD track centre distance), to measure
    mps_at_limit: float = 0.45     # PLACEHOLDER: floor speed at 100 % (JGB37-520 178 rpm on a ~60 mm sprocket ~ 0.5 m/s), to measure
    max_pct: int = FIRMWARE_MAX_PCT
    ticks_per_m: float | None = None   # None = no encoder calibration yet
    min_pct: int = 0   # static-friction floor: a non-zero track command below this percent is raised to it (sign kept). 0 = off. MOSS_MIN_PCT, to measure on the floor (the planner's 0.2 rad/s pivots map to ~8 %, which does not move the tracks)

    @classmethod
    def from_env(cls) -> "MossGeometry":
        pct = int(_env_float("MOSS_MAX_PCT", FIRMWARE_MAX_PCT))
        tpm = _env_float("MOSS_TICKS_PER_M", None)
        max_pct = max(1, min(FIRMWARE_MAX_PCT, pct))
        floor = int(_env_float("MOSS_MIN_PCT", cls.min_pct))
        return cls(track_m=_env_float("MOSS_TRACK_M", cls.track_m),
                   mps_at_limit=_env_float("MOSS_MPS_AT_LIMIT", cls.mps_at_limit),
                   max_pct=max_pct,
                   ticks_per_m=(tpm if tpm and tpm > 0 else None),
                   min_pct=max(0, min(max_pct, floor)))


def twist_to_tracks(vx: float, wz: float, g: MossGeometry) -> tuple[int, int]:
    """Body twist -> (left, right) percent, clamped to +-g.max_pct.

    Differential drive: v_left = vx - wz*track/2, v_right = vx + wz*track/2.
    Percent is proportional to speed with g.mps_at_limit <-> g.max_pct.
    A twist too small to round to 1 % IS a stop (0, 0). A non-zero percent
    below g.min_pct is raised to g.min_pct with its sign (static-friction
    floor: the tracks do not turn below it, so the alternative is a stall)."""
    half = g.track_m / 2.0
    v_l = vx - wz * half
    v_r = vx + wz * half
    scale = g.max_pct / g.mps_at_limit if g.mps_at_limit > 0 else 0.0
    lim = int(g.max_pct)
    floor = max(0, min(lim, int(g.min_pct)))

    def pct(v: float) -> int:
        p = int(round(v * scale))
        p = max(-lim, min(lim, p))
        if p and abs(p) < floor:
            p = floor if p > 0 else -floor
        return p

    return pct(v_l), pct(v_r)


TURN_GAIN_DEFAULT = 2.0   # MOSS_TURN_GAIN: turn authority vs forward speed, see teleop_speeds


def turn_gain() -> float:
    return max(0.5, min(4.0, _env_float("MOSS_TURN_GAIN", TURN_GAIN_DEFAULT)))


def teleop_envelope(g: MossGeometry) -> dict:
    """The gamepad's last gate (clamp_twist), sized to the tracks instead of
    VECTOR's mecanum rim. Per-axis ceilings = teleop_speeds; the rim is set
    so that a mixed command is NEVER scaled down as a whole: the adapter
    clamps each track on its own (twist_to_tracks), which is what gives the
    tank feel below. Keys match gamepad.TwistEnvelope."""
    lin, ang = teleop_speeds(g)
    lever = g.track_m / 2.0
    return {"linear": lin, "angular": ang, "lever": lever, "rim": lin + ang * lever}


def teleop_speeds(g: MossGeometry) -> tuple[float, float]:
    """(linear m/s, angular rad/s) at FULL stick for the gamepad module, sized
    to the base: full stick forward = g.max_pct on both tracks. Full stick
    sideways = turn_gain() times the spin-in-place rate (v = wz * track/2):
    with the default gain 2, at full forward the inner track STOPS at half
    turn stick and REVERSES beyond it, full turn = a full pivot (operator
    request 10/10: the inner track should reverse past a turn threshold, not
    only stop); gain 1 = the inner track only ever stops.
    VECTOR's defaults (0.6 m/s, 1.2 rad/s) would saturate at 25 % from the
    deadzone on, an on/off throttle; this keeps the stick proportional."""
    lin = float(g.mps_at_limit)
    ang = turn_gain() * 2.0 * lin / g.track_m if g.track_m > 0 else 0.0
    return lin, ang


def tracks_to_twist(v_l: float, v_r: float, g: MossGeometry) -> tuple[float, float, float]:
    """(left, right) track speeds in m/s -> (vx, vy=0, wz)."""
    vx = (v_l + v_r) / 2.0
    wz = (v_r - v_l) / g.track_m if g.track_m > 0 else 0.0
    return vx, 0.0, wz


class MossBaseAdapter:
    """TwistBaseAdapter for MOSS (two tracks, QT Py ESP32-S3 + MDD3A firmware, USB CDC).

    Args:
        dof: reported DOF (3, see module docstring).
        address: serial port; None = default_port() (MOSS_PORT, the QT Py by-id, or the sentinel).
        port_factory: injectable callable returning a serial-like object
            (write / readline / in_waiting / close / is_open). Benches inject
            the fake firmware; MOSS_MOCK_BUS does the same in a real dimOS run.
        geometry: MossGeometry; None = MossGeometry.from_env().
    """

    def __init__(self, dof: int = 3, address: str | None = None,
                 hardware_id: str = "base", port_factory=None,
                 geometry: MossGeometry | None = None, **_ignored):
        self._dof = dof
        self._port = address or default_port()
        self._hardware_id = hardware_id
        self._port_factory = port_factory
        self._geometry = geometry or MossGeometry.from_env()
        self._ser = None
        self._mock = False
        self._lock = threading.RLock()
        self._reply_cv = threading.Condition(self._lock)
        self._replies: list[str] = []
        self._telemetry: dict | None = None
        self._tel_logged_t = 0.0
        self._telemetry_t: float | None = None
        self._connected = False
        self._enabled = False
        self._target = (0, 0)
        self._target_twist = (0.0, 0.0, 0.0)
        self._target_t = 0.0
        self._stopped_for_stale = True
        self._last_logged = None
        self._reader = None
        self._keepalive = None
        self._stop_threads = threading.Event()
        # odometry from ticks
        self._pose = [0.0, 0.0, 0.0]
        self._last_ticks: tuple[int, int] | None = None
        self._last_ticks_t: float | None = None
        self._velocity = (0.0, 0.0, 0.0)

    # ── properties for benches ─────────────────────────────────────────
    @property
    def geometry(self) -> MossGeometry:
        return self._geometry

    @property
    def mock_bus(self) -> bool:
        return self._mock

    @property
    def target_pct(self) -> tuple[int, int]:
        return self._target

    @property
    def firmware_mode(self) -> int | None:
        """Mode from the last telemetry line, None if none or stale."""
        with self._lock:
            if self._telemetry is None or self._telemetry_t is None:
                return None
            if time.monotonic() - self._telemetry_t > TELEMETRY_FRESH_S:
                return None
            try:
                return int(self._telemetry.get("mode"))
            except (TypeError, ValueError):
                return None

    @property
    def telemetry(self) -> dict | None:
        with self._lock:
            return dict(self._telemetry) if self._telemetry else None

    @property
    def telemetry_age_s(self) -> float | None:
        """Seconds since the last telemetry line arrived (same monotonic stamp
        firmware_mode uses), None before the first one."""
        with self._lock:
            if self._telemetry_t is None:
                return None
            return max(0.0, time.monotonic() - self._telemetry_t)

    # ── connection ─────────────────────────────────────────────────────
    def _open_port(self):
        if self._port_factory is not None:
            self._mock = getattr(self._port_factory, "is_mock", False)
            return self._port_factory()
        if _env_truthy(MOCK_BUS_ENV):
            from .moss_mock import FakeMossFirmware
            self._mock = True
            _log().info(f"MOSS base: MOCK BUS ({MOCK_BUS_ENV} set) - no motors will move")
            return FakeMossFirmware(auto_time=True)   # wall clock: live telemetry, real deadmen
        import serial
        return serial.Serial(self._port, BAUDRATE, timeout=0.05)

    def connect(self) -> bool:
        with self._lock:
            if self._connected:
                return True
            try:
                self._ser = self._open_port()
            except Exception as exc:
                _log().error(f"MOSS base: cannot open {self._port} @{BAUDRATE}: {exc!r}")
                self._ser = None
                return False
            self._stop_threads.clear()
            self._reader = threading.Thread(target=self._read_loop, name="moss-rx", daemon=True)
            self._reader.start()
            # probe: the firmware answers "status" with one telemetry line
            self._replies.clear()
            if not self._send("status", expect_status=True, timeout=CONNECT_PROBE_S):
                _log().error(f"MOSS base: no telemetry from the firmware on "
                             f"{self._port} @{BAUDRATE} (is it the QT Py by-id port, "
                             "and is the firmware flashed?)")
                self._teardown()
                return False
            self._connected = True
            self._keepalive = threading.Thread(target=self._keepalive_loop,
                                               name="moss-keepalive", daemon=True)
            self._keepalive.start()
            ramp = os.environ.get("MOSS_RAMP_MS", "").strip()   # "10 5" = ms per % up / down
            if ramp and not self._mock:
                if self._send(f"ramp {ramp}"):
                    _log().info(f"MOSS base: firmware ramp set to {ramp} ms per % (rise fall)")
                else:
                    _log().warning(f"MOSS base: 'ramp {ramp}' refused ({self._last_err}) - firmware default kept")
            _log().info(f"MOSS base: connected on {self._port}, firmware mode="
                        f"{self.firmware_mode} geometry={self._geometry}")
            return True

    def _teardown(self) -> None:
        self._stop_threads.set()
        ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        self._connected = False
        self._enabled = False

    def disconnect(self) -> None:
        with self._lock:
            if self._connected:
                try:
                    self._send("stop")
                except Exception:
                    pass
            self._teardown()

    def is_connected(self) -> bool:
        return self._connected

    def get_dof(self) -> int:
        return self._dof

    # ── serial plumbing ────────────────────────────────────────────────
    def _read_loop(self) -> None:
        while not self._stop_threads.is_set():
            ser = self._ser
            if ser is None:
                return
            try:
                raw = ser.readline()
            except Exception as exc:
                # the port closed under us (stack stopping): not an error
                if self._stop_threads.is_set() or not getattr(ser, "is_open", True):
                    return
                _log().error(f"MOSS base: serial read failed: {exc!r}")
                time.sleep(0.05)
                continue
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            if not line:
                continue
            with self._lock:
                if line.startswith("{"):
                    try:
                        self._telemetry = json.loads(line)
                        self._telemetry_t = time.monotonic()
                        self._ingest_ticks(self._telemetry)
                    except (ValueError, TypeError):
                        pass
                    self._replies.append("STATUS")
                elif line == "OK" or line.startswith("ERR"):
                    self._replies.append(line)
                else:
                    # boot banner and the like
                    _log().info(f"MOSS firmware: {line}")
                    continue
                self._reply_cv.notify_all()

    def _send(self, line: str, expect_status: bool = False, timeout: float | None = None) -> bool:
        """Write one command line and wait for its reply. Called with the lock
        held (RLock). True on "OK" (or a telemetry line when expect_status)."""
        ser = self._ser
        if ser is None:
            return False
        before = len(self._replies)
        try:
            ser.write((line + "\n").encode())
        except Exception as exc:
            _log().error(f"MOSS base: serial write failed ({line!r}): {exc!r}")
            return False
        deadline = time.monotonic() + (REPLY_TIMEOUT_S if timeout is None else timeout)
        while True:
            for reply in self._replies[before:]:
                if expect_status and reply == "STATUS":
                    return True
                if not expect_status and reply == "OK":
                    return True
                if reply.startswith("ERR"):
                    self._last_err = reply
                    return False
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._last_err = "timeout"
                return False
            self._reply_cv.wait(remaining)

    _last_err = ""

    # ── enable ─────────────────────────────────────────────────────────
    def write_enable(self, enable: bool) -> bool:
        with self._lock:
            if not self._connected:
                return False
            if not enable:
                ok = self._send("stop")
                self._enabled = False
                self._target = (0, 0)
                self._stopped_for_stale = True
                return ok
            if not self._arm():
                self._enabled = False
                return False
            self._enabled = True
            return True

    def _arm(self) -> bool:
        ok = self._send("arm")
        if not ok:
            _log().error(f"MOSS base: 'arm' refused ({self._last_err})")
        return ok

    def read_enabled(self) -> bool:
        return self._enabled

    # ── commands ───────────────────────────────────────────────────────
    def write_velocities(self, velocities: list[float]) -> bool:
        with self._lock:
            if not self._connected:
                return False
            try:
                vx, vy, wz = (list(velocities) + [0.0, 0.0, 0.0])[:3]
                vx, vy, wz = float(vx), float(vy), float(wz)
            except Exception:
                return False
            left, right = twist_to_tracks(vx, wz, self._geometry)
            if (left, right) != (0, 0) and not self._enabled:
                return False
            ok = self._drive(left, right)
            if ok:
                self._target = (left, right)
                self._target_twist = (vx, 0.0, wz)
                self._target_t = time.monotonic()
                self._stopped_for_stale = (left, right) == (0, 0)
                self._velocity = self._velocity if self._geometry.ticks_per_m else (vx, 0.0, wz)
                self._log_cmd((vx, vy, wz), (left, right))
            return ok

    def _drive(self, left: int, right: int) -> bool:
        """Send 'drive L R'; on 'ERR not_armed' (arm expiry, invalid line,
        the firmware's own deadman) re-arm and retry ONCE. Never pre-arm
        from telemetry: see the module docstring. A (0, 0) drive is sent as
        'stop' when the firmware is not armed: stopping never needs an arm."""
        if (left, right) == (0, 0) and self.firmware_mode in (MODE_OFF, None):
            return self._send("stop")
        if self._send(f"drive {left} {right}"):
            return True
        if self._last_err == "ERR not_armed" and self._enabled:
            reason = (self._telemetry or {}).get("reason")
            _log().warning(f"MOSS base: 'drive' refused not_armed (firmware reason: "
                           f"{reason}) - re-arming once")
            if self._arm() and self._send(f"drive {left} {right}"):
                return True
        _log().error(f"MOSS base: 'drive {left} {right}' refused ({self._last_err})")
        return False

    def write_stop(self) -> bool:
        with self._lock:
            if not self._connected:
                return False
            ok = self._send("stop")
            if ok:
                self._target = (0, 0)
                self._target_twist = (0.0, 0.0, 0.0)
                self._target_t = time.monotonic()
                self._stopped_for_stale = True
                if not self._geometry.ticks_per_m:
                    self._velocity = (0.0, 0.0, 0.0)
                self._log_cmd((0.0, 0.0, 0.0), (0, 0))
            return ok

    def _keepalive_loop(self) -> None:
        """Refresh the last drive inside the firmware deadman while it is
        fresh; past STALE_S send one 'stop' and hold until a new write."""
        while not self._stop_threads.is_set():
            time.sleep(KEEPALIVE_S)
            with self._lock:
                if not self._connected or self._stopped_for_stale:
                    continue
                age = time.monotonic() - self._target_t
                if age > STALE_S:
                    _log().warning(f"MOSS base: no twist for {age:.2f}s - stop (deadman)")
                    self._send("stop")
                    self._target = (0, 0)
                    self._target_twist = (0.0, 0.0, 0.0)
                    self._stopped_for_stale = True
                    if not self._geometry.ticks_per_m:
                        self._velocity = (0.0, 0.0, 0.0)
                    continue
                if self._target != (0, 0):
                    self._send(f"drive {self._target[0]} {self._target[1]}")
                    # what the firmware actually applies, and the battery under
                    # load (10/10, "on manque de puissance"): one line per 2 s
                    tel = self._telemetry
                    now = time.monotonic()
                    if tel and now - self._tel_logged_t > 2.0:
                        self._tel_logged_t = now
                        _log().info(f"MOSS telemetry: out={tel.get('output_pct')} target={tel.get('target_pct')} "
                                    f"bus={tel.get('bus_mV')} mV ticks={tel.get('ticks')} mode={tel.get('mode')}")

    def _log_cmd(self, twist, pct) -> None:
        if pct == self._last_logged:
            return
        self._last_logged = pct
        head = "MOSS base MOCK:" if self._mock else "MOSS base:"
        vx, vy, wz = twist
        _log().info(f"{head} twist vx={vx:+.3f} vy={vy:+.3f}(ignored) wz={wz:+.3f} "
                    f"-> tracks L={pct[0]:+d}% R={pct[1]:+d}% (limit {self._geometry.max_pct}%)")

    # ── feedback ───────────────────────────────────────────────────────
    def _ingest_ticks(self, tel: dict) -> None:
        """Called with the lock held, on every telemetry line."""
        tpm = self._geometry.ticks_per_m
        if not tpm:
            return
        try:
            l, r = tel["ticks"]
            l, r = int(l), int(r)
        except (KeyError, TypeError, ValueError):
            return
        now = time.monotonic()
        if self._last_ticks is not None and self._last_ticks_t is not None:
            dt = now - self._last_ticks_t
            if dt > 0:
                v_l = (l - self._last_ticks[0]) / tpm / dt
                v_r = (r - self._last_ticks[1]) / tpm / dt
                vx, _, wz = tracks_to_twist(v_l, v_r, self._geometry)
                self._velocity = (vx, 0.0, wz)
                x, y, th = self._pose
                d = (v_l + v_r) / 2.0 * dt
                dth = wz * dt
                x += d * math.cos(th + dth / 2.0)
                y += d * math.sin(th + dth / 2.0)
                th += dth
                self._pose = [x, y, th]
        self._last_ticks = (l, r)
        self._last_ticks_t = now

    def read_velocities(self) -> list[float]:
        with self._lock:
            return list(self._velocity)

    def read_odometry(self) -> list[float] | None:
        with self._lock:
            if not self._geometry.ticks_per_m:
                return None
            return list(self._pose)


# ── telemetry payload for the bus (VectorControlCoordinator.base_telemetry) ──

TELEMETRY_KEYS = ("fw", "driver", "mode", "reason", "bus_mV", "ina_ok", "ticks",
                  "target_pct", "output_pct", "ramp_ms")


def telemetry_payload(adapter, now: float) -> dict | None:
    """One base_telemetry document from anything shaped like MossBaseAdapter
    (`telemetry` dict or None, `telemetry_age_s`, `mock_bus`); None when the
    adapter has no telemetry (VECTOR's adapter, or MOSS before its first
    line), so the coordinator publishes nothing.

    Firmware keys are copied as they are; a key the flashed build does not
    send (driver before the MDD10A build) is None, never invented. `ts` is
    the caller's wall clock, `firmware_age_s` the adapter's monotonic age.
    """
    tel = getattr(adapter, "telemetry", None)
    if not isinstance(tel, dict) or not tel:
        return None
    age = getattr(adapter, "telemetry_age_s", None)
    doc = {"ts": float(now)}
    for key in TELEMETRY_KEYS:
        doc[key] = tel.get(key)
    doc["firmware_age_s"] = None if age is None else round(float(age), 3)
    doc["mock"] = bool(getattr(adapter, "mock_bus", False))
    return doc

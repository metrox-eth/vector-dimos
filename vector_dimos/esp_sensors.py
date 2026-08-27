"""Contact switches + sonar via the ESP32-S3 bridge (USB serial).

Born 25/08/2026 after the TXB0108 verdict (docs/verdict_museau_20260825.md):
the Jetson 40-pin could not read these sensors; a 2$ ESP32 does it natively.
The ESP firmware (firmware/esp32_sonar/) prints:
    SW a b c d     on every debounced change + 500 ms heartbeat (1 = pressed)
    SONAR <m>      at 10 Hz (-1 = no echo)

Sensor doctrine: this module does NOT write the map. The
lidar is the backbone of localization and mapping; the RealSense camera is
the only other sensor allowed to put obstacles down. The switches and the
sonar are reflexes:

- ``bump``        (Bool) front corner contact -> planner: stop, back off, replan
- ``bump_rear``   (Bool) rear corner contact  -> planner: stop, move forward, replan
- ``sonar_range`` (Float32, metres) front proximity -> the base adapter's
  forward brake (adapter.brake_forward). A reading is a speed limit for the
  next second, never a fact about the world.

Corner map (validated live 2026-08-25): SW order = GPIO 1,2,3,4 =
avant-gauche, arriere-gauche, arriere-droit, avant-droit (the firmware's own
labels: front-left, rear-left, rear-right, front-right).
Sonar: front centre of the bumper, usable range measured on the robot = 66 cm
-> trust cap 0.55 m, i.e. the measured range minus a margin. Median of 3,
spread < 0.10 m.
"""

from __future__ import annotations

import sys
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.utils.logging_config import setup_logger
from dimos_lcm.std_msgs import Bool

logger = setup_logger()


def _log(msg: str) -> None:
    """One line on BOTH channels, because neither alone is enough.

    stderr is only captured while the launcher is still in the foreground: it
    stops at "All modules started", and everything a worker prints during the
    run itself goes nowhere (measured 26/08 on run B - the launch log ends at
    03:15:18, the run ended at 03:23:33). The dimOS structured logger is the
    channel that keeps writing into the run's main.jsonl for the whole run,
    and this module was the only one in the package not using it: it held a
    bare `logging.getLogger(__name__)`, which no worker configures. So every
    BUMP after start-up was published to LCM and logged to a black hole, and
    the switches looked dead while they were working.
    """
    logger.info(f"[esp_sensors] {msg}")
    print(f"[esp_sensors] {msg}", file=sys.stderr, flush=True)

ESP_PORT = "/dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_80b54ee325280000-if00"
ESP_BAUD = 115200

# SW index -> (name, body position (x, y), rear?)
CORNERS = (   # body 62.5 x 46 cm with the bumper bars (measured)
    ("avant-gauche",  (0.31,  0.23), False),
    ("arriere-gauche", (-0.31, 0.23), True),
    ("arriere-droit", (-0.31, -0.23), True),
    ("avant-droit",   (0.31, -0.23), False),
)
SONAR_MAX_TRUSTED = 0.55        # 66 cm measured on the robot, minus a margin
SONAR_MEDIAN = 3
SONAR_SPREAD_MAX = 0.10
CONTACT_COOLDOWN_S = 1.0
BUMP_HOLD_S = 0.10        # a corner must stay closed this long to be a CONTACT.
                          # measured 2026-08-27: both FRONT corners closed 104 ms
                          # apart on a STANDING rover, then more on every
                          # acceleration - the bumper bar (much softer springs
                          # since 2026-08-26)
                          # flutters and closes its own switches. A real
                          # collision keeps pressing until the escape; a flutter
                          # releases within the window (the ESP sends a state
                          # line on EVERY change, measured). 0.10 s = 1.5 cm of
                          # push at cruise - inside the bar's spring travel.
SONAR_PUBLISH_PERIOD_S = 0.2    # <= 5 Hz on sonar_range
# Decided 2026-08-26, after the bumper cushion lifted a few millimetres and sat
# in front of the sonar for two hours (0.08 m readings, every drive clamped
# before the brake itself was ripped out): NO sonar at all. A sensor we do not
# yet know how to use stays off until we do. The switches stay: they are the
# safety. The preflight still reads the raw ESP stream and fails loudly under
# 0.30 m ("coussinet ?"), so a blocked sonar is caught at every flight even
# while nothing consumes it.
SONAR_ENABLED = False
SONAR_CLEAR_AFTER_S = 1.0       # nothing believable for this long = the way is clear
SONAR_CLEAR_PERIOD_S = 1.0      # heartbeat rate while clear
SONAR_CLEAR_M = 9.9             # "clear" value: past every brake threshold


def parse_line(line: str):
    """('sw', (a,b,c,d)) | ('sonar', metres) | None — cold-testable."""
    parts = line.strip().split()
    if len(parts) == 5 and parts[0] == "SW":
        try:
            return ("sw", tuple(int(p) for p in parts[1:5]))
        except ValueError:
            return None
    if len(parts) == 2 and parts[0] == "SONAR":
        try:
            return ("sonar", float(parts[1]))
        except ValueError:
            return None
    return None


class SonarFilter:
    """Median of 3, spread gate, trust cap — cold-testable."""

    def __init__(self) -> None:
        self._readings: list[float] = []

    def feed(self, d: float) -> float | None:
        if d <= 0 or d > SONAR_MAX_TRUSTED:
            return None
        self._readings.append(d)
        self._readings = self._readings[-SONAR_MEDIAN:]
        if len(self._readings) < SONAR_MEDIAN:
            return None
        if max(self._readings) - min(self._readings) >= SONAR_SPREAD_MAX:
            return None
        return sorted(self._readings)[SONAR_MEDIAN // 2]


def sonar_publication(median: float | None, clear_for_s: float,
                      since_publish_s: float) -> float | None:
    """What to publish on ``sonar_range`` right now, in metres, or None.

    Two cases, and nothing in between:

    * a fresh filtered median (something is within the trust cap) -> that
      distance, at most one publication per SONAR_PUBLISH_PERIOD_S (5 Hz);
    * no believable echo for SONAR_CLEAR_AFTER_S (the sonar reported no echo,
      or every echo was past the trust cap) -> SONAR_CLEAR_M once per
      SONAR_CLEAR_PERIOD_S. That heartbeat is what releases the brake in
      adapter.brake_forward; without it the brake would simply age out.

    A close but noisy burst (spread gate rejects it) publishes nothing: the
    brake keeps its last value until it ages out. Pure — cold-testable.
    """
    if median is not None:
        return float(median) if since_publish_s >= SONAR_PUBLISH_PERIOD_S else None
    if clear_for_s >= SONAR_CLEAR_AFTER_S and since_publish_s >= SONAR_CLEAR_PERIOD_S:
        return SONAR_CLEAR_M
    return None


class EspSensors(Module):
    """Outs: bump, bump_rear (safety reflexes), sonar_range (proximity brake).

    No map-writing Out by design — see the module docstring.
    """

    bump: Out[Bool]
    bump_rear: Out[Bool]
    sonar_range: Out[Float32]

    def __init__(self, port: str = ESP_PORT, enabled: bool = True,
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.port, self.enabled = port, enabled
        self._reset_state()

    def _reset_state(self) -> None:
        """The reader's whole state in one place: the cold bench builds a
        module without dimOS's Module machinery and has to start exactly
        where the constructor does."""
        self._sw = (0, 0, 0, 0)
        self._last_contact = 0.0
        self._sonar = SonarFilter()
        self._last_sonar_publish = 0.0
        self._clear_since: float | None = None
        self._running = False
        self.contacts = 0

    # ---- plumbing -----------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        if not self.enabled:
            logger.info("esp_sensors disabled by config")
            return
        self._running = True
        threading.Thread(target=self._serial_loop, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running = False
        super().stop()

    # ---- serial -------------------------------------------------------------
    def _serial_loop(self) -> None:
        """Read the ESP forever. This thread is not allowed to die quietly:
        anything it swallows is a sensor the robot no longer has."""
        import serial
        while self._running:
            ser = None
            try:
                ser = serial.Serial(self.port, ESP_BAUD, timeout=2.0)
                _log(f"ESP link open ({self.port})")
                while self._running:
                    raw = ser.readline()
                    if not raw:
                        continue
                    try:
                        self._handle_line(raw.decode(errors="replace"))
                    except Exception as e:  # noqa: BLE001
                        # One bad line (or one refused publish) used to tear the
                        # link down for 2 s and take the sonar brake with it.
                        _log(f"line dropped ({type(e).__name__}: {e}): {raw!r}")
                        logger.exception("esp_sensors: handling a serial line failed")
            except Exception as e:  # noqa: BLE001 - the ESP can be unplugged
                _log(f"ESP link LOST ({type(e).__name__}: {e}), retrying in 2 s")
                logger.exception("esp_sensors: serial link lost")
                time.sleep(2.0)
            finally:
                if ser is not None:
                    try:
                        ser.close()          # reconnecting used to leak the fd
                    except Exception:        # noqa: BLE001
                        pass

    def _handle_line(self, line: str) -> None:
        parsed = parse_line(line)
        if parsed is None:
            return
        kind, value = parsed
        if kind == "sw":
            prev, self._sw = self._sw, value
            if any(value):
                # Permanent instrumentation: a heartbeat with a bit set is the
                # only proof the switches are talking. ~2 lines/s while a
                # corner is held, nothing at all the rest of the time.
                _log("SW rx: " + " ".join(str(v) for v in value))
            for i in range(4):
                if value[i] and not prev[i]:
                    # confirm after BUMP_HOLD_S: fire only if still pressed
                    threading.Timer(BUMP_HOLD_S, self._confirm_contact, args=(i,)).start()
            return
        now = time.monotonic()
        median = self._sonar.feed(value)
        # "clear" = no echo at all, or an echo past the trust cap
        if value <= 0 or value > SONAR_MAX_TRUSTED:
            if self._clear_since is None:
                self._clear_since = now
        else:
            self._clear_since = None
        clear_for = 0.0 if self._clear_since is None else now - self._clear_since
        if not SONAR_ENABLED:
            return
        out = sonar_publication(median, clear_for, now - self._last_sonar_publish)
        if out is not None:
            self._last_sonar_publish = now
            self.sonar_range.publish(Float32(data=out))

    # ---- contacts -----------------------------------------------------------
    def _confirm_contact(self, i: int) -> None:
        """BUMP_HOLD_S after a rising edge: still pressed = real contact,
        released meanwhile = bar flutter, dropped silently."""
        if self._sw[i]:
            name, _xy, rear = CORNERS[i]
            self._contact(name, rear)

    def _contact(self, name: str, rear: bool) -> None:
        now = time.monotonic()
        if now - self._last_contact < CONTACT_COOLDOWN_S:
            return
        self._last_contact = now
        self.contacts += 1
        # Log first, publish second: a contact we saw must be on the record even
        # if the stream refuses it, and a refused stream must not cost the link.
        _log(f"BUMP #{self.contacts}: {name} ({'rear' if rear else 'front'}) "
             "-> stop, back off, replan")
        out = self.bump_rear if rear else self.bump
        try:
            out.publish(Bool(data=True))
        except Exception as e:  # noqa: BLE001
            _log(f"BUMP #{self.contacts} NOT published on "
                 f"{'bump_rear' if rear else 'bump'} ({type(e).__name__}: {e})")

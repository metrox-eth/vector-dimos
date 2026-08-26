"""Integration cold test for the ESP RX path: a real pty, real pyserial, the
real ``EspSensors._serial_loop`` thread. No robot, no ESP32.

Why this file exists (26/08): every earlier validation of the contact switches
used an ad-hoc listener on the port, so ``vector_dimos/esp_sensors.py`` itself -
its serial loop, its decoding, its edge detection, its publishing - had NEVER
been exercised end to end. ``test_esp_sensors_cold.py`` covers the pure pieces
(parse_line, the sonar filter, the corner map) and stops exactly where the
field doubt lived: the loop that feeds them.

Rule #2, in physical units: a known byte stream on a known port must give a
known message on a known Out, and the timings here are real seconds (the 1 s
contact cooldown is physical time, not a mock).

What is replayed is the firmware's actual interleaved output:
    "MUSEAU-ESP ..."   banner, once at boot
    "SONAR <m>"        10 Hz
    "SW a b c d"       500 ms heartbeat + one line per debounced change

Every check reads BOTH channels the module can log on (the dimOS structured
logger, which reaches the run's main.jsonl, and stderr, which a daemon launch
stops capturing once the modules are up), because "the switches never fired"
and "we never saw them fire" look identical from the outside.

Run:  .venv/bin/python3 tests/test_esp_serial_rx_cold.py
"""
import io
import os
import pty
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dimos.core.stream import Out  # noqa: E402
from dimos.utils.logging_config import set_run_log_dir  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402

# A run's log directory, exactly as `dimos` sets one up before build(). It has
# to be set BEFORE the module is imported: that is when its logger is built.
RUN_LOG_DIR = Path(tempfile.mkdtemp(prefix="esp_rx_runlog_"))
set_run_log_dir(RUN_LOG_DIR)

from vector_dimos import esp_sensors  # noqa: E402
from vector_dimos.esp_sensors import CONTACT_COOLDOWN_S, EspSensors  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- the harness -------------------------------------------------------------

class FakeTransport:
    """Stands in for LCMTransport: records, or refuses if `boom`."""

    def __init__(self, boom=False):
        self.msgs, self.boom = [], boom

    def broadcast(self, stream, msg):
        if self.boom:
            raise RuntimeError("transport not ready")
        self.msgs.append(msg)


class LogRecorder:
    """Reads the dimOS structured logger channel without muting it: what the
    module logs is recorded here AND passed on to the real logger, so section D
    can then look for it in the run's main.jsonl."""

    def __init__(self, tee=None):
        self.lines, self.tee = [], tee

    def _add(self, msg, *a, **k):
        self.lines.append(str(msg))
        if self.tee is not None:
            self.tee.info(msg)

    info = warning = error = exception = _add


class Rig:
    """A pty pretending to be the ESP32, plus a live EspSensors reader thread."""

    def __init__(self, bump_boom=False):
        self.master, slave = pty.openpty()
        self.port = os.ttyname(slave)
        self.bump = Out(Bool, "bump", None, FakeTransport(boom=bump_boom))
        self.bump_rear = Out(Bool, "bump_rear", None, FakeTransport())
        self.sonar_range = Out(esp_sensors.Float32, "sonar_range", None, FakeTransport())

        m = EspSensors.__new__(EspSensors)
        m.port, m.enabled = self.port, True
        m._reset_state()
        m.bump, m.bump_rear, m.sonar_range = self.bump, self.bump_rear, self.sonar_range
        m._running = True
        self.module = m

        # both channels captured: neither is allowed to be the only witness
        self.log = LogRecorder(tee=esp_sensors.logger)
        self._saved_logger, esp_sensors.logger = esp_sensors.logger, self.log
        self.err = io.StringIO()
        self._saved_stderr, sys.stderr = sys.stderr, self.err

        self.thread = threading.Thread(target=m._serial_loop, daemon=True)
        self.thread.start()
        time.sleep(0.2)                      # let the port open

    def send(self, text):
        os.write(self.master, text.encode())

    def close(self):
        self.module._running = False
        os.write(self.master, b"\n")         # unblock a pending readline
        self.thread.join(timeout=3.0)
        esp_sensors.logger = self._saved_logger
        sys.stderr = self._saved_stderr
        os.close(self.master)

    # what the operator could possibly have seen, on either channel
    def _all(self):
        return self.log.lines + self.err.getvalue().splitlines()

    def saw(self, needle):
        return any(needle in line for line in self._all())

    def count(self, needle):
        """Every line goes to both channels, so count the busiest one."""
        return max(sum(1 for line in self.log.lines if needle in line),
                   sum(1 for line in self.err.getvalue().splitlines() if needle in line))

    @property
    def bumps(self):
        return self.bump._transport.msgs

    @property
    def rear_bumps(self):
        return self.bump_rear._transport.msgs

    @property
    def sonars(self):
        return self.sonar_range._transport.msgs


def replay_field_stream(rig):
    """The firmware's own interleaving, in real time: 3.4 s of a run in which
    the rover drives into its front-left corner and holds it for 2 s, releases,
    then clips its rear-left corner with a click.

    Returns the lines actually written, for the report.
    """
    rig.send("MUSEAU-ESP v2: switches GPIO 1-4 (active low) + sonar 5/6\r\n")
    state = (0, 0, 0, 0)
    pressed = released = clicked = False
    t0 = time.monotonic()
    last_beat = last_ping = -1.0
    sent = []

    def emit(line):
        sent.append(line)
        rig.send(line + "\r\n")

    while True:
        t = time.monotonic() - t0
        if t >= 3.4:
            break
        if t >= 0.5 and not pressed:
            pressed, state = True, (1, 0, 0, 0)      # LONG press, front-left
            emit("SW 1 0 0 0")
        if t >= 2.5 and pressed and not released:
            released, state = True, (0, 0, 0, 0)     # release
            emit("SW 0 0 0 0")
        if t >= 3.0 and not clicked:
            clicked = True
            emit("SW 0 1 0 0")                       # short CLICK, rear-left
            emit("SW 0 0 0 0")                       # ... back to rest at once
        if t - last_beat >= 0.5:
            last_beat = t
            emit("SW " + " ".join(str(v) for v in state))
        if t - last_ping >= 0.1:
            last_ping = t
            emit("SONAR 0.420")
        time.sleep(0.005)
    time.sleep(0.3)                                  # let the reader drain
    return sent


# --- A. the field stream -----------------------------------------------------

print("A. the firmware's real interleaved stream, over a real pty (3.4 s)")
rig = Rig()
sent = replay_field_stream(rig)
sw_sent = [line for line in sent if line.startswith("SW")]
sw_set_sent = [line for line in sw_sent if line != "SW 0 0 0 0"]
sonar_sent = [line for line in sent if line.startswith("SONAR")]
print(f"  ({len(sent)} lines written: {len(sw_sent)} SW of which {len(sw_set_sent)} "
      f"with a bit set, {len(sonar_sent)} SONAR)")

check("a 2 s press on the front-left corner publishes exactly one bump",
      len(rig.bumps) == 1, f"{len(rig.bumps)} for {len(sw_set_sent)} SW lines with a set bit")
check("the bump Bool carries data=True",
      bool(rig.bumps) and rig.bumps[0].data is True)
check("the short rear-left click publishes exactly one bump_rear",
      len(rig.rear_bumps) == 1, f"{len(rig.rear_bumps)}")
check("the BUMP line names the front corner",
      rig.saw("BUMP #1: avant-gauche (front)"))
check("the BUMP line names the rear corner",
      rig.saw("BUMP #2: arriere-gauche (rear)"))
check("every SW line with a bit set is traced verbatim",
      rig.count("SW rx:") == len(sw_set_sent),
      f"{rig.count('SW rx:')} traced / {len(sw_set_sent)} sent")
check("the trace is the line itself",
      rig.saw("SW rx: 1 0 0 0") and rig.saw("SW rx: 0 1 0 0"))
check("the sonar keeps flowing on the same link (0.42 m, <= 5 Hz)",
      len(rig.sonars) >= 8 and all(abs(m.data - 0.42) < 1e-6 for m in rig.sonars),
      f"{len(rig.sonars)} publications")
check("the link was never lost", not rig.saw("LOST"))
rig.close()


# --- B. a line split across two reads ---------------------------------------

print("B. a SW line delivered in two chunks (buffering never eats a contact)")
rig = Rig()
rig.send("SONAR -1\r\nSW 0 0 1 ")
time.sleep(0.3)
rig.send("0\r\nSW 0 0 0 0\r\n")
time.sleep(0.3)
check("the halves are reassembled and the rear-right corner fires",
      len(rig.rear_bumps) == 1, f"{len(rig.rear_bumps)}")
check("it is logged as one whole line", rig.saw("SW rx: 0 0 1 0"))
check("nothing leaked onto the front stream", len(rig.bumps) == 0)
rig.close()


# --- C. a publish that raises must not kill the reader -----------------------

print("C. the reader survives a refused publish (LCM hiccup, Out not wired yet)")
rig = Rig(bump_boom=True)
rig.send("SW 1 0 0 0\r\n")
time.sleep(0.3)
check("the contact is logged even though the publish failed",
      rig.saw("BUMP #1: avant-gauche"))
check("the refusal is logged, named",
      rig.saw("NOT published") and rig.saw("RuntimeError"))
check("the serial link is NOT torn down", not rig.saw("LOST"))
time.sleep(CONTACT_COOLDOWN_S)
rig.send("SW 0 0 0 1\r\nSONAR 0.300\r\nSONAR 0.300\r\nSONAR 0.300\r\n")
time.sleep(0.4)
check("the next contact still reaches the loop (front-right)",
      rig.saw("BUMP #2: avant-droit"))
check("the sonar still flows after the failure", len(rig.sonars) >= 1,
      f"{len(rig.sonars)}")
rig.close()


# --- D. where a line ends up ------------------------------------------------

print("D. visibility: after start-up, a daemon run only keeps the run log")
err = io.StringIO()
saved_stderr, sys.stderr = sys.stderr, err
try:
    esp_sensors._log("witness-line")
finally:
    sys.stderr = saved_stderr
main_jsonl = RUN_LOG_DIR / "main.jsonl"
written = main_jsonl.read_text() if main_jsonl.exists() else ""
check("_log() lands in the run's main.jsonl (the only channel a daemon keeps)",
      "witness-line" in written, str(main_jsonl))
check("_log() also reaches stderr (foreground launches)", "witness-line" in err.getvalue())
check("a BUMP would land there too, from the reader thread",
      "BUMP #1: avant-gauche" in written)

print(f"{OK} OK, {KO} KO")
sys.exit(1 if KO else 0)

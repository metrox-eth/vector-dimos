"""FakeMossFirmware: the MOSS ESP32-S3 bench firmware's serial behaviour in Python.

A serial-shaped object (write / readline / in_waiting / close / is_open) that
runs the same state machine as `firmware/MOSS_ESP32S3/Control.h` in the moss
repo: OFF / READY / DRIVE / TEST, the 300 ms drive deadman, the 10 s arm
expiry, the +-25 % number check, the replies ("OK", "ERR not_armed",
"ERR invalid_command", "ERR stopped_busy"), the telemetry JSON line.

Time is the fake's own millisecond clock: benches call advance(ms) to make
the firmware's watchdogs fire deterministically. Telemetry lines are queued
on `status`, on every 200 ms of advanced time, and on emit_telemetry().

Also the bus behind MOSS_MOCK_BUS, so the real dimOS runtime can be driven
end to end on a machine with no motors wired.
"""
from __future__ import annotations

import json
import threading
import time
from collections import deque

MAX_PERCENT = 25
COMMAND_TIMEOUT_MS = 300
ARM_TIMEOUT_MS = 10000
TEST_MS = 1000
TELEMETRY_MS = 200

OFF, READY, DRIVE, TEST = 0, 1, 2, 3


class FakeMossFirmware:
    is_mock = True   # read by MossBaseAdapter when used as a port_factory

    def __init__(self, auto_time: bool = False):
        """auto_time=True: the clock follows the wall clock (MOSS_MOCK_BUS
        runs); False: it only moves with advance(ms) (benches)."""
        self._auto = auto_time
        self._t0 = time.monotonic()
        self._ms = 0
        self.mode = OFF
        self.left = 0
        self.right = 0
        self.since = 0
        self.reason = "boot"
        self.ticks = [0, 0]
        self.commands: list[str] = []     # every line received, in order
        self.sent: list[str] = []         # every line emitted, in order
        self._out: deque[bytes] = deque()
        self._lock = threading.Lock()
        self._last_telemetry = 0
        self.is_open = True
        self._queue_line("MOSS V0.3 bench firmware; USB-UART 115200; limit 25%; type status")

    # ── clock ──────────────────────────────────────────────────────────
    def now(self) -> int:
        if self._auto:
            self._ms = int((time.monotonic() - self._t0) * 1000)
        return self._ms

    def advance(self, ms: int) -> None:
        """Move the fake clock; runs the watchdogs and the 200 ms telemetry."""
        with self._lock:
            target = self._ms + int(ms)
            while self._ms < target:
                self._ms = min(target, self._ms + 10)
                self._poll()
                if self._ms - self._last_telemetry >= TELEMETRY_MS:
                    self._last_telemetry = self._ms
                    self._queue_line(self._telemetry_line())

    def _poll(self) -> None:
        age = self._ms - self.since
        if self.mode == DRIVE and age >= COMMAND_TIMEOUT_MS:
            self._stop("timeout")
        elif self.mode == READY and age >= ARM_TIMEOUT_MS:
            self._stop("arm_expired")
        elif self.mode == TEST and age >= TEST_MS:
            self._stop("test_done")

    def _stop(self, why: str) -> None:
        self.mode = OFF
        self.left = self.right = 0
        self.reason = why

    # ── serial side ────────────────────────────────────────────────────
    def _queue_line(self, text: str) -> None:
        self.sent.append(text)
        self._out.append((text + "\n").encode())

    @property
    def in_waiting(self) -> int:
        return sum(len(b) for b in self._out)

    def readline(self) -> bytes:
        for _ in range(4):   # ~20 ms of patience, like a serial timeout
            with self._lock:
                if self._out:
                    return self._out.popleft()
            time.sleep(0.005)
        return b""

    def write(self, data: bytes) -> int:
        text = data.decode(errors="replace")
        for line in text.split("\n"):
            line = line.replace("\r", "")
            if line == "" and not text.endswith("\n"):
                continue
            if line == "":
                continue
            with self._lock:
                self._handle(line)
        return len(data)

    def close(self) -> None:
        self.is_open = False

    def emit_telemetry(self) -> None:
        with self._lock:
            self._queue_line(self._telemetry_line())

    # ── Control.h ──────────────────────────────────────────────────────
    @staticmethod
    def _number(s: str):
        try:
            n = int(s, 10)
        except ValueError:
            return None
        if str(n) != s.lstrip("+") and not (s.startswith("-") and str(n) == s):
            # strtol accepts "+5"; keep it simple: accept what int() accepts
            pass
        if n < -MAX_PERCENT or n > MAX_PERCENT:
            return None
        return n

    def _handle(self, line: str) -> None:
        self.commands.append(line)
        now = self.now()
        self._poll()
        if any((ord(c) < 32 and c != "\t") or ord(c) > 126 for c in line) or len(line) > 95:
            self._stop("serial_invalid")
            return
        argv = line.split()
        if len(argv) > 4:
            self._stop("invalid")
            self._queue_line("ERR invalid_command")
            return
        if not argv:
            self._queue_line("OK")
            return
        if argv == ["stop"]:
            self._stop("stop")
            self._queue_line("OK")
            return
        if argv == ["status"]:
            self._queue_line(self._telemetry_line())
            return
        if argv == ["zero"]:
            if self.mode != OFF:
                self._stop("zero_requires_stop")
                self._queue_line("ERR stopped_busy")
                return
            self.ticks = [0, 0]
            self._queue_line("OK")
            return
        if argv == ["arm"]:
            if self.mode in (DRIVE, TEST):
                self._stop("arm_while_moving")
                self._queue_line("ERR stopped_busy")
                return
            self.mode = READY
            self.left = self.right = 0
            self.since = now
            self.reason = "armed"
            self._queue_line("OK")
            return
        if len(argv) == 3 and argv[0] == "drive":
            l, r = self._number(argv[1]), self._number(argv[2])
            if l is not None and r is not None:
                if self.mode not in (READY, DRIVE):
                    self._stop("not_armed")
                    self._queue_line("ERR not_armed")
                    return
                self.left, self.right = l, r
                self.mode = DRIVE
                self.since = now
                self.reason = "drive"
                self._queue_line("OK")
                return
        if len(argv) == 3 and argv[0] == "test" and argv[1] in ("left", "right"):
            p = self._number(argv[2])
            if p is not None:
                if self.mode != OFF:
                    self._stop("test_requires_stop")
                    self._queue_line("ERR stopped_busy")
                    return
                self.left = p if argv[1] == "left" else 0
                self.right = p if argv[1] == "right" else 0
                self.mode = TEST
                self.since = now
                self.reason = "test"
                self._queue_line("OK")
                return
        self._stop("invalid")
        self._queue_line("ERR invalid_command")

    def _telemetry_line(self) -> str:
        return json.dumps({
            "ms": self.now(), "mode": self.mode, "reason": self.reason,
            "target_pct": [self.left, self.right],
            "output_pct": [self.left, self.right],   # ramp not modelled
            "ticks": list(self.ticks), "invalid_edges": [0, 0],
            "ina_ok": False, "bus_mV": None, "shunt_uV": None, "current_mA": None,
        }, separators=(",", ":"))

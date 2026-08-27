"""Minimal RPLIDAR C1 reader over pyserial - standard scan, no third-party lib.

The C1 answers the classic SLAMTEC protocol: SCAN (A5 20) -> 7-byte descriptor
(A5 5A 05 00 00 40 81) -> endless 5-byte samples. Written 2026-08-23 after
rplidar-roboticia 0.9.5 failed on this unit ("Descriptor length mismatch") while
the raw protocol worked first try. Proven on the bench: ~3.8 k samples/s at
460800 baud, 10 Hz scans.

Sample (5 bytes):  b0: S (bit0), !S (bit1), quality (bits 2-7)
                   b1: C (bit0, always 1), angle_q6 low 7 bits
                   b2: angle_q6 high 8 bits      -> angle_deg = angle_q6 / 64
                   b3-b4: distance_q2 (u16 LE)   -> dist_mm  = distance_q2 / 4
"""
from __future__ import annotations

import time
from collections.abc import Iterator

import serial

SCAN = b"\xa5\x20"
STOP = b"\xa5\x25"
RESET = b"\xa5\x40"
GET_HEALTH = b"\xa5\x52"
GET_INFO = b"\xa5\x50"
SCAN_DESCRIPTOR = b"\xa5\x5a\x05\x00\x00\x40\x81"
SPINUP_GRACE_S = 10.0   # motor spin-up after a STOP before the first sample


class C1Scanner:
    def __init__(self, port: str, baudrate: int = 460800, timeout: float = 2.0):
        self.port, self.baudrate, self.timeout = port, baudrate, timeout
        self._ser: serial.Serial | None = None

    # ── lifecycle ──────────────────────────────────────────────────────
    def open(self) -> None:
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        self._ser.dtr = False
        self.stop()
        self._wake()

    def _wake(self) -> None:
        """Le C1 reste parfois engourdi apres un debranchement a chaud : sa
        liaison ne repond plus jusqu'a recevoir un deluge de requetes
        (constate et resolu le 25/08). On sonde GET_INFO ; si silence, on
        martele A5 50 et on re-sonde toutes les secondes.

        6 s : la vraie torpeur s'est toujours reglee en 6 (metrox 27/08 -
        l'essai a 30 s du 26/08 au soir n'a rien reveille de plus, il ne
        faisait que retarder le verdict)."""
        try:
            self.info()
            return
        except Exception:  # noqa: BLE001 - engourdi, on le reveille
            pass
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            burst_until = time.monotonic() + 1.0
            while time.monotonic() < burst_until:
                self._ser.write(bytes([0xA5, 0x50]) * 50)
                time.sleep(0.1)
            self._ser.reset_input_buffer()
            try:
                self.info()
                return
            except Exception:  # noqa: BLE001 - toujours engourdi, on insiste
                pass
        raise RuntimeError("lidar muet meme apres le reveil-rafale")

    def close(self) -> None:
        if self._ser is not None:
            try:
                self.stop()
            finally:
                self._ser.close()
                self._ser = None

    def __enter__(self) -> C1Scanner:
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def stop(self) -> None:
        assert self._ser is not None
        self._ser.write(STOP)
        time.sleep(0.05)
        self._ser.reset_input_buffer()

    def reset(self) -> None:
        assert self._ser is not None
        self._ser.write(RESET)
        time.sleep(2.0)
        self._ser.reset_input_buffer()

    # ── queries ────────────────────────────────────────────────────────
    def _query(self, cmd: bytes, payload_len: int) -> bytes:
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(cmd)
        head = self._ser.read(7)
        if len(head) != 7 or head[:2] != b"\xa5\x5a":
            raise RuntimeError(f"no descriptor for {cmd.hex()}: {head.hex()}")
        return self._ser.read(payload_len)

    def health(self) -> tuple[str, int]:
        p = self._query(GET_HEALTH, 3)
        status = {0: "Good", 1: "Warning", 2: "Error"}.get(p[0], str(p[0]))
        return status, p[1] | (p[2] << 8)

    def info(self) -> dict[str, object]:
        p = self._query(GET_INFO, 20)
        return {"model": p[0], "firmware": (p[2], p[1]), "hardware": p[3],
                "serial": p[4:20].hex().upper()}

    # ── scanning ───────────────────────────────────────────────────────
    def samples(self) -> Iterator[tuple[bool, int, float, float]]:
        """Yield (new_turn, quality, angle_deg, dist_m) forever; dist 0.0 = no return."""
        assert self._ser is not None
        s = self._ser
        s.reset_input_buffer()
        s.write(SCAN)
        head = s.read(7)
        if head != SCAN_DESCRIPTOR:
            raise RuntimeError(f"unexpected scan descriptor: {head.hex()}")
        buf = b""
        # After a STOP the C1 spins its motor down; on SCAN it answers the
        # descriptor at once but streams samples only once the motor is back
        # up - a few seconds. Give the first sample that grace, then the
        # normal timeout applies.
        first_deadline = time.monotonic() + SPINUP_GRACE_S
        while True:
            chunk = s.read(max(5, s.in_waiting or 0))
            if not chunk:
                if not buf and time.monotonic() < first_deadline:
                    continue
                raise RuntimeError("scan stream timed out")
            buf += chunk
            i = 0
            while len(buf) - i >= 5:
                b0, b1 = buf[i], buf[i + 1]
                start, nstart, check = b0 & 1, (b0 >> 1) & 1, b1 & 1
                if start == nstart or check != 1:      # lost sync: slide one byte
                    i += 1
                    continue
                quality = b0 >> 2
                angle = ((b1 >> 1) | (buf[i + 2] << 7)) / 64.0
                dist = (buf[i + 3] | (buf[i + 4] << 8)) / 4.0 / 1000.0
                yield bool(start), quality, angle, dist
                i += 5
            buf = buf[i:]

    def scans(self, min_len: int = 30) -> Iterator[list[tuple[int, float, float]]]:
        """Yield one list of (quality, angle_deg, dist_m) per revolution."""
        current: list[tuple[int, float, float]] = []
        for new_turn, q, ang, d in self.samples():
            if new_turn and len(current) >= min_len:   # the S flag marks a revolution
                yield current
                current = []
            current.append((q, ang, d))


class C1Lidar:
    """Drop-in for the subset of the rplidar-roboticia API that rplidar_c1.py
    uses: opens the port in __init__ (so a missing device raises right there),
    get_info/get_health, iter_scans() yielding (quality, angle_deg, distance_MM)
    per revolution, stop/stop_motor/disconnect. The C1 stops its motor on STOP,
    so stop_motor is a no-op kept for interface parity."""

    def __init__(self, port: str, baudrate: int = 460800, timeout: float = 2.0,
                 logger: object | None = None):
        self._s = C1Scanner(port, baudrate=baudrate, timeout=timeout)
        self._s.open()

    def get_info(self) -> dict[str, object]:
        return self._s.info()

    def get_health(self) -> tuple[str, int]:
        return self._s.health()

    def iter_scans(self, min_len: int = 30) -> Iterator[list[tuple[int, float, float]]]:
        for scan in self._s.scans(min_len=min_len):
            yield [(q, ang, d * 1000.0) for (q, ang, d) in scan]

    def stop(self) -> None:
        self._s.stop()

    def stop_motor(self) -> None:
        return None

    def disconnect(self) -> None:
        self._s.close()

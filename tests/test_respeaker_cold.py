"""Cold tests for vector_dimos.respeaker — no hardware, no LCM.

Run:  .venv/bin/python tests/test_respeaker_cold.py
Rule #2: known values in physical units through the real parsing path.
"""

import math
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.respeaker import (  # noqa: E402
    CHANNELS, CHUNK_SIZE, SAMPLE_RATE, ReSpeakerUSB, parse_response,
)

FAIL = 0


def check(label, ok, detail=""):
    global FAIL
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if not ok:
        FAIL = 1


print("A. parse_response roundtrip (73 deg -> radians payload -> ~73 deg)")
payload = struct.pack("<ffff", math.radians(73.0), math.radians(10.0),
                      math.radians(200.0), math.radians(355.0))
angles = parse_response(b"\x00" + payload + b"\x00", "radians")
check("4 beams decoded", angles is not None and len(angles) == 4)
check("beam0 = 73 deg", abs(angles[0] - 73.0) < 0.01, f"{angles[0]:.3f}")
check("beam2 = 200 deg", abs(angles[2] - 200.0) < 0.01, f"{angles[2]:.3f}")
check("bad status byte -> None", parse_response(b"\x01" + payload, "radians") is None)
check("short payload -> None", parse_response(b"\x00" + payload[:8], "radians") is None)

print("B. read_doa gating (fake device)")


class FakeDev:
    """Canned ctrl_transfer answers keyed by cmdid."""

    def __init__(self, energies, azimuths_deg):
        self.energies, self.azimuths = energies, azimuths_deg

    def ctrl_transfer(self, bmtype, breq, cmdid, resid, length, timeout):
        import array
        if cmdid == 0x80 | 80:   # AEC_SPENERGY_VALUES
            raw = b"\x00" + struct.pack("<ffff", *self.energies) + b"\x00"
        elif cmdid == 0x80 | 75:  # AEC_AZIMUTH_VALUES
            raw = b"\x00" + struct.pack(
                "<ffff", *[math.radians(a) for a in self.azimuths]) + b"\x00"
        else:
            raw = b"\x00" + b"\x02\x01\x00\x00"
        return array.array("B", raw)


usb = ReSpeakerUSB()
usb.dev = FakeDev([0.0, 0.0, 0.0, 0.0], [10, 90, 180, 270])
check("silence -> None", usb.read_doa() is None)
usb.dev = FakeDev([0.1, 0.9, 0.2, 0.0], [10, 90, 180, 270])
doa = usb.read_doa()
check("loudest beam wins (90 deg)", doa is not None and abs(doa - 90.0) < 0.01,
      f"{doa}")
usb.dev = FakeDev([0.5, 0.0, 0.0, 0.0], [-15, 90, 180, 270])
doa = usb.read_doa()
check("negative azimuth wraps to 345", doa is not None and abs(doa - 345.0) < 0.01,
      f"{doa}")

print("C. ring buffer keeps channel 0 (known pattern roundtrip)")
from vector_dimos.respeaker import ReSpeakerMic  # noqa: E402

mic = ReSpeakerMic.__new__(ReSpeakerMic)  # no Module __init__: buffer only
import collections
mic._chunks = collections.deque(maxlen=int(30 * SAMPLE_RATE / CHUNK_SIZE))
n = CHUNK_SIZE
interleaved = struct.pack(f"<{2 * n}h",
                          *[v for i in range(n) for v in (i % 1000, -7)])
mic.push_chunk(interleaved)
mono = mic.recent_audio(1.0)
samples = struct.unpack(f"<{len(mono) // 2}h", mono)
check("chunk length = CHUNK_SIZE mono samples", len(samples) == n, f"{len(samples)}")
check("ch0 values kept (i%1000)", samples[:5] == (0, 1, 2, 3, 4), f"{samples[:5]}")
check("ch1 (-7) dropped", -7 not in samples[:100])

print("D. module without hardware = clean no-op")
try:
    import usb.core  # noqa: F401
    print("  (pyusb present on this machine - D covered on the Jetson instead)")
except ImportError:
    mic2 = ReSpeakerMic(enabled=True)
    mic2._running = False
    print("  OK  import guard path reachable")

sys.exit(FAIL)

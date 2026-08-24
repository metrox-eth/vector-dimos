"""ReSpeaker XVF3800 microphone array: beamformed audio + direction of voice.

Ported from Sam's ROS2 ``respeaker_node.py`` (rover_nvme_archive). Two
transports on one USB device, exactly like the original:

- **pyaudio**: 2-channel 16 kHz capture. Channel 0 is the XMOS-beamformed
  voice. Audio stays *inside this module* (ring buffer) — the future STT
  stage consumes it in-process; only the direction crosses the LCM bus.
- **pyusb**: vendor control transfers -> speech energy per beam + azimuth.
  ``doa`` (Float32, degrees) is published only while speech is detected.

Hard-won hardware lesson kept from Sam: on a fresh Jetson boot the XVF3800
often initialises into a humming state; a DFU detach (= pressing RST) fixes
it, so we reset on start by default.

Angle frame: RAW device degrees for now. The mapping to the body frame
(after the 24/08 180-degree body flip) needs one live calibration: someone
speaks from a known side, we read the raw angle. Do not trust the sign
before that test.
"""

from __future__ import annotations

import collections
import logging
import math
import struct
import threading
import time
from typing import Any

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.std_msgs.Float32 import Float32

logger = logging.getLogger(__name__)

RESPEAKER_VID = 0x2886
RESPEAKER_PID = 0x001A
DFU_INTERFACE = 4       # "reSpeaker DFU Factory" interface
DFU_DETACH = 0
XMOS_TIMEOUT = 100000   # microseconds, Sam's value

SAMPLE_RATE = 16000
CHANNELS = 2
CHUNK_SIZE = 1600       # 100 ms at 16 kHz

# (resid, cmdid, length, type) - subset of Sam's PARAMETERS table
PARAMETERS = {
    "VERSION": (48, 0, 4, "uint8"),
    "AEC_AZIMUTH_VALUES": (33, 75, 16 + 1, "radians"),
    "AEC_SPENERGY_VALUES": (33, 80, 16 + 1, "float"),
}


class ReSpeakerUSB:
    """DOA reader over vendor control transfers (Sam's ReSpeakerUSB, no ROS)."""

    def __init__(self, vid: int = RESPEAKER_VID, pid: int = RESPEAKER_PID) -> None:
        self.vid, self.pid = vid, pid
        self.dev: Any = None

    def open(self) -> None:
        import usb.core
        self.dev = usb.core.find(idVendor=self.vid, idProduct=self.pid)
        if self.dev is None:
            raise RuntimeError(f"ReSpeaker not found (VID={hex(self.vid)}, PID={hex(self.pid)})")

    def read(self, name: str):
        if self.dev is None or name not in PARAMETERS:
            return None
        import usb.core
        import usb.util
        resid, cmdid, length, data_type = PARAMETERS[name]
        try:
            response = self.dev.ctrl_transfer(
                usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
                0, 0x80 | cmdid, resid, length, XMOS_TIMEOUT)
        except usb.core.USBError:
            return None
        return parse_response(bytes(response), data_type)

    def read_doa(self):
        """Angle in degrees (0-360) of the loudest speech beam, or None."""
        spenergy = self.read("AEC_SPENERGY_VALUES")
        if not spenergy:
            return None
        max_energy = max(spenergy)
        if max_energy <= 0:
            return None
        azimuth = self.read("AEC_AZIMUTH_VALUES")
        if azimuth is not None and len(azimuth) >= 4:
            return azimuth[spenergy.index(max_energy)] % 360.0
        return None

    def close(self) -> None:
        if self.dev is not None:
            import usb.util
            usb.util.dispose_resources(self.dev)
            self.dev = None


def parse_response(raw: bytes, data_type: str):
    """Decode one XVF3800 control-transfer response (status byte + payload)."""
    if len(raw) < 1 or raw[0] != 0x00:
        return None
    payload = raw[1:]
    if data_type == "uint8":
        return list(payload)
    if data_type in ("radians", "float") and len(payload) >= 16:
        floats = struct.unpack("<ffff", payload[:16])
        if data_type == "radians":
            return [math.degrees(f) for f in floats]
        return list(floats)
    return None


def dfu_reset() -> bool:
    """Firmware reset (Sam's boot fix: humming mics on cold boot)."""
    import usb.core
    import usb.util
    dev = usb.core.find(idVendor=RESPEAKER_VID, idProduct=RESPEAKER_PID)
    if dev is None:
        logger.warning("respeaker: device not found for DFU reset")
        return False
    try:
        dev.ctrl_transfer(0x21, DFU_DETACH, 1000, DFU_INTERFACE, None)
        usb.util.dispose_resources(dev)
    except usb.core.USBError as e:
        logger.warning("respeaker: DFU reset failed: %s", e)
        return False
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        time.sleep(0.2)
        dev = usb.core.find(idVendor=RESPEAKER_VID, idProduct=RESPEAKER_PID)
        if dev is not None:
            usb.util.dispose_resources(dev)
            logger.info("respeaker: re-enumerated after DFU reset")
            time.sleep(2.0)  # let ALSA settle before opening the stream
            return True
    logger.error("respeaker: did not re-enumerate after DFU reset")
    return False


class ReSpeakerMic(Module):
    """Outs: ``doa`` (Float32, RAW device degrees, only while speech).

    Audio ring buffer (channel 0, int16 mono 16 kHz) is kept in-process for
    the future STT stage: ``recent_audio(seconds)``.
    """

    doa: Out[Float32]

    def __init__(self, doa_rate: float = 10.0, reset_on_start: bool = True,
                 enable_audio: bool = True, buffer_seconds: float = 30.0,
                 enabled: bool = True, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.doa_rate = doa_rate
        self.reset_on_start = reset_on_start
        self.enable_audio = enable_audio
        self.enabled = enabled
        self._chunks: collections.deque[bytes] = collections.deque(
            maxlen=max(1, int(buffer_seconds * SAMPLE_RATE / CHUNK_SIZE)))
        self._usb: ReSpeakerUSB | None = None
        self._pa = None
        self._stream = None
        self._running = False
        self.last_doa: float | None = None

    # ---- plumbing -----------------------------------------------------------
    @rpc
    def start(self) -> None:
        super().start()
        if not self.enabled:
            logger.info("respeaker disabled by config")
            return
        try:
            import usb.core  # noqa: F401
        except ImportError:
            logger.warning("respeaker: pyusb missing, module inactive")
            return
        if self.reset_on_start:
            dfu_reset()
        try:
            self._usb = ReSpeakerUSB()
            self._usb.open()
            version = self._usb.read("VERSION")
            logger.info("respeaker: DOA open (firmware %s)", version)
        except Exception:  # noqa: BLE001 - no mic, robot still runs
            logger.exception("respeaker: USB DOA unavailable, module inactive")
            self._usb = None
            return
        self._running = True
        threading.Thread(target=self._doa_loop, daemon=True).start()
        if self.enable_audio:
            threading.Thread(target=self._audio_loop, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running = False
        time.sleep(0.15)
        if self._stream is not None:
            try:
                self._stream.stop_stream()
                self._stream.close()
            except Exception:  # noqa: BLE001
                pass
            self._stream = None
        if self._pa is not None:
            try:
                self._pa.terminate()
            except Exception:  # noqa: BLE001
                pass
            self._pa = None
        if self._usb is not None:
            self._usb.close()
            self._usb = None
        super().stop()

    # ---- capture ------------------------------------------------------------
    def _find_input_index(self):
        for i in range(self._pa.get_device_count()):
            info = self._pa.get_device_info_by_index(i)
            name = str(info.get("name", "")).lower()
            if ("respeaker" in name or "xvf3800" in name or "seeed" in name) \
                    and int(info.get("maxInputChannels", 0)) >= CHANNELS:
                return i
        return None

    def _audio_loop(self) -> None:
        try:
            import pyaudio
        except ImportError:
            logger.warning("respeaker: pyaudio missing, audio capture off (DOA still on)")
            return
        try:
            self._pa = pyaudio.PyAudio()
            self._stream = self._pa.open(
                format=pyaudio.paInt16, channels=CHANNELS, rate=SAMPLE_RATE,
                input=True, input_device_index=self._find_input_index(),
                frames_per_buffer=CHUNK_SIZE)
        except Exception:  # noqa: BLE001
            logger.exception("respeaker: audio stream failed, DOA still on")
            return
        logger.info("respeaker: audio capture on (16 kHz, ch0 beamformed)")
        while self._running:
            try:
                data = self._stream.read(CHUNK_SIZE, exception_on_overflow=False)
            except Exception:  # noqa: BLE001
                if self._running:
                    logger.exception("respeaker: audio read error, capture stopped")
                return
            self.push_chunk(data)

    def push_chunk(self, interleaved: bytes) -> None:
        """Keep channel 0 (beamformed voice) of one interleaved 2-ch chunk."""
        samples = memoryview(interleaved).cast("h")
        self._chunks.append(struct.pack(f"<{len(samples) // CHANNELS}h",
                                        *samples[::CHANNELS]))

    def recent_audio(self, seconds: float) -> bytes:
        """Last N seconds of mono int16 16 kHz voice, oldest first."""
        n = max(1, int(seconds * SAMPLE_RATE / CHUNK_SIZE))
        return b"".join(list(self._chunks)[-n:])

    # ---- DOA ----------------------------------------------------------------
    def _doa_loop(self) -> None:
        period = 1.0 / self.doa_rate
        while self._running:
            t0 = time.monotonic()
            try:
                angle = self._usb.read_doa() if self._usb else None
            except Exception:  # noqa: BLE001
                angle = None
            if angle is not None:
                self.last_doa = angle
                self.doa.publish(Float32(data=float(angle)))
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

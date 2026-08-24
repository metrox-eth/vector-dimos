"""ReSpeaker XVF3800 microphone array: beamformed audio + direction of voice.

Ported from Sam's ROS2 ``respeaker_node.py`` (rover_nvme_archive). Two
transports on one USB device, exactly like the original:

- **arecord subprocess**: 2-channel 16 kHz raw capture straight from ALSA
  (``plughw``). PortAudio/Pulse hung forever inside its constructor when
  running in a dimos forkserver worker (24/08: thread stuck in pipe_read,
  capture fd held, zero log lines) - a child process reading raw PCM cannot
  hang us. Channel 0 is the XMOS-beamformed voice; audio stays *inside this
  module* (ring buffer), only the direction and transcripts cross the bus.
- **pyusb**: vendor control transfers -> speech energy per beam + azimuth.
  ``doa`` (Float32, degrees) is published only while speech is detected.
- **STT stage** (in-process): energy VAD segments utterances from the ch0
  stream, faster-whisper (local, int8) transcribes them, ``transcript``
  (String) is published. No cloud - Sam's chain used ElevenLabs, this one
  keeps the understanding on the robot.

Hard-won hardware lesson kept from Sam: on a fresh Jetson boot the XVF3800
often initialises into a humming state; a DFU detach (= pressing RST) fixes
it, so we reset on start by default.

Acoustics lesson (metrox, live test 24/08, front-right at 3 m): the DOA is
tight (+/-3 deg) when the speaker FACES the rover; speaking away from it
yields wall reflections (readings drifted 273 -> 258, one 175 outlier).
Fine in practice: people face a robot when talking to it.

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
from dimos.msgs.std_msgs.String import String

logger = logging.getLogger(__name__)


def _log(msg: str) -> None:
    """Worker INFO logs are swallowed by default logging config: print to
    stderr, which dimos captures in the launch log (the ALSA lines prove it)."""
    import sys
    print(f"[respeaker] {msg}", file=sys.stderr, flush=True)

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


def _read_file(path: str) -> str | None:
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def find_respeaker_card(cards_text: str) -> str | None:
    """Card number of the XVF3800 in /proc/asound/cards content, or None."""
    for line in cards_text.splitlines():
        if "XVF3800" in line or "respeaker" in line.lower():
            parts = line.strip().split()
            if parts and parts[0].isdigit():
                return parts[0]
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
    transcript: Out[String]

    def __init__(self, doa_rate: float = 10.0, reset_on_start: bool = True,
                 enable_audio: bool = True, buffer_seconds: float = 30.0,
                 enabled: bool = True, stt: bool = True,
                 stt_model: str = "base", stt_language: str | None = None,
                 **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.doa_rate = doa_rate
        self.reset_on_start = reset_on_start
        self.enable_audio = enable_audio
        self.enabled = enabled
        self.stt_enabled = stt
        self.stt_model = stt_model
        self.stt_language = stt_language
        self._vad = EnergyVAD()
        self._utterances: "collections.deque[bytes]" = collections.deque(maxlen=4)
        self._utt_event = threading.Event()
        self.last_transcript: str | None = None
        self._chunks: collections.deque[bytes] = collections.deque(
            maxlen=max(1, int(buffer_seconds * SAMPLE_RATE / CHUNK_SIZE)))
        self._usb: ReSpeakerUSB | None = None
        self._proc = None
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
            _log(f"DOA open (firmware {version})")
        except Exception:  # noqa: BLE001 - no mic, robot still runs
            logger.exception("respeaker: USB DOA unavailable, module inactive")
            self._usb = None
            return
        self._running = True
        threading.Thread(target=self._doa_loop, daemon=True).start()
        if self.enable_audio:
            threading.Thread(target=self._audio_loop, daemon=True).start()
            if self.stt_enabled:
                threading.Thread(target=self._stt_loop, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running = False
        time.sleep(0.15)
        if self._proc is not None:
            try:
                self._proc.terminate()
            except Exception:  # noqa: BLE001
                pass
            self._proc = None
        if self._usb is not None:
            self._usb.close()
            self._usb = None
        super().stop()

    # ---- capture (arecord subprocess, raw ALSA) -----------------------------
    def _audio_loop(self) -> None:
        import subprocess
        card = None
        for attempt in range(15):   # post-DFU, ALSA re-registers the card slowly
            card = find_respeaker_card(_read_file("/proc/asound/cards") or "")
            if card is not None:
                break
            time.sleep(1.0)
        if card is None:
            _log("no reSpeaker ALSA card after 15 s - no capture, DOA still on")
            return
        dev = f"plughw:{card},0"
        chunk_bytes = CHUNK_SIZE * CHANNELS * 2
        cmd = ["arecord", "-D", dev, "-f", "S16_LE", "-r", str(SAMPLE_RATE),
               "-c", str(CHANNELS), "-t", "raw", "-q"]
        restarts = 0
        while self._running and restarts < 20:
            try:
                self._proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                              stderr=subprocess.DEVNULL)
            except OSError as e:
                _log(f"arecord launch failed: {e}")
                return
            _log(f"audio capture on ({dev} via arecord, 16 kHz)")
            n, level_max = 0, 0.0
            run_started = time.monotonic()
            while self._running:
                buf = b""
                while self._running and len(buf) < chunk_bytes:
                    r = self._proc.stdout.read(chunk_bytes - len(buf))
                    if not r:
                        break
                    buf += r
                if len(buf) < chunk_bytes:
                    break   # arecord died (DFU replug, unplug...)
                self.push_chunk(buf)
                n += 1
                level_max = max(level_max, self._vad._floor)
                if n % 300 == 0:   # every 30 s: proof of life + noise floor
                    _log(f"capture alive: {n} chunks, noise floor {self._vad._floor:.0f}")
            if not self._running:
                return
            if time.monotonic() - run_started > 60.0:
                restarts = 0   # a long stable run forgives earlier flapping
            restarts += 1
            _log(f"arecord ended, restart {restarts}/20")
            time.sleep(2.0)

    def push_chunk(self, interleaved: bytes) -> None:
        """Keep channel 0 (beamformed voice) of one interleaved 2-ch chunk."""
        samples = memoryview(interleaved).cast("h")
        mono = struct.pack(f"<{len(samples) // CHANNELS}h", *samples[::CHANNELS])
        self._chunks.append(mono)
        if self.stt_enabled:
            utterance = self._vad.feed(mono)
            if utterance is not None:
                _log(f"utterance captured: {len(utterance) / 2 / SAMPLE_RATE:.1f} s")
                self._utterances.append(utterance)
                self._utt_event.set()

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


    # ---- STT (local faster-whisper) -----------------------------------------
    def _stt_loop(self) -> None:
        try:
            from faster_whisper import WhisperModel
        except ImportError:
            logger.warning("respeaker: faster-whisper missing, STT off")
            return
        try:
            model = WhisperModel(self.stt_model, device="cpu", compute_type="int8")
        except Exception as e:  # noqa: BLE001
            _log(f"whisper model load FAILED: {e} - STT off")
            return
        _log(f"STT on (faster-whisper {self.stt_model} int8, lang={self.stt_language or 'auto'})")
        import numpy as np
        while self._running:
            if not self._utt_event.wait(timeout=0.5):
                continue
            self._utt_event.clear()
            while self._utterances:
                raw = self._utterances.popleft()
                audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0
                t0 = time.monotonic()
                try:
                    segments, _info = model.transcribe(
                        audio, language=self.stt_language, beam_size=1)
                    text = " ".join(seg.text.strip() for seg in segments
                                    if seg.no_speech_prob < 0.6).strip()
                except Exception as e:  # noqa: BLE001
                    _log(f"transcribe FAILED: {e}")
                    continue
                dt = time.monotonic() - t0
                if not text:
                    continue
                self.last_transcript = text
                _log(f"heard {text!r} ({len(audio) / SAMPLE_RATE:.1f} s audio, {dt:.1f} s stt, doa {self.last_doa})")
                self.transcript.publish(String(data=text))


class EnergyVAD:
    """Chunk-level energy gate turning the 100 ms stream into utterances.

    Known values through the real path (cold-tested): RMS of int16 chunks;
    speech = RMS above max(abs_threshold, 4x rolling noise floor); an
    utterance starts after 2 speech chunks, ends after 6 silent ones
    (0.6 s), keeps 3 chunks of pre-roll, minimum 4 speech chunks (0.4 s),
    hard cap 10 s.
    """

    ABS_THRESHOLD = 300.0
    FLOOR_FACTOR = 4.0
    START_CHUNKS = 2
    END_SILENCE = 6
    PREROLL = 3
    MIN_SPEECH = 4
    MAX_CHUNKS = 100

    def __init__(self) -> None:
        self._floor = 100.0
        self._preroll: collections.deque[bytes] = collections.deque(maxlen=self.PREROLL)
        self._current: list[bytes] = []
        self._speech_run = 0
        self._silence_run = 0
        self._speech_total = 0
        self._in_utterance = False

    @staticmethod
    def rms(chunk: bytes) -> float:
        samples = memoryview(chunk).cast("h")
        if not len(samples):
            return 0.0
        return math.sqrt(sum(v * v for v in samples) / len(samples))

    def feed(self, chunk: bytes) -> bytes | None:
        """Feed one mono chunk; returns a finished utterance or None."""
        level = self.rms(chunk)
        threshold = max(self.ABS_THRESHOLD, self._floor * self.FLOOR_FACTOR)
        is_speech = level > threshold
        if not is_speech:
            self._floor = 0.95 * self._floor + 0.05 * max(level, 1.0)

        if not self._in_utterance:
            self._preroll.append(chunk)
            self._speech_run = self._speech_run + 1 if is_speech else 0
            if self._speech_run >= self.START_CHUNKS:
                self._in_utterance = True
                self._current = list(self._preroll)
                self._speech_total = self._speech_run
                self._silence_run = 0
            return None

        self._current.append(chunk)
        if is_speech:
            self._speech_total += 1
            self._silence_run = 0
        else:
            self._silence_run += 1
        if self._silence_run >= self.END_SILENCE or len(self._current) >= self.MAX_CHUNKS:
            utterance = b"".join(self._current)
            speech_ok = self._speech_total >= self.MIN_SPEECH
            self._in_utterance = False
            self._current = []
            self._speech_run = 0
            self._preroll.clear()
            return utterance if speech_ok else None
        return None

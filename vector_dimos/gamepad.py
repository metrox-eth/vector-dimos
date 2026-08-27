"""PlayStation-style gamepad teleop for VECTOR - Twist out, pygame.joystick.

dimOS ships keyboard/phone/quest teleop but no gamepad; this module fills
the gap for VECTOR. Publishes into ``tele_cmd_vel`` (wire through dimOS's
MovementManager so navigation and teleop coexist), or remap to ``cmd_vel``
for direct drive.

Mapping (standard dual-stick):
    left stick Y  -> vx (forward)     left stick X  -> vy (strafe)
    right stick X -> wz (rotation)    R2 held       -> boost

Runs headless: the Jetson has no display, so SDL is pinned to its dummy
video/audio drivers below. The pad is hot-pluggable - the module waits for
one instead of giving up at startup, and goes back to waiting if it is
unplugged, so ``dimos run vector-dimos.gamepad`` can be started before the
receiver is in.
"""
from __future__ import annotations

from dataclasses import dataclass
import os
import threading
import time
from typing import Any

# SDL picks its drivers at import time, so this has to run before "import
# pygame" (which happens lazily in the worker thread below). setdefault, not
# assignment: an operator who exports SDL_VIDEODRIVER=x11 keeps their display.
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")  # no banner in daemon logs

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_LINEAR_SPEED = 0.6    # m/s at full stick
DEFAULT_ANGULAR_SPEED = 1.2   # rad/s at full stick
DEFAULT_DEADZONE = 0.32
DEFAULT_RATE_HZ = 50.0
DEFAULT_BOOST = 2.0
RESCAN_PERIOD_S = 2.0         # how often we re-scan SDL for a pad

# SDL axis indices on a standard dual-stick pad (DS4/DS5, Xbox, 8BitDo).
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 2   # MEASURED 27/08 15h21: right-stick X sweeps axis 2 full travel on this pad/mode (axis 3 is its vertical leakage - the random rotation)
AXIS_R2 = 5


@dataclass(frozen=True)
class TeleopConfig:
    """Speed envelope for :func:`axes_to_twist` - no pygame, no module."""

    linear_speed: float = DEFAULT_LINEAR_SPEED
    angular_speed: float = DEFAULT_ANGULAR_SPEED
    deadzone: float = DEFAULT_DEADZONE
    boost_multiplier: float = DEFAULT_BOOST


def apply_deadzone(value: float, deadzone: float) -> float:
    """Hard deadzone: below the threshold the axis reads zero.

    No rescaling of the remaining travel - a stick just past the deadzone
    gives a small command, which is what you want for fine positioning.
    """
    return 0.0 if abs(value) < deadzone else value


# Absolute ceilings, applied LAST, regardless of config or boost. Born of the
# 27/08 13h42 runaway: a fresh xboxdrv delivered full-deflection uninitialised
# axes straight into tele_cmd_vel (priority channel, no guard anywhere) and
# the rover left at 1200 W. No teleop output may ever exceed these.
CLAMP_LINEAR_MS = 0.45
SLEW_LINEAR_MS2 = 0.6     # max change of linear command per second: full stick
                          # reaches the ceiling in ~0.75 s instead of one step -
                          # the mecanum wheelspin fix, on top of the 400 ms ZLAC
                          # ramps the adapter writes (the era tuning)
CLAMP_ANGULAR_RADS = 0.8
# Mecanum wheels ADD the commands: rim speed = |vx| + |vy| + (Lx+Ly)|wz|.
# The owner felt it on the first lap (27/08 17h08): stick + rotation mixed and
# "tout a coup il va tres vite" - each axis was under its own clamp while the
# wheels ran near double. Envelope = the fastest single-stick feel (0.45).
MECANUM_LEVER_M = 0.50    # Lx+Ly: half wheelbase 0.27 + half track 0.23 (54x46 cm chassis)
WHEEL_ENVELOPE_MS = 0.45  # max rim speed however the sticks are mixed
DEADMAN_BUTTON = 7        # MEASURED 27/08 14h56: the owner pressed his chosen deadman 21x -> index 7 (shanwan pad, Android mode). Held = commands allowed; released = zeros, always
# NOTE (27/08 17h20, owner: "il faut que ce soit proportionne"): when this
# pad's radio dies mid-drive (sleep, battery - lived on the 17h05 lap), the
# rover STOPS cleanly - observed behaviour, no radio watchdog needed. Revisit
# ONLY if a radio death ever leaves a non-zero command running.


def slew(prev: float, target: float, dt: float) -> float:
    """Rate-limit a linear command - pure, cold-testable."""
    step = SLEW_LINEAR_MS2 * dt
    return max(prev - step, min(prev + step, target))


def clamp_twist(vx: float, vy: float, wz: float) -> tuple[float, float, float]:
    """The last gate before the bus - pure, cold-testable. Per-axis clamps,
    then the mecanum wheel envelope: translation and rotation ADD at the rim,
    so a mixed command is scaled down proportionally (the feel is preserved,
    the top speed is not exceeded)."""
    lim = CLAMP_LINEAR_MS
    vx = max(-lim, min(lim, vx))
    vy = max(-lim, min(lim, vy))
    wz = max(-CLAMP_ANGULAR_RADS, min(CLAMP_ANGULAR_RADS, wz))
    rim = abs(vx) + abs(vy) + MECANUM_LEVER_M * abs(wz)
    if rim > WHEEL_ENVELOPE_MS:
        k = WHEEL_ENVELOPE_MS / rim
        vx, vy, wz = vx * k, vy * k, wz * k
    return (vx, vy, wz)


def axes_neutral(ax0: float, ax1: float, ax3: float, deadzone: float) -> bool:
    """True when every drive axis rests inside the deadzone - the trust gate:
    a pad (or a userspace driver) must be SEEN at neutral once before a single
    non-zero command is believed. Uninitialised axes never pass this."""
    return abs(ax0) < deadzone and abs(ax1) < deadzone and abs(ax3) < deadzone


def axes_to_twist(ax0: float, ax1: float, ax3: float, ax5: float,
                  cfg: TeleopConfig) -> tuple[float, float, float]:
    """Raw SDL axis values -> (vx m/s, vy m/s, wz rad/s).

    Pure function - the whole mapping lives here so it is testable without a
    pad, a display, or pygame.

    SDL reports stick up and stick left as NEGATIVE, while the robot frame is
    x forward / y left / wz counter-clockwise, hence the three sign flips:
        ax0 left stick X  -> vy   (stick left  = -1 -> strafe left,  +y)
        ax1 left stick Y  -> vx   (stick up    = -1 -> forward,      +x)
        ax3 right stick X -> wz   (stick right = +1 -> clockwise,    -wz)
        ax5 R2            -> boost when > 0 (trigger rests at -1)

    Boost multiplies the linear axes only: rotation stays at its capped rate,
    because a spin fast enough to be useful is already fast enough to hurt.
    """
    boost = cfg.boost_multiplier if ax5 > 0.0 else 1.0
    vx = -apply_deadzone(ax1, cfg.deadzone) * cfg.linear_speed * boost
    vy = -apply_deadzone(ax0, cfg.deadzone) * cfg.linear_speed * boost
    wz = -apply_deadzone(ax3, cfg.deadzone) * cfg.angular_speed
    return clamp_twist(vx, vy, wz)


class GamepadTeleop(Module):
    """pygame.joystick teleop. Outputs Twist on tele_cmd_vel."""

    dedicated_worker = True

    tele_cmd_vel: Out[Twist]

    def __init__(self, linear_speed: float = DEFAULT_LINEAR_SPEED,
                 angular_speed: float = DEFAULT_ANGULAR_SPEED,
                 deadzone: float = DEFAULT_DEADZONE,
                 rate_hz: float = DEFAULT_RATE_HZ,
                 boost_multiplier: float = DEFAULT_BOOST,
                 joystick_index: int = 0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.cfg = TeleopConfig(linear_speed=linear_speed,
                                angular_speed=angular_speed,
                                deadzone=deadzone,
                                boost_multiplier=boost_multiplier)
        self.rate_hz = rate_hz
        self.joystick_index = joystick_index
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # @rpc is not decoration for its own sake: Module.start/stop carry it,
    # and an override that drops it falls out of the class's rpcs table.
    # dimOS then proxies the call by pickling the module across the worker
    # pipe, which dies on our threading.Event ('cannot pickle _thread.lock').
    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker,
                                        name="vector-gamepad", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        super().stop()

    def _publish(self, vx: float, vy: float, wz: float) -> None:
        self.tele_cmd_vel.publish(Twist(
            linear=Vector3(x=vx, y=vy, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=wz)))

    def _acquire(self, pygame: Any) -> Any:
        """Re-scan SDL for the pad. Returns an initialised joystick or None."""
        try:
            pygame.event.pump()          # let SDL process hotplug events
            pygame.joystick.quit()       # full re-scan: SDL caches the device
            pygame.joystick.init()       # list from the last init()
            if pygame.joystick.get_count() <= self.joystick_index:
                return None
            pad = pygame.joystick.Joystick(self.joystick_index)
            pad.init()
            return pad
        except pygame.error as exc:
            logger.debug("Gamepad scan failed: %s", exc)
            return None

    # Named _worker, NOT _loop: dimOS's Module keeps its asyncio event
    # loop in self._loop, which would shadow the method and make the
    # thread target the event loop object.
    def _worker(self) -> None:
        try:
            import pygame
        except ImportError:
            logger.error("pygame is not installed - gamepad teleop is off "
                         "(pip install 'vector-dimos[gamepad]')")
            return

        pygame.init()  # dummy video/audio drivers: works with no display
        pygame.joystick.init()
        period = 1.0 / self.rate_hz
        pad = None
        trusted = False            # neutral-first trust gate (13h42 runaway)
        prev_vx = prev_vy = 0.0    # slew-limiter state
        waiting_logged = False
        try:
            while not self._stop_event.is_set():
                if pad is None:
                    pad = self._acquire(pygame)
                    if pad is None:
                        if not waiting_logged:
                            logger.info("waiting for gamepad (index %d)",
                                        self.joystick_index)
                            waiting_logged = True
                        self._stop_event.wait(RESCAN_PERIOD_S)
                        continue
                    waiting_logged = False
                    trusted = False   # every (re)connection re-earns trust at neutral
                    logger.info("Gamepad connected: %s - waiting to see the sticks at NEUTRAL "
                                "before trusting a single command", pad.get_name())
                try:
                    pygame.event.pump()
                    axes = (_axis(pad, AXIS_LEFT_X), _axis(pad, AXIS_LEFT_Y),
                            _axis(pad, AXIS_RIGHT_X), _axis(pad, AXIS_R2))
                    deadman = bool(pad.get_button(DEADMAN_BUTTON)) if pad.get_numbuttons() > DEADMAN_BUTTON else False
                    if pygame.joystick.get_count() <= self.joystick_index:
                        raise pygame.error("joystick disappeared")
                except pygame.error as exc:
                    pad = None
                    trusted = False
                    self._publish(0.0, 0.0, 0.0)  # one zero Twist, then wait
                    logger.warning("Gamepad lost (%s) - back to waiting", exc)
                    continue
                # TRUST GATE (the 13h42 runaway): until the sticks have been
                # SEEN at neutral once, this pad's words are worth nothing.
                if not trusted:
                    if axes_neutral(axes[0], axes[1], axes[2], self.cfg.deadzone):
                        trusted = True
                        logger.info("Gamepad axes seen at neutral - commands now trusted "
                                    "(hold the deadman button to drive)")
                    else:
                        self._publish(0.0, 0.0, 0.0)
                        time.sleep(period)
                        continue
                # DEADMAN: no held button, no motion - ever.
                if not deadman:
                    self._publish(0.0, 0.0, 0.0)
                    time.sleep(period)
                    continue
                vx, vy, wz = axes_to_twist(*axes, self.cfg)
                vx = slew(prev_vx, vx, period)
                vy = slew(prev_vy, vy, period)
                prev_vx, prev_vy = vx, vy
                # envelope AFTER the slew too: while vx decays and wz is
                # instant, the mix could transiently exceed the rim ceiling
                vx, vy, wz = clamp_twist(vx, vy, wz)
                self._publish(vx, vy, wz)
                time.sleep(period)
        finally:
            try:
                self._publish(0.0, 0.0, 0.0)
            except Exception:  # stream already torn down: nothing to do
                logger.debug("no zero Twist on shutdown", exc_info=True)
            pygame.joystick.quit()
            pygame.quit()


def _axis(pad: Any, index: int) -> float:
    """Axis value, or 0.0 when this pad has no such axis.

    Pads differ (some report 4 axes, not 6). A missing R2 must read as "no
    boost", not throw the loop into the reconnect path on every tick.
    """
    if index >= pad.get_numaxes():
        return 0.0
    return float(pad.get_axis(index))

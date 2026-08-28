"""Cold bench: gamepad mapping (no pygame) + the headless hot-plug cycle.

Sections:
  A. axes_to_twist - known axis values -> known m/s and rad/s. Pure, no pygame.
  B. the module's hot-plug loop driven by a FAKE pygame: absent pad -> pad
     plugged in -> deadman released and pressed again -> pad pulled out ->
     pad back -> at 50 Hz, the 0.5 s brake then SILENCE. Still no pygame needed.
  C. real pygame with no display, if it is installed: the dummy SDL drivers
     have to make init()/event.pump() work on a headless Jetson.
  F. the 27/08 safety nets: trust gate, absolute ceilings, wheel envelope.
  G. the slew ramp.
"""
import logging
import subprocess
import sys
import time
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

import os

from vector_dimos import gamepad as gp
from vector_dimos.gamepad import GamepadTeleop, TeleopConfig, axes_to_twist

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and bool(cond)


def close(a, b, tol=1e-12):
    return abs(a - b) <= tol


def twist_close(got, expected, tol=1e-12):
    return all(close(g, e, tol) for g, e in zip(got, expected))


def wait_for(predicate, timeout=6.0, tick=0.02):
    """Poll until true or timeout. The waiting loop re-scans every 2 s."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(tick)
    return bool(predicate())


# --- A. pure mapping: known axes -> known physical units ------------------
# Defaults: 0.6 m/s and 1.2 rad/s at full stick, deadzone 0.12, boost x2.
CFG = TeleopConfig()
print("A. axes_to_twist (no pygame)")

check("pygame" not in sys.modules, "mapping is testable with pygame unimported")

# The module pins SDL with setdefault, so this cannot be asserted in THIS
# process: a shell that exported SDL_VIDEODRIVER=x11 would fail it, and
# honouring that export is the documented intent. Import it in a subprocess
# with a controlled environment instead - hermetic either way.
SDL_PROBE = ("import os, vector_dimos.gamepad; "
             "print(os.environ['SDL_VIDEODRIVER'], "
             "os.environ['SDL_AUDIODRIVER'])")


def sdl_after_import(**preset):
    """SDL_VIDEODRIVER/SDL_AUDIODRIVER once the module is imported fresh."""
    env = {k: v for k, v in os.environ.items()
           if k not in ("SDL_VIDEODRIVER", "SDL_AUDIODRIVER")}
    env["PYTHONPATH"] = str(REPO_ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    env.update(preset)
    done = subprocess.run([sys.executable, "-c", SDL_PROBE], env=env,
                          capture_output=True, text=True)
    if done.returncode != 0:
        print(done.stderr.strip()[-400:])
    return done.stdout.strip()


unset = sdl_after_import()
check(unset == "dummy dummy",
      f"with SDL unset, importing the module pins the dummy (headless) "
      f"drivers: {unset!r}")
exported = sdl_after_import(SDL_VIDEODRIVER="x11")
check(exported == "x11 dummy",
      f"an operator who exported SDL_VIDEODRIVER=x11 keeps their display "
      f"(setdefault, not assignment): {exported!r}")

# SDL: stick up = -1. Full forward + R2 held -> 0.6 * 2.0 = 1.2 m/s.
check(twist_close(axes_to_twist(0.0, -1.0, 0.0, 1.0, CFG), (gp.CLAMP_LINEAR_MS, 0.0, 0.0)),
      "stick fully forward + boost -> vx CLAMPED at 0.2 m/s (post-runaway ceiling)")
check(twist_close(axes_to_twist(0.0, -1.0, 0.0, -1.0, CFG), (gp.CLAMP_LINEAR_MS, 0.0, 0.0)),
      "stick fully forward -> vx CLAMPED at 0.2 m/s")
check(twist_close(axes_to_twist(0.0, -0.5, 0.0, -1.0, CFG), (0.3, 0.0, 0.0)),
      "half stick forward -> vx = 0.3 m/s (proportional below the ceiling)")
check(twist_close(axes_to_twist(0.0, 1.0, 0.0, -1.0, CFG), (-gp.CLAMP_LINEAR_MS, 0.0, 0.0)),
      "stick fully back -> vx CLAMPED at -0.2 m/s")

# Deadzone: 0.32 since the 2026-08-27 feel tuning - the previous deadband was
# consistently too small. Below it the axis is dead, above it goes through unscaled.
check(twist_close(axes_to_twist(0.05, -0.10, 0.11, -1.0, CFG), (0.0, 0.0, 0.0)),
      "all sticks inside the deadzone -> 0.0 on every axis")
check(twist_close(axes_to_twist(0.0, -0.20, 0.0, -1.0, CFG), (0.0, 0.0, 0.0)),
      "0.20 is INSIDE the 0.32 deadzone now -> 0.0 (feel tuning 15h20)")
check(twist_close(axes_to_twist(0.0, -0.35, 0.0, -1.0, CFG), (0.21, 0.0, 0.0)),
      "0.35 just outside the deadzone -> vx = 0.21 m/s (no rescaling)")

# Rotation: right stick right (+1) turns clockwise = negative wz.
check(twist_close(axes_to_twist(0.0, 0.0, 1.0, -1.0, CFG), (0.0, 0.0, -gp.CLAMP_ANGULAR_RADS)),
      "right stick X = +1 -> wz CLAMPED at -0.6 rad/s")
check(twist_close(axes_to_twist(0.0, 0.0, 1.0, 1.0, CFG), (0.0, 0.0, -gp.CLAMP_ANGULAR_RADS)),
      "boost does not touch rotation -> wz stays at the -0.6 ceiling")

# Strafe: left stick left (-1) strafes to +y.
check(twist_close(axes_to_twist(-1.0, 0.0, 0.0, -1.0, CFG), (0.0, gp.CLAMP_LINEAR_MS, 0.0)),
      "left stick X = -1 -> vy CLAMPED at +0.2 m/s (strafe left)")
check(twist_close(axes_to_twist(1.0, 0.0, 0.0, -1.0, CFG), (0.0, -gp.CLAMP_LINEAR_MS, 0.0)),
      "left stick X = +1 -> vy CLAMPED at -0.2 m/s (strafe right)")
check(twist_close(axes_to_twist(-0.5, -0.5, 0.5, -1.0, CFG),
                  (0.15, 0.15, -0.3)),
      "diagonal + turn (half sticks: 0.3+0.3+0.5*0.6 = 0.9 m/s rim) -> wheel "
      "envelope halves it to (0.15, 0.15, -0.3) - the '17h08 additive speeds' fix")

# A non-default envelope must scale, nothing hard-coded.
SLOW = TeleopConfig(linear_speed=0.25, angular_speed=0.5, deadzone=0.2,
                    boost_multiplier=3.0)
vx, vy, wz = axes_to_twist(0.0, -1.0, -1.0, 1.0, SLOW)
rim = abs(vx) + abs(vy) + gp.MECANUM_LEVER_M * abs(wz)
check(abs(rim - gp.WHEEL_ENVELOPE_MS) < 1e-9 and abs(vx / wz - 0.45 / 0.5) < 1e-9,
      "custom cfg full fwd+turn+boost -> rim held at the envelope, gesture proportions kept")


# --- B. hot-plug cycle on a fake pygame -----------------------------------
print("\nB. hot-plug loop (fake pygame, no display, no pad)")


class FakeError(Exception):
    """Stand-in for pygame.error."""


class FakePad:
    def __init__(self, name, axes, buttons=8):
        self.name, self.axes = name, axes
        self.buttons = [0] * buttons

    def init(self):
        pass

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, index):
        return self.axes[index]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, index):
        return self.buttons[index]


class FakeJoystickModule:
    """SDL joystick subsystem stand-in; the test plugs `pad` in and out."""

    def __init__(self):
        self.pad = None
        self.rescans = 0

    def init(self):
        self.rescans += 1

    def quit(self):
        pass

    def get_count(self):
        return 0 if self.pad is None else 1

    def Joystick(self, index):  # noqa: N802 - pygame's own name
        if self.pad is None:
            raise FakeError("no joystick at index %d" % index)
        return self.pad


class LogSpy:
    """Records what the module logs (dimOS's logger is structlog, not stdlib)."""

    def __init__(self):
        self.lines = []

    def _record(self, msg, *args):
        self.lines.append(msg % args if args else msg)

    info = warning = error = debug = _record

    def count(self, needle):
        return sum(needle in line for line in self.lines)


js = FakeJoystickModule()
fake_pygame = types.SimpleNamespace(
    error=FakeError,
    init=lambda: (5, 0),
    quit=lambda: None,
    event=types.SimpleNamespace(pump=lambda: None),
    joystick=js,
)
sys.modules["pygame"] = fake_pygame
spy = LogSpy()
real_logger, gp.logger = gp.logger, spy

pad_module = GamepadTeleop(rate_hz=200.0)
published = []
pad_module.tele_cmd_vel.subscribe(published.append)
pad_module.start()

# 1. no pad at startup: the module waits instead of returning.
check(wait_for(lambda: spy.count("waiting for gamepad (index 0)") == 1, 3.0),
      "no pad at startup -> logs 'waiting for gamepad (index 0)'")
check(wait_for(lambda: js.rescans >= 3, 5.0),
      f"keeps re-scanning while waiting ({js.rescans} scans in ~5 s)")
check(pad_module._thread.is_alive() and not published,
      "loop still alive after several empty scans, nothing published")
check(spy.count("waiting for gamepad (index 0)") == 1,
      "the waiting line is logged ONCE, not once per scan")

# 2. pad appears WITH FULL-DEFLECTION AXES (the 13h42 runaway pattern):
# the trust gate must hold everything at zero until neutral is seen.
js.pad = FakePad("Fake DS4", [0.0, -1.0, -1.0, 0.0, 0.0, -1.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake DS4") == 1, 5.0),
      "pad plugged in -> logs its name")
time.sleep(0.3)
check(all(close(t.linear.x, 0.0) for t in published[-10:]) if published else True,
      "full deflection at connection -> ZEROS only (trust gate holds)")
# neutral seen once -> trust earned. Axis 2 IS the rotation stick since the
# 15h21 measurement - its rest is 0.0, not the old axis-3-era -1.0.
js.pad.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: spy.count("axes seen at neutral") == 1, 3.0),
      "sticks at rest -> trust earned (logged once)")
# deadman NOT held: still zeros even with a deflected stick (axis 2 = the
# rotation stick since 15h21: at rest 0.0, or the pose commands a full turn)
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
time.sleep(0.3)
check(all(close(t.linear.x, 0.0) for t in published[-5:]),
      "stick forward WITHOUT the deadman held -> still zeros")
# deadman held -> motion, CLAMPED at the ceiling
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: published and close(published[-1].linear.x, gp.CLAMP_LINEAR_MS), 3.0),
      f"deadman held + stick forward -> vx = 0.2 m/s CEILING (got {published[-1].linear.x if published else None})")
check(close(published[-1].linear.y, 0.0) and close(published[-1].angular.z, 0.0),
      "and vy = 0.0 m/s, wz = 0.0 rad/s on the other axes")

# 2b. deadman RELEASED at speed, then re-pressed with the sticks centred: the
# slew state has to restart from rest. Before the 28/08 fix prev_vx kept the
# last driven 0.45 m/s, so the re-press published 0.438 m/s with the sticks at
# rest and the rover left on its own for ~0.75 s.
js.pad.buttons[gp.DEADMAN_BUTTON] = 0
js.pad.axes = [0.0, 0.0, 0.0, 0.0, 0.0, -1.0]   # sticks back to neutral
check(wait_for(lambda: close(published[-1].linear.x, 0.0), 3.0),
      "deadman released while driving at the ceiling -> zeros published")
re_press = len(published)                       # index taken BEFORE the re-press
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: len(published) - re_press >= 20, 3.0),
      "deadman pressed again -> the loop keeps publishing")
after = published[re_press:]
peak = max(abs(m.linear.x) for m in after)
check(all(close(m.linear.x, 0.0) and close(m.linear.y, 0.0)
          and close(m.angular.z, 0.0) for m in after),
      f"re-press with the sticks centred -> 0.000 m/s from the FIRST command "
      f"(peak |vx| = {peak:.3f} m/s over {len(after)} messages; 0.438 before the fix)")
# and the ramp climbs again from rest to the ceiling (drive state restored)
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: close(published[-1].linear.x, gp.CLAMP_LINEAR_MS), 3.0),
      f"stick forward again -> ramps back up to the {gp.CLAMP_LINEAR_MS} m/s ceiling")

# 3. pad pulled out: exactly one zero Twist, then back to waiting.
sent_before_unplug = len(published)
js.pad = None
check(wait_for(lambda: spy.count("back to waiting") == 1, 3.0),
      "pad unplugged -> logs the loss once")
check(wait_for(lambda: close(published[-1].linear.x, 0.0), 2.0),
      "publishes a zero Twist on loss")
zeros = [m for m in published[sent_before_unplug:] if close(m.linear.x, 0.0)]
check(len(zeros) == 1, f"exactly ONE zero Twist on loss (got {len(zeros)})")
check(pad_module._thread.is_alive(), "loop survives the unplug")
check(spy.count("waiting for gamepad (index 0)") == 2,
      "goes back to the waiting state")

# 4. pad comes back: trust must be RE-EARNED at neutral, then deadman,
# then boost + turn - both CLAMPED at the ceilings.
js.pad = FakePad("Fake DS5", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake DS5") == 1, 5.0),
      "pad plugged back in -> logs the new name")
check(wait_for(lambda: spy.count("axes seen at neutral") == 2, 3.0),
      "re-connection re-earns trust at neutral")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 1.0, 0.0, 0.0, 1.0]   # full fwd + full turn (axis 2) + boost
expected_k = gp.WHEEL_ENVELOPE_MS / (gp.CLAMP_LINEAR_MS + gp.MECANUM_LEVER_M * gp.CLAMP_ANGULAR_RADS)
check(wait_for(lambda: published and close(published[-1].linear.x, gp.CLAMP_LINEAR_MS * expected_k), 3.0),
      f"boost + full turn -> vx enveloped to {gp.CLAMP_LINEAR_MS * expected_k:.3f} "
      f"(the rim, not each axis, is the ceiling; got {published[-1].linear.x if published else None})")
check(close(published[-1].angular.z, -gp.CLAMP_ANGULAR_RADS * expected_k),
      f"... and wz enveloped with the SAME factor (proportions of the gesture kept)")

# 5. a pad that has no R2 axis at all must read as "no boost", not crash.
js.pad = None
check(wait_for(lambda: spy.count("waiting for gamepad (index 0)") == 3, 4.0),
      "pad pulled again -> waiting")
js.pad = FakePad("Fake 4-axis", [0.0, 0.0, 0.0, 0.0])
check(wait_for(lambda: spy.count("Gamepad connected: Fake 4-axis") == 1, 5.0),
      "4-axis pad (no R2) accepted")
check(wait_for(lambda: spy.count("axes seen at neutral") == 3, 3.0),
      "4-axis pad earns trust at neutral")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 0.0, 0.0]
check(wait_for(lambda: close(published[-1].linear.x, gp.CLAMP_LINEAR_MS), 3.0)
      and pad_module._thread.is_alive(),
      f"missing R2 = no boost; full stick -> the {gp.CLAMP_LINEAR_MS} ceiling, loop alive")

t0 = time.monotonic()
pad_module.stop()
stop_s = time.monotonic() - t0
check(not pad_module._thread.is_alive() and stop_s < 2.0,
      f"stop() joins the loop in {stop_s:.2f} s")
check(close(published[-1].linear.x, 0.0) and close(published[-1].angular.z, 0.0),
      "last message on shutdown is a zero Twist")

# 6. BRAKE THEN SILENCE, at the real 50 Hz rate (28/08 autonomy fix). dimOS's
# MovementManager reads EVERY tele_cmd_vel message - zeros included - as "a
# human is driving": it cancels the nav goal and mutes nav_cmd_vel for
# tele_cooldown_sec (1.0 s). Publishing 50 zeros/s at rest therefore left
# GAMEPAD=1 exploring nothing, for ever. The released pad now brakes for 0.5 s
# (<= 25 messages at 50 Hz) and then says NOTHING at all.
js.pad = FakePad("Fake 50Hz", [0.0, 0.0, 0.0, 0.0, 0.0, -1.0])
brake_module = GamepadTeleop(rate_hz=50.0)
brake_pub = []
brake_module.tele_cmd_vel.subscribe(brake_pub.append)
brake_module.start()
check(wait_for(lambda: spy.count("Gamepad connected: Fake 50Hz") == 1, 5.0),
      "50 Hz module: pad connected")
check(wait_for(lambda: spy.count("axes seen at neutral") == 4, 3.0),
      "50 Hz module: trust earned at neutral")
check(not brake_pub, f"deadman never held -> not a single message published "
                     f"(got {len(brake_pub)})")
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
js.pad.axes = [0.0, -1.0, 0.0, 0.0, 0.0, -1.0]
check(wait_for(lambda: close(brake_pub[-1].linear.x, gp.CLAMP_LINEAR_MS) if brake_pub else False, 3.0),
      f"deadman held -> the 50 Hz flow ramps to the {gp.CLAMP_LINEAR_MS} m/s ceiling")
js.pad.buttons[gp.DEADMAN_BUTTON] = 0   # release, at speed
time.sleep(1.2)                          # 0.5 s of brake + 0.7 s past its end
last_drive = max(i for i, m in enumerate(brake_pub) if not close(m.linear.x, 0.0))
brake = brake_pub[last_drive + 1:]
check(all(close(m.linear.x, 0.0) and close(m.linear.y, 0.0)
          and close(m.angular.z, 0.0) for m in brake),
      f"release -> every message after the last driven one is a zero Twist "
      f"({len(brake)} of them)")
check(1 <= len(brake) <= 25,
      f"... and at most 25 of them (BRAKE_S {gp.BRAKE_S} s x 50 Hz): got {len(brake)}")
silent_from = len(brake_pub)
time.sleep(0.6)                          # 30 ticks at 50 Hz
check(len(brake_pub) == silent_from,
      f"then SILENCE: 0 messages in the next 0.6 s (got "
      f"{len(brake_pub) - silent_from}; pre-fix: 30 zeros, autonomy muted)")
# and the flow restarts on the next press - the pad is not disarmed, just quiet
js.pad.buttons[gp.DEADMAN_BUTTON] = 1
check(wait_for(lambda: len(brake_pub) - silent_from >= 20, 2.0),
      f"deadman pressed again -> the 50 Hz flow resumes "
      f"({len(brake_pub) - silent_from} messages)")
restart = brake_pub[silent_from:]
step = gp.SLEW_LINEAR_MS2 / 50.0        # 0.012 m/s per tick at 50 Hz
check(abs(restart[0].linear.x) <= step + 1e-9,
      f"... from rest: first vx = {restart[0].linear.x:.3f} m/s <= one slew "
      f"step ({step:.3f}), not the 0.438 of the stale-state bug")
check(wait_for(lambda: close(brake_pub[-1].linear.x, gp.CLAMP_LINEAR_MS), 3.0),
      f"... and ramps back to the {gp.CLAMP_LINEAR_MS} m/s ceiling (stick still forward)")
brake_module.stop()

gp.logger = real_logger
del sys.modules["pygame"]


# --- C. the real pygame, headless -----------------------------------------
print("\nC. real pygame with no display")
try:
    import pygame
except ImportError:
    print("  ..  pygame not installed here - headless check skipped")
else:
    logging.disable(logging.CRITICAL)  # keep SDL chatter out of the report
    n_ok, n_fail = pygame.init()
    pygame.joystick.init()
    pygame.event.pump()                      # the call that needs a display
    before = pygame.joystick.get_count()
    pygame.joystick.quit()
    pygame.joystick.init()                   # the hot-plug re-scan
    after = pygame.joystick.get_count()
    pygame.quit()
    logging.disable(logging.NOTSET)
    check(n_fail == 0, f"pygame.init() with SDL_VIDEODRIVER=dummy ({n_ok} modules up)")
    check(before == after, f"event.pump() and joystick re-scan work headless "
                           f"({after} pad(s) connected)")

print("\nF. the 27/08 safety nets (13h42 runaway)")
from vector_dimos.gamepad import (
    CLAMP_ANGULAR_RADS, CLAMP_LINEAR_MS, TeleopConfig, axes_neutral, axes_to_twist, clamp_twist,
)

cfg = TeleopConfig()
# axes uninitialised at full deflection: neutral was never observed
check(axes_neutral(1.0, -1.0, 1.0, cfg.deadzone) is False,
      "full deflection = NOT neutral (the trust gate refuses it)")
check(axes_neutral(0.01, -0.02, 0.0, cfg.deadzone) is True,
      "pad at rest = neutral (the gate opens)")
# absolute ceilings: even an insane config never exceeds them
wild = TeleopConfig(linear_speed=5.0, angular_speed=9.0, boost_multiplier=4.0)
vx, vy, wz = axes_to_twist(-1.0, -1.0, 1.0, 1.0, wild)
check(abs(vx) <= CLAMP_LINEAR_MS + 1e-9, f"full stick + boost + insane config -> |vx| <= {CLAMP_LINEAR_MS}")
check(abs(vy) <= CLAMP_LINEAR_MS + 1e-9, f"... and |vy| <= {CLAMP_LINEAR_MS}")
check(abs(wz) <= CLAMP_ANGULAR_RADS + 1e-9, f"... and |wz| <= {CLAMP_ANGULAR_RADS}")
# since 2026-08-27 clamp_twist ends with the wheel envelope (mecanum: the
# commands ADD UP at the rim - learned on the first piloted lap)
from vector_dimos.gamepad import MECANUM_LEVER_M, WHEEL_ENVELOPE_MS
vx, vy, wz = clamp_twist(9.0, -9.0, 9.0)
check(abs(abs(vx) + abs(vy) + MECANUM_LEVER_M * abs(wz) - WHEEL_ENVELOPE_MS) < 1e-9,
      "pure clamp_twist: 9 m/s everywhere -> rim exactly at the envelope")
# a single stick at full travel loses NOTHING (the envelope = the feel of one stick alone)
check(clamp_twist(0.45, 0.0, 0.0) == (0.45, 0.0, 0.0), "pure forward 0.45 -> unchanged")
check(clamp_twist(0.0, 0.0, 0.8) == (0.0, 0.0, 0.8), "pure rotation 0.8 -> unchanged (rim 0.40 < 0.45)")
# the mix experienced on the piloted lap: full forward + full rotation
vx, vy, wz = clamp_twist(0.45, 0.0, 0.8)
rim = abs(vx) + MECANUM_LEVER_M * abs(wz)
check(abs(rim - WHEEL_ENVELOPE_MS) < 1e-9,
      f"forward+rotation mix -> rim {rim:.3f} = envelope {WHEEL_ENVELOPE_MS} (was 0.85)")
check(abs(vx / wz - 0.45 / 0.8) < 1e-9,
      "... and the proportions of the gesture are kept (vx/wz constant)")


print("\nG. the teleop ramp (27/08 15h17 wheelspin)")
from vector_dimos.gamepad import SLEW_LINEAR_MS2, slew
check(abs(slew(0.0, 0.45, 0.1) - 0.06) < 1e-9,
      "no jump: 0 -> full stick limited to 0.06 m/s per 0.1 s step")
check(abs(slew(0.45, 0.0, 0.1) - 0.39) < 1e-9, "ramp down limited too (0.45 -> 0)")
check(slew(0.05, 0.06, 0.1) == 0.06, "a near target is reached exactly")

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

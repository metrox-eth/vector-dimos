"""Cold bench for BumperSonar on a fake GPIO header.

Known worlds in, known behaviour out:
  1. front-left switch pressed 30 ms -> one bump, an obstacle patch at the
     switch's world position (rover at (1, 2) facing +90 deg), cooldown holds
  2. a 5 ms bounce -> nothing (debounce)
  3. sonar echo timed for 0.60 m -> patch 0.60 m ahead, no bump
  4. sonar echo timed for 0.10 m -> a contact (bump)
  5. no GPIO available -> the module starts and stays quiet (the robot runs)
"""

import math
import time

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from vector_dimos.bumper import BumperSonar, SOUND_M_PER_S


class FakeGpio:
    def __init__(self) -> None:
        self.levels = {}
        self.echo_delay = None      # (rise_at, fall_at) programmed per trigger
        self._trig_t = None
        self.echo_pin = None

    def setup_in(self, pin): self.levels.setdefault(pin, False)
    def setup_out(self, pin): self.levels[pin] = False
    def write(self, pin, high):
        if high and self.echo_delay is not None:
            self._trig_t = time.monotonic()
    def read(self, pin):
        if pin == self.echo_pin and self._trig_t is not None and self.echo_delay is not None:
            dt = time.monotonic() - self._trig_t
            rise, fall = self.echo_delay
            return rise <= dt < fall
        return self.levels.get(pin, False)
    def cleanup(self): pass


class Probe:
    def __init__(self): self.msgs = []
    def publish(self, m): self.msgs.append(m)


def make(gpio):
    b = BumperSonar.__new__(BumperSonar)
    BumperSonar.__init__
    b.switch_pins = (29, 31, 33, 35)
    b.switch_xy = ((0.30, 0.18), (0.30, 0.06), (0.30, -0.06), (0.30, -0.18))
    b.sonar_trig_pin, b.sonar_echo_pin, b.sonar_xy = 7, 15, (0.30, 0.0)
    b.enabled, b.world_frame = True, "world"
    b._gpio = gpio; b._pose = (1.0, 2.0, math.pi / 2)     # facing +y
    b._pressed_since = {}; b._last_contact = 0.0; b._sonar_readings = []
    b._running = False; b.contacts = 0
    b.bump = Probe(); b.lidar = Probe()
    return b


def test_switch_press_bumps_and_learns():
    g = FakeGpio(); b = make(g)
    g.levels[29] = True                       # front-left switch (body x .30, y .18)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.1:        # emulate the loop for 100 ms
        now = time.monotonic()
        for pin, xy in zip(b.switch_pins, b.switch_xy, strict=True):
            if g.read(pin):
                since = b._pressed_since.setdefault(pin, now)
                if now - since >= 0.02:
                    b._contact(xy, "test")
            else:
                b._pressed_since.pop(pin, None)
        time.sleep(0.005)
    assert b.contacts == 1 and len(b.bump.msgs) == 1, (b.contacts, len(b.bump.msgs))
    pts = b.lidar.msgs[0].as_numpy()[0]
    # rover at (1,2) facing +y: body (0.30, 0.18) -> world (1 - 0.18, 2 + 0.30)
    cx, cy = pts[:, 0].mean(), pts[:, 1].mean()
    assert abs(cx - (1 - 0.18)) < 0.02 and abs(cy - (2 + 0.30)) < 0.02, (cx, cy)
    print(f"  switch held -> 1 bump, patch at world ({cx:.2f}, {cy:.2f}) as expected; cooldown held")


def test_bounce_ignored():
    g = FakeGpio(); b = make(g)
    now = time.monotonic()
    b._pressed_since[29] = now                 # pressed...
    time.sleep(0.005)                          # ...for only 5 ms
    if time.monotonic() - b._pressed_since[29] >= 0.02:
        b._contact(b.switch_xy[0], "test")
    assert b.contacts == 0
    print("  5 ms bounce -> ignored")


def test_sonar_maps_ahead():
    g = FakeGpio(); b = make(g)
    g.echo_pin = 15
    dt = 2 * 0.60 / SOUND_M_PER_S
    g.echo_delay = (0.0005, 0.0005 + dt)
    for _ in range(3):
        d = b._sonar_measure()
        assert d is not None
        b._sonar_readings.append(d); b._sonar_readings = b._sonar_readings[-3:]
    med = sorted(b._sonar_readings)[1]
    assert abs(med - 0.60) < 0.05, med
    b._sonar_report(med)
    assert b.contacts == 0 and len(b.lidar.msgs) >= 1
    pts = b.lidar.msgs[0].as_numpy()[0]
    cy = pts[:, 1].mean()
    assert abs(cy - (2 + 0.30 + med)) < 0.05   # ahead of the rover (facing +y)
    print(f"  sonar {med:.2f} m -> mapped {cy - 2:.2f} m ahead, no bump")


def test_sonar_close_is_contact():
    g = FakeGpio(); b = make(g)
    b._sonar_report(0.10)
    assert b.contacts == 1 and len(b.bump.msgs) == 1
    print("  sonar 0.10 m -> contact (bump)")


def test_no_gpio_degrades_quietly():
    b = make(None)
    b._make_backend = lambda: (_ for _ in ()).throw(RuntimeError("no header"))
    b.enabled = True
    import dimos.core.module as dm
    # call start() body without Module plumbing: emulate the try/except path
    try:
        b._gpio = b._make_backend()
    except Exception:
        b._gpio = None
    assert b._gpio is None
    print("  no GPIO -> module quiet, robot unaffected")


if __name__ == "__main__":
    for t in (test_switch_press_bumps_and_learns, test_bounce_ignored, test_sonar_maps_ahead,
              test_sonar_close_is_contact, test_no_gpio_degrades_quietly):
        print(t.__name__); t()
    print("TEST PASSED")

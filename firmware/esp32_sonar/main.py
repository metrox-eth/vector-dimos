# ESP32-S3: 4 contact switches + HC-SR04 sonar (MicroPython) - MUSEAU-ESP v3
# Switches: NO -> GPIO 1,2,3,4 ; COM chain -> GND ; internal pull-ups.
#   Pressed = low. v3: hardware IRQ on the falling edge with a LATCH - a
#   1 ms impact click is captured even while the sonar ping blocks the
#   loop (v2 polled every 5 ms with a 20 ms debounce and the 30 ms
#   time_pulse_us made it deaf 30% of the time: finger presses passed,
#   real collision clicks never did - owner heard them, logs stayed empty,
#   26/08). The receiving module has its own 1 s cooldown, so switch
#   bounce costs nothing here.
#   Lines: "SW a b c d" on every latched hit and on state change (1 =
#   pressed) + a 500 ms heartbeat of the steady state.
# Sonar: TRIG GPIO 5, ECHO GPIO 6 -> "SONAR <metres>" 10x/s (-1 = none).
from machine import Pin, time_pulse_us
import time

sw_pins = [Pin(n, Pin.IN, Pin.PULL_UP) for n in (1, 2, 3, 4)]
trig = Pin(5, Pin.OUT, value=0)
echo = Pin(6, Pin.IN)

latched = [0, 0, 0, 0]   # set by the IRQ, cleared after publishing

def _make_irq(i):
    def _irq(_pin):
        latched[i] = 1   # ISR: flag only - no allocation, no print
    return _irq

for i, p in enumerate(sw_pins):
    p.irq(trigger=Pin.IRQ_FALLING, handler=_make_irq(i))

def sw_state():
    return tuple(0 if p.value() else 1 for p in sw_pins)   # 1 = pressed

last = sw_state()
last_beat = 0
last_ping = 0

print("MUSEAU-ESP v3: IRQ latch switches GPIO 1-4 (active low) + sonar 5/6")
while True:
    now = time.ticks_ms()

    if any(latched):
        hit = tuple(max(l, s) for l, s in zip(latched, sw_state()))
        print("SW", *hit)                       # the impact, even if released
        for i in range(4):
            latched[i] = 0
        last = sw_state()
        if last != hit:
            print("SW", *last)                  # back to rest right away

    s = sw_state()
    if s != last:                               # slow presses / releases
        last = s
        print("SW", *s)

    if time.ticks_diff(now, last_beat) >= 500:
        last_beat = now
        print("SW", *last)

    if time.ticks_diff(now, last_ping) >= 100:
        last_ping = now
        trig.value(1)
        time.sleep_us(10)
        trig.value(0)
        d = time_pulse_us(echo, 1, 30000)
        print("SONAR {:.3f}".format(d / 1e6 * 343 / 2) if d > 0 else "SONAR -1")

    time.sleep_ms(5)

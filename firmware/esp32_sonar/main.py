# Sonar HC-SR04 sur ESP32-S3 — TRIG GPIO 5, ECHO GPIO 6 (alimentation 3V3)
# Sort une ligne "SONAR <metres>" 10 fois par seconde ; -1 = pas d'echo.
from machine import Pin, time_pulse_us
import time

trig = Pin(5, Pin.OUT, value=0)
echo = Pin(6, Pin.IN)

while True:
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)
    d = time_pulse_us(echo, 1, 30000)
    if d > 0:
        print("SONAR {:.3f}".format(d / 1e6 * 343 / 2))
    else:
        print("SONAR -1")
    time.sleep_ms(100)

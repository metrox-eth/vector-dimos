# Contact sensors + sonar on ESP32-S3 (MicroPython)

## Corner map (validated 2026-08-25, deterministic on the first pass)

| Corner | GPIO | leg |
|---|---|---|
| front-left | 1 | active low (COMs chained to GND, internal pull-ups) |
| rear-left | 2 | same |
| rear-right | 3 | same |
| front-right | 4 | same |

Serial output: `SW a b c d` (GPIO order 1,2,3,4; 1 = pressed) on every change
plus a 500 ms heartbeat. The comb's resistors were CLIPPED (they were fighting
the internal pull-ups).

Born 2026-08-25, after the TXB0108 verdict (docs/verdict_museau_20260825.md):
the sonar could not live on the Jetson's 40-pin header; on the ESP32-S3 it
worked on the first try, at 3.3 V, with ZERO resistors.

- Wiring: VCC→3V3, TRIG→GPIO5, ECHO→GPIO6, GND→GND (hardware/bumper/schema_esp32.png)
- Firmware: MicroPython v1.25.0 (ESP32_GENERIC_S3) + this main.py
- Output: "SONAR <metres>" at 10 Hz on the native USB (-1 = no echo)
- Jetson port: /dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_80b54ee325280000-if00
- Flash: esptool erase_flash + write_flash 0 (hold BOOT + tap RST to enter the
  bootloader), then mpremote cp main.py :main.py
- Validated: hand at ~30 cm → readings 0.24–0.30 m; stage backdrop stable at 1.074 m.
- **Useful range MEASURED mounted at 3.3 V: 66 cm max → software confidence
  threshold = 0.55 m.** Beyond that: "out of range", never "clear".

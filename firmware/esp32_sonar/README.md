# Sonar HC-SR04 sur ESP32-S3 (MicroPython)

Né le 25/08/2026, après le verdict TXB0108 (docs/verdict_museau_20260825.md) :
le sonar ne pouvait pas vivre sur le 40-pin du Jetson ; sur l'ESP32-S3 il a
marché du premier coup, en 3,3 V, ZÉRO résistance.

- Câblage : VCC→3V3, TRIG→GPIO5, ECHO→GPIO6, GND→GND (hardware/bumper/schema_esp32.png)
- Firmware : MicroPython v1.25.0 (ESP32_GENERIC_S3) + ce main.py
- Sortie : "SONAR <mètres>" à 10 Hz sur l'USB natif (-1 = pas d'écho)
- Port Jetson : /dev/serial/by-id/usb-Espressif_Systems_Espressif_Device_80b54ee325280000-if00
- Flash : esptool erase_flash + write_flash 0 (BOOT enfoncé + RST pour entrer
  en bootloader), puis mpremote cp main.py :main.py
- Validé : main à ~30 cm → lectures 0,24-0,30 m ; fond de scène stable à 1,074 m.

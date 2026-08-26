"""VECTOR flight check: every sensor and bus probed read-only, one verdict each.

Born 26/08 (owner: "fais-toi un flight check... sinon tous les jours tu vas
devoir reapprendre qu'on controle un rover"). The 25/08 health-check script
lived in /tmp and died there; this one is committed. Run it BEFORE any run,
after any power cycle, and whenever a device looks dead - it answers "what do
you see, what do you not see" in ten seconds, with the real port map:

    ttyUSB0  FTDI FT232R (Waveshare dongle)  ZLAC motor bus, 115200
    ttyUSB1  CH340                           PZEM-017 battery shunt, 9600 8N2
    ttyACM0  Espressif ESP32                 contact switches + sonar, 115200
    ttyUSB2  CP2102 Silicon Labs              RPLIDAR C1, 460800

Read-only: nothing is enabled, written or moved.

    .venv/bin/python tools/preflight.py
"""
from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import serial  # noqa: E402

RESULTS: list[tuple[bool, str]] = []


def verdict(ok: bool, label: str, detail: str = "") -> None:
    RESULTS.append((ok, label))
    print(f"  {'OK ' if ok else 'KO '} {label}" + (f" - {detail}" if detail else ""))


def check_ports() -> None:
    print("ports serie (by-id)")
    from vector_dimos.adapter import DEFAULT_PORT as MOTOR_PORT
    from vector_dimos.esp_sensors import ESP_PORT
    from vector_dimos.rplidar_c1 import DEFAULT_PORT as LIDAR_PORT
    for label, path in (("moteurs (FTDI/Waveshare)", MOTOR_PORT),
                        ("lidar C1 (CP2102)", LIDAR_PORT),
                        ("ESP32 contacts+sonar", ESP_PORT),
                        ("shunt PZEM (CH340)", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")):
        verdict(Path(path).exists(), f"{label} enumere", path.rsplit('/', 1)[-1])


def check_motors() -> None:
    print("moteurs (ZLAC, sonde lecture seule)")
    try:
        from pymodbus.client.sync import ModbusSerialClient

        from vector_dimos.adapter import BACK_ID, BAUDRATE, DEFAULT_PORT, FRONT_ID
        c = ModbusSerialClient(method="rtu", port=DEFAULT_PORT, baudrate=BAUDRATE, timeout=0.5)
        if not c.connect():
            verdict(False, "bus moteurs: le port ne s'ouvre pas")
            return
        for unit, name in ((BACK_ID, "arriere"), (FRONT_ID, "avant")):
            t0 = time.perf_counter()
            r = c.read_holding_registers(0x20AB, 2, unit=unit)
            ms = (time.perf_counter() - t0) * 1000
            if hasattr(r, "registers"):
                verdict(True, f"drive {name} (id {unit}) repond",
                        f"{ms:.0f} ms, feedback {r.registers}")
            else:
                verdict(False, f"drive {name} (id {unit}) muet", str(r))
        c.close()
    except Exception as exc:  # noqa: BLE001
        verdict(False, "bus moteurs", str(exc))


def check_battery() -> None:
    print("batterie (PZEM-017 sur le shunt)")
    try:
        from pymodbus.client.sync import ModbusSerialClient
        c = ModbusSerialClient(method="rtu", port="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
                               baudrate=9600, parity="N", stopbits=2, timeout=1.0)
        if not c.connect():
            verdict(False, "shunt: le port ne s'ouvre pas")
            return
        r = c.read_input_registers(0x0000, 4, unit=1)
        if hasattr(r, "registers"):
            v = r.registers[0] * 0.01
            i = r.registers[1] * 0.01
            w = (r.registers[2] | (r.registers[3] << 16)) * 0.1
            verdict(v > 20.0, "batterie", f"{v:.2f} V, {i:.2f} A, {w:.1f} W")
        else:
            verdict(False, "shunt muet", str(r))
        c.close()
    except Exception as exc:  # noqa: BLE001
        verdict(False, "shunt PZEM", str(exc))


def check_lidar() -> None:
    print("lidar C1 (info + salves de reveil integrees)")
    try:
        from vector_dimos.c1_serial import C1Lidar
        from vector_dimos.rplidar_c1 import DEFAULT_PORT
        lidar = C1Lidar(DEFAULT_PORT)
        info = lidar.get_info()
        status, code = lidar.get_health()
        lidar.stop()
        lidar.disconnect()
        verdict(status == "Good", "lidar repond", f"health {status} ({code}), info {info}")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "lidar MUET (salves comprises)",
                f"{exc} -> echelle: RESET+salves deja dans open(); ensuite c'est "
                "PHYSIQUE (5 V / cable TX / connecteur - cf 25/08: reboot hub + recollage)")


def check_esp() -> None:
    print("ESP32 (contacts + sonar)")
    try:
        from vector_dimos.esp_sensors import ESP_BAUD, ESP_PORT
        ser = serial.Serial(ESP_PORT, ESP_BAUD, timeout=2.5)
        lines = [ser.readline().decode(errors="replace").strip() for _ in range(3)]
        ser.close()
        alive = [ln for ln in lines if ln]
        verdict(bool(alive), "ESP32 emet", alive[0] if alive else "silence en 7,5 s")
        # the evening of 26/08: the sonar read 0.08 m for two hours, the
        # adapter brake clamped every forward command to zero, and the check
        # PRINTED the number without judging it. A resting rover with a wall
        # at less than 0.30 m cannot drive: say it loud.
        import re as _re
        sonar = [float(m.group(1)) for ln in lines
                 for m in [_re.search(r"SONAR ([0-9.]+)", ln)] if m]
        if sonar:
            # WARNING only, never a KO: the sonar is out of the drive path
            # (SONAR_ENABLED=False, owner's vote 26/08 19h45) and a disabled
            # sensor must not be able to cancel a flight. The loud cushion
            # message stays - it is the reminder for the owner's bench.
            blocked = min(sonar) < 0.30
            print(f"  {'!! ' if blocked else 'OK '}sonar (info seulement) - {min(sonar):.2f} m"
                  + (" -> LE COUSSINET DU BUMPER S'EST RELEVE DEVANT LE SONAR: le remettre"
                     " (cause du 26/08 - le sonar est coupe dans la stack, ceci ne bloque PAS le vol)"
                     if blocked else ""))
    except Exception as exc:  # noqa: BLE001
        verdict(False, "ESP32", str(exc))


def check_usb_devices() -> None:
    print("USB (camera, micro)")
    try:
        out = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=10).stdout
        verdict("8086:0b5c" in out, "RealSense D455F presente")
        verdict("2886:" in out, "ReSpeaker 4-mic present")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "lsusb", str(exc))


def main() -> int:
    for step in (check_ports, check_motors, check_battery, check_lidar,
                 check_esp, check_usb_devices):
        step()
    ko = [label for ok, label in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(ko)}/{len(RESULTS)} OK" + (f" - KO: {', '.join(ko)}" if ko else " - PARE AU VOL"))
    return 1 if ko else 0


if __name__ == "__main__":
    raise SystemExit(main())

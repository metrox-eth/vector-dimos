"""VECTOR rover stats server for the control panel.

Serves GET /metrics on :8900 as JSON: CPU, RAM, thermal zones, disks,
and battery via PZEM-017 (MODBUS-RTU over USB-RS485) once its dongle
is plugged in. Stdlib only; pyserial optional (battery reading).

Run on the Jetson:  python3 tools/stats_server.py
State of charge: Sam's linear scale, 20.0 V = 0% -> 28.0 V = 100%.
"""

import glob
import json
import os
import socket
import struct
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial  # optional, only for PZEM
except ImportError:
    serial = None

PORT = 8900
SOC_V_EMPTY = 20.0
SOC_V_FULL = 28.0
LIDAR_ID_HINT = "7271bbd88d71f011af43029f1045c30f"  # lidar's CP2102N: never probe it

_prev_cpu = None  # (idle, total)
_pzem_port_cache = None


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except Exception:  # sysfs can raise odd codec/IO errors on absent sensors
        return None


def cpu_percent():
    """CPU usage since the previous call (first call: 0.2 s sample)."""
    global _prev_cpu

    def sample():
        parts = _read("/proc/stat").splitlines()[0].split()[1:]
        vals = [int(x) for x in parts]
        idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
        return idle, sum(vals)

    if _prev_cpu is None:
        _prev_cpu = sample()
        time.sleep(0.2)
    idle0, total0 = _prev_cpu
    idle1, total1 = sample()
    _prev_cpu = (idle1, total1)
    dt = total1 - total0
    return round(100.0 * (1 - (idle1 - idle0) / dt), 1) if dt > 0 else 0.0


def meminfo():
    info = {}
    for line in _read("/proc/meminfo").splitlines():
        k, v = line.split(":", 1)
        info[k] = int(v.strip().split()[0])  # kB
    total = info["MemTotal"]
    used = total - info.get("MemAvailable", info.get("MemFree", 0))
    return {
        "ram_used_gb": round(used / 1024**2, 1),
        "ram_total_gb": round(total / 1024**2, 1),
        "ram_percent": round(100.0 * used / total, 1),
    }


def gpu_percent():
    """Orin GPU load: /sys value is in tenths of a percent."""
    for path in ("/sys/devices/platform/bus@0/17000000.gpu/load",
                 "/sys/devices/gpu.0/load"):
        raw = _read(path)
        if raw is not None:
            try:
                return round(int(raw) / 10.0, 1)
            except ValueError:
                pass
    return None


def temps():
    zones = {}
    for z in glob.glob("/sys/class/thermal/thermal_zone*"):
        name = _read(os.path.join(z, "type")) or "?"
        raw = _read(os.path.join(z, "temp"))
        if raw is None:
            continue
        t = int(raw) / 1000.0
        if t <= 0:
            continue
        zones[name.replace("-thermal", "").replace("_thermal", "")] = round(t, 1)
    return zones


def disks():
    seen, out = set(), []
    for line in (_read("/proc/mounts") or "").splitlines():
        dev, mnt, fs = line.split()[:3]
        if fs not in ("ext4", "vfat", "xfs", "btrfs", "f2fs") or mnt in seen:
            continue
        if mnt.startswith("/boot"):
            continue  # boot partitions: noise on the panel
        seen.add(mnt)
        try:
            st = os.statvfs(mnt)
        except OSError:
            continue
        total = st.f_blocks * st.f_frsize
        free = st.f_bavail * st.f_frsize
        used = total - free
        if total == 0:
            continue
        out.append({
            "mountpoint": mnt,
            "fstype": fs,
            "used_gb": round(used / 1024**3, 1),
            "total_gb": round(total / 1024**3, 1),
            "percent": round(100.0 * used / total, 1),
        })
    return sorted(out, key=lambda d: d["mountpoint"])


def _crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _pzem_read(port):
    """Read PZEM-017 input registers 0..7. Returns dict or raises."""
    req = bytes([0x01, 0x04, 0x00, 0x00, 0x00, 0x08])
    req += struct.pack("<H", _crc16(req))
    with serial.Serial(port, 9600, bytesize=8, parity="N", stopbits=2, timeout=0.5) as s:
        s.reset_input_buffer()
        s.write(req)
        resp = s.read(21)  # addr, fc, count, 16 data bytes, crc x2
    if len(resp) < 21 or resp[1] != 0x04:
        raise IOError(f"bad response ({len(resp)} bytes)")
    if struct.pack("<H", _crc16(resp[:-2])) != resp[-2:]:
        raise IOError("CRC mismatch")
    regs = struct.unpack(">8H", resp[3:19])
    voltage = regs[0] * 0.01
    current = regs[1] * 0.01
    power = (regs[2] | (regs[3] << 16)) * 0.1
    return voltage, current, power


def battery():
    global _pzem_port_cache
    if serial is None:
        return {"available": False, "reason": "pyserial missing in this venv"}
    ports = [p for p in glob.glob("/dev/serial/by-id/*") if LIDAR_ID_HINT not in p]
    if _pzem_port_cache in ports:
        ports = [_pzem_port_cache] + [p for p in ports if p != _pzem_port_cache]
    if not ports:
        return {"available": False, "reason": "RS-485 dongle not plugged in"}
    for port in ports:
        try:
            voltage, current, power = _pzem_read(port)
        except Exception:
            continue
        _pzem_port_cache = port
        soc = (voltage - SOC_V_EMPTY) / (SOC_V_FULL - SOC_V_EMPTY) * 100.0
        return {
            "available": True,
            "voltage_v": round(voltage, 2),
            "percent": round(max(0.0, min(100.0, soc)), 0),
            "current_a": round(current, 2),
            "power_w": round(power, 1),
        }
    return {"available": False, "reason": "dongle seen but PZEM not answering"}


def collect():
    load_1m = os.getloadavg()[0]
    uptime = float((_read("/proc/uptime") or "0 0").split()[0])
    return {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "hostname": socket.gethostname(),
        "uptime_s": int(uptime),
        "cpu_percent": cpu_percent(),
        "gpu_percent": gpu_percent(),
        "load_1m": round(load_1m, 2),
        **meminfo(),
        "temps": temps(),
        "disks": disks(),
        "battery": battery(),
    }


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/metrics", "/"):
            self.send_error(404)
            return
        try:
            body = json.dumps(collect()).encode()
        except Exception as e:  # never die on a probe error
            body = json.dumps({"error": str(e)}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # quiet


if __name__ == "__main__":
    print(f"VECTOR stats server on 0.0.0.0:{PORT}", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()

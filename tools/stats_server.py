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
# The ONE port the PZEM lives on. The old code scanned every by-id port with
# MODBUS frames (motor bus, ESP, even the new lidar stick - its exclusion
# hint named the DEAD CP2102N): bus contention sprayed on every /metrics
# poll, all day on 26/08. Fixed port, never scan.
PZEM_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

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
    if serial is None:
        return {"available": False, "reason": "pyserial missing in this venv"}
    if not os.path.exists(PZEM_PORT):
        return {"available": False, "reason": "PZEM dongle not plugged in"}
    try:
        voltage, current, power = _pzem_read(PZEM_PORT)
    except Exception as e:
        return {"available": False, "reason": f"PZEM not answering: {e}"}
    soc = (voltage - SOC_V_EMPTY) / (SOC_V_FULL - SOC_V_EMPTY) * 100.0
    return {
        "available": True,
        "voltage_v": round(voltage, 2),
        "percent": round(max(0.0, min(100.0, soc)), 0),
        "current_a": round(current, 2),
        "power_w": round(power, 1),
    }


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


# --- sensor liveness (owner, 26/08: "une UI qui me montre TOUT sur le robot") ---
#
# PASSIVE only: we listen to the LCM bus and read /proc & /dev - never a
# serial port (the day taught what contention costs). When no stack runs the
# topics fall silent and the panel says so honestly.

_seen = {}          # family -> last wall time
_counts = {}        # family -> msgs in the current window
_FAMILIES = (("lidar_scan", ("pointcloud",)),
             ("odometry", ("/odom",)),
             ("camera", ("color_image", "depth_image")),
             ("costmap", ("global_costmap",)),
             ("drive", ("cmd_vel",)),
             ("switches", ("bump",)),
             ("sonar", ("sonar_range",)))


def _family_of(channel):
    for fam, needles in _FAMILIES:
        if any(n in channel for n in needles):
            return fam
    return None


def _lcm_listener():
    try:
        import lcm as lcmlib
    except ImportError:
        return
    def cb(channel, _data):
        fam = _family_of(channel)
        if fam:
            _seen[fam] = time.time()
            _counts[fam] = _counts.get(fam, 0) + 1
    while True:
        try:
            lc = lcmlib.LCM()
            lc.subscribe(".*", cb)
            while True:
                lc.handle_timeout(1000)
        except Exception:
            time.sleep(3.0)


def _stack_running():
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                if b"bin/dimos" in f.read():
                    return True
        except Exception:
            continue
    return False


def sensors():
    now = time.time()
    out = {"stack_running": _stack_running(),
           "ports_plugged": sorted(os.path.basename(p) for p in glob.glob("/dev/serial/by-id/*")),
           "sonar_note": "DISABLED in the stack (owner vote 26/08: bumper cushion incident)"}
    home = os.path.expanduser("~")
    pm = os.path.join(home, ".local/state/vector/persistent_map.npz")
    ko = os.path.join(home, ".local/state/vector/keepout.json")
    if os.path.exists(pm):
        out["persistent_map_saved"] = time.strftime("%H:%M:%S", time.localtime(os.path.getmtime(pm)))
    if os.path.exists(ko):
        try:
            out["zones"] = len(json.load(open(ko)).get("zones", []))
        except Exception:
            pass
    for fam, _needles in _FAMILIES:
        last = _seen.get(fam)
        out[fam] = {"alive": last is not None and now - last < 5.0,
                    "age_s": None if last is None else round(now - last, 1),
                    "msgs": _counts.get(fam, 0)}
    return out


_PANEL = """<meta http-equiv="refresh" content="2"><body style="background:#101014;color:#e8e8e2;
font-family:system-ui;padding:4vw"><h2 style="margin:0 0 3vh">VECTOR — organes</h2>
<table style="font-size:2.6vh;border-spacing:0 1vh">%s</table>
<div style="color:#777;font-size:2vh;margin-top:3vh">%s &middot; ports: %s</div></body>"""


def _panel_html():
    d = sensors()
    b = battery()
    rows = []
    def dot(ok, warn=False):
        return f'<td style="font-size:3vh;padding-right:1.5vw">{"&#128994;" if ok else ("&#128992;" if warn else "&#128308;")}</td>'
    rows.append(f"<tr>{dot(d['stack_running'])}<td>stack dimOS</td><td>{'en vol' if d['stack_running'] else 'arretee'}</td></tr>")
    labels = {"lidar_scan": "lidar C1", "odometry": "odometrie", "camera": "RealSense",
              "costmap": "carte", "drive": "commandes roues", "switches": "switchs (contacts)"}
    for fam, label in labels.items():
        st = d[fam]
        detail = ("jamais vu" if st["age_s"] is None else f"il y a {st['age_s']} s &middot; {st['msgs']} msgs")
        if fam == "switches" and st["age_s"] is None:
            detail = "aucun contact (normal)"
            rows.append(f"<tr>{dot(True, warn=True)}<td>{label}</td><td>{detail}</td></tr>")
            continue
        rows.append(f"<tr>{dot(st['alive'])}<td>{label}</td><td>{detail}</td></tr>")
    rows.append(f"<tr>{dot(False, warn=True)}<td>sonar</td><td>{d['sonar_note']}</td></tr>")
    if b.get("available"):
        ok = b["voltage_v"] > 24.0
        rows.append(f"<tr>{dot(ok)}<td>batterie</td><td>{b['voltage_v']} V &middot; {b['current_a']} A &middot; {b['percent']:.0f}%</td></tr>")
    else:
        rows.append(f"<tr>{dot(False)}<td>batterie</td><td>{b.get('reason','?')}</td></tr>")
    extra = f"carte sauvee {d.get('persistent_map_saved','?')} &middot; {d.get('zones','?')} zones"
    return _PANEL % ("".join(rows), extra, ", ".join(d["ports_plugged"]))


# One URL for the whole flight deck (owner, 26/08 21h10: "a chaque fois je
# dois tout lancer, reorganiser les fenetres... au pire une url avec les
# iframes"). The iframe sources resolve in the BROWSER on the rig: the
# cockpit through its 127.0.0.1:7780 tunnel, the rest straight over the LAN.
# Rerun (the 3D map) stays its own native window - fly.sh opens it.
_VOL = """<!doctype html><html><head><meta charset="utf-8"><title>VECTOR - vol</title>
<style>
 body{margin:0;background:#111;color:#ddd;font:13px sans-serif;height:100vh;display:flex;flex-direction:column}
 .row{flex:1;display:flex;min-height:0}
 iframe{flex:1;border:1px solid #333;background:#fff}
 .tag{position:absolute;background:#111a;padding:1px 6px;font-size:11px}
</style></head><body>
<div class="row"><iframe src="http://127.0.0.1:7780/" title="cockpit"></iframe>
<iframe src="/panel" title="organes"></iframe></div>
<div class="row"><iframe src="http://192.168.0.56:8902/" title="zones"></iframe></div>
</body></html>"""


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/vol":
            body = _VOL.encode()
            ctype = "text/html"
        elif self.path in ("/panel", "/"):
            try:
                body = _panel_html().encode()
            except Exception as e:
                body = f"<pre>panel error: {e}</pre>".encode()
            ctype = "text/html"
        elif self.path == "/metrics":
            try:
                data = collect()
                data["sensors"] = sensors()
                body = json.dumps(data).encode()
            except Exception as e:  # never die on a probe error
                body = json.dumps({"error": str(e)}).encode()
            ctype = "application/json"
        else:
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *args):
        pass  # quiet


if __name__ == "__main__":
    import threading
    threading.Thread(target=_lcm_listener, daemon=True).start()
    print(f"VECTOR stats server on 0.0.0.0:{PORT} (/panel = organes, /metrics = JSON)", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()

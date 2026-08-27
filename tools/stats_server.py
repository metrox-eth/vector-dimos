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
# poll, all day on 2026-08-26. Fixed port, never scan.
PZEM_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

_prev_cpu = None  # (idle, total)
_prev_cores = None  # [(idle, total)] per core - per-core view (added 2026-08-27)
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


def cpu_per_core():
    """Per-core usage since the previous call - same delta method as cpu_percent.
    Ordered cpu0..cpuN; the panel draws one small bar per core so a single
    saturated worker reads differently from a spread load."""
    global _prev_cores

    def sample():
        out = []
        for line in _read("/proc/stat").splitlines()[1:]:
            if not line.startswith("cpu"):
                break
            vals = [int(x) for x in line.split()[1:]]
            idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
            out.append((idle, sum(vals)))
        return out

    if _prev_cores is None:
        _prev_cores = sample()
        time.sleep(0.2)
    prev = _prev_cores
    cur = sample()
    _prev_cores = cur
    res = []
    for (i0, t0), (i1, t1) in zip(prev, cur):
        dt = t1 - t0
        res.append(round(100.0 * (1 - (i1 - i0) / dt), 1) if dt > 0 else 0.0)
    return res


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


_gpu_samples = []   # (t, pct) - 3 s sliding average (measured 2026-08-27: the
                    # instantaneous read missed the 4 ms CUDA bursts and the
                    # panel oscillated between 0% and 13% with no meaning)


def _gpu_read_once():
    """Orin GPU load: /sys value is in tenths of a percent. INSTANTANE."""
    for path in ("/sys/devices/platform/bus@0/17000000.gpu/load",
                 "/sys/devices/gpu.0/load"):
        raw = _read(path)
        if raw is not None:
            try:
                return int(raw) / 10.0
            except ValueError:
                pass
    return None


def _gpu_sampler():
    """Continuous 20 Hz sampling: the duty cycle of a mapper that works in 4 ms
    bursts only exists as an AVERAGE, never as a single sample."""
    while True:
        v = _gpu_read_once()
        now = time.time()
        if v is not None:
            _gpu_samples.append((now, v))
            while _gpu_samples and now - _gpu_samples[0][0] > 3.0:
                _gpu_samples.pop(0)
        time.sleep(0.05)


def gpu_percent():
    if not _gpu_samples:
        return _gpu_read_once()
    return round(sum(v for _, v in _gpu_samples) / len(_gpu_samples), 1)


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
        "cpu_per_core": cpu_per_core(),
        "gpu_percent": gpu_percent(),
        "load_1m": round(load_1m, 2),
        **meminfo(),
        "temps": temps(),
        "disks": disks(),
        "battery": battery(),
    }


# --- sensor liveness: one view showing EVERYTHING on the robot at once -------
#
# PASSIVE only: we listen to the LCM bus and read /proc & /dev - never a
# serial port (a day of bus contention taught what that costs). When no stack
# runs the topics fall silent and the panel says so honestly.

_seen = {}          # family -> last wall time
_reloc_state = ["?"]   # frame_id of the last reloc_frame (reloc:persistent/fresh/searching)
_watcher_last = [0.0]  # last /metrics?watcher=iris request (the monitoring vigil)
_cuda_state = [None]   # {"torch": bool, "open3d": bool} - probed once at startup


def _probe_cuda() -> None:
    """One-shot, in a thread: the imports cost seconds on the Jetson and the
    answer cannot change while the process lives. The panel row exists because
    the GPU sat at 0% for days while the CPU burned and nobody could SEE that
    the wheels were CPU-only (2026-08-27)."""
    state = {"torch": False, "open3d": False}
    try:
        import torch
        state["torch"] = bool(torch.cuda.is_available())
    except Exception:
        pass
    try:
        import open3d.core as o3c
        state["open3d"] = bool(o3c.cuda.is_available())
    except Exception:
        pass
    _cuda_state[0] = state
_counts = {}        # family -> msgs in the current window
_FAMILIES = (("lidar_scan", ("pointcloud",)),
             ("odometry", ("/odom",)),
             ("imu", ("imu",)),          # the rotation prior since 2026-08-26 - a monitored organ now
             ("camera", ("color_image", "depth_image")),
             ("costmap", ("global_costmap",)),
             ("drive", ("cmd_vel",)),
             ("switches", ("bump",)),
             ("sonar", ("sonar_range",)),
             ("reloc", ("reloc_frame",)))
GAMEPAD_DEV = "/dev/input/js0"
_gamepad_last_input = [0.0]      # wall time of the last REAL pad event (radio proof)


def _gamepad_listener():
    """Reads js0 events (8-byte records) to timestamp real pad ACTIVITY.
    The device existing only proves the DONGLE is plugged in, not that a pad is
    paired and talking - the row used to go green on the dongle alone and lied
    (2026-08-27). Multiple readers are fine on a joystick device."""
    import struct
    while True:
        try:
            with open(GAMEPAD_DEV, "rb") as f:
                while True:
                    ev = f.read(8)
                    if not ev:
                        break
                    _t, _v, ev_type, _num = struct.unpack("IhBB", ev)
                    if not (ev_type & 0x80):        # ignore init events
                        _gamepad_last_input[0] = time.time()
        except Exception:
            time.sleep(3.0)


def _family_of(channel):
    for fam, needles in _FAMILIES:
        if any(n in channel for n in needles):
            return fam
    return None


_bus_seen = {"lcm": 0.0, "zenoh": 0.0}   # last message seen per bus (zenoh migration, 2026-08-27)


def _on_bus_message(bus, channel, data):
    """Logic shared by BOTH buses: organ families + relocalization state."""
    _bus_seen[bus] = time.time()
    fam = _family_of(channel)
    if fam:
        _seen[fam] = time.time()
        _counts[fam] = _counts.get(fam, 0) + 1
    if "reloc_frame" in channel:
        try:
            from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
            _reloc_state[0] = str(PoseStamped.lcm_decode(data).frame_id).replace("reloc:", "")
        except Exception:
            pass


def _lcm_listener():
    try:
        import lcm as lcmlib
    except ImportError:
        return
    def cb(channel, data):
        _on_bus_message("lcm", channel, data)
    while True:
        try:
            lc = lcmlib.LCM()
            lc.subscribe(".*", cb)
            while True:
                lc.handle_timeout(1000)
        except Exception:
            time.sleep(3.0)


def _zenoh_listener():
    """The panel listens on BOTH buses during the zenoh migration (2026-08-27):
    whichever transport the stack uses, the organ rows light up - no flag day.
    Same host as the stack, so default loopback discovery is enough. The zenoh
    key is normalised into a channel name (leading slash) so the same organ
    needles match on either bus."""
    try:
        import zenoh
    except ImportError:
        return
    def cb(sample):
        try:
            channel = "/" + str(sample.key_expr).lstrip("/")
            data = bytes(sample.payload)
        except Exception:
            return
        _on_bus_message("zenoh", channel, data)
    while True:
        try:
            s = zenoh.open(zenoh.Config())
            s.declare_subscriber("**", cb)
            while True:
                time.sleep(1.0)
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
           "sonar_note": "disabled"}
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
    import glob as _glob
    import os as _os
    import subprocess as _sp
    runs = sorted(_glob.glob(_os.path.expanduser("~/.local/state/dimos/logs/*-vector-dimos-explore")), key=_os.path.getmtime)
    run_id = _os.path.basename(runs[-1]) if runs else None
    try:
        ss_out = _sp.run(["ss", "-tn", "state", "established", "( sport = :9877 )"],
                         capture_output=True, text=True, timeout=3).stdout
        rerun_connected = len([ln for ln in ss_out.splitlines() if ":9877" in ln]) > 0
    except Exception:
        rerun_connected = False
    garde = any("garde_vitesse" in (open(f"/proc/{p}/cmdline", "rb").read().decode(errors="replace") if _os.path.isdir(f"/proc/{p}") else "")
                for p in _os.listdir("/proc") if p.isdigit()) if _os.path.isdir("/proc") else False
    out["software"] = {"stack_running": out.get("stack_running", False),
                       "run_id": run_id,
                       "rerun_connected": rerun_connected,
                       "garde_vitesse": garde,
                       "reloc_state": _reloc_state[0]}
    # The panel is also the monitoring agent's instrument: whoever operates the
    # stack must be able to see what it is doing. The vigil polls
    # /metrics?watcher=iris and this row proves someone is actually watching.
    w_last = _watcher_last[0]
    w_age = (time.time() - w_last) if w_last else None
    out["software"]["monitoring"] = {"alive": w_age is not None and w_age < 45.0,
                                     "age_s": round(w_age, 1) if w_age is not None else None}
    out["software"]["cuda"] = _cuda_state[0]
    # which bus carries the stack (zenoh migration): fresh = message < 5 s
    now_b = time.time()
    lcm_ok = now_b - _bus_seen["lcm"] < 5.0
    zen_ok = now_b - _bus_seen["zenoh"] < 5.0
    out["software"]["bus"] = ("les deux" if lcm_ok and zen_ok else
                              "zenoh" if zen_ok else
                              "lcm" if lcm_ok else "aucun")
    dongle = _os.path.exists(GAMEPAD_DEV)
    last = _gamepad_last_input[0]
    age = (time.time() - last) if last else None
    out["gamepad"] = {"alive": bool(dongle and age is not None and age < 120.0),
                      "age_s": round(age, 1) if age is not None else None,
                      "msgs": 1 if dongle else 0}
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
    labels = {"lidar_scan": "lidar C1", "odometry": "odometrie", "imu": "IMU (gyro)", "gamepad": "manette", "camera": "RealSense",
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


# One URL for the whole flight deck, so a run does not start with launching
# four things and rearranging windows. The iframe sources resolve in the
# BROWSER on the rig: the cockpit through its 127.0.0.1:7780 tunnel, the rest
# straight over the LAN. Rerun (the 3D map) stays its own native window -
# fly.sh opens it.
#
# OPEN THIS PAGE AS http://127.0.0.1:8900/vol (through the SSH tunnel), never
# by the LAN address: WebTransport in the cockpit iframe requires a SECURE
# CONTEXT, and an iframe is only secure if every ANCESTOR is - localhost
# qualifies, 192.168.0.56 does not ("Not a secure context", observed
# 2026-08-26 21:55).
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
        elif self.path.startswith("/metrics"):
            if "watcher=iris" in self.path:
                _watcher_last[0] = time.time()
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
    threading.Thread(target=_zenoh_listener, daemon=True).start()
    threading.Thread(target=_gpu_sampler, daemon=True).start()
    threading.Thread(target=_gamepad_listener, daemon=True).start()
    threading.Thread(target=_probe_cuda, daemon=True).start()
    print(f"VECTOR stats server on 0.0.0.0:{PORT} (/panel = organes, /metrics = JSON)", flush=True)
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()

"""Cold bench for tools/stats_server.py: a process that dies between the /proc
listing and the read must not take the organ panel down.

Rule #2: known input -> known output. The input is the RACE ITSELF, made
deterministic - /proc's listing names a ghost pid, isdir still says yes, and the
read raises FileNotFoundError, exactly what a process that just exited does.
The output is a complete sensors() dict and a /metrics body that still carries
the word "sensors" (fly.sh gate 5 greps for it). No rover needed. Groups:

  A. pid fantome - the scan swallows it: sensors() returns, software block intact
  B. voit quand  - with the ghost FIRST in the listing, a live process carrying
     meme          the needle is STILL found (the guard skips, it does not stop)
  C. /metrics    - the real Handler over a real socket under the same fault:
                   HTTP 200 AND "sensors" in the body (pre-fix: 200 with
                   {"error": ...} and no organs -> 'NO ORGAN PANEL - no flight')
  D. le scan     - the guarded helper both sees a live cmdline and says no to a
                   needle nobody carries
  E. un seul     - two threads call battery() at the same instant against a
     maitre        counting serial stub: never two of them inside the MODBUS
                   transaction, and both read the same 24.50 V (pre-fix: 2)
  F. le cache    - a second call within PZEM_CACHE_S opens the port ZERO times
                   and answers the same volts; once the entry ages out, the
                   next call goes back to the port

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_stats_sensors_cold.py
"""

import json
import os
import struct
import subprocess
import sys
import tempfile
import threading
import time
import types
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import stats_server  # noqa: E402

# This bench must never write a MODBUS frame on a real port: a CH340 on the dev
# rig is NOT the rover's shunt. battery() only opens the port if it exists.
stats_server.PZEM_PORT = "/dev/does-not-exist/pzem-cold-bench"

NEEDLE = "vector-cold-bench-garde_vitesse"  # contains "garde_vitesse" on purpose
ABSENT = "vector-cold-bench-needle-nobody-carries"
GHOST = "999999"                            # a pid that is not there
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


_real_listdir = os.listdir
_real_isdir = os.path.isdir


def _ghost_listdir(path, *a, **k):
    # the ghost is named FIRST: the scan meets it before any live process
    return [GHOST] + _real_listdir("/proc") if path == "/proc" else _real_listdir(path, *a, **k)


def _ghost_isdir(path):
    return True if path == f"/proc/{GHOST}" else _real_isdir(path)


class ghost_pid:
    """A pid that is in the listing and already gone when it is read."""

    def __enter__(self):
        os.listdir = _ghost_listdir
        os.path.isdir = _ghost_isdir
        return self

    def __exit__(self, *exc):
        os.listdir = _real_listdir
        os.path.isdir = _real_isdir
        return False


print("stats_server - le panneau des organes sous un pid fantome")
assert not _real_isdir(f"/proc/{GHOST}"), f"pid {GHOST} is alive, pick another ghost"

proc = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)", NEEDLE],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    deadline = time.monotonic() + 5   # the child is ready when /proc carries its cmdline
    while time.monotonic() < deadline:
        try:
            with open(f"/proc/{proc.pid}/cmdline", "rb") as f:
                if NEEDLE.encode() in f.read():
                    break
        except OSError:
            pass
        time.sleep(0.05)

    # --- A + B. the ghost pid ------------------------------------------------
    with ghost_pid():
        try:
            s, raised = stats_server.sensors(), None
        except Exception as exc:  # noqa: BLE001
            s, raised = None, exc
    check("un pid disparu entre le listing et la lecture: sensors() rend la main",
          raised is None, "" if raised is None else f"{type(raised).__name__}: {raised}")
    sw = (s or {}).get("software", {})
    check("... le bloc software est complet",
          isinstance(sw.get("garde_vitesse"), bool) and "run_id" in sw and "reloc_state" in sw,
          str(sorted(sw))[:90])
    check("... et le processus vivant DERRIERE le fantome est quand meme vu",
          sw.get("garde_vitesse") is True)

    # --- C. /metrics never 500s and never loses "sensors" ---------------------
    srv = ThreadingHTTPServer(("127.0.0.1", 0), stats_server.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{srv.server_address[1]}/metrics"
    with ghost_pid():
        try:
            with urllib.request.urlopen(url, timeout=10) as r:
                code, body = r.status, r.read().decode()
        except urllib.error.HTTPError as e:
            code, body = e.code, e.read().decode()
        except Exception as e:  # noqa: BLE001 - a dead connection is a failure too
            code, body = None, f"{type(e).__name__}: {e}"
    srv.shutdown()
    srv.server_close()
    check("/metrics repond 200 sous le meme defaut (jamais 500)", code == 200, str(code))
    check('... et le corps porte toujours "sensors" (le grep de fly.sh 5/7)',
          '"sensors"' in body, body[:110])
    d = json.loads(body) if body.startswith("{") else {}
    check("... avec les organes et la charge dedans",
          "software" in d.get("sensors", {}) and isinstance(d.get("load_1m"), float),
          f"load {d.get('load_1m')}")

    # --- D. the guarded scan itself ------------------------------------------
    scan = getattr(stats_server, "_proc_scan", None)
    if scan is None:
        check("le scan /proc garde existe (_proc_scan)", False, "absent: lecture /proc non gardee")
    else:
        check("un besoin porte par un processus vivant -> True", scan(NEEDLE.encode()) is True)
        check("un besoin qu'aucun processus ne porte -> False", scan(ABSENT.encode()) is False)
finally:
    proc.kill()
    proc.wait(timeout=10)


# --- E + F. one MODBUS master on the shunt, and a cache that spares it -------
#
# Known input: a stub PZEM whose registers encode 24.50 V / 1.20 A / 29.4 W.
# Known output: battery() in physical units (24.5 V -> 65% on the 20.0/26.91 V
# scale) AND two counters no real port can give us - how many callers were
# inside the transaction at once, and how many times the port was opened.

V_IN, A_IN, W_IN = 24.50, 1.20, 29.4
PCT_IN = 65.0  # (24.50 - 20.0) / (26.91 - 20.0) * 100, rounded


def _pzem_frame(volts, amps, watts):
    """The 21-byte reply a PZEM-017 sends for a read of registers 0..7."""
    deci_w = round(watts * 10)
    regs = [round(volts * 100), round(amps * 100), deci_w & 0xFFFF, deci_w >> 16, 0, 0, 0, 0]
    head = bytes([0x01, 0x04, 0x10]) + struct.pack(">8H", *regs)
    return head + struct.pack("<H", stats_server._crc16(head))


FRAME = _pzem_frame(V_IN, A_IN, W_IN)


class CountingSerial:
    """A PZEM that answers the frame above and counts who is on the wire."""

    guard = threading.Lock()  # protects the counters ONLY, never the transaction
    opens = 0
    inside = 0
    max_inside = 0

    @classmethod
    def reset(cls):
        cls.opens = cls.inside = cls.max_inside = 0

    def __init__(self, port, *a, **k):
        with CountingSerial.guard:
            CountingSerial.opens += 1
            CountingSerial.inside += 1
            CountingSerial.max_inside = max(CountingSerial.max_inside, CountingSerial.inside)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        with CountingSerial.guard:
            CountingSerial.inside -= 1
        return False

    def reset_input_buffer(self):
        pass

    def write(self, data):
        pass

    def read(self, n):
        time.sleep(0.05)  # a real transaction is ~10 ms: wide enough to collide
        return FRAME[:n]


CACHE = getattr(stats_server, "_pzem_cache", None)  # absent pre-fix


def arm_pzem(cache_s):
    CountingSerial.reset()
    stats_server.PZEM_CACHE_S = cache_s
    if CACHE is not None:
        CACHE[:] = [0.0, None]


print("stats_server - un seul maitre MODBUS sur le shunt")
port_file = tempfile.NamedTemporaryFile(prefix="pzem-stub-", delete=False)
port_file.close()
log_file = tempfile.NamedTemporaryFile(prefix="battery-log-", suffix=".csv", delete=False)
log_file.close()
real_serial, real_port, real_log = stats_server.serial, stats_server.PZEM_PORT, stats_server.BATTERY_LOG
stats_server.serial = types.SimpleNamespace(Serial=CountingSerial)
stats_server.PZEM_PORT = port_file.name       # exists, so battery() goes to the stub
stats_server.BATTERY_LOG = log_file.name      # never the rover's flight recorder
try:
    if CACHE is None:
        check("le cache PZEM existe (_pzem_cache)", False, "absent: transaction non gardee")

    # --- E. two threads at the same instant, cache disabled so both must read --
    arm_pzem(0.0)
    ready = threading.Barrier(2)
    got = {}

    def one(name):
        ready.wait(timeout=5)
        got[name] = stats_server.battery()

    threads = [threading.Thread(target=one, args=(n,)) for n in ("a", "b")]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=15)

    check("deux appels simultanes: jamais 2 dans la transaction PZEM",
          CountingSerial.max_inside == 1, f"max {CountingSerial.max_inside} en meme temps")
    check("... les deux ont bien parle au port (cache desactive)",
          CountingSerial.opens == 2, f"{CountingSerial.opens} ouvertures")
    check(f"... et chacun lit {V_IN} V / {PCT_IN}% (roundtrip physique)",
          all(got.get(n, {}).get("voltage_v") == V_IN and got.get(n, {}).get("percent") == PCT_IN
              for n in ("a", "b")), str(got)[:120])
    check(f"... avec {A_IN} A et {W_IN} W",
          got.get("a", {}).get("current_a") == A_IN and got.get("a", {}).get("power_w") == W_IN,
          str(got.get("a"))[:100])

    # --- F. the 1 s cache: the second caller does not touch the port ----------
    arm_pzem(1.0)
    first = stats_server.battery()
    after_first = CountingSerial.opens
    second = stats_server.battery()
    check("une lecture -> une ouverture du port", after_first == 1, f"{after_first}")
    check("un 2e appel sous PZEM_CACHE_S: 0 ouverture de plus",
          CountingSerial.opens == after_first, f"{CountingSerial.opens} au total")
    check("... et il rend la MEME tension, pas un trou",
          second.get("voltage_v") == first.get("voltage_v") == V_IN, str(second)[:100])
    if CACHE is not None:
        CACHE[0] -= 2.0  # age the entry past the window, no sleep needed
        stats_server.battery()
        check("une fois le cache perime, l'appel suivant retourne au port",
              CountingSerial.opens == after_first + 1, f"{CountingSerial.opens} au total")
finally:
    stats_server.serial, stats_server.PZEM_PORT = real_serial, real_port
    stats_server.BATTERY_LOG = real_log
    os.unlink(port_file.name)
    os.unlink(log_file.name)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

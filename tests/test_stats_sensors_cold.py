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

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_stats_sensors_cold.py
"""

import json
import os
import subprocess
import sys
import threading
import time
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

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

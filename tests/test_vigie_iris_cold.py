"""Cold bench for tools/vigie_iris.py: a 200 body with no "sensors" key is a
SICK PANEL, not a dead robot.

stats_server answers HTTP 200 with {"error": ...} whenever a probe fails. Read
as organs, that hole printed the loudest line the vigil has - stack OFF, garde
off, viewer OFF, every organ MORT - in the middle of a healthy flight, then
flipped everything back 15 s later, and reset the RAM hysteresis on the way
(audit 2026-08-28). Rule #2: known bodies in, known lines out. The real main()
loop is driven over a scripted /metrics; no network, no rover. Groups:

  A. corps sain   - a known body -> a known state (up / armee / OK / charge ok)
  B. corps troue  - 200 without "sensors" -> {"panneau": "EN ERREUR"} alone: no
                    organ, no invented OFF
  C. une ligne    - main() over sain -> troue -> sain prints exactly ONE line for
                    the hole, and neither it nor the recovery says MORT
  D. hysterese    - a CRITIQUE memory bucket SURVIVES the hole (the old path read
                    ram_percent as 0.0 and reset it to ok)
  E. injoignable  - a transport failure still says INJOIGNABLE (unchanged)
  F. charge       - load 14.0 -> 'charge HAUTE' (the bucket still works)

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_vigie_iris_cold.py
"""

import io
import json
import sys
import urllib.request
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import vigie_iris  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def healthy(load=3.0, ram=40.0):
    return {"load_1m": load, "ram_percent": ram,
            "sensors": {"software": {"stack_running": True, "reloc_state": "reloc:persistent",
                                     "garde_vitesse": True, "rerun_connected": True},
                        "lidar_scan": {"alive": True}, "odometry": {"alive": True}}}


TROUE = {"error": "[Errno 2] No such file or directory: '/proc/12345/cmdline'"}


class FakeResp:
    def __init__(self, payload):
        self._b = json.dumps(payload).encode()

    def read(self, *a):
        return self._b

    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


_real_urlopen = urllib.request.urlopen
_real_sleep = vigie_iris.time.sleep


class Stop(Exception):
    """out of scripted answers - ends main() from OUTSIDE snapshot's except"""


def scripted(bodies):
    seq = list(bodies)

    def fake_urlopen(url, timeout=None):
        item = seq.pop(0)
        if isinstance(item, Exception):
            raise item
        return FakeResp(item)

    def fake_sleep(_s):
        if not seq:
            raise Stop()
    return fake_urlopen, fake_sleep, seq


def snap(body, prev_bucket="charge ok"):
    fake_urlopen, _, _ = scripted([body])
    urllib.request.urlopen = fake_urlopen
    try:
        return vigie_iris.snapshot(prev_bucket)
    finally:
        urllib.request.urlopen = _real_urlopen


def run_main(bodies):
    fake_urlopen, fake_sleep, _ = scripted(bodies)
    urllib.request.urlopen = fake_urlopen
    vigie_iris.time.sleep = fake_sleep
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            vigie_iris.main()
    except Stop:
        pass
    finally:
        urllib.request.urlopen = _real_urlopen
        vigie_iris.time.sleep = _real_sleep
    return buf.getvalue().strip().splitlines()


print("vigie_iris - un panneau malade n'est pas un robot mort")

# --- A. a known body -> a known state -----------------------------------------
state, load = snap(healthy())
check("corps sain: stack up, garde armee, viewer connecte, organes OK",
      state == {"stack": "up", "frame": "reloc:persistent", "garde": "armee", "viewer": "connecte",
                "charge": "charge ok", "memoire": "ok", "lidar_scan": "OK", "odometry": "OK"}, str(state))
check("... et la charge remontee telle quelle", load == 3.0, str(load))

# --- F. the load bucket still works -------------------------------------------
state, load = snap(healthy(load=14.0))
check("charge 14.0 -> 'charge HAUTE'", state["charge"] == "charge HAUTE", state["charge"])

# --- B + D. the holed body ----------------------------------------------------
vigie_iris._mem_prev[0] = "CRITIQUE"
state, load = snap(TROUE)
check("corps troue (200 sans \"sensors\") -> etat panneau distinct",
      state == {"panneau": "EN ERREUR"}, str(state))
check("... aucun organe declare MORT, aucun OFF invente",
      "MORT" not in str(state) and "OFF" not in str(state) and load is None)
check("... et l'hysterese memoire CRITIQUE survit au trou",
      vigie_iris._mem_prev[0] == "CRITIQUE", vigie_iris._mem_prev[0])
vigie_iris._mem_prev[0] = "ok"

# --- E. unreachable is still unreachable --------------------------------------
state, load = snap(OSError("connection refused"))
check("panneau injoignable -> INJOIGNABLE (inchange)", state == {"panneau": "INJOIGNABLE"}, str(state))

# --- C. the real loop: sain -> troue -> sain ----------------------------------
vigie_iris._mem_prev[0] = "ok"
lines = run_main([healthy(), TROUE, healthy()])
check("main() sur sain -> troue -> sain: 3 lignes (armement, trou, retour)",
      len(lines) == 3, " || ".join(lines)[:160])
check("... le trou tient en UNE ligne d'etat panneau",
      len(lines) > 1 and lines[1] == "panneau: ? -> EN ERREUR", lines[1] if len(lines) > 1 else "?")
check("... jamais la fausse urgence (aucun MORT, aucun -> OFF)",
      all("MORT" not in ln and "-> OFF" not in ln for ln in lines[1:]), " || ".join(lines[1:])[:160])

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

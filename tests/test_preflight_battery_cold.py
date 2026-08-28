"""Cold bench for tools/preflight.py check_battery: the flight check reads the
battery through the process that already OWNS the shunt.

vector-stats is 'always up' and polls the PZEM on every /metrics and /panel hit;
preflight opening the same RS-485 link makes two MODBUS masters eat each other's
frames, and a perfectly healthy pack reads 'KO shunt muet' -> 'HARDWARE KO - no
flight' (audit 2026-08-28). Rule #2: known input -> known output in VOLTS.
pymodbus is stubbed by a tap that counts port openings, so this bench proves not
only the reading but the SILENCE on the wire. No rover, no serial. Groups:

  A. panneau debout - /metrics says 25.53 V while the wire is contended (the tap
                      returns a corrupted frame): verdict OK 25.53 V, and the
                      port is opened ZERO times
  B. panneau mort   - connection refused: the serial fallback runs, exactly ONE
                      client, right port at 9600 8N2, registers -> 25.53 V
  C. shunt muet     - service up but battery unavailable: KO with the panel's
                      own reason, and still zero openings (a live vector-stats
                      keeps the port to itself)

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_preflight_battery_cold.py
"""

import io
import json
import sys
import types
import urllib.error
import urllib.request
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

PZEM = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"  # the shunt, spelled out here on purpose
OPENS = []          # every ModbusSerialClient built = one master on the bus
CORRUPT = [False]   # the collision symptom: a reply that is not a register frame


class FakeModbus:
    def __init__(self, **kw):
        OPENS.append(kw)

    def connect(self):
        return True

    def read_input_registers(self, addr, count, unit=1):
        assert (addr, count, unit) == (0x0000, 4, 1), (addr, count, unit)
        if CORRUPT[0]:
            return types.SimpleNamespace(message="Modbus Error: bad frame")  # no .registers
        return types.SimpleNamespace(registers=[2553, 120, 306, 0])  # 25.53 V, 1.20 A, 30.6 W

    def close(self):
        pass


# pymodbus is not in this venv (and 3.x has no client.sync anyway): stub the
# three package levels `from pymodbus.client.sync import ...` walks.
_sync = types.ModuleType("pymodbus.client.sync")
_sync.ModbusSerialClient = FakeModbus
_client = types.ModuleType("pymodbus.client")
_client.sync = _sync
_pm = types.ModuleType("pymodbus")
_pm.client = _client
sys.modules.update({"pymodbus": _pm, "pymodbus.client": _client, "pymodbus.client.sync": _sync})

import preflight  # noqa: E402


class _NoSerial:
    def Serial(self, *a, **k):
        raise AssertionError("a cold bench never opens a real serial port")


preflight.serial = _NoSerial()

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


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
SEEN_URLS = []


def run_case(answer, corrupt=False):
    """answer: a /metrics payload, or an exception = service down."""
    def fake_urlopen(url, timeout=None):
        SEEN_URLS.append(url)
        if isinstance(answer, Exception):
            raise answer
        return FakeResp(answer)

    OPENS.clear()
    CORRUPT[0] = corrupt
    preflight.RESULTS.clear()
    urllib.request.urlopen = fake_urlopen
    buf = io.StringIO()
    try:
        with redirect_stdout(buf):
            preflight.check_battery()
    finally:
        urllib.request.urlopen = _real_urlopen
    return preflight.RESULTS[:], buf.getvalue().strip().replace("\n", " | "), list(OPENS)


LIVE = {"battery": {"available": True, "voltage_v": 25.53, "percent": 80.0,
                    "current_a": 1.2, "power_w": 30.6}, "sensors": {}}
MUTE = {"battery": {"available": False, "reason": "PZEM not answering: bad response (0 bytes)"}}
DOWN = urllib.error.URLError(ConnectionRefusedError(111, "Connection refused"))

print("preflight check_battery - la fin de la contention sur le shunt")

# --- A. vector-stats is up, the wire is contended -----------------------------
res, out, opens = run_case(LIVE, corrupt=True)
check("panneau debout: verdict OK sur la tension du panneau", res == [(True, "batterie")], str(res))
check("... 25.53 V, 1.20 A, 30.6 W lus via vector-stats",
      "25.53 V, 1.20 A, 30.6 W" in out and "vector-stats" in out, out)
check("... et le shunt n'est JAMAIS ouvert par le preflight (0 maitre MODBUS)",
      opens == [], f"{len(opens)} ouverture(s)")
check("... le panneau est bien interroge, sans se faire passer pour la vigie (pas de watcher=)",
      SEEN_URLS[-1:] == ["http://127.0.0.1:8900/metrics"], str(SEEN_URLS[-1:]))

# --- B. vector-stats is down: the port is ours --------------------------------
res, out, opens = run_case(DOWN)
check("panneau mort: repli serie, une seule ouverture", len(opens) == 1, f"{len(opens)} ouverture(s)")
check("... sur le bon port, 9600 8N2",
      opens and opens[0].get("port") == PZEM and opens[0].get("baudrate") == 9600
      and opens[0].get("stopbits") == 2 and opens[0].get("parity") == "N", str(opens[:1])[:120])
check("... registres 2553/120/306 -> 25.53 V, 1.20 A, 30.6 W en direct",
      res == [(True, "batterie")] and "25.53 V, 1.20 A, 30.6 W" in out and "directe" in out, out)

# --- C. vector-stats is up and the shunt is genuinely mute --------------------
res, out, opens = run_case(MUTE)
check("shunt muet cote panneau: KO, avec la raison du panneau",
      res == [(False, "shunt muet")] and "bad response (0 bytes)" in out, out)
check("... et toujours zero ouverture (le service vivant garde le port)",
      opens == [], f"{len(opens)} ouverture(s)")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

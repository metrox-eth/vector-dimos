"""Cold bench for tools/preflight_nav.py check_maps: the keep-out file that
flies is the keep-out file the preflight accepts.

persistent_map.load_keepouts() takes both shapes the zone file has ever had -
the `{"frame": ..., "zones": [...]}` document AND a bare list. The preflight
re-parsed the file next to the loader and called `.get` on that bare list, so a
hand-edited (or older-generation) file the stack enforces perfectly well
printed "KO keepout.json illisible", exit 1, and fly.sh gate 2/7 announced
"NAV KO - no flight" (audit 2026-08-28). One loader, one contract. Rule #2:
known input -> known output in METRES. No rover, no map, no serial. Groups:

  A. liste nue     - 1 forbidden rect + 1 legacy no_slip -> verdict OK,
                     "1 active(s)", rect back as 1.0..3.0 x 2.0..4.5 m
  B. document      - the SAME zones under {"frame", "zones"} -> the SAME
                     verdict and the same metres: the shape must not matter
  C. corrompu      - truncated JSON, then a zone with an unknown type -> KO,
                     the flight is refused (a broken file still gets to say no)
  D. pas de zones  - no file at all -> OK, 0 active(s), no flight refused

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_preflight_nav_cold.py
"""

import io
import json
import os
import sys
import tempfile
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import preflight_nav  # noqa: E402

from vector_dimos import persistent_map  # noqa: E402

# check_maps also probes the persistent map and the checkpoints: point both at
# an empty scratch dir so the only verdict under test is the zone one.
TMP = tempfile.mkdtemp(prefix="preflight_nav_cold_")
persistent_map.MAP_PATH = os.path.join(TMP, "no_such_map.npz")
persistent_map.CHECKPOINT_DIR = os.path.join(TMP, "checkpoints")
persistent_map.KEEPOUT_PATH = os.path.join(TMP, "keepout.json")

# the bathroom, in metres, written x1 < x0 nowhere: what comes back must be
# byte for byte these numbers
RECT = {"label": "wc", "type": "forbidden", "x0": 1.0, "y0": 2.0, "x1": 3.0, "y1": 4.5}
LEGACY = {"label": "tapis", "type": "no_slip_reflex",  # died 26/08, old files carry it
          "x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 1.0}
BOUNDS_M = (1.0, 2.0, 3.0, 4.5)

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def run_case(payload):
    """payload: the file's content (a str lands raw), None = no file at all.

    Returns (last verdict, the printed line, the labels that would abort the
    flight) - `ko` is exactly what main() turns into exit 1.
    """
    if payload is None:
        if os.path.isfile(persistent_map.KEEPOUT_PATH):
            os.remove(persistent_map.KEEPOUT_PATH)
    else:
        body = payload if isinstance(payload, str) else json.dumps(payload)
        with open(persistent_map.KEEPOUT_PATH, "w") as fh:
            fh.write(body)
    preflight_nav.RESULTS.clear()
    buf = io.StringIO()
    with redirect_stdout(buf):
        preflight_nav.check_maps()
    lines = [ln.strip() for ln in buf.getvalue().splitlines() if "zones keep-out" in ln or "keepout" in ln]
    ko = [label for ok, label in preflight_nav.RESULTS if not ok]
    return preflight_nav.RESULTS[-1], lines[-1] if lines else "", ko


def zone_metres():
    """The loader's own reading of the file on disk, in metres."""
    zones = persistent_map.load_keepouts(persistent_map.KEEPOUT_PATH)
    return [persistent_map.zone_bounds(z) for z in zones]


print("preflight_nav check_maps - le contrat des formes appartient au loader")

# --- A. bare list: the shape the audit refused --------------------------------
res, line, ko = run_case([RECT, LEGACY])
check("liste nue: verdict OK (le vol n'est plus refuse)", res[0] is True and not ko, f"{res} ko={ko}")
check("... 2 entrees dont 1 legacy -> 1 active(s)", res[1] == "zones keep-out" and "1 active(s)" in line, line)
check("... et la zone revient 1.0..3.0 x 2.0..4.5 m", zone_metres() == [BOUNDS_M], str(zone_metres()))

# --- B. document shape: same file, same answer --------------------------------
res_doc, line_doc, ko_doc = run_case({"frame": "flat_20260826", "zones": [RECT, LEGACY]})
check("forme document: meme verdict que la liste nue", (res_doc, ko_doc) == (res, ko), f"{res_doc} vs {res}")
check("... meme ligne imprimee", line_doc == line, f"{line_doc!r} vs {line!r}")
check("... memes metres", zone_metres() == [BOUNDS_M], str(zone_metres()))

# --- C. broken files are still refused ----------------------------------------
res, line, ko = run_case('{"zones": [{"label": "wc",')
check("JSON tronque: KO keepout.json illisible", res[0] is False and res[1] == "keepout.json illisible", str(res))
check("... le vol est refuse", ko == ["keepout.json illisible"], str(ko))

res, line, ko = run_case([{"label": "wc", "type": "poubelle", "x0": 0.0, "y0": 0.0, "x1": 1.0, "y1": 1.0}])
check("type de zone inconnu: KO aussi (le loader dit non)", res[0] is False and ko, f"{res} ko={ko}")

# --- D. no zone file: a fresh house is not a fault ----------------------------
res, line, ko = run_case(None)
check("aucun fichier: verdict OK, 0 active(s)",
      res[0] is True and "0 active(s)" in line and not ko, f"{res} {line!r} ko={ko}")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

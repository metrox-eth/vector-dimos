"""Entry-point resolution bench - requires dimos (git main) installed.

The external-blueprints mechanism (entry points group "dimos.blueprints")
is newer than the current PyPI release: install dimos from git main.

Section B guards the forkserver bug described in vector_dimos/blueprints.py:
resolving a blueprint in THIS process is not enough, because dimOS builds the
adapter in a forkserver worker that starts from a clean interpreter.
"""
import pickle
import subprocess
import sys
import textwrap
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dimos.robot.get_all_blueprints import get_by_name

ok = True

print("A. entry points resolve")
for name in ("vector-dimos.base", "vector-dimos.gamepad", "vector-dimos.rplidar"):
    try:
        bp = get_by_name(name)
        n = len(getattr(bp, "blueprints", ()))
        print(f"  OK  {name} -> {type(bp).__name__} ({n} atom(s))")
    except Exception as e:
        print(f"  KO  {name}: {type(e).__name__}: {e}")
        ok = False


def check(label: str, cond: bool) -> None:
    global ok
    print(f"  {'OK ' if cond else 'KO '} {label}")
    if not cond:
        ok = False


print("\nB. the worker that builds the adapter registers 'vector'")
# dimOS deploys modules into forkserver workers: a fresh interpreter that
# imports dimos but not this package. It only imports what it needs to
# UNPICKLE the module class it was told to deploy. So the pickled coordinator
# has to drag vector_dimos.blueprints in with it - otherwise _setup_hardware
# dies with KeyError: "Unknown twist base adapter: vector".
from vector_dimos.blueprints import VectorControlCoordinator

payload = pickle.dumps(VectorControlCoordinator)
check("coordinator pickles by reference to this package",
      b"vector_dimos.blueprints" in payload)

# Replay what the worker does, in a clean interpreter that never imports
# vector_dimos itself: registry first (must NOT know 'vector'), then unpickle.
probe = textwrap.dedent("""
    import pickle, sys
    from dimos.hardware.drive_trains.registry import twist_base_adapter_registry as r
    before = "vector" in r.available()
    cls = pickle.loads(sys.stdin.buffer.read())
    after = "vector" in r.available()
    print(f"{before}|{after}|{cls.__name__}|{'vector_dimos' in sys.modules}")
""")
proc = subprocess.run([sys.executable, "-c", probe], input=payload,
                      capture_output=True, timeout=180)
if proc.returncode != 0:
    print(f"  KO  worker probe failed rc={proc.returncode}: "
          f"{proc.stderr.decode()[-400:]}")
    ok = False
else:
    before, after, cls_name, imported = proc.stdout.decode().strip().split("|")
    check("clean interpreter does not know 'vector' before unpickling",
          before == "False")
    check("unpickling the coordinator imports vector_dimos", imported == "True")
    check(f"'vector' registered after unpickling {cls_name}", after == "True")

# The shipped dimOS RPC clients address the coordinator by this exact name;
# ControlCoordinator.start() warns if a subclass drops it.
base = get_by_name("vector-dimos.base")
names = [atom.name for atom in base.blueprints]
check(f"coordinator still serves as 'ControlCoordinator' (got {names})",
      "ControlCoordinator" in names)

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

"""Entry-point resolution bench - requires dimos (git main) installed.

The external-blueprints mechanism (entry points group "dimos.blueprints")
is newer than the current PyPI release: install dimos from git main.
"""
from dimos.robot.get_all_blueprints import get_by_name

ok = True
for name in ("vector-dimos.base", "vector-dimos.gamepad", "vector-dimos.rplidar"):
    try:
        bp = get_by_name(name)
        n = len(getattr(bp, "blueprints", ()))
        print(f"  OK  {name} -> {type(bp).__name__} ({n} atom(s))")
    except Exception as e:
        print(f"  KO  {name}: {type(e).__name__}: {e}")
        ok = False

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

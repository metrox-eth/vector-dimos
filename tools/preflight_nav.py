"""VECTOR nav-stack flight check: the software chain probed before a run.

Companion to tools/preflight.py (hardware). Same contract: read-only, one
verdict per line, exit 1 if anything is KO. Answers "will the explore stack
come up sane?" without arming a single motor:

  imports        dimos + every vector_dimos module the explore blueprint pulls
  blueprints     base / nav / explore resolve; which explorer the flag selects
  persistent map the saved flat loads; the newest checkpoint loads (a power
                 cut used to leave a corrupt one - save() is atomic since 26/08)
  keep-out zones the file loads through the map loader itself - the shapes it
                 accepts (document or bare list) are the ones that fly
  runway         no dimos stack already running, motor + lidar ports free
                 (serial contention has killed whole sessions), disk and RAM

    .venv/bin/python tools/preflight_nav.py
"""
from __future__ import annotations

import glob
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

RESULTS: list[tuple[bool, str]] = []


def verdict(ok: bool, label: str, detail: str = "") -> None:
    RESULTS.append((ok, label))
    print(f"  {'OK ' if ok else 'KO '} {label}" + (f" - {detail}" if detail else ""))


def check_imports() -> None:
    print("imports (dimos + modules du blueprint explore)")
    try:
        import dimos
        verdict(True, "dimos", getattr(dimos, "__version__", "version inconnue"))
    except Exception as exc:  # noqa: BLE001
        verdict(False, "dimos", str(exc))
        return
    for name in ("adapter", "blueprints", "c1_serial", "costmap2d", "esp_sensors",
                 "explorer2", "lidar_odometry", "memory", "persistent_map",
                 "recovering_planner", "rplidar_c1", "respeaker"):
        try:
            __import__(f"vector_dimos.{name}")
            verdict(True, f"vector_dimos.{name}")
        except Exception as exc:  # noqa: BLE001
            verdict(False, f"vector_dimos.{name}", str(exc))


def check_blueprints() -> None:
    print("blueprints (les lourds: torch + open3d)")
    try:
        from vector_dimos import blueprints
        verdict(blueprints.base_blueprint is not None, "base resout")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "base", str(exc))
    try:
        from vector_dimos import nav_blueprints
        verdict(nav_blueprints.nav_blueprint is not None, "nav resout")
        verdict(nav_blueprints.explore_blueprint is not None, "explore resout")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "nav/explore", str(exc))
    try:
        from vector_dimos.explorer2 import explorer_v2_enabled
        verdict(True, "explorateur selectionne",
                "explorer2 (defaut)" if explorer_v2_enabled() else "fast_explorer (EXPLORER_V2=0)")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "selection explorateur", str(exc))


def check_maps() -> None:
    print("cartes (persistante + dernier checkpoint + zones)")
    import numpy as np

    from vector_dimos import persistent_map
    from vector_dimos.costmap2d import ScoredGrid

    if os.path.isfile(persistent_map.MAP_PATH):
        try:
            g = ScoredGrid.load(persistent_map.MAP_PATH)
            verdict(True, "carte persistante charge",
                    f"{persistent_map.MAP_PATH.rsplit('/', 1)[-1]}, {g.n}x{g.n} cellules")
        except Exception as exc:  # noqa: BLE001
            verdict(False, "carte persistante CORROMPUE", str(exc))
    else:
        verdict(True, "pas de carte persistante (run en repere frais)", persistent_map.MAP_PATH)

    cks = sorted(glob.glob(os.path.join(persistent_map.CHECKPOINT_DIR, "*", "*.npz")),
                 key=os.path.getmtime)
    if not cks:
        verdict(True, "aucun checkpoint (premier run)")
    else:
        skipped = 0
        for ck in reversed(cks):
            try:
                ScoredGrid.load(ck); np.load(ck)
                # a corrupt NEWEST is history (pre-atomic-save battery cuts),
                # not a blocker: what matters is that ONE loads
                verdict(True, "dernier checkpoint charge",
                        os.path.basename(ck) + (f" ({skipped} corrompu(s) saute(s) - renommer en .corrompu)" if skipped else ""))
                break
            except Exception:  # noqa: BLE001
                skipped += 1
        else:
            verdict(False, "TOUS les checkpoints corrompus", f"{len(cks)} fichiers")

    try:
        # ONE reader: the loader owns the shape contract (the file's
        # {"frame", "zones"} document AND a bare list of zones). A second
        # parse here called .get on a bare list, and a file the stack enforces
        # fine refused the flight as "illisible" (audit 2026-08-28). The
        # legacy no_slip count died with it - the loader drops those silently.
        zones = persistent_map.load_keepouts(persistent_map.KEEPOUT_PATH)
        verdict(True, "zones keep-out", f"{len(zones)} active(s)")
    except Exception as exc:  # noqa: BLE001
        verdict(False, "keepout.json illisible", str(exc))


def check_runway() -> None:
    print("piste (stack, ports, disque, RAM)")
    # match the dimos CLI itself, not the vector-dimos DIRECTORY in every
    # tool's path (pgrep -f matched zone_server.py through its cwd, 26/08)
    out = subprocess.run(["pgrep", "-af", "[b]in/dimos"], capture_output=True, text=True).stdout.strip()
    verdict(not out, "aucune stack dimos deja en route", out.splitlines()[0] if out else "")

    from vector_dimos.adapter import DEFAULT_PORT as MOTOR_PORT
    from vector_dimos.rplidar_c1 import DEFAULT_PORT as LIDAR_PORT
    for label, port in (("port moteurs libre", MOTOR_PORT), ("port lidar libre", LIDAR_PORT)):
        real = os.path.realpath(port)
        holder = subprocess.run(["sudo", "fuser", real], capture_output=True, text=True).stdout.strip()
        verdict(not holder, label, f"tenu par pid {holder}" if holder else "")

    st = os.statvfs(os.path.expanduser("~/.local/state"))
    free_gb = st.f_bavail * st.f_frsize / 1e9
    verdict(free_gb > 5.0, "disque pour logs+recordings", f"{free_gb:.1f} Go libres")

    avail_kb = 0
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable"):
            avail_kb = int(line.split()[1])
            break
    verdict(avail_kb > 2.5e6, "RAM disponible", f"{avail_kb / 1e6:.1f} Go")


def main() -> int:
    for step in (check_imports, check_blueprints, check_maps, check_runway):
        step()
    ko = [label for ok, label in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(ko)}/{len(RESULTS)} OK"
          + (f" - KO: {', '.join(ko)}" if ko else " - STACK PARE AU VOL"))
    return 1 if ko else 0


if __name__ == "__main__":
    raise SystemExit(main())

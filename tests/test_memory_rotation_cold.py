"""Cold bench for the recording rotation: a SIGKILLed run keeps its WAL.

A recording is a TRIO - explore.db, explore.db-wal, explore.db-shm - and a run
killed with SIGKILL (`dimos stop` escalates to it, see tools/fly.sh) leaves its
last commits in the -wal alone. dimOS's backup_file() renames the .db by itself,
so the backup the autopsy reads opens with an empty schema and the next run's
fresh db writes over the -wal that held everything: the crashed run, the one
worth keeping, is the one destroyed. Sections, all in physical units (metres of
recorded pose x, bytes on disk):

  A. what a kill leaves  - the writer dies by SIGKILL with 500 rows committed;
                           the .db alone reads as "no such table: obs"
  B. healthy rotation    - the trio is renamed together and the rotated
                           recording gives back 500 rows and x = 124.75 m
                           through the read-only URI tools/bench_run.py uses;
                           the next run's fresh db leaves it alone
  C. torn trio           - -wal/-shm with no db: dated quarantine, byte for
                           byte, never rm; the new db is fresh, the quarantined
                           -wal survives it, and re-paired with its own db it
                           still returns the 500 rows / 124.75 m
  D. no overwrite        - two rotations in the same second keep both trios
                           whole (500 rows and 300 rows, each with its own -wal)
                           and the prune retires whole trios, never a widowed
                           -wal that would shadow a later recording
  E. the wiring          - VectorMemory* really rotates on start(), and only
                           where dimOS would have called backup_file (BACKUP,
                           not replay, not APPEND)

Run:  PYTHONPATH=. .venv/bin/python tests/test_memory_rotation_cold.py
"""

import hashlib
import re
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# dimos.memory.module reaches dimos.models.embedding -> torch, absent from the
# rig venv. Only annotations need it here; nothing in this bench calls torch.
if "torch" not in sys.modules:
    class _Dtype:
        pass

    class _TorchStub(types.ModuleType):
        dtype = _Dtype
        Tensor = type("Tensor", (), {})
        device = str
        cuda = types.SimpleNamespace(is_available=lambda: False)
        backends = types.SimpleNamespace(mps=types.SimpleNamespace(is_available=lambda: False))

        def __getattr__(self, name):
            return _Dtype()

    sys.modules["torch"] = _TorchStub("torch")

from dimos.memory.module import Recorder  # noqa: E402

from vector_dimos import memory as M  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# A recorded pose every 0.25 m: row i is at x = i * 0.25 m, so 500 rows end at
# 124.75 m - a known input read back as a known output.
STEP_M = 0.25
WRITER = """
import os, signal, sqlite3, sys
db, rows = sys.argv[1], int(sys.argv[2])
c = sqlite3.connect(db)
c.execute("PRAGMA journal_mode=WAL")
c.execute("CREATE TABLE obs (id INTEGER PRIMARY KEY, x REAL)")
c.executemany("INSERT INTO obs VALUES (?, ?)", [(i, i * %r) for i in range(rows)])
c.commit()
os.kill(os.getpid(), signal.SIGKILL)   # what `dimos stop` escalates to
""" % STEP_M


def killed_recording(d: Path, rows: int = 500) -> Path:
    """A run that committed *rows* poses and was then SIGKILLed: db + -wal + -shm."""
    db = d / "explore.db"
    proc = subprocess.run([sys.executable, "-c", WRITER, str(db), str(rows)],
                          capture_output=True, timeout=120)
    assert proc.returncode == -9, f"writer should die by SIGKILL, rc={proc.returncode}"
    for suffix in ("", "-wal", "-shm"):
        assert db.with_name(db.name + suffix).exists(), f"missing explore.db{suffix}"
    return db


def read_rows(db: Path):
    """(row count, x of the last pose) read the way the autopsy tools read it:
    read-only URI, exactly tools/bench_run.py open_recording()."""
    conn = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    try:
        n = conn.execute("SELECT count(*) FROM obs").fetchone()[0]
        x = conn.execute("SELECT x FROM obs ORDER BY id DESC LIMIT 1").fetchone()[0]
        return n, x
    except sqlite3.OperationalError as e:
        # a recording whose -wal was left behind: empty schema, nothing to read
        print(f"      sqlite: {e}")
        return 0, float("nan")
    finally:
        conn.close()


def digest(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def names(d: Path):
    return sorted(p.name for p in d.iterdir())


ROTATED = re.compile(r"^explore\.\d{14}(-\d+)?\.db$")
root = Path(tempfile.mkdtemp(prefix="vector-rotation-"))

# --- A. what a SIGKILL leaves behind --------------------------------------
print("A. a killed run's rows live in the -wal, not in the .db")
a = root / "a"
a.mkdir()
db = killed_recording(a, rows=500)
check("the writer died by SIGKILL and left the trio",
      names(a) == ["explore.db", "explore.db-shm", "explore.db-wal"], str(names(a)))
check("the -wal is where the bytes are",
      db.with_name("explore.db-wal").stat().st_size > 20_000,
      f"{db.with_name('explore.db-wal').stat().st_size} B of -wal "
      f"vs {db.stat().st_size} B of .db")

alone = root / "a_db_alone"
alone.mkdir()
shutil.copy2(db, alone / "explore.db")   # the .db moved by itself, as backup_file does
n, _ = read_rows(alone / "explore.db")
check("the .db by itself has an empty schema (the pre-fix loss)", n == 0, f"{n} rows")

# --- B. the rotation moves the trio ---------------------------------------
print("B. rotation renames db + -wal + -shm together")
rotated = M.rotate_recording(db, keep_last=10)
check("rotate_recording returns the rotated db", rotated is not None and rotated.exists(),
      str(rotated))
check("named the way run_autopsy globs (explore.<stamp>.db)",
      bool(ROTATED.match(rotated.name)), rotated.name)
check("nothing of the previous run is left at the live path",
      not any(db.with_name(db.name + s).exists() for s in ("", "-wal", "-shm")),
      str(names(a)))
check("the whole trio moved",
      all(rotated.with_name(rotated.name + s).exists() for s in ("-wal", "-shm")),
      str(names(a)))
n, x = read_rows(rotated)
check("the rotated recording reads back its 500 poses", n == 500, f"{n} rows")
check("... and the last one is at 124.75 m", abs(x - 124.75) < 1e-9, f"x = {x} m")

fresh = sqlite3.connect(str(db))          # the next run opens its own explore.db
fresh.execute("PRAGMA journal_mode=WAL")
fresh.execute("CREATE TABLE obs (id INTEGER PRIMARY KEY, x REAL)")
fresh.execute("INSERT INTO obs VALUES (0, 0.5)")
fresh.commit()
fresh.close()
n, x = read_rows(rotated)
check("the new run does not touch it: still 500 poses at 124.75 m",
      n == 500 and abs(x - 124.75) < 1e-9, f"{n} rows, x = {x} m")

# --- C. the torn trio: -wal/-shm with no db --------------------------------
print("C. orphaned -wal/-shm -> dated quarantine, intact")
c = root / "c"
c.mkdir()
db = killed_recording(c, rows=500)
widowed = c / "explore.20260101000000.db"
db.rename(widowed)                        # the pre-fix rotation: the .db alone
before = {p.name: (p.stat().st_size, digest(p)) for p in
          (c / "explore.db-wal", c / "explore.db-shm")}
check("the state a pre-fix rotation leaves: sidecars with no db",
      names(c) == ["explore.20260101000000.db", "explore.db-shm", "explore.db-wal"],
      str(names(c)))

check("rotate_recording has no recording to rotate", M.rotate_recording(db) is None)
quarantine = c / "quarantine"
dated = sorted(quarantine.iterdir()) if quarantine.is_dir() else []
check("a dated quarantine directory was created",
      len(dated) == 1 and re.match(r"^\d{14}(-\d+)?$", dated[0].name),
      str([p.name for p in dated]))
if dated:
    after = {p.name: (p.stat().st_size, digest(p)) for p in dated[0].iterdir()}
    check("both sidecars are in it, byte for byte", after == before,
          f"{sorted(after)} vs {sorted(before)}")
check("and none is left beside the live db",
      not (c / "explore.db-wal").exists() and not (c / "explore.db-shm").exists(),
      str(names(c)))

fresh = sqlite3.connect(str(db))          # the next run, on a clean slate
fresh.execute("PRAGMA journal_mode=WAL")
fresh.execute("CREATE TABLE obs (id INTEGER PRIMARY KEY, x REAL)")
fresh.execute("INSERT INTO obs VALUES (0, 0.5)")
fresh.commit()
fresh.close()
n, x = read_rows(db)
check("the new recording is fresh (1 pose at 0.5 m)", n == 1 and abs(x - 0.5) < 1e-9,
      f"{n} rows, x = {x} m")
if dated:
    still = {p.name: (p.stat().st_size, digest(p)) for p in dated[0].iterdir()}
    check("the quarantined -wal survived the new run", still == before)
    # nothing was lost: paired back with its own db, the killed run reads out
    for p in dated[0].iterdir():
        shutil.copy2(p, c / p.name.replace("explore.db", widowed.name))
    n, x = read_rows(widowed)
    check("re-paired with its db it still holds 500 poses, last at 124.75 m",
          n == 500 and abs(x - 124.75) < 1e-9, f"{n} rows, x = {x} m")

# --- D. no overwrite, and the prune retires whole trios ---------------------
print("D. two rotations in the same second, then the prune")
d = root / "d"
d.mkdir()
db = d / "explore.db"
killed_recording(d, rows=500)
first = M.rotate_recording(db, keep_last=10)
killed_recording(d, rows=300)
second = M.rotate_recording(db, keep_last=10)
check("the second rotation took a free name", first != second, f"{first.name} / {second.name}")
n1, x1 = read_rows(first)
n2, x2 = read_rows(second)
check("the first recording still holds its own 500 poses (124.75 m)",
      n1 == 500 and abs(x1 - 124.75) < 1e-9, f"{n1} rows, x = {x1} m")
check("the second holds its own 300 poses (74.75 m)",
      n2 == 300 and abs(x2 - 74.75) < 1e-9, f"{n2} rows, x = {x2} m")

killed_recording(d, rows=200)
third = M.rotate_recording(db, keep_last=2)
kept = [p.name for p in d.iterdir() if ROTATED.match(p.name)]
check("keep_last=2 retires the oldest recording", sorted(kept) == sorted(
    [second.name, third.name]), str(sorted(kept)))
check("... the whole trio of it, no widowed -wal left to shadow a later run",
      all((p.parent / p.name[:-len("-wal")]).exists()
          for p in d.iterdir() if p.name.endswith("-wal")),
      str(names(d)))
n3, x3 = read_rows(third)
check("the newest recording is the one that reads back (200 poses, 49.75 m)",
      n3 == 200 and abs(x3 - 49.75) < 1e-9, f"{n3} rows, x = {x3} m")

# --- E. the module rotates on start ----------------------------------------
print("E. VectorMemory's start() is what rotates")
check("the recorder still asks dimOS for BACKUP (the rotation we replace)",
      M.VectorMemoryConfig().on_existing is M.OnExisting.BACKUP)
check("start() is ours, on both recorders, and still an rpc",
      M.VectorMemoryLight.start is not Recorder.start
      and M.VectorMemory.start is M.VectorMemoryLight.start
      and getattr(M.VectorMemoryLight.start, "__rpc__", False) is True)

saved_start, Recorder.start = Recorder.start, lambda self: None   # no store, no bus
try:
    e = root / "e"
    e.mkdir()
    db = killed_recording(e, rows=500)
    M.VectorMemoryLight(db_path=str(db)).start()
    live = [p.name for p in e.iterdir() if ROTATED.match(p.name)]
    check("start() rotated the trio", len(live) == 1 and not db.exists(), str(names(e)))
    if live:
        n, x = read_rows(e / live[0])
        check("... and the killed run reads back: 500 poses, last at 124.75 m",
              n == 500 and abs(x - 124.75) < 1e-9, f"{n} rows, x = {x} m")

    e2 = root / "e_append"
    e2.mkdir()
    db2 = killed_recording(e2, rows=500)
    M.VectorMemoryLight(db_path=str(db2), on_existing="append").start()
    check("on_existing=append rotates nothing (upstream's business)",
          names(e2) == ["explore.db", "explore.db-shm", "explore.db-wal"], str(names(e2)))

    e3 = root / "e_replay"
    e3.mkdir()
    db3 = killed_recording(e3, rows=500)
    m3 = M.VectorMemoryLight(db_path=str(db3))
    m3.config.g.replay = True
    try:
        m3.start()
    finally:
        m3.config.g.replay = False
    check("a replay leaves the recording untouched",
          names(e3) == ["explore.db", "explore.db-shm", "explore.db-wal"], str(names(e3)))
finally:
    Recorder.start = saved_start
    shutil.rmtree(root, ignore_errors=True)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

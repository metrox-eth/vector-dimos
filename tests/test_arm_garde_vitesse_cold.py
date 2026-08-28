"""Cold bench for tools/arm_garde_vitesse.sh: gate 7 of fly.sh must REPLACE the
previous flight's speed watchdog, not be satisfied by its corpse.

Rule #2: known input -> known output, here in pids and in seconds since boot.
Every check runs the REAL script (no copy of its logic) against fake watchdogs
and, in group F, against the REAL tools/garde_vitesse.py on a fake HOME. The
rover is not needed. Groups:

  A. the corpse   - a watchdog of the previous flight is alive and named by the
                    pid file: it is KILLED, the pid file names a different live
                    pid, and that pid was born AFTER the launch mark (the old one
                    was born before it) - the 2026-08-28 audit's failure
  B. first flight - no pid file at all: one watchdog is armed, born after
  C. stale file   - the pid file names a pid that is gone: armed anyway
  D. pid reuse    - the pid file names a LIVE process that is not a garde_vitesse:
                    it is left alone, a watchdog is armed beside it
  E. no proof     - the watchdog that never comes up, and the pid file that names
                    a live process born BEFORE the launch: both refuse (rc 1), so
                    fly.sh never prints "armed" over an unguarded flight
  F. the real one - tools/garde_vitesse.py armed twice on a fake HOME: the second
                    arming kills the first and the pid CHANGES (the spec's check)
  G. flight path  - fly.sh parses, calls the script at gate 7, keeps no pgrep
                    guard, and stops the flight when arming fails

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_arm_garde_vitesse_cold.py
"""

import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
ARM = ROOT / "tools" / "arm_garde_vitesse.sh"
FLY = ROOT / "tools" / "fly.sh"
CLK_TCK = os.sysconf("SC_CLK_TCK")
OK = 0
KO = 0
SPAWNED = []


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def uptime_s() -> int:
    """Seconds since boot, the clock /proc/<pid>/stat dates births on."""
    return int(float(Path("/proc/uptime").read_text().split()[0]))


def born_s(pid) -> int:
    """Second since boot at which pid was born (field 22 of /proc/<pid>/stat)."""
    raw = Path(f"/proc/{pid}/stat").read_text()
    return int(raw.split(") ", 1)[1].split()[19]) // CLK_TCK


def alive(pid) -> bool:
    """Same liveness the script uses: a zombie answers kill -0 but tails nothing."""
    try:
        raw = Path(f"/proc/{int(pid)}/stat").read_text()
    except (OSError, ValueError, TypeError):
        return False
    return raw.split(") ", 1)[1].split()[0] != "Z"


def wait_dead(pid, timeout_s=5.0) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end and alive(pid):
        time.sleep(0.05)
    return not alive(pid)


TMP = tempfile.TemporaryDirectory()
FAKE_DIR = Path(TMP.name) / "fake"
FAKE_DIR.mkdir()

# a stand-in watchdog: same file name (so /proc/<pid>/cmdline says garde_vitesse),
# same contract - stamp the pid file at startup, then stay up
FAKE = FAKE_DIR / "garde_vitesse.py"
FAKE.write_text(
    "import os, time\n"
    "open(os.environ['GARDE_PID_FILE'], 'w').write(f'{os.getpid()}\\n')\n"
    "time.sleep(600)\n"
)
# a watchdog that dies at once without ever stamping a pid
DEAD = FAKE_DIR / "dies_at_once.py"
DEAD.write_text("import sys\nsys.exit('no dimos run logs found')\n")
# a watchdog that stamps SOMEONE ELSE's pid: alive, but born before the launch
LIAR = FAKE_DIR / "stamps_an_old_pid.py"
LIAR.write_text(
    "import os\n"
    "open(os.environ['GARDE_PID_FILE'], 'w').write(os.environ['OLD_PID'] + '\\n')\n"
)
# a live process that is NOT a watchdog (the pid the pid file reuses after a reboot)
INNOCENT = FAKE_DIR / "innocent_worker.py"
INNOCENT.write_text("import time\ntime.sleep(600)\n")


def spawn(script: Path, env_extra=None):
    env = {**os.environ, **(env_extra or {})}
    p = subprocess.Popen([sys.executable, str(script)], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    SPAWNED.append(p)
    return p


def run_arm(pid_file, garde_py=None, extra=None, timeout_s=60):
    """Run the REAL arm script; returns (rc, stdout, seconds, mark) where mark is
    the second since boot taken just before the launch."""
    env = {**os.environ,
           "GARDE_PID_FILE": str(pid_file),
           "GARDE_PYTHON": sys.executable,
           "GARDE_LOG": str(Path(pid_file).with_suffix(".log")),
           "GARDE_ARM_DEADLINE_S": "20"}
    if garde_py:
        env["GARDE_PY"] = str(garde_py)
    env.update(extra or {})
    mark = uptime_s()
    t0 = time.monotonic()
    r = subprocess.run(["bash", str(ARM)], env=env, cwd=str(ROOT), text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout_s)
    return r.returncode, r.stdout, time.monotonic() - t0, mark


def wait_pid_file(path, timeout_s=20.0):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            return int(Path(path).read_text().strip())
        except (OSError, ValueError):
            time.sleep(0.05)
    return None


print("A. the corpse of the previous flight satisfied the old pgrep guard")
PID_A = Path(TMP.name) / "a.pid"
spawn(FAKE, {"GARDE_PID_FILE": str(PID_A)})
OLD_A = wait_pid_file(PID_A)
OLD_BORN = born_s(OLD_A)
time.sleep(1.2)                      # so the two births land in different seconds
rc, out, took, mark = run_arm(PID_A, garde_py=FAKE)
NEW_A = int(PID_A.read_text().strip())
check("the arm gate exits 0", rc == 0, f"rc {rc}")
check("the previous watchdog is DEAD", wait_dead(OLD_A), f"pid {OLD_A}")
check("and the script says so", "retired" in out, out.strip().splitlines()[0] if out.strip() else "(silence)")
check("the pid file names a DIFFERENT pid", NEW_A != OLD_A, f"{OLD_A} -> {NEW_A}")
check("which is alive", alive(NEW_A))
check("born AFTER the launch mark", born_s(NEW_A) >= mark, f"born {born_s(NEW_A)} s, mark {mark} s (since boot)")
check("while the retired one was born BEFORE it", OLD_BORN < mark, f"born {OLD_BORN} s, mark {mark} s")
check("it announces the armed pid", f"pid {NEW_A}" in out and "armed" in out,
      out.strip().splitlines()[-1] if out.strip() else "(silence)")
check("arming a flight takes seconds, not a flight", took < 10.0, f"{took:.1f} s")

print("B. first flight after a reboot: no pid file at all")
PID_B = Path(TMP.name) / "b.pid"
rc, out, took, mark = run_arm(PID_B, garde_py=FAKE)
NEW_B = int(PID_B.read_text().strip())
check("armed, rc 0", rc == 0, f"rc {rc}")
check("the pid file names a live process", alive(NEW_B), f"pid {NEW_B}")
check("born after the launch mark", born_s(NEW_B) >= mark, f"born {born_s(NEW_B)} s, mark {mark} s")
check("nothing is claimed about a previous watchdog", "retired" not in out)

print("C. stale pid file: the pid it names is gone (a SIGKILLed watchdog)")
DEAD_PID = subprocess.Popen([sys.executable, "-c", "pass"])   # reaped: its pid is gone
DEAD_PID.wait()
PID_C = Path(TMP.name) / "c.pid"
PID_C.write_text(f"{DEAD_PID.pid}\n")
rc, out, took, mark = run_arm(PID_C, garde_py=FAKE)
NEW_C = int(PID_C.read_text().strip())
check("armed anyway, rc 0", rc == 0, f"rc {rc}")
check("the stale pid file is named as stale", "stale pid file" in out,
      out.strip().splitlines()[0] if out.strip() else "(silence)")
check("and a live watchdog replaces the dead number", alive(NEW_C) and NEW_C != DEAD_PID.pid,
      f"{DEAD_PID.pid} -> {NEW_C}")

print("D. pid reuse: the pid file names a live process that is not a watchdog")
PID_D = Path(TMP.name) / "d.pid"
victim = spawn(INNOCENT)
PID_D.write_text(f"{victim.pid}\n")
rc, out, took, mark = run_arm(PID_D, garde_py=FAKE)
NEW_D = int(PID_D.read_text().strip())
check("armed, rc 0", rc == 0, f"rc {rc}")
check("the innocent process is NOT killed", alive(victim.pid), f"pid {victim.pid} still alive")
check("and the script says why", "not a garde_vitesse" in out,
      out.strip().splitlines()[0] if out.strip() else "(silence)")
check("a watchdog is armed beside it", alive(NEW_D) and NEW_D != victim.pid, f"pid {NEW_D}")
check("born after the launch mark", born_s(NEW_D) >= mark, f"born {born_s(NEW_D)} s, mark {mark} s")

print("E. no proof of a fresh watchdog: the gate refuses (fly.sh then stops the flight)")
PID_E = Path(TMP.name) / "e.pid"
rc, out, took, mark = run_arm(PID_E, garde_py=DEAD, extra={"GARDE_ARM_DEADLINE_S": "2"})
check("a watchdog that never comes up -> rc 1", rc == 1, f"rc {rc}")
check("and the reason is on screen", "DID NOT COME UP" in out,
      out.strip().splitlines()[-2] if len(out.strip().splitlines()) > 1 else out.strip())
check("nothing claims 'armed'", "armed" not in out)
check("it refuses within its deadline, not a flight", took < 8.0, f"{took:.1f} s")
PID_E2 = Path(TMP.name) / "e2.pid"
elder = spawn(INNOCENT)
time.sleep(1.2)
rc, out, took, mark = run_arm(PID_E2, garde_py=LIAR,
                              extra={"GARDE_ARM_DEADLINE_S": "2", "OLD_PID": str(elder.pid)})
check("a pid file naming a LIVE process born BEFORE the launch -> rc 1", rc == 1, f"rc {rc}")
check("(liveness alone never proves an arming)", born_s(elder.pid) < mark and alive(elder.pid),
      f"that pid was born {mark - born_s(elder.pid)} s before the mark")

print("F. the REAL tools/garde_vitesse.py, armed twice (the spec's check)")
HOME_F = Path(TMP.name) / "rover_home"
RUN_F = HOME_F / ".local" / "state" / "dimos" / "logs" / "20260828_220003-vector-dimos-explore"
RUN_F.mkdir(parents=True)
LOG_F = RUN_F / "main.jsonl"
LOG_F.write_text('{"timestamp": "2026-08-28T22:00:03.000000Z", "message": "lidar odom #1: x=+0.000 y=+0.000"}\n')
PID_F = Path(TMP.name) / "f.pid"
env_f = {"HOME": str(HOME_F), "GARDE_IDLE_S": "1e6", "GARDE_POLL_S": "0.5"}
rc, out, took, mark = run_arm(PID_F, extra=env_f)
FIRST = int(PID_F.read_text().strip()) if PID_F.exists() else None
check("the real watchdog arms, rc 0", rc == 0, f"rc {rc}")
check("the pid file names it, alive", FIRST is not None and alive(FIRST), f"pid {FIRST}")
check("born after the launch mark", FIRST is not None and born_s(FIRST) >= mark,
      f"born {born_s(FIRST) if FIRST else '?'} s, mark {mark} s")
garde_log = Path(str(PID_F.with_suffix(".log")))
for _ in range(40):
    if "watching" in garde_log.read_text():
        break
    time.sleep(0.1)
check("and it tails THIS run's log", str(LOG_F) in garde_log.read_text(),
      garde_log.read_text().strip().splitlines()[0] if garde_log.read_text().strip() else "(silence)")
time.sleep(1.2)
rc, out, took, mark = run_arm(PID_F, extra=env_f)
SECOND = int(PID_F.read_text().strip()) if PID_F.exists() else None
check("the next flight arms again, rc 0", rc == 0, f"rc {rc}")
check("the PID CHANGED - the gate replaced it", SECOND is not None and SECOND != FIRST,
      f"{FIRST} -> {SECOND}")
check("the first real watchdog is dead", wait_dead(FIRST), f"pid {FIRST}")
check("the second was born after the second launch mark", SECOND is not None and born_s(SECOND) >= mark,
      f"born {born_s(SECOND) if SECOND else '?'} s, mark {mark} s")
if SECOND:
    os.kill(SECOND, 15)
    wait_dead(SECOND)

print("G. the flight path: gate 7 of fly.sh")
fly = FLY.read_text()
check("fly.sh parses", subprocess.run(["bash", "-n", str(FLY)]).returncode == 0)
check("the arm script parses", subprocess.run(["bash", "-n", str(ARM)]).returncode == 0)
check("no pgrep guard is left for the watchdog", "pgrep -f \"[g]arde_vitesse\"" not in fly)
gate7 = fly.split("== 7/8 exploration ==")[1].split("== 8/8")[0] if "== 7/8 exploration ==" in fly else ""
check("gate 7 arms through the script", "bash tools/arm_garde_vitesse.sh" in gate7)
check("a failed arming stops the flight", "exit 1" in gate7 and "NOT ARMED" in gate7)
check("no unconditional 'speed watchdog armed' echo remains in fly.sh",
      "speed watchdog armed" not in fly)

for p in SPAWNED:
    p.kill()
for p in SPAWNED:
    p.wait()
for leftover in (PID_A, PID_B, PID_C, PID_D):
    try:
        os.kill(int(leftover.read_text().strip()), 9)
    except (OSError, ValueError):
        pass

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

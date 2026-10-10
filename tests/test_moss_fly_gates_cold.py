#!/usr/bin/env python3
"""Cold bench for tools/moss-fly.sh: each subcommand x the gate the operator gets.

The REAL script runs here, gate logic untouched: FLY_TEST=1 makes it source a
stub file (written below) where dimos, the venv python, pgrep, pkill, nohup,
curl, kill, hostname and sleep become recorders. /dev/serial/by-id and the
dimOS log root are temp dirs; the fake `dimos run` writes the run's main.jsonl
with whatever proof lines the case asks for. No Jetson, no stack, no port.

  A. check        - runs ONLY the flight check (full: lidar spin included),
                    saved where the cockpit reads it; its exit code is the result
  B. stack up     - a registered run is refused before any preflight or launch,
                    nothing killed; a zombie (no registered run) is swept, and
                    one that survives kill -9 refuses; one still building
                    (unregistered but younger than the proof window) is refused
  B2. launch lock   - a second check/map/explore is refused while one runs,
                    stop never is; the lock never reaches the daemon or cockpit
  C. QT Py absent - refused before the preflight; MOCK=1 flies without it;
                    the lidar is required either way
  D. map          - preflight -> dimos run vector-dimos.moss-nav (env: profile
                    moss, PERSISTENT_MAP=0, MOSS_PORT = the QT Py by-id even if
                    the shell exported the lidar) -> proof -> cockpit; refusals
                    on a missing proof line, a stale run dir, a Traceback, a
                    dead cockpit (the stack is stopped each time)
  E. explore      - without EXPLORE=1 exploration is never published
  F. EXPLORE=1    - start published only after the stack AND cockpit proofs;
                    no explorer line in 15 s -> explore stop, then dimos stop;
                    EXPLORE=1 with map is a contradiction, refused untouched
  G. stop         - explore_ctl stop BEFORE dimos stop; log/cockpit/usage
  H. bash -n on the script and the stubs
  I. real pgrep     - the cockpit pattern matches the interpreter, not a viewer
  J. the unit       - KillMode=process, proven on a transient user unit

Run:  PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. python3 tests/test_moss_fly_gates_cold.py
"""

import fcntl
import os
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / "tools" / "moss-fly.sh"
OK = 0
KO = 0

STUBS = r"""# Stand-ins sourced by tools/moss-fly.sh under FLY_TEST=1 (tests/test_moss_fly_gates_cold.py).
: "${FLY_TEST_LOG:?}"
: "${FLY_TEST_DIR:?}"
: > "$FLY_TEST_LOG"
BYID_DIR="$FLY_TEST_DIR/by-id"
LOG_ROOT="$FLY_TEST_DIR/logs"
LAUNCH_LOG="$FLY_TEST_DIR/launch.log"
COCKPIT_LOG="$FLY_TEST_DIR/cockpit.log"
FLY_LOCK="$FLY_TEST_DIR/fly.lock"
DIMOS=fake_dimos
PY=fake_py
FLY_TEST_STACK_UP="${FLY_TEST_STACK_UP:-0}"        # 1 = a dimos run process exists
FLY_TEST_REGISTERED="${FLY_TEST_REGISTERED:-1}"    # 1 = it has a registered run (live flight)
FLY_TEST_SURVIVES="${FLY_TEST_SURVIVES:-0}"        # 1 = the zombie survives kill -9
FLY_TEST_STACK_AGE="${FLY_TEST_STACK_AGE:-600}"    # seconds since that process started (ps etimes)
FLY_TEST_LAUNCH="${FLY_TEST_LAUNCH:-ok}"           # the daemon launcher: ok | error (a module fails after the proofs) | silent
FLY_TEST_PREFLIGHT_RC="${FLY_TEST_PREFLIGHT_RC:-0}"
FLY_TEST_PROOF="${FLY_TEST_PROOF-MOSS base: connected on X|RPLIDAR C1 up on X|lidar odometry up (kiss-icp)}"
FLY_TEST_RUNDIR="${FLY_TEST_RUNDIR:-1}"            # 0 = the launch never creates a run dir
FLY_TEST_COCKPIT_UP="${FLY_TEST_COCKPIT_UP:-0}"    # 1 = a cockpit already answers
FLY_TEST_COCKPIT_DEAD="${FLY_TEST_COCKPIT_DEAD:-0}" # 1 = a started cockpit never answers
FLY_TEST_EXPLORE_PROOF="${FLY_TEST_EXPLORE_PROOF:-1}"

_rec() { printf '%s' "$*" | tr '\n' ' ' >> "$FLY_TEST_LOG"; printf '\n' >> "$FLY_TEST_LOG"; }
_fd9() { if [ -e /proc/self/fd/9 ]; then echo open; else echo closed; fi; }

fake_dimos() {
  case " $* " in
    *" run "*)
      _rec "dimos $* | ENV $(env | grep -E '^(ROBOT_PROFILE|PERSISTENT_MAP|MOSS_[A-Z_]+|TRANSPORT|GLIBC_TUNABLES)=' | sort | tr '\n' ' ')"
      local bp=""
      for a in "$@"; do case "$a" in vector-dimos.*) bp="${a#vector-dimos.}" ;; esac; done
      if [ "$FLY_TEST_RUNDIR" = "1" ]; then
        local d="$LOG_ROOT/$(date +%Y%m%d-%H%M%S)-vector-dimos-$bp"
        mkdir -p "$d"
        printf '%s\n' "$FLY_TEST_PROOF" | tr '|' '\n' | while IFS= read -r ev; do
          [ -n "$ev" ] && printf '{"event": "%s", "level": "info"}\n' "$ev" >> "$d/main.jsonl"
        done
        touch "$d/main.jsonl"
      fi
      _rec "dimos-run fd9=$(_fd9)"
      # as dimOS: the launcher reports the build's outcome after the modules logged
      case "$FLY_TEST_LAUNCH" in
        ok)    command sleep 0.1; echo "✓ DimOS running in background" ;;
        error) command sleep 0.3; echo "Error: RuntimeError: Failed to start camera pipeline" ;;
      esac ;;
    *" status "*)
      _rec "dimos status"
      if [ "$FLY_TEST_REGISTERED" = "1" ]; then echo "Run ID: 20261011-000000-vector-dimos-moss-nav"; else echo "No running DimOS instance"; fi ;;
    *" stop "*) _rec "dimos stop"; echo "stopped" ;;
    *" log "*)
      _rec "dimos $*"
      # a 300-line run log: the two startup lines first, then telemetry chatter;
      # like dimOS, `log` prints its last 50 lines unless -n says otherwise
      local n=50 prev=""
      for a in "$@"; do [ "$prev" = "-n" ] && n="$a"; prev="$a"; done
      { echo "MOSS base: connected on X"; echo "RPLIDAR C1 up on X"
        for i in $(seq 1 298); do echo "MOSS telemetry $i"; done; } | command tail -n "$n" ;;
    *) _rec "dimos $*" ;;
  esac
  return 0
}

fake_py() {
  _rec "py $*"
  case "$1" in
    tools/moss_preflight.py) _rec "preflight fd9=$(_fd9)"; echo "N/M OK (stub)"; return "$FLY_TEST_PREFLIGHT_RC" ;;
    tools/explore_ctl.py)
      echo "published /${2}_cmd on lcm"
      if [ "$2" = "start" ] && [ "$FLY_TEST_EXPLORE_PROOF" = "1" ]; then
        local d; d=$(ls -td "$LOG_ROOT"/*/ 2>/dev/null | head -1)
        [ -n "$d" ] && echo '{"event": "costmap: explore_cmd received - map writing OPEN for the mission"}' >> "$d/main.jsonl"
      fi ;;
    tools/moss_cockpit.py)
      _rec "cockpit-env EXPLORE=${EXPLORE-unset} MOCK=${MOCK-unset} MOSS_MOCK_BUS=${MOSS_MOCK_BUS-unset} fd9=$(_fd9)"
      : > "$FLY_TEST_DIR/cockpit_started" ;;
  esac
  return 0
}

pgrep() {
  _rec "pgrep $*"
  case "$*" in
    *"[b]in/dimos"*)
      [ "$FLY_TEST_STACK_UP" = "1" ] || return 1
      [ -e "$FLY_TEST_DIR/killed" ] && [ "$FLY_TEST_SURVIVES" != "1" ] && return 1
      echo 4242; return 0 ;;
  esac
  return 1
}
pkill()    { _rec "pkill $*"; : > "$FLY_TEST_DIR/killed"; return 0; }
kill()     { _rec "kill $*"; return 0; }
ps()       { _rec "ps $*"; echo "  $FLY_TEST_STACK_AGE"; }
nohup()    { _rec "nohup $*"; "$@"; }
curl()     {
  _rec "curl $*"
  [ "$FLY_TEST_COCKPIT_DEAD" = "1" ] && return 7
  [ "$FLY_TEST_COCKPIT_UP" = "1" ] || [ -e "$FLY_TEST_DIR/cockpit_started" ]
}
hostname() { echo "rover-test-address"; }
sleep()    { command sleep 0.02; }
"""

TMP = tempfile.TemporaryDirectory()
TMPD = Path(TMP.name)
STUB_FILE = TMPD / "moss_fly_stubs.sh"
STUB_FILE.write_text(STUBS)
QTPY_NAME = "usb-Adafruit_QT_Py_ESP32-S3_No_PSRAM_00:00:00:00:00:00-if00"
C1_NAME = "usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_0000-if00-port0"


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def fly(args=(), ports=("qtpy", "c1"), stale_run=None, locked=False, **env):
    """Run the real script cold. Returns (rc, what the operator saw, recorded commands, the case dir)."""
    case = TMPD / f"case_{len(list(TMPD.glob('case_*')))}"
    (case / "by-id").mkdir(parents=True)
    (case / "logs").mkdir()
    for p in ports:
        (case / "by-id" / (QTPY_NAME if p == "qtpy" else C1_NAME)).write_text("")
    if stale_run:
        d = case / "logs" / "20200101-000000-vector-dimos-moss-nav"
        d.mkdir()
        (d / "main.jsonl").write_text("".join(f'{{"event": "{e}"}}\n' for e in stale_run))
        os.utime(d, (1_577_836_800, 1_577_836_800))
    log = case / "rec.log"
    e = {k: v for k, v in os.environ.items()
         if not k.startswith(("MOSS_", "FLY_TEST")) and k not in ("EXPLORE", "MOCK", "ROBOT_PROFILE", "PERSISTENT_MAP", "TRANSPORT")}
    e.update(FLY_TEST="1", FLY_TEST_STUBS=str(STUB_FILE), FLY_TEST_LOG=str(log), FLY_TEST_DIR=str(case),
             MOSS_STATE_DIR=str(case / "state"))
    e.update({k: str(v) for k, v in env.items()})
    with open(case / "fly.lock", "a") as lk:
        if locked:   # another moss-fly.sh holds the launch lock
            fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
        p = subprocess.run(["bash", str(SCRIPT), *args], env=e, capture_output=True, text=True, timeout=120)
    return p.returncode, p.stdout + p.stderr, (log.read_text() if log.exists() else ""), case


def ran(log, needle):
    return [ln for ln in log.splitlines() if needle in ln]


def first(log, needle):
    for i, ln in enumerate(log.splitlines()):
        if needle in ln:
            return i
    return -1


GOOD_PROOF = "MOSS base: connected on X|RPLIDAR C1 up on X|lidar odometry up (kiss-icp)"

print("A. check = the flight check only")
rc, out, log, case = fly(["check"])
pre = ran(log, "py tools/moss_preflight.py")
check("exit 0 when the check is green", rc == 0, f"rc {rc}")
check("the preflight ran once, FULL (no --quick, no --no-spin)",
      len(pre) == 1 and "--quick" not in pre[0] and "--no-spin" not in pre[0], pre[0] if pre else "none")
check("saved where the cockpit reads it (<state>/preflight.json)",
      bool(pre) and f"--save {case / 'state' / 'preflight.json'}" in pre[0])
check("nothing launched: no dimos run, no nohup, no curl",
      not ran(log, "dimos --rerun-open") and not ran(log, "nohup") and not ran(log, "curl"))
rc, out, log, case = fly(["check"], FLY_TEST_PREFLIGHT_RC=1)
check("a KO check -> exit 1", rc == 1, f"rc {rc}")

print("B. a stack already up")
rc, out, log, case = fly(["map"], FLY_TEST_STACK_UP=1)
check("refused (rc 1) with the fix", rc == 1 and "A DIMOS STACK IS ALREADY RUNNING - tools/moss-fly.sh stop first" in out, f"rc {rc}")
check("no preflight, no launch, no cockpit",
      not ran(log, "moss_preflight") and not ran(log, "--daemon") and not ran(log, "curl"))
check("the live flight is not killed", not ran(log, "pkill") and not ran(log, "kill "))
check("asked dimos status before refusing", bool(ran(log, "dimos status")))
check("gate 0 matches `dimos <options> run`, not `bin/dimos run` only",
      any("[b]in/dimos( [^ ]+)* run( |$)" in ln for ln in ran(log, "pgrep")), ran(log, "pgrep")[0] if ran(log, "pgrep") else "")
rc, out, log, case = fly(["map"], FLY_TEST_STACK_UP=1, FLY_TEST_REGISTERED=0)
check("zombie (no registered run): swept with kill -9, then the flight goes on",
      rc == 0 and "zombie" in out and any("-9" in ln for ln in ran(log, "pkill")) and bool(ran(log, "--daemon")), f"rc {rc}")
check("... the sweep comes before the preflight", first(log, "pkill") < first(log, "moss_preflight"))
check("... the zombie's age was read first (ps etimes)", 0 <= first(log, "ps -o etimes=") < first(log, "pkill"))
rc, out, log, case = fly(["map"], FLY_TEST_STACK_UP=1, FLY_TEST_REGISTERED=0, FLY_TEST_STACK_AGE=12)
check("a stack still building (unregistered, 12 s old) -> refused as running, nothing killed",
      rc == 1 and "A DIMOS STACK IS STILL STARTING (pid 4242, 12 s old" in out and not ran(log, "pkill")
      and not ran(log, "moss_preflight") and not ran(log, "--daemon"), f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_STACK_UP=1, FLY_TEST_REGISTERED=0, FLY_TEST_STACK_AGE=90)
check("... 90 s old (past the 60 s proof + 30 s) with no registered run -> a zombie, swept",
      rc == 0 and any("-9" in ln for ln in ran(log, "pkill")), f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_STACK_UP=1, FLY_TEST_REGISTERED=0, FLY_TEST_SURVIVES=1)
check("a zombie that survives kill -9 -> refused, no launch",
      rc == 1 and "THE ZOMBIE SURVIVED" in out and not ran(log, "--daemon"), f"rc {rc}")

print("B2. one launcher at a time (the launch lock)")
rc, out, log, case = fly(["map"], locked=True, FLY_TEST_STACK_UP=1, FLY_TEST_REGISTERED=0, FLY_TEST_STACK_AGE=600)
check("lock held by another launcher -> map refused with the fix, nothing ran (no sweep)",
      rc == 1 and "ANOTHER moss-fly.sh IS RUNNING" in out and log.strip() == "", f"rc {rc} {log[:60]}")
rc, out, log, case = fly(["explore"], locked=True)
check("... explore refused the same way", rc == 1 and "ANOTHER moss-fly.sh IS RUNNING" in out and log.strip() == "")
rc, out, log, case = fly(["check"], locked=True)
check("... check refused too (it opens the board)", rc == 1 and "ANOTHER moss-fly.sh IS RUNNING" in out
      and not ran(log, "moss_preflight"))
rc, out, log, case = fly(["stop"], locked=True)
check("... stop is never locked out", rc == 0 and bool(ran(log, "dimos stop")))
rc, out, log, case = fly(["map"])
check("the lock is held while the gates run (the preflight sees fd 9)", ran(log, "preflight fd9=") == ["preflight fd9=open"],
      str(ran(log, "preflight fd9=")))
check("... and NOT passed on to the dimOS daemon or the cockpit (they outlive the launcher)",
      ran(log, "dimos-run fd9=") == ["dimos-run fd9=closed"] and ran(log, "fd9=closed") == ["dimos-run fd9=closed",
      "cockpit-env EXPLORE=unset MOCK=unset MOSS_MOCK_BUS=unset fd9=closed"], str(ran(log, "fd9=")))
with open(case / "fly.lock", "a") as lk:
    try:
        fcntl.flock(lk, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check("... released when the launcher exits", True)
    except OSError:
        check("... released when the launcher exits", False)

print("C. QT Py missing")
rc, out, log, case = fly(["map"], ports=("c1",))
check("refused (rc 1) with the fix", rc == 1 and "NO QT PY on USB" in out, f"rc {rc}")
check("before the preflight and the launch", not ran(log, "moss_preflight") and not ran(log, "--daemon"))
rc, out, log, case = fly(["map"], ports=("c1",), MOCK=1)
run_l = ran(log, "| ENV")
check("MOCK=1 flies without the board, MOSS_MOCK_BUS=1 in the stack's env",
      rc == 0 and len(run_l) == 1 and "MOSS_MOCK_BUS=1" in run_l[0], f"rc {rc}")
check("MOCK=1 with no board: MOSS_PORT not exported at all", bool(run_l) and "MOSS_PORT=" not in run_l[0])
rc, out, log, case = fly(["map"], ports=("qtpy",))
check("lidar missing -> refused, no launch", rc == 1 and "NO LIDAR on USB" in out and not ran(log, "--daemon"))
rc, out, log, case = fly(["map"], FLY_TEST_PREFLIGHT_RC=1)
check("preflight KO -> refused, no launch", rc == 1 and "FLIGHT CHECK KO" in out and not ran(log, "--daemon"))

print("D. map: the happy path and its refusals")
rc, out, log, case = fly(["map"], MOSS_PORT=str(TMPD / "by-id-lidar-alias"), PERSISTENT_MAP=1, ROBOT_PROFILE="vector")
qtpy = str(case / "by-id" / QTPY_NAME)
c1 = str(case / "by-id" / C1_NAME)
run_l = ran(log, "| ENV")
check("rc 0 and READY", rc == 0 and "== READY: vector-dimos.moss-nav ==" in out, f"rc {rc}")
order = [first(log, "py tools/moss_preflight.py"), first(log, " run vector-dimos.moss-nav"), first(log, "curl")]
check("order: preflight -> dimos run -> cockpit curl", -1 not in order and order == sorted(order), str(order))
check("gate 1 runs the preflight with --no-spin (battery + disarmed read) and saves it",
      any("--no-spin" in ln and "--save" in ln for ln in ran(log, "py tools/moss_preflight.py")))
check("one launch: --rerun-open none --nerf-speed 0.5 run vector-dimos.moss-nav --daemon",
      len(run_l) == 1 and "--rerun-open none --nerf-speed 0.5 run vector-dimos.moss-nav --daemon" in run_l[0],
      run_l[0][:140] if run_l else "none")
env_l = run_l[0].split("| ENV ", 1)[1] if run_l and "| ENV " in run_l[0] else ""
for want in ("ROBOT_PROFILE=moss", "PERSISTENT_MAP=0", f"MOSS_PORT={qtpy}", f"MOSS_LIDAR_PORT={c1}",
             "TRANSPORT=lcm", "GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304", "MOSS_MAX_PCT=100",
             "MOSS_RAMP_MS=5 5", "MOSS_RECORD=1", "MOSS_CAMERA=1", f"MOSS_STATE_DIR={case / 'state'}"):
    check(f"stack env: {want.split('=')[0]}={want.split('=', 1)[1][-40:]}", f"{want} " in env_l + " ", env_l[-120:])
check("the shell's PERSISTENT_MAP=1 / ROBOT_PROFILE=vector did not reach the stack",
      "PERSISTENT_MAP=1" not in env_l and "ROBOT_PROFILE=vector" not in env_l)
check("the shell's MOSS_PORT was overridden by the by-id path, and said so",
      "ignoring MOSS_PORT=" in out and f"MOSS_PORT={qtpy}" in env_l)
check("cockpit started on 8904 when absent", any("tools/moss_cockpit.py --port 8904" in ln for ln in ran(log, "nohup")))
check("cockpit probed on 127.0.0.1:8904/state.json",
      any("http://127.0.0.1:8904/state.json" in ln for ln in ran(log, "curl")))
check("prints the cockpit URL and the stop line",
      "cockpit: http://rover-test-address:8904/" in out and "stop:    tools/moss-fly.sh stop" in out)
check("map never touches exploration", not ran(log, "explore_ctl"))
rc, out, log, case = fly(["map"], FLY_TEST_COCKPIT_UP=1)
check("cockpit already answering -> not started twice", rc == 0 and not ran(log, "tools/moss_cockpit.py --port"))
rc, out, log, case = fly(["map"], MOSS_NERF_SPEED="0.3", MOSS_MAX_PCT="60", TRANSPORT="zenoh")
run_l = ran(log, "| ENV")
check("MOSS_NERF_SPEED / MOSS_MAX_PCT / TRANSPORT pass through",
      bool(run_l) and "--nerf-speed 0.3" in run_l[0] and "MOSS_MAX_PCT=60 " in run_l[0] and "TRANSPORT=zenoh " in run_l[0])
rc, out, log, case = fly(["map"], FLY_TEST_PROOF="MOSS base: connected on X|lidar odometry up (kiss-icp)")
check("no 'RPLIDAR C1 up' in 60 s -> refused naming the lidar",
      rc == 1 and "STACK NOT PROVEN" in out and "lidar never up" in out, f"rc {rc}")
check("... the stack is stopped and the cockpit never probed", bool(ran(log, "dimos stop")) and not ran(log, "curl"))
rc, out, log, case = fly(["map"], FLY_TEST_RUNDIR=0,
                         stale_run=["MOSS base: connected on X", "RPLIDAR C1 up on X", "lidar odometry up (kiss-icp)"])
check("an OLD run dir full of proof lines does not prove this launch",
      rc == 1 and "STACK NOT PROVEN" in out and "no new run dir" in out, f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_PROOF="MOSS base: cannot open /dev/serial/by-id/x|Traceback (most recent call last)")
check("a Traceback / cannot open stops the wait early and refuses",
      rc == 1 and "saw:" in out and bool(ran(log, "dimos stop")), f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_PROOF="MOSS base: MOCK BUS (MOSS_MOCK_BUS set)|RPLIDAR C1 up on X|lidar odometry up", MOCK=1)
check("the MOCK BUS line proves the base in MOCK", rc == 0, f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_LAUNCH="error")
check("proofs logged, then the launcher says 'Error:' (a module failed later) -> refused, never READY",
      rc == 1 and "STACK NOT PROVEN" in out and "saw: Error: RuntimeError: Failed to start camera pipeline" in out
      and "READY" not in out and not ran(log, "curl") and bool(ran(log, "dimos stop")), f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_LAUNCH="silent")
check("proofs logged but the launcher never reports success in 60 s -> refused naming it",
      rc == 1 and "never said 'DimOS running in background'" in out and "READY" not in out, f"rc {rc}")
rc, out, log, case = fly(["map"], FLY_TEST_COCKPIT_DEAD=1)
check("dead cockpit -> refused and the stack stopped",
      rc == 1 and "NO COCKPIT" in out and bool(ran(log, "dimos stop")), f"rc {rc}")

rc, out, log, case = fly(["explore"], EXPLORE=1, MOCK=1, ports=("c1",))
ck = ran(log, "cockpit-env")
check("a cockpit started by EXPLORE=1 MOCK=1 explore gets none of the run's switches",
      rc == 0 and ck == ["cockpit-env EXPLORE=unset MOCK=unset MOSS_MOCK_BUS=unset fd9=closed"], str(ck))
check("... while the stack itself still runs on the fake bus", any("MOSS_MOCK_BUS=1 " in ln for ln in ran(log, "| ENV")))

print("E. explore without EXPLORE=1: never armed")
rc, out, log, case = fly(["explore"])
run_l = ran(log, "| ENV")
check("rc 0, stack vector-dimos.moss-explore", rc == 0 and bool(run_l) and "run vector-dimos.moss-explore --daemon" in run_l[0], f"rc {rc}")
check("explore_cmd never published", not ran(log, "explore_ctl.py start"))
check("says exploration is NOT armed", "exploration NOT armed" in out)

print("F. EXPLORE=1 explore: published only after the proofs")
rc, out, log, case = fly(["explore"], EXPLORE=1)
start = first(log, "explore_ctl.py start")
check("rc 0 and 4/4 exploration armed", rc == 0 and "== 4/4 exploration ==" in out and "exploration armed" in out, f"rc {rc}")
check("start published exactly once", len(ran(log, "explore_ctl.py start")) == 1)
check("... after the stack launch and the cockpit proof",
      first(log, " run vector-dimos.moss-explore") < start and first(log, "curl") < start)
check("no stop on the happy path", not ran(log, "explore_ctl.py stop") and not ran(log, "dimos stop"))
rc, out, log, case = fly(["explore"], EXPLORE=1, FLY_TEST_EXPLORE_PROOF=0)
check("no explorer line in 15 s -> refused", rc == 1 and "EXPLORATION NOT PROVEN" in out, f"rc {rc}")
check("... explore stop THEN dimos stop",
      0 <= first(log, "explore_ctl.py stop") < first(log, "dimos stop"), f"{first(log, 'explore_ctl.py stop')} < {first(log, 'dimos stop')}")
rc, out, log, case = fly(["explore"], EXPLORE=1, FLY_TEST_PROOF="MOSS base: connected on X|RPLIDAR C1 up on X")
check("stack not proven -> exploration never published", rc == 1 and not ran(log, "explore_ctl.py start"))
rc, out, log, case = fly(["map"], EXPLORE=1)
check("EXPLORE=1 with map -> contradiction refused, nothing ran",
      rc == 1 and "CONTRADICTORY" in out and log.strip() == "", log[:80])

print("G. stop, log, cockpit, usage")
rc, out, log, case = fly(["stop"])
check("stop: rc 0", rc == 0, f"rc {rc}")
check("explore_ctl stop BEFORE dimos stop", 0 <= first(log, "explore_ctl.py stop") < first(log, "dimos stop"))
check("stop never launches anything", not ran(log, "--daemon") and not ran(log, "moss_preflight"))
rc, out, log, case = fly(["stop"], FLY_TEST_STACK_UP=1, FLY_TEST_SURVIVES=0)
check("stop: still up after dimos stop -> pkill fallback", any("pkill -f" in ln for ln in ran(log, "pkill")))
rc, out, log, case = fly(["log", "5"])
check("log 5: the last 5 filtered lines of a 200-line window",
      rc == 0 and ran(log, "dimos log") == ["dimos log -n 200"]
      and out.splitlines() == [f"MOSS telemetry {i}" for i in range(294, 299)], " / ".join(out.splitlines()[-5:]))
rc, out, log, case = fly(["log", "300"])
lines = out.splitlines()
check("log 300 (300-line run log): reads 1500, shows all 300, startup lines included",
      ran(log, "dimos log") == ["dimos log -n 1500"] and len(lines) == 300
      and lines[:2] == ["MOSS base: connected on X", "RPLIDAR C1 up on X"], f"{len(lines)} lines")
rc, out, log, case = fly(["log"])
check("log: 30 lines by default", rc == 0 and len(out.splitlines()) == 30 and ran(log, "dimos log") == ["dimos log -n 200"])
rc, out, log, case = fly(["log", "x5"])
check("log x5 -> usage, rc 1, dimos never asked", rc == 1 and "usage:" in out and not ran(log, "dimos log"))
rc, out, log, case = fly(["cockpit"])
check("cockpit: starts it and prints the URL",
      rc == 0 and "cockpit: http://rover-test-address:8904/" in out and bool(ran(log, "moss_cockpit.py --port 8904")))
rc, out, log, case = fly(["cockpit"], MOSS_COCKPIT_PORT="8911", FLY_TEST_COCKPIT_DEAD=1)
check("cockpit dead -> rc 1, on the port asked", rc == 1 and "NO COCKPIT on :8911" in out)
rc, out, log, case = fly([])
check("no subcommand -> usage, rc 1, nothing ran", rc == 1 and "usage:" in out and log.strip() == "")
rc, out, log, case = fly(["fly"])
check("unknown subcommand -> usage, rc 1", rc == 1 and "usage:" in out)

print("H. the scripts parse")
for f in (SCRIPT, STUB_FILE):
    check(f"bash -n {f.name}", subprocess.run(["bash", "-n", str(f)]).returncode == 0)
text = SCRIPT.read_text()
check("the FLY_TEST hook comes after set -u", 0 <= text.find("set -u") < text.find("FLY_TEST_STUBS"))
check("no LAN address literal in the script",
      not any(tok.count(".") == 3 and all(p.isdigit() for p in tok.split(".")) and not tok.startswith("127.")
              for tok in text.replace(":", " ").replace("/", " ").split()))

print("I. the cockpit is found by its interpreter, not by its file name (real pgrep)")
cockpit_re = subprocess.run(["bash", "-c", f'eval "$(grep -m1 "^COCKPIT_RE=" {SCRIPT})"; printf %s "$COCKPIT_RE"'],
                            capture_output=True, text=True).stdout
fake_repo = TMPD / "fake_repo"
(fake_repo / "tools").mkdir(parents=True)
(fake_repo / "tools" / "moss_cockpit.py").write_text("import time\nprint('ready', flush=True)\ntime.sleep(30)\n")
procs = {}
for name, argv0 in (("cockpit", "/home/x/vector-dimos/.venv/bin/python"), ("viewer", "less"), ("editor", "vim"),
                    ("ssh line", "ssh")):
    procs[name] = subprocess.Popen(["bash", "-c", f'exec -a "{argv0}" "{sys.executable}" tools/moss_cockpit.py --port 8904'],
                                   cwd=fake_repo, stdout=subprocess.PIPE, text=True)
procs["shell line"] = subprocess.Popen(["bash", "-c", f'cd {fake_repo} && .venv/bin/python tools/moss_cockpit.py; '
                                       f'"{sys.executable}" -c "import time; print(1, flush=True); time.sleep(30)"'],
                                       stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
try:
    for pr in procs.values():
        pr.stdout.readline()
    hits = {int(x) for x in subprocess.run(["pgrep", "-f", cockpit_re], capture_output=True, text=True).stdout.split()}
    check("COCKPIT_RE read from the script", cockpit_re.startswith("^") and "moss_cockpit" in cockpit_re, cockpit_re)
    check("`.venv/bin/python tools/moss_cockpit.py --port 8904` IS the cockpit", procs["cockpit"].pid in hits)
    check("`less` / `vim` / `ssh` / a `bash -c` line naming the file are NOT",
          not hits & {procs[k].pid for k in ("viewer", "editor", "ssh line", "shell line")}, str(hits))
finally:
    for pr in procs.values():
        pr.kill()
        pr.wait()

print("J. moss-cockpit.service: a cockpit restart never ends a flight it started")
svc = ROOT / "tools" / "moss-cockpit.service"
section, killmode = "", "control-group"
for ln in svc.read_text().splitlines():
    ln = ln.strip()
    if ln.startswith("["):
        section = ln
    elif section == "[Service]" and ln.startswith("KillMode="):
        killmode = ln.split("=", 1)[1]
check("[Service] KillMode=process", killmode == "process", killmode)


def survivor_after_main_exits(mode):
    """A transient user unit with this KillMode whose main process starts a
    new-session child (as Launcher.start does), then exits. Returns whether the
    child is still alive once systemd is done with the unit, or None without
    a user systemd."""
    unit = f"moss-killmode-bench-{os.getpid()}-{mode}"
    pidf = TMPD / f"child_{mode}"
    r = subprocess.run(["systemd-run", "--user", "--quiet", "--collect", f"--unit={unit}", "-p", f"KillMode={mode}",
                        "bash", "-c", f'setsid sleep 30 < /dev/null > /dev/null 2>&1 & echo $! > {pidf}; '
                        f'while [ ! -s {pidf} ]; do :; done'], capture_output=True, text=True)
    if r.returncode != 0:
        return None
    t0 = time.time()
    while time.time() - t0 < 10:
        st = subprocess.run(["systemctl", "--user", "is-active", unit], capture_output=True, text=True).stdout.strip()
        if pidf.exists() and pidf.read_text().strip() and st not in ("active", "activating", "deactivating"):
            break
        time.sleep(0.1)
    pid = int(pidf.read_text().strip())
    t0 = time.time()
    while time.time() - t0 < 3 and os.path.exists(f"/proc/{pid}"):   # SIGTERM takes a moment
        if mode == "process":
            break
        time.sleep(0.1)
    alive = os.path.exists(f"/proc/{pid}")
    if alive:
        os.kill(pid, 9)
    subprocess.run(["systemctl", "--user", "stop", unit], capture_output=True)
    return alive


if shutil.which("systemd-run"):
    with_file, default = survivor_after_main_exits(killmode), survivor_after_main_exits("control-group")
    if with_file is None or default is None:
        print("  --  no user systemd here: the live KillMode check is skipped")
    else:
        check("systemd, the unit's KillMode: the launcher it started outlives the cockpit", with_file is True)
        check("... control check: the default control-group would have killed it", default is False)
else:
    print("  --  no systemd-run here: the live KillMode check is skipped")

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

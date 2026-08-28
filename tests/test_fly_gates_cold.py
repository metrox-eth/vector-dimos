#!/usr/bin/env python3
"""Cold bench for tools/fly.sh: the flag matrix (PERSISTENT_MAP, STOCK_NAV,
EXPLORE, DRY, TRANSPORT) x the gate the operator actually gets.

The REAL fly.sh runs here, gate logic untouched: FLY_TEST=1 makes it source a
stub file (written below) where ssh, rsync, curl, pkill, nohup, xdotool, ss and
sleep become recorders. No rover, no screen, no waiting - and every remote
command the flight would have run is readable in $FLY_TEST_LOG.

  A. contradiction - EXPLORE=1 with an EXPLICIT DRY=1 refuses before touching
                     anything (it used to silently fly the autonomous one)
  B. the default   - no flag = piloted lap: exploration never armed
  C. gate 8/8      - persistent custom mode: still a GATE (46 x 15 s = 690 s of
                     grace, then explore stop + e-stop + dimos stop)
  D. the two modes where the gate's four log lines CANNOT be printed
                     (PERSISTENT_MAP=0, STOCK_NAV=1): a warning, not an abort -
                     the flight lives and says the zones are inactive
  E. gate 2/7      - STOCK_NAV=1 over a flat with zones drawn: stock nav has no
                     zone concept, so the operator is warned BEFORE the stack
                     starts - and only there (custom nav applies them)
  F. transport     - every ssh that publishes or arms carries the run's bus
  G. the watchdog  - TRANSPORT crosses arm_garde_vitesse.sh into the python it
                     launches (known input zenoh -> known output zenoh)
  I. already flying - a live stack is refused BEFORE any kill, any sync and any
                     e-stop: the running flight keeps its panel and its cockpit
  J. gate 0/7      - on a REAL pty (the interactive path): the hand-push step is
                     offered only when estop_rs485.py exited 0 on both drives
  K. mode flags    - GAMEPAD, STOCK_NAV and PERSISTENT_MAP cross the ssh into
                     BOTH preflights, not only into the stack launch
  H. bash -n on fly.sh, the arm script and the stubs

Run:   PYTHONPATH=. .venv/bin/python3 tests/test_fly_gates_cold.py
       ... tests/test_fly_gates_cold.py <a copy of fly.sh>   (the FLY_TEST hook
       is injected if that copy predates it - this is how the pre-fix bite was
       run on 2026-08-28: `git show HEAD:tools/fly.sh` failed A, D and E with
       the audited symptoms.)

Side effect: fly.sh redirects its viewer/relay launches into /tmp/dimos_viewer.log
and /tmp/udp_forward_rig.log, so those two are truncated. Nothing is launched
and nothing is killed. Do not run it while a flight is in the air.
"""

import os
import pty
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FLY = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "tools" / "fly.sh"
ARM = ROOT / "tools" / "arm_garde_vitesse.sh"
OK = 0
KO = 0

STUBS = r"""# Stand-ins sourced by fly.sh under FLY_TEST=1 (tests/test_fly_gates_cold.py).
# Everything that reaches the rover, the screen or the clock is recorded instead.
: "${FLY_TEST_LOG:?}"
: > "$FLY_TEST_LOG"
FLY_TEST_RELOC="${FLY_TEST_RELOC:-}"     # what gate 8/8 finds in the run log
FLY_TEST_KEEPOUT="${FLY_TEST_KEEPOUT:-0}"   # 1 = keepout.json exists on the rover
FLY_TEST_STACK_UP="${FLY_TEST_STACK_UP:-0}" # 1 = a dimOS stack is ALREADY flying
FLY_TEST_SYNC_HIT="${FLY_TEST_SYNC_HIT:-0}" # 1 = the rsync updated stats_server.py
FLY_TEST_ESTOP_RC="${FLY_TEST_ESTOP_RC:-0}" # what tests/estop_rs485.py exits with
                                            # (255 = the ssh itself never landed)

_flyrec() { printf '%s' "$*" | tr '\n' ' ' >> "$FLY_TEST_LOG"; printf '\n' >> "$FLY_TEST_LOG"; }

ssh() {
  _flyrec "ssh $*"
  case "$*" in
    *"date +%s"*)                       echo 1755000000 ;;
    *estop_rs485.py*)
      case "$FLY_TEST_ESTOP_RC" in                     # the real tool's own lines
        0) echo "unit 1 (front): rpm 0/0 acked=True disable acked=True feedback rpm=(0, 0)"
           echo "E-STOP DONE: both drives stopped and disabled" ;;
        1) echo "port /dev/ttyUSB0 did not open - cut the motor power" ;;
        *) ;;                                          # dead ssh: not one line
      esac
      return "$FLY_TEST_ESTOP_RC" ;;
    *"[b]in/dimos"*)                    [ "$FLY_TEST_STACK_UP" = "1" ]; return $? ;;
    *wt_url*)                           echo 4433 ;;
    *keepout.json*)                     [ "$FLY_TEST_KEEPOUT" = "1" ]; return $? ;;
    *"CONTINUING the persistent map"*)  [ -n "$FLY_TEST_RELOC" ] && echo "$FLY_TEST_RELOC" ;;
    *"prior: gyro="*)                   echo "prior: gyro=yes rot=0.02" ;;
  esac
  return 0
}
rsync()   { _flyrec "rsync $*"; [ "$FLY_TEST_SYNC_HIT" = "1" ] && echo ">f.st...... tools/stats_server.py"; return 0; }
curl()    { _flyrec "curl $*"; printf '%s' '{"sensors":{"software":{"monitoring":{"alive":true}}}}'; return 0; }
pkill()   { _flyrec "pkill $*"; return 0; }
nohup()   { _flyrec "nohup $*"; return 0; }
xdotool() { _flyrec "xdotool $*"; return 1; }
ss()      { _flyrec "ss $*"; return 1; }
sleep()   { return 0; }
timeout() { shift; "$@"; }
"""

FAKE_WATCHDOG = """import os, time
open(os.environ["GARDE_SEEN"], "w").write(os.environ.get("TRANSPORT", "<unset>"))
open(os.environ["GARDE_PID_FILE"], "w").write(str(os.getpid()))
time.sleep(20)
"""

TMP = tempfile.TemporaryDirectory()
TMPD = Path(TMP.name)
STUB_FILE = TMPD / "fly_stubs.sh"
STUB_FILE.write_text(STUBS)


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def flyable(path: Path) -> Path:
    """The script under test, with the FLY_TEST hook injected if it predates it."""
    text = path.read_text()
    if "FLY_TEST_STUBS" in text:
        return path
    patched = text.replace("set -u\n", 'set -u\n[ "${FLY_TEST:-0}" = "1" ] && . "$FLY_TEST_STUBS"\n', 1)
    out = TMPD / "fly_prefix.sh"
    out.write_text(patched)
    print(f"  (FLY_TEST hook injected into {path} - gate logic verbatim)")
    return out


SCRIPT = flyable(FLY)


def _env(reloc, env):
    log = TMPD / f"run_{len(list(TMPD.glob('run_*')))}.log"
    e = dict(os.environ)
    e.pop("EXPLORE", None)
    e.pop("DRY", None)
    e.update(
        FLY_TEST="1",
        FLY_TEST_STUBS=str(STUB_FILE),
        FLY_TEST_LOG=str(log),
        FLY_TEST_RELOC=reloc,
        FLY_TEST_KEEPOUT="0",   # no zones drawn on the rover unless a case says so
        FLY_TEST_STACK_UP="0",  # nothing already flying unless a case says so
        FLY_TEST_SYNC_HIT="0",  # the sync touched no stats_server.py
        FLY_TEST_ESTOP_RC="0",  # both drives ack the release unless a case says so
        REPOSITIONNE="1",       # detached path: no interactive read, no e-stop prompt
    )
    e.update({k: str(v) for k, v in env.items()})
    return log, e


def fly(reloc="", **env):
    """Run the whole flight cold. Returns (rc, what the operator saw, what it ran)."""
    log, e = _env(reloc, env)
    p = subprocess.run(["bash", str(SCRIPT)], env=e, capture_output=True, text=True, timeout=180)
    return p.returncode, p.stdout + p.stderr, (log.read_text() if log.exists() else "")


def fly_tty(reloc="", **env):
    """The INTERACTIVE path (REPOSITIONNE unset): a REAL pty on stdin, so fly.sh's
    own `[ -t 0 ]` is true, the torque release runs and the read prompt is shown.
    The answer is pre-typed, so the gate never waits for a human."""
    log, e = _env(reloc, env)
    e["REPOSITIONNE"] = "0"
    master, slave = pty.openpty()
    os.write(master, b"\n")          # the operator's Enter, already in the buffer
    try:
        p = subprocess.run(["bash", str(SCRIPT)], env=e, stdin=slave,
                           capture_output=True, text=True, timeout=180)
    finally:
        os.close(slave)
        os.close(master)
    return p.returncode, p.stdout + p.stderr, (log.read_text() if log.exists() else "")


def ran(log, needle):
    return [ln for ln in log.splitlines() if needle in ln]


print("A. EXPLORE=1 and an explicit DRY=1 are contradictory flags")
rc, out, log = fly(EXPLORE=1, DRY=1)
check("refused (rc 1)", rc == 1, f"rc {rc}")
check("says the flags contradict", "CONTRADICTORY FLAGS" in out, out.strip().splitlines()[-1] if out.strip() else "")
check("nothing was touched: no rsync, no ssh", log.strip() == "", log[:80])
rc, out, log = fly(EXPLORE=1, DRY=0, reloc="CONTINUING the persistent map")
check("EXPLORE=1 DRY=0 flies autonomous", rc == 0 and "== 7/8 exploration ==" in out, f"rc {rc}")
rc, out, log = fly(EXPLORE=1, reloc="CONTINUING the persistent map")
check("EXPLORE=1 alone flies autonomous", rc == 0 and "== 7/8 exploration ==" in out, f"rc {rc}")

print("B. no flag = the piloted lap, exploration never armed")
rc, out, log = fly()
check("rc 0 and 7/7 DRY", rc == 0 and "== 7/7 DRY" in out, f"rc {rc}")
check("exploration never started", not ran(log, "explore_ctl.py start"))
check("the watchdog was never armed", not ran(log, "arm_garde_vitesse"))
rc, out, log = fly(DRY=0, reloc="CONTINUING the persistent map")
check("DRY=0 alone arms the flight", rc == 0 and "== 7/8 exploration ==" in out, f"rc {rc}")

print("C. persistent custom mode: gate 8/8 still stops the flight")
rc, out, log = fly(EXPLORE=1)   # PERSISTENT_MAP defaults to 1, STOCK_NAV to 0
check("never relocalized -> rc 1", rc == 1 and "NEVER UNDERSTOOD WHERE IT IS" in out, f"rc {rc}")
polls = ran(log, "CONTINUING the persistent map")
check("polled 46 x 15 s = 690 s of grace", len(polls) == 46, f"{len(polls)} polls")
check("the abort stops exploration", len(ran(log, "explore_ctl.py stop")) == 1)
check("the abort e-stops the wheels", len(ran(log, "estop_rs485.py")) == 1)
rc, out, log = fly(EXPLORE=1, reloc="CONTINUING the persistent map")
check("relocalized -> IN FLIGHT", rc == 0 and "IN FLIGHT" in out and "relocalized (1x15 s)" in out, f"rc {rc}")
check("relocalized -> no e-stop", not ran(log, "estop_rs485.py"))
rc, out, log = fly(EXPLORE=1, reloc="relocalization: gave up")
check("gave up -> the gate still bites", rc == 1 and "NEVER UNDERSTOOD WHERE IT IS" in out, f"rc {rc}")

print("D. the modes where those four lines cannot be printed: a WARNING, not an abort")
for label, env in (("PERSISTENT_MAP=0", dict(PERSISTENT_MAP=0)),
                   ("STOCK_NAV=1", dict(STOCK_NAV=1)),
                   ("both", dict(PERSISTENT_MAP=0, STOCK_NAV=1))):
    rc, out, log = fly(EXPLORE=1, **env)
    check(f"{label}: flight lives (rc 0, IN FLIGHT)", rc == 0 and "IN FLIGHT" in out, f"rc {rc}")
    check(f"{label}: warns the gate is not enforced", "GATE 8/8 NOT ENFORCED" in out)
    check(f"{label}: names the mode and the missing zones",
          f"PERSISTENT_MAP={env.get('PERSISTENT_MAP', 1)} STOCK_NAV={env.get('STOCK_NAV', 0)}" in out
          and "NO keep-out zones" in out)
    check(f"{label}: no 11.5 min poll", not ran(log, "CONTINUING the persistent map"))
    check(f"{label}: no e-stop, no explore stop",
          not ran(log, "estop_rs485.py") and not ran(log, "explore_ctl.py stop"))
rc, out, log = fly(EXPLORE=1, ALLOW_FRESH=1)
check("ALLOW_FRESH=1 still skips the gate", rc == 0 and "ALLOW_FRESH=1" in out and not ran(log, "CONTINUING"), f"rc {rc}")

print("E. STOCK_NAV=1 over a flat with zones drawn: warned at gate 2/7, never refused")
rc, out, log = fly(EXPLORE=1, STOCK_NAV=1, FLY_TEST_KEEPOUT=1)
check("the flight lives (rc 0, IN FLIGHT)", rc == 0 and "IN FLIGHT" in out, f"rc {rc}")
check("names the file and what stock nav does not apply",
      "STOCK_NAV=1 AND ZONES DRAWN" in out and "keepout.json" in out and "NO keep-out" in out)
warn_at, stack_at = out.find("AND ZONES DRAWN"), out.find("== 3/7 stack ==")
check("warned BEFORE the stack is launched", 0 <= warn_at < stack_at,
      f"warning at {warn_at}, stack launch at {stack_at}")
check("the warning stops nothing: no e-stop, no dimos stop",
      not ran(log, "estop_rs485.py") and not ran(log, "dimos stop"))
rc, out, log = fly(EXPLORE=1, STOCK_NAV=1, FLY_TEST_KEEPOUT=0)
check("stock nav, no zones drawn -> silent", rc == 0 and "AND ZONES DRAWN" not in out, f"rc {rc}")
rc, out, log = fly(EXPLORE=1, FLY_TEST_KEEPOUT=1, reloc="CONTINUING the persistent map")
check("custom nav applies the zones -> no warning", rc == 0 and "AND ZONES DRAWN" not in out, f"rc {rc}")
check("custom nav: the rover is not even asked for keepout.json", not ran(log, "keepout.json"))

print("F. the run's bus crosses every ssh that publishes or arms")
rc, out, log = fly(EXPLORE=1, TRANSPORT="zenoh", reloc="CONTINUING the persistent map")
pub = ran(log, "explore_ctl.py")
check("explore start published on zenoh", len(pub) == 1 and "TRANSPORT=zenoh" in pub[0], pub[0][-70:] if pub else "none")
arm = ran(log, "arm_garde_vitesse.sh")
check("the watchdog is armed with zenoh", len(arm) == 1 and "TRANSPORT=zenoh" in arm[0], arm[0][-60:] if arm else "none")
stack = ran(log, "dimos --rerun-open")
check("the stack runs on zenoh", len(stack) == 1 and "TRANSPORT=zenoh " in stack[0])
rc, out, log = fly(EXPLORE=1, TRANSPORT="zenoh")     # abort path
stops = ran(log, "explore_ctl.py stop")
check("the abort's stop is published on zenoh too", len(stops) == 1 and "TRANSPORT=zenoh" in stops[0],
      stops[0][-70:] if stops else "none")
rc, out, log = fly(EXPLORE=1, reloc="CONTINUING the persistent map")
check("no TRANSPORT set -> lcm everywhere it matters",
      all("TRANSPORT=lcm" in ln for ln in ran(log, "explore_ctl.py") + ran(log, "arm_garde_vitesse.sh")),
      f"{len(ran(log, 'explore_ctl.py') + ran(log, 'arm_garde_vitesse.sh'))} lines")

print("G. TRANSPORT crosses arm_garde_vitesse.sh into the watchdog process")
fake = TMPD / "fake_garde_vitesse.py"
fake.write_text(FAKE_WATCHDOG)
for want, env in (("zenoh", {"TRANSPORT": "zenoh"}), ("lcm", {})):
    seen = TMPD / f"seen_{want}"
    pid_file = TMPD / f"pid_{want}"
    e = dict(os.environ)
    e.pop("TRANSPORT", None)
    e.update(env, GARDE_PID_FILE=str(pid_file), GARDE_PY=str(fake), GARDE_SEEN=str(seen),
             GARDE_PYTHON=sys.executable, GARDE_LOG=str(TMPD / f"log_{want}"))
    # the real arm script, launching a watchdog that only reports the env it got
    p = subprocess.run(["bash", str(ARM)], env=e, capture_output=True, text=True, timeout=40)
    got = seen.read_text() if seen.exists() else "<never launched>"
    check(f"the watchdog's env says TRANSPORT={want}", got == want, f"got {got!r} (arm rc {p.returncode})")
    try:
        os.kill(int(pid_file.read_text().strip()), 9)
    except (OSError, ValueError):
        pass

print("I. a stack already in the air: refused before ANYTHING is touched")
rc, out, log = fly(FLY_TEST_STACK_UP=1, FLY_TEST_SYNC_HIT=1)
check("refused (rc 1)", rc == 1 and "A DIMOS STACK IS ALREADY RUNNING" in out, f"rc {rc}")
kills = ran(log, "kill")
check("ZERO kill invoked on the live flight", not kills, kills[0][:90] if kills else "")
check("its panel, sonar readout, cockpit and relays survive",
      not (ran(log, "stats_server") or ran(log, "sonar_live")
           or ran(log, "deno") or ran(log, "udp_forward")))
check("its wheels are not e-stopped", not ran(log, "estop_rs485.py"))
check("its code is not even overwritten", not ran(log, "rsync"))
check("the refusal is the FIRST rover command",
      bool(log.splitlines()) and "[b]in/dimos" in log.splitlines()[0],
      log.splitlines()[0][:90] if log.splitlines() else "nothing ran")
check("no preflight, no stack launch", not ran(log, "preflight") and not ran(log, "dimos --rerun-open"))
rc, out, log = fly(FLY_TEST_SYNC_HIT=1)
check("no stack in the air -> the cleanup runs, unchanged",
      rc == 0
      and len(ran(log, '[s]tats_server"); do kill')) == 1   # synced: retire the old one
      and len(ran(log, "[s]onar_live")) == 1
      and len(ran(log, "pgrep -x deno")) == 1
      and len(ran(log, "rsync")) == 1, f"rc {rc}")
check("and the cleanup comes AFTER the refusal it used to precede",
      log.find("[b]in/dimos") < log.find("[s]onar_live"))

print("J. gate 0/7 interactive: the hand-push is offered only if the torque IS off")
for label, rc_estop, reason in (("a drive NAKs / the port is busy", 1, "did not open"),
                                ("the ssh never landed (not one line)", 255, "")):
    rc, out, log = fly_tty(FLY_TEST_ESTOP_RC=rc_estop)
    check(f"{label}: the interactive path really ran", "lancement detache" not in out
          and len(ran(log, "estop_rs485.py")) == 1, f"{len(ran(log, 'estop_rs485.py'))} e-stops")
    check(f"{label}: refused (rc 1)", rc == 1, f"rc {rc}")
    check(f"{label}: NEVER says the rover can be hand-pushed", "se pousse a la main" not in out)
    check(f"{label}: the operator is never asked to confirm", "Rover repositionne au" not in out)
    check(f"{label}: says the torque is NOT released", "COUPLE MOTEUR NON RELACHE" in out)
    if reason:
        check(f"{label}: keeps the tool's own reason", reason in out)
    check(f"{label}: the flight stops there", not ran(log, "preflight") and not ran(log, "[s]onar_live"))
rc, out, log = fly_tty(FLY_TEST_ESTOP_RC=0)
check("both drives acked: the release is announced and the prompt is reached",
      "E-STOP DONE" in out and "se pousse a la main" in out and "Rover repositionne au" in out)
check("and the flight goes on (rc 0, 7/7 DRY)", rc == 0 and "== 7/7 DRY" in out, f"rc {rc}")
check("no COUPLE MOTEUR NON RELACHE on the happy path", "COUPLE MOTEUR NON RELACHE" not in out)

print("K. the mode flags reach BOTH preflights, not just the stack")
rc, out, log = fly(GAMEPAD=1, STOCK_NAV=1, PERSISTENT_MAP=0)
pre = ran(log, "preflight.py") + ran(log, "preflight_nav.py")
check("both preflights ran", len(pre) == 2, f"{len(pre)} lines")
check("the pad gate can fire: GAMEPAD=1 in both",
      all("GAMEPAD=1" in ln for ln in pre), pre[0][-80:] if pre else "none")
check("the nav mode crosses too: STOCK_NAV=1 PERSISTENT_MAP=0 in both",
      all("STOCK_NAV=1" in ln and "PERSISTENT_MAP=0" in ln for ln in pre))
check("the stack still gets them", any("GAMEPAD=1" in ln and "STOCK_NAV=1" in ln
                                       for ln in ran(log, "dimos --rerun-open")))
rc, out, log = fly()
pre = ran(log, "preflight.py") + ran(log, "preflight_nav.py")
check("default flight: GAMEPAD=0 in both (the pad stays informative)",
      len(pre) == 2 and all("GAMEPAD=0" in ln for ln in pre), pre[0][-60:] if pre else "none")
check("default flight: STOCK_NAV=0 PERSISTENT_MAP=1 in both",
      all("STOCK_NAV=0" in ln and "PERSISTENT_MAP=1" in ln for ln in pre))
# the other end of the wire: the gate on the rover reads the name we send
check("preflight.py's pad gate reads that very variable",
      'os.environ.get("GAMEPAD", "0") == "1"' in (ROOT / "tools" / "preflight.py").read_text())

print("H. the scripts parse")
for f in (FLY, ARM, STUB_FILE):
    check(f"bash -n {f.name}", subprocess.run(["bash", "-n", str(f)]).returncode == 0)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

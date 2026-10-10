#!/usr/bin/env bash
# MOSS flight: gates -> mapping or exploration stack -> cockpit. Runs ON THE
# ROVER (the Jetson, where the board, the lidar and the cameras are plugged in):
#
#     tools/moss-fly.sh check              flight check (tools/moss_preflight.py), nothing launched
#     tools/moss-fly.sh map                gates, then vector-dimos.moss-nav (mapping + pad, recording)
#     tools/moss-fly.sh explore            gates, then vector-dimos.moss-explore; exploration NOT armed
#     EXPLORE=1 tools/moss-fly.sh explore  ... and arm exploration once the stack is proven
#     tools/moss-fly.sh stop               stop exploration, then the stack
#     tools/moss-fly.sh log [N]            what the base, the lidar, the map and the pad said
#     tools/moss-fly.sh cockpit            start the cockpit (tools/moss_cockpit.py) if needed, print its URL
#     MOCK=1 tools/moss-fly.sh map         fake firmware (MOSS_MOCK_BUS=1); the lidar is still real
#
# From another machine: ssh <user>@<jetson> 'cd ~/vector-dimos && tools/moss-fly.sh map'
# tools/moss-teleop.sh stays the pad-only launcher.
#
# Every gate REFUSES rather than guesses (the fly.sh doctrine), with the fix on
# the same line. Nothing moves by itself: in `map` only the pad drives (deadman
# held); in `explore` the planner moves only on a goal or once exploration is
# armed (EXPLORE=1 here, or the cockpit's button). The firmware stops the
# tracks 300 ms after the last command whatever happens up here.
set -u
cd "$(dirname "$0")/.." || exit 1
DIMOS=.venv/bin/dimos
PY=.venv/bin/python
BYID_DIR=/dev/serial/by-id
LOG_ROOT="$HOME/.local/state/dimos/logs"
LAUNCH_LOG=/tmp/moss_fly_launch.log
COCKPIT_LOG=/tmp/moss_cockpit.log
# One check/map/explore at a time: a second launcher would sweep the first one's
# half-built stack as a zombie, or open the same board (fly.sh, 30/08).
FLY_LOCK=/tmp/moss_fly.lock
# `dimos [root options] run <bp>`: the options sit between dimos and run, so a
# plain "bin/dimos run" never matches a real launch.
STACK_RE='[b]in/dimos( [^ ]+)* run( |$)'
# The cockpit itself (an interpreter running it), not an editor, a viewer or a
# shell line that only names the file.
COCKPIT_RE='^[^ ]*python[0-9.]* ([^ ]*/)?[t]ools/moss_cockpit\.py( |$)'
PROOF_S=${PROOF_S:-120}   # the lidar odometry (kiss-icp import) took over 60 s on a cold cache, 11/10
EXPLORE_PROOF_S=15
# FLY_TEST=1 swaps every command that reaches the stack, the clock or the
# network for a recorder (tests/test_moss_fly_gates_cold.py). Nothing else sets it.
if [ "${FLY_TEST:-0}" = "1" ]; then . "${FLY_TEST_STUBS:?FLY_TEST=1 needs FLY_TEST_STUBS}"; fi

usage() {
  echo "usage: tools/moss-fly.sh check|map|explore|stop|log [N]|cockpit   (EXPLORE=1 with explore, MOCK=1 with map/explore)"
}

# The stack's environment, exported once so the flight check sees exactly what
# the stack will see.
export GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304   # Jetson TLS reserve, as fly.sh
export TRANSPORT="${TRANSPORT:-lcm}"
export ROBOT_PROFILE=moss                     # MOSS geometry in every vector_dimos module
export PERSISTENT_MAP=0                       # fresh map each run: the saved map on this Jetson is not MOSS's
export MOSS_MAX_PCT="${MOSS_MAX_PCT:-100}"    # firmware limit 100; lower it for a cautious lap
export MOSS_RAMP_MS="${MOSS_RAMP_MS:-5 5}"    # firmware ramp, ms per % up/down
export MOSS_STATE_DIR="${MOSS_STATE_DIR:-$HOME/.local/state/moss}"
export MOSS_RECORD="${MOSS_RECORD:-1}"
export MOSS_CAMERA="${MOSS_CAMERA:-1}"
NERF_SPEED="${MOSS_NERF_SPEED:-0.5}"
COCKPIT_PORT="${MOSS_COCKPIT_PORT:-8904}"
PREFLIGHT_JSON="$MOSS_STATE_DIR/preflight.json"   # served by the cockpit as /preflight.json
if [ "${MOCK:-0}" = "1" ]; then export MOSS_MOCK_BUS=1; fi

# The ports come from by-id, never from the shell: an old export of MOSS_PORT
# (an earlier doc pointed it at the CP2102N) would aim motor commands at the lidar.
resolve_ports() {
  QTPY=$(ls "$BYID_DIR"/*QT_Py* 2>/dev/null | head -1)
  C1=$(ls "$BYID_DIR"/*CP2102N* 2>/dev/null | head -1)
  if [ -n "${MOSS_PORT:-}" ] && [ "$MOSS_PORT" != "$QTPY" ]; then
    echo "  (ignoring MOSS_PORT=$MOSS_PORT from the shell: the by-id path wins)"
  fi
  if [ -n "${MOSS_LIDAR_PORT:-}" ] && [ "$MOSS_LIDAR_PORT" != "$C1" ]; then
    echo "  (ignoring MOSS_LIDAR_PORT=$MOSS_LIDAR_PORT from the shell: the by-id path wins)"
  fi
  if [ -n "$QTPY" ]; then export MOSS_PORT="$QTPY"; else unset MOSS_PORT; fi
  if [ -n "$C1" ]; then export MOSS_LIDAR_PORT="$C1"; else unset MOSS_LIDAR_PORT; fi
}

cockpit_url() {
  local host
  host=$(hostname -I 2>/dev/null | awk '{print $1}')
  echo "http://${host:-127.0.0.1}:$COCKPIT_PORT/"
}

cockpit_answers() {
  curl -sf -m 3 -o /dev/null "http://127.0.0.1:$COCKPIT_PORT/state.json"
}

# Passive server (bus listener, never a serial port): start it when absent,
# restart it when its file is newer than the process (fly.sh gate 5 lesson: an
# old server keeps serving old code on a live port after a sync).
cockpit_up() {
  local p
  p=$(pgrep -f "$COCKPIT_RE" | head -1)
  if [ -n "$p" ] && [ "$(stat -c %Y tools/moss_cockpit.py)" -gt "$(stat -c %Y "/proc/$p" 2>/dev/null || echo 0)" ]; then
    echo "  cockpit pid $p is older than tools/moss_cockpit.py - restarting it"
    if command -v systemctl >/dev/null 2>&1 && systemctl is-active --quiet moss-cockpit 2>/dev/null; then
      # the unit owns the port: restart it, never race it with a nohup twin
      echo "  it runs under moss-cockpit.service - restarting the unit"
      sudo -n systemctl restart moss-cockpit 2>/dev/null || systemctl restart moss-cockpit 2>/dev/null
      sleep 2; cockpit_answers && return 0; return 1
    fi
    kill "$p" 2>/dev/null; sleep 1; p=""
  fi
  cockpit_answers && return 0
  if [ -z "$p" ]; then
    # Without this run's switches: a cockpit born under EXPLORE=1 or MOCK=1 would
    # pass them to every launch from its page. fd 9 (the launch lock) stays here.
    ( unset EXPLORE MOCK MOSS_MOCK_BUS
      nohup $PY tools/moss_cockpit.py --port "$COCKPIT_PORT" > "$COCKPIT_LOG" 2>&1 < /dev/null 9>&- & )
  fi
  for _ in $(seq 1 15); do
    sleep 1
    cockpit_answers && return 0
  done
  return 1
}

take_lock() {
  exec 9>>"$FLY_LOCK" || { echo "CANNOT OPEN $FLY_LOCK - remove it if another user owns it"; exit 1; }
  flock -n 9 || { echo "ANOTHER moss-fly.sh IS RUNNING (check, map or explore) - wait for it, or tools/moss-fly.sh stop"; exit 1; }
}

stack_down() {
  $DIMOS stop 2>&1 | tail -2
  sleep 1
  if pgrep -f "$STACK_RE" >/dev/null; then
    echo "dimos still up - killing"; pkill -f "$STACK_RE"
  fi
}

CMD="${1:-}"
case "$CMD" in
  stop)
    # exploration first: the explorer stops publishing goals before the stack goes
    $PY tools/explore_ctl.py stop 2>&1 | tail -1
    stack_down
    echo "stack down - the tracks stopped on the firmware deadman (300 ms) before this line"
    exit 0 ;;
  log)
    N="${2:-30}"
    case "$N" in ''|*[!0-9]*) usage; exit 1 ;; esac
    # dimos log keeps its last 50 lines by default: filter a window wide enough
    # that the startup lines survive the 2 s telemetry chatter.
    W=$(( N * 5 > 200 ? N * 5 : 200 ))
    $DIMOS log -n "$W" 2>/dev/null | grep -a -E "MOSS|RPLIDAR|lidar odom|costmap|Gamepad|Traceback|Error" | tail -n "$N"
    exit 0 ;;
  cockpit)
    cockpit_up || { echo "NO COCKPIT on :$COCKPIT_PORT - see $COCKPIT_LOG"; exit 1; }
    echo "cockpit: $(cockpit_url)"
    exit 0 ;;
  check)
    take_lock
    resolve_ports
    $PY tools/moss_preflight.py --save "$PREFLIGHT_JSON"
    exit $? ;;
  map) BP=moss-nav ;;
  explore) BP=moss-explore ;;
  *) usage; exit 1 ;;
esac

if [ "${EXPLORE:-0}" = "1" ] && [ "$CMD" != "explore" ]; then
  echo "CONTRADICTORY: EXPLORE=1 with '$CMD' - the mapping stack has no explorer; use: EXPLORE=1 tools/moss-fly.sh explore"; exit 1
fi
take_lock
N=3; [ "${EXPLORE:-0}" = "1" ] && N=4

echo "== 0/$N no dimOS stack already up =="
# A dimos process WITH a registered run is a live flight: refuse, untouched. One
# WITHOUT (the CLI says "No running DimOS instance") is either still building
# (dimOS registers a run only after its build and health check) or a zombie
# holding the ports. Younger than a proof window = building: refuse, untouched.
# Older, with that explicit answer = zombie: swept.
if pgrep -f "$STACK_RE" >/dev/null; then
  if ! $DIMOS status 2>/dev/null | grep -q "No running DimOS instance"; then
    echo "A DIMOS STACK IS ALREADY RUNNING - tools/moss-fly.sh stop first"; exit 1
  fi
  for sp in $(pgrep -f "$STACK_RE"); do
    age=$(ps -o etimes= -p "$sp" 2>/dev/null | tr -d ' ')
    if [ -n "$age" ] && [ "$age" -lt $((PROOF_S + 30)) ]; then
      echo "A DIMOS STACK IS STILL STARTING (pid $sp, ${age} s old, not registered yet) - wait for it, or tools/moss-fly.sh stop"; exit 1
    fi
  done
  echo "  zombie dimos process (no registered run) - killing it (the firmware deadman already stopped the tracks)"
  pkill -9 -f "$STACK_RE"; sleep 2
  if pgrep -f "$STACK_RE" >/dev/null; then
    echo "THE ZOMBIE SURVIVED kill -9 - no flight, look at it by hand: pgrep -af 'bin/dimos'"; exit 1
  fi
fi
echo "  none"

echo "== 1/$N flight check =="
resolve_ports
if [ "${MOCK:-0}" != "1" ] && [ -z "$QTPY" ]; then
  echo "NO QT PY on USB ($BYID_DIR/*QT_Py*) - cable, power, or the board is in its bootloader"; exit 1
fi
if [ -z "$C1" ]; then
  echo "NO LIDAR on USB ($BYID_DIR/*CP2102N*) - lidar cable or hub (mapping needs it, MOCK or not)"; exit 1
fi
[ "${MOCK:-0}" = "1" ] && echo "  MOCK=1: fake firmware, the board is not touched"
# --no-spin, not --quick: gate 2 proves the lidar live, but only this listen
# reads the battery and proves the firmware disarmed before the stack opens it.
$PY tools/moss_preflight.py --no-spin --save "$PREFLIGHT_JSON" \
  || { echo "FLIGHT CHECK KO - no flight: fix the KO lines above (tools/moss-fly.sh check for the full check)"; exit 1; }

echo "== 2/$N stack: vector-dimos.$BP =="
: > "$LAUNCH_LOG"
LAUNCH_MARK=$(date +%s)
nohup $DIMOS --rerun-open none --nerf-speed "$NERF_SPEED" run "vector-dimos.$BP" --daemon > "$LAUNCH_LOG" 2>&1 < /dev/null 9>&- &
# The proof is read in THIS launch's run dir (created after LAUNCH_MARK): an
# older run's log also says "RPLIDAR C1 up" and once passed a dead launch as
# alive (fly.sh, 26/08). The three log lines are not enough: the build starts
# every module in parallel, and one failing after them (the camera) tears the
# whole stack down. Only the launcher's own success line closes the build.
PROOF_RE="MOSS base: connected|MOSS base: MOCK BUS|RPLIDAR C1 up|lidar odometry up|Traceback|cannot open|no telemetry"
LAUNCH_OK_RE="DimOS running in background"
RUN_DIR=""; OK_BASE=0; OK_LIDAR=0; OK_ODOM=0; OK_LAUNCH=0; BAD=""
for _ in $(seq 1 "$PROOF_S"); do
  sleep 1
  if grep -q "^Error:" "$LAUNCH_LOG" 2>/dev/null; then BAD=$(grep -m1 "^Error:" "$LAUNCH_LOG"); break; fi
  D=$(ls -td "$LOG_ROOT"/*-vector-dimos-"$BP"/ 2>/dev/null | head -1)
  if [ -n "$D" ] && [ "$(stat -c %Y "$D")" -ge "$LAUNCH_MARK" ]; then
    RUN_DIR="${D%/}"
    L=$(grep -a -E "$PROOF_RE" "$RUN_DIR/main.jsonl" 2>/dev/null)
    echo "$L" | grep -q -E "MOSS base: connected|MOSS base: MOCK BUS" && OK_BASE=1
    echo "$L" | grep -q "RPLIDAR C1 up" && OK_LIDAR=1
    echo "$L" | grep -q "lidar odometry up" && OK_ODOM=1
    BAD=$(echo "$L" | grep -a -E "Traceback|cannot open|no telemetry" | head -1)
    [ -n "$BAD" ] && break
  fi
  grep -q "$LAUNCH_OK_RE" "$LAUNCH_LOG" 2>/dev/null && OK_LAUNCH=1
  [ "$OK_BASE$OK_LIDAR$OK_ODOM$OK_LAUNCH" = "1111" ] && break
done
if [ -n "$RUN_DIR" ]; then
  grep -a -E "MOSS|RPLIDAR|lidar odom|costmap|Gamepad|Traceback|Error" "$RUN_DIR/main.jsonl" 2>/dev/null | tail -6
fi
if [ -n "$BAD" ] || [ "$OK_BASE$OK_LIDAR$OK_ODOM$OK_LAUNCH" != "1111" ]; then
  WHY=""
  [ "$OK_BASE" = "1" ] || WHY="$WHY base never connected (QT Py port, firmware);"
  [ "$OK_LIDAR" = "1" ] || WHY="$WHY lidar never up (CP2102N port, lidar power);"
  [ "$OK_ODOM" = "1" ] || WHY="$WHY no lidar odometry (kiss_icp);"
  [ "$OK_LAUNCH" = "1" ] || WHY="$WHY the dimOS launcher never said '$LAUNCH_OK_RE';"
  [ -z "$RUN_DIR" ] && WHY="$WHY no new run dir under $LOG_ROOT;"
  [ -n "$BAD" ] && WHY="$WHY saw: $(echo "$BAD" | cut -c1-160)"
  echo "STACK NOT PROVEN in ${PROOF_S} s -$WHY - see $LAUNCH_LOG and tools/moss-fly.sh log; stopping it"
  tail -5 "$LAUNCH_LOG"
  stack_down >/dev/null 2>&1
  exit 1
fi
echo "  proven: base, lidar and lidar odometry up, build done, in $(basename "$RUN_DIR")"

echo "== 3/$N cockpit =="
cockpit_up || {
  echo "NO COCKPIT on :$COCKPIT_PORT (see $COCKPIT_LOG) - a run nobody can watch does not fly: stopping the stack"
  stack_down >/dev/null 2>&1
  exit 1
}
echo "  answering: $(cockpit_url)"

if [ "${EXPLORE:-0}" = "1" ]; then
  echo "== 4/$N exploration =="
  # count before publishing: only a NEW line proves this start was heard
  EXPLORE_RE="costmap: explore_cmd received|Received exploration start command|Started autonomous frontier exploration"
  BEFORE=$(grep -a -c -E "$EXPLORE_RE" "$RUN_DIR/main.jsonl" 2>/dev/null)
  BEFORE=${BEFORE:-0}
  $PY tools/explore_ctl.py start
  OK_EXP=0
  for _ in $(seq 1 "$EXPLORE_PROOF_S"); do
    sleep 1
    NOW=$(grep -a -c -E "$EXPLORE_RE" "$RUN_DIR/main.jsonl" 2>/dev/null)
    [ "${NOW:-0}" -gt "$BEFORE" ] && { OK_EXP=1; break; }
  done
  if [ "$OK_EXP" != "1" ]; then
    echo "EXPLORATION NOT PROVEN (no 'explore_cmd received' in ${EXPLORE_PROOF_S} s on $TRANSPORT) - stopping exploration and the stack"
    $PY tools/explore_ctl.py stop 2>&1 | tail -1
    stack_down >/dev/null 2>&1
    exit 1
  fi
  echo "  exploration armed: the explorer heard explore_cmd"
fi

echo "== READY: vector-dimos.$BP =="
if [ "$BP" = "moss-nav" ]; then
  echo "  pad: hold R1 (deadman), left stick forward/back, right stick turn - the map builds as you drive"
elif [ "${EXPLORE:-0}" != "1" ]; then
  echo "  exploration NOT armed: the cockpit's 'Start exploration', or a goal click on its map"
fi
[ "$MOSS_RECORD" = "0" ] || echo "  recording into $MOSS_STATE_DIR/recordings/"
echo "  cockpit: $(cockpit_url)"
echo "  stop:    tools/moss-fly.sh stop"

#!/usr/bin/env bash
# Arm the speed watchdog for THIS flight (gate 7 of tools/fly.sh, run on the rover).
#
# NEVER "start one only if pgrep finds none": the previous flight's watchdog
# survives its run (dimos stop only sweeps DIMOS_RUN_ID-tagged processes) and its
# mere presence satisfied that guard while it tailed a log that no longer grows -
# "speed watchdog armed" on screen, 0.35 m/s envelope unenforced in the room
# (2026-08-28 audit). The handle is the pid file garde_vitesse.py stamps at
# startup: kill THAT pid, start a fresh watchdog, then prove the pid file names a
# process born AFTER that launch. No proof -> non-zero, and the gate stops the flight.
set -u
cd "$(dirname "$0")/.." || exit 1

PID_FILE="${GARDE_PID_FILE:-/tmp/garde_vitesse.pid}"
GARDE_PY="${GARDE_PY:-tools/garde_vitesse.py}"
PYTHON="${GARDE_PYTHON:-.venv/bin/python}"
LOG="${GARDE_LOG:-/tmp/garde_vitesse.log}"
DEADLINE_S="${GARDE_ARM_DEADLINE_S:-15}"   # the fresh watchdog must show up within this
TICK=$(getconf CLK_TCK)

alive() {   # a zombie still answers kill -0 and still tails nothing: it is not alive
  local rest state
  rest=$(cut -d')' -f2- "/proc/$1/stat" 2>/dev/null) || return 1
  read -r state _ <<<"$rest"
  [ "$state" != "Z" ]
}

is_garde() { tr '\0' ' ' < "/proc/$1/cmdline" 2>/dev/null | grep -q "garde_vitesse"; }

born_s() {   # seconds since boot at which pid $1 was born (/proc/<pid>/stat field 22)
  local rest
  local -a f
  rest=$(cut -d')' -f2- "/proc/$1/stat" 2>/dev/null) || return 1
  read -r -a f <<<"$rest"        # f[0] is the state, so field 22 overall is f[19]
  [ "${#f[@]}" -ge 20 ] || return 1
  echo $(( f[19] / TICK ))
}

# 1. retire the previous flight's watchdog, by the pid IT left behind
OLD=$(cat "$PID_FILE" 2>/dev/null || true)
case "$OLD" in
  '') ;;
  *[!0-9]*) echo "  $PID_FILE holds no pid ('$OLD') - ignored" ;;
  *)
    if ! alive "$OLD"; then
      echo "  previous watchdog (pid $OLD) already gone - stale pid file"
    elif ! is_garde "$OLD"; then
      echo "  pid $OLD is not a garde_vitesse (pid reused) - NOT killed"
    else
      kill "$OLD" 2>/dev/null
      for _ in $(seq 1 50); do alive "$OLD" || break; sleep 0.1; done
      if alive "$OLD"; then
        kill -9 "$OLD" 2>/dev/null
        for _ in $(seq 1 20); do alive "$OLD" || break; sleep 0.1; done
      fi
      if alive "$OLD"; then
        echo "PREVIOUS WATCHDOG (pid $OLD) WILL NOT DIE - not arming over it"
        exit 1
      fi
      echo "  previous watchdog (pid $OLD) retired"
    fi ;;
esac
rm -f "$PID_FILE"      # only the NEW watchdog may satisfy the check below

# 2. launch the fresh one, keeping the mark it must postdate
read -r UP _ < /proc/uptime
LAUNCH_S="${UP%%.*}"                       # seconds since boot, taken BEFORE the launch
nohup "$PYTHON" "$GARDE_PY" > "$LOG" 2>&1 < /dev/null &

# 3. the pid file must now name a live process born after that mark
NEW=""
END=$((SECONDS + DEADLINE_S))
while [ "$SECONDS" -lt "$END" ]; do
  P=$(cat "$PID_FILE" 2>/dev/null || true)
  case "$P" in
    ''|*[!0-9]*) sleep 0.2; continue ;;
  esac
  if alive "$P"; then
    B=$(born_s "$P" || true)
    if [ -n "$B" ] && [ "$B" -ge "$LAUNCH_S" ]; then NEW="$P"; break; fi
  fi
  sleep 0.2
done
if [ -z "$NEW" ] || ! alive "$NEW"; then
  echo "SPEED WATCHDOG DID NOT COME UP: $PID_FILE names no live process born after the launch (${DEADLINE_S} s) - see $LOG"
  tail -3 "$LOG" 2>/dev/null | sed 's/^/  /'
  exit 1
fi
echo "speed watchdog armed (kills exploration beyond 0.35 m/s) - pid $NEW, born after this launch"

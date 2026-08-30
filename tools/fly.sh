#!/usr/bin/env bash
# VECTOR flight: preflight -> stack -> EVERYTHING ON THE OPERATOR'S SCREEN -> explore.
#
# Hard requirement: a run nobody can watch does not launch. THREE displays are
# GATES in the flight sequence, not options - the map (Rerun), the organ panel
# (:8900/panel) and the camera cockpit (7780, Firefox - proven there
# 2026-08-24). If one is dead, the stack is stopped and the flight is refused;
# a failure nobody can see turns into a long blind debugging session, which is
# exactly what this sequence exists to prevent.
#
# Runs on the RIG (the machine with the screen), not on the rover:
#     tools/fly.sh              piloted flight (DEFAULT): everything up, exploration NOT armed
#     EXPLORE=1 tools/fly.sh    autonomous exploration flight (gates 7/8 + 8/8 run)
set -u
ROVER=metrox@192.168.0.56
VIEWER=/home/openclaw/miniconda3/envs/lerobot052/bin/dimos-viewer
RELAY_EXT=45817          # fixed rover-side UDP port relaying to the run's dynamic QUIC port
# The piloted lap is the DEFAULT (2026-08-28 00:00: DRY was forgotten twice at
# midnight, autonomous exploration armed itself into a piloted evening and the
# flight then hung on gate 8/8 - a run lost to a switch. The common case must be
# the default; the autonomous case is the explicit exception.)
# EXPLORE=1 used to override an EXPLICIT DRY=1 without a word: the operator asked
# for the two opposite flights at once and silently got the autonomous one
# (2026-08-28 audit). Contradiction = refusal, before anything is touched.
if [ "${EXPLORE:-0}" = "1" ]; then
  [ "${DRY:-0}" = "1" ] && { echo "CONTRADICTORY FLAGS: EXPLORE=1 (autonomous) with DRY=1 (piloted) - pick one, no flight"; exit 1; }
  DRY=0
else
  DRY="${DRY:-1}"
fi
# The run's bus, resolved ONCE here: EVERY ssh that publishes (explore start and
# stop) or arms the watchdog must carry it. Only the stack launch did, so on a
# zenoh run the rover-side publishes defaulted to lcm - the start, the abort's
# stop and the watchdog's own stop all reached nobody (2026-08-28 audit).
TRANSPORT="${TRANSPORT:-lcm}"
# FLY_TEST=1 replaces every command that reaches the rover, the screen or the
# clock with a recorder, so the gates below can be flown cold on the rig
# (tests/test_fly_gates_cold.py). Nothing but that bench ever sets it.
if [ "${FLY_TEST:-0}" = "1" ]; then . "${FLY_TEST_STUBS:?FLY_TEST=1 needs FLY_TEST_STUBS}"; fi

# A forgotten running stack holds the motor bus and the hardware preflight
# collides with its feedback polling: both drives read "mute" (2026-08-26 19:00,
# one hour lost on a stack launched at 18:22 for map viewing and never stopped).
# THIS REFUSAL COMES FIRST - before one byte is synced, one process killed or one
# log truncated. It used to sit AFTER the cleanup, so a second invocation tore
# down the LIVE flight's panel, sonar readout, cockpit server and both relays -
# and, on the interactive path, e-stopped its wheels - and only THEN refused to
# fly. Flight 1 kept driving, blind, and nothing restored it (2026-08-28 audit).
echo "== 0/7 no dimOS stack already flying =="
# The refusal protects a LIVE flight - a dimos process WITH a registered run.
# A dimos process WITHOUT one is a ZOMBIE (metrox 2026-08-30 "inclus les
# nettoyages au flycheck": a half-born gate-3 daemon killed mid-boot kept the
# motor port and made every e-stop INCOMPLETE; two fly.sh raced for one log).
# Zombies are swept here; a registered run still refuses, exactly as before.
for GHOST in $(pgrep -f "[t]ools/fly.sh"); do
  [ "$GHOST" = "$$" ] || [ "$GHOST" = "$PPID" ] \
    || { echo "ghost fly.sh (pid $GHOST) from an older invocation - killed"; kill "$GHOST" 2>/dev/null; }
done
if ssh $ROVER "pgrep -f \"[b]in/dimos\" >/dev/null"; then
  # Safe by default: ONLY the explicit "No running DimOS instance" answer makes
  # a zombie. A live run, a garbled answer or a dead ssh all REFUSE, untouched
  # (the cold bench kills any version that guesses the other way).
  if ! ssh $ROVER "~/vector-dimos/.venv/bin/dimos status 2>/dev/null" | grep -q "No running DimOS instance"; then
    echo "A DIMOS STACK IS ALREADY RUNNING - stop it first (estop, then dimos stop)"; exit 1
  fi
  echo "zombie dimos process (no registered run) - sweeping: estop, kill, estop"
  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py >/dev/null 2>&1;
    for p in $(pgrep -f "[b]in/dimos"); do kill -9 "$p" 2>/dev/null; done; sleep 2;
    .venv/bin/python tests/estop_rs485.py 2>&1 | tail -1'
  ssh $ROVER "pgrep -f \"[b]in/dimos\" >/dev/null" \
    && { echo "the zombie survived kill -9 - no flight, look at it by hand"; exit 1; }
fi

echo "== SYNC code rig -> rover =="
# The rover has NO git clone - this rsync IS the deployment (found 2026-08-27:
# nothing else ships code, so rig and rover can silently diverge). Gate, not
# goodwill: a flight on stale code is a flight on unknown code.
SYNC_OUT=$(rsync -ai --exclude .venv --exclude .git --exclude __pycache__ --exclude "*.log" \
  ~/vector-dimos/ "${ROVER}":vector-dimos/) \
  || { echo "SYNC FAILED - no flight (le code rig/rover divergerait)"; exit 1; }
if echo "$SYNC_OUT" | grep -q "stats_server.py"; then
  ssh $ROVER 'for p in $(pgrep -f "[s]tats_server"); do kill "$p"; done' 2>/dev/null
  echo "stats_server change: ancien processus tue (la porte 5 relance le neuf)"
fi

echo "== 0/7 rover REPOSITIONNE (torque off) =="
# NEVER launch without the rover physically put back on its start point first -
# between runs it can be anywhere (the 2026-08-27 12:18 run started on a blank
# map 10 cm from the sofa; its first act was bumping it). Interactive: ask and
# wait. Detached: REPOSITIONNE=1 is required, and is only set once a human has
# actually moved the rover.
# Torque OFF before any repositioning: the rover weighs 26-27 kg and cannot be
# pushed by hand while the motors hold position. The release is part of the gate
# itself, never goodwill.
# The release is a RESULT, never an announcement: estop_rs485.py exits 0 only
# when BOTH drives acked rpm 0 AND disable. Its exit code used to be discarded
# (a `| tail -1` pipeline whose status is tail's, stderr to /dev/null), so a busy
# FTDI dongle, a renumbered port or a NAKing drive printed nothing and the script
# still said "se pousse a la main" - a hand-push invited on 26-27 kg whose drives
# were still holding position; a dead ssh printed NOTHING AT ALL and that false
# line was the only thing on screen (2026-08-28 audit).
if [ "${REPOSITIONNE:-0}" != "1" ]; then
  if [ -t 0 ]; then
    if ESTOP_OUT=$(ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py' 2>&1); then
      echo "$ESTOP_OUT" | tail -1
      echo "Couple moteurs RELACHE - le rover se pousse a la main."
      read -r -p "Rover repositionne au point de depart ? [Entree pour confirmer] " _
    else
      [ -n "$ESTOP_OUT" ] && echo "$ESTOP_OUT" | tail -3 | sed 's/^/  /'
      echo "COUPLE MOTEUR NON RELACHE (estop_rs485.py n'a pas rendu 0) - NE PAS pousser le rover"
      echo "  a la main: les drives tiennent peut-etre toujours leur position. Couper"
      echo "  l'alimentation moteur, puis relancer le vol."
      exit 1
    fi
  else
    echo "REFUS: lancement detache sans REPOSITIONNE=1 (le rover doit etre replace d'abord - regle 27/08)"
    exit 1
  fi
fi
ssh $ROVER 'for p in $(pgrep -f "[s]onar_live"); do kill "$p"; done' 2>/dev/null   # the readout UI holds the ESP port
# a STALE deno (cockpit server of a previous run) keeps port 7780 and its QUIC
# socket, and a stale relay keeps $RELAY_EXT: the new run's cockpit is stillborn
ssh $ROVER 'for p in $(pgrep -x deno) $(pgrep -f "[u]dp_forward"); do kill "$p"; done' 2>/dev/null

# The mode flags must cross into the PREFLIGHTS too, in the same form as the
# stack line: a preflight that does not know which flight this is cannot judge
# it. GAMEPAD is the one that bites today - preflight.py turns "manette absente"
# into a refusal only when GAMEPAD=1, and that variable stopped at the rig, so
# the pad gate never fired once and a padless GAMEPAD=1 flight walked all seven
# gates before the operator found nothing could drive (2026-08-28 audit).
MODE_ENV="GAMEPAD=${GAMEPAD:-0} STOCK_NAV=${STOCK_NAV:-0} PERSISTENT_MAP=${PERSISTENT_MAP:-0} EXPLORER_V2=${EXPLORER_V2:-1}"
echo "== 1/7 preflight hardware =="
ssh $ROVER "cd ~/vector-dimos && $MODE_ENV .venv/bin/python tools/preflight.py" || { echo "HARDWARE KO - no flight"; exit 1; }
echo "== 2/7 preflight nav =="
ssh $ROVER "cd ~/vector-dimos && $MODE_ENV .venv/bin/python tools/preflight_nav.py" || { echo "NAV KO - no flight"; exit 1; }
# STOCK_NAV=1 runs dimOS's CostMapper, which has no zone concept at all: the
# zones drawn on the persistent map are NOT applied by this flight, and nothing
# in the run ever says so (2026-08-28 audit). A WARNING, never a refusal - the
# flag is a deliberate A/B - but the operator hears it before the wheels move,
# not after the rover has walked into the bathroom.
if [ "${STOCK_NAV:-0}" = "1" ] && ssh $ROVER 'test -s ~/.local/state/vector/keepout.json' 2>/dev/null; then
  echo "!! STOCK_NAV=1 AND ZONES DRAWN (~/.local/state/vector/keepout.json): stock nav has NO keep-out"
  echo "   zones - the bathroom is NOT forbidden this flight. Pilot it, or fly without STOCK_NAV."
fi

echo "== 3/7 stack =="
LAUNCH_MARK=$(ssh $ROVER 'date +%s')
# the mode flags MUST cross the ssh boundary (2026-08-27 13:38: STOCK_NAV and
# GAMEPAD were set on the rig and never reached the rover - the midday
# "stock" flights were not stock)
# PERSISTENT_MAP=0 is the DEFAULT since 2026-08-28 18:43 (metrox's order: back to
# Sunday's exploration). Every flight now starts on a VIRGIN map: costmap.start()
# logs "PERSISTENT_MAP=0: no saved map, no relocalization, no keep-out zone" and
# lidar_odometry opens a fresh frame with no boot relocalization ("as before this
# existed" - the feature's own off-switch). NOTHING is deleted: keepout.json
# stays on disk and the zones are a DORMANT bonus, re-armed by PERSISTENT_MAP=1
# the day persistence is trusted again. What killed the trust: the 2026-08-27
# 16:40 deadlock (weak-score checkpoint never matchable -> frozen map ->
# reference expired -> no exit). Together with ODOM_GUARDS=0 below, this IS the
# configuration Sunday's exploration flew.
# ODOM_GUARDS=0 is the DEFAULT since 2026-08-28 18:37 (metrox's order after the
# external audit): the scan gate rejects healthy driving under load and freezes
# the map (measured 27/08 16h52, and written in lidar_odometry.py's own
# comment). Fly with ODOM_GUARDS=1 to get the B arm of the A/B back - one
# variable at a time, same course.
# GLIBC_TUNABLES: the in-house open3d-CUDA wheel carries a 41 KB TLS block that
# the default dlopen reserve (~1.6 KB) cannot hold - a 4 MiB reserve gives it
# room (256 KB was not enough inside loaded workers) (measured 2026-08-27 20:00: without it, "cannot allocate memory in static
# TLS block").
ssh $ROVER "cd ~/vector-dimos && GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304 TRANSPORT=$TRANSPORT STOCK_NAV=${STOCK_NAV:-0} GAMEPAD=${GAMEPAD:-0} PERSISTENT_MAP=${PERSISTENT_MAP:-0} ODOM_GUARDS=${ODOM_GUARDS:-0} RECORD_CLOUDS=${RECORD_CLOUDS:-1} RELOC_MAP=${RELOC_MAP:-} EXPLORER_V2=${EXPLORER_V2:-1} ~/vector-dimos/.venv/bin/dimos --rerun-open none --rerun-host 0.0.0.0 --nerf-speed 0.4 run vector-dimos.explore --local-relay --daemon > /tmp/dimos_launch.log 2>&1 < /dev/null"
sleep 12
# the run dir must POSTDATE this launch: on 2026-08-26 a failed launch over a live
# stack passed the lidar check against the PREVIOUS run's log (false IN FLIGHT)
ssh $ROVER "d=\$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); [ \$(stat -c %Y \"\$d\") -ge $LAUNCH_MARK ] && grep -q 'RPLIDAR C1 up' \"\$d/main.jsonl\"" \
  || { echo "no NEW run with a live lidar - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }

echo "== 4/7 the map on the operator's screen (GATE) =="
# A stale viewer shows the PREVIOUS run frozen (observed 2026-08-26 17:50):
# kill it, start a fresh one against THIS run's server, then verify from the
# rover that the connection is ESTABLISHED. A window is not a gate; a live
# TCP connection to this run's rerun port is.
pkill -f dimos-viewer 2>/dev/null
sleep 1
# systemd-run --user: the viewer must NOT inherit the caller's niceness. Born
# from the user manager it runs at nice 0 (2026-08-30: every nohup-launched
# viewer inherited nice +12 from the operator shell and rendered at 118 % of
# one core, 18-37 s behind reality - the map lag). Transient auto-named unit,
# --collect cleans it up; viewer output goes to the user journal.
systemd-run --user --collect --setenv=DISPLAY="${DISPLAY:-:1}" "$VIEWER" "rerun+http://192.168.0.56:9877/proxy"
sleep 8
# Nobody should have to grab, move and resize the map window on every run: it
# opens small and on the wrong screen. Park it fullscreen on the LEFT monitor
# (DP-1, 0,0) - the panel lives on the right.
# BOUNDED (2026-08-27 11:43: --sync waited forever on a title mismatch and froze
# the whole sequence at gate 4 - the exact mute hang this file exists to kill)
WID=$(DISPLAY="${DISPLAY:-:1}" timeout 10 xdotool search --onlyvisible --name "[Rr]erun" 2>/dev/null | head -1)
[ -n "$WID" ] && DISPLAY="${DISPLAY:-:1}" xdotool windowmove "$WID" 0 0 windowsize "$WID" 3840 2160 2>/dev/null
[ -n "$WID" ] && echo "map window parked fullscreen on the left monitor" || echo "(map window not found by name - place it by hand this once)" 
ssh $ROVER "ss -tn state established \"( sport = :9877 )\" | grep -q ." \
  || { echo "NO LIVE MAP CONNECTION - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
echo "viewer CONNECTED to this run - the map is live"

echo "== 5/7 the organ panel on the operator's screen (GATE) =="
# stats_server: passive LCM listener + battery meter, port 8900 on the LAN.
# A running stats_server OLDER than its file is stale code lying with a live
# port (bitten 2026-08-27 16:25: the old process 404'd ?watcher= and the vigil went
# blind). Compare process start vs file mtime; restart when stale.
ssh $ROVER 'F=~/vector-dimos/tools/stats_server.py
P=$(pgrep -f "[s]tats_server" | head -1)
if [ -n "$P" ] && [ "$(stat -c %Y "$F")" -gt "$(stat -c %Y "/proc/$P")" ]; then kill "$P"; sleep 1; P=""; fi
[ -n "$P" ] || (cd ~/vector-dimos && GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304 nohup ./.venv/bin/python tools/stats_server.py >> /tmp/stats_server.log 2>&1 & sleep 6)'
curl -sf -m 5 "http://192.168.0.56:8900/metrics" | grep -q '"sensors"' \
  || { echo "NO ORGAN PANEL - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
# Every monitor the supervising agent runs joins the flight check and the panel
# ("Software > Monitoring"). That vigil (tools/vigie_iris.py, on the rig) polls
# /metrics?watcher=iris; loud warning if nobody is watching.
curl -sf -m 5 "http://192.168.0.56:8900/metrics" | python3 -c '
import json, sys
m = json.load(sys.stdin).get("sensors", {}).get("software", {}).get("monitoring", {})
sys.exit(0 if m.get("alive") else 1)' 2>/dev/null \
  || echo "!! VIGIE IRIS ABSENTE (Software > Monitoring rouge) - lancer tools/vigie_iris.py"
# NO tab is ever opened by the flight: one new tab per run piles up a dozen
# stale tabs in a day. The operator keeps two pinned tabs - Control Panel and
# cockpit - and opens the zones UI only when editing zones.
echo "organ block answering (your pinned Control Panel tab shows it)"

echo "== 6/7 the camera cockpit on the operator's screen (GATE) =="
# The cockpit page (deno, 7780) and its video (WebTransport over QUIC/UDP) both
# live on the rover's LOOPBACK only. Page: SSH tunnel. Video: udp_forward on
# BOTH sides - rover 0.0.0.0:$RELAY_EXT -> 127.0.0.1:<run's QUIC port>, rig
# 127.0.0.1:<same QUIC port> -> rover:$RELAY_EXT. The QUIC port changes every
# run (wt_url in main.jsonl). 2026-08-26 midday the relay was bound on the WRONG
# side (rover, on deno's own port -> Address already in use): page up, video
# never came. Firefox is fine (proven 2026-08-24) - the browser was never the bug.
# every network command is bounded: the first dry flight (2026-08-26 20:51) hung
# HERE in silence past the 3-minute timeout - a flight sequence may fail loud,
# never hang mute.
WT_PORT=$(timeout 15 ssh $ROVER "d=\$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); grep -oE \"wt_url='https://127.0.0.1:[0-9]+\" \"\$d/main.jsonl\" | tail -1 | grep -oE '[0-9]+\$'")
[ -n "$WT_PORT" ] || { echo "NO COCKPIT (no wt_url in this run's log) - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
timeout 15 ssh $ROVER "cd ~/vector-dimos && nohup .venv/bin/python tools/udp_forward.py $RELAY_EXT 127.0.0.1 $WT_PORT 0.0.0.0 > /tmp/udp_forward_rover.log 2>&1 < /dev/null &"
pkill -f "[u]dp_forward" 2>/dev/null; sleep 1
nohup python3 "$(dirname "$0")/udp_forward.py" "$WT_PORT" 192.168.0.56 "$RELAY_EXT" 127.0.0.1 > /tmp/udp_forward_rig.log 2>&1 &
# ONE tunnel for page + deck: 7780 (cockpit) and 8900 (/vol + panel). The
# /vol page MUST be browsed as 127.0.0.1: WebTransport in the cockpit iframe
# needs a secure context, and every ancestor frame must be localhost - the
# LAN address gives "Not a secure context" (observed 2026-08-26 21:55).
for p in $(ss -tlnp 2>/dev/null | grep -oE "127.0.0.1:7780.*pid=[0-9]+" | grep -oE "pid=[0-9]+" | cut -d= -f2 | sort -u); do kill "$p"; done 2>/dev/null
timeout 15 ssh -fN -L 7780:127.0.0.1:7780 -L 8900:127.0.0.1:8900 $ROVER
# the deno cockpit server sometimes takes longer than one probe (fly25 and
# fly28 were refused on a single 2 s-late curl). Widened to 30 x 3 s
# (2026-08-28): cold after a reboot it needs well over 30 s - fly50 was
# refused for being exactly 30 s patient with the one slow thing.
COCKPIT_OK=0
for _ in $(seq 1 30); do
  curl -sf -m 5 -o /dev/null "http://127.0.0.1:7780/" && { COCKPIT_OK=1; break; }
  sleep 3
done
[ "$COCKPIT_OK" = "1" ] \
  || { echo "NO COCKPIT PAGE (90 s de retries) - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
echo "cockpit relayed on QUIC port $WT_PORT - RELOAD your pinned cockpit tab (address never changes)"

if [ "$DRY" = "1" ]; then
  echo "== 7/7 DRY: exploration NOT started =="
  echo "Everything is up and displayed. Stop cleanly (wheels are still, but always):"
  echo "  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'"
  exit 0
fi

echo "== 7/8 exploration =="
ssh $ROVER "cd ~/vector-dimos && TRANSPORT=$TRANSPORT .venv/bin/python tools/explore_ctl.py start"
# The speed watchdog flies with every flight (2026-08-26 it was armed by hand).
# NOT "start one if pgrep finds none": the previous flight's watchdog survives its
# run and satisfied that guard while tailing a dead log - armed on screen,
# unguarded in the room (2026-08-28 audit). The arm script kills the pid the pid
# file names, starts a fresh watchdog and proves the pid file now names a process
# born after that launch. No proof = exploration running unguarded = no flight.
ssh $ROVER "cd ~/vector-dimos && TRANSPORT=$TRANSPORT bash tools/arm_garde_vitesse.sh" || {
  echo "SPEED WATCHDOG NOT ARMED - exploration would run unguarded, stopping the flight"
  ssh $ROVER "cd ~/vector-dimos && TRANSPORT=$TRANSPORT .venv/bin/python tools/explore_ctl.py stop"
  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py; .venv/bin/dimos stop; sleep 2; .venv/bin/python tests/estop_rs485.py'
  exit 1
}

echo "== 8/8 the rover must understand WHERE IT IS (GATE) =="
# Requirement: as soon as the rover maps, it must understand where it is and
# apply its limits - every run. Relocalizing into the persistent frame is what
# brings BOTH the grid alignment and the keep-out zones (bathroom!). The
# 2026-08-26 21:05 run explored 10 min in its own frame with the zones INACTIVE
# - never again: no persistent frame within the grace period -> the flight stops
# itself.
# ALLOW_FRESH=1 skips this gate (bootstrapping the very first map only).
# The four lines below are printed by VectorCostMap continuing a persistent map,
# and by nothing else: PERSISTENT_MAP=0 disables relocalization by construction
# and STOCK_NAV=1 swaps VectorCostMap for dimOS's CostMapper, so in those two
# modes the gate polled 11.5 min for a state that cannot exist and then killed a
# run doing exactly what the flag asked (2026-08-28 audit). There it is a WARNING;
# in persistent custom mode - the mode that owes us the zones - it still stops
# the flight.
if [ "${ALLOW_FRESH:-0}" = "1" ]; then
  echo "ALLOW_FRESH=1: fresh-frame flight allowed (first-map bootstrap) - NO keep-out zones"
elif [ "${PERSISTENT_MAP:-0}" = "0" ] || [ "${STOCK_NAV:-0}" = "1" ]; then
  echo "!! GATE 8/8 NOT ENFORCED (PERSISTENT_MAP=${PERSISTENT_MAP:-0} STOCK_NAV=${STOCK_NAV:-0}): this mode never relocalizes,"
  echo "   so the flight runs in its OWN frame - NO keep-out zones (bathroom!), no grid alignment. Watch it."
else
  RELOC_OK=""
  for i in $(seq 1 46); do    # ~11.5 min: 10 min of grace + margin
    sleep 15
    V=$(timeout 15 ssh $ROVER "d=\$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); grep -oE 'CONTINUING the persistent map|relocalized late - dropping|resumed in the persistent frame|relocalization: gave up' \"\$d/main.jsonl\" | tail -1")
    case "$V" in
      "CONTINUING the persistent map"|"relocalized late - dropping"|"resumed in the persistent frame")
        RELOC_OK=1; echo "relocalized (${i}x15 s) - grid aligned, keep-out zones ACTIVE"
        timeout 15 ssh $ROVER "d=\$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); grep -oE 'prior: gyro=[A-Za-z]+[^\"]{0,30}' \"\$d/main.jsonl\" | tail -1" | sed 's/^/  rotation /'
        break ;;
      "relocalization: gave up")
        break ;;
      *) [ $((i % 4)) -eq 0 ] && echo "  still finding itself... ($((i * 15)) s)" ;;
    esac
  done
  if [ -z "$RELOC_OK" ]; then
    echo "THE ROVER NEVER UNDERSTOOD WHERE IT IS - zones inactive, stopping the flight"
    ssh $ROVER "cd ~/vector-dimos && TRANSPORT=$TRANSPORT .venv/bin/python tools/explore_ctl.py stop"
    ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py; .venv/bin/dimos stop; sleep 2; .venv/bin/python tests/estop_rs485.py'
    exit 1
  fi
fi

echo "IN FLIGHT. Stop: E-STOP FIRST, then the stack:"
echo "  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'"
echo "  (dimos stop can escalate to SIGKILL: killed asymmetrically, one axle keeps its last"
echo "   command for up to 1 s - the 26/08 17h50 quarter-turn. Wheels dead first, always.)"

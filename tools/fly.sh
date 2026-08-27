#!/usr/bin/env bash
# VECTOR flight: preflight -> stack -> EVERYTHING ON THE OWNER'S SCREEN -> explore.
#
# Owner's rule (26/08, asked three days running, hardened 26/08 evening): a run
# he cannot watch does not launch. THREE displays are GATES in the flight
# sequence, not options - the map (Rerun), the organ panel (:8900/panel) and
# the camera cockpit (7780, Firefox - proven there 24/08). If one is dead, the
# stack is stopped and the flight is refused: "si ca merde de nouveau et que je
# vois pas d'ou ca vient, ca va de nouveau etre le drame".
#
# Runs on the RIG (the machine with the screen), not on the rover:
#     tools/fly.sh              piloted flight (DEFAULT): everything up, exploration NOT armed
#     EXPLORE=1 tools/fly.sh    autonomous exploration flight (gates 7/8 + 8/8 run)
set -u
ROVER=metrox@192.168.0.56
VIEWER=/home/openclaw/miniconda3/envs/lerobot052/bin/dimos-viewer
RELAY_EXT=45817          # fixed rover-side UDP port relaying to the run's dynamic QUIC port
# The piloted lap is the DEFAULT (28/08 00h: DRY was forgotten twice at midnight,
# autonomous exploration armed itself into a piloted evening and the flight then
# hung on gate 8/8 - the owner lost his run to a switch. The common case must be
# the default; the autonomous case is the explicit exception.)
if [ "${EXPLORE:-0}" = "1" ]; then DRY=0; else DRY="${DRY:-1}"; fi

echo "== SYNC code rig -> rover =="
# The rover has NO git clone - this rsync IS the deployment (found 27/08 16h:
# nothing else ships code, so rig and rover can silently diverge). Gate, not
# goodwill: a flight on stale code is a flight on unknown code.
SYNC_OUT=$(rsync -ai --exclude .venv --exclude .git --exclude __pycache__ --exclude "*.log" \
  ~/vector-dimos/ "${ROVER}":vector-dimos/) \
  || { echo "SYNC FAILED - no flight (le code rig/rover divergerait)"; exit 1; }
if echo "$SYNC_OUT" | grep -q "stats_server.py"; then
  ssh $ROVER 'for p in $(pgrep -f "[s]tats_server"); do kill "$p"; done' 2>/dev/null
  echo "stats_server change: ancien processus tue (la porte 5 relance le neuf)"
fi

echo "== 0/7 rover REPOSITIONNE + no stack already flying =="
# Owner's rule (27/08 12h20): NEVER launch without the rover repositioned by
# his hands first - "il peut etre n'importe ou" (the 12h18 run started on a
# blank map 10 cm from the sofa; its first act was bumping it). Interactive:
# ask and wait. Detached (Iris): REPOSITIONNE=1 required, given only after
# his word.
# Torque OFF before any repositioning (owner, 27/08 12h25: "il faut toujours
# enlever la torque des moteurs, sinon je dois le trainer sur le sol - il
# fait 26-27 kg"). The release is part of the gate itself, never goodwill.
if [ "${REPOSITIONNE:-0}" != "1" ]; then
  if [ -t 0 ]; then
    ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py' 2>/dev/null | tail -1
    echo "Couple moteurs RELACHE - le rover se pousse a la main."
    read -r -p "Rover repositionne au point de depart ? [Entree pour confirmer] " _
  else
    echo "REFUS: lancement detache sans REPOSITIONNE=1 (le rover doit etre replace d'abord - regle 27/08)"
    exit 1
  fi
fi
ssh $ROVER 'for p in $(pgrep -f "[s]onar_live"); do kill "$p"; done' 2>/dev/null   # the readout UI holds the ESP port
# a STALE deno (cockpit server of a previous run) keeps port 7780 and its QUIC
# socket, and a stale relay keeps $RELAY_EXT: the new run's cockpit is stillborn
ssh $ROVER 'for p in $(pgrep -x deno) $(pgrep -f "[u]dp_forward"); do kill "$p"; done' 2>/dev/null

# A forgotten running stack holds the motor bus and the hardware preflight
# collides with its feedback polling: both drives read "mute" (26/08 19h00,
# one hour lost on a stack launched at 18h22 for map viewing and never stopped).
ssh $ROVER "pgrep -f \"[b]in/dimos\" >/dev/null" \
  && { echo "A DIMOS STACK IS ALREADY RUNNING - stop it first (estop, then dimos stop)"; exit 1; }

echo "== 1/7 preflight hardware =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/preflight.py' || { echo "HARDWARE KO - no flight"; exit 1; }
echo "== 2/7 preflight nav =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/preflight_nav.py' || { echo "NAV KO - no flight"; exit 1; }

echo "== 3/7 stack =="
LAUNCH_MARK=$(ssh $ROVER 'date +%s')
# the mode flags MUST cross the ssh boundary (27/08 13h38: STOCK_NAV and
# GAMEPAD were set on the rig and never reached the rover - the midday
# "stock" flights were not stock)
# PERSISTENT_MAP=0 = Sunday's world: no relocalization, no freeze, fresh frame
# (the feature's own off-switch - its doc says "exactly as before this
# existed"). Default stays 1; 0 is the teleop-lap escape after the 27/08 16h40
# deadlock (weak-score checkpoint never matchable -> frozen map -> reference
# expired -> no exit).
# GLIBC_TUNABLES: la roue open3d-CUDA maison porte un TLS de 41 Ko que la
# reserve dlopen par defaut (~1,6 Ko) ne loge pas - 256 Ko la met au large
# (mesure 27/08 20h: sans = "cannot allocate memory in static TLS block").
ssh $ROVER "cd ~/vector-dimos && GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304 TRANSPORT=${TRANSPORT:-lcm} STOCK_NAV=${STOCK_NAV:-0} GAMEPAD=${GAMEPAD:-0} PERSISTENT_MAP=${PERSISTENT_MAP:-1} ODOM_GUARDS=${ODOM_GUARDS:-1} RECORD_CLOUDS=${RECORD_CLOUDS:-1} RELOC_MAP=${RELOC_MAP:-} ~/vector-dimos/.venv/bin/dimos --rerun-open none --rerun-host 0.0.0.0 --nerf-speed 0.4 run vector-dimos.explore --local-relay --daemon > /tmp/dimos_launch.log 2>&1 < /dev/null"
sleep 12
# the run dir must POSTDATE this launch: on 26/08 a failed launch over a live
# stack passed the lidar check against the PREVIOUS run's log (false IN FLIGHT)
ssh $ROVER "d=\$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); [ \$(stat -c %Y \"\$d\") -ge $LAUNCH_MARK ] && grep -q 'RPLIDAR C1 up' \"\$d/main.jsonl\"" \
  || { echo "no NEW run with a live lidar - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }

echo "== 4/7 the map on the owner's screen (GATE) =="
# A stale viewer shows the PREVIOUS run frozen (owner caught it, 26/08 17h50):
# kill it, start a fresh one against THIS run's server, then verify from the
# rover that the connection is ESTABLISHED. A window is not a gate; a live
# TCP connection to this run's rerun port is.
pkill -f dimos-viewer 2>/dev/null
sleep 1
DISPLAY="${DISPLAY:-:1}" nohup "$VIEWER" "rerun+http://192.168.0.56:9877/proxy" >/tmp/dimos_viewer.log 2>&1 &
sleep 8
# The owner should never have to grab, move and resize the map window (27/08:
# "une petite fenetre... a chaque run la mettre sur l'autre ecran"). Park it
# fullscreen on the LEFT monitor (DP-1, 0,0) - the panel lives on the right.
# BOUNDED (11h43: --sync waited forever on a title mismatch and froze the
# whole sequence at gate 4 - the exact mute-hang sin this file exists to kill)
WID=$(DISPLAY="${DISPLAY:-:1}" timeout 10 xdotool search --onlyvisible --name "[Rr]erun" 2>/dev/null | head -1)
[ -n "$WID" ] && DISPLAY="${DISPLAY:-:1}" xdotool windowmove "$WID" 0 0 windowsize "$WID" 3840 2160 2>/dev/null
[ -n "$WID" ] && echo "map window parked fullscreen on the left monitor" || echo "(map window not found by name - place it by hand this once)" 
ssh $ROVER "ss -tn state established \"( sport = :9877 )\" | grep -q ." \
  || { echo "NO LIVE MAP CONNECTION - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
echo "viewer CONNECTED to this run - the map is live"

echo "== 5/7 the organ panel on the owner's screen (GATE) =="
# stats_server: passive LCM listener + battery meter, port 8900 on the LAN.
# A running stats_server OLDER than its file is stale code lying with a live
# port (bitten 27/08 16h25: the old process 404'd ?watcher= and the vigil went
# blind). Compare process start vs file mtime; restart when stale.
ssh $ROVER 'F=~/vector-dimos/tools/stats_server.py
P=$(pgrep -f "[s]tats_server" | head -1)
if [ -n "$P" ] && [ "$(stat -c %Y "$F")" -gt "$(stat -c %Y "/proc/$P")" ]; then kill "$P"; sleep 1; P=""; fi
[ -n "$P" ] || (cd ~/vector-dimos && GLIBC_TUNABLES=glibc.rtld.optional_static_tls=4194304 nohup ./.venv/bin/python tools/stats_server.py >> /tmp/stats_server.log 2>&1 & sleep 6)'
curl -sf -m 5 "http://192.168.0.56:8900/metrics" | grep -q '"sensors"' \
  || { echo "NO ORGAN PANEL - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
# Owner's rule (27/08 15h57): every monitor Iris runs joins the flight check and
# the panel ("Software > Monitoring"). Her vigil (tools/vigie_iris.py, on the
# rig) polls /metrics?watcher=iris; loud warning if nobody is watching.
curl -sf -m 5 "http://192.168.0.56:8900/metrics" | python3 -c '
import json, sys
m = json.load(sys.stdin).get("sensors", {}).get("software", {}).get("monitoring", {})
sys.exit(0 if m.get("alive") else 1)' 2>/dev/null \
  || echo "!! VIGIE IRIS ABSENTE (Software > Monitoring rouge) - lancer tools/vigie_iris.py"
# NO tab is ever opened by the flight (owner 27/08 13h20: "a chaque fois tu
# m'ouvres des nouveaux onglets, j'en avais 15"). He keeps two pinned tabs -
# Control Panel and cockpit - and the zones UI only when he edits zones.
echo "organ block answering (your pinned Control Panel tab shows it)"

echo "== 6/7 the camera cockpit on the owner's screen (GATE) =="
# The cockpit page (deno, 7780) and its video (WebTransport over QUIC/UDP) both
# live on the rover's LOOPBACK only. Page: SSH tunnel. Video: udp_forward on
# BOTH sides - rover 0.0.0.0:$RELAY_EXT -> 127.0.0.1:<run's QUIC port>, rig
# 127.0.0.1:<same QUIC port> -> rover:$RELAY_EXT. The QUIC port changes every
# run (wt_url in main.jsonl). 26/08 midday the relay was bound on the WRONG
# side (rover, on deno's own port -> Address already in use): page up, video
# never came. Firefox is fine (proven 24/08) - the browser was never the bug.
# every network command is bounded: the first dry flight (26/08 20h51) hung
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
# LAN address gives "Not a secure context" (owner caught it, 26/08 21h55).
for p in $(ss -tlnp 2>/dev/null | grep -oE "127.0.0.1:7780.*pid=[0-9]+" | grep -oE "pid=[0-9]+" | cut -d= -f2 | sort -u); do kill "$p"; done 2>/dev/null
timeout 15 ssh -fN -L 7780:127.0.0.1:7780 -L 8900:127.0.0.1:8900 $ROVER
# the deno cockpit server sometimes takes longer than one probe (fly25 and
# fly28 were refused on a single 2 s-late curl): bounded retry, 10 x 3 s.
COCKPIT_OK=0
for _ in 1 2 3 4 5 6 7 8 9 10; do
  curl -sf -m 5 -o /dev/null "http://127.0.0.1:7780/" && { COCKPIT_OK=1; break; }
  sleep 3
done
[ "$COCKPIT_OK" = "1" ] \
  || { echo "NO COCKPIT PAGE (30 s de retries) - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
echo "cockpit relayed on QUIC port $WT_PORT - RELOAD your pinned cockpit tab (address never changes)"

if [ "$DRY" = "1" ]; then
  echo "== 7/7 DRY: exploration NOT started =="
  echo "Everything is up and displayed. Stop cleanly (wheels are still, but always):"
  echo "  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'"
  exit 0
fi

echo "== 7/8 exploration =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/explore_ctl.py start'
# the speed watchdog flies with every flight (26/08 it was armed by hand)
ssh $ROVER 'pgrep -f "[g]arde_vitesse" >/dev/null || (cd ~/vector-dimos && nohup .venv/bin/python tools/garde_vitesse.py > /tmp/garde_vitesse.log 2>&1 < /dev/null &)'
echo "speed watchdog armed (kills exploration beyond 0.35 m/s)"

echo "== 8/8 the rover must understand WHERE IT IS (GATE) =="
# Owner (26/08 21h49): as soon as it maps, it must understand where it is and
# lay its limits down - every run. Relocalizing into the persistent frame is
# what brings BOTH the grid alignment and the keep-out zones (bathroom!). The
# 21h05 run explored 10 min in its own frame with the zones INACTIVE - never
# again: no persistent frame within the grace -> the flight stops itself.
# ALLOW_FRESH=1 skips this gate (bootstrapping the very first map only).
if [ "${ALLOW_FRESH:-0}" = "1" ]; then
  echo "ALLOW_FRESH=1: fresh-frame flight allowed (first-map bootstrap) - NO keep-out zones"
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
    ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/explore_ctl.py stop'
    ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py; .venv/bin/dimos stop; sleep 2; .venv/bin/python tests/estop_rs485.py'
    exit 1
  fi
fi

echo "IN FLIGHT. Stop: E-STOP FIRST, then the stack:"
echo "  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'"
echo "  (dimos stop can escalate to SIGKILL: killed asymmetrically, one axle keeps its last"
echo "   command for up to 1 s - the 26/08 17h50 quarter-turn. Wheels dead first, always.)"

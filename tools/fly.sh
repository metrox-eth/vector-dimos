#!/usr/bin/env bash
# VECTOR flight: preflight -> stack -> THE MAP ON THE OWNER'S SCREEN -> explore.
#
# Owner's rule (26/08, asked three days running): a run he cannot watch does not
# launch. The viewer is a GATE in the flight sequence, not an option - if the
# map is not on his screen, the stack is stopped and the flight is refused.
#
# Runs on the RIG (the machine with the screen), not on the rover:
#     tools/fly.sh
set -u
ROVER=metrox@192.168.0.56
VIEWER=/home/openclaw/miniconda3/envs/lerobot052/bin/dimos-viewer

echo "== 1/5 preflight hardware =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/preflight.py' || { echo "HARDWARE KO - no flight"; exit 1; }
echo "== 2/5 preflight nav =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/preflight_nav.py' || { echo "NAV KO - no flight"; exit 1; }

echo "== 3/5 stack =="
ssh $ROVER 'cd ~/vector-dimos && ~/vector-dimos/.venv/bin/dimos --rerun-open none --rerun-host 0.0.0.0 --nerf-speed 0.4 run vector-dimos.explore --local-relay --daemon > /tmp/dimos_launch.log 2>&1 < /dev/null'
sleep 12
ssh $ROVER 'd=$(ls -td ~/.local/state/dimos/logs/*-vector-dimos-explore/ | head -1); grep -q "RPLIDAR C1 up" "$d/main.jsonl"' \
  || { echo "lidar missing from the run - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }

echo "== 4/5 the map on the owner's screen (GATE) =="
# A stale viewer shows the PREVIOUS run frozen (owner caught it, 26/08 17h50):
# kill it, start a fresh one against THIS run's server, then verify from the
# rover that the connection is ESTABLISHED. A window is not a gate; a live
# TCP connection to this run's rerun port is.
pkill -f dimos-viewer 2>/dev/null
sleep 1
DISPLAY="${DISPLAY:-:1}" nohup "$VIEWER" "rerun+http://192.168.0.56:9877/proxy" >/tmp/dimos_viewer.log 2>&1 &
sleep 8
ssh $ROVER "ss -tn state established \"( sport = :9877 )\" | grep -q ." \
  || { echo "NO LIVE MAP CONNECTION - no flight"; ssh $ROVER '~/vector-dimos/.venv/bin/dimos stop'; exit 1; }
echo "viewer CONNECTED to this run - the map is live"

echo "== 5/5 exploration =="
ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tools/explore_ctl.py start'
echo "IN FLIGHT. Stop: E-STOP FIRST, then the stack:"
echo "  ssh $ROVER 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'"
echo "  (dimos stop can escalate to SIGKILL: killed asymmetrically, one axle keeps its last"
echo "   command for up to 1 s - the 26/08 17h50 quarter-turn. Wheels dead first, always.)"

#!/bin/bash
# Guarded exploration, v2: camera bands + lidar sectors, blind = blocked, stuck detector (wheels vs lidar pose).
# usage: explore.sh N TAGPREFIX   (env: XMIN XMAX YMIN YMAX = virtual walls in the lidar-odometry frame)
cd ~/vector-dimos && source .venv/bin/activate
N=${1:-4}; P=${2:-step}
LOG=$(ls -td ~/.local/state/dimos/logs/*vector-dimos-nav* | head -1)
pose() { grep -o "lidar odom #[0-9]*: x=[^(]*" "$LOG/main.jsonl" | tail -1 | grep -oE "[-+]?[0-9]+\.[0-9]+" | head -3 | tr "\n" " "; }
for i in $(seq 1 $N); do
  G=$(python ~/mars/sense.py ${P}_$i 3 2>&1 | grep "^guard")
  [ -z "$G" ] && { echo "step $i: no guard line -> STOP"; break; }
  P0=$(pose)
  ACT=$(python3 - "$G" "$P0" <<PY
import sys, re, math, os
g = dict(re.findall(r"(ahead|sides|F|L|R|C)=([0-9.]+)", sys.argv[1]))
ahead, sides = float(g["ahead"]), float(g["sides"])
lidL, lidR = float(sys.argv[1].split("lidar")[1].split("L=")[1].split()[0]), float(sys.argv[1].split("lidar")[1].split("R=")[1].split()[0])
m = [float(v) for v in sys.argv[2].split()]; x, y, yaw = (m[0], m[1], math.radians(m[2])) if len(m) >= 3 else (0.0, 0.0, 0.0)
xmin, xmax, ymin, ymax = (float(os.environ.get(k, d)) for k, d in (("XMIN", "-99"), ("XMAX", "99"), ("YMIN", "-99"), ("YMAX", "99")))
ax, ay = x + 0.5 * math.cos(yaw), y + 0.5 * math.sin(yaw)
if not (xmin <= ax <= xmax and ymin <= ay <= ymax): print("turnL" if lidL >= lidR else "turnR")
elif ahead >= 1.0 and sides >= 0.40: print("fwd30")
elif ahead >= 0.60 and sides >= 0.30: print("fwd15")
elif lidL >= lidR: print("turnL")
else: print("turnR")
PY
)
  case $ACT in
    fwd30) python tests/publish_twist.py --vx 0.15 --vy 0 --wz 0 --duration 2.0 > /dev/null; EXP=0.30;;
    fwd15) python tests/publish_twist.py --vx 0.10 --vy 0 --wz 0 --duration 1.5 > /dev/null; EXP=0.15;;
    turnR) python tests/publish_twist.py --vx 0 --vy 0 --wz -0.3 --duration 1.8 > /dev/null; EXP=0;;
    turnL) python tests/publish_twist.py --vx 0 --vy 0 --wz 0.3 --duration 1.8 > /dev/null; EXP=0;;
  esac
  sleep 1.2
  P1=$(pose)
  MOVED=$(python3 -c "
import sys; a=[float(v) for v in '$P0'.split()]; b=[float(v) for v in '$P1'.split()]
print(((b[0]-a[0])**2+(b[1]-a[1])**2)**0.5 if len(a)>=2 and len(b)>=2 else 0.0)")
  echo "step $i: $G -> $ACT | lidar moved ${MOVED:0:4} m | pose $P1"
  if python3 -c "import sys; sys.exit(0 if $EXP >= 0.15 and $MOVED < 0.4*$EXP else 1)"; then
    echo "   STUCK: commanded $EXP m, lidar saw $MOVED m -> back off 10 cm and STOP"
    python tests/publish_twist.py --vx -0.1 --vy 0 --wz 0 --duration 1.0 > /dev/null; break
  fi
done

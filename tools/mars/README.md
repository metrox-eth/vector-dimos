# Mars tools — the step-by-step exploration loop (runs on the rover, `~/mars/`)

- `sense.py TAG [wait]` — the guard: photo + full-height 3D obstacle check from the RealSense depth
  (camera 0.80 m, level; floor removed by geometry) + RPLIDAR front/side sectors, read from dimOS's
  own streams. Prints one `guard …` line. Too few valid depth pixels = blind = blocked.
- `explore.sh N TAG` — N guarded steps: forward 30/15 cm or turn toward the freer lidar side; virtual
  walls via `XMIN/XMAX/YMIN/YMAX` (glass is invisible to depth and lidar); stuck detector (wheels
  commanded but the lidar pose did not move -> back off, stop).
- `step2.sh TAG VX VY WZ DUR` — one twist, then the guard. `look.py` — colour photo only.
- `udp_forward.py PORT HOST RPORT [BIND]` — UDP relay used to reach the on-robot cockpit relay (QUIC)
  from a workstation browser through an ssh tunnel.

Lessons that shaped them (2026-08-22/23): a heavy blueprint delayed twists 2x (fixed in the package);
the camera misses anything under ~0.45 m within 0.6 m (mast geometry) -> lidar in the guard, bumpers
next; glass is invisible to both sensors; thin black table legs are invisible to the lidar.

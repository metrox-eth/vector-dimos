# MOSS cockpit: flight check, mapping, exploration, recording, one page

MOSS is a small tracked rover (two geared DC motors behind a QT Py ESP32-S3
motor board, an RPLidar C1, a RealSense D455F on the nose, a UVC camera on the
arm). This document is the operator's manual for running it on dimOS with the
VECTOR stack: the same lidar odometry, 2D costmap, planner and frontier
explorer that fly VECTOR, under a tracked base, with a flight check and a web
cockpit in front of it.

Everything below runs ON THE ROBOT (the Jetson in MOSS). The cockpit is a web
page served by the robot; open it from any machine on the same network.

## The four pieces

| Piece | File | Runs where |
|---|---|---|
| Flight check | `tools/moss_preflight.py` | Jetson, read-only, ten seconds |
| Blueprints | `vector_dimos/moss_blueprints.py` → `vector-dimos.moss-nav`, `vector-dimos.moss-explore` | Jetson, inside `dimos run` |
| Launcher | `tools/moss-fly.sh` | Jetson, gates then stack |
| Cockpit | `tools/moss_cockpit.py` + `tools/moss_cockpit/` | Jetson, port 8904 |

The pad-only launcher from the first drive, `tools/moss-teleop.sh`, still
exists and still works: it is the smallest thing that moves the robot.

## Run it

```
$ tools/moss-fly.sh check              # flight check only, nothing launched
$ tools/moss-fly.sh map                # gates -> moss-nav: lidar map + pad teleop, recording
$ tools/moss-fly.sh explore            # gates -> moss-explore: map + navigation + explorer, NOT armed
$ EXPLORE=1 tools/moss-fly.sh explore  # ... and arm exploration once the stack is proven
$ tools/moss-fly.sh stop               # stop exploration, then the stack
$ tools/moss-fly.sh log                # what the base, the lidar, the odometry and the map said
$ tools/moss-fly.sh cockpit            # start the cockpit server if it is not up, print the URL
$ MOCK=1 tools/moss-fly.sh map         # fake motor board, no motor; the lidar must still be plugged (mapping needs it)
```

Every gate refuses rather than guesses, with the fix on the same line: a stack
already up, a missing board, a port held by a bench, a mute firmware, a battery
at the BMS floor, a lidar that does not spin, a stack that does not prove itself
in the log within a minute.

The map opens when the rover has moved 5 cm from its boot pose (the costmap's
mission gate, a VECTOR rule) or when an `explore_cmd` arrives. On MOSS a stop (the
cockpit's STOP, a goal, the end of exploration) leaves map writing open:
`freeze_map_on_stop` is False in the MOSS profile, while VECTOR keeps its
end-of-mission freeze. A new launch starts a fresh map.

Nothing moves by itself. `moss-nav` has no planner: the pad drives, the map
builds. `moss-explore` adds the planner, the movement manager and the frontier
explorer, but the explorer only starts on an explicit `explore_cmd` (the
launcher with `EXPLORE=1`, the cockpit's button, or `tools/explore_ctl.py
start`), and the planner only on a goal (a click on the cockpit map, or
`goal_request` on the bus). Any pad input cancels navigation (dimOS
MovementManager), and the two deadmen of the first drive stay: the adapter
stops the tracks 0.5 s after the last twist, the firmware 300 ms after the last
drive.

## What the blueprints contain

`vector-dimos.moss-nav` (mapping):

- the control coordinator with the MOSS base adapter (`vector_dimos/moss_adapter.py`,
  see `docs/moss_base.md`), now publishing the firmware telemetry on
  `/base_telemetry` (JSON: firmware, mode, battery mV, ticks, applied PWM) so
  nothing else ever has to open the serial port;
- the gamepad module with the MOSS envelope, wired straight to `cmd_vel`;
- the RPLidar C1 reader (`vector_dimos/rplidar_c1.py`) on its own USB stick;
- lidar odometry (kiss-icp, `vector_dimos/lidar_odometry.py`), the only pose
  source on MOSS tonight: the encoders are not calibrated (`MOSS_TICKS_PER_M`
  unset) and there is no IMU wired besides the one inside the D455F;
- the 2D scored costmap (`vector_dimos/costmap2d.py`), lidar writes obstacles;
- the RealSense colour stream at 640x480, 5 fps, carried as JPEG on the bus
  (`MOSS_CAMERA=0` to drop it);
- the wrist camera (the UVC camera on the arm, found by its `/dev/v4l/by-id`
  name, `MOSS_WRIST_DEV` to point at another `/dev/videoN`), 640x480, 5 fps, JPEG on
  the bus as `wrist_image` (`MOSS_WRIST=0` to drop it; an absent camera is a warning
  and the module is skipped);
- the recorder (below), `MOSS_RECORD=0` to drop it.

`vector-dimos.moss-explore` adds the recovering A* planner with the MOSS
footprint, the movement manager (pad input wins over navigation for one second)
and Explorer2 with the MOSS clearances. The pad is still there, through the
manager.

Neither blueprint starts Rerun or dimOS's own web modules: the cockpit is the
display. `MOSS_RERUN=1` adds the Rerun bridge for the desktop viewer.

## The robot profile

VECTOR's geometry used to live in module constants (lidar height, scan masks,
body box, footprint, planner widths). `vector_dimos/robot_profile.py` carries
both robots now; `ROBOT_PROFILE=moss` (exported by the launcher, set by the
MOSS blueprints when they are imported) selects MOSS, and VECTOR is the default
so nothing changes for it.

The MOSS values are PLACEHOLDERS, each overridable by an environment variable,
until measured on the robot:

| Value | Placeholder | Env |
|---|---|---|
| lidar height above the floor | 0.20 m | `MOSS_LIDAR_HEIGHT_M` |
| lidar yaw offset | 0 deg | `MOSS_LIDAR_YAW_DEG` |
| returns that are the robot itself (half extents) | 0.22 x 0.18 m | `MOSS_BODY_BOX_M="0.22 0.18"` |
| robot width for the planner | 0.34 m | `MOSS_ROBOT_WIDTH_M` |
| pivot diameter | 0.50 m | `MOSS_PIVOT_DIAMETER_M` |
| track centre distance | 0.266 m (CAD) | `MOSS_TRACK_M` |
| speed ceiling (recorded only, no watchdog reads it yet) | 0.30 m/s | `MOSS_MAX_SPEED_MPS` |
| static-friction floor of the base adapter (percent) | 0 (off) | `MOSS_MIN_PCT` |

The real speed knobs today are `MOSS_MAX_PCT` (the PWM cap of the base adapter, 100 by
default) and `MOSS_NERF_SPEED` (the launcher's `--nerf-speed` for the planner, 0.5 by
default); the profile's ceiling waits for a MOSS speed watchdog.

`MOSS_MIN_PCT` is the static-friction floor: a non-zero track command below it is
raised to it, sign kept (a command that rounds to zero stays a stop). The first armed
exploration showed why: the planner's slowest pivot (0.2 rad/s) maps to 8 % PWM on
MOSS, and the tracks do not turn below roughly 25-40 %, so the rover sat still and
the planner logged `Robot is stuck. Replanning.` until the goal timed out. The floor
is 0 (off) until measured: find the lowest percent that turns both tracks on the
floor, and launch with `MOSS_MIN_PCT=<that> EXPLORE=1 tools/moss-fly.sh explore`.

The lidar's floor projection is `base_link`: if the C1 is not mounted at the
rover's rotation centre, the footprint and the pivot clearance are off by that
offset. An off-centre lidar is not supported yet: `lidar_xy_base_m`
(`MOSS_LIDAR_XY_M`) only records the number until the odometry applies it.

## Recording

`MossRecorder` (a dimOS `Recorder`, WAL-safe rotation from `vector_dimos/memory.py`)
writes the whole run into one SQLite database,
`~/.local/state/moss/recordings/moss.db` (`MOSS_STATE_DIR` moves the state
directory; on the Jetson it is a symlink to the NVMe). Streams: `odom`,
`global_costmap`, `lidar` (world-frame cloud), `cmd_vel`, `base_telemetry`,
`color_image` (JPEG), `wrist_image` (JPEG) and `episode_marker`. The previous run is rotated aside
(`moss.<stamp>.db`, last ten kept).

The cockpit's "Record episode" button does not start or stop the recorder: it
writes a marker (`{"action": "start"|"stop", "name": ..., "ts": ...}`) on the
`episode_marker` stream. Cutting episodes on those markers is a script to write
(dimOS's `dataprep` cuts on its own `status` stream or on hand-entered ranges,
not on these markers). Reading a recording without the robot:

```
$ .venv/bin/dimos mem summary ~/.local/state/moss/recordings/moss.db
$ .venv/bin/dimos dataprep build --source <db> --format lerobot --output <dir> --config <json>
```

`--config` is required: a JSON `DataPrepConfig` naming the observation and action
streams (for MOSS: `color_image`, `odom`, `cmd_vel`) and the episode extractor.

## The cockpit

`tools/moss_cockpit.py` is a stdlib HTTP server on port 8904 that taps the
dimOS bus (explicit LCM channels, decoded on request, never a serial port) and
serves one page, `tools/moss_cockpit/index.html`, in the MOSS brand (Space
Grotesk, studio blue, mint accent, salmon counterpoint; fonts and marks served
locally, no CDN, the field has no internet).

Panels: flight check (last `moss_preflight.py --json`), base (battery on a 3S
scale, mode, applied PWM per track, ticks, firmware), system (CPU, RAM, temps,
disk), the live map (occupancy grid + lidar scan + pose + path + goal; drag to
pan, wheel or pinch to zoom, tap to send a goal), the two cameras under the map
(nose = RealSense colour, wrist = the arm's camera; MJPEG, 5 fps each, one
"Pause video" for both), controls (launch map / explore, stop stack, start / stop exploration, record
episode, STOP) and the log tail.

STOP on the page is a soft stop: it publishes `stop_explore_cmd`,
`stop_movement` and a zero twist. The hard stops are the firmware deadman
(300 ms) and `tools/moss-fly.sh stop`.

A goal tapped on the map goes out at once when nothing is exploring. While
exploring, the page stops the explorer first and sends the goal three seconds
later, after the explorer's own last goal, which would override it otherwise.
The cockpit refuses goals, exploration and launches when the running stack is
not a MOSS one, so VECTOR's stack on the same computer never receives its
commands; STOP is always allowed. Every `POST /cmd/*` must carry the header
`X-Requested-With: moss-cockpit` (the page does, a cross-site form cannot).

Endpoints, for anything else that wants the data: `/state.json`, `/map.png`,
`/map_meta`, `/scan.json`, `/path.json`, `/log.json`, `/preflight.json`,
`/launch.json`, `/camera.mjpg` (`?cam=nose`, the default, or `?cam=wrist`), and
`POST /cmd/{explore,goal,stop,episode,stack}`.
`tools/moss_cockpit.py --demo` serves synthetic data for working on the page
without a robot.

The server runs as `moss-cockpit.service` (`tools/moss-cockpit.service`,
`KillMode=process` so restarting the cockpit never kills a launcher or a stack
it started) on the Jetson, or by hand with `tools/moss-fly.sh cockpit`.

## Not done, on purpose

- No calibration happened: ground speed, ticks per metre, track width, lidar
  pose, the footprint and the static-friction floor are placeholders. The first measurements are listed in
  `docs/moss_base.md`.
- No gyro prior for the lidar odometry (`use_gyro_prior=False`): the D455F IMU
  can feed it once the camera mount is measured. No switch is wired for it yet.
- No persistent map, no keep-out zones on MOSS yet (`PERSISTENT_MAP=0` is
  exported by the launcher; the saved map on the Jetson belongs to VECTOR).
- No replay of a MOSS recording through the stack; the SQLite is readable with
  the dimOS tools above.
- The planner's minimum speeds (0.2 m/s, 0.2 rad/s) sit inside the motors'
  dead band at the current gearing: planned turns in place may stall until the
  higher-ratio motors and driver are in, or a static-friction floor is added in
  the adapter.

## Deploying to the robot

The robot runs an rsync of this repo (no git clone there), with the package
installed editable in its venv. New blueprint names only become runnable after
`pip install -e . --no-deps` on the robot, and an rsync from a development
machine overwrites `vector_dimos.egg-info/` with that machine's copy: regenerate
it there first (`python3 -c "from setuptools import setup; setup()" egg_info`)
or exclude it from the sync, otherwise the flight check reports the MOSS
blueprints missing while the venv knows them.

## Cold benches

`tests/test_robot_profile_cold.py`, `tests/test_moss_blueprints_cold.py`,
`tests/test_moss_preflight_cold.py`, `tests/test_moss_fly_gates_cold.py`,
`tests/test_moss_cockpit_cold.py`, plus the existing `tests/test_moss_adapter_cold.py`.
On the Jetson: `PYTHONPATH=. .venv/bin/python3 tests/<name>.py`; on a machine
without dimOS the sections that need it are skipped and say so.

# vector-dimos

[VECTOR](docs/hardware.md) is a custom-designed holonomic mobile platform, built from
scratch: four mecanum wheels on brushless hub motors, a steel chassis, and enough
torque to carry a robot arm. It handles gravel, grass, tiles and ramps.

This package integrates VECTOR into [dimOS](https://github.com/dimensionalOS/dimos)
as an **external blueprint package** — no fork, no patches. dimOS discovers it
through Python entry points and composes our blueprints with its own.

## How it plugs in

- `VectorBaseAdapter` implements the dimOS `TwistBaseAdapter` protocol (holonomic,
  3 DOF: `[vx, vy, wz]`) on top of two ZLAC8015D dual-channel drivers spoken to
  over MODBUS RTU (RS485).
- The adapter registers itself under the name `"vector"` in the dimOS twist-base
  adapter registry at import time.
- Blueprints are exposed through the `dimos.blueprints` entry-point group, so the
  dimOS CLI sees them natively:

```console
$ dimos run vector-dimos.base vector-dimos.gamepad
```

| Blueprint | What it does |
|---|---|
| `vector-dimos.base` | ControlCoordinator driving the mecanum base through `VectorBaseAdapter` (velocity task on `cmd_vel`) |
| `vector-dimos.gamepad` | Gamepad teleop (pygame joystick) publishing holonomic twists on `tele_cmd_vel` |
| `vector-dimos.rplidar` | RPLIDAR C1 published as a flat `PointCloud2`, ready for the dimOS costmap / A* stack |

Perception composes with the stock dimOS `real-sense-camera` module (a RealSense
D455F sits on VECTOR's mast). Localization doctrine — why wheel odometry is never
the reference on mecanum, and how the point cloud and the lidar split the job —
is in [docs/localization.md](docs/localization.md).

## Install

```console
$ pip install -e .
```

The core (kinematics, ZLAC8015D driver, adapter) is pure Python — `numpy` +
`pymodbus` — and testable without dimOS or hardware. For the dimOS runtime,
install dimOS **from git main**: the external-blueprints mechanism is newer than
the current PyPI release.

```console
$ pip install "vector-dimos[gamepad,rplidar] @ ."
$ pip install git+https://github.com/dimensionalOS/dimos.git
```

## Cold benches

Everything below runs without the robot, on a mocked MODBUS bus:

```console
$ python tests/test_kinematics.py        # inverse/forward mecanum, roundtrip identity
$ python tests/test_adapter_cold.py      # known twist -> known per-wheel RPM (real
                                         # wheel topology and signs), odometry roundtrip
$ python tests/test_blueprints_cold.py   # the three blueprints resolve through dimOS
```

## Layout

```
vector_dimos/
  kinematics.py    # X-config mecanum inverse/forward kinematics + real wheel topology
  zlac8015d.py     # minimal ZLAC8015D MODBUS RTU driver (velocity mode)
  adapter.py       # VectorBaseAdapter: dimOS TwistBaseAdapter over the two drivers
  blueprints.py    # base / gamepad / rplidar blueprint definitions
  gamepad.py       # pygame joystick -> Twist module
  rplidar_c1.py    # RPLIDAR C1 -> flat PointCloud2 module
  wheel_odom.py    # optional low-confidence wheel odometry (see localization doctrine)
  mock.py          # mock MODBUS client for the cold benches
docs/
  hardware.md      # what VECTOR is made of
  localization.md  # the localization doctrine
```

## Credits

- First version of the robot code by Sam.
- [dimOS](https://github.com/dimensionalOS/dimos) by Dimensional, Apache-2.0.
- License: Apache-2.0.

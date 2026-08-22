# dimOS on a Jetson Orin Nano Super (JetPack 6.2) — bring-up notes

VECTOR's compute board. Everything below was checked on the board itself on
2026-08-22; anything that was not checked says so explicitly. Most of it is not
VECTOR-specific — the install path applies to any headless Jetson running dimOS.

## Board and OS

| | |
|---|---|
| Board | Jetson Orin Nano Super — `/proc/device-tree/model`: "NVIDIA Jetson Orin Nano Engineering Reference Developer Kit Super", SoC `tegra234` |
| JetPack | 6.2 = L4T R36.4.3 (`/etc/nv_tegra_release`), kernel `5.15.148-tegra` |
| Userspace | Ubuntu 22.04.5 LTS, aarch64, glibc 2.35 |
| RAM | 7.4 GiB |
| Boot media | microSD — root on `/dev/mmcblk0p1`, 116 GB. Image written with NVIDIA's `jetson-disk-image-creator` flow, pre-provisioned headless (no monitor, no keyboard, ssh from first boot) |
| Power mode | 25 W (`sudo nvpmodel -q`) |
| System Python | 3.10.12 — not used, see below |

## Install path that worked

The system interpreter is 3.10; we run 3.12 from a `uv` venv, which keeps the
JetPack system packages untouched.

```console
$ uv venv --python 3.12          # -> .venv, Python 3.12.14
$ uv pip install -e .            # numpy 2.5.2, pymodbus 2.5.3, pyserial 3.5
```

dimOS itself has to come from **git main**: the external-blueprints mechanism
this package plugs into is newer than the PyPI release (0.0.13). Installing from
git builds an sdist, and that build fails on a missing web asset — the cockpit
UI is produced by a `deno` build step and is not shipped in the source tree.
dimOS's `setup.py` provides the escape hatch:

```console
$ DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install "git+https://github.com/dimensionalOS/dimos.git"
```

That builds a UI-less wheel, which is what a headless robot wants anyway.
Installed here: `dimos 0.0.14b1`, commit `a7d19f76`. The variable is a
build-time flag only — the CLI runs fine without it afterwards.

Verification that the entry points are seen:

```console
$ dimos list
[...stock blueprints...]

External blueprints:
  vector-dimos.base
  vector-dimos.gamepad
  vector-dimos.rplidar
```

At run time, pass `--no-local-relay`. The cockpit relay bridge lives in dimOS's
`web` extra, which a UI-less build does not have; dimOS's `local_relay` already
defaults to false, so the flag is belt and braces — it states the intent and
survives a config file that turns the relay on.

## aarch64 extras

| Package | Status on this board |
|---|---|
| `numpy` 2.5.2 | wheel `cp312-manylinux_2_28_aarch64` |
| `pymodbus` 2.5.3, `pyserial` 3.5 | pure python |
| `pygame` 2.6.1 | wheel `cp312-manylinux2014_aarch64`, installs clean |
| `rplidar-roboticia` 0.9.5 | pure python |
| `pyrealsense2` (upstream) | installs, **does not import**: the bundled `.so` needs `GLIBC_2.38`, JetPack 6.2 ships glibc 2.35. The wheel is tagged `manylinux2014` but is not manylinux2014 in practice |
| `pyrealsense2-extended` (what dimOS actually depends on) | cp312 aarch64 wheel imports fine on this glibc; `rs.context().query_devices()` returns 0 devices, which is correct with no camera plugged |

So the dimOS RealSense path needs no local librealsense build on this board.
Checked in a throwaway venv, import + device enumeration only: streaming from a
real D455F is untested (no camera mounted yet).

## Serial

The 40-pin header UART is **`/dev/ttyTHS1`** (`root:dialout`; the login user is
in `dialout`, so no udev rule and no `sudo`). It is free, and there is nothing
to disable to keep it free:

- `nvgetty.service` is enabled and its description says "UART on ttyTHS0", but
  on Orin it is a no-op: `/etc/systemd/nvgetty.sh` only starts a getty when the
  SoC is `tegra194` (Xavier). This is `tegra234`, so the unit exits immediately
  (`inactive (dead)`) and `/dev/ttyTHS0` does not even exist on this board.
- The serial login console is on the debug UART instead:
  `serial-getty@ttyTCU0` (plus `ttyGS0` over the USB device port). Neither
  touches ttyTHS1.

Also present and unused: `/dev/ttyTHS2`, `/dev/spidev0.0`, `/dev/spidev0.1`,
`/dev/spidev1.x`, `/dev/i2c-{0,1,2,4,5,7,9}`. No USB serial adapter
(`/dev/ttyUSB*`, `/dev/ttyACM*` absent) and no joystick (`/dev/input/js*`
absent) are connected today.

### Probing the RS485 bus

`tests/probe_rs485.py` is a read-only MODBUS probe: it opens the port and reads
the feedback-RPM register pair (`0x20AB`) on both driver ids. It writes nothing,
so it is safe to run with the motors powered.

```console
$ python tests/probe_rs485.py                 # defaults to /dev/ttyTHS1
port /dev/ttyTHS1 open: True
unit 1 (back): no answer (504 ms) -> Modbus Error: [Input/Output] Modbus Error: [Invalid Message] No response received, expected at least 2 bytes (0 received)
unit 2 (front): no answer (508 ms) -> Modbus Error: [Input/Output] Modbus Error: [Invalid Message] No response received, expected at least 2 bytes (0 received)
```

That is exactly what an unwired bus looks like, and it is the current state of
VECTOR: the port opens (a UART always opens — `open: True` proves nothing about
wiring) and each unit times out after the 500 ms pymodbus timeout. A driver that
answers prints `unit N (side): regs=[...]` in a few milliseconds instead.

That 500 ms is `SERIAL_TIMEOUT_S` in `vector_dimos/adapter.py`, and it is the
price of a silent drive at run time too: pymodbus waits the whole timeout before
giving up, so one silent controller adds it to every read and every write of a
control tick. The adapter no longer polls the second controller once the first
has gone quiet, which halves that, but the number itself is a conservative
bring-up value — time an *answering* drive with this probe and pass a smaller
`timeout_s` to the adapter.

## Commands

Cold benches — no hardware, no pytest. Every script under `tests/` puts the repo
root on `sys.path`, so they run straight from a checkout, from any working
directory. The first three need only the standard library — the drive core of
the package imports nothing else — while the last three import dimOS modules
(and numpy, for the lidar one):

```console
$ python tests/test_kinematics.py          # inverse/forward mecanum, roundtrip identity
$ python tests/test_adapter_cold.py        # known twist -> known per-wheel RPM, odometry
$ python tests/test_adapter_bus_faults.py  # silent/dying bus, frozen odometry, mock bus
$ python tests/test_gamepad_cold.py        # known axis values -> known twists, pad hot-plug
$ python tests/test_rplidar_cold.py        # polar -> metres, sensor hot-plug and retry
$ python tests/test_blueprints_cold.py     # the three blueprints resolve through dimOS
```

All six pass on this board (2026-08-22).

Runtime:

```console
$ dimos list                                              # blueprints, ours included
$ dimos run vector-dimos.base --no-local-relay            # foreground
$ dimos run vector-dimos.base --no-local-relay --daemon   # background
$ dimos status                                            # what is running
$ dimos log -f                                            # follow the run log
$ dimos stop                                              # stop the run
```

With no motors on the bus, add the mock bus switch:

```console
$ VECTOR_MOCK_BUS=1 dimos run vector-dimos.base --no-local-relay
```

`VECTOR_MOCK_BUS=1` makes the adapter bind to the in-memory MODBUS mock
(`vector_dimos/mock.py`) instead of opening the serial port, so the dimOS
pipeline — coordinator, control ticks, twist to per-wheel RPM, feedback,
odometry integration — can be exercised on a machine with no hardware at all.
It exercises plumbing, never motion: the mock is a perfect no-slip bus that
echoes back whatever was commanded.

`dimos run` completes on this board. Captured 2026-08-22 with `VECTOR_MOCK_BUS=1`,
`tests/publish_twist.py --vx 0.37 --vy -0.21 --wz 0.83` from a second shell, then
`dimos log`:

```
[inf] adapter.py    VECTOR base: MOCK BUS (VECTOR_MOCK_BUS set) - no motors will move
[inf] coordinator.py Added hardware base with joints: ['base/vx', 'base/vy', 'base/wz']
[inf] tick_loop.py  TickLoop started at 100.0Hz
[inf] coordinator.py ControlCoordinator started at 100.0Hz
[inf] adapter.py    VECTOR base MOCK: twist vx=+0.370 vy=-0.210 wz=+0.830 -> wheel RPM FL=+26 FR=+41 BL=-12 BR=+79 (bus raw front L/R=-26/+41 back L/R=+12/+79)
[war] velocity_task.py JointVelocityTask vel_base timed out (no update for 0.205s)
```

The publisher printed `expected wheel RPM  FL=+26 FR=+41 BL=-12 BR=+79` from the
geometry before publishing anything (transcript taken with the 0.105 m radius
the package shipped at the time; with the measured 0.085 m the same twist
reads `FL=+33 FR=+51 BL=-15 BR=+98`, bus raw front L/R=-33/+51 back L/R=+15/+98), so that is a known twist in and the known
per-wheel RPM out, through the whole runtime. The timeout warning is the
`JointVelocityTask` watchdog doing its job once the publisher stopped.

## First run: system configuration

dimOS checks a few system settings before building the blueprint and fixes them
itself through `sudo`. On this board, first run applied:

```
- Multicast: sudo ip link set lo multicast on
- Multicast: sudo ip route add 224.0.0.0/4 dev lo
- socket buffer optimization for LCM: sudo sysctl -w net.core.rmem_max=67108864
- socket buffer optimization for LCM: sudo sysctl -w net.core.rmem_default=67108864
```

Two consequences for a headless robot. The user running dimOS needs passwordless
`sudo`, otherwise the run dies in the preflight (`CalledProcessError` on
`sudo ip link set lo multicast on`) — that is what happens on a workstation with
a password-protected sudo. And none of those settings survive a reboot: dimOS
re-applies them at every run, which is fine as long as the sudo rule stays.

## Resolved on this board

- **`vector-dimos.base` used to die at module start** with
  `KeyError: "Unknown twist base adapter: vector. Available: ['flowbase', 'mock_twist_base', ...]"`.
  dimOS deploys modules into `forkserver` workers, and a worker is a fresh
  interpreter: it re-imports dimos, gets a brand-new adapter registry, and never
  imports `vector_dimos.blueprints`, so the `register_path("vector", ...)` that
  runs in the CLI process is simply absent where the hardware is built. Fixed by
  deploying our own `ControlCoordinator` subclass: classes pickle by reference,
  so unpickling `vector_dimos.blueprints:VectorControlCoordinator` in the worker
  imports the module and runs the registration. The module docstring of
  `vector_dimos/blueprints.py` has the full reasoning, and
  `tests/test_blueprints_cold.py` pins it in a clean interpreter.

## Open items

- **Control-tick budget on the real bus. Unmeasured.** The coordinator runs at
  the dimOS default 100 Hz, i.e. a 10 ms budget per tick. On the mock bus that
  is free; on RS485 a tick is two `read_holding_registers` and — once any twist
  has arrived, because the velocity task stays active — two `write_registers`,
  roughly 76 bytes, about 7 ms of wire time at 115200 8N1 before any drive
  turnaround. `TickLoop._loop` only sleeps when there is slack, so an overrun
  silently lowers the rate instead of complaining. Measure it as soon as the
  drivers are wired (`log_ticks=True` for a minute), then set `tick_rate`
  explicitly in `_coordinator_blueprint()` to what the bus sustains. Test
  first, then limit.
- **One-sided bus outage: decision pending.** `write_velocities()` writes the
  two controllers independently, so if one RS485 link drops mid-drive the
  healthy axle keeps following the stick while the silent one holds its last
  command — the robot would turn on two wheels. Software cannot reach a drive
  that does not answer, so the choices are: leave it (the healthy axle at least
  still obeys), zero the healthy axle when its partner refuses, or configure a
  communication-loss stop on the ZLAC8015D itself if its parameter set has one.
  `tests/test_adapter_bus_faults.py` case (f) pins today's behaviour so the
  change, if any, is visible. Decide with the drivers wired, not before.
- **ZLAC8015D wiring.** The two drivers are not on `/dev/ttyTHS1` yet (probe
  times out on both ids). Nothing about the drive chain is verified on hardware:
  wheel topology, signs and geometry are cold-bench facts only.
- **RPLIDAR C1.** Not connected. It attaches through a CP2102 USB-UART adapter,
  4 wires (GND, 5 V, TX, RX — the C1's RX is 3.3 V logic), 460800 baud, which is
  what `rplidar_c1.py` defaults to (the `rplidar-roboticia` library itself
  defaults to 115200).
- **RealSense D455F.** Not mounted. The python binding imports; streaming,
  frame rates and USB behaviour under load are untested.
- **Gamepad.** `vector-dimos.gamepad` runs headless on this board (it waits for
  a pad and logs that once), but nothing has been plugged in yet, so the teleop
  path itself — axes to twist to wheels — is a cold-bench fact only.


## First motion on blocks (2026-08-22)

Rover on blocks, wheels in the air, one hand on the motor power. Both drives
answered the read-only probe (`tests/probe_rs485.py`: back 5 ms, front 7 ms),
the software e-stop was acknowledged by both before anything was armed
(`tests/estop_rs485.py`: 0 RPM then DISABLE), then:

```
dimos run vector-dimos.base --no-local-relay --daemon > /dev/null 2>&1 < /dev/null   # no VECTOR_MOCK_BUS
dimos log                      # both: "ZLAC8015D id N (...) enabled, faults L/R=0/0"
python tests/publish_twist.py --vx 0.2 --vy 0   --wz 0   --duration 3.0   # forward
python tests/publish_twist.py --vx 0   --vy 0.2 --wz 0   --duration 3.0   # strafe left
python tests/publish_twist.py --vx 0   --vy 0   --wz 0.5 --duration 3.0   # yaw left
dimos stop                     # SIGTERM -> coordinator.stop() -> adapter.disconnect(): 0 RPM + DISABLE
python tests/estop_rs485.py    # belt and braces, both drives ack
```

Known twist in, RPM the drives measured out (FL/FR/BL/BR, r = 0.085 m, k = 0.35 m):

| twist | expected RPM | encoder feedback (one 0.5 s sample) |
|---|---|---|
| vx = +0.2 m/s | +22 / +22 / +22 / +22 | +22.9 / +23.3 / +22.9 / +23.0 |
| vy = +0.2 m/s | −22 / +22 / +22 / −22 | −21.0 / +22.3 / +21.8 / −21.0 |
| wz = +0.5 rad/s | −20 / +20 / −20 / +20 | −20.2 / +21.4 / −20.4 / +20.6 |

Both sides of that table are in the run's log: the adapter now logs the RPM it
commanded (`VECTOR base: twist ... -> wheel RPM ...`, one line per change, a
zero always) and what the drives report turning (`VECTOR base feedback: wheel
RPM ...`, one line per 0.5 s while any wheel moves). Read them in the run's
`main.jsonl` under `~/.local/state/dimos/logs/` — `dimos log -n` did not show
the command lines in this session. The 400 ms ramp and the
`JointVelocityTask` watchdog (zero ~0.2 s after the last Twist, wheels at rest
within ~0.5 s) are visible in the feedback lines. 12 RPM (`--wz 0.3`) did turn
the wheels: the ~15 RPM floor noted in the first robot code is not a floor here.

What this does and does not prove. It proves the whole chain (LCM -> coordinator
-> JointVelocityTask -> adapter -> MODBUS -> drives -> encoders) with the
FL/FR/BL/BR topology and the inverted left ports self-consistent. It does NOT
prove which end of the chassis is +x: encoders cannot see that. A look at the
wheels (or the D455F gyro on the ground: wz > 0 must read counter-clockwise)
settles it.

Drive parameters read afterwards (`tests/read_zlac_params.py`, read-only):
**0x2000 communication-offline time = 0 on both drives, i.e. the drive-side
watchdog is off.** If the runtime dies uncleanly with a non-zero target, the
wheels keep turning until the power is cut or `tests/estop_rs485.py` runs; the
clean path (`dimos stop`) zeroes and disables. Max RPM 0x2008 = 1000,
ramps 400/400 persisted, mode 3 (velocity).

# MOSS on the VECTOR stack: the tracked base adapter

`vector_dimos/moss_adapter.py` lets dimOS drive MOSS (metrox-eth/moss) the way
it drives VECTOR: the same coordinator, the same gamepad blueprint, a different
base underneath. MOSS has two tracks on geared DC motors, driven by an ESP32-S3
running the moss repo's bench firmware (`firmware/MOSS_ESP32S3`), over the
board's USB-UART at 115200 baud.

## Run it (Jetson, MOSS wired)

The gated way, on the Jetson (pad receiver and QT Py on its USB):

```
$ tools/moss-teleop.sh            # gates: no stack up, js0, QT Py port free, firmware disarmed, battery >= 10.8 V -> stack
$ tools/moss-teleop.sh log        # what the base and the pad said
$ tools/moss-teleop.sh stop       # stack down (the firmware deadman stopped the tracks 300 ms after the last drive)
$ MOCK=1 tools/moss-teleop.sh     # same stack on the fake firmware, no motor
```

Pad (shanwan receiver, Android mode, the one VECTOR flies with): **R1 held = deadman**
(SDL button 7), left stick up/down = forward/back, right stick left/right = turn,
R2 = boost (capped at `MOSS_MAX_PCT`). The stick scale is the base's envelope
(`teleop_speeds`: full stick = `MOSS_MPS_AT_LIMIT`, full turn = a spin at the
limit), not VECTOR's 0.6 m/s - with those the stick was an on/off switch at 25 %.
Below ~8 % duty the motors do not start (bench 05/10: nothing at 5 %, fine at 15 %).

By hand:

```
$ ls /dev/serial/by-id/            # find the ESP32 USB-UART (CP210x / CH340)
$ export MOSS_PORT=/dev/serial/by-id/usb-Silicon_Labs_CP2102N_...-if00-port0
$ dimos run vector-dimos.moss-gamepad          # gamepad -> tracks
$ dimos run vector-dimos.moss-base             # base only, cmd_vel from elsewhere
```

Without hardware: `MOSS_MOCK_BUS=1 dimos run vector-dimos.moss-gamepad` drives
the fake firmware (`vector_dimos/moss_mock.py`) end to end; the `dimos log`
lines `MOSS base MOCK: twist ... -> tracks L=..% R=..%` are the proof.

Cold bench (no hardware, no dimOS): `python3 tests/test_moss_adapter_cold.py`.

## What the firmware imposes, and what the adapter does about it

| Firmware (Control.h) | Adapter |
|---|---|
| `arm` puts it READY for 10 s, then it disarms itself | a drive refused `ERR not_armed` re-arms and retries once; nothing is guessed from telemetry |
| `drive L R` in percent, `-25..+25`, refused beyond | `MossGeometry.max_pct` <= 25, clamped; the twist is scaled to it |
| a drive not refreshed within 300 ms stops the tracks (`timeout`) | keepalive re-sends the last drive every 100 ms while the last twist is < 0.5 s old; beyond that one `stop`, then silence |
| any invalid line stops and disarms | only `arm` / `drive` / `stop` / `status` ever go on the wire |
| telemetry JSON every 200 ms (`mode`, `ticks`, ...) | mode and encoder ticks are read from it; ticks feed odometry once calibrated |

Two deadmen, both on the robot: the adapter's (0.5 s, in the dimOS process on
the Jetson) and the firmware's (0.3 s, on the ESP32). A cockpit on a laptop
that loses the link stops the tracks without anyone's help.

## Calibrate before trusting a number

The firmware is open loop (PWM percent). Three numbers turn a twist into
tracks and ticks into metres; all three ship as PLACEHOLDERS and each has an
env override, so a flight can be re-scaled without a code change:

| Env | Meaning | How to measure |
|---|---|---|
| `MOSS_TRACK_M` | distance between the two track centrelines | tape measure, sprocket centre to sprocket centre |
| `MOSS_MPS_AT_LIMIT` | floor speed at `MOSS_MAX_PCT` | `drive 25 25` for 3 s on the floor (firmware `test` or the gamepad), tape measure, divide |
| `MOSS_TICKS_PER_M` | encoder ticks per metre of track travel | `zero`, push or drive the rover exactly 1 m, read `ticks` in a `status` line, average the two sides |

Until `MOSS_TICKS_PER_M` is set, `read_odometry()` returns `None` and
`read_velocities()` returns the commanded twist: an honest "no measurement"
rather than a pose made of guesses. Set it and both come from the encoders.

`vy` (strafing) is accepted and ignored: a tracked base cannot do it, and the
stock gamepad blueprint still sends it. `dof` is reported as 3 for that reason.

# VECTOR hardware

Custom design, built from scratch. One-off — this repo is its software.

## Drive

| Component | Details |
|---|---|
| Wheels | 4× mecanum (X configuration) on brushless hub motors |
| Motor drivers | 2× ZLAC8015D dual-channel AC servo drivers |
| Driver bus | MODBUS RTU over RS485, 115200 baud 8N1 — unit 2 = front pair, unit 1 = back pair |
| Compute | NVIDIA Jetson Orin Nano Super (JetPack 6.2) — bring-up notes: [jetson_orin_nano.md](jetson_orin_nano.md) |
| RS485 interface | Waveshare RS485/CAN HAT on the Jetson's 40-pin header — the bus is `/dev/ttyTHS1` (the header UART; the login console is elsewhere, nothing to disable) |

Wheel topology (as wired, encoded in `kinematics.py` and verified by the cold
bench): each driver's left channel is inverted — a negative RPM command drives
the left wheels forward.

## Perception

| Component | Details |
|---|---|
| Depth camera | Intel RealSense D455F, mounted on a mast (drives the primary point cloud), USB |
| Lidar | Slamtec RPLIDAR C1 (360° 2D), published as a flat `PointCloud2` |
| Lidar interface | CP2102 USB-UART adapter, 460800 baud — 4 wires: GND, 5 V, TX, RX. The C1's RX is 3.3 V logic |
| IMU | The one inside the D455F — no separate IMU on the platform |

## Power

| Component | Details |
|---|---|
| Battery | LiFePO4 24 V 30 Ah |
| BMS | 8S 60 A |
| Charger | 29.2 V 20 A |
| Protection | General disconnect, per-driver breakers, 100 A main fuse |
| Metering | PZEM-017 shunt meter (100 A) — shares the RS485 bus with the drivers |

## Geometry

Values used by `kinematics.py`:

- Wheel radius: 0.105 m. Careful with that number — the wheel is *nominally* 8"
  and the two are not the same thing: 8 in is 0.2032 m, i.e. a 0.1016 m radius,
  while 0.105 m is 8.27 in. 0.105 m is the value the first robot code (Sam's
  driver, `R_Wheel`) ran on this chassis, and it is what `kinematics.py` ships;
  the nominal 8" is where it comes from, not a confirmation of it. Radius
  scales odometry linearly, so it stays unvalidated until an odometry roundtrip
  on the real chassis (drive a known distance, compare `/odom`) says which of
  the two — or neither — the wheel actually turns at.
- Half wheelbase: 0.15 m
- Half track: 0.20 m
- Kinematic constant K (half wheelbase + half track): 0.35 m

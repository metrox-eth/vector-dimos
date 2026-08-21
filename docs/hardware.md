# VECTOR hardware

Custom design, built from scratch. One-off — this repo is its software.

## Drive

| Component | Details |
|---|---|
| Wheels | 4× mecanum (X configuration) on brushless hub motors |
| Motor drivers | 2× ZLAC8015D dual-channel AC servo drivers |
| Driver bus | MODBUS RTU over RS485, 115200 baud — unit 2 = front pair, unit 1 = back pair |
| Compute | NVIDIA Jetson Orin Nano Super |
| RS485 interface | Waveshare RS485/CAN HAT on the Jetson |

Wheel topology (as wired, encoded in `kinematics.py` and verified by the cold
bench): each driver's left channel is inverted — a negative RPM command drives
the left wheels forward.

## Perception

| Component | Details |
|---|---|
| Depth camera | Intel RealSense D455F, mounted on a mast (drives the primary point cloud) |
| Lidar | Slamtec RPLIDAR C1 (360° 2D), published as a flat `PointCloud2` |

## Power

| Component | Details |
|---|---|
| Battery | LiFePO4 24 V 30 Ah |
| BMS | 8S 60 A |
| Charger | 29.2 V 20 A |
| Protection | General disconnect, per-driver breakers, 100 A main fuse |
| Metering | PZEM-017 shunt meter (100 A) — shares the RS485 bus with the drivers |

## Geometry

Values used by `kinematics.py` (from the platform's first-life code; the wheel
radius is pending a physical re-measure and clearly marked in the code):

- Half wheelbase: 0.15 m
- Half track: 0.20 m
- Kinematic constant K (half wheelbase + half track): 0.35 m

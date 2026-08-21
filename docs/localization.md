# Localization doctrine

VECTOR has no GPS. Indoor and outdoor dead reckoning rests on three sources, and
the doctrine below decides which one is allowed to be the reference.

## 1. Wheel odometry is never the reference

On a mecanum platform, wheel odometry is unreliable **by construction**, not by
tuning: the rollers slip as part of normal operation (that is how the platform
strafes), so integrated wheel motion diverges from true motion in exactly the
maneuvers holonomic drive exists for. Terrain makes it worse — gravel and grass
add slip that no covariance tuning recovers.

`wheel_odom.py` still publishes it, low-confidence, because it is cheap, always
available, and useful as a sanity signal and a short-horizon prior. It must
never be fused as a trusted pose source.

## 2. The point cloud is primary

The RealSense D455F on the mast provides the dense 3D point cloud that the dimOS
mapping/navigation stack consumes (costmap, A* replanning). Visual/depth
odometry against that cloud is the primary motion estimate.

## 3. The 2D lidar anchors it

The RPLIDAR C1 gives a 360° planar scan with long, stable returns on walls and
structure. Its role is to anchor the point-cloud estimate: planar scans are far
less drift-prone over long traverses than pure visual odometry, and they cover
the camera's blind sides.

dimOS's navigation stack is PointCloud2-native (there is no 2D LaserScan path),
so `rplidar_c1.py` publishes the scan directly as a flat point cloud — it drops
into the same costmap as the camera cloud with zero extra plumbing.

## Summary

| Source | Role |
|---|---|
| D455F point cloud | Primary motion/pose estimate |
| RPLIDAR C1 (flat PointCloud2) | Long-horizon anchor, 360° coverage |
| Wheel odometry | Sanity signal only — never the reference |

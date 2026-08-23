"""Mapping / navigation blueprints - kept apart from blueprints.py because
dimOS's mapping modules pull torch + open3d; a missing heavy dependency must
only take `vector-dimos.nav` down, never the base / cockpit / lidar blueprints.
"""
from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect

from vector_dimos.blueprints import VectorControlCoordinator, _coordinator_blueprint
def _nav_blueprint():
    """Mapping stack: base + RealSense (colour/depth for the depth guard) +
    RPLIDAR C1 -> our lidar odometry (kiss-icp, the pose source: wheels are
    never the reference here) -> dimOS's stock VoxelGridMapper (CPU) and
    CostMapper -> Rerun. Stream names line up by construction: rplidar
    `pointcloud` -> LidarOdometry `pointcloud`; LidarOdometry `lidar` (world
    frame) -> VoxelGridMapper `lidar`; `global_map` -> CostMapper."""
    from dimos.core.global_config import global_config
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1

    return autoconnect(
        _coordinator_blueprint(),
        RealSenseCamera.blueprint(width=640, height=480, fps=15,
                                  enable_depth=True, enable_pointcloud=False,
                                  enable_imu=False),
        RPLidarC1.blueprint(),
        LidarOdometry.blueprint(),
        VoxelGridMapper.blueprint(voxel_size=0.05, device="CPU:0", frame_id="world"),
        CostMapper.blueprint(),
        vis_module(viewer_backend=global_config.viewer),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


nav_blueprint = _nav_blueprint()

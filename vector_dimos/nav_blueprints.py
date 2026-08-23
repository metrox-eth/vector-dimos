"""Mapping / navigation blueprints - kept apart from blueprints.py because
dimOS's mapping modules pull torch + open3d; a missing heavy dependency must
only take `vector-dimos.nav` down, never the base / cockpit / lidar blueprints.
"""
from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect

from vector_dimos.blueprints import VectorControlCoordinator, _coordinator_blueprint
def camera_mount():
    """base_link -> camera_link as mounted on VECTOR: 0.30 m ahead of the lidar
    axis, 0.57 m up (floor-plane fit 23/08; matches CAMERA_XYZ_BASE in
    lidar_odometry.py). Their default is identity,
    which drew the Rerun frustum on the floor under the lidar (metrox, 23/08)."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    return Transform(translation=Vector3(0.30, 0.0, 0.57), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))


CAMERA_MOUNT = camera_mount()


def costmapper_blueprint():
    """"simple" occupancy, not "height_cost": height_cost is a terrain-slope map
    for a walking robot and it discards any cell whose 4 neighbours are not
    observed - a table leg (1-2 cells at 0.37 m with unseen floor around it)
    came out UNKNOWN = traversable (measured 23/08 with tools/mars/stages.py:
    legs present in global_map, absent from the costmap). Simple: any point
    between min_height and max_height = OCCUPIED, a point below = FREE.
    Band = the rover: can climb 3 cm, passes under 0.65 m (mast top 0.60)."""
    from dimos.mapping.costmapper import CostMapper
    from dimos.mapping.pointclouds.occupancy import SimpleOccupancyConfig
    return CostMapper.blueprint(algo="simple", config=SimpleOccupancyConfig(min_height=0.03, max_height=0.65))


def mapper_blueprint():
    from dimos.mapping.voxels.module import VoxelGridMapper
    # emit_every 10 (~1 Hz): at 3 Hz the Rerun bridge ate 25 load and 2.5 GB
    return VoxelGridMapper.blueprint(device="CPU:0", voxel_size=0.05, frame_id="world", emit_every=10, carve_columns=False)


def _nav_blueprint():
    """Mapping stack: base + RealSense (colour/depth for the depth guard) +
    RPLIDAR C1 -> our lidar odometry (kiss-icp, the pose source: wheels are
    never the reference here) -> dimOS's stock VoxelGridMapper (CPU) and
    CostMapper -> Rerun. Stream names line up by construction: rplidar
    `pointcloud` -> LidarOdometry `pointcloud`; LidarOdometry `lidar` (world
    frame) -> VoxelGridMapper `lidar`; `global_map` -> CostMapper."""
    from dimos.core.global_config import global_config
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1

    return autoconnect(
        _coordinator_blueprint(),
        RealSenseCamera.blueprint(width=640, height=480, fps=15,
                                  enable_depth=True, enable_pointcloud=False,
                                  enable_imu=False,
                                  base_transform=CAMERA_MOUNT),   # their 2nd (motion) pipeline fails on the RSUSB build: "No device connected"
        RPLidarC1.blueprint(),
        LidarOdometry.blueprint(use_gyro_prior=False),
        mapper_blueprint(),
        costmapper_blueprint(),
        vis_module(viewer_backend=global_config.viewer),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


nav_blueprint = _nav_blueprint()


def _explore_blueprint():
    """Autonomous exploration with dimOS's own stack on top of `nav`:
    CostMapper -> ReplanningAStarPlanner (A* + its P controller -> nav_cmd_vel)
    -> MovementManager (-> cmd_vel, the coordinator's twist) and the wavefront
    frontier explorer choosing goals on the costmap. Start it by publishing
    Bool(True) on `explore_cmd`; cap the speed with `dimos --nerf-speed 0.3`
    (the local planner's default is 0.55 m/s)."""
    from dimos.core.global_config import global_config
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import WavefrontFrontierExplorer
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from vector_dimos.recovering_planner import RecoveringPlanner
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1
    from vector_dimos.stuck_guard import StuckGuard

    return autoconnect(
        _coordinator_blueprint(),
        RealSenseCamera.blueprint(width=640, height=480, fps=15,
                                  enable_depth=True, enable_pointcloud=False,
                                  enable_imu=False,
                                  base_transform=CAMERA_MOUNT),
        RPLidarC1.blueprint(),
        LidarOdometry.blueprint(use_gyro_prior=False),
        # carve_columns=False: a collision's virtual obstacle (stuck_guard) and low boxes
        # seen once must persist even when the lidar plane passes above them
        mapper_blueprint(),
        # dimOS's height-cost defaults are for a quadruped (can_climb 0.15 m, pass
        # under 0.6 m): VECTOR climbs nothing (3 cm) and its camera top is ~0.85 m.
        costmapper_blueprint(),
        RecoveringPlanner.blueprint(robot_width=0.62, robot_rotation_diameter=0.80),
        MovementManager.blueprint(),
        WavefrontFrontierExplorer.blueprint(safe_distance=0.35, lookahead_distance=4.0, min_frontier_perimeter=0.3,
                                            info_gain_threshold=0.01, max_explored_distance=12.0, goal_timeout=15.0),
        StuckGuard.blueprint(),
        vis_module(viewer_backend=global_config.viewer),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


explore_blueprint = _explore_blueprint()

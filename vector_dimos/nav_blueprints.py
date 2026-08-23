"""Mapping / navigation blueprints - kept apart from blueprints.py because
dimOS's mapping modules pull torch + open3d; a missing heavy dependency must
only take `vector-dimos.nav` down, never the base / cockpit / lidar blueprints.
"""
from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect

from vector_dimos.blueprints import VectorControlCoordinator, _coordinator_blueprint
def camera_mount():
    """base_link -> camera_link as mounted: 0.30 m ahead of the lidar axis,
    0.57 m up (CAMERA_XYZ_BASE). Their default is identity, which drew the
    Rerun frustum on the floor under the lidar (metrox noticed, 23/08)."""
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    return Transform(translation=Vector3(0.30, 0.0, 0.57), rotation=Quaternion(0.0, 0.0, 0.0, 1.0))


CAMERA_MOUNT = camera_mount()


def _nav_blueprint():
    """Mapping stack: base + RealSense (colour/depth for the depth guard) +
    RPLIDAR C1 -> our lidar odometry (kiss-icp, the pose source: wheels are
    never the reference here) -> dimOS's stock VoxelGridMapper (CPU) and
    CostMapper -> Rerun. Stream names line up by construction: rplidar
    `pointcloud` -> LidarOdometry `pointcloud`; LidarOdometry `lidar` (world
    frame) -> VoxelGridMapper `lidar`; `global_map` -> CostMapper."""
    from dimos.core.global_config import global_config
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from vector_dimos.costmap2d import VectorCostMap
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
        VoxelGridMapper.blueprint(voxel_size=0.05, device="CPU:0", frame_id="world", emit_every=3),
        VectorCostMap.blueprint(),   # our 2D map (learns/unlearns, two layers) - dimOS's CostMapper erased table legs
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
    from vector_dimos.costmap2d import VectorCostMap
    from dimos.mapping.voxels.module import VoxelGridMapper
    from vector_dimos.fast_explorer import VectorExplorer
    from vector_dimos.memory import VectorMemory
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
        VoxelGridMapper.blueprint(voxel_size=0.05, device="CPU:0", frame_id="world", emit_every=10, carve_columns=False),   # ~1 Hz: the Rerun bridge re-sends the whole map each time (load 25, viewer 2.5 GB at 3 Hz)
        # dimOS's height-cost defaults are for a quadruped (can_climb 0.15 m, pass
        # under 0.6 m): VECTOR climbs nothing (3 cm) and its camera top is ~0.85 m.
        VectorCostMap.blueprint(),   # our 2D map (learns/unlearns, two layers) - dimOS's CostMapper erased table legs
        # the rover is 54x46 cm but its corners sweep 0.71 m when it pivots and the
        # camera mast stands at the front bumper: give the planner a footprint with margin
        RecoveringPlanner.blueprint(robot_width=0.62, robot_rotation_diameter=0.80),
        MovementManager.blueprint(),
        VectorExplorer.blueprint(safe_distance=0.35, lookahead_distance=4.0, min_frontier_perimeter=0.3,
                                            # 1 % gain per goal made it quit once the first room was known while the
                                            # workshop was still unknown (23/08 18:36, after 29 m): 0.1 %, 6 tries
                                            info_gain_threshold=0.001, num_no_gain_attempts=6,
                                            max_explored_distance=12.0, goal_timeout=15.0),
        StuckGuard.blueprint(),
        VectorMemory.blueprint(),    # every run recorded (dimOS memory): replay, draw, tune planners without the robot
        vis_module(viewer_backend=global_config.viewer),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


explore_blueprint = _explore_blueprint()

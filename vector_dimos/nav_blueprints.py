"""Mapping / navigation blueprints - kept apart from blueprints.py because
dimOS's mapping modules pull torch + open3d; a missing heavy dependency must
only take `vector-dimos.nav` down, never the base / cockpit / lidar blueprints.
"""
from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect

from vector_dimos.blueprints import VectorControlCoordinator, _coordinator_blueprint
def camera_mount():
    """base_link -> camera_link as mounted since 24/08 (bumper build): 0.20 m
    BEHIND the lidar axis (metrox's tape), 0.56 m up (floor fit), nose 1.1 deg
    down. Matches CAMERA_XYZ_BASE in lidar_odometry.py.

    Frame convention, read off dimOS 25/08: ``base_transform`` is
    base_link -> camera_link in REP-103 BODY axes (x forward, y left, z up).
    ``RealSenseCamera._publish_tf`` publishes it unchanged, then
    ``_build_mount_edges`` appends camera_link -> camera_*_frame (factory
    extrinsics, body axes) -> camera_*_optical_frame carrying OPTICAL_ROTATION
    (-0.5, 0.5, -0.5, 0.5) from dimos/hardware/sensors/camera/spec.py. dimOS
    adds the optical rotation itself, so this mount must carry the MECHANICAL
    pose only - putting the optical rotation here would apply it twice.

    Hence a pure pitch: +1.1 deg about base_link's +y (left) axis = nose down.
    Axis map, following the optical line of sight z_opt = (0, 0, 1) outwards:
        OPTICAL_ROTATION:  z_opt -> +x (forward), x_opt -> -y, y_opt -> -z
        this mount:        +x    -> (cos 1.1, 0, -sin 1.1) = (0.9998, 0, -0.0192)
    i.e. the frustum looks forward over the body, 1.1 deg down.
    """
    from dimos.msgs.geometry_msgs.Quaternion import Quaternion
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    return Transform(translation=Vector3(-0.20, 0.0, 0.56),
                     rotation=Quaternion(0.0, 0.00959916, 0.0, 0.99995393))


CAMERA_MOUNT = camera_mount()


# --- Rerun display -------------------------------------------------------
#
# dimOS's bridge (dimos/visualization/rerun/bridge.py, ``_on_message``) converts
# every message with a bare ``msg.to_rerun()`` - no arguments - then attaches the
# entity to its TF frame with ``rr.Transform3D(parent_frame="tf#/<frame_id>")``.
# Two layers need that default overridden. ``visual_override`` is the hook, and
# its callables must be module-level: the blueprint is pickled to the worker.

VOXEL_SIZE = 0.05
# Height band the voxel map is coloured over, metres in the world frame. Fixed
# on purpose: PointCloud2.to_rerun() normalises its turbo ramp over the map's
# OWN z range, so a map with no height in it - which is all a 2D lidar can
# produce, every voxel sitting in the scan plane - has z.max() == z.min(), every
# class_id collapses to 0, and the layer is drawn in the darkest turbo colour on
# the blueprint's black background: invisible.
VOXEL_COLOR_FLOOR = 0.0
VOXEL_COLOR_CEILING = 1.2


def voxel_map_view(cloud):
    """``global_map`` coloured by ABSOLUTE height, so a flat map stays visible."""
    import numpy as np
    import rerun as rr

    points = cloud.points_f32()
    if len(points) == 0:
        return rr.Points3D([])
    height = (points[:, 2] - VOXEL_COLOR_FLOOR) / (VOXEL_COLOR_CEILING - VOXEL_COLOR_FLOOR)
    class_ids = (np.clip(height, 0.0, 1.0) * 255).astype(np.uint8)
    # class_ids resolve through the turbo AnnotationContext dimOS logs static at
    # "/" in rerun_init(); radii = half a voxel is the stock look.
    return rr.Points3D(positions=points, radii=VOXEL_SIZE / 2, class_ids=class_ids)


def camera_frustum_view(camera_info):
    """The camera frustum on the mast instead of at the world origin.

    A rerun 0.32 Pinhole owns the frame of the entity it is logged on: it
    connects that frame to the frame of the entity-path parent, here
    ``tf#/world``. So on ``world/camera_info`` the bridge's
    ``Transform3D(parent_frame="tf#/camera_color_optical_frame")`` is a SECOND
    parent for the same frame, rerun refuses it - "Any frame is only ever
    allowed to have a single parent at any given time" (re_tf::transform_forest)
    - and keeps the pinhole edge: the frustum is drawn at the world origin along
    world +z, at the sky, where no mount transform can reach it.

    Logging ``CoordinateFrame("tf#/camera_color_optical_frame")`` instead is
    worse, and silently: the pinhole then re-parents THAT tf frame to
    ``tf#/world`` (no warning), so the frustum still sits at the origin.

    What works, measured in the viewer 25/08: pose the entity, and give the
    pinhole a child path of its own to own. Returning a list of
    (path, archetype) also makes the bridge skip its own frame attachment.
    """
    import rerun as rr

    return [
        ("world/camera_info", rr.Transform3D(parent_frame=f"tf#/{camera_info.frame_id}")),
        # 1 m (the stock plane distance) dwarfs a 0.62 m rover.
        ("world/camera_info/frustum", camera_info.to_rerun(image_plane_distance=0.4)),
    ]


RERUN_CONFIG = {
    # 512MB replay history instead of the 25% default: on the 8 GB Jetson that
    # default let the Rerun bridge grow to 2.7 GB (measured 24/08).
    "memory_limit": "512MB",
    "visual_override": {
        "world/global_map": voxel_map_view,
        "world/camera_info": camera_frustum_view,
        # Aligned depth shares camera_color_optical_frame, so this is the same
        # frustum a second time - and it would stay pinned at the origin.
        "world/depth_camera_info": None,
    },
}


def _nav_blueprint():
    """Mapping stack: base + RealSense (colour/depth for the depth guard) +
    RPLIDAR C1 -> our lidar odometry (kiss-icp, the pose source: wheels are
    never the reference here) -> dimOS's stock VoxelGridMapper (CPU) and
    CostMapper -> Rerun. Stream names line up by construction: rplidar
    `pointcloud` -> LidarOdometry `pointcloud`; LidarOdometry `lidar` (world
    frame) -> VoxelGridMapper `lidar`; `global_map` -> CostMapper."""
    from dimos.core.global_config import global_config
    from vector_dimos.camera import VectorCamera
    from vector_dimos.costmap2d import VectorCostMap
    from dimos.mapping.voxels.module import VoxelGridMapper
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1
    from vector_dimos.respeaker import ReSpeakerMic

    return autoconnect(
        _coordinator_blueprint(),
        VectorCamera.blueprint(width=640, height=480, fps=15,
                               enable_depth=True, enable_pointcloud=False,
                               enable_imu=True, imu_hz=200,
                               base_transform=CAMERA_MOUNT),   # their 2nd (motion) pipeline fails on the RSUSB build: "No device connected"
        RPLidarC1.blueprint(),
        ReSpeakerMic.blueprint(stt_language="fr"),   # auto-detect mangles short utterances
        LidarOdometry.blueprint(use_gyro_prior=False),
        VoxelGridMapper.blueprint(voxel_size=0.05, device="CPU:0", frame_id="world", emit_every=3),
        VectorCostMap.blueprint(),   # our 2D map (learns/unlearns, two layers) - dimOS's CostMapper erased table legs
        vis_module(viewer_backend=global_config.viewer, rerun_config=RERUN_CONFIG),
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
    from vector_dimos.camera import VectorCamera
    from vector_dimos.costmap2d import VectorCostMap
    from dimos.mapping.voxels.module import VoxelGridMapper
    from vector_dimos.fast_explorer import VectorExplorer
    from vector_dimos.esp_sensors import EspSensors
    from vector_dimos.imu_slip import ImuSlipDetector
    from vector_dimos.memory import VectorMemory
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from vector_dimos.recovering_planner import RecoveringPlanner
    from dimos.visualization.vis_module import vis_module
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.rplidar_c1 import RPLidarC1
    from vector_dimos.respeaker import ReSpeakerMic
    from vector_dimos.stuck_guard import StuckGuard

    return autoconnect(
        _coordinator_blueprint(),
        VectorCamera.blueprint(width=640, height=480, fps=15,
                               enable_depth=True, enable_pointcloud=False,
                               enable_imu=True, imu_hz=200,
                               base_transform=CAMERA_MOUNT),
        RPLidarC1.blueprint(),
        ReSpeakerMic.blueprint(stt_language="fr"),   # auto-detect mangles short utterances
        LidarOdometry.blueprint(use_gyro_prior=False),
        # carve_columns=False: this voxel map is the Rerun visual only (navigation runs on costmap2d). True ('latest
        # observation wins') erased the camera's table tops on every lidar hit in the same column and the map lost its detail
        # (metrox, 23/08 22:30). Cost of False: a passer-by leaves a ghost trail, until a health-based mapper replaces this one.
        VoxelGridMapper.blueprint(voxel_size=0.05, device="CPU:0", frame_id="world", emit_every=10, carve_columns=False),   # ~1 Hz: the Rerun bridge re-sends the whole map each time (load 25, viewer 2.5 GB at 3 Hz)
        # dimOS's height-cost defaults are for a quadruped (can_climb 0.15 m, pass
        # under 0.6 m): VECTOR climbs nothing (3 cm) and its camera top is ~0.85 m.
        VectorCostMap.blueprint(),   # our 2D map (learns/unlearns, two layers) - dimOS's CostMapper erased table legs
        # the rover is 54x46 cm but its corners sweep 0.71 m when it pivots and the
        # camera mast stands at the front bumper: give the planner a footprint with margin
        # width 0.46 + 4 cm: in discovery mode the rover drives along its length and turns in place,
        # so its lateral clearance is what matters (metrox, 23/08: 'pas de crabe pendant qu'on mappe');
        # 0.62 (a disc for the corners) walled off every 60-70 cm gap - 11 'no path' from a 50 cm clearance
        # 62.5 x 46 cm with the bumper bars (metrox 25/08): real diagonal 0.776, so 0.78 is honest.
        # Measured 25/08 on the newest costmap checkpoint (20260825-221223/costmap_224656.npz, 19.4 m2
        # observed): 3084 observed free cells clear an inflation radius of 0.39 m, 3278 clear 0.36 m -
        # 0.78 costs 5.9 % of the goal-capable space, not the walling-off it was suspected of. Same
        # verdict on all 18 recorded runs (worst case 13.9 %), so the honest number stays.
        RecoveringPlanner.blueprint(robot_width=0.50, robot_rotation_diameter=0.78),
        MovementManager.blueprint(),
        VectorExplorer.blueprint(safe_distance=0.35, lookahead_distance=4.0, min_frontier_perimeter=0.3,
                                            # 1 % gain per goal made it quit once the first room was known while the
                                            # workshop was still unknown (23/08 18:36, after 29 m): 0.1 %, 6 tries
                                            info_gain_threshold=0.001, num_no_gain_attempts=6,
                                            max_explored_distance=12.0,
                                            # 15 s was sized for the stock 0.55 m/s follower: at the capped
                                            # 0.149 m/s a 3 m frontier needs ~25 s - every goal timed out mid-drive
                                            # (4 goals, 0 reached, 25/08 21:31)
                                            goal_timeout=45.0),
        StuckGuard.blueprint(),
        ImuSlipDetector.blueprint(),   # the body as witness: slip in 0.2-0.5 s, wheels in the air included
        # contact corners + sonar via the ESP32 USB bridge. Neither writes the map (sensor
        # doctrine, 25/08): front bump = back off, rear bump = move forward, sonar = the forward
        # brake in adapter.py (creep under 0.55 m, stop under 0.30 m).
        EspSensors.blueprint(),
        VectorMemory.blueprint(),    # every run recorded (dimOS memory): replay, draw, tune planners without the robot
        vis_module(viewer_backend=global_config.viewer, rerun_config=RERUN_CONFIG),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


explore_blueprint = _explore_blueprint()

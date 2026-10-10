"""dimOS blueprints for MOSS: mapping and exploration on the tracked rover.

Entry points (pyproject [project.entry-points."dimos.blueprints"]):

    dimos run vector-dimos.moss-nav       mapping + pad teleop, recording
    dimos run vector-dimos.moss-explore   mapping + planner + frontier
                                          exploration + pad teleop, recording

Both are VECTOR's modules (RPLidarC1 -> LidarOdometry -> VectorCostMap, the
coordinator, RecoveringPlanner, MovementManager, Explorer2) on MOSS's body:
importing this module sets ROBOT_PROFILE=moss (setdefault: an explicit value
wins) so every geometry hook reads robot_profile.MOSS, and PERSISTENT_MAP=0
(setdefault) so a MOSS run never relocalizes against, or saves over,
VECTOR's flat. VECTOR's blueprints never import this module. The env is set
in the CLI process before any worker starts, and workers inherit it;
tools/moss-fly.sh exports both as well.

Nothing here moves the robot by itself:
  * moss-nav has no planner and no explorer: the pad drives the coordinator
    directly (tele_cmd_vel -> cmd_vel), and only while its deadman is held.
  * moss-explore's explorer starts only on Bool(True) on `explore_cmd`
    (tools/explore_ctl.py start, the cockpit), and the planner moves only
    toward a goal on `goal_request` / `clicked_point`. MovementManager
    arbitrates: any pad input cancels navigation.
  * the adapter's 0.5 s deadman and the firmware's 300 ms deadman are
    untouched.

Devices (moss_ports): the QT Py and the C1 by their /dev/serial/by-id path,
MOSS_PORT / MOSS_LIDAR_PORT win. A missing device resolves to a sentinel
path that cannot open - logged as an error at build time and again by the
module that fails to open it - never to another device.

Switches (env, read when the blueprint is built):
    MOSS_CAMERA=0     no RealSense (default on: colour 640x480 @ 5 fps, JPEG on the bus)
    MOSS_RS_SERIAL    pick the RealSense by serial number
    MOSS_RECORD=0     no recording (default on: MossRecorder, <MOSS_STATE_DIR>/recordings/moss.db)
    MOSS_RERUN=1      add the stock Rerun/websocket viewers (default off: no 7779/9877/3030 on MOSS)

This module imports without dimOS (moss_ports and the pure helpers are used by
the benches and tools); the blueprint objects are built on first access.
"""
from __future__ import annotations

import math
import os

os.environ.setdefault("ROBOT_PROFILE", "moss")
os.environ.setdefault("PERSISTENT_MAP", "0")

from vector_dimos import robot_profile  # noqa: E402
from vector_dimos.robot_profile import (MOSS_LIDAR_MISSING, MOSS_QTPY_MISSING,  # noqa: E402
                                        RobotProfile)

LIDAR_BAUDRATE = 460800
CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS = 640, 480, 5
EXPLORER_MIN_FRONTIER_PERIMETER_M = 0.3   # VECTOR's: doorway-sized frontiers stay visible
EXPLORER_GOAL_TIMEOUT_S = 45.0


def moss_ports(env: dict | None = None) -> dict:
    """{"base": QT Py port, "lidar": C1 port}: env first, then the by-id glob
    under robot_profile.BYID_ROOT, then the sentinel.

    Raises ValueError when both resolve to the same device (realpath): the
    base adapter would open the lidar's tty at 115200, retune the line under
    RPLidarC1 (460800) and write its 'status' probe into the C1. The usual
    cause is a stale MOSS_PORT pointing at the CP2102N."""
    ports = {"base": robot_profile.moss_qtpy_port(env),
             "lidar": robot_profile.moss_lidar_port(env)}
    if (ports["base"] != MOSS_QTPY_MISSING and ports["lidar"] != MOSS_LIDAR_MISSING
            and os.path.realpath(ports["base"]) == os.path.realpath(ports["lidar"])):
        raise ValueError(
            f"MOSS: the base port and the lidar port are the same device "
            f"({ports['base']} -> {os.path.realpath(ports['base'])}); the motor board is the "
            "QT Py (*QT_Py*), the lidar the CP2102N - check MOSS_PORT / MOSS_LIDAR_PORT")
    return ports


def _env_on(name: str, default: bool, env: dict | None = None) -> bool:
    env = os.environ if env is None else env
    raw = env.get(name, "").strip().lower()
    if not raw:
        return default
    return raw not in ("0", "false", "no", "off")


def camera_mount_xyzq(profile: RobotProfile) -> tuple[float, ...]:
    """base_link -> camera_link for the RealSense module, as
    (x, y, z, qx, qy, qz, qw): the mechanical pose only (dimOS adds the
    optical rotation itself), a pure pitch about +y, positive = nose down
    (nav_blueprints.camera_mount's convention)."""
    x, y, z = profile.camera_xyz_base_m
    half = math.radians(profile.camera_pitch_deg) / 2.0
    return (x, y, z, 0.0, math.sin(half), 0.0, math.cos(half))


def _logger():
    try:
        from dimos.utils.logging_config import setup_logger
        return setup_logger()
    except Exception:  # noqa: BLE001 - no dimOS: the stdlib one
        import logging
        return logging.getLogger("vector_dimos.moss_blueprints")


def _announce(ports: dict, profile: RobotProfile) -> None:
    """The build-time lines: which devices, which body - and loudly when a
    device is missing."""
    log = _logger()
    if profile.name != "moss":
        log.warning(f"MOSS blueprint built with ROBOT_PROFILE={profile.name}: "
                    "the geometry is NOT MOSS's")
    if ports["base"] == MOSS_QTPY_MISSING:
        log.error("MOSS: no QT Py under /dev/serial/by-id (*QT_Py*) and MOSS_PORT unset - "
                  f"the base gets {MOSS_QTPY_MISSING} and will not connect")
    if ports["lidar"] == MOSS_LIDAR_MISSING:
        log.error("MOSS: no CP2102N under /dev/serial/by-id (*CP2102N*) and MOSS_LIDAR_PORT unset - "
                  f"the lidar gets {MOSS_LIDAR_MISSING} and will publish nothing")
    log.info(f"MOSS blueprint: base {ports['base']}, lidar {ports['lidar']}, profile {profile}")


def _common_parts(ports: dict, profile: RobotProfile) -> list:
    """Base, pad, lidar, odometry, costmap, camera, recorder, viewer: the
    modules both MOSS blueprints share."""
    from vector_dimos.blueprints import _coordinator_blueprint, _moss_base
    from vector_dimos.costmap2d import VectorCostMap
    from vector_dimos.gamepad import GamepadTeleop
    from vector_dimos.lidar_odometry import LidarOdometry
    from vector_dimos.moss_adapter import MossGeometry, teleop_envelope, teleop_speeds
    from vector_dimos.rplidar_c1 import RPLidarC1

    g = MossGeometry.from_env()
    linear, angular = teleop_speeds(g)
    parts = [
        _coordinator_blueprint(_moss_base(address=ports["base"])),
        # stick scale and clamp = the base's envelope, as moss-gamepad
        GamepadTeleop.blueprint(linear_speed=linear, angular_speed=angular,
                                envelope=teleop_envelope(g)),
        RPLidarC1.blueprint(port=ports["lidar"], baudrate=LIDAR_BAUDRATE, frame_id="lidar_link"),
        # no IMU wired on MOSS tonight: kiss-icp on its constant-velocity
        # prior (the D455F gyro as on VECTOR is a later step)
        LidarOdometry.blueprint(use_gyro_prior=False),
        VectorCostMap.blueprint(),
    ]
    if _env_on("MOSS_CAMERA", True):
        from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
        from dimos.msgs.geometry_msgs.Quaternion import Quaternion
        from dimos.msgs.geometry_msgs.Transform import Transform
        from dimos.msgs.geometry_msgs.Vector3 import Vector3

        x, y, z, qx, qy, qz, qw = camera_mount_xyzq(profile)
        cam_kwargs = dict(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS,
                          enable_depth=False, enable_pointcloud=False, enable_imu=False,
                          base_transform=Transform(translation=Vector3(x, y, z),
                                                   rotation=Quaternion(qx, qy, qz, qw)))
        serial = os.environ.get("MOSS_RS_SERIAL", "").strip()
        if serial:
            cam_kwargs["serial_number"] = serial
        parts.append(RealSenseCamera.blueprint(**cam_kwargs))
    if _env_on("MOSS_RECORD", True):
        from vector_dimos.moss_memory import MossRecorder
        parts.append(MossRecorder.blueprint(db_path=str(robot_profile.moss_recording_path())))
    if _env_on("MOSS_RERUN", False):
        from dimos.core.global_config import global_config
        from dimos.visualization.vis_module import vis_module
        parts.append(vis_module(viewer_backend=global_config.viewer))
    return parts


def _jpeg_camera(bp):
    """Colour frames cross the bus as JPEG (encoded once in the camera's
    process): the cockpit passes them through, the recorder stores JPEG."""
    if not _env_on("MOSS_CAMERA", True):
        return bp
    from dimos.core.transport import JpegLcmTransport
    from dimos.msgs.sensor_msgs.Image import Image
    return bp.transports({("color_image", Image): JpegLcmTransport.spec("/color_image", Image)})


def build_moss_nav():
    """Mapping + pad teleop. No planner, no explorer, no MovementManager: the
    pad drives the coordinator directly, like moss-gamepad."""
    from dimos.core.coordination.blueprints import autoconnect
    from vector_dimos.blueprints import VectorControlCoordinator
    from vector_dimos.gamepad import GamepadTeleop

    profile = robot_profile.current()
    ports = moss_ports()
    _announce(ports, profile)
    return _jpeg_camera(autoconnect(*_common_parts(ports, profile))).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
        (GamepadTeleop, "tele_cmd_vel", "cmd_vel"),
    ])


def build_moss_explore():
    """moss-nav's modules plus the planner, the arbiter and the explorer. The
    pad stays on tele_cmd_vel: MovementManager lets it win and cancels any
    navigation. Exploration waits for `explore_cmd`."""
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.msgs.geometry_msgs.Twist import Twist
    from dimos.msgs.geometry_msgs.Vector3 import Vector3
    from dimos.navigation.movement_manager.movement_manager import MovementManager
    from vector_dimos.blueprints import VectorControlCoordinator
    from vector_dimos.explorer2 import Explorer2
    from vector_dimos.recovering_planner import RecoveringPlanner

    profile = robot_profile.current()
    ports = moss_ports()
    _announce(ports, profile)
    lethal, pivot = robot_profile.derived(profile)
    parts = _common_parts(ports, profile) + [
        RecoveringPlanner.blueprint(robot_width=profile.robot_width_m,
                                    robot_rotation_diameter=profile.pivot_diameter_m),
        # pass-through scaling: the pad's own envelope is the guard
        MovementManager.blueprint(
            tele_cmd_vel_scaling=Twist(Vector3(1.0, 1.0, 0.0), Vector3(0.0, 0.0, 1.0))),
        Explorer2.blueprint(min_frontier_perimeter=EXPLORER_MIN_FRONTIER_PERIMETER_M,
                            goal_timeout=EXPLORER_GOAL_TIMEOUT_S,
                            lethal_clearance_m=lethal, pivot_clearance_m=pivot),
    ]
    return _jpeg_camera(autoconnect(*parts)).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
    ])


_BUILDERS = {"moss_nav_blueprint": build_moss_nav,
             "moss_explore_blueprint": build_moss_explore}


def __getattr__(name: str):
    """Entry points load `moss_nav_blueprint` / `moss_explore_blueprint` as
    attributes: build on first access (dimOS present), then cache."""
    builder = _BUILDERS.get(name)
    if builder is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    bp = builder()
    globals()[name] = bp
    return bp

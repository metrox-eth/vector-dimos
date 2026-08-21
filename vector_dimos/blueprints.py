"""dimOS blueprints for the VECTOR mecanum platform.

Entry points (pyproject [project.entry-points."dimos.blueprints"]):
    dimos run vector-dimos.base                    # coordinator + VECTOR base
    dimos run vector-dimos.gamepad                 # base + PS-style gamepad
    dimos run vector-dimos.rplidar                 # RPLIDAR C1 -> pointcloud

The 'vector' drive-train adapter is registered here at import time: dimOS's
internal drive_trains manifest scan cannot see external packages, so we use
the public registry escape hatch.
"""
from __future__ import annotations

from dimos.control.components import (HardwareComponent, HardwareType,
                                      make_twist_base_joints)
from dimos.control.coordinator import ControlCoordinator, TaskConfig
from dimos.core.coordination.blueprints import autoconnect
from dimos.hardware.drive_trains.registry import twist_base_adapter_registry

# Register the VECTOR adapter under its name (idempotent-guarded).
try:
    twist_base_adapter_registry.register_path(
        "vector", "vector_dimos.adapter:VectorBaseAdapter")
except Exception:  # already registered (e.g. re-import): keep the first one
    pass

_base_joints = make_twist_base_joints("base")


def _vector_base(hw_id: str = "base",
                 address: str | None = None) -> HardwareComponent:
    """VECTOR mecanum base (3-DOF: vx, vy, wz) - 2x ZLAC8015D over RS485."""
    return HardwareComponent(
        hardware_id=hw_id,
        hardware_type=HardwareType.BASE,
        joints=make_twist_base_joints(hw_id),
        adapter_type="vector",
        address=address,   # RS485 serial port; adapter default: /dev/ttyTHS1
    )


base_blueprint = ControlCoordinator.blueprint(
    hardware=[_vector_base()],
    tasks=[
        TaskConfig(name="vel_base", type="velocity",
                   joint_names=_base_joints, priority=10),
    ],
).remappings([(ControlCoordinator, "twist_command", "cmd_vel")])


def _gamepad_blueprint():
    from vector_dimos.gamepad import GamepadTeleop

    return autoconnect(
        ControlCoordinator.blueprint(
            hardware=[_vector_base()],
            tasks=[
                TaskConfig(name="vel_base", type="velocity",
                           joint_names=_base_joints, priority=10),
            ],
        ),
        GamepadTeleop.blueprint(),
    ).remappings([
        (ControlCoordinator, "twist_command", "cmd_vel"),
        # direct drive: the gamepad IS the commander in this blueprint
        (GamepadTeleop, "tele_cmd_vel", "cmd_vel"),
    ])


gamepad_blueprint = _gamepad_blueprint()


def _rplidar_blueprint():
    from vector_dimos.rplidar_c1 import RPLidarC1

    return RPLidarC1.blueprint()


rplidar_blueprint = _rplidar_blueprint()

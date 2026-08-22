"""dimOS blueprints for the VECTOR mecanum platform.

Entry points (pyproject [project.entry-points."dimos.blueprints"]):
    dimos run vector-dimos.base                    # coordinator + VECTOR base
    dimos run vector-dimos.gamepad                 # base + PS-style gamepad
    dimos run vector-dimos.rplidar                 # RPLIDAR C1 -> pointcloud

Two things have to happen for an out-of-tree drive train to work, and the
second one is not obvious:

1. The 'vector' adapter name must be registered. dimOS's drive_trains
   manifest scan only walks its own package, so external packages use the
   public registry escape hatch (register_path, below).

2. That registration must also happen in the process that BUILDS the
   adapter. dimOS deploys every module into a `forkserver` worker
   (dimos/core/coordination/python_worker.py), and a forkserver worker is a
   fresh interpreter: it re-imports dimos, gets a brand-new
   twist_base_adapter_registry, and never imports this module. Registering
   in the CLI parent alone is not enough - ControlCoordinator._setup_hardware
   runs in the worker and dies with:

       KeyError: "Unknown twist base adapter: vector.
                  Available: ['flowbase', 'mock_twist_base', ...]"

   The worker does import whatever it needs to UNPICKLE the module class it
   was told to deploy, and classes pickle by reference. So the coordinator we
   hand to dimOS is our own subclass, defined here: unpickling
   `vector_dimos.blueprints:VectorControlCoordinator` in the worker imports
   this module, which runs the registration above it. That mechanism holds
   for fork, forkserver and spawn alike, unlike a forkserver preload hack.

   Subclassing the coordinator is dimOS's own documented pattern for this
   (see dimos/robot/manipulators/common/blueprints.py:coordinator(cls=...));
   it requires instance_name="ControlCoordinator" so the shipped RPC clients
   still find the coordinator by the name they expect, and ControlCoordinator
   .start() warns if a subclass omits it.
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

# The shipped dimOS RPC clients address the coordinator by this name; keep it
# even though the class below is a subclass.
COORDINATOR_INSTANCE_NAME = "ControlCoordinator"

_base_joints = make_twist_base_joints("base")


class VectorControlCoordinator(ControlCoordinator):
    """Stock ControlCoordinator, deployed under this package's import path.

    No behaviour is added or changed. Its only job is to make the forkserver
    worker import vector_dimos.blueprints (see this module's docstring), which
    is what registers the 'vector' twist base adapter in the process that
    actually instantiates it.
    """


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


def _coordinator_blueprint():
    """Coordinator driving one VECTOR base at velocity."""
    return VectorControlCoordinator.blueprint(
        hardware=[_vector_base()],
        tasks=[
            TaskConfig(name="vel_base", type="velocity",
                       joint_names=_base_joints, priority=10),
        ],
        instance_name=COORDINATOR_INSTANCE_NAME,
    )


base_blueprint = _coordinator_blueprint().remappings(
    [(VectorControlCoordinator, "twist_command", "cmd_vel")])


def _gamepad_blueprint():
    from vector_dimos.gamepad import GamepadTeleop

    return autoconnect(
        _coordinator_blueprint(),
        GamepadTeleop.blueprint(),
    ).remappings([
        (VectorControlCoordinator, "twist_command", "cmd_vel"),
        # direct drive: the gamepad IS the commander in this blueprint
        (GamepadTeleop, "tele_cmd_vel", "cmd_vel"),
    ])


gamepad_blueprint = _gamepad_blueprint()


def _rplidar_blueprint():
    from vector_dimos.rplidar_c1 import RPLidarC1

    return RPLidarC1.blueprint()


rplidar_blueprint = _rplidar_blueprint()

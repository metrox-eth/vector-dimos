"""Cold bench: VectorCamera.stop stays an @rpc, so dimOS can stop the camera.

Module.rpcs (dimos/core/module.py) only collects callables carrying __rpc__.
An override that drops the decorator takes the parent's entry off the table:
RPCClient.__getattr__ then stops issuing an RpcCall and falls back to a raw
attribute fetch, whose answer is a bound method the worker cannot pickle
('cannot pickle _thread.lock', from the camera's _pointcloud_lock). The worker
dies on shutdown, RealSenseCamera.stop never runs, the device stays streaming
and the next run cannot enumerate it.

Three sections, no camera needed:
  A. the class's rpc table, against the parent and the two siblings that
     already guard the same footgun (rplidar_c1, wheel_odom).
  B. dimOS's own resolver: RPCClient.__getattr__ -> RpcCall, not a getattr.
  C. what the missing entry costs: the bound method is unpicklable, while the
     table entry really releases the motion sensor.
"""
import pickle
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dimos.core.rpc_client import RPCClient, RpcCall
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera

from vector_dimos.camera import VectorCamera
from vector_dimos.rplidar_c1 import RPLidarC1
from vector_dimos.wheel_odom import VectorWheelOdometry

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and bool(cond)


# --- A. the rpc table ------------------------------------------------------
print("A. Module.rpcs")
cam_rpcs = set(VectorCamera.rpcs)
check("stop" in cam_rpcs, "VectorCamera.rpcs carries 'stop'")
check("start" in cam_rpcs, "VectorCamera.rpcs carries 'start'")

parent_rpcs = set(RealSenseCamera.rpcs)
check("stop" in parent_rpcs, "the parent RealSenseCamera exposes 'stop'")
check(not (parent_rpcs - cam_rpcs),
      f"the subclass drops none of the parent's rpcs (missing: "
      f"{sorted(parent_rpcs - cam_rpcs)})")

for sibling in (RPLidarC1, VectorWheelOdometry):
    names = set(sibling.rpcs)
    check({"start", "stop"} <= names,
          f"sibling {sibling.__name__} keeps start+stop (the same guard)")


# --- B. dimOS's own resolver ----------------------------------------------
print("\nB. RPCClient.__getattr__ (the path ModuleCoordinator.stop takes)")


class StubRpc:
    """Stands in for the RPCSpec backend: the resolver only stores it."""


def resolve(cls, name):
    # remote mode (actor_instance=None): an rpc resolves to an RpcCall,
    # anything else raises instead of falling back to attribute access.
    client = RPCClient(None, cls, rpc=StubRpc())
    try:
        return getattr(client, name)
    except AttributeError as error:
        return error


for cls in (VectorCamera, RPLidarC1, VectorWheelOdometry):
    call = resolve(cls, "stop")
    check(isinstance(call, RpcCall) and call.rpc_name == "stop"
          and call.remote_name == cls.__name__,
          f"{cls.__name__}.stop resolves to RpcCall(stop) on {cls.__name__}")

missing = resolve(VectorCamera, "not_an_rpc")
check(isinstance(missing, AttributeError),
      "a name outside the table still raises - the resolver discriminates")


# --- C. what the missing entry costs --------------------------------------
print("\nC. the fallback the decorator avoids")
cam = VectorCamera()

try:
    pickle.dumps(cam.stop)
    shipped = None
except Exception as error:  # noqa: BLE001
    shipped = error
check(isinstance(shipped, TypeError) and "pickle" in str(shipped),
      f"the bound method a GetAttrRequest would ship is unpicklable "
      f"({type(shipped).__name__}: {shipped})")


class FakeMotionSensor:
    def __init__(self):
        self.calls = []

    def stop(self):
        self.calls.append("stop")

    def close(self):
        self.calls.append("close")


sensor = FakeMotionSensor()
cam._motion_sensor = sensor
# call it exactly as dispatched: off the table, not off the instance
dispatched = VectorCamera.rpcs.get("stop")
if dispatched is not None:
    dispatched(cam)
check(sensor.calls == ["stop", "close"],
      f"the table's 'stop' releases the motion sensor (calls: {sensor.calls})")
check(cam._motion_sensor is None, "and clears the handle")

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

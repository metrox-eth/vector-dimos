"""VECTOR's spatial memory: every exploration run recorded to SQLite.

dimOS's Recorder (their "memory") stores the In ports below with a pose per
observation; afterwards the run is replayed without the robot - draw the
map, try a costmap function or a planner on it, compare (docs/capabilities/
memory, upstream pointer 23/08: "render how it would path on top of your map
without running a robot"). Also the answer to "when it crashes we
lose everything": the recording survives the crash.

Streams: the lidar world cloud (lidar returns + camera obstacles, from
lidar_odometry), the camera floor samples, the odometry and our costmap.
No colour image: 15 fps x 640x480 is ~45 MB/min; the lidar is ~4 MB/min.
"""

from __future__ import annotations

from pathlib import Path

from dimos.core.stream import In
from dimos.memory.module import Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class VectorMemoryConfig(RecorderConfig):
    db_path: str | Path = Path("~/.local/state/vector/recordings/explore.db").expanduser()
    # world-frame streams need no pose anchoring; the odom stream sets the pose
    poseless_streams: list[str] = ["global_costmap"]
    # tf at 23 Hz was recorded by default as the POSE FALLBACK for streams
    # without a setter - every one of our streams has a setter (below) or is
    # poseless, so this was ~23 sqlite commits/s bought for nothing (measured
    # 27/08: the recorder ate a full core, one commit per observation).
    record_tf: bool = False


class VectorMemoryLight(Recorder):
    """Teleop-phase recording: trajectory + decision map, nothing else.
    The raw clouds (lidar ~10 Hz, camera_floor) are the WRITE VOLUME of the
    recorder; run_autopsy reads only odom + costmap (+ main.jsonl). Chosen by
    RECORD_CLOUDS=0 in the blueprint - full replay recording stays the
    default."""
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    config: VectorMemoryConfig

    _last_odom_pose: Pose | None = None

    @pose_setter_for("odom")
    async def _odom_pose(self, msg: PoseStamped) -> Pose | None:
        self._last_odom_pose = Pose(position=msg.position, orientation=msg.orientation)
        return self._last_odom_pose


class VectorMemory(VectorMemoryLight):
    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]

    @pose_setter_for("lidar", "camera_floor")
    async def _world_cloud_pose(self, msg: PointCloud2) -> Pose | None:
        # clouds are already in the world frame; anchor them at the rover's pose
        # so memory queries ("what did it see from here") still work
        return self._last_odom_pose

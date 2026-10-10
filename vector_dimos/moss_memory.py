"""MOSS's run recording: what the cockpit shows, written to SQLite.

Same machinery as VECTOR's (vector_dimos.memory: dimOS's Recorder plus the
WAL-trio rotation), MOSS's own streams and its own state dir:

    odom             PoseStamped    the pose source of every anchored stream
    global_costmap   OccupancyGrid  world frame, poseless
    lidar            PointCloud2    world-frame cloud from lidar_odometry, at the last odom pose
    color_image      Image          D455F colour (JPEG on the bus and in the db), at the last odom pose
    cmd_vel          Twist          what the base was told, poseless
    base_telemetry   String         firmware JSON (battery, mode, track outputs), poseless
    episode_marker   String         {action, name, ts} from the cockpit, poseless

The db path is absolute (robot_profile.moss_recording_path, default
~/.local/state/moss/recordings/moss.db, MOSS_STATE_DIR moves the whole
tree): a relative path would land in dimOS's site-packages. The previous
run is rotated aside with its -wal/-shm before a new one opens
(VectorMemoryLight.start -> rotate_recording).
"""
from __future__ import annotations

from pathlib import Path

from pydantic import Field

from dimos.core.stream import In
from dimos.memory.module import pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.String import String

from vector_dimos.memory import VectorMemoryConfig, VectorMemoryLight
from vector_dimos.robot_profile import moss_recording_path

MOSS_POSELESS_STREAMS = ["global_costmap", "cmd_vel", "base_telemetry", "episode_marker"]


class MossMemoryConfig(VectorMemoryConfig):
    # a factory, not a value: MOSS_STATE_DIR is read when the module is built
    db_path: str | Path = Field(default_factory=moss_recording_path)
    poseless_streams: list[str] = Field(default_factory=lambda: list(MOSS_POSELESS_STREAMS))
    record_tf: bool = False


class MossRecorder(VectorMemoryLight):
    """odom + global_costmap (inherited, with the odom pose setter and the
    WAL-safe rotation in start()), plus the MOSS streams."""

    lidar: In[PointCloud2]
    color_image: In[Image]
    cmd_vel: In[Twist]
    base_telemetry: In[String]
    episode_marker: In[String]
    config: MossMemoryConfig

    @pose_setter_for("lidar", "color_image")
    async def _anchor_on_odom(self, msg) -> Pose | None:
        # the cloud is already in the world frame and the image has no tf
        # chain on MOSS: anchor both at the rover's last pose, so memory
        # queries ("what did it see from here") still work
        return self._last_odom_pose

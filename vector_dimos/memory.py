"""VECTOR's spatial memory: every exploration run recorded to SQLite.

dimOS's Recorder (their "memory") stores the In ports below with a pose per
observation; afterwards the run is replayed without the robot - draw the
map, try a costmap function or a planner on it, compare (docs/capabilities/
memory, upstream pointer 2026-08-23: "render how it would path on top of your
map without running a robot"). Also the answer to "when it crashes we lose
everything": the recording survives the crash.

Streams: the lidar world cloud (lidar returns + camera obstacles, from
lidar_odometry), the camera floor samples, the odometry and our costmap.
No colour image: 15 fps x 640x480 is ~45 MB/min; the lidar is ~4 MB/min.

That promise only holds if the rotation of the previous recording keeps the
whole SQLite WAL trio together - see rotate_recording() below.
"""

from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory.module import OnExisting, Recorder, RecorderConfig, pose_setter_for
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# a WAL recording is three files: explore.db + these two
WAL_SIDECARS = ("-wal", "-shm")


def _sidecars(db: Path) -> list[Path]:
    """The -wal/-shm of *db* that exist right now."""
    return [p for p in (db.with_name(db.name + s) for s in WAL_SIDECARS) if p.exists()]


def _rotated(db: Path, stamp: str) -> Path:
    return db.with_name(f"{db.stem}.{stamp}{db.suffix}")


def _free_stamp(db: Path) -> str:
    """A stamp no rotated file carries yet: a rotation never overwrites."""
    now = stamp = datetime.now().strftime("%Y%m%d%H%M%S")
    n = 0
    while True:
        taken = _rotated(db, stamp)
        if not any(taken.with_name(taken.name + s).exists() for s in ("", *WAL_SIDECARS)):
            return stamp
        n += 1
        stamp = f"{now}-{n}"


def quarantine_orphans(db: Path) -> Path | None:
    """Park a -wal/-shm left with no db beside it, dated, never deleted.

    Whatever the killed run committed can live in that -wal alone, and it takes
    its own db to read it back: moving it out of the way is all we may do. The
    next run then opens a fresh db with nothing stale beside it - opening one on
    top of an orphaned -wal rewrites the -wal, which is the loss itself.
    """
    orphans = _sidecars(db)
    if db.exists() or not orphans:
        return None
    stamp = datetime.now().strftime("%Y%m%d%H%M%S")
    dest = db.parent / "quarantine" / stamp
    n = 0
    while dest.exists():
        n += 1
        dest = db.parent / "quarantine" / f"{stamp}-{n}"
    dest.mkdir(parents=True)
    for p in orphans:
        p.rename(dest / p.name)
    logger.warning("Orphaned %s with no %s -> quarantined in %s",
                   [p.name for p in orphans], db.name, dest)
    return dest


def _prune(db: Path, keep_last: int) -> None:
    """backup_keep_last, applied to whole trios - dimOS's prune unlinks the .db
    and leaves its -wal behind to shadow a later recording. keep_last=0 means
    'delete the previous recording' upstream; here it prunes nothing, a
    recording is never destroyed to save room."""
    if keep_last <= 0:
        return
    pattern = re.compile(rf"^{re.escape(db.stem)}\.(\d{{14}})(?:-(\d+))?{re.escape(db.suffix)}$")
    found = [(m, p) for m, p in
             ((pattern.match(p.name), p) for p in db.parent.glob(f"{db.stem}.*{db.suffix}"))
             if m]
    # oldest first, by stamp then by same-second rank: sorting the names would
    # put "<stamp>-2.db" before "<stamp>.db" and retire the newest recording
    olds = [p for _, p in sorted(found, key=lambda f: (f[0].group(1), int(f[0].group(2) or 0)))]
    for old in olds[:-keep_last]:
        for p in (old, *_sidecars(old)):
            p.unlink()


def rotate_recording(db_path: str | Path, keep_last: int = 10) -> Path | None:
    """Move the previous recording aside - the WAL trio, not the db alone.

    A recording is explore.db + explore.db-wal + explore.db-shm, and a run
    killed with SIGKILL (`dimos stop` escalates to it, see tools/fly.sh) leaves
    its last commits in the -wal. dimOS's backup_file() renames the .db alone:
    the backup then opens with an empty schema and the next run's fresh db
    writes over the -wal, so the crashed run - the one worth autopsying - is
    exactly the one destroyed. Rename the three together instead.

    Returns the rotated .db, or None when there was no recording to rotate.
    """
    db = Path(db_path)
    db.parent.mkdir(parents=True, exist_ok=True)
    if not db.exists():
        quarantine_orphans(db)
        return None

    rotated = _rotated(db, _free_stamp(db))
    # the db moves first: interrupted mid-rotation, the sidecars are quarantined
    # by the next run instead of being adopted by a fresh recording
    for src in (db, *_sidecars(db)):
        src.rename(rotated.with_name(rotated.name + src.name[len(db.name):]))
    logger.info("Rotated recording %s -> %s (WAL trio)", db, rotated)
    quarantine_orphans(db)
    _prune(db, keep_last)
    return rotated


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

    # dimOS rotates in Recorder.start(), with backup_file(): the .db alone.
    # Rotate here first, trio and all - Recorder then finds no db and rotates
    # nothing. Only BACKUP is ours; APPEND/OVERWRITE/ERROR stay upstream's.
    @rpc
    def start(self) -> None:
        if not self.config.g.replay and self.config.on_existing is OnExisting.BACKUP:
            rotate_recording(self.config.db_path, keep_last=self.config.backup_keep_last)
        super().start()

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

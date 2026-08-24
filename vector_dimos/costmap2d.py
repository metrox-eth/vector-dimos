"""A 2D costmap for a wheeled rover that learns AND unlearns.

dimOS's CostMapper turns the 3D voxel map (a set that never forgets) into a
terrain-slope map meant for a walking robot; measured on 23/08 with
tools/mars/stages.py it erased table legs (erosion drops any cell whose four
neighbours are unobserved). metrox's spec, from living with a Xiaomi vacuum:

  * the map must REINFORCE - but with a ceiling, or a ramp it struggled on
    becomes a wall after a day;
  * it must UNLEARN what it sees absent again - the telescope under the desk
    was gone after run 1 and still avoided at run 10;
  * reinforcement needs new viewpoints: from one spot a thing gets no more
    certain than "occupied" (two misses from gone); a table leg seen from
    twenty positions is solid (up to ten misses);
  * two layers, because the sensors do not see the same things: what the
    lidar put down a lidar ray may clear; what the camera or a bump put down
    (a low box, under the 0.37 m scan plane) only the camera seeing the floor
    there may clear.

Numbers (HIT_CAP, FREE_FLOOR, OCCUPIED_AT) are a starting point to be tested.
"""

from __future__ import annotations

import math
import os
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos_lcm.std_msgs import Bool
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RESOLUTION_M = 0.05
GRID_SPAN_M = 24.0           # world-fixed square, centred on the start pose
HIT_CAP = 10                 # ceiling: no cell gets more certain than this
FREE_FLOOR = -3              # floor: no cell gets more "free" than this
OCCUPIED_AT = 2              # two hits from two places = an obstacle
NEW_VIEWPOINT_M = 0.10       # a hit counts only if the rover moved this much since the cell's last hit
LIDAR_Z_M = 0.37             # points at exactly this height come from the lidar (lidar_odometry.LIDAR_HEIGHT_M)
PUBLISH_EVERY = 5            # lidar revolutions between two costmap publications (10 Hz -> 2 Hz)
RAY_MAX_M = 4.0              # rays are carved (misses) up to this range only: 12 m x 0.025 m steps cost 227 ms per revolution (2.3 cores at 10 Hz, 23/08 20:20)
RAY_EVERY = 2                # carve on every other revolution; hits are taken on all
ROLLBACK_WINDOW_S = 2.5      # writes stay undoable this long: on a slip the last ~2 s already polluted the map before the guard could fire (metrox, 24/08: 'retroactif a une ou deux secondes')
CHECKPOINT_EVERY_S = 30.0    # metrox 23/08: 'when it crashes we lose everything - resume from a minute before'
CHECKPOINT_KEEP = 40         # 20 minutes of history
CHECKPOINT_DIR = os.path.expanduser("~/.local/state/vector/checkpoints")
SLIP_ROLLBACK_S = 2.0        # how far back a slip erases


class ScoredGrid:
    """Two int8 score layers over a fixed world grid, plus where each cell was last hit from."""

    def __init__(self, resolution: float = RESOLUTION_M, span_m: float = GRID_SPAN_M,
                 centre: tuple[float, float] = (0.0, 0.0)) -> None:
        self.res = resolution
        self.n = int(round(span_m / resolution))
        self.ox = centre[0] - span_m / 2.0
        self.oy = centre[1] - span_m / 2.0
        self.lidar = np.zeros((self.n, self.n), dtype=np.int8)     # what the lidar saw at 0.37 m
        self.low = np.zeros((self.n, self.n), dtype=np.int8)       # what the camera / a bump saw below the scan plane
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self._last_hit_xy = np.full((self.n, self.n, 2), np.nan, dtype=np.float32)
        # undo journal: (monotonic time, layer, flat indices, applied deltas, seen-was-new mask)
        self._journal: list[tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []

    # ---- coordinates -------------------------------------------------------
    def cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gx = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        gy = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        ok = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        return gx[ok], gy[ok]

    # ---- learning ----------------------------------------------------------
    def _hit(self, layer: np.ndarray, xs: np.ndarray, ys: np.ndarray, from_xy: tuple[float, float]) -> int:
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return 0
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        last = self._last_hit_xy[gy, gx]
        moved = np.isnan(last[:, 0]) | (np.hypot(last[:, 0] - from_xy[0], last[:, 1] - from_xy[1]) >= NEW_VIEWPOINT_M)
        cur = layer[gy, gx].astype(np.int16)
        # From one viewpoint a thing may become "occupied" (it IS seen) but no
        # more certain than that; only new viewpoints reinforce beyond. So a
        # parked rover sees its obstacles at once, and a false positive
        # repeated from the same spot stays two misses away from gone.
        cap = np.where(moved, HIT_CAP, OCCUPIED_AT)
        new = np.minimum(cur + 1, cap).astype(np.int8)
        self._journal.append((time.monotonic(), layer, gy * self.n + gx,
                              (new - cur).astype(np.int8), ~self.seen[gy, gx]))
        layer[gy, gx] = new
        self._last_hit_xy[gy[moved], gx[moved]] = from_xy
        self.seen[gy, gx] = True
        return int(moved.sum())

    def _miss(self, layer: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> None:
        if len(gx) == 0:
            return
        cur = layer[gy, gx].astype(np.int16)
        new = np.maximum(cur - 1, FREE_FLOOR).astype(np.int8)
        self._journal.append((time.monotonic(), layer, gy * self.n + gx,
                              (new - cur).astype(np.int8), ~self.seen[gy, gx]))
        layer[gy, gx] = new
        self.seen[gy, gx] = True

    _revs: int = 0

    def lidar_revolution(self, hits_xy: np.ndarray, from_xy: tuple[float, float]) -> None:
        """World-frame lidar hits of one revolution, seen from `from_xy`:
        the cells along each ray (up to just short of the hit, RAY_MAX_M at
        most, every RAY_EVERY-th revolution) take a miss, the hit cells a hit.
        Lidar layer only."""
        if len(hits_xy) == 0:
            return
        self._revs += 1
        if self._revs % RAY_EVERY == 0:
            cells = self._ray_cells(from_xy, hits_xy)
            if cells is not None:
                self._miss(self.lidar, cells[0], cells[1])
        self._hit(self.lidar, hits_xy[:, 0], hits_xy[:, 1], from_xy)

    def camera_obstacles(self, pts_xy: np.ndarray, from_xy: tuple[float, float]) -> None:
        self._hit(self.low, pts_xy[:, 0], pts_xy[:, 1], from_xy)

    def camera_floor(self, pts_xy: np.ndarray) -> None:
        """The camera saw bare floor here: a miss on BOTH layers (the only way a
        low object is ever forgotten)."""
        gx, gy = self.cell(pts_xy[:, 0], pts_xy[:, 1])
        if len(gx) == 0:
            return
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        self._miss(self.low, gx, gy)
        self._miss(self.lidar, gx, gy)

    def _ray_cells(self, from_xy: tuple[float, float], hits_xy: np.ndarray):
        """Cells crossed by the rays from `from_xy` to each hit (capped at
        RAY_MAX_M), stopping one cell short of the hit; one sample per cell."""
        dx, dy = hits_xy[:, 0] - from_xy[0], hits_xy[:, 1] - from_xy[1]
        r = np.hypot(dx, dy)
        keep = r > self.res
        if not keep.any():
            return None
        dx, dy, r = dx[keep], dy[keep], r[keep]
        end = np.minimum(r - self.res, RAY_MAX_M)
        n_steps = np.floor(end / self.res).astype(int)
        nmax = int(n_steps.max())
        if nmax < 1:
            return None
        # (rays x steps) sample grid in one shot, masked past each ray's end
        d = (np.arange(1, nmax + 1) * self.res)[None, :]
        ux, uy = (dx / r)[:, None], (dy / r)[:, None]
        valid = d <= end[:, None]
        xs = (from_xy[0] + ux * d)[valid]
        ys = (from_xy[1] + uy * d)[valid]
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return None
        flat = np.unique(gy * self.n + gx)
        return flat % self.n, flat // self.n

    def prune_journal(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        cutoff = now - ROLLBACK_WINDOW_S
        while self._journal and self._journal[0][0] < cutoff:
            self._journal.pop(0)

    def rollback(self, seconds: float, now: float | None = None) -> int:
        """Undo every write of the last `seconds` (metrox's retroactive freeze:
        by the time the slip guard fires, the sliding pose has already written
        ~1-2 s of garbage into the map). Deltas are the values actually
        applied, so the undo is exact even across the score clamps. Returns
        the number of batches undone."""
        now = time.monotonic() if now is None else now
        cutoff = now - seconds
        undone = 0
        while self._journal and self._journal[-1][0] >= cutoff:
            _, layer, flat, delta, seen_was_new = self._journal.pop()
            gy, gx = flat // self.n, flat % self.n
            layer[gy, gx] = (layer[gy, gx].astype(np.int16) - delta).astype(np.int8)
            self.seen[gy[seen_was_new], gx[seen_was_new]] = False
            undone += 1
        return undone

    # ---- checkpoints -------------------------------------------------------
    def save(self, path: str, pose_xy: tuple[float, float] | None = None) -> int:
        """Full state to a compressed .npz; returns the file size in bytes."""
        np.savez_compressed(path, lidar=self.lidar, low=self.low, seen=self.seen, last_hit_xy=self._last_hit_xy,
                            res=self.res, ox=self.ox, oy=self.oy, n=self.n,
                            pose_xy=np.array(pose_xy if pose_xy else (np.nan, np.nan)), ts=time.time())
        return os.path.getsize(path)

    @classmethod
    def load(cls, path: str) -> "ScoredGrid":
        z = np.load(path)
        g = cls.__new__(cls)
        g.res, g.n, g.ox, g.oy = float(z["res"]), int(z["n"]), float(z["ox"]), float(z["oy"])
        g.lidar, g.low, g.seen, g._last_hit_xy = z["lidar"], z["low"], z["seen"], z["last_hit_xy"]
        g._revs = 0
        return g

    # ---- output ------------------------------------------------------------
    def occupancy(self) -> np.ndarray:
        """int8 grid: 100 occupied, 0 free (observed), -1 unknown."""
        score = np.maximum(self.lidar, self.low)
        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[self.seen] = 0
        out[score >= OCCUPIED_AT] = 100
        return out

    def value_at(self, x: float, y: float) -> int:
        gx, gy = self.cell(np.array([x]), np.array([y]))
        return int(self.occupancy()[gy[0], gx[0]]) if len(gx) else -1

    def cropped(self, margin_cells: int = 20) -> tuple[np.ndarray, float, float] | None:
        """The occupancy grid cropped to the observed area (+ margin): (grid, origin_x, origin_y)."""
        if not self.seen.any():
            return None
        ys, xs = np.nonzero(self.seen)
        y0, y1 = max(0, ys.min() - margin_cells), min(self.n, ys.max() + margin_cells + 1)
        x0, x1 = max(0, xs.min() - margin_cells), min(self.n, xs.max() + margin_cells + 1)
        return self.occupancy()[y0:y1, x0:x1], self.ox + x0 * self.res, self.oy + y0 * self.res


class VectorCostMap(Module):
    """Replaces dimOS's CostMapper on VECTOR. Ins: `lidar` (world cloud from
    lidar_odometry: lidar returns at z = 0.37, camera obstacles and bump
    patches at other heights), `camera_floor` (world floor samples, z = 0),
    `odom` (lidar pose in world). Out: `global_costmap` (the stream name the
    planner and the explorer already listen to)."""

    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]
    odom: In[PoseStamped]
    slip: In[Bool]                  # stuck_guard: undo the last SLIP_ROLLBACK_S of map writes
    global_costmap: Out[OccupancyGrid]

    def __init__(self, world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.world_frame = world_frame
        self._grid: ScoredGrid | None = None
        self._pose_xy: tuple[float, float] | None = None
        self._revolutions = 0
        self._ckpt_dir = os.path.join(CHECKPOINT_DIR, time.strftime("%Y%m%d-%H%M%S"))
        self._last_ckpt = time.monotonic()

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(f"VECTOR costmap up: {RESOLUTION_M} m cells, hit cap {HIT_CAP}, free floor {FREE_FLOOR}, occupied at {OCCUPIED_AT}")

    @rpc
    def stop(self) -> None:
        super().stop()

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._pose_xy = (float(msg.position.x), float(msg.position.y))
        if self._grid is None:
            self._grid = ScoredGrid(centre=self._pose_xy)

    async def handle_slip(self, msg: Bool) -> None:
        if self._grid is not None and getattr(msg, "data", False):
            undone = self._grid.rollback(SLIP_ROLLBACK_S)
            logger.warning(f"slip: rolled back the last {SLIP_ROLLBACK_S:.0f} s of map writes ({undone} batches)")

    async def handle_camera_floor(self, msg: PointCloud2) -> None:
        if self._grid is None:
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts):
            self._grid.camera_floor(pts[:, :2])

    async def handle_lidar(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None:
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts) == 0:
            return
        is_lidar = np.abs(pts[:, 2] - LIDAR_Z_M) < 0.005
        if is_lidar.all():
            self._grid.lidar_revolution(pts[:, :2], self._pose_xy)
            self._revolutions += 1
            if self._revolutions % PUBLISH_EVERY == 0:
                self._publish()
            self._grid.prune_journal()
            if time.monotonic() - self._last_ckpt >= CHECKPOINT_EVERY_S:
                self._checkpoint()
        else:
            self._grid.camera_obstacles(pts[~is_lidar][:, :2], self._pose_xy)
            if is_lidar.any():
                self._grid.lidar_revolution(pts[is_lidar][:, :2], self._pose_xy)

    def _checkpoint(self) -> None:
        assert self._grid is not None
        self._last_ckpt = time.monotonic()
        try:
            os.makedirs(self._ckpt_dir, exist_ok=True)
            path = os.path.join(self._ckpt_dir, time.strftime("costmap_%H%M%S.npz"))
            size = self._grid.save(path, self._pose_xy)
            old = sorted(f for f in os.listdir(self._ckpt_dir) if f.endswith(".npz"))
            for f in old[:-CHECKPOINT_KEEP]:
                os.remove(os.path.join(self._ckpt_dir, f))
            logger.info(f"costmap checkpoint {os.path.basename(path)}: {size / 1024:.0f} kB, {int(self._grid.seen.sum())} cells seen")
        except Exception:  # noqa: BLE001
            logger.exception("costmap checkpoint failed")

    def _publish(self) -> None:
        assert self._grid is not None
        crop = self._grid.cropped()
        if crop is None:
            return
        grid, ox, oy = crop
        origin = Pose()
        origin.position.x, origin.position.y, origin.position.z = ox, oy, 0.0
        origin.orientation.w = 1.0
        self.global_costmap.publish(OccupancyGrid(grid=grid, resolution=RESOLUTION_M, origin=origin,
                                                  frame_id=self.world_frame, ts=time.time()))

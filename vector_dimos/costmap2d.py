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
    lidar put down a lidar ray may clear; what the camera put down (a low box,
    under the 0.37 m scan plane) only the camera seeing the floor there may
    clear.

Sensor doctrine (metrox, 25/08): only the lidar and the camera write here.
The sonar brakes and the contact switches protect - neither leaves a trace in
the map. What the body drove over (body_clear) is still cleared: that one is a
physical certainty, not a sensor reading.

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

from vector_dimos import persistent_map

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
CHECKPOINT_DIR = persistent_map.CHECKPOINT_DIR
SLIP_ROLLBACK_S = 2.0        # how far back a slip erases
PROMOTE_EVERY_S = 300.0      # the persistent map is refreshed this often (and on a clean stop)
PROMOTE_MIN_CELLS = 2000     # a run that mapped almost nothing never replaces the saved flat
FRAME_DECISION_S = 20.0      # if no relocalization verdict arrives by then, start fresh as before


class ScoredGrid:
    """Two int8 score layers over a fixed world grid, plus where each cell was last hit from."""

    def __init__(self, resolution: float = RESOLUTION_M, span_m: float = GRID_SPAN_M,
                 centre: tuple[float, float] = (0.0, 0.0)) -> None:
        self.res = resolution
        self.n = int(round(span_m / resolution))
        self.ox = centre[0] - span_m / 2.0
        self.oy = centre[1] - span_m / 2.0
        self.lidar = np.zeros((self.n, self.n), dtype=np.int8)     # what the lidar saw at 0.37 m
        self.low = np.zeros((self.n, self.n), dtype=np.int8)       # what the camera saw below the scan plane
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self._last_hit_xy = np.full((self.n, self.n, 2), np.nan, dtype=np.float32)
        # undo journal: (monotonic time, layer, flat indices, applied deltas, seen-was-new mask)
        self._journal: list[tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        self._keepout: np.ndarray | None = None   # cells the rover may never enter, whatever the layers say

    # ---- coordinates -------------------------------------------------------
    def cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gx = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        gy = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        ok = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        return gx[ok], gy[ok]

    # ---- keep-out zones ----------------------------------------------------
    def set_keepouts(self, zones: list[dict]) -> int:
        """The `forbidden` zones the owner drew once on the persistent map.

        They are NOT a score, they are a decision - so they are applied in
        `occupancy()`, after every layer. Nothing writes them away: not a
        lidar ray seeing through the doorway, not `body_clear` (the rover
        cannot certify a cell it should never have stood on), not a slip
        rollback. The only way out is the keep-out file.

        The other zone type, `no_slip_reflex`, is not a map fact at all - it
        is read by the two slip guards, and this grid ignores it.
        """
        forbidden = persistent_map.zones_of(zones, persistent_map.FORBIDDEN)
        self._keepout = (persistent_map.keepout_mask(forbidden, self.res, self.ox, self.oy, self.n)
                         if forbidden else None)
        return 0 if self._keepout is None else int(self._keepout.sum())

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

    def body_clear(self, pose: tuple) -> None:
        """The body IS here: every cell under the body footprint (0.625 x
        0.46 m with the bumper bars - metrox 25/08 22h - exact, never wider,
        so a wall against the bumper survives)
        is certainly free. Both layers to the floor, seen. No journal entry:
        a slip rollback must not resurrect an obstacle under the chassis.
        Born 25/08: the rover kept walling itself in with patches laid on
        cells it then drove over."""
        x, y, yaw = pose
        c, s = np.cos(yaw), np.sin(yaw)
        lx = np.arange(-0.31, 0.31 + 1e-9, self.res / 2)
        ly = np.arange(-0.23, 0.23 + 1e-9, self.res / 2)
        bx, by = np.meshgrid(lx, ly)
        gx, gy = self.cell((x + c * bx - s * by).ravel(), (y + s * bx + c * by).ravel())
        if len(gx):
            self.lidar[gy, gx] = FREE_FLOOR
            self.low[gy, gx] = FREE_FLOOR
            self.seen[gy, gx] = True

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
        # a loaded map is CONTINUED now, not just inspected: it needs the undo
        # journal and the keep-out slot __init__ would have given it.
        g._journal = []
        g._keepout = None
        return g

    # ---- output ------------------------------------------------------------
    def occupancy(self) -> np.ndarray:
        """int8 grid: 100 occupied, 0 free (observed), -1 unknown."""
        score = np.maximum(self.lidar, self.low)
        out = np.full((self.n, self.n), -1, dtype=np.int8)
        out[self.seen] = 0
        out[score >= OCCUPIED_AT] = 100
        if self._keepout is not None:
            out[self._keepout] = 100        # last word, after every layer
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


def _zone_summary(zones: list[dict], forbidden_cells: int) -> str:
    """One readable line: what is forbidden, and where the reflexes stay quiet."""
    parts = []
    forbidden = persistent_map.zones_of(zones, persistent_map.FORBIDDEN)
    quiet = persistent_map.zones_of(zones, persistent_map.NO_SLIP_REFLEX)
    if forbidden:
        parts.append(f"{len(forbidden)} forbidden over {forbidden_cells} cells "
                     f"({', '.join(z['label'] for z in forbidden)})")
    if quiet:
        parts.append(f"{len(quiet)} no-slip-reflex, read by the slip guards, not by the map "
                     f"({', '.join(z['label'] for z in quiet)})")
    return "; ".join(parts)


class VectorCostMap(Module):
    """Replaces dimOS's CostMapper on VECTOR. Ins: `lidar` (world cloud from
    lidar_odometry: lidar returns at z = 0.37, camera obstacles at other
    heights), `camera_floor` (world floor samples, z = 0), `odom` (lidar pose
    in world), `slip` (stuck_guard). Out: `global_costmap` (the stream name
    the planner and the explorer already listen to).

    The lidar and the camera are the only writers: the sonar and the contact
    switches are reflexes, not mappers (sensor doctrine, 25/08)."""

    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]
    odom: In[PoseStamped]
    slip: In[Bool]                  # stuck_guard: undo the last SLIP_ROLLBACK_S of map writes
    reloc_frame: In[PoseStamped]    # lidar_odometry's verdict: which frame this run lives in, and when to freeze
    global_costmap: Out[OccupancyGrid]

    def __init__(self, world_frame: str = "world", **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.world_frame = world_frame
        self._grid: ScoredGrid | None = None
        self._pose_xy: tuple[float, float] | None = None
        self._revolutions = 0
        self._ckpt_dir = os.path.join(CHECKPOINT_DIR, time.strftime("%Y%m%d-%H%M%S"))
        self._last_ckpt = time.monotonic()
        self._last_clear: tuple | None = None   # (x, y, yaw) of the last body_clear
        # which frame this run writes in. None until lidar_odometry says: the
        # map is not created before, or a fresh grid would be born in the wrong
        # place and the persistent map could never be continued.
        self._frame: str | None = None
        self._frozen = False                    # relocalizing: write NOTHING, never corrupt the map
        self._t0 = time.monotonic()
        self._last_promote = time.monotonic()
        self._keepout_mtime = 0.0
        self._zones: list[dict] = []

    @rpc
    def start(self) -> None:
        super().start()
        logger.info(f"VECTOR costmap up: {RESOLUTION_M} m cells, hit cap {HIT_CAP}, free floor {FREE_FLOOR}, occupied at {OCCUPIED_AT}")
        if not persistent_map.enabled():
            self._decide("fresh", "PERSISTENT_MAP=0: no saved map, no relocalization, no keep-out zone")

    @rpc
    def stop(self) -> None:
        """A clean shutdown is the best moment to hand the flat over to the
        next session: checkpoint, then promote."""
        try:
            if self._grid is not None:
                self._checkpoint()
                self._promote(force=True)
        except Exception:  # noqa: BLE001
            logger.exception("costmap: promoting the map on stop failed")
        super().stop()

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._pose_xy = (float(msg.position.x), float(msg.position.y))
        if self._frame is None:
            if time.monotonic() - self._t0 < FRAME_DECISION_S:
                return              # waiting for the relocalization verdict: create nothing yet
            self._decide("fresh", f"no relocalization verdict in {FRAME_DECISION_S:.0f} s")
        if self._grid is None:
            self._grid = ScoredGrid(centre=self._pose_xy)
            logger.info(f"costmap: fresh grid centred on ({self._pose_xy[0]:+.2f}, {self._pose_xy[1]:+.2f})")
        if self._frozen:
            return                  # the pose is not trusted: not even body_clear
        q = msg.orientation
        yaw = float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        lc = self._last_clear
        if (lc is None
                or (self._pose_xy[0] - lc[0]) ** 2 + (self._pose_xy[1] - lc[1]) ** 2 > 0.03 ** 2
                or abs((yaw - lc[2] + np.pi) % (2 * np.pi) - np.pi) > np.radians(10.0)):
            self._last_clear = (self._pose_xy[0], self._pose_xy[1], yaw)
            self._grid.body_clear(self._last_clear)

    async def handle_reloc_frame(self, msg: PoseStamped) -> None:
        """lidar_odometry publishes its frame state on every revolution:
        ``reloc:searching`` (relocalizing - write nothing), ``reloc:persistent``
        (this run continues the saved map) or ``reloc:fresh`` (own frame, as
        before this existed). Idempotent on purpose: the state is republished
        at 10 Hz, so no start-up race can lose the verdict."""
        state = str(getattr(msg, "frame_id", "") or "").removeprefix("reloc:")
        if state == "searching":
            if not self._frozen:
                self._frozen = True
                logger.warning("costmap: relocalizing - map writing frozen (no hit, no miss, no body_clear)")
            return
        if state not in ("persistent", "fresh"):
            return
        if self._frame is None:
            self._decide(state, "lidar_odometry relocalized against the saved map"
                         if state == "persistent" else "lidar_odometry started a fresh frame")
        elif state == "persistent" and self._frame == "fresh":
            # The boot grace window paid off: a standing rover could not tell
            # where it was, it drove a little and now it can. The few minutes
            # of fresh-frame map built meanwhile are dropped - they are in the
            # wrong frame, and everything they saw is about to be seen again.
            logger.warning(f"costmap: relocalized late - dropping the fresh map "
                           f"({int(self._grid.seen.sum()) if self._grid else 0} cells) and continuing "
                           "the persistent one")
            self._grid = None
            self._last_clear = None
            self._frame = None
            self._decide("persistent", "lidar_odometry relocalized during the boot grace window")
        if self._frozen:
            self._frozen = False
            logger.info(f"costmap: relocalized, map writing resumed in the {self._frame} frame")

    def _decide(self, frame: str, why: str) -> None:
        """Settle the frame this run writes in - once, and out loud."""
        self._frame = frame
        if frame == "persistent":
            try:
                self._grid = ScoredGrid.load(persistent_map.MAP_PATH)
                self._zones = persistent_map.load_keepouts()
                self._keepout_mtime = (os.path.getmtime(persistent_map.KEEPOUT_PATH)
                                       if os.path.isfile(persistent_map.KEEPOUT_PATH) else 0.0)
                cells = self._grid.set_keepouts(self._zones)
                logger.info(f"costmap: CONTINUING the persistent map ({why}) - "
                            f"{int(self._grid.seen.sum())} cells already known, origin "
                            f"({self._grid.ox:+.1f}, {self._grid.oy:+.1f}), "
                            f"zones drawn on {persistent_map.keepout_frame()!r}: "
                            + (_zone_summary(self._zones, cells) or "none"))
                return
            except Exception:  # noqa: BLE001
                logger.exception("costmap: the persistent map would not load - falling back to a fresh frame")
                self._frame = "fresh"
        logger.warning(f"costmap: FRESH frame ({why}). This map has its own arbitrary origin, so the "
                       "keep-out zones do NOT apply to it - they are coordinates in the persistent frame.")

    def _reload_keepouts(self) -> None:
        """Pick up an edit of keepout.json without a restart - the stack is not
        something the owner can restart on a whim."""
        if self._frame != "persistent" or self._grid is None:
            return
        try:
            mtime = (os.path.getmtime(persistent_map.KEEPOUT_PATH)
                     if os.path.isfile(persistent_map.KEEPOUT_PATH) else 0.0)
            if mtime == self._keepout_mtime:
                return
            self._keepout_mtime = mtime
            self._zones = persistent_map.load_keepouts()
            cells = self._grid.set_keepouts(self._zones)
            logger.info("costmap: zones reloaded - " + (_zone_summary(self._zones, cells) or "none"))
        except Exception:  # noqa: BLE001
            logger.exception("costmap: keep-out zones would not reload - the previous ones stay in force")

    async def handle_slip(self, msg: Bool) -> None:
        if self._grid is not None and getattr(msg, "data", False):
            undone = self._grid.rollback(SLIP_ROLLBACK_S)
            logger.warning(f"slip: rolled back the last {SLIP_ROLLBACK_S:.0f} s of map writes ({undone} batches)")

    async def handle_camera_floor(self, msg: PointCloud2) -> None:
        if self._grid is None or self._frozen:
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts):
            self._grid.camera_floor(pts[:, :2])

    async def handle_lidar(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None or self._frozen:
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
            self._reload_keepouts()
            self._promote()
        except Exception:  # noqa: BLE001
            logger.exception("costmap checkpoint failed")

    _promote_refused = False

    def _promote(self, force: bool = False) -> None:
        """Hand the freshest checkpoint over to the next session.

        The one rule that matters: a run with its OWN arbitrary frame never
        silently replaces a persistent map that already exists. Overwriting it
        would move the whole flat under the keep-out zones, which are
        coordinates in the persistent frame - the toilets would end up
        somewhere else. Bootstrapping the first map is allowed; replacing one
        is an explicit decision (PERSISTENT_MAP_REBASE=1).
        """
        assert self._grid is not None
        if not persistent_map.enabled():
            return
        if not force and time.monotonic() - self._last_promote < PROMOTE_EVERY_S:
            return
        self._last_promote = time.monotonic()
        seen = int(self._grid.seen.sum())
        if seen < PROMOTE_MIN_CELLS:
            return                       # a run that saw almost nothing is not a flat
        if self._frame != "persistent" and persistent_map.map_exists() and not persistent_map.rebase_allowed():
            if not self._promote_refused:
                self._promote_refused = True
                logger.warning("costmap: NOT promoting this run - it has its own frame and a persistent map "
                               "already exists. Set PERSISTENT_MAP_REBASE=1 to replace the saved flat.")
            return
        ckpt = persistent_map.newest_checkpoint(self._ckpt_dir)
        if ckpt is None:
            return
        persistent_map.promote(ckpt)
        logger.info(f"costmap: persistent map updated from {os.path.basename(ckpt)} "
                    f"({seen} cells known, {len(persistent_map.generations())} older generations kept) "
                    f"-> {persistent_map.MAP_PATH}")

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

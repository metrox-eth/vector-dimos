"""A 2D costmap for a wheeled rover that learns AND unlearns.

dimOS's CostMapper turns the 3D voxel map (a set that never forgets) into a
terrain-slope map meant for a walking robot; measured on 23/08 with
tools/mars/stages.py it erased table legs (erosion drops any cell whose four
neighbours are unobserved). The spec, learned from sharing a flat with a
robot vacuum:

  * the map must REINFORCE - but with a ceiling, or a ramp it struggled on
    becomes a wall after a day;
  * it must UNLEARN what it sees absent again - the telescope under the desk
    was gone after run 1 and still avoided at run 10;
  * reinforcement needs new viewpoints: from one spot a thing gets no more
    certain than "occupied" (two misses from gone); a table leg seen from
    twenty positions is solid (up to ten misses);
  * two layers, because the sensors do not see the same things: what the
    lidar put down a lidar ray may clear; what the camera put down (a low box,
    under the 0.37 m scan plane) only the camera may clear - by seeing the
    floor there, or by seeing THROUGH it at low height (`camera_rays` - a
    persistent map is good, but a ghost must never survive being looked at).

A cell the camera JUST called an obstacle is deaf to floor samples for
LOW_HIT_PROTECT_S. Measured 26/08 on run B: a table leg thinner than a 5 cm
cell was found at low = -3 after ten real contacts. lidar_odometry drops floor
samples that land on an obstacle cell of the SAME frame
(`split_floor_and_obstacles`), but the next frame is a new frame: the leg is
seen in one frame and the floor beside it - same cell, 5 cm of quantisation -
in the next, so the leg never accumulated the two hits it needs. This is not a
weakening of the unlearning: an object that is really gone stops being hit, and
after 3 s of floor samples it fades exactly as before.

Sensor doctrine: only the lidar and the camera write here.
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
from dimos.utils.logging_config import setup_logger

from vector_dimos import persistent_map

logger = setup_logger()

RESOLUTION_M = 0.05
GRID_SPAN_M = 24.0           # world-fixed square, centred on the start pose
HIT_CAP = 10                 # ceiling: no cell gets more certain than this
FREE_FLOOR = -3              # floor: no cell gets more "free" than this
OCCUPIED_AT = 2              # two hits from two places = an obstacle
NEW_VIEWPOINT_M = 0.10       # a hit counts only if the rover moved this much since the cell's last hit
LOW_HIT_PROTECT_S = 3.0      # after a camera obstacle hit, that cell's LOW layer ignores floor samples this long
LIDAR_Z_M = 0.37             # points at exactly this height come from the lidar (lidar_odometry.LIDAR_HEIGHT_M)
LIDAR_WRITES_OBSTACLES = True   # doctrine switch: the lidar's job is anchoring the
                                # map (SLAM), not obstacle detection - flip to False
                                # once the camera proves its widened coverage in flight; the lidar
                                # then only anchors kiss-icp and the camera is THE detector
PUBLISH_EVERY = 5            # ~2 Hz in PILOTED mode: in teleop the planner sleeps and this 2D map reads the sensors live - the only consumer fed by this rate is THE VIEWER, which drowned at 5 Hz (latency 9 -> 30 s). Full rates come back for autonomy TOGETHER with moving the bridge off-board. Earlier note: 10 Hz (2026-08-27) drowned the CONSUMERS: 230 KB
                             # per grid x 10 Hz = 2.3 MB/s to decode for every subscriber, and
                             # the recorder wrote the map 5x to SD - load 5.7 -> 11 in
                             # 5 min (measurably more CPU burned than before). The map QUALITY
                             # does not depend on this rate (odometry gives it);
                             # 10 Hz will only serve autonomy, once the recorder is decoupled.
                             # Earlier stopgap:
                             # capping at 5 (2 Hz) was a CPU stopgap from before CUDA+zenoh
                             # (2026-08-27 - removing it was the point of moving to CUDA)
RAY_MAX_M = 4.0              # rays are carved (misses) up to this range only: 12 m x 0.025 m steps cost 227 ms per revolution (2.3 cores at 10 Hz, 23/08 20:20)
RAY_EVERY = 2                # carve on every other revolution; hits are taken on all
CAMERA_HEIGHT_M = 0.56       # = lidar_odometry.CAMERA_XYZ_BASE[2] (rear mast, 24/08) - keep in sync
CARVE_Z_BAND = (0.10, 0.45)  # a camera ray crossing a cell at this height proves nothing low stands
                             # there; a higher ray flies OVER boxes (0.9 m up says nothing about a
                             # 0.3 m box) and must not erase them
CHECKPOINT_EVERY_S = 30.0    # a crash must cost at most a minute of map
CHECKPOINT_KEEP = 40         # 20 minutes of history
CHECKPOINT_DIR = persistent_map.CHECKPOINT_DIR
PROMOTE_EVERY_S = 300.0      # the persistent map is refreshed this often (and on a clean stop)
PROMOTE_MIN_CELLS = 2000     # a run that mapped almost nothing never replaces the saved flat
FRAME_DECISION_S = 20.0      # if no relocalization verdict arrives by then, start fresh as before
WALLED_IN_MIN_M2 = 3.0       # below this reachable free area the map is a prison, not a flat
                             # (explorer2's WALLED-IN threshold) - such a map is never promoted


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
        # when the camera last called each cell an obstacle (monotonic seconds).
        # Runtime only, never saved: it protects a leg for 3 s, and a monotonic
        # clock means nothing in another process anyway.
        self._last_low_hit = np.full((self.n, self.n), -np.inf, dtype=np.float64)
        self._cam_prev = np.zeros((self.n, self.n), dtype=bool)   # camera obstacle cells of the PREVIOUS frame (moving-object gate)
        self._keepout: np.ndarray | None = None   # cells the rover may never enter, whatever the layers say

    # ---- coordinates -------------------------------------------------------
    def cell(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gx = np.floor((np.asarray(x) - self.ox) / self.res).astype(np.int64)
        gy = np.floor((np.asarray(y) - self.oy) / self.res).astype(np.int64)
        ok = (gx >= 0) & (gx < self.n) & (gy >= 0) & (gy < self.n)
        return gx[ok], gy[ok]

    # ---- keep-out zones ----------------------------------------------------
    def set_keepouts(self, zones: list[dict]) -> int:
        """The `forbidden` zones drawn once, by hand, on the persistent map.

        They are NOT a score, they are a decision - so they are applied in
        `occupancy()`, after every layer. Nothing writes them away: not a
        lidar ray seeing through the doorway, not `body_clear` (the rover
        cannot certify a cell it should never have stood on). The only way
        out is the keep-out file.

        Rectangle or polygon, the answer is one bool mask: the rasterising
        lives in `persistent_map.keepout_mask` (even-odd ray casting on the
        cell centres plus the outline, numpy only), so the CLI, the simulator
        and this grid all forbid exactly the same cells. A polygon matters
        because the house is 5.75 deg off the map axes: an enclosing rectangle
        either eats the corridor or leaks the corner.

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
        # An obstacle is REAL when it was seen from two viewpoints: a
        # passer-by hammered from one parked spot must never become a wall
        # (whole flights were once spent walled in by exactly such cells).
        # From one viewpoint a cell rises to OCCUPIED_AT - 1 and no
        # further; the second viewpoint (0.10 m of motion) makes it a wall.
        # Driving toward anything real crosses viewpoints within a metre, so
        # legs and furniture still map on approach.
        cap = np.where(moved, HIT_CAP, OCCUPIED_AT - 1)
        new = np.minimum(cur + 1, cap).astype(np.int8)
        layer[gy, gx] = new
        self._last_hit_xy[gy[moved], gx[moved]] = from_xy
        self.seen[gy, gx] = True
        return int(moved.sum())

    def _miss(self, layer: np.ndarray, gx: np.ndarray, gy: np.ndarray) -> None:
        if len(gx) == 0:
            return
        cur = layer[gy, gx].astype(np.int16)
        new = np.maximum(cur - 1, FREE_FLOOR).astype(np.int8)
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

    def camera_obstacles(self, pts_xy: np.ndarray, from_xy: tuple[float, float],
                         now: float | None = None) -> None:
        """The camera saw something standing here: a hit on the LOW layer, and
        the cell is stamped - for the next LOW_HIT_PROTECT_S the floor beside it
        may not erase it (see the module docstring: the table leg of run B).

        MOVING things are never written into the map: a cell
        only takes a hit if the camera ALSO saw an obstacle there on the
        previous frame. At 5 Hz, a walking person or a rolling vacuum moves on
        before the second look; furniture repeats. One frame of latency on
        genuinely new static obstacles - the lidar layer covers the interval."""
        gx, gy = self.cell(pts_xy[:, 0], pts_xy[:, 1])
        if len(gx) == 0:
            self._cam_prev[:] = False
            return
        # NEIGHBOURHOOD repeat, not exact-cell repeat: a
        # THIN standing leg re-projects a cell off between frames (5 cm of
        # quantisation from a moving rover) - exact-cell repeat would filter
        # it out like a mover. One cell of tolerance keeps every immobile
        # thing; a walker or the vacuum still jumps further than 10 cm/frame.
        prev_near = self._cam_prev.copy()
        prev_near[:-1, :] |= self._cam_prev[1:, :]
        prev_near[1:, :] |= self._cam_prev[:-1, :]
        prev_near[:, :-1] |= prev_near[:, 1:]
        prev_near[:, 1:] |= prev_near[:, :-1]
        keep = prev_near[gy, gx]
        self._cam_prev[:] = False
        self._cam_prev[gy, gx] = True
        if not keep.any():
            return
        rx = self.ox + (gx[keep] + 0.5) * self.res
        ry = self.oy + (gy[keep] + 0.5) * self.res
        self._hit(self.low, rx, ry, from_xy)
        self._last_low_hit[gy[keep], gx[keep]] = time.monotonic() if now is None else now

    def camera_floor(self, pts_xy: np.ndarray, now: float | None = None) -> None:
        """The camera saw bare floor here: a miss on BOTH layers (the only way a
        low object is ever forgotten).

        Except on the LOW layer of a cell the camera called an obstacle less
        than LOW_HIT_PROTECT_S ago: there, the floor sample is dropped. A thing
        thinner than a 5 cm cell is seen in one frame and its floor in the next,
        and the two cancelled forever. The lidar-layer miss is untouched - a
        lidar hit is a different claim, and this protects nothing it says.
        """
        now = time.monotonic() if now is None else now
        gx, gy = self.cell(pts_xy[:, 0], pts_xy[:, 1])
        if len(gx) == 0:
            return
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        protected = self._last_low_hit[gy, gx] > now - LOW_HIT_PROTECT_S
        self._miss(self.low, gx[~protected], gy[~protected])
        self._miss(self.lidar, gx, gy)

    def camera_rays(self, pts_xyz: np.ndarray, from_xy: tuple[float, float],
                    now: float | None = None) -> None:
        """The camera's rays carve the LOW layer, symmetric to the lidar's
        (design rule: the SLAM must correct the map every time the
        RealSense passes over it - without this, a ghost lives until someone
        rebuilds the map by hand).

        Every cell a ray crosses at low height (CARVE_Z_BAND) held nothing
        standing, or the ray would have ended on it: a miss. Works for
        obstacle endpoints and floor samples alike - a floor point at 3 m
        proves the whole low corridor in front of it. Cells the camera called
        an obstacle less than LOW_HIT_PROTECT_S ago are skipped: a real thing
        is re-hit every frame, so only what the camera has STOPPED confirming
        fades. The lidar layer is never touched - a lidar hit is a different
        claim, and only a lidar ray may retract it."""
        if len(pts_xyz) == 0:
            return
        now = time.monotonic() if now is None else now
        dx = pts_xyz[:, 0] - from_xy[0]
        dy = pts_xyz[:, 1] - from_xy[1]
        r = np.hypot(dx, dy)
        keep = r > self.res
        if not keep.any():
            return
        dx, dy, r = dx[keep], dy[keep], r[keep]
        z_end = pts_xyz[keep, 2]
        end = np.minimum(r - self.res, RAY_MAX_M)   # stop one cell short of the hit
        nmax = int(np.floor(end.max() / self.res))
        if nmax < 1:
            return
        d = (np.arange(1, nmax + 1) * self.res)[None, :]
        f = d / r[:, None]
        z = CAMERA_HEIGHT_M + (z_end[:, None] - CAMERA_HEIGHT_M) * f
        valid = (d <= end[:, None]) & (z > CARVE_Z_BAND[0]) & (z < CARVE_Z_BAND[1])
        if not valid.any():
            return
        ux, uy = (dx / r)[:, None], (dy / r)[:, None]
        xs = (from_xy[0] + ux * d)[valid]
        ys = (from_xy[1] + uy * d)[valid]
        gx, gy = self.cell(xs, ys)
        if len(gx) == 0:
            return
        flat = np.unique(gy * self.n + gx)
        gx, gy = flat % self.n, flat // self.n
        unprotected = self._last_low_hit[gy, gx] <= now - LOW_HIT_PROTECT_S
        self._miss(self.low, gx[unprotected], gy[unprotected])

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

    def reachable_free_m2(self, pose_xy: tuple[float, float]) -> float | None:
        """Free area (m2) connected to the rover's position - None if unknowable.

        Born 26/08 evening: a run that starts walled in by ghost cells must
        never hand its prison over as the persistent map. Same 8-connected
        flood as explorer2's survey, computed here on the occupancy so the
        promotion gate needs no cross-module plumbing."""
        try:
            from scipy import ndimage
        except Exception:  # noqa: BLE001
            return None
        gx, gy = self.cell(np.array([pose_xy[0]]), np.array([pose_xy[1]]))
        if len(gx) == 0:
            return None
        labels, _ = ndimage.label(self.occupancy() == 0, structure=np.ones((3, 3), dtype=bool))
        lab = labels[gy[0], gx[0]]
        if lab == 0:
            # the rover's own cell is not free (fresh grid, or it stands on a
            # ghost): take the biggest free patch in a 30 cm window around it
            w = 6
            window = labels[max(0, gy[0] - w):gy[0] + w + 1, max(0, gx[0] - w):gx[0] + w + 1]
            vals = window[window > 0]
            if len(vals) == 0:
                return 0.0
            lab = int(np.bincount(vals).argmax())
        return float((labels == lab).sum()) * self.res ** 2

    def body_clear(self, pose: tuple) -> None:
        """The body IS here: every cell under the body footprint (0.625 x
        0.46 m with the bumper bars - measured, exact, never wider,
        so a wall against the bumper survives)
        is certainly free. Both layers to the floor, seen. Born 25/08: the rover kept walling itself in with patches laid on
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

    # ---- checkpoints -------------------------------------------------------
    def save(self, path: str, pose_xy: tuple[float, float] | None = None) -> int:
        """Full state to a compressed .npz; returns the file size in bytes.

        Written to a sibling tmp file then renamed: the 26/08 13h00 battery
        death cut a checkpoint mid-write and the truncated .npz crashed every
        later reader. os.replace is atomic on the same filesystem."""
        tmp = path + ".tmp.npz"      # numpy appends .npz to any other suffix
        np.savez_compressed(tmp, lidar=self.lidar, low=self.low, seen=self.seen, last_hit_xy=self._last_hit_xy,
                            res=self.res, ox=self.ox, oy=self.oy, n=self.n,
                            pose_xy=np.array(pose_xy if pose_xy else (np.nan, np.nan)), ts=time.time())
        os.replace(tmp, path)
        return os.path.getsize(path)

    @classmethod
    def load(cls, path: str) -> "ScoredGrid":
        z = np.load(path)
        g = cls.__new__(cls)
        g.res, g.n, g.ox, g.oy = float(z["res"]), int(z["n"]), float(z["ox"]), float(z["oy"])
        g.lidar, g.low, g.seen, g._last_hit_xy = z["lidar"], z["low"], z["seen"], z["last_hit_xy"]
        g._revs = 0
        # a loaded map is CONTINUED now, not just inspected: it needs the
        # keep-out slot __init__ would have given it.
        g._keepout = None
        g._last_low_hit = np.full((g.n, g.n), -np.inf, dtype=np.float64)
        g._cam_prev = np.zeros((g.n, g.n), dtype=bool)
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
    if forbidden:
        parts.append(f"{len(forbidden)} forbidden over {forbidden_cells} cells "
                     f"({', '.join(z['label'] for z in forbidden)})")
    return "; ".join(parts)


class VectorCostMap(Module):
    """Replaces dimOS's CostMapper on VECTOR. Ins: `lidar` (world cloud from
    lidar_odometry: lidar returns at z = 0.37, camera obstacles at other
    heights), `camera_floor` (world floor samples, z = 0), `odom` (lidar pose
    in world). Out: `global_costmap` (the stream name the planner and the
    explorer already listen to).

    The lidar and the camera are the only writers: the sonar and the contact
    switches are reflexes, not mappers (sensor doctrine, 25/08)."""

    lidar: In[PointCloud2]
    camera_floor: In[PointCloud2]
    odom: In[PoseStamped]
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
        """Pick up an edit of keepout.json without a restart - restarting the
        whole stack to change a zone is not a reasonable ask mid-session."""
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

    async def handle_camera_floor(self, msg: PointCloud2) -> None:
        if self._grid is None or self._frozen:
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts):
            self._grid.camera_floor(pts[:, :2])
            if self._pose_xy is not None:
                self._grid.camera_rays(pts, self._pose_xy)

    async def handle_lidar(self, msg: PointCloud2) -> None:
        if self._grid is None or self._pose_xy is None or self._frozen:
            return
        pts = np.asarray(msg.as_numpy()[0], dtype=np.float64)
        if len(pts) == 0:
            return
        is_lidar = np.abs(pts[:, 2] - LIDAR_Z_M) < 0.005
        if not LIDAR_WRITES_OBSTACLES:
            cam = pts[~is_lidar]
            if len(cam):
                self._grid.camera_obstacles(cam[:, :2], self._pose_xy)
                self._grid.camera_rays(cam, self._pose_xy)
            self._revolutions += int(is_lidar.any())
            if self._revolutions % PUBLISH_EVERY == 0:
                self._publish()
            if time.monotonic() - self._last_ckpt >= CHECKPOINT_EVERY_S:
                self._checkpoint()
            return
        # One path for pure and mixed clouds. The old split published only from
        # the pure-lidar branch; once lidar_odometry began attaching camera
        # points to EVERY revolution (the 15h30 anti-blink), no cloud was ever
        # pure again and the costmap silently stopped publishing (found 27/08
        # 16h20 by py-spy + bus listening: grid building in memory, zero out).
        cam = pts[~is_lidar]
        if len(cam):
            # hits first: they stamp the 3 s protection, so a frame never
            # carves the very cells it is confirming
            self._grid.camera_obstacles(cam[:, :2], self._pose_xy)
            self._grid.camera_rays(cam, self._pose_xy)
        if is_lidar.any():
            self._grid.lidar_revolution(pts[is_lidar][:, :2], self._pose_xy)
            self._revolutions += 1
            if self._revolutions % PUBLISH_EVERY == 0:
                self._publish()
            if time.monotonic() - self._last_ckpt >= CHECKPOINT_EVERY_S:
                self._checkpoint()

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
        if self._pose_xy is not None:
            free = self._grid.reachable_free_m2(self._pose_xy)
            if free is not None and free < WALLED_IN_MIN_M2:
                logger.warning(f"costmap: NOT promoting - the rover believes itself walled in "
                               f"({free:.1f} m2 reachable, needs {WALLED_IN_MIN_M2:.0f}). A map that "
                               "imprisons the rover must never become the flat (26/08 evening).")
                return
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

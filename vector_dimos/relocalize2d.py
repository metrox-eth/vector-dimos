"""Global 2D relocalization: put a lidar revolution back on a saved map.

Why not dimOS's own brick (`dimos/mapping/relocalization/`, evaluated 26/08):
its `relocalize()` is FPFH + RANSAC + ICP on 3D point clouds through Open3D.
It estimates normals, filters candidates by gravity tilt and scores on a
"wall subset" selected by `|normal_z| < 0.7` -- every one of those steps needs
a cloud with height in it. VECTOR's lidar puts ~400 points per revolution in a
single plane at z = 0.37 m: normals are undefined, the gravity filter is
meaningless, the wall subset falls back to the full cloud, and FPFH
descriptors on a planar cloud are near-degenerate. Their module also gates on
`MIN_LOCAL_POINTS = 50_000` (we have 400), consumes `.pc2.lcm` premaps built
by `dimos map global` on a Go2, and publishes a world->map TF instead of
rebasing the odometry: the continued map would keep living in the fresh
arbitrary frame, which is exactly the amnesia we are removing.

So: the public 2D approach, homemade in numpy. Multi-resolution correlative
scan matching (the classic Cartographer shape, without the branch-and-bound
bookkeeping -- on a 24 x 24 m grid at 0.05 m an exhaustive coarse pass is
already sub-second). The map is turned once into a distance field (metres to
the nearest occupied cell) and a likelihood field `exp(-d^2 / 2 sigma^2)`;
a candidate pose scores as the mean likelihood over the scan points. Coarse
levels use MAX-pooled likelihood, which is an upper bound of the fine score,
so a true basin cannot be pruned by the coarse pass.

Confidence is three explicit numbers, all reported:
  * `score`     - mean likelihood at the winner, roughly "the fraction of the
                  scan that lands on a wall the saved map already knows";
  * `margin`    - that score divided by the best score of a DIFFERENT PLACE
                  (further than BASIN_M or turned by more than BASIN_DEG). A
                  symmetric room or a repeating corridor gives margin ~= 1:
                  ambiguous, refused, however high the score;
  * `median_dist_m` - the walls actually overlapping, in metres: the median
                  distance from a relocalized scan point to the nearest
                  occupied cell. Measured on the flat: 0.0 cm for a good
                  match, 35 cm for a scan of a room the map has never seen.

BASIN_M is metres, not centimetres, on purpose. Measured on two runs of the
flat on 26/08: the runner-up of a CORRECT match is the same alignment slid
0.6 m along the corridor (score 0.91 against the winner's 0.98) - one
hypothesis with a long ridge, not two places. The first genuinely different
place sits 1.7 m away at 0.68. The search itself lands within 6 cm on that
same data, twenty times finer than the basin, so a wide basin costs no
precision and stops a correct answer from being thrown away as ambiguous.

Units are metres and radians throughout. No dimOS import: this module runs
under a bare numpy, which is what lets the cold bench check it anywhere.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

# --- the map turned into something a scan can be scored against -------------
SIGMA_M = 0.10          # a scan point 10 cm off a wall still scores 0.61
FIELD_RADIUS_M = 0.40   # distance field saturates here (exact below it)

# --- search --------------------------------------------------------------
COARSE_M = 0.40         # coarse translation step = 8 base cells
MAX_RANGE_M = 12.0      # scan points beyond this are dropped (grid is 24 m wide)
MAX_COARSE_CANDIDATES = 6000
MAX_SCAN_POINTS = 1800  # a stride, NOT a dedupe (see relocalize())
KEEP_COARSE = 40        # distinct basins carried into the refinement
KEEP_FINE = 10

# --- acceptance ----------------------------------------------------------
MIN_SCORE = 0.55        # more than half the scan must land on known walls
MIN_MARGIN = 1.25       # the winner must beat the best rival PLACE by 25 %
MAX_MEDIAN_M = 0.15     # and the walls must really overlap, not just nearly
BASIN_M = 1.50          # closer than this = the same alignment, slid (see the docstring)
BASIN_DEG = 30.0
MIN_POINTS = 120        # a revolution carries ~400; below this, no verdict
WALL_INLIER_M = 0.15    # "the walls agree" distance, for the reported inlier fraction


@dataclass
class Match:
    """The verdict, with the numbers that justify it."""

    x: float                 # translation of the scan frame into the map frame
    y: float
    yaw: float
    score: float             # mean likelihood at the winner, 0..1
    margin: float            # score / best score of another basin (inf if there is none)
    rival: float             # that rival score
    rival_pose: tuple[float, float, float]   # where that rival place was
    median_dist_m: float     # median distance from a scan point to the nearest occupied cell
    inlier_frac: float       # fraction of scan points within WALL_INLIER_M of one
    n_points: int            # scan points actually used (deduplicated at the map resolution)
    seconds: float
    accepted: bool
    reason: str

    def as_log(self) -> str:
        return (f"score={self.score:.3f} margin={self.margin:.2f} "
                f"(rival {self.rival:.3f} at {self.rival_pose[0]:+.2f}, {self.rival_pose[1]:+.2f}, "
                f"{math.degrees(self.rival_pose[2]):+.0f} deg) "
                f"median_wall={self.median_dist_m*100:.1f} cm inliers={self.inlier_frac*100:.0f}% "
                f"pose=({self.x:+.2f}, {self.y:+.2f}, {math.degrees(self.yaw):+.1f} deg) "
                f"{self.n_points} pts in {self.seconds:.2f} s -> {self.reason}")


# --- fields ---------------------------------------------------------------

def _shift(a: np.ndarray, dy: int, dx: int) -> np.ndarray:
    """`out[y, x] = a[y + dy, x + dx]`, zero (False) outside."""
    h, w = a.shape
    out = np.zeros_like(a)
    out[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)] = \
        a[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)]
    return out


def distance_field(occupied: np.ndarray, res: float, radius_m: float = FIELD_RADIUS_M) -> np.ndarray:
    """Metres from each cell to the nearest occupied cell.

    Exact Euclidean inside `radius_m` (every offset of the disc is tested),
    saturated at `radius_m` beyond -- which is all the scoring needs, and
    keeps this to ten lines of numpy instead of a scipy dependency.
    """
    r = int(round(radius_m / res))
    out = np.full(occupied.shape, radius_m, dtype=np.float32)
    out[occupied] = 0.0
    offsets = [(dy, dx, math.hypot(dy, dx) * res)
               for dy in range(-r, r + 1) for dx in range(-r, r + 1)]
    for dy, dx, d in sorted(offsets, key=lambda o: o[2]):
        if d <= 0.0 or d >= radius_m:
            continue
        near = _shift(occupied, dy, dx) & (out > d)
        out[near] = d
    return out


def _pool_max(a: np.ndarray, k: int, pad_value: float = 0.0) -> np.ndarray:
    """Max over every k x k block: an upper bound of the fine values."""
    if k == 1:
        return a
    h, w = a.shape
    ph, pw = (-h) % k, (-w) % k
    if ph or pw:
        a = np.pad(a, ((0, ph), (0, pw)), constant_values=pad_value)
    return a.reshape(a.shape[0] // k, k, a.shape[1] // k, k).max(axis=(1, 3))


class MapField:
    """A saved occupancy grid, ready to be matched against.

    `occupancy()` speaks the ScoredGrid language: 100 occupied, 0 free
    (observed), -1 unknown. Occupied cells make the distance field; observed
    free cells are where the rover is allowed to be, i.e. the candidate
    translations of the global search.
    """

    def __init__(self, occupied: np.ndarray, free: np.ndarray, res: float, ox: float, oy: float,
                 sigma_m: float = SIGMA_M, radius_m: float = FIELD_RADIUS_M) -> None:
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        self.radius_m = radius_m
        self.occupied = occupied
        self.free = free
        self.dist = distance_field(occupied, self.res, radius_m)
        self.lik = np.exp(-(self.dist.astype(np.float64) ** 2) / (2.0 * sigma_m ** 2)).astype(np.float32)

    @classmethod
    def from_grid(cls, grid, **kwargs) -> "MapField":
        """From anything with `occupancy()`, `res`, `ox`, `oy` (a ScoredGrid)."""
        occ = grid.occupancy()
        return cls(occ == 100, occ == 0, grid.res, grid.ox, grid.oy, **kwargs)

    @property
    def n_occupied(self) -> int:
        return int(self.occupied.sum())

    @property
    def n_free(self) -> int:
        return int(self.free.sum())


# --- scoring --------------------------------------------------------------

def _score_poses(lik: np.ndarray, res: float, ox: float, oy: float,
                 pts: np.ndarray, poses: np.ndarray, block: int = 2048) -> np.ndarray:
    """Mean likelihood of `pts` placed by each pose. `poses` is (M, 3) x/y/yaw."""
    h, w = lik.shape
    out = np.empty(len(poses), dtype=np.float32)
    for i in range(0, len(poses), block):
        p = poses[i:i + block]
        c, s = np.cos(p[:, 2])[:, None], np.sin(p[:, 2])[:, None]
        px = c * pts[None, :, 0] - s * pts[None, :, 1] + p[:, 0:1]
        py = s * pts[None, :, 0] + c * pts[None, :, 1] + p[:, 1:2]
        gx = np.floor((px - ox) / res).astype(np.int32)
        gy = np.floor((py - oy) / res).astype(np.int32)
        inside = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        np.clip(gx, 0, w - 1, out=gx)
        np.clip(gy, 0, h - 1, out=gy)
        v = lik[gy, gx]
        v[~inside] = 0.0
        out[i:i + block] = v.mean(axis=1)
    return out


def _place(pts: np.ndarray, pose: tuple[float, float, float]) -> np.ndarray:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.stack([c * pts[:, 0] - s * pts[:, 1] + pose[0],
                     s * pts[:, 0] + c * pts[:, 1] + pose[1]], axis=1)


def _dedupe(pts: np.ndarray, cell: float) -> np.ndarray:
    keys = np.floor(pts / cell).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def _same_basin(a: np.ndarray, b: np.ndarray) -> bool:
    dyaw = abs(math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2])))
    return math.hypot(a[0] - b[0], a[1] - b[1]) < BASIN_M and dyaw < math.radians(BASIN_DEG)


def _distinct_top(poses: np.ndarray, scores: np.ndarray, keep: int) -> np.ndarray:
    """Greedy non-max suppression over basins: distinct hypotheses, best first."""
    order = np.argsort(-scores)
    out: list[int] = []
    for i in order:
        if all(not _same_basin(poses[i], poses[j]) for j in out):
            out.append(int(i))
            if len(out) >= keep:
                break
    return np.asarray(out, dtype=int)


def _grid_around(poses: np.ndarray, half_xy: float, step_xy: float,
                 half_yaw: float, step_yaw: float) -> np.ndarray:
    """Every pose of a local box around each seed, stacked."""
    d = np.arange(-half_xy, half_xy + 1e-9, step_xy)
    a = np.arange(-half_yaw, half_yaw + 1e-9, step_yaw)
    dx, dy, da = (v.ravel() for v in np.meshgrid(d, d, a, indexing="ij"))
    seeds = poses[:, None, :]
    box = np.stack([dx, dy, da], axis=1)[None, :, :]
    return (seeds + box).reshape(-1, 3)


# --- the search -----------------------------------------------------------

def relocalize(field: MapField, pts_xy: np.ndarray, *,
               max_range_m: float = MAX_RANGE_M,
               min_score: float = MIN_SCORE,
               min_margin: float = MIN_MARGIN) -> Match:
    """Where, in `field`'s frame, does this scan come from?

    `pts_xy` are the scan points in the frame to be relocated (at boot: the
    lidar-odometry world frame, whose origin is wherever the rover happened
    to start). The returned (x, y, yaw) is the transform that carries that
    frame into the map frame: `p_map = R(yaw) @ p_scan + (x, y)`.
    """
    t0 = time.monotonic()
    res = field.res
    pts = np.asarray(pts_xy, dtype=np.float64)[:, :2]
    pts = pts[np.hypot(pts[:, 0], pts[:, 1]) <= max_range_m]
    # Thin by a STRIDE, never by cell. Eight revolutions of a moving rover
    # cost 3.9 s of search; something has to give. Measured on the flat
    # (26/08, checkpoint 002637 against run 003627, answer known = identity):
    #   3475 points, no thinning ......  6.0 cm   3.85 s
    #   1738 points, every other one ..  3.0 cm   1.53 s
    #    622 points, one per 2.5 cm cell ... 31.1 cm   1.40 s
    # Deduplicating by cell is what breaks it: the multiplicity IS the
    # measurement. A wall seen by forty rays from six positions must weigh
    # forty, not one, or a corridor slides under the scan. A stride thins
    # every wall in the same proportion and keeps the weighting intact.
    if len(pts) > MAX_SCAN_POINTS:
        pts = pts[::int(np.ceil(len(pts) / MAX_SCAN_POINTS))]
    n_in = len(pts)
    if n_in < MIN_POINTS or field.n_occupied < 200 or field.n_free < 200:
        return Match(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0), field.radius_m, 0.0, n_in,
                     time.monotonic() - t0, False,
                     f"not enough to match on ({n_in} pts, {field.n_occupied} occupied cells)")

    k_coarse = max(1, int(round(COARSE_M / res)))
    lik_coarse = _pool_max(field.lik, k_coarse)
    free_coarse = _pool_max(field.free.astype(np.uint8), k_coarse).astype(bool)
    res_coarse = res * k_coarse

    # candidate translations: the observed free cells of the saved map. The
    # rover boots somewhere the map already calls floor.
    cy, cx = np.nonzero(free_coarse)
    if len(cx) > MAX_COARSE_CANDIDATES:
        step = int(np.ceil(len(cx) / MAX_COARSE_CANDIDATES))
        cy, cx = cy[::step], cx[::step]
    xs = field.ox + (cx + 0.5) * res_coarse
    ys = field.oy + (cy + 0.5) * res_coarse

    # yaw step sized so a far point cannot slide past one coarse cell
    r90 = float(np.percentile(np.hypot(pts[:, 0], pts[:, 1]), 90))
    yaw_step = float(np.clip(res_coarse / max(r90, 1.0), math.radians(1.5), math.radians(8.0)))
    yaws = np.arange(-math.pi, math.pi, yaw_step)

    pts_coarse = _dedupe(pts, res_coarse)
    best_per_yaw = []
    for yaw in yaws:
        poses = np.stack([xs, ys, np.full(len(xs), yaw)], axis=1)
        sc = _score_poses(lik_coarse, res_coarse, field.ox, field.oy, pts_coarse, poses)
        best_per_yaw.append((poses, sc))
    poses = np.concatenate([p for p, _ in best_per_yaw])
    scores = np.concatenate([s for _, s in best_per_yaw])

    seeds = poses[_distinct_top(poses, scores, KEEP_COARSE)]

    # refinement: 0.10 m, then 0.05 m, then a 0.02 m polish. Each level keeps
    # distinct basins so the runner-up stays an honest rival, not a neighbour.
    pts_mid = _dedupe(pts, res * 2)
    cand = _grid_around(seeds, res_coarse / 2, res * 2, yaw_step, yaw_step / 4)
    sc = _score_poses(_pool_max(field.lik, 2), res * 2, field.ox, field.oy, pts_mid, cand)
    seeds = cand[_distinct_top(cand, sc, KEEP_FINE)]

    cand = _grid_around(seeds, res * 2, res, yaw_step / 4, yaw_step / 16)
    sc = _score_poses(field.lik, res, field.ox, field.oy, pts, cand)
    keep = _distinct_top(cand, sc, KEEP_FINE)
    cand, sc = cand[keep], sc[keep]

    polish = _grid_around(cand[:5], 0.04, 0.02, math.radians(0.6), math.radians(0.2))
    sc_p = _score_poses(field.lik, res, field.ox, field.oy, pts, polish)
    cand = np.concatenate([cand, polish])
    sc = np.concatenate([sc, sc_p])

    best_i = int(np.argmax(sc))
    best, score = cand[best_i], float(sc[best_i])

    # the best hypothesis that is somewhere ELSE
    rival, rival_pose = 0.0, (0.0, 0.0, 0.0)
    for i in np.argsort(-sc):
        if not _same_basin(cand[i], best):
            rival = float(sc[i])
            rival_pose = (float(cand[i][0]), float(cand[i][1]), float(cand[i][2]))
            break
    margin = float("inf") if rival <= 1e-6 else score / rival

    placed = _place(pts, (best[0], best[1], best[2]))
    gx = np.clip(np.floor((placed[:, 0] - field.ox) / res).astype(np.int32), 0, field.dist.shape[1] - 1)
    gy = np.clip(np.floor((placed[:, 1] - field.oy) / res).astype(np.int32), 0, field.dist.shape[0] - 1)
    d = field.dist[gy, gx]
    median_d = float(np.median(d))
    inlier = float((d <= WALL_INLIER_M).mean())

    accepted = score >= min_score and margin >= min_margin and median_d <= MAX_MEDIAN_M
    if accepted:
        reason = "ACCEPTED"
    elif score < min_score:
        reason = f"REJECTED: score {score:.3f} < {min_score} (the map does not explain this scan)"
    elif margin < min_margin:
        reason = f"REJECTED: margin {margin:.2f} < {min_margin} (two places fit as well)"
    else:
        reason = (f"REJECTED: walls {median_d * 100:.0f} cm apart > {MAX_MEDIAN_M * 100:.0f} cm "
                  "(they do not overlap)")

    return Match(float(best[0]), float(best[1]),
                 float(math.atan2(math.sin(best[2]), math.cos(best[2]))),
                 score, margin, rival, rival_pose, median_d, inlier, n_in,
                 time.monotonic() - t0, accepted, reason)

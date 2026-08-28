"""Cold bench for lidar_odometry's relocalization STATE MACHINE.

The map freezes while the rover looks for itself, and until the 28/08 audit
that freeze had no exit: a search opened by an anchor, a bump, a hand-carry or
a lost scan stayed "searching" for the rest of the run when the reference map
was missing, expired or never matchable - no `lidar` published, costmap frozen,
therefore no checkpoint, therefore no reference. The field name for it is the
2026-08-27 16:40 deadlock. Sections here, all in physical units (seconds,
metres, degrees):

  A. which checkpoints a run may see  - this run's, never a previous run's, and
                                        never a half-written *.npz.tmp.npz
  B. the freeze matrix                - every entry path (anchor / carried /
                                        lost / bump) x every reference state
                                        (present / absent / expired / foreign):
                                        no freeze ever outlives BOOT_GRACE_S
  C. PERSISTENT_MAP=0                 - the off switch: no state but "idle",
                                        ever, whatever the guards see
  D. what the costmap hears           - the give-up really unfreezes the real
                                        VectorCostMap.handle_reloc_frame
  E. roundtrip 73 deg                 - a scan taken at (1.20, -0.70, 73.0 deg)
                                        comes back out of the re-anchor path as
                                        73 deg of published heading

Needs dimOS (kiss-icp is faked, the messages and the relocalizer are real).

Run:  PYTHONPATH=. .venv/bin/python tests/test_lidar_odometry_cold.py
"""

import asyncio
import math
import os
import shutil
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

os.environ.setdefault("ODOM_GUARDS", "1")
os.environ["PERSISTENT_MAP"] = "1"

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: E402

from vector_dimos import lidar_odometry as LO  # noqa: E402
from vector_dimos import persistent_map  # noqa: E402
from vector_dimos.costmap2d import ScoredGrid  # noqa: E402
from vector_dimos.relocalize2d import Match  # noqa: E402

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- a private state directory ---------------------------------------------

TMP = tempfile.mkdtemp(prefix="lidar_odom_cold_")
persistent_map.CHECKPOINT_DIR = os.path.join(TMP, "checkpoints")
persistent_map.MAP_PATH = os.path.join(TMP, "persistent_map.npz")
persistent_map.KEEPOUT_PATH = os.path.join(TMP, "keepout.json")


def fresh_state_dir():
    shutil.rmtree(persistent_map.CHECKPOINT_DIR, ignore_errors=True)
    os.makedirs(persistent_map.CHECKPOINT_DIR, exist_ok=True)


def run_dir(stamp: float) -> str:
    """The directory costmap2d would create for a run started at `stamp`."""
    d = os.path.join(persistent_map.CHECKPOINT_DIR,
                     time.strftime(persistent_map.RUN_DIR_FMT, time.localtime(stamp)))
    os.makedirs(d, exist_ok=True)
    return d


def small_grid() -> ScoredGrid:
    """A 6 m grid with a few hundred occupied cells: a loadable checkpoint."""
    g = ScoredGrid(span_m=6.0)
    ring = np.array([[2.5 * math.cos(a), 2.5 * math.sin(a)] for a in np.linspace(0, 2 * math.pi, 400)])
    for vp in ((0.0, 0.0), (0.3, 0.2), (-0.3, -0.2)):
        g.lidar_revolution(ring, vp)
    return g


GRID = small_grid()


def checkpoint(directory: str, age_s: float = 0.0, name: str = "costmap_120000.npz") -> str:
    """A real checkpoint file in `directory`, `age_s` seconds old."""
    path = os.path.join(directory, name)
    GRID.save(path, (0.0, 0.0))
    if age_s:
        os.utime(path, (time.time() - age_s,) * 2)
    return path


# --- a fake kiss-icp: the pose it reports is the bench's to command ----------

class Threshold:
    def __init__(self, v=0.2):
        self.v = v

    def get_threshold(self):
        return self.v

    def update_model_deviation(self, corr):
        pass


class Preproc:
    def preprocess(self, pts, ts, prior):
        return pts


class LocalMap:
    def update(self, frame, pose):
        pass


class Registration:
    def __init__(self, kiss):
        self.kiss = kiss

    def align_points_to_map(self, points, voxel_map, initial_guess, max_correspondance_distance, kernel):
        # what the registration "found" on top of the prediction: identity is
        # honest driving, a metre is the jump the per-scan gate must refuse
        return np.asarray(initial_guess) @ self.kiss.slip


class Kiss:
    """Advances `step_m` per revolution, and hands the odometry `slip` as the
    correction its registration found."""

    def __init__(self, step_m=0.0, step_rad=0.0, slip=None, sigma=0.2):
        self.adaptive_threshold = Threshold(sigma)
        self.preprocessor = Preproc()
        self.local_map = LocalMap()
        self.registration = Registration(self)
        self.last_pose = np.eye(4)
        self.last_delta = LO._se2(step_m, 0.0, step_rad)
        self.slip = np.eye(4) if slip is None else slip

    def voxelize(self, frame):
        return frame, frame


class Sink:
    """An Out stream that keeps what was published, and when."""

    def __init__(self):
        self.msgs = []
        self.at = []

    def publish(self, msg):
        self.msgs.append(msg)
        self.at.append(time.monotonic())


LOOP = asyncio.new_event_loop()


def drive(o, n_revs: int, sleep_s: float = 0.0, pts=None):
    """`n_revs` lidar revolutions through the real handle_pointcloud."""
    if pts is None:
        pts = SCAN_FLAT
    msg = PointCloud2.from_numpy(pts.astype(np.float32), frame_id="lidar_link", timestamp=time.time())
    for _ in range(n_revs):
        LOOP.run_until_complete(o.handle_pointcloud(msg))
        o.seen_states.add(o._reloc_state)
        if sleep_s:
            time.sleep(sleep_s)


def odometry(kiss, run_started, frame="fresh"):
    o = LO.LidarOdometry.__new__(LO.LidarOdometry)
    o.world_frame, o.base_frame, o.lidar_frame = "world", "base_link", "lidar_link"
    o.log_every_s = 1e9                 # this bench reads publications, not the log
    o.use_gyro_prior = False
    o._lock = threading.Lock()
    o._kiss, o._cfg = kiss, None
    o._n, o._last_log, o._last_ms = 0, 0.0, 0.0
    o.pose2d = (0.0, 0.0, 0.0)
    o._gyro_acc, o._gyro_seen, o._gyro_totals = 0.0, False, np.zeros(3)
    o._wheel, o._prior_used, o._K = None, "cv", None
    o._pose_hist, o._wheel_hist, o._yaw_rate = [], [], 0.0
    o._depth_n, o._depth_pts_last, o._pending_cam_pts = 0, 0, None
    o._origin, o._frame = (0.0, 0.0, 0.0), frame
    o._reloc_state, o._reloc_reason = "idle", ""
    o._boot_deadline, o._reloc_deadline, o._reloc_next = 0.0, 0.0, 0.0
    o._anchor_ref, o._anchor_turn, o._scan_rejects = None, 0.0, 0
    o._reloc_gen, o._reloc_pts, o._reloc_thread, o._reloc_result = 0, [], None, None
    o._no_ref_tries, o._no_ref_logged = 0, 0.0
    o._reloc_ready_at, o._reloc_ready_ans, o._gave_up_pending = 0.0, False, False
    o._lost_since, o._carry_cooldown = 0.0, 0.0
    o._run_started = run_started
    o.odom, o.lidar, o.reloc_frame, o.tf, o.camera_floor = Sink(), Sink(), Sink(), Sink(), Sink()
    o.seen_states = {"idle"}
    return o


def states(o):
    return [str(m.frame_id).removeprefix("reloc:") for m in o.reloc_frame.msgs]


def freeze_episodes(o):
    """[(seconds frozen, closed?)] read off the published frame states."""
    out, start = [], None
    seq, at = states(o), o.reloc_frame.at
    for i, s in enumerate(seq):
        if s == "searching" and start is None:
            start = at[i]
        elif s != "searching" and start is not None:
            out.append((at[i] - start, True))
            start = None
    if start is not None:
        out.append((at[-1] - start, False))
    return out


def yaw_of(msg) -> float:
    """Published heading, in degrees."""
    return math.degrees(2.0 * math.atan2(msg.orientation.z, msg.orientation.w))


def verdict(x=0.0, y=0.0, yaw=0.0, accepted=True, score=0.9, margin=1.4):
    return Match(x, y, yaw, score, margin, score / margin, (9.0, 9.0, 0.0), 0.0, 1.0, 400, 0.1,
                 accepted, "ACCEPTED" if accepted else "REJECTED: margin too low")


SCAN_FLAT = np.stack([np.cos(np.linspace(0, 2 * math.pi, 360)) * 2.0,
                      np.sin(np.linspace(0, 2 * math.pi, 360)) * 2.0,
                      np.zeros(360)], axis=1)


# --- A. which checkpoints a run may see -------------------------------------

print("A. newest_checkpoint / current_run_dir: only THIS run, only whole files")
fresh_state_dir()
now = time.time()
prev = run_dir(now - 600)            # a run started 10 min ago
cur = run_dir(now)                   # this run
prev_ck = checkpoint(prev, age_s=60, name="costmap_110000.npz")   # its newest file, 1 min old

check("a run directory of its own is still readable", persistent_map.newest_checkpoint(prev) == prev_ck)
check("no run directory -> no reference", persistent_map.newest_checkpoint(None) is None)
check("this run's directory is the one stamped at its start",
      persistent_map.current_run_dir(now) == cur, f"{persistent_map.current_run_dir(now)}")
check("a run that never started has no directory", persistent_map.current_run_dir(0.0) is None)
check("the newest .npz on disk belongs to the PREVIOUS run -> not offered to this one",
      persistent_map.newest_checkpoint(persistent_map.current_run_dir(now)) is None)
check("a directory stamped 30 s before the start is this run (60 s of slack)",
      persistent_map.current_run_dir(now + 30) == cur)
check("a directory stamped 61 s before the start is a previous run",
      persistent_map.current_run_dir(now + 61) is None)

cur_ck = checkpoint(cur, age_s=0, name="costmap_120100.npz")
check("once this run checkpoints, that is the reference",
      persistent_map.newest_checkpoint(persistent_map.current_run_dir(now)) == cur_ck)
tmp_ck = checkpoint(cur, age_s=0, name="costmap_120200.npz.tmp.npz")
os.utime(tmp_ck, (time.time() + 30,) * 2)          # the freshest file on disk is a torn write
check("a half-written *.npz.tmp.npz never wins over a whole checkpoint",
      persistent_map.newest_checkpoint(cur) == cur_ck,
      os.path.basename(str(persistent_map.newest_checkpoint(cur))))

# the same rule seen from the module that consumes it
fresh_state_dir()
now = time.time()
prev, cur = run_dir(now - 600), run_dir(now)
checkpoint(prev, age_s=30)
o = odometry(Kiss(), now)
check("_reference_map: a fresh checkpoint from a PREVIOUS run is not a reference",
      o._reference_map("anchor") is None)
mine = checkpoint(cur, age_s=0, name="costmap_120100.npz")
o._reloc_ready_at = 0.0
check("_reference_map: this run's own checkpoint is", o._reference_map("anchor") == mine)
os.utime(mine, (time.time() - LO.CURRENT_MAP_MAX_AGE_S - 10,) * 2)
check(f"_reference_map: older than {LO.CURRENT_MAP_MAX_AGE_S:.0f} s is not the current map any more",
      o._reference_map("anchor") is None)


# --- B. the freeze matrix ---------------------------------------------------

print("B. every entry path x every reference state: a freeze always ends")

REAL_GRACE, REAL_RETRY, REAL_RELOC = LO.BOOT_GRACE_S, LO.RELOC_RETRY_S, LO.relocalize
LO.BOOT_GRACE_S = 0.5            # 600 s in the field; the bench measures the same seconds
LO.RELOC_RETRY_S = 0.0
LO.relocalize = lambda field, pts: verdict(accepted=False)   # never matchable: the 27/08 case

REVS, SLEEP = 60, 0.02           # 1.2 s of driving, at half the real revolution rate


def arm(entry, reference):
    """Set the disk up, build the module, fire `entry`, drive. Returns the module."""
    fresh_state_dir()
    started = time.time()
    prev, cur = run_dir(started - 600), run_dir(started)
    if reference == "present":
        checkpoint(cur, age_s=0)
    elif reference == "expired":
        checkpoint(cur, age_s=LO.CURRENT_MAP_MAX_AGE_S + 10)
    elif reference == "foreign":
        checkpoint(prev, age_s=5)
    kiss = Kiss(step_m=0.10 if entry == "anchor" else 0.0,
                slip=LO._se2(1.0, 0.0, 0.0) if entry == "lost" else None,
                sigma=2.0 if entry == "carried" else 0.2)
    o = odometry(kiss, started)
    if entry == "carried":
        o._lost_since = time.monotonic() - LO.LOST_SIGMA_S - 1.0
    if entry == "bump":
        LOOP.run_until_complete(o.handle_bump(None))
    drive(o, REVS, SLEEP)
    return o


for entry in ("anchor", "carried", "lost", "bump"):
    for reference in ("present", "absent", "expired", "foreign"):
        o = arm(entry, reference)
        eps = freeze_episodes(o)
        seq = states(o)
        worst = max((d for d, _ in eps), default=0.0)
        if reference == "present":
            # a trigger that keeps firing (driving on, still lost) may open a
            # SECOND search before the drive ends: what must hold is that no
            # single freeze outlives the grace, and that each one ends by
            # handing the map back
            unfrozen = sum(1 for s in seq if s != "searching")
            first = seq.index("gave_up") if "gave_up" in seq else -1
            check(f"{entry:8s} / map {reference:8s}: froze, and no freeze outlives "
                  f"{LO.BOOT_GRACE_S:.1f} s",
                  bool(eps) and worst <= LO.BOOT_GRACE_S + SLEEP + 0.05
                  and any(closed for _, closed in eps),
                  f"{len(eps)} freeze(s), longest {worst:.2f} s")
            check(f"{entry:8s} / map {reference:8s}: says reloc:gave_up, then maps in its own frame",
                  0 <= first < len(seq) - 1 and seq[first + 1] == "fresh"
                  and len(o.lidar.msgs) == unfrozen,
                  f"{seq.count('gave_up')} gave_up then {seq[first + 1] if 0 <= first < len(seq) - 1 else '?'}, "
                  f"{len(o.lidar.msgs)} clouds for {unfrozen} unfrozen revolutions")
        else:
            check(f"{entry:8s} / map {reference:8s}: never freezes, keeps mapping",
                  not eps and o.seen_states == {"idle"} and len(o.lidar.msgs) == REVS,
                  f"states {sorted(o.seen_states)}, {len(o.lidar.msgs)}/{REVS} clouds")

# the freeze is bounded by BOOT_GRACE_S, and that bound is set at entry
o = arm("bump", "present")
t0 = time.monotonic()
o2 = odometry(Kiss(), time.time())
o2._begin_relocalization("anchor", reset_kiss=False)
check(f"a search is given exactly BOOT_GRACE_S ({LO.BOOT_GRACE_S:.1f} s) to answer",
      abs(o2._reloc_deadline - (t0 + LO.BOOT_GRACE_S)) < 0.05,
      f"{o2._reloc_deadline - t0:.3f} s")

# no reference at all, three tries, done - without waiting out the grace
fresh_state_dir()
started = time.time()
cur = run_dir(started)
ck = checkpoint(cur, age_s=0)
o = odometry(Kiss(), started)
LOOP.run_until_complete(o.handle_bump(None))
check("bump with a reference on disk -> searching", o._searching)
os.remove(ck)                                   # the reference vanishes mid-search
drive(o, LO.RELOC_REVS * LO.NO_REF_MAX + 4)
check(f"reference gone: {LO.NO_REF_MAX} referenceless attempts and the search ends",
      o._reloc_state == "idle" and states(o).count("gave_up") == 1 and states(o)[-1] == "fresh",
      f"{states(o).count('gave_up')} gave_up in {len(states(o))} revolutions")
check("the map is written again straight after the give-up",
      len(o.lidar.msgs) >= 4, f"{len(o.lidar.msgs)} clouds published")

LO.BOOT_GRACE_S, LO.RELOC_RETRY_S, LO.relocalize = REAL_GRACE, REAL_RETRY, REAL_RELOC


# --- C. PERSISTENT_MAP=0 -----------------------------------------------------

print("C. PERSISTENT_MAP=0: the off switch (a checkpoint IS on disk, and is ignored)")
os.environ["PERSISTENT_MAP"] = "0"
try:
    for entry in ("anchor", "carried", "lost", "bump"):
        fresh_state_dir()
        started = time.time()
        checkpoint(run_dir(started), age_s=0)
        kiss = Kiss(step_m=0.10 if entry == "anchor" else 0.0,
                    slip=LO._se2(1.0, 0.0, 0.0) if entry == "lost" else None,
                    sigma=2.0 if entry == "carried" else 0.2)
        o = odometry(kiss, started)
        if entry == "carried":
            o._lost_since = time.monotonic() - LO.LOST_SIGMA_S - 1.0
        if entry == "bump":
            LOOP.run_until_complete(o.handle_bump(None))
        drive(o, 40)
        check(f"{entry:8s}: no state but idle, no freeze, every revolution mapped",
              o.seen_states == {"idle"} and set(states(o)) == {"fresh"} and len(o.lidar.msgs) == 40,
              f"states {sorted(o.seen_states)}, frames {sorted(set(states(o)))}, "
              f"{len(o.lidar.msgs)}/40 clouds")
    o = odometry(Kiss(), time.time())
    check("travelled 4 m with the flag off -> still idle", (drive(o, 40), o._reloc_state)[1] == "idle")
finally:
    os.environ["PERSISTENT_MAP"] = "1"


# --- D. what the costmap hears ----------------------------------------------

print("D. the give-up unfreezes the real VectorCostMap")
from vector_dimos.costmap2d import VectorCostMap  # noqa: E402

cm = VectorCostMap.__new__(VectorCostMap)
cm._grid, cm._frame, cm._frozen, cm._last_clear = None, None, False, None
cm._zones, cm._keepout_mtime = [], 0.0


def tell(frame_id):
    LOOP.run_until_complete(cm.handle_reloc_frame(type("M", (), {"frame_id": frame_id})()))


tell("reloc:fresh")
check("a fresh run starts unfrozen", not cm._frozen and cm._frame == "fresh")
tell("reloc:searching")
check("reloc:searching -> the costmap writes nothing", cm._frozen)
tell("reloc:gave_up")
check("reloc:gave_up alone is only an announcement (unknown state, still frozen)", cm._frozen)
tell("reloc:fresh")
check("the frame republished on the very next revolution -> writing resumes", not cm._frozen)


# --- E. roundtrip: 73 deg in, 73 deg out ------------------------------------

print("E. roundtrip: a scan taken at (1.20, -0.70, +73.0 deg) through the re-anchor path")


def flat_walls():
    """An L-shaped flat with a pillar: asymmetric, so one place beats the other."""
    walls = [((-4, -3), (4, -3)), ((4, -3), (4, 1)), ((4, 1), (1, 1)),
             ((1, 1), (1, 4)), ((1, 4), (-4, 4)), ((-4, 4), (-4, -3))]
    px, py, half = 2.0, -1.5, 0.2
    walls += [((px - half, py - half), (px + half, py - half)),
              ((px + half, py - half), (px + half, py + half)),
              ((px + half, py + half), (px - half, py + half)),
              ((px - half, py + half), (px - half, py - half))]
    return walls


def revolution(walls, origin, n_rays=400, max_r=12.0, min_r=0.15):
    ox, oy = origin
    hits = []
    for a in np.linspace(-math.pi, math.pi, n_rays, endpoint=False):
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for (x1, y1), (x2, y2) in walls:
            ex, ey = x2 - x1, y2 - y1
            den = dx * ey - dy * ex
            if abs(den) < 1e-12:
                continue
            t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
            u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
            if t > min_r and 0.0 <= u <= 1.0 and (best is None or t < best):
                best = t
        if best is not None and best < max_r:
            hits.append((ox + dx * best, oy + dy * best))
    return np.array(hits)


TRUE = (1.20, -0.70, math.radians(73.0))
walls = flat_walls()
flat = ScoredGrid(span_m=24.0)
for vp in ((0.0, 0.0), (0.6, 0.4), (-0.8, 0.6), (1.5, -1.5)):
    flat.lidar_revolution(revolution(walls, vp), vp)

fresh_state_dir()
started = time.time()
cur = run_dir(started)
flat.save(os.path.join(cur, "costmap_121500.npz"), (0.0, 0.0))     # what this run just checkpointed

world = revolution(walls, (TRUE[0], TRUE[1]))
c, s = math.cos(-TRUE[2]), math.sin(-TRUE[2])
d = world - np.array([TRUE[0], TRUE[1]])
body = np.stack([c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]], axis=1)   # what the lidar sees
scan = np.concatenate([body, np.zeros((len(body), 1))], axis=1)

o = odometry(Kiss(), started)
LOOP.run_until_complete(o.handle_bump(None))       # a contact: re-anchor against the current map
check("the bump froze the map (a reference exists)", o._searching and len(o.lidar.msgs) == 0)
drive(o, LO.RELOC_REVS, pts=scan)
if o._reloc_thread is not None:
    o._reloc_thread.join(30.0)
drive(o, 2, pts=scan)

pose = o.odom.msgs[-1]
err_xy = math.hypot(pose.position.x - TRUE[0], pose.position.y - TRUE[1])
err_yaw = abs(yaw_of(pose) - 73.0)
print(f"     published pose ({pose.position.x:+.3f}, {pose.position.y:+.3f}, {yaw_of(pose):+.2f} deg)")
check("73.0 deg in -> 73 deg out (within 2 deg)", err_yaw < 2.0, f"{err_yaw:.2f} deg off")
check("(1.20, -0.70) m in -> the same place out (within 5 cm)", err_xy < 0.05, f"{err_xy * 100:.1f} cm")
check("re-anchored -> unfrozen, mapping again, still the fresh frame",
      o._reloc_state == "idle" and len(o.lidar.msgs) == 2 and o._frame == "fresh",
      f"{len(o.lidar.msgs)} clouds after the match")
check("the origin the search wrote is the pose it found",
      abs(math.degrees(o._origin[2]) - 73.0) < 2.0, f"origin yaw {math.degrees(o._origin[2]):+.2f} deg")

shutil.rmtree(TMP, ignore_errors=True)
print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

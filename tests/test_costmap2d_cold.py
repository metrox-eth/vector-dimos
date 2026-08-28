"""Cold bench for ScoredGrid - the field failure stories, written as numbers.

Known input -> known output, in metres:
  1. a table leg at (1.0, 0) seen from two places -> occupied; the floor along
     the ray (0.3..0.9 m) -> free; nothing else touched
  2. the ramp: 50 hits from the SAME spot -> never occupied (one observation)
  3. the office chair: occupied, then 5 lidar revolutions see through it -> free
  4. the telescope under the desk (camera only): lidar rays passing over it
     change nothing; the camera seeing floor there 5 times -> free
  5. the ceiling: 30 hits from 30 places -> score 10, then 13 misses -> free
     (bounded unlearning, never a wall for a day)
  6. the table leg of run B (2026-08-26): thinner than a cell, seen ten times, found
     at low = -3 - the floor sampled on its own cell in the NEXT frame erased
     every hit. A cell hit by the camera is deaf to floor samples for
     LOW_HIT_PROTECT_S; past that window it is forgotten exactly as before.
  7. the parked rover (2026-08-28): a wall at 1.0 m raised to HIT_CAP by 12
     viewpoints, then one more revolution from the spot it is parked on ->
     still HIT_CAP, still occupancy 100 (the viewpoint rule caps the
     increase, it never lowers a score)
  8. the torn save (2026-08-28): the scratch file of an interrupted checkpoint
     is not named .npz, so no .npz scan can pick a truncated map
  9. midnight (2026-08-28): 23xxxx and 00xxxx checkpoints in one run
     directory -> the pruner keeps the freshest by mtime, not the oldest by name
 10. the person at 0.37 m (2026-08-28): a camera point exactly at the lidar's
     tag height stays a CAMERA point - LOW layer, moving-object gate, and the
     camera can take it back
 11. the mount offset (2026-08-28): camera rays start 0.20 m behind the base,
     where the camera is - the carve band opens at 0.45 m, not 0.60 m
"""

import asyncio
import math
import os
import tempfile
import time

import numpy as np

from vector_dimos import persistent_map
from vector_dimos.costmap2d import (
    CAMERA_X_BASE_M, FREE_FLOOR, HIT_CAP, LIDAR_Z_M, LOW_HIT_PROTECT_S, OCCUPIED_AT,
    SAVE_TMP_SUFFIX, ScoredGrid, VectorCostMap, camera_xy, lidar_returns, prune_checkpoints,
)

LEG = np.array([[1.0, 0.0]])


def fresh() -> ScoredGrid:
    return ScoredGrid(span_m=6.0)


def _cloud(pts: np.ndarray):
    """A world cloud on the wire, float32 like the real one."""
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
    return PointCloud2.from_numpy(pts.astype(np.float32), frame_id="world", timestamp=time.time())


def test_leg_needs_two_viewpoints_and_ray_is_free() -> None:
    """Rule under test (2026-08-26): an obstacle is REAL when seen from two
    viewpoints. A passer-by hammered from one parked spot must never become a
    wall - the evening flights were spent walled in by exactly such cells."""
    g = fresh()
    for _ in range(10):
        g.lidar_revolution(LEG, (0.0, 0.0))
    assert g.value_at(1.0, 0.0) == 0, "any number of hits from ONE spot: not a wall"
    g.lidar_revolution(LEG, (0.0, 0.15))          # second viewpoint (>= NEW_VIEWPOINT_M)
    assert g.value_at(1.0, 0.0) == 100, "one hit from a SECOND viewpoint: obstacle"
    for x in (0.3, 0.5, 0.7, 0.9):
        assert g.value_at(x, 0.0) == 0, f"floor along the ray at {x} m must be free"
    assert g.value_at(1.3, 0.0) == -1, "behind the leg: never seen"
    print("  10 hits one spot -> free; +1 hit second viewpoint -> occupied; ray -> free")


def test_ramp_hit_from_one_spot_never_becomes_a_wall() -> None:
    g = fresh()
    for _ in range(50):
        g.lidar_revolution(LEG, (0.0, 0.0))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.value_at(1.0, 0.0) == 0 and g.lidar[gy[0], gx[0]] == 1, "same spot forever: capped BELOW occupied"
    g.lidar_revolution(LEG, (0.0, 0.15))
    assert g.value_at(1.0, 0.0) == 100, "the second viewpoint makes it a wall"
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
    assert n <= 4, n
    print(f"  50 hits from the same spot -> score 2, forgotten after {n} clear revolutions (never a wall)")


def test_chair_moved_is_forgotten_by_lidar_rays() -> None:
    g = fresh()
    for i in range(3):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    assert g.value_at(1.0, 0.0) == 100
    far = np.array([[3.0, 0.0]])                      # the chair is gone: the ray now reaches the wall at 3 m
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(far, (0.0, 0.0)); n += 1
        assert n < 20
    assert g.value_at(1.0, 0.0) == 0
    print(f"  chair (score 3) -> free after {n} revolutions seeing through it")


def test_low_object_only_the_camera_can_forget() -> None:
    g = fresh()
    box = np.array([[1.0, 0.0]])
    for i in range(3):
        g.camera_obstacles(box, (0.0, 0.15 * i))
    assert g.value_at(1.0, 0.0) == 100
    for _ in range(20):
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0))   # lidar rays pass over the box
    assert g.value_at(1.0, 0.0) == 100, "a lidar ray at 0.37 m does not prove the box is gone"
    n = 0
    later = __import__("time").monotonic() + LOW_HIT_PROTECT_S + 1.0   # past the fresh-hit window
    while g.value_at(1.0, 0.0) == 100:
        g.camera_floor(box, now=later + n); n += 1
        assert n < 10
    print(f"  low box: 20 lidar rays over it -> still occupied; camera floor x{n} -> free")


def test_thin_leg_survives_the_floor_sampled_beside_it() -> None:
    """Run B, 2026-08-26: a table leg thinner than a 5 cm cell, seen ten times, sat
    at low = -3. The camera calls the cell an obstacle in one frame and bare
    floor in the next (5 cm of quantisation), and the two cancel forever."""
    g = fresh()
    leg = np.array([[1.0, 0.0]])
    t = 1000.0                                     # an injected clock, in seconds
    g.camera_obstacles(leg, (0.0, 0.00), now=t - 0.1)   # seed frame (the anti-moving-object gate)
    g.camera_obstacles(leg, (0.0, 0.00), now=t)
    g.camera_obstacles(leg, (0.0, 0.15), now=t + 0.1)
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.low[gy[0], gx[0]] == 2 and g.value_at(1.0, 0.0) == 100, "two hits, two viewpoints"
    for i in range(25):                            # 2.5 s of floor samples on its own cell
        g.camera_floor(leg, now=t + 0.2 + 0.1 * i)
    assert g.low[gy[0], gx[0]] == 2, "a leg seen 0.1 s ago is not erased by the floor beside it"
    assert g.value_at(1.0, 0.0) == 100
    assert g.lidar[gy[0], gx[0]] == FREE_FLOOR, "the LIDAR layer still takes every floor miss"
    n = 0                                          # nothing hits it any more
    while g.value_at(1.0, 0.0) == 100:
        n += 1
        g.camera_floor(leg, now=t + LOW_HIT_PROTECT_S + n)
        assert n < 10
    assert g.low[gy[0], gx[0]] < OCCUPIED_AT
    print(f"  thin leg: 25 floor samples inside the {LOW_HIT_PROTECT_S:.0f} s window -> still "
          f"occupied; {n} past it -> forgotten (legitimate unlearning untouched)")


def test_reinforcement_is_capped_and_unlearning_bounded() -> None:
    g = fresh()
    for i in range(30):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.lidar[gy[0], gx[0]] == HIT_CAP
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
    assert n <= 2 * HIT_CAP and g.lidar[gy[0], gx[0]] >= FREE_FLOOR
    print(f"  30 viewpoints -> score capped at {HIT_CAP}; forgotten after {n} clear revolutions (bounded)")


def test_parked_rover_never_erases_an_established_wall() -> None:
    """28/08 audit: the viewpoint rule capped the VALUE instead of the increase,
    so a hit from a spot a cell already knew forced it back to OCCUPIED_AT - 1.
    A wall raised to HIT_CAP by 12 approach viewpoints collapsed to 1 on the
    very next revolution taken from a parked rover, and occupancy() published
    free floor 0.6 m in front of the wall (the checkpoint saved the hole too)."""
    g = fresh()
    for i in range(12):                                # approach: 12 viewpoints, 0.15 m apart
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.lidar[gy[0], gx[0]] == HIT_CAP and g.value_at(1.0, 0.0) == 100, "12 viewpoints -> a wall"
    parked = (0.0, 0.15 * 11)                          # the rover stops where it last looked from
    g.lidar_revolution(LEG, parked)
    assert g.lidar[gy[0], gx[0]] == HIT_CAP, "a same-viewpoint hit adds nothing - and takes nothing"
    assert g.value_at(1.0, 0.0) == 100, "the wall in front of a parked rover stays a wall"
    # driving slowly is the same failure: 0.2 m/s at 10 Hz = 0.02 m per
    # revolution, so 4 revolutions out of 5 come from a viewpoint the cell knows
    for k in range(1, 26):                             # 0.5 m of creep
        g.lidar_revolution(LEG, (0.02 * k, parked[1]))
        assert g.value_at(1.0, 0.0) == 100, f"wall published FREE while creeping, revolution {k}"
    assert g.lidar[gy[0], gx[0]] == HIT_CAP
    # same rule on the LOW layer: the camera can hold an obstacle too
    c = fresh()
    box = np.array([[1.0, 0.0]])
    t0 = 500.0
    c.camera_obstacles(box, (0.0, 0.0), now=t0)        # seed frame (the anti-moving-object gate)
    for i in range(12):
        c.camera_obstacles(box, (0.0, 0.15 * i), now=t0 + 0.1 * i)
    assert c.low[gy[0], gx[0]] == HIT_CAP and c.value_at(1.0, 0.0) == 100
    c.camera_obstacles(box, (0.0, 0.15 * 11), now=t0 + 1.3)
    assert c.low[gy[0], gx[0]] == HIT_CAP, "the LOW layer is capped the same way"
    assert c.value_at(1.0, 0.0) == 100, "a camera obstacle survives a parked rover too"
    print(f"  wall at 1.0 m, 12 viewpoints -> score {HIT_CAP}; parked + 1 revolution and 0.5 m of "
          f"creep -> still {HIT_CAP}, occupancy 100 (lidar and LOW)")


def test_checkpoint_roundtrip(tmp_path=None) -> None:
    import os, tempfile
    g = fresh()
    for i in range(3):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    d = tempfile.mkdtemp(); path = os.path.join(d, "ck.npz")
    size = g.save(path, (0.3, 0.0))
    g2 = ScoredGrid.load(path)
    assert g2.value_at(1.0, 0.0) == 100 and np.array_equal(g2.occupancy(), g.occupancy())
    print(f"  checkpoint saved ({size / 1024:.0f} kB for a {g.n}x{g.n} grid) and reloaded identical")


def test_ghost_dies_when_the_camera_sees_through_it() -> None:
    """Requirement (2026-08-26): the map must correct itself every time the
    RealSense passes over it. A ghost low cell at 2.0 m, no longer confirmed,
    dies as soon as the camera looks through that spot at low height."""
    g = fresh()
    t0 = 100.0
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)   # seed frame (the anti-moving-object gate)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.15), now=t0)   # second viewpoint
    assert g.value_at(2.0, 0.0) == 100, "the ghost starts as a real-looking obstacle"
    later = t0 + LOW_HIT_PROTECT_S + 1.0
    # the camera now sees a box at 3 m, 0.30 m tall: its ray crosses the ghost
    # cell at z = 0.56 + (0.30 - 0.56) * 2/3 = 0.39 m - inside the carve band
    g.camera_rays(np.array([[3.0, 0.0, 0.30]]), (0.0, 0.0), now=later)
    assert g.value_at(2.0, 0.0) == 0, "one look through the ghost at low height kills it"
    print("  ghost at 2.0 m, no longer confirmed -> erased by one ray seen through it")


def test_high_ray_never_erases_a_low_box() -> None:
    g = fresh()
    t0 = 100.0
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)   # seed frame (the anti-moving-object gate)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.15), now=t0)
    later = t0 + LOW_HIT_PROTECT_S + 1.0
    # a shelf edge at 3 m, 1.25 m up: its ray crosses the box cell at
    # z = 0.56 + (1.25 - 0.56) * 2/3 = 1.02 m - flying OVER the box
    g.camera_rays(np.array([[3.0, 0.0, 1.25]]), (0.0, 0.0), now=later)
    assert g.value_at(2.0, 0.0) == 100, "a ray at 1 m height says nothing about a low box"
    print("  ray over the box (1.0 m up at the cell) -> box untouched")


def test_fresh_hits_are_protected_from_carving() -> None:
    g = fresh()
    t0 = 100.0
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)   # seed frame (the anti-moving-object gate)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.15), now=t0)
    # 1 s later (inside LOW_HIT_PROTECT_S): a real thing is re-hit every
    # frame, so a stray ray through its cell must not erase it
    g.camera_rays(np.array([[3.0, 0.0, 0.30]]), (0.0, 0.0), now=t0 + 1.0)
    assert g.value_at(2.0, 0.0) == 100, "a cell the camera just confirmed is immune to carving"
    print("  ray through a just-confirmed cell -> protected, obstacle stays")


def test_floor_ray_carves_the_low_corridor() -> None:
    """A floor point at 3 m proves the whole low corridor before it: the ray
    descends from 0.56 m at the camera to 0 at the floor, crossing the carve
    band between 0.59 m and 2.46 m of range."""
    g = fresh()
    t0 = 100.0
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)   # seed frame (the anti-moving-object gate)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.0), now=t0)
    g.camera_obstacles(np.array([[2.0, 0.0]]), (0.0, 0.15), now=t0)
    later = t0 + LOW_HIT_PROTECT_S + 1.0
    g.camera_rays(np.array([[3.0, 0.0, 0.0]]), (0.0, 0.0), now=later)
    assert g.value_at(2.0, 0.0) == 0, "bare floor seen at 3 m erases the stale cell at 2 m"
    assert g.value_at(3.0, 0.0) == -1, "the ray stops short of its own endpoint"
    print("  floor at 3 m -> stale cell at 2 m erased; endpoint untouched")


def test_camera_rays_never_touch_the_lidar_layer() -> None:
    g = fresh()
    g.lidar_revolution(np.array([[2.0, 0.0]]), (0.0, 0.0))
    g.lidar_revolution(np.array([[2.0, 0.0]]), (0.0, 0.15))    # a real lidar wall
    assert g.value_at(2.0, 0.0) == 100
    g.camera_rays(np.array([[3.0, 0.0, 0.30]]), (0.0, 0.0), now=200.0)
    assert g.value_at(2.0, 0.0) == 100, "only a lidar ray may retract a lidar claim"
    print("  camera ray through a lidar wall -> wall stays (layer doctrine)")


def test_moving_object_never_writes_the_map() -> None:
    """Requirement (2026-08-27): the RealSense must not write moving things. A cell
    only takes a camera hit if the PREVIOUS frame also saw an obstacle there -
    a rolling vacuum or a walking person moves on before the second look."""
    g = fresh()
    # the Xiaomi rolls by: a different cell every frame, two viewpoints anyway
    for k, x in enumerate((1.0, 1.2, 1.4, 1.6, 1.8)):
        g.camera_obstacles(np.array([[x, 0.0]]), (0.0, 0.15 * k))
    for x in (1.0, 1.2, 1.4, 1.6, 1.8):
        assert g.value_at(x, 0.0) != 100, f"a moving object left a wall at {x} m"
    # a box STANDS there: same cell three frames, two viewpoints
    g2 = fresh()
    g2.camera_obstacles(np.array([[1.0, 0.0]]), (0.0, 0.0))
    g2.camera_obstacles(np.array([[1.0, 0.0]]), (0.0, 0.0))
    g2.camera_obstacles(np.array([[1.0, 0.0]]), (0.0, 0.15))
    assert g2.value_at(1.0, 0.0) == 100, "a standing box must still map"
    print("  rolling object -> no wall anywhere on its path; standing box -> mapped")


def test_walled_in_map_is_measured_as_a_prison() -> None:
    """2026-08-26 evening: a run that starts surrounded by ghost cells must never
    hand its prison over as the persistent map. The gate reads the reachable
    free area around the rover."""
    g = fresh()
    g.body_clear((0.0, 0.0, 0.0))                       # the rover certifies its own footprint
    # a closed box of walls 0.5 m around the rover, each cell seen from two places
    edge = np.arange(-0.5, 0.5 + 1e-9, 0.025)
    box = np.concatenate([
        np.stack([edge, np.full_like(edge, -0.5)], 1), np.stack([edge, np.full_like(edge, 0.5)], 1),
        np.stack([np.full_like(edge, -0.5), edge], 1), np.stack([np.full_like(edge, 0.5), edge], 1)])
    g.lidar_revolution(box, (0.0, 0.0))
    g.lidar_revolution(box, (0.0, 0.15))
    walled = g.reachable_free_m2((0.0, 0.0))
    assert walled is not None and walled < 3.0, f"a 1x1 m box is a prison, got {walled}"
    g2 = fresh()
    g2.body_clear((0.0, 0.0, 0.0))
    ang = np.radians(np.arange(0, 360, 2.0))
    room = np.stack([2.5 * np.cos(ang), 2.5 * np.sin(ang)], 1)   # a 5 m round room
    g2.lidar_revolution(room, (0.0, 0.0))
    g2.lidar_revolution(room, (0.0, 0.15))
    open_m2 = g2.reachable_free_m2((0.0, 0.0))
    assert open_m2 is not None and open_m2 > 3.0, f"a 5 m room is not a prison, got {open_m2}"
    print(f"  1x1 m box -> {walled:.1f} m2 (prison, promotion refused); 5 m room -> {open_m2:.1f} m2 (fine)")


def test_torn_save_is_invisible_to_every_npz_scan() -> None:
    """28/08 audit: the scratch file of the atomic save was called
    `<name>.npz.tmp.npz` (numpy appends .npz to any other suffix), so a save cut
    by a battery death left a TRUNCATED file that every `.npz` scan accepted -
    and being the freshest by mtime it won."""
    d = tempfile.mkdtemp()
    g = fresh()
    g.lidar_revolution(LEG, (0.0, 0.0)); g.lidar_revolution(LEG, (0.0, 0.15))
    path = os.path.join(d, "costmap_120000.npz")
    g.save(path, (0.0, 0.0))
    assert os.listdir(d) == ["costmap_120000.npz"], "a finished save leaves the map and nothing else"
    assert not SAVE_TMP_SUFFIX.endswith(".npz")
    # the next save dies mid-write: its scratch file stays, freshest on disk
    torn = os.path.join(d, "costmap_120030.npz" + SAVE_TMP_SUFFIX)
    with open(torn, "wb") as fh:
        fh.write(b"PK\x03\x04truncated")
    os.utime(torn, (time.time() + 60, time.time() + 60))
    scan = sorted(f for f in os.listdir(d) if f.endswith(".npz"))    # the shape of every scan in the tree
    assert scan == ["costmap_120000.npz"], f"a torn write must not look like a map: {scan}"
    assert persistent_map.newest_checkpoint(d) == path
    assert ScoredGrid.load(path).value_at(1.0, 0.0) == 100, "the map picked is the finished one"
    # the pre-fix name, same torn bytes: the scan takes it, and it is the newest
    old_style = "costmap_120030.npz.tmp.npz"
    os.rename(torn, os.path.join(d, old_style))
    assert sorted(f for f in os.listdir(d) if f.endswith(".npz")) == ["costmap_120000.npz", old_style]
    print(f"  torn save named *{SAVE_TMP_SUFFIX} -> invisible to a .npz scan (as *.npz.tmp.npz: picked, and newest)")


def test_checkpoint_retention_survives_midnight() -> None:
    """28/08 audit: checkpoints are stamped %H%M%S and the run directory once,
    so a run across midnight holds 23xxxx and 00xxxx together. Sorted by NAME
    the two files just written come first, and the pruner deleted exactly
    those - every checkpoint of the rest of the night."""
    d = tempfile.mkdtemp()
    names = ["costmap_235900.npz", "costmap_235930.npz", "costmap_000000.npz", "costmap_000030.npz"]
    for i, f in enumerate(names):                      # written in this order, 30 s apart
        p = os.path.join(d, f)
        with open(p, "wb") as fh:
            fh.write(b"x")
        os.utime(p, (1.0e9 + 30 * i, 1.0e9 + 30 * i))
    pre_fix_deletes = sorted(f for f in os.listdir(d) if f.endswith(".npz"))[:-2]
    assert pre_fix_deletes == names[2:], "the name sort deletes the two files written after midnight"
    scratch = "costmap_000100.npz" + SAVE_TMP_SUFFIX
    with open(os.path.join(d, scratch), "wb") as fh:
        fh.write(b"x")
    gone = prune_checkpoints(d, keep=2)
    left = sorted(os.listdir(d))
    assert gone == 2 and [f for f in left if f.endswith(".npz")] == sorted(names[2:]), left
    assert scratch in left, "a scratch file is not a checkpoint: never counted, never pruned"
    assert persistent_map.newest_checkpoint(d) == os.path.join(d, "costmap_000030.npz")
    print("  23:59 + 00:00 checkpoints, keep 2 -> the two MIDNIGHT ones survive (by name: they were the deleted ones)")


def test_camera_point_at_the_lidar_height_stays_a_camera_point() -> None:
    """28/08 audit: the two sensors share one cloud and the lidar's returns are
    tagged by height (z = 0.37 to the last bit of a float32). Read as a 5 mm
    BAND, that tag stole every camera point between 0.365 and 0.375 m: written
    into the lidar layer past the two-frame moving-object gate, and beyond the
    reach of camera_rays, which only carves `low`."""
    person_z = 0.372                                  # a person's side, 0.372 m above the floor
    assert abs(person_z - LIDAR_Z_M) < 0.005, "this is the height the old 5 mm window claimed"
    assert not lidar_returns(np.array([[1.0, 0.0, person_z]]))[0]
    assert lidar_returns(np.array([[1.0, 0.0, np.float64(np.float32(LIDAR_Z_M))]]))[0], "a real lidar return, after the float32 round trip"
    m = VectorCostMap()
    m._frame, m._grid, m._pose_yaw = "fresh", fresh(), 0.0
    old = time.monotonic() - 10.0                     # the hits are outside the 3 s protection
    cloud = np.array([[2.0, 0.0, np.float32(LIDAR_Z_M)], [1.0, 0.0, person_z]], dtype=np.float32)
    msg = _cloud(cloud)
    for pose in ((0.0, 0.0), (0.0, 0.0), (0.0, 0.15)):          # two viewpoints, three frames
        m._pose_xy = pose
        asyncio.run(m.handle_lidar(msg))
    gx, gy = m._grid.cell(np.array([1.0]), np.array([0.0]))
    assert m._grid.low[gy[0], gx[0]] == 2, "the person is in the LOW layer, gated and retractable"
    assert m._grid.lidar[gy[0], gx[0]] <= 0, "the lidar layer never claimed a cell the lidar never returned"
    assert m._grid.value_at(1.0, 0.0) == 100
    m._grid._last_low_hit[:] = old                    # the person walked off: nothing confirms the cell any more
    m._pose_xy = (0.0, 0.0)                           # back on the axis, looking at the floor 3 m ahead
    asyncio.run(m.handle_camera_floor(_cloud(np.array([[3.0, 0.0, 0.0]], dtype=np.float32))))
    assert m._grid.value_at(1.0, 0.0) == 0, "the camera looked through it: a camera claim can be taken back"
    print(f"  camera point at {person_z} m -> LOW layer (lidar layer untouched), and one look through it -> free")


def test_camera_rays_start_at_the_camera_not_the_base() -> None:
    """28/08 audit: camera_rays started at CAMERA_HEIGHT_M (0.56 m, half the
    mount constant) but from the BASE position - 0.20 m in front of the camera,
    while the endpoints carried the full offset. Three cells of corridor were
    never carved, and off-axis rays swept beside the cells the camera had
    really looked through."""
    assert camera_xy((0.0, 0.0), 0.0) == (CAMERA_X_BASE_M, 0.0), "facing +x: the camera is 0.20 m behind"
    x, y = camera_xy((1.0, 2.0), math.pi / 2)
    assert abs(x - 1.0) < 1e-9 and abs(y - 1.80) < 1e-9, "turned 90 deg: the offset turns with the rover"
    # a floor sample 3.0 m ahead of the base: the carve band (z < 0.45) is
    # crossed 0.43 m ahead of the base, not 0.59 m - one cell resolution: 0.45 vs 0.60
    def first_carved(origin: tuple[float, float]) -> float:
        g = fresh()
        g.camera_rays(np.array([[3.0, 0.0, 0.0]]), origin, now=1000.0)
        return g.ox + np.nonzero(g.low < 0)[1].min() * g.res
    from_camera, from_base = first_carved(camera_xy((0.0, 0.0), 0.0)), first_carved((0.0, 0.0))
    assert abs(from_camera - 0.45) < 1e-9 and abs(from_base - 0.60) < 1e-9, (from_camera, from_base)
    # a ghost in the 0.15 m the base-cast ray flew over, at the true camera range
    ghost = np.array([[0.525, 0.0]])
    m = VectorCostMap()
    m._frame, m._grid, m._pose_xy, m._pose_yaw = "fresh", fresh(), (0.0, 0.0), 0.0
    old = time.monotonic() - 10.0
    m._grid.camera_obstacles(ghost, (0.0, 0.00), now=old)      # seed frame (the anti-moving-object gate)
    m._grid.camera_obstacles(ghost, (0.0, 0.00), now=old)
    m._grid.camera_obstacles(ghost, (0.0, 0.15), now=old)
    assert m._grid.value_at(0.525, 0.0) == 100
    asyncio.run(m.handle_camera_floor(_cloud(np.array([[3.0, 0.0, 0.0]], dtype=np.float32))))
    assert m._grid.value_at(0.525, 0.0) == 0, "the ray from the camera crosses the ghost cell low"
    g = fresh()                                                 # the same scene, ray cast from the base
    g.camera_obstacles(ghost, (0.0, 0.00), now=old); g.camera_obstacles(ghost, (0.0, 0.00), now=old)
    g.camera_obstacles(ghost, (0.0, 0.15), now=old)
    g.camera_rays(np.array([[3.0, 0.0, 0.0]]), (0.0, 0.0), now=time.monotonic())
    assert g.value_at(0.525, 0.0) == 100, "cast from the base, the same look leaves the ghost standing"
    print(f"  floor at 3 m: carving starts at {from_camera:.2f} m from the camera vs {from_base:.2f} m from the base; "
          "ghost at 0.53 m killed only by the camera-cast ray")


def test_revolution_cost() -> None:
    import time
    g = fresh()
    ang = np.radians(np.arange(0, 360, 0.9)); r = np.random.default_rng(0).uniform(0.5, 12.0, len(ang))
    hits = np.stack([r * np.cos(ang), r * np.sin(ang)], 1)
    g = ScoredGrid(span_m=24.0)
    t0 = time.perf_counter()
    for i in range(10):
        g.lidar_revolution(hits, (0.01 * i, 0.0))
    ms = (time.perf_counter() - t0) / 10 * 1000
    assert ms < 40, ms
    print(f"  one revolution (400 rays, 0.5-12 m, carving every other one) = {ms:.0f} ms (was 227)")



if __name__ == "__main__":
    for t in (test_leg_needs_two_viewpoints_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
              test_chair_moved_is_forgotten_by_lidar_rays, test_low_object_only_the_camera_can_forget,
              test_thin_leg_survives_the_floor_sampled_beside_it,
              test_reinforcement_is_capped_and_unlearning_bounded,
              test_parked_rover_never_erases_an_established_wall, test_checkpoint_roundtrip,
              test_ghost_dies_when_the_camera_sees_through_it, test_high_ray_never_erases_a_low_box,
              test_fresh_hits_are_protected_from_carving, test_floor_ray_carves_the_low_corridor,
              test_camera_rays_never_touch_the_lidar_layer, test_moving_object_never_writes_the_map,
              test_walled_in_map_is_measured_as_a_prison,
              test_torn_save_is_invisible_to_every_npz_scan,
              test_checkpoint_retention_survives_midnight,
              test_camera_point_at_the_lidar_height_stays_a_camera_point,
              test_camera_rays_start_at_the_camera_not_the_base,
              test_revolution_cost):
        print(t.__name__); t()
    print("TEST PASSED")

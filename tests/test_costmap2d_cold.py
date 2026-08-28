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
"""

import numpy as np

from vector_dimos.costmap2d import (
    FREE_FLOOR, HIT_CAP, LOW_HIT_PROTECT_S, OCCUPIED_AT, ScoredGrid,
)

LEG = np.array([[1.0, 0.0]])


def fresh() -> ScoredGrid:
    return ScoredGrid(span_m=6.0)


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
              test_revolution_cost):
        print(t.__name__); t()
    print("TEST PASSED")

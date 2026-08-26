"""Cold bench for ScoredGrid - metrox's two Xiaomi stories in numbers.

Known input -> known output, in metres:
  1. a table leg at (1.0, 0) seen from two places -> occupied; the floor along
     the ray (0.3..0.9 m) -> free; nothing else touched
  2. the ramp: 50 hits from the SAME spot -> never occupied (one observation)
  3. the office chair: occupied, then 5 lidar revolutions see through it -> free
  4. the telescope under the desk (camera only): lidar rays passing over it
     change nothing; the camera seeing floor there 5 times -> free
  5. the ceiling: 30 hits from 30 places -> score 10, then 13 misses -> free
     (bounded unlearning, never a wall for a day)
  6. the table leg of run B (26/08): thinner than a cell, seen ten times, found
     at low = -3 - the floor sampled on its own cell in the NEXT frame erased
     every hit. A cell hit by the camera is deaf to floor samples for
     LOW_HIT_PROTECT_S; past that window it is forgotten exactly as before.
"""

import numpy as np

from vector_dimos.costmap2d import (
    FREE_FLOOR, HIT_CAP, LOW_HIT_PROTECT_S, OCCUPIED_AT, ScoredGrid,
)

LEG = np.array([[1.0, 0.0]])


def fresh() -> ScoredGrid:
    return ScoredGrid(span_m=6.0)


def test_leg_seen_from_one_spot_is_occupied_and_ray_is_free() -> None:
    g = fresh()
    g.lidar_revolution(LEG, (0.0, 0.0))
    assert g.value_at(1.0, 0.0) == 0, "one hit is not an obstacle yet"
    g.lidar_revolution(LEG, (0.0, 0.0))
    assert g.value_at(1.0, 0.0) == 100, "second hit, even from the same spot: obstacle (a parked rover must see its walls)"
    for x in (0.3, 0.5, 0.7, 0.9):
        assert g.value_at(x, 0.0) == 0, f"floor along the ray at {x} m must be free"
    assert g.value_at(1.3, 0.0) == -1, "behind the leg: never seen"
    assert g.value_at(1.0, 0.5) == -1, "beside the ray: never seen"
    print("  leg from one spot -> occupied after 2 hits; ray -> free; behind/beside -> unknown")


def test_ramp_hit_from_one_spot_never_becomes_a_wall() -> None:
    g = fresh()
    for _ in range(50):
        g.lidar_revolution(LEG, (0.0, 0.0))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.value_at(1.0, 0.0) == 100 and g.lidar[gy[0], gx[0]] == 2, "same spot: occupied but capped at 2"
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
    """Run B, 26/08: a table leg thinner than a 5 cm cell, seen ten times, sat
    at low = -3. The camera calls the cell an obstacle in one frame and bare
    floor in the next (5 cm of quantisation), and the two cancel forever."""
    g = fresh()
    leg = np.array([[1.0, 0.0]])
    t = 1000.0                                     # an injected clock, in seconds
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
    for t in (test_leg_seen_from_one_spot_is_occupied_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
              test_chair_moved_is_forgotten_by_lidar_rays, test_low_object_only_the_camera_can_forget,
              test_thin_leg_survives_the_floor_sampled_beside_it,
              test_reinforcement_is_capped_and_unlearning_bounded, test_checkpoint_roundtrip, test_revolution_cost):
        print(t.__name__); t()
    print("TEST PASSED")

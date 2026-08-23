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
"""

import numpy as np

from vector_dimos.costmap2d import FREE_FLOOR, HIT_CAP, ScoredGrid

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
    assert n <= 2, n
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
    while g.value_at(1.0, 0.0) == 100:
        g.camera_floor(box); n += 1
        assert n < 10
    print(f"  low box: 20 lidar rays over it -> still occupied; camera floor x{n} -> free")


def test_reinforcement_is_capped_and_unlearning_bounded() -> None:
    g = fresh()
    for i in range(30):
        g.lidar_revolution(LEG, (0.0, 0.15 * i))
    gx, gy = g.cell(np.array([1.0]), np.array([0.0]))
    assert g.lidar[gy[0], gx[0]] == HIT_CAP
    n = 0
    while g.value_at(1.0, 0.0) == 100:
        g.lidar_revolution(np.array([[3.0, 0.0]]), (0.0, 0.0)); n += 1
    assert n <= HIT_CAP and g.lidar[gy[0], gx[0]] >= FREE_FLOOR
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


if __name__ == "__main__":
    for t in (test_leg_seen_from_one_spot_is_occupied_and_ray_is_free, test_ramp_hit_from_one_spot_never_becomes_a_wall,
              test_chair_moved_is_forgotten_by_lidar_rays, test_low_object_only_the_camera_can_forget,
              test_reinforcement_is_capped_and_unlearning_bounded, test_checkpoint_roundtrip):
        print(t.__name__); t()
    print("TEST PASSED")



"""Cold bench for the obstacle cost gradient: known geometry in -> known route out.

No robot, no stack. Every case is a synthetic grid at the real 0.05 m
resolution, so the numbers are metres a tape measure would agree with.

The three things the gradient has to get right at once:
  1. given two ways round, take the wide one;
  2. given open space beside a wall, keep away from the wall;
  3. given a tight gap as the ONLY way through, still go through it. That is the
     one that bit us before - a lethal inflation of 0.30 m walls off every
     60-70 cm doorway in the flat, and the rover explored nothing.
"""

import numpy as np
from scipy import ndimage

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from vector_dimos.recovering_planner import (
    LETHAL_CLEARANCE_M,
    PIVOT_PENALTY,
    PIVOT_CLEARANCE_M,
    ROBOT_WIDTH_M,
    clearance_cost_map,
)

RES = 0.05


def make_grid(cells: np.ndarray) -> OccupancyGrid:
    origin = Pose()
    origin.position.x, origin.position.y, origin.position.z = 0.0, 0.0, 0.0
    origin.orientation.w = 1.0
    return OccupancyGrid(
        grid=cells.astype(np.int8), resolution=RES, origin=origin, frame_id="map", ts=0.0
    )


def world(gx: int, gy: int) -> tuple[float, float]:
    return ((gx + 0.5) * RES, (gy + 0.5) * RES)


def path_cells(path, shape) -> list[tuple[int, int]]:
    out = []
    for pose in path.poses:
        gx, gy = int(pose.position.x / RES), int(pose.position.y / RES)
        if 0 <= gy < shape[0] and 0 <= gx < shape[1]:
            out.append((gx, gy))
    return out


def clearance_field(cells: np.ndarray) -> np.ndarray:
    """Metres from every cell to the nearest occupied cell."""
    return ndimage.distance_transform_edt(cells < CostValues.OCCUPIED) * RES


def usable_cells(cells: np.ndarray, row: int, lo: int, hi: int) -> int:
    """Cells of that corridor the planner may actually stand on."""
    field = clearance_field(cells)
    return int(np.sum(field[row, lo:hi] >= LETHAL_CLEARANCE_M))


def test_constants_are_physically_ordered() -> None:
    """The three radii must stay in the order the design assumes: the body
    fits, the margin is real, and pivoting needs more room than driving."""
    assert ROBOT_WIDTH_M / 2 < LETHAL_CLEARANCE_M < PIVOT_CLEARANCE_M, (
        ROBOT_WIDTH_M / 2,
        LETHAL_CLEARANCE_M,
        PIVOT_CLEARANCE_M,
    )
    print(
        f"  body/2 {ROBOT_WIDTH_M / 2:.3f} < lethal {LETHAL_CLEARANCE_M:.3f} "
        f"< pivot {PIVOT_CLEARANCE_M:.3f} m"
    )


def test_prefers_the_wide_corridor() -> None:
    """Two parallel corridors of the same length, both joining start to goal.

    Sized so that after the lethal radius one leaves 3 usable cells and the
    other 8 - the 3-vs-8 of the brief, counted in cells the planner can really
    stand on. (3 and 8 raw cells would be 0.15 m and 0.40 m, both narrower than
    the 0.50 m body, so neither would be a corridor at all.)
    """
    height, width = 101, 121
    cells = np.zeros((height, width), dtype=np.int8)
    band_lo, band_hi = 25, 76           # the corridor band; rooms above and below
    narrow_lo, narrow_hi = 8, 21        # 13 cells = 0.65 m between the faces
    wide_lo, wide_hi = 34, 52           # 18 cells = 0.90 m between the faces
    cells[band_lo:band_hi, :] = CostValues.OCCUPIED
    cells[band_lo:band_hi, narrow_lo:narrow_hi] = 0
    cells[band_lo:band_hi, wide_lo:wide_hi] = 0

    mid = height // 2
    n_narrow = usable_cells(cells, mid, narrow_lo, narrow_hi)
    n_wide = usable_cells(cells, mid, wide_lo, wide_hi)
    print(f"  usable width: narrow 0.65 m -> {n_narrow} cells, wide 0.90 m -> {n_wide} cells")
    assert (n_narrow, n_wide) == (3, 8), (n_narrow, n_wide)

    # start and goal sit between the two corridor mouths, so neither route is
    # shorter: narrow centre x=14, wide centre x=42.5, start x=28.
    start = world(28, 10)
    goal = world(28, height - 11)
    costmap = clearance_cost_map(make_grid(cells))
    path = min_cost_astar(costmap, goal, start)
    assert path is not None and path.poses, "no path through either corridor"

    crossing = [gx for gx, gy in path_cells(path, cells.shape) if band_lo + 10 <= gy < band_hi - 10]
    assert crossing, "path never crossed the corridor band"
    mean_x = float(np.mean(crossing))
    took_wide = mean_x > (narrow_hi + wide_lo) / 2
    print(
        f"  narrow centre x={(narrow_lo + narrow_hi) / 2 * RES:.2f} m, "
        f"wide centre x={(wide_lo + wide_hi) / 2 * RES:.2f} m, "
        f"path crossed at x={mean_x * RES:.2f} m -> {'WIDE' if took_wide else 'NARROW'}"
    )
    assert took_wide, "path took the narrow corridor with a wide one available"


def test_the_penalty_is_what_chooses_the_wide_corridor() -> None:
    """Pin the constant to the behaviour it buys.

    With the penalty off, dimOS's ridge cost alone is indifferent: it puts cost
    0 on the medial axis of BOTH corridors, so the tie falls to length and the
    narrow one wins. This is the measurement that justifies PIVOT_PENALTY
    existing at all - if this test ever passes at penalty 0, the term is dead
    weight and should go.
    """
    height, width = 101, 121
    cells = np.zeros((height, width), dtype=np.int8)
    cells[25:76, :] = CostValues.OCCUPIED
    cells[25:76, 8:21] = 0          # 0.65 m
    cells[25:76, 34:52] = 0         # 0.90 m
    grid = make_grid(cells)
    start, goal = world(28, 10), world(28, height - 11)

    for penalty, expected in ((0, "NARROW"), (PIVOT_PENALTY, "WIDE")):
        path = min_cost_astar(clearance_cost_map(grid, penalty=penalty), goal, start)
        assert path is not None and path.poses, f"no path at penalty {penalty}"
        crossing = [gx for gx, gy in path_cells(path, cells.shape) if 35 <= gy < 66]
        got = "WIDE" if float(np.mean(crossing)) > 27 else "NARROW"
        print(f"  penalty {penalty:>4} -> {got}")
        assert got == expected, f"penalty {penalty} took the {got} corridor"


def test_keeps_away_from_a_lone_wall() -> None:
    """One wall with open space beside it: the path must stop hugging it.

    Start and goal sit 0.325 m off the wall - passable, but closer than the
    0.39 m pivot radius. Without a gradient A* runs straight along that line.
    With one it has to bow away wherever there is room; the ends are pinned by
    the start and goal, so the >= 0.35 m rule is checked on the middle.
    """
    height, width = 61, 121
    cells = np.zeros((height, width), dtype=np.int8)
    cells[0, :] = CostValues.OCCUPIED          # the wall, along y = 0
    grid = make_grid(cells)
    field = clearance_field(cells)

    start, goal = world(6, 6), world(width - 7, 6)   # 0.325 m off the wall: passable, under the pivot radius
    flat = min_cost_astar(grid, goal, start)                 # no gradient at all
    graded = min_cost_astar(clearance_cost_map(grid), goal, start)
    assert flat is not None and graded is not None

    def stats(path):
        pts = path_cells(path, cells.shape)
        keep = pts[len(pts) // 6 : -len(pts) // 6]           # drop the pinned ends
        vals = [field[gy, gx] for gx, gy in keep]
        return float(np.mean(vals)), float(np.min(vals))

    flat_mean, flat_min = stats(flat)
    graded_mean, graded_min = stats(graded)
    print(
        f"  middle-of-path clearance: no gradient mean {flat_mean:.3f} min {flat_min:.3f} m"
        f"  ->  gradient mean {graded_mean:.3f} min {graded_min:.3f} m"
    )
    assert graded_min >= 0.35, f"path came within {graded_min:.3f} m of the wall"
    assert graded_min > flat_min, "the gradient bought no clearance at all"


def test_still_takes_a_tight_gap_when_it_is_the_only_way() -> None:
    """A gap and nothing else. VECTOR is 0.50 m wide, so 0.55 m must go through.

    Regression guard. dimOS's own 1.1x inflation walls this off - measured
    26/08, every gap up to 0.62 m plans "no path" - which is what left the
    rover staring at the 60-70 cm doorways of this flat.
    """
    for gap_m, must_pass in (
        (0.40, False), (0.45, False), (0.50, False),
        (0.55, True), (0.62, True), (0.80, True),
    ):
        gap = int(round(gap_m / RES))
        height, width = 61, 41
        cells = np.zeros((height, width), dtype=np.int8)
        wall_row = height // 2
        x0 = (width - gap) // 2
        cells[wall_row, :x0] = CostValues.OCCUPIED
        cells[wall_row, x0 + gap:] = CostValues.OCCUPIED

        costmap = clearance_cost_map(make_grid(cells))
        passable = int(np.sum(costmap.grid[wall_row] < CostValues.OCCUPIED))
        path = min_cost_astar(costmap, world(width // 2, height - 3), world(width // 2, 2))
        crossed = path is not None and any(gy == wall_row for gx, gy in path_cells(path, cells.shape))
        print(
            f"  gap {gap_m:.2f} m ({gap} cells): {passable} passable cell(s), "
            f"path {'CROSSES' if crossed else 'does not cross'}"
        )
        if must_pass:
            assert crossed, f"{gap_m:.2f} m gap walled off - VECTOR is {ROBOT_WIDTH_M:.2f} m wide"
        else:
            assert not crossed, f"{gap_m:.2f} m gap accepted - narrower than the body"


if __name__ == "__main__":
    for test in (
        test_constants_are_physically_ordered,
        test_prefers_the_wide_corridor,
        test_the_penalty_is_what_chooses_the_wide_corridor,
        test_keeps_away_from_a_lone_wall,
        test_still_takes_a_tight_gap_when_it_is_the_only_way,
    ):
        print(test.__name__)
        test()
    print("TEST PASSED")

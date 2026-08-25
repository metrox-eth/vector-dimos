"""Cold tests for explorer2: known maps in, known decisions out.

Rule #2 applies: a known input must give a known output in physical units
(metres, seconds, a named directive) - not "it returned something". Groups:

  A. frontier + goal placement  - where the target lands on a plain room
  B. prefer-forward             - two mirror-image clusters, the one ahead wins
  C. failed-target memory       - an exclusion WAITS, it never ends the run (7.1)
  D. born cornered              - the pocket signature gives ONE back-off (7.3)
  E. keep-out                   - a forbidden block is never targeted (7.2)
  F. the end of a run           - None exactly when there is nothing left
  G. purity                     - the costmap, the pose and the rest of the
                                  state come back untouched, and the function
                                  is deterministic

No robot, no LCM, no dimOS needed: that is the whole point of the rewrite.

Run:  .venv/bin/python3 tests/test_explorer2_cold.py
"""
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "vector_dimos"))

try:  # the package __init__ pulls dimOS; off the Jetson, load the module directly
    from vector_dimos.explorer2 import (  # noqa: E402
        DIRECTIVE_BACK_OFF, DIRECTIVE_FRONTIER, DIRECTIVE_WAIT, DEFAULT_TUNING,
        ExploreState, PoseStamped, Tuning, next_target,
    )
except ImportError:  # pragma: no cover
    from explorer2 import (  # type: ignore[no-redef]  # noqa: E402
        DIRECTIVE_BACK_OFF, DIRECTIVE_FRONTIER, DIRECTIVE_WAIT, DEFAULT_TUNING,
        ExploreState, PoseStamped, Tuning, next_target,
    )

FREE, UNKNOWN, OCCUPIED = 0, -1, 100
RES = 0.05

OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


# --- the smallest thing that looks like a costmap --------------------------

class Grid:
    """Everything next_target reads off an OccupancyGrid, and nothing else."""

    def __init__(self, grid, res=RES, ox=0.0, oy=0.0, ts=0.0):
        self.grid = grid
        self.resolution = res
        self.frame_id = "world"
        self.ts = ts
        self.origin = type("O", (), {
            "position": type("P", (), {"x": ox, "y": oy, "z": 0.0})()})()


def pose(x, y, yaw=0.0):
    p = PoseStamped(ts=0.0, frame_id="world")
    p.position.x, p.position.y = x, y
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def cell(x, y, ox=0.0, oy=0.0, res=RES):
    return int((y - oy) / res), int((x - ox) / res)


def room(n=140, x0=30, x1=110, y0=30, y1=110, walls=True):
    """A square of free floor in the middle of unknown, walled all round.

    n x n cells at 5 cm; with the defaults that is 7 x 7 m of grid holding a
    4 x 4 m room. Openings are punched into the wall afterwards.
    """
    g = np.full((n, n), UNKNOWN, dtype=np.int8)
    g[y0:y1, x0:x1] = FREE
    if walls:
        g[y0 - 1:y1 + 1, x0 - 1] = OCCUPIED
        g[y0 - 1:y1 + 1, x1] = OCCUPIED
        g[y0 - 1, x0 - 1:x1 + 1] = OCCUPIED
        g[y1, x0 - 1:x1 + 1] = OCCUPIED
    return g


def door(g, side, centre, width_cells, x0=30, x1=110, y0=30, y1=110):
    """Punch an opening of `width_cells` in one wall, centred on `centre`."""
    half = width_cells // 2
    if side == "east":
        g[centre - half:centre + half, x1] = UNKNOWN
    elif side == "west":
        g[centre - half:centre + half, x0 - 1] = UNKNOWN
    elif side == "north":
        g[y1, centre - half:centre + half] = UNKNOWN
    elif side == "south":
        g[y0 - 1, centre - half:centre + half] = UNKNOWN
    return g


def w_of(cell_index):
    """Cell index -> the world coordinate of its centre, origin at 0."""
    return (cell_index + 0.5) * RES


# ===========================================================================
print("A. a room with one door: the target stands on free floor, at the door")

g = door(room(), "east", 70, 24)
cm = Grid(g)
here = pose(w_of(70), w_of(70), 0.0)          # middle of the room, facing +x
st = ExploreState()
target = next_target(cm, here, st, now=100.0)

check("a target comes back", target is not None)
assert target is not None
check("directive is 'frontier'", target.directive == DIRECTIVE_FRONTIER, target.directive)
gy, gx = cell(target.position.x, target.position.y)
check("the target stands on FREE floor, not in the unknown",
      g[gy, gx] == FREE and not target.on_frontier, f"grid value {g[gy, gx]}")
check("the target is at the door, not at the centroid of the room",
      target.position.x > w_of(100), f"x = {target.position.x:.2f} m")
check("the look-at point is in the unknown beyond the door",
      g[cell(*target.look_at_xy)] == UNKNOWN)
nearest_wall = min(math.hypot(target.position.x - w_of(cx), target.position.y - w_of(cy))
                   for cy, cx in zip(*np.nonzero(g == OCCUPIED)))
check("it is at least a body radius from any wall",
      nearest_wall >= DEFAULT_TUNING.lethal_clearance_m,
      f"{nearest_wall:.2f} m >= {DEFAULT_TUNING.lethal_clearance_m} m")
check("path cost is the real walking distance, not a straight line",
      target.path_cost_m >= abs(target.position.x - here.position.x) - 1e-6,
      f"{target.path_cost_m:.2f} m")
check("state: one target issued, one visited, one observed pose",
      (st.targets_issued, len(st.visited), len(st.observed)) == (1, 1, 1),
      f"{st.targets_issued}, {len(st.visited)}, {len(st.observed)}")


# ===========================================================================
print("B. prefer-forward: two mirror-image doors, the one ahead wins")

g = door(door(room(), "east", 70, 24), "west", 70, 24)
cm = Grid(g)
centre = w_of(70)

east = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
west = next_target(cm, pose(centre, centre, math.pi), ExploreState(), now=100.0)
check("facing +x picks the east door", east is not None and east.position.x > centre,
      f"x = {east.position.x:.2f}" if east else "None")
check("facing -x picks the west door", west is not None and west.position.x < centre,
      f"x = {west.position.x:.2f}" if west else "None")
check("the two doors really are the same cluster size and the same distance",
      east is not None and west is not None
      and east.info_cells == west.info_cells
      and abs(east.path_cost_m - west.path_cost_m) < 0.11,
      f"{east.info_cells} vs {west.info_cells} cells, "
      f"{east.path_cost_m:.2f} vs {west.path_cost_m:.2f} m" if east and west else "")
sideways = next_target(cm, pose(centre, centre, math.pi / 2), ExploreState(), now=100.0)
check("facing sideways, neither bonus applies and one is still chosen",
      sideways is not None and sideways.directive == DIRECTIVE_FRONTIER)


# ===========================================================================
print("C. failed-target memory: an exclusion WAITS, it never ends the run (7.1)")

g = door(room(), "east", 70, 24)
cm = Grid(g)
here = pose(centre, centre, 0.0)

probe = next_target(cm, here, ExploreState(), now=0.0)
assert probe is not None
gx, gy = probe.position.x, probe.position.y

st = ExploreState(failed=[(gx, gy, 0.0)])
waited = next_target(cm, here, st, now=10.0)
check("the only cluster is excluded -> NOT None", waited is not None)
assert waited is not None
check("directive is 'wait'", waited.directive == DIRECTIVE_WAIT, waited.directive)
check("wait_s is what is left of the 60 s hold: 50.0 s",
      abs(waited.wait_s - 50.0) < 1e-9, f"{waited.wait_s}")
check("the rover is told to stay where it is",
      (waited.position.x, waited.position.y) == (here.position.x, here.position.y))
check("the cluster is still counted as a frontier, not as absence",
      waited.n_clusters == 1 and waited.n_excluded == 1,
      f"{waited.n_clusters} clusters, {waited.n_excluded} excluded")

after = next_target(cm, here, ExploreState(failed=[(gx, gy, 0.0)]), now=61.0)
check("once the hold has expired the same cluster is targeted again",
      after is not None and after.directive == DIRECTIVE_FRONTIER,
      after.directive if after else "None")

# two exclusions, different ages: the wait is the SOONEST expiry
g2 = door(door(room(), "east", 70, 24), "west", 70, 24)
cm2 = Grid(g2)
e = next_target(cm2, pose(centre, centre, 0.0), ExploreState(), now=0.0)
wst = next_target(cm2, pose(centre, centre, math.pi), ExploreState(), now=0.0)
assert e is not None and wst is not None
both = ExploreState(failed=[(e.position.x, e.position.y, 0.0),
                            (wst.position.x, wst.position.y, 40.0)])
waited2 = next_target(cm2, pose(centre, centre, 0.0), both, now=45.0)
check("with two exclusions the wait is the soonest expiry (60 - 45 = 15 s)",
      waited2 is not None and waited2.directive == DIRECTIVE_WAIT
      and abs(waited2.wait_s - 15.0) < 1e-9,
      f"{waited2.wait_s}" if waited2 else "None")

# one of two excluded: the other one is simply used, no wait at all
one = ExploreState(failed=[(e.position.x, e.position.y, 40.0)])
other = next_target(cm2, pose(centre, centre, 0.0), one, now=45.0)
check("only one of two excluded -> the other is targeted, no wait",
      other is not None and other.directive == DIRECTIVE_FRONTIER
      and other.position.x < centre,
      f"{other.directive} x={other.position.x:.2f}" if other else "None")


# ===========================================================================
print("D. born cornered: the pocket signature gives ONE back-off (7.3)")

# 3 x 3 free cells (0.0225 m2) sealed by a ring of obstacle, unknown beyond.
g = np.full((40, 40), UNKNOWN, dtype=np.int8)
g[19:22, 19:22] = FREE
g[18:23, 18] = OCCUPIED
g[18:23, 22] = OCCUPIED
g[18, 18:23] = OCCUPIED
g[22, 18:23] = OCCUPIED
cm = Grid(g)
here = pose(w_of(20), w_of(20), 0.0)
st = ExploreState()

first = next_target(cm, here, st, now=100.0)
check("a cornered rover gets a directive, not None", first is not None)
assert first is not None
check("directive is 'back_off'", first.directive == DIRECTIVE_BACK_OFF, first.directive)
check("the back-off is 0.22 m straight behind the heading",
      abs(first.position.x - (here.position.x - 0.22)) < 1e-9
      and abs(first.position.y - here.position.y) < 1e-9,
      f"({first.position.x:.3f}, {first.position.y:.3f})")
check("it reports the pocket it measured: 9 cells = 0.0225 m2",
      abs(first.reachable_free_m2 - 0.0225) < 1e-9, f"{first.reachable_free_m2}")
check("state remembers the back-off went out", st.back_off_issued is True)

second = next_target(cm, here, st, now=101.0)
check("the SAME pocket does not give a second back-off: it gives None",
      second is None, str(second))

# facing another way, the back-off follows the heading
st3 = ExploreState()
turned = next_target(cm, pose(w_of(20), w_of(20), math.pi / 2), st3, now=100.0)
check("heading +y -> back-off is 0.22 m in -y",
      turned is not None and abs(turned.position.y - (w_of(20) - 0.22)) < 1e-9
      and abs(turned.position.x - w_of(20)) < 1e-9,
      f"({turned.position.x:.3f}, {turned.position.y:.3f})" if turned else "None")


# ===========================================================================
print("E. keep-out: a forbidden block is never targeted (7.2)")

# The room's only door leads into a corridor the owner declared forbidden.
# costmap2d forces those cells to 100 before the explorer ever sees the map.
g = door(room(), "east", 70, 24)
keep = np.zeros_like(g, dtype=bool)
keep[58:82, 110:118] = True          # the corridor behind the door
g[keep] = OCCUPIED
cm = Grid(g)
st = ExploreState()
blocked = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)

check("with the only way out forbidden, exploration ends: None",
      blocked is None, str(blocked))

# now give the room a second, legal door and check the target never lands in
# the forbidden block, nor within a body radius of it
g2 = door(g.copy(), "north", 70, 24)
cm2 = Grid(g2)
seen_in_zone = 0
closest = math.inf
st2 = ExploreState()
for i in range(12):
    t = next_target(cm2, pose(centre, centre, 0.0), st2, now=100.0 + i)
    if t is None or t.directive != DIRECTIVE_FRONTIER:
        break
    ty, tx = cell(t.position.x, t.position.y)
    if keep[ty, tx]:
        seen_in_zone += 1
    for ky, kx in zip(*np.nonzero(keep)):
        closest = min(closest, math.hypot(t.position.x - w_of(kx), t.position.y - w_of(ky)))
check("no target ever lands inside the forbidden block", seen_in_zone == 0,
      f"{seen_in_zone} did")
check("and none comes within the lethal clearance of it",
      closest >= DEFAULT_TUNING.lethal_clearance_m,
      f"closest {closest:.2f} m >= {DEFAULT_TUNING.lethal_clearance_m} m")


# ===========================================================================
print("F. the end of a run: None exactly when there is nothing left")

# a completely known map: free room, wall, and solid obstacle everywhere else
g = np.full((140, 140), OCCUPIED, dtype=np.int8)
g[30:110, 30:110] = FREE
cm = Grid(g)
st = ExploreState()
done = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a map with no unknown cell at all -> None", done is None, str(done))
check("and it is not the cornered branch: the floor is 16 m2, not a pocket",
      st.back_off_issued is False and st.last_directive == "")

# one unknown cell short of complete, but too small to be a frontier cluster
g2 = g.copy()
g2[60:62, 60:62] = UNKNOWN            # 4 unknown cells, min cluster is 6
check("an unknown speck below min_frontier_perimeter is not a frontier -> None",
      next_target(Grid(g2), pose(centre, centre, 0.0), ExploreState(), now=100.0) is None)

g3 = g.copy()
g3[60:70, 60:70] = UNKNOWN            # 100 unknown cells in the middle of the room
kept = next_target(Grid(g3), pose(w_of(100), w_of(100), 0.0), ExploreState(), now=100.0)
check("a real unknown pocket inside the room IS a frontier -> a target",
      kept is not None and kept.directive == DIRECTIVE_FRONTIER,
      kept.directive if kept else "None")


# ===========================================================================
print("G. purity: nothing but the state's own fields changes")

g = door(room(), "east", 70, 24)
grid_before = g.copy()
cm = Grid(g)
here = pose(centre, centre, 0.3)
pose_before = (here.position.x, here.position.y, here.orientation.z, here.orientation.w)

st = ExploreState(failed=[(1.0, 2.0, 50.0)], back_off_issued=False)
before = st.copy()
out = next_target(cm, here, st, now=60.0)

check("the costmap is not written to", np.array_equal(g, grid_before))
check("the pose is not written to",
      (here.position.x, here.position.y, here.orientation.z, here.orientation.w) == pose_before)
check("failed entries still inside their hold are kept as they were",
      st.failed == before.failed, f"{st.failed}")
check("back_off_issued untouched when nothing was cornered",
      st.back_off_issued == before.back_off_issued)
check("heading was refreshed from the pose (0.3 rad)", abs(st.heading - 0.3) < 1e-6,
      f"{st.heading}")
check("visited / observed / targets_issued / last_directive are the only other changes",
      (len(st.visited), len(st.observed), st.targets_issued, st.last_directive)
      == (1, 1, 1, DIRECTIVE_FRONTIER))

expired = ExploreState(failed=[(1.0, 2.0, 0.0)])
next_target(cm, here, expired, now=61.0)
check("a failed entry past its 60 s hold is pruned", expired.failed == [], f"{expired.failed}")

a = next_target(cm, here, ExploreState(), now=100.0)
b = next_target(cm, here, ExploreState(), now=100.0)
check("same map + same pose + same state -> same answer",
      a is not None and b is not None
      and (a.position.x, a.position.y) == (b.position.x, b.position.y)
      and a.score == b.score)

tuned = next_target(cm, here, ExploreState(), now=100.0,
                    tuning=Tuning(forward_bonus=0.0, revisit_radius_m=0.1))
check("tuning is honoured and is not global state",
      tuned is not None
      and next_target(cm, here, ExploreState(), now=100.0).score == a.score)

check("no clock is read when `now` is given: a 1970 timestamp behaves",
      next_target(cm, here, ExploreState(failed=[(gx, gy, -1e9)]), now=0.0) is not None)


print(f"{OK} OK, {KO} KO")
sys.exit(1 if KO else 0)

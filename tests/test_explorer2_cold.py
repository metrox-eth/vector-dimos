"""Cold tests for explorer2: known maps in, known decisions out.

Rule #2 applies: a known input must give a known output in physical units
(metres, seconds, a named directive) - not "it returned something". Groups:

  A. frontier + goal placement  - where the target lands on a plain room
  B. prefer-forward             - two mirror-image clusters, the one ahead wins
  C. failed-target memory       - an exclusion WAITS, never ends the run, and
                                  reopens on TRIGGERS (map changed / new
                                  viewpoint), never on a clock (7.1)
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
print("C. failed-target memory: an exclusion WAITS, reopens on triggers, never a clock (7.1)")

g = door(room(), "east", 70, 24)
cm = Grid(g)
here = pose(centre, centre, 0.0)

probe = next_target(cm, here, ExploreState(), now=0.0)
assert probe is not None
gx, gy = probe.position.x, probe.position.y

st = ExploreState()
st.note_failed(gx, gy, (centre, centre), cm)
waited = next_target(cm, here, st, now=10.0)
check("the only cluster is excluded -> NOT None", waited is not None)
assert waited is not None
check("directive is 'wait'", waited.directive == DIRECTIVE_WAIT, waited.directive)
check("the rover is told to stay where it is",
      (waited.position.x, waited.position.y) == (here.position.x, here.position.y))
check("the cluster is still counted as a frontier, not as absence",
      waited.n_clusters == 1 and waited.n_excluded == 1,
      f"{waited.n_clusters} clusters, {waited.n_excluded} excluded")

much_later = next_target(cm, here, st, now=1e9)
check("NO clock reopens it: same map, same pose, a billion seconds later -> still wait",
      much_later is not None and much_later.directive == DIRECTIVE_WAIT,
      much_later.directive if much_later else "None")

# starvation escape: after WAIT_REOPEN_POLLS silent asks the oldest entry reopens
from vector_dimos.explorer2 import WAIT_REOPEN_POLLS
starved = ExploreState()
starved.note_failed(gx, gy, (centre, centre), cm)
out = None
for _ in range(WAIT_REOPEN_POLLS + 2):
    out = next_target(cm, here, starved, now=10.0)
    if out is not None and out.directive == DIRECTIVE_FRONTIER:
        break
check(f"starved {WAIT_REOPEN_POLLS} asks -> the oldest failed goal reopens and is targeted",
      out is not None and out.directive == DIRECTIVE_FRONTIER and starved.failed == [],
      out.directive if out else "None")

# trigger 1: the rover stands a viewpoint away from where it failed
moved_st = ExploreState()
moved_st.note_failed(gx, gy, (centre, centre), cm)
moved = next_target(cm, pose(centre - 1.2, centre, 0.0), moved_st, now=10.0)
check("the rover 1.2 m from where it failed -> the exclusion reopens (viewpoint trigger)",
      moved is not None and moved.directive == DIRECTIVE_FRONTIER,
      moved.directive if moved else "None")
check("and the reopened entry is pruned from the state", moved_st.failed == [],
      f"{moved_st.failed}")

# trigger 2: the map around the goal changed (new cells observed)
changed_st = ExploreState()
changed_st.note_failed(gx, gy, (centre, centre), cm)
g_changed = g.copy()
cgx, cgy = int((gx - cm.origin.position.x) / cm.resolution), int((gy - cm.origin.position.y) / cm.resolution)
patch = g_changed[cgy - 4:cgy + 5, cgx - 4:cgx + 5]
patch[patch == UNKNOWN] = FREE                     # the world got observed there
changed = next_target(Grid(g_changed), here, changed_st, now=10.0)
check("new observations around the failed goal -> the exclusion reopens (map trigger)",
      changed is not None and changed.directive == DIRECTIVE_FRONTIER,
      changed.directive if changed else "None")

# two exclusions: both held -> wait; one held -> the other is simply used
g2 = door(door(room(), "east", 70, 24), "west", 70, 24)
cm2 = Grid(g2)
e = next_target(cm2, pose(centre, centre, 0.0), ExploreState(), now=0.0)
wst = next_target(cm2, pose(centre, centre, math.pi), ExploreState(), now=0.0)
assert e is not None and wst is not None
both = ExploreState()
both.note_failed(e.position.x, e.position.y, (centre, centre), cm2)
both.note_failed(wst.position.x, wst.position.y, (centre, centre), cm2)
waited2 = next_target(cm2, pose(centre, centre, 0.0), both, now=45.0)
check("with two exclusions held -> wait, both counted",
      waited2 is not None and waited2.directive == DIRECTIVE_WAIT
      and waited2.n_excluded == 2,
      f"{waited2.n_excluded if waited2 else None} excluded")

one = ExploreState()
one.note_failed(e.position.x, e.position.y, (centre, centre), cm2)
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
print("H. fresh ground beats ground already worked (ticket 1, run B goal 10)")

# Two mirror-image doors. Test B has already shown that, facing east, the east
# door wins. Now the rover has already been sent to the east door once - and
# has been somewhere else since - so the east door is a REVISIT.
g = door(door(room(), "east", 70, 24), "west", 70, 24)
cm = Grid(g)
centre = w_of(70)

plain = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
assert plain is not None
east_goal = (plain.position.x, plain.position.y)
check("(setup) facing east with no memory, the east door is the pick",
      east_goal[0] > centre, f"x = {east_goal[0]:.2f}")

faded = next_target(cm, pose(centre, centre, 0.0),
                    ExploreState(visited=[east_goal, (-9.0, -9.0)]), now=100.0)
check("a door already published, with a goal since, loses to the fresh one",
      faded is not None and faded.position.x < centre,
      f"x = {faded.position.x:.2f}" if faded else "None")

# ... but the goal it was driving to when it last stopped is not a revisit:
# that one is unfinished business and must not be dropped.
current = next_target(cm, pose(centre, centre, 0.0),
                      ExploreState(visited=[(-9.0, -9.0), east_goal]), now=100.0)
check("the goal just attempted keeps its score: no fade on unfinished business",
      current is not None and current.position.x > centre,
      f"x = {current.position.x:.2f}" if current else "None")

# The fade is the SQUARE of the distance: at half the revisit radius a cluster
# keeps a quarter of its score, not a half. One door, so the same cluster comes
# back either way and only the fade differs.
one_door = Grid(door(room(), "east", 70, 24))
clean = next_target(one_door, pose(centre, centre, 0.0), ExploreState(), now=100.0)
assert clean is not None
half = (clean.position.x - DEFAULT_TUNING.revisit_radius_m / 2, clean.position.y)
faded_half = next_target(one_door, pose(centre, centre, 0.0),
                         ExploreState(visited=[half, (-9.0, -9.0)]), now=100.0)
ratio = (faded_half.score / clean.score) if faded_half is not None else None
check("a cluster half a revisit radius from an older goal keeps a QUARTER of its score",
      ratio is not None and abs(ratio - 0.25) < 1e-9, f"ratio {ratio}")


# ===========================================================================
print("H2. the gain is what the viewpoint can SEE, not unknown behind the wall")

# Same room, two doors. Behind the east door, a cupboard: 0.35 m of unknown
# closed by walls. Behind the west door, open unknown. The two frontiers have
# the same cell count and the same distance, and the rover faces EAST, so a
# gain that counts unknown through a wall picks the cupboard.
g = door(door(room(), "east", 70, 24), "west", 70, 24)
g[57:83, 117] = OCCUPIED           # cupboard back wall, 0.35 m beyond the door
g[57, 110:118] = OCCUPIED          # and its two sides
g[82, 110:118] = OCCUPIED
cm = Grid(g)
seen = next_target(cm, pose(centre, centre, 0.0), ExploreState(), now=100.0)
check("facing the cupboard, the rover still takes the door with a room behind it",
      seen is not None and seen.position.x < centre,
      f"x = {seen.position.x:.2f}" if seen else "None")
check("and the gain it reports is a fraction of a disc, not a cell count",
      seen is not None and 0.0 <= seen.info_gain <= 1.0, f"{seen.info_gain}" if seen else "")


# ===========================================================================
print("I. shut in is not finished (ticket 2, run B at 6 min 33)")

# A room the body cannot leave: its only opening is 0.50 m wide, and the body
# needs 0.60 m of clearance to pass. Beyond it, a corridor and real unknown.
def pinched_map():
    g = np.full((160, 160), OCCUPIED, dtype=np.int8)
    g[30:110, 30:110] = FREE                  # the room the rover is in
    g[65:75, 110:140] = FREE                  # a 0.50 m x 1.50 m throat, floor seen
    g[30:110, 140:155] = UNKNOWN              # the room beyond, never seen
    return g

g = pinched_map()
cm = Grid(g)
st = ExploreState()
shut_in = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a rover shut in with frontiers left on the map does NOT report completion",
      shut_in is not None, str(shut_in))
assert shut_in is not None
check("it gets one back-off, the reflex that freed it by hand",
      shut_in.directive == DIRECTIVE_BACK_OFF, shut_in.directive)
check("and it says how many clusters the map still holds",
      getattr(shut_in, "n_on_the_map", 0) >= 1, f"{getattr(shut_in, 'n_on_the_map', 0)}")
# NOT the born-cornered case: the 4 x 4 m room, minus the 0.30 m the body
# cannot use along each wall, is a 3.50 x 3.50 m square = 12.25 m2, plus the
# six cells at the mouth of the throat where the wall opens out and the
# clearance rises back over 0.30 m: 12.265 m2, twenty-four times the 0.5 m2
# that means "born cornered".
check("this is NOT the born-cornered case: 12.265 m2 of floor under it",
      abs(getattr(shut_in, "reachable_free_m2", 0.0) - 12.265) < 1e-9,
      f"{getattr(shut_in, 'reachable_free_m2', float('nan')):.4f} m2")
again = next_target(cm, pose(centre, centre, 0.0), st, now=101.0)
check("one back-off per pocket and no more: then the run ends", again is None, str(again))


# ===========================================================================
print("I2. a frontier the body cannot walk to is looked at from where it can stand")

# The same throat, but the unknown starts right at its mouth: the frontier is
# 0.55 m from floor the body can stand on. That is the run B geometry - eleven
# clusters like it were on the map when the run declared itself finished.
g = np.full((160, 160), OCCUPIED, dtype=np.int8)
g[30:110, 30:110] = FREE
g[65:75, 110:116] = FREE                      # 0.50 m throat, 0.30 m of seen floor
g[62:78, 116:150] = UNKNOWN                   # the unmapped room, opening out
cm = Grid(g)
st = ExploreState()
peek = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
check("a target comes back rather than None", peek is not None, str(peek))
assert peek is not None
check("it is a frontier goal", peek.directive == DIRECTIVE_FRONTIER, peek.directive)
gy, gx = cell(peek.position.x, peek.position.y)
check("the rover is told to stand on floor it can reach, not in the unknown",
      g[gy, gx] == FREE and not peek.on_frontier, f"grid {g[gy, gx]}")
nearest_wall = min(math.hypot(peek.position.x - w_of(cx), peek.position.y - w_of(cy))
                   for cy, cx in zip(*np.nonzero(g == OCCUPIED)))
check("and it stands a body radius clear of the walls",
      nearest_wall >= DEFAULT_TUNING.lethal_clearance_m,
      f"{nearest_wall:.2f} m")
check("the look-at point is the unknown behind the throat",
      peek.look_at_xy[0] > w_of(110), f"{peek.look_at_xy}")


# ===========================================================================
print("I3. a frontier is retired by being SEEN, not by a viewpoint being used")

# One door, one cluster. The rover drives to the target and decides from there;
# the map does not change (the shadow the 2D scan plane never enters). Standing
# in plain view of it and learning nothing is the end of that frontier - this
# is the 300-goals-and-184-m protection, and it must still hold.
g = door(room(), "east", 70, 24)
cm = Grid(g)
st = ExploreState()
first = next_target(cm, pose(centre, centre, 0.0), st, now=100.0)
assert first is not None
arrived = next_target(cm, pose(first.position.x, first.position.y, 0.0), st, now=101.0)
check("looked straight at it from close up and nothing changed: that cluster is done",
      arrived is None, str(arrived))

# Same distance, same everything, one wall of difference. A closed room with
# one unknown pocket in the middle of its floor, and a spot the rover has
# already decided from, 0.70 m from that pocket.
def pocket_room(partition: bool):
    g = np.full((140, 140), OCCUPIED, dtype=np.int8)
    g[30:110, 30:110] = FREE
    g[66:74, 66:74] = UNKNOWN            # 0.40 x 0.40 m of unknown floor
    if partition:
        g[60:80, 80] = OCCUPIED          # a screen between the spot and the pocket
    return g

stood_at = (w_of(84), w_of(70))          # 0.50 m east of the pocket's edge
in_view = next_target(Grid(pocket_room(False)), pose(*stood_at, 0.0),
                      ExploreState(observed=[stood_at]), now=100.0)
check("a pocket in plain view of a spot already decided from is retired",
      in_view is None, str(in_view))
hidden = next_target(Grid(pocket_room(True)), pose(*stood_at, 0.0),
                     ExploreState(observed=[stood_at]), now=100.0)
check("the same pocket behind a screen is NOT retired: it was never seen",
      hidden is not None and hidden.directive == DIRECTIVE_FRONTIER,
      (hidden.directive if hidden else "None"))
if hidden is not None:
    check("and the viewpoint it is given is on the pocket's side of the screen",
          hidden.position.x < w_of(80), f"x = {hidden.position.x:.2f} m")


# ===========================================================================
print("I4. and it always terminates: no map, however awkward, spins for ever")

for label, grid_in in (("one door", door(room(), "east", 70, 24)),
                       ("three doors", door(door(door(room(), "east", 70, 24),
                                                 "west", 70, 24), "north", 70, 24)),
                       ("shut in", pinched_map())):
    st = ExploreState()
    here = pose(centre, centre, 0.0)
    calls = 0
    for i in range(60):
        calls += 1
        t = next_target(Grid(grid_in), here, st, now=100.0 + i)
        if t is None:
            break
        if t.directive == DIRECTIVE_FRONTIER:
            here = pose(t.position.x, t.position.y, 0.0)   # the rover arrives
    check(f"{label}: the map runs out and the function says so ({calls} calls)",
          t is None and calls < 60, f"{calls} calls, last {t}")


# ===========================================================================
print("G. purity: nothing but the state's own fields changes")

g = door(room(), "east", 70, 24)
grid_before = g.copy()
cm = Grid(g)
here = pose(centre, centre, 0.3)
pose_before = (here.position.x, here.position.y, here.orientation.z, here.orientation.w)

st = ExploreState(back_off_issued=False)
st.note_failed(1.0, 2.0, (here.position.x, here.position.y), cm)   # robot has not moved since
before = st.copy()
out = next_target(cm, here, st, now=60.0)

check("the costmap is not written to", np.array_equal(g, grid_before))
check("the pose is not written to",
      (here.position.x, here.position.y, here.orientation.z, here.orientation.w) == pose_before)
check("failed entries whose triggers have not fired are kept as they were",
      st.failed == before.failed, f"{st.failed}")
check("back_off_issued untouched when nothing was cornered",
      st.back_off_issued == before.back_off_issued)
check("heading was refreshed from the pose (0.3 rad)", abs(st.heading - 0.3) < 1e-6,
      f"{st.heading}")
check("visited / observed / targets_issued / last_directive are the only other changes",
      (len(st.visited), len(st.observed), st.targets_issued, st.last_directive)
      == (1, 1, 1, DIRECTIVE_FRONTIER))

reopened = ExploreState()
reopened.note_failed(1.0, 2.0, (here.position.x - 5.0, here.position.y), cm)   # failed 5 m from here
next_target(cm, here, reopened, now=61.0)
check("a failed entry whose rover has since moved a viewpoint away is pruned",
      reopened.failed == [], f"{reopened.failed}")

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

noclock = ExploreState()
noclock.note_failed(gx, gy, (here.position.x, here.position.y), cm)
check("no clock is read when `now` is given: a 1970 timestamp behaves",
      next_target(cm, here, noclock, now=-1e9) is not None)


print(f"{OK} OK, {KO} KO")
sys.exit(1 if KO else 0)

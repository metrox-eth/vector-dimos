"""Minimal reproduction: PR #2830's scorer yields ZERO candidates whenever the
robot stands more than one cell inside blocked space of the costmap it is
handed.

The chain, line by line:
1. The exploration loop inflates the costmap by 0.25 m before selection
   (simple_inflate; upstream behaviour, both selectors receive the same grid).
2. The robot can legitimately sit closer than 0.25 m to an obstacle: the
   planner's own lethal clearance is 0.225 m, smaller than the selection
   inflation, so any pose the planner accepts can be inside the selection ring.
   In the bigoffice_ply demo the spawn sits 0.112 m (2.2 cells) inside it.
3. selector_head._compute_path_cost calls min_cost_astar with
   cost_threshold=occupancy_threshold (99). min_cost_astar escapes a start ONE
   cell deep in blocked space, but not two (demonstrated below), so it returns
   None and _compute_path_cost returns float('inf') ("Returns inf when the
   frontier is unreachable, so it scores 0 ... and is never selected").
4. Every frontier scores inf -> the head's _rank_frontiers discards all ->
   detect_frontiers returns [] -> the explorer emits no goal, forever.
The base selector has no A* in its scoring and is immune.

Run: LESH_WORLD=ply python test_pr2830_blocked_start_repro.py
"""
import os, sys
os.environ.setdefault("LESH_WORLD", "ply")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import dimos_selector as DS
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar

V = DS.Vector3

def disk_probe(r_cells):
    g = np.zeros((21, 21), dtype=np.int8)
    yy, xx = np.mgrid[0:21, 0:21]
    g[np.hypot(yy - 10, xx - 10) <= r_cells] = 100
    cm = DS.to_occupancy_grid(g, 0.05, 0.0, 0.0, 0.0)
    p = min_cost_astar(cm, goal=V(0.975, 0.525, 0), start=V(0.525, 0.525, 0),
                       cost_threshold=99, unknown_penalty=0.95)
    return bool(p and p.poses)

assert disk_probe(1) is True, "expected: escapes a 1-cell-deep blocked start"
assert disk_probe(2) is False, "expected: trapped 2 cells deep -> None -> inf"
print("repro OK: min_cost_astar escapes 1 cell of blocked start, not 2.")

snap = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dissect_snap.npz")
if os.path.exists(snap):
    z = np.load(snap)
    g = z["grid"]
    cm = DS.to_occupancy_grid(g, float(z["res"]), float(z["ox"]), float(z["oy"]), 0.0)
    pose = V(float(z["start"][0]), float(z["start"][1]), 0.0)
    exb = DS.make_explorer(DS.sel_stock)
    exh = DS.make_explorer(DS.sel_pr)
    nb, nh = len(exb.detect_frontiers(pose, cm)), len(exh.detect_frontiers(pose, cm))
    print(f"real first-decision snapshot: base detects {nb} frontiers, head {nh}.")
    assert nb > 0 and nh == 0

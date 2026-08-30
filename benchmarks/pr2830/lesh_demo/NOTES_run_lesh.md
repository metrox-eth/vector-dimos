# The lesh run, taken literally, on the dataset's own global map

## Verdict, first line, per arm: does it explore the left side THEN the right?

- **stock / shipped stop: NO.** 9 goals, 64 m, 42 % of the ceiling, then the
  info-gain self-stop quits with the whole lower half of the building dark.
- **PR #2830 / shipped stop: NO, it never starts.** 0 goals, 0 m, 4 % (the
  spawn scans only). ROOT-CAUSED, see below and
  `test_pr2830_blocked_start_repro.py`.
- **stock + signed momentum + completion stop: YES.** 51 goals, 299 m,
  **98.2 % of the ceiling = 97 % of everything the recording observed**, one
  single cross-building jump in the whole run, orderly room-by-room walks.
  This is the picture lesh asked for.
- **PR #2830 + signed momentum + completion stop: NO** - same root cause as
  arm 2, dies instantly ("no frontier remains").

## Root cause of the pr2830 zero-candidate (pinned to the line, reproduced)

The chain, each step measured here:
1. The exploration loop inflates the costmap 0.25 m before selection
   (upstream `simple_inflate`; both selectors receive the same ternary grid -
   100 / 0 / -1, no graded costs, so the graded-cost suspect is dead).
2. The spawn sits **0.112 m (2.2 cells) inside that selection inflation**.
   This pose is legitimate: the planner's own lethal clearance is 0.225 m,
   SMALLER than the 0.25 m selection inflation, so any pose the planner
   accepts can be inside the selection ring. A live robot that stopped near
   furniture is in the same position.
3. `selector_head._compute_path_cost` calls `min_cost_astar` with
   `cost_threshold=occupancy_threshold` (99). Measured on a synthetic disk:
   min_cost_astar escapes a start ONE cell deep in blocked space but returns
   None from TWO cells deep. It therefore returns None here, and
   `_compute_path_cost` returns inf ("Returns inf when the frontier is
   unreachable, so it scores 0 ... and is never selected").
4. All 10 frontiers the base selector finds on the SAME snapshot get
   path_cost = inf under the head (measured, all 10), the head's
   `_rank_frontiers` discards them, `detect_frontiers` returns 0 candidates
   at every decision, and the explorer never emits a goal.
The base selector has no A* in its scoring and is immune (10 candidates, 9
goals, 42 %).

**Responsibility verdict: the behaviour belongs to #2830's code, not to our
adapter or world** (ternary grid faithful to the live convention, their real
min_cost_astar, their inflation port, and a pose their own planner accepts).
Prudent phrasing for any public use: "#2830's scoring A* returns inf for every
frontier whenever the robot stands more than one cell inside the selection
inflation ring, and the selector then emits no goal; reproduction attached."
Honest nuance: a start with more than 0.25 m clearance sidesteps it (from a
free cell 0.11 m away, the same A* reaches the same frontiers), so our
centre-nearest start made it fire AT SPAWN; a live robot would hit it after
stopping near furniture rather than at every start.

## The world (the fix that made the demo honest)

`bigoffice_ply.npz` is built from `grid_ply.npy` = dimOS's own two-pass global
map of the dataset (`big_office.ply` -> their `height_cost_occupancy`),
mapping: cost 100 = occupied, 0 <= cost < 100 = free (their planner drives on
sub-lethal cost), -1 = never observed = a wall in replay (hatched on figures).
**No carving, no repair of any kind.** Verified here at the planner gauge
(lethal 0.225 m): 268 m2 passable, **259 m2 in ONE component (97 %), columns
24-522 of 542** - the whole building is connected as-is. Our earlier
accumulated raster (every transient scored forever) was the artifact that cut
the floor into 188 islands; the dataset's own SLAM product has no such issue.
The accumulated worlds (raw and carved) remain in this workspace as the
contrast annex (`lesh_centre_rep_bigoffice_4m.png`, `lesh_rep_*.gif`,
`NOTES history in git-less files`), nothing re-run on them.

## Start and denominators

- Start = body-passable cell nearest the geometric centre of the FULL map:
  (-5.15, 2.80) m, 0.24 m from the exact centre, inside the largest passable
  pocket (294 m2, rank 1 of 138).
- Grid 1000 m2; observed by the recording 450 m2 (45 %); this start's ceiling
  447 m2 = 99 % of the observed floor. The winning run sees 439 m2 = 98.2 % of
  the ceiling = 97 % of the observed = 44 % of the raster (the rest was never
  recorded and is inexplorable in replay, whatever the strategy).

## Discipline

- All four cells executed twice in separate processes: field-for-field PASS
  (goals, poses, coverage curves, scan counts; wall-clock named and excluded).
- GIF frames replayed from each run's own logged scan positions; final frame
  asserted equal to the harness's dumped grid, cell for cell.
- Upstream selector files untouched: md5 e77c328643c959a49077115e8a341f2c /
  1ccc0c69fe88a72e402565feca988d26.
- Faithful loop (T_sel 15 s, old goal pursued during compute), upstream 15 s
  goal timeout, Go2 body at 0.55 m/s, 4 m lidar.

## Files

- `lesh_centre_ply_bigoffice_ply_4m.png` - the four-panel still (full extent,
  fog of war, denominator box, world provenance line, quit-point curves).
- `lesh_ply_stock_shipped.gif`, `lesh_ply_pr2830_shipped.gif`,
  `lesh_ply_stock_M4.3_complete.gif`, `lesh_ply_pr2830_M4.3_complete.gif`.
- `out/lesh_ply_*.json` (8 runs), `masks/*leshcentre_ply*`,
  `lesh_demo_ply.log`, `lesh_start_ply.json`, `lesh_determinism_ply.txt`,
  `trace_pr_first.py` (the zero-candidate trace),
  `test_pr2830_blocked_start_repro.py` (minimal reproduction, asserts),
  `dissect_snap.npz` (the exact first-decision costmap).

## GIF clock

All four GIFs share ONE simulated-time scale: 1 frame = 20 s simulated (the
longest run, 1464 s, fits in 74 frames), same playback fps; a run that ends
early holds its last frame while the others continue. Stated in each GIF's
legend. No per-goal or per-run sampling: speed comparisons are honest by eye.

## Known limits

1. The pr2830 root cause is pinned and reproduced here on the harness path;
   confirming it on a live dimOS node (real LCM loop) has not been done.
2. Coverage percentages are against this start's ceiling (447 m2), stated on
   the figure; nothing can explore the never-recorded 55 % of the raster.
3. One demo run per arm (deterministic, replayed twice) - a demo, not a bench.

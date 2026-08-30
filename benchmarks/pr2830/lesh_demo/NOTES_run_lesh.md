# The lesh run, final form: the forced quartet on the dataset's own global map

## Is the ping-pong solved, and by WHAT? (metrox's question, answered first)

**On this floor, once every arm is forced to explore everything on a sane
world, the ping-pong barely exists in ANY arm - and the main solver is NOT the
momentum term.** At comparable coverage (97.1 to 97.6 %), same exact centre
start, same conditioning:

| arm (all with COMPLETE stop) | goals | path | coverage | real crossings | round trips |
|---|---|---|---|---|---|
| 1. stock, no momentum (control) | 41 | **254.7 m** | 97.1 % | 1 | 0 |
| 2. stock + signed momentum | 49 | 292.5 m | 97.6 % | 2 | 1 |
| 3. PR #2830, no momentum (control) | 50 | 319.1 m | 97.4 % | 2 | 0 |
| 4. PR #2830 + signed momentum | 49 | **261.1 m** | 97.3 % | **0** | 0 |

What solved the ping-pong here, in order of measured contribution:
1. **The world**: the dataset's own two-pass global map (big_office.ply) is
   connected at the planner gauge with no repair; our accumulated raster
   (transients scored forever) was what created prisons and forced crossings.
2. **The completion stop**: exploring until no reachable frontier remains
   removes the quit-early behaviour that made every earlier picture look
   truncated.
3. **The footprint clearing** (standard ROS practice, applied identically to
   all arms): without it, #2830's scorer never emits a goal from a start
   inside the selection inflation (root cause below, kept as a finding).
4. **The momentum term**: mixed on this single run. On stock it added a
   crossing, a round trip and 38 m; on #2830 it removed both crossings and
   58 m. Counts of 0-2 events on one deterministic run per arm: no champion
   can honestly be declared from this demo. The disease lesh described
   (hallway A, cross, hallway B, cross back, repeatedly) does not appear at
   scale in ANY completed arm on this floor: the old 24-crossing pictures came
   from the broken world and the unfaithful loop, not from the scorers.

Who wins when everyone can play: stock+complete walks the least; #2830+momentum
crosses the least. The honest headline is that BOTH selectors explore this
building fine once the world, the stop and the start conditioning are sane.

## The world and the conditioning (one line each, as on the figure)

- WORLD = dataset's own two-pass global map (`big_office.ply` -> their
  `height_cost_occupancy`; cost 100 = occupied, 0..99 = free, -1 = never
  observed = wall in replay). Connected at the planner gauge, no carving:
  259 m2 in one component (97 %), columns 24-522 of 542. Verified twice.
- SELECTION COSTMAP: the robot's footprint is cleared (marked free) before
  each decision, identically for all arms - standard ROS costmap practice,
  matching min_cost_astar's own one-cell start tolerance. World conditioning,
  not a favor.
- Start = body-passable cell nearest the geometric centre of the full map:
  (-5.15, 2.80) m, 0.24 m from the exact centre. 4 m lidar, Go2 body at
  0.55 m/s, faithful loop (T_sel 15 s, old goal pursued during compute),
  upstream 15 s goal timeout for the shipped arms.

## Finding kept: #2830's blocked-start zero-candidate (root-caused, reproduced)

Without footprint clearing, a spawn 0.112 m (2.2 cells) inside the 0.25 m
selection inflation kills #2830 completely: its scoring A*
(`selector_head._compute_path_cost`, `cost_threshold=occupancy_threshold`)
escapes a start one cell deep in blocked space but returns None from two,
every frontier scores inf ("never selected"), and the selector emits zero
goals forever. The base selector has no A* in scoring and is immune.
Reproduction with asserts: `test_pr2830_blocked_start_repro.py` +
`dissect_snap.npz`. Why no earlier bench ever saw it: every mid-space start
was filtered at >= 0.35 m clearance (`midstarts.CLEAR_MIN_M`), above the
0.25 m ring; the literal centre start dropped that filter. Prudent public
phrasing stays as before; a live robot would hit this after stopping near
furniture rather than at every spawn.

## Denominators (the number must match what the eye sees)

Grid 1000 m2; observed by the recording 450 m2 (45 %); this start's ceiling
447 m2 = 99 % of the observed floor. The quartet's runs see 97.1-97.6 % of
that ceiling; the never-recorded 55 % of the raster is hatched on every
figure and inexplorable in replay whatever the strategy.

## GIF clock

All four GIFs share ONE simulated-time scale, 1 frame = 10 s simulated, same
fps; a run that ends early holds its last frame. Pose continuity verified in
the raw data before choosing the cadence: max consecutive-pose step 0.050 m =
exactly one cell (v_max 0.550 m/s = the cap); apparent teleports at 20 s/frame
were a rendering artefact, not a harness one. The blue path trace grows frame
by frame.

## Discipline

- 12 runs (6 cells x 2 reps, quartet + the two shipped annex arms) in separate
  processes: field-for-field determinism PASS everywhere.
- GIF frames replayed from each run's own logged scan positions; final frame
  asserted equal to the harness's dumped grid, cell for cell.
- Upstream selector files untouched: md5 e77c328643c959a49077115e8a341f2c /
  1ccc0c69fe88a72e402565feca988d26. Nothing pushed, nothing posted.

## Annex: the shipped arms (same conditioning, for context)

stock/shipped: 9 goals, 64.4 m, 42.2 %, info-gain self-stop.
pr2830/shipped: 14 goals, 101.5 m, 50.0 %, info-gain self-stop - with footprint
clearing the PR not only plays, it out-covers stock in its own config.
Pre-footprint history (the 0-goal runs and their figures) archived in
`pre_footprint/`.

## Files

- `lesh_centre_ply_bigoffice_ply_4m.png` - the quartet, full extent, fog of
  war, denominator box, conditioning lines, quit-point curves.
- `lesh_ply_stock_complete.gif`, `lesh_ply_stock_M4.3_complete.gif`,
  `lesh_ply_pr2830_complete.gif`, `lesh_ply_pr2830_M4.3_complete.gif`.
- `out/lesh_ply_*.json` (12 runs), `masks/*leshcentre_ply*`,
  `lesh_start_ply.json`, `lesh_determinism_ply.txt`, logs,
  `test_pr2830_blocked_start_repro.py`, `dissect_snap.npz`,
  `trace_pr_first.py`, `pre_footprint/` (annex).

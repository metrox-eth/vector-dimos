# PR #2830 field notes, and the exploration ping-pong dossier

Offline benchmarks of dimOS frontier exploration on recorded maps. All runs use the
real upstream selector files, unmodified; every volume pre-declares its hypotheses
before running and re-derives its headline counts from raw goal coordinates.

Verdict of the dossier, in its final honest form: **the evaluation world dominates
the outcome.** The same selectors ping-pong everywhere on a raw accumulated map and
barely at all on the dataset's own two-pass global map ([lesh_demo/](lesh_demo/), the
forced quartet: all four arms reach ~97 % coverage from the exact building centre,
0 to 2 crossings each). Simulation verdicts about exploration *strategy* are therefore
fragile; a real robot decides. What survives across worlds: the info-gain self-stop
quits floors early on slow or short-range setups; PR #2830 has a start-in-inflation
edge case (A* returns None from a start 2 cells inside the inflation ring; minimal
repro in lesh_demo/) and performs well once the start is cleared; the signed momentum
term is mixed and map-dependent (it helped on some worlds, added path on the clean
one). Earlier per-volume verdicts stand as historical record of that path, each
corrected in place where wrong.

**The forced quartet**, the maintainer's literal test (spawn at the exact
geometric centre, 4 m lidar, faithful loop timing) on the dataset's own two-pass
global map, every arm forced to explore to completion. All four reach ~97 %
coverage with 0 to 2 cross-map crossings:

![the forced quartet](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/lesh_demo/lesh_centre_ply_bigoffice_ply_4m.png)

Animated runs (common clock, 1 frame = 9 simulated seconds) in [lesh_demo/](lesh_demo/):

![stock, complete stop](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/lesh_demo/lesh_ply_stock_complete.gif)

## The path, newest first

Each volume's verdict was honest at the time and was then revised by a better
instrument. **Every volume before `lesh_demo` ran on worlds built by raw lidar
accumulation** (transients baked in forever), the final volume showed that this
choice of world, not the selectors, produced most of the ping-pong being measured.
Read per-row verdicts as history; the verdict above supersedes them.

| volume | world | question | verdict at the time (struck where superseded) |
|---|---|---|---|
| [lesh_demo/](lesh_demo/) | **official two-pass map** | the maintainer's literal test, all arms forced to completion | **~97 % everywhere, 0-2 crossings, no champion, the world was the variable** |
| [fidelity/](fidelity/) | accumulated | external audit: does anything survive a faithful loop? | swing survives on that world; 47.5 % of path walked during selection computes |
| [resid/](resid/) | accumulated | remaining crossings: fixable or locally forced? | ~~fixable class goes to zero under the remedy~~ (radius-dependent split; see correction_sensibilite.md) |
| [go2/](go2/) | accumulated | re-judged with the Go2's own body and measured speed | ~~the swing is real in Go2 conditions~~ (later shown world-dependent) |
| [cand2/](cand2/) | accumulated | signed direction term, lazy TSP, upstream 15 s timeout | direction term strong where measurable; timeout dominates |
| [fleet/](fleet/) | accumulated (6 new floors) | does the fixed-radius policy generalize? | no; the radius was a one-floor artefact |
| [fix/](fix/) | accumulated | why does each swing happen; does finish-your-branch help? | diagnosis A/B/C; fixed radius helps one floor |
| [midstart/](midstart/) | accumulated | middle spawns, low lidar (maintainer's setup) | swings appear under stock; #2830 does not reduce them |
| this file, below | accumulated | does PR #2830's scoring reproduce its claims? | dispersion yes, travel on large maps only |

Raw run JSONs are versioned in each volume (fidelity/raw, resid, go2, lesh_demo) and the extracted occupancy maps in [maps/](maps/), so every table can be recomputed from this repo alone.

## The original comment posted on PR #2830

# Offline field test of dimensionalOS/dimos#2830 on recorded maps

The harness and results behind our comment on
[dimensionalOS/dimos#2830](https://github.com/dimensionalOS/dimos/pull/2830).
`upstream/` holds the two selector files under test, fetched from GitHub and
not edited (Apache-2.0, (c) Dimensional Inc. and @samuelokpor). Everything
else is our harness. Text below = the comment as posted.

---

First contribution here: we've been using dimOS for about a week, so corrections welcome.

We have an offline harness for comparing exploration strategies on recorded maps, and pointed it at this PR. Both arms are the real files, unmodified: stock = `dimensionalOS/dimos@6fcc4e2` (this PR's base, byte-identical to the installed 0.0.14b1), #2830 = `samuelokpor/dimos@ff9d5ae`; `min_cost_astar` is the installed module, C++ extension included. Only shim: instantiating the selector without an LCM bus. The harness simulates incremental discovery (360-ray 12 m lidar, one revolution per 0.25 m of travel), same planner and parameters for both arms. Maps: four recorded maps from our rover (2D lidar, mecanum, small flat) and your `go2_bigoffice` dataset. There is no costmap stream in the .db, so we accumulated the 2 251 lidar frames and ran `dimos.mapping.pointclouds.occupancy` on them; the extraction chain checks out against the shipped `big_office.ply` → ground-truth PNG at IoU 1.0000. 128 runs, 352 head-to-head decisions. Harness and raw CSVs: https://github.com/metrox-eth/vector-dimos/tree/main/benchmarks/pr2830.

![map extracted from go2_bigoffice.db](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/extracted_map_bigoffice.png)

**Goal dispersion: reproduces.** Median robot→goal distance drops on every map in both configs (12/12). Big office: 8.39 → 6.16 m, goals beyond 5 m 79.8 % → 58.3 %. Head-to-head on identical inputs (same frontiers, costmap and history): same pick about half the time; when they differ, #2830 takes the nearer frontier in 143 of 164 decisions.

![goal sequences, go2_bigoffice, stock left / #2830 right](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/goal_sequences_bigoffice.png)

**Travel at given coverage: reproduces on the large map only.** Big office: path to 80 % of visible ceiling 24.8 → 20.9 m (−15.6 %), total path −7.6 %, better on 10 of 12 paired starts. One caveat against our own favorable number: the 45 s goal timeout our deployment configures (upstream default is 15 s) mostly truncates stock's long goals; in the config without it, the advantage shrinks. Our own flat could not test this leg at all: a geodesic check shows only 0–2.4 % of its visible area lies beyond one lidar range of walking (13.8 % on bigoffice). Nothing to save there.

![coverage vs path, big office](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/coverage_vs_path_bigoffice.png)
![goal sequences, our flat](https://raw.githubusercontent.com/metrox-eth/vector-dimos/main/benchmarks/pr2830/goal_sequences_flat.png)

**Two observations.** (1) Both scorers issue goals at frontiers our 46 cm body cannot reach. The selector inflates the costmap by a fixed 0.25 m. Is that meant to be robot-specific, or should callers pre-inflate to their own footprint? (#2830 abandons such goals faster, unreachable-goal churn median 6.5 to 2, which helped us.) (2) The added A* runs once per candidate, so its cost tracks frontier count: +1.5 % at 15 clusters, +27 % at 29. The overall hot spot remains `detect_frontiers` (pure-Python BFS per decision), unrelated to this PR.

Caveats: `go2_bigoffice` is one recording read through two occupancy algos (12 paired starts, not 24); our simulated body is 46 cm and wheeled, the Go2 that recorded it is ~31 cm and walks, so two thirds of that floor is closed to our body; two of our four flat maps are snapshots of the same map minutes apart; simulated pose is perfect (no slip, no reloc jumps); "better on N/M pairs" are counts, not significance tests.

Happy to rerun any configuration on either map set.
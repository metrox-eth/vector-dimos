# PR #2830 field notes, and the exploration ping-pong dossier

Offline benchmarks of dimOS frontier exploration on recorded maps. All runs use the
real upstream selector files, unmodified; every volume pre-declares its hypotheses
before running and re-derives its headline counts from raw goal coordinates.

| volume | question | answer |
|---|---|---|
| this file, below | does PR #2830's scoring reproduce its claims? | dispersion yes, travel on large maps only |
| [midstart/](midstart/) | middle-of-space starts, low lidar range (maintainer's setup) | swings appear under stock; #2830 does not reduce them |
| [fix/](fix/) | why does each swing happen, and does a finish-your-branch policy help? | diagnosis A/B/C; fixed radius helps one floor |
| [fleet/](fleet/) | does that policy generalize? 6 new floors from public dimOS datasets | no; the radius was a one-floor artefact |
| [cand2/](cand2/) | signed direction term and lazy TSP tour; upstream 15 s timeout | direction term strong where measurable; timeout dominates |
| [go2/](go2/) | everything re-judged with the Go2's own body and measured speed | the swing is real in Go2 conditions; remedy passes on 2 maps |
| [resid/](resid/) | are the remaining crossings fixable or locally forced? | fixable class goes to zero under the remedy; residual is locally forced under R=6 m (see correction_sensibilite.md: the split is radius-dependent) |
| [fidelity/](fidelity/) | external audit found the harness froze the robot during selection and ran above the 0.55 m/s planner cap; does anything survive a faithful loop? | the swing survives strengthened (the frozen harness under-counted); the remedy verdict is unchanged; 47.5 % of path is walked during selection computes |

Raw run JSONs are versioned in each volume (fidelity/raw, resid, go2) and the extracted occupancy maps in [maps/](maps/), so every table can be recomputed from this repo alone.

Verdict of the dossier: about half of stock's cross-map crossings in Go2 conditions
are fixable errors, one signed-direction change removes all of them, and the rest is
a robot that finished a region and has to walk. Full numbers in each volume.

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
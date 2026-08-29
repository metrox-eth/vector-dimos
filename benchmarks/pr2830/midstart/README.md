# PR #2830, re-run from middle-of-space starts with a low-range lidar

Offline bench, no robot, no flight. 2026-08-29. Same harness, same two arms, same map
recording as the shipped big-office bench in `../sim_2830_bigoffice/`. Two parameters
changed, at the request of dimOS maintainer lesh.

## What lesh asked for, and what was changed

> "The real issue: starting from the middle of a space with several hallways, stock explores
> part of one hallway, then walks all the way across to another hallway, then back. From a
> dead-end start of an L-shaped hallway you can't see the bug. Spawn the synthetic robot at
> the middle of the space, and use a low-range lidar (3-5 m)."

Two parameters, and nothing else:

| | shipped bench | this re-run |
|---|---|---|
| simulated lidar range | 12.0 m (RPLIDAR C1) | **4.0 m** primary, 3.0 m and 5.0 m brackets |
| starts | `origin`, `centre`, `pose`, `spread1..4` | **`centre`, `mid1..mid5`** |

Both arms are the unmodified upstream files. `pr2830/selector_base.py` md5
`e77c328643c959a49077115e8a341f2c`, still byte-identical to the file installed in
`/home/openclaw/dimos-rig/.venv` (dimos 0.0.14b1, the PR's base commit 6fcc4e2);
`pr2830/selector_head.py` md5 `1ccc0c69fe88a72e402565feca988d26`
(samuelokpor/dimos @ ff9d5ae). Both configurations `shipped` and `scoring`, the same
ported planner, the same 0.46 m body, the same paired starts for both arms.

Two numbers were **added to the report** (they change no behaviour, they only read the
goal list a run already produced):

- **(a) goal-to-goal distance, total per run**: the sum of the straight-line distances
  between consecutive issued goals.
- **(b) cross-map swings**: how many of those jumps are longer than half the bounding-box
  diagonal of the body-passable floor. That is **11.80 m** on `bigoffice` and **11.83 m**
  on `bigoffice_hc`. These are the direct measure of "walks all the way across, then back".

## How the middle-of-space starts were chosen

`midstarts.py`, a rule and not a hand placement. `centre` is unchanged from the shipped
bench (the body-passable cell closest to the centroid of the largest body-passable region).
`mid1..mid5` come from cells that pass three filters and are then spread apart:

1. **Graph centre.** Approximate geodesic eccentricity inside the body-passable floor,
   keeping the 60 % closest to the centre. A dead end has the highest eccentricity there is,
   so this is the filter that removes exactly the starts lesh objected to. On `bigoffice`,
   eccentricity over the floor runs 12.7 m to 26.1 m and the cut falls at 21.0 m.
2. **Clearance** at least 0.35 m from the nearest obstacle (the body half-width is 0.23 m).
3. **Not a dead end.** Cast 32 rays; a direction is open if the body could drive 1.5 m along
   it; count the contiguous angular runs of open directions (dead end 1, corridor 2, T 3,
   crossing 4). At least 2 required.

Farthest-point sampling over the survivors then picks the five, seeded on `centre`, with the
junction criterion as the tie-break: among candidates within 90 % of the best spread, most
ways out wins, then most clearance.

The junction criterion is a **preference and not a hard filter**, and that is a real
compromise worth naming: on this map only 19 of the 74 central candidate cells have three or
more ways out, and they sit in two clusters. Filtering hard on them returned five starts
inside a 2 m ball, which is not "well separated". The ways-out count of each chosen start is
reported below instead. This map is a big room plus one long corridor, not several hallways.

`bigoffice` starts (closest pair 3.20 m apart):

| start | x, y | clearance | ways out | eccentricity |
|---|---|---|---|---|
| `centre` | -11.90, 9.65 | 0.25 m | 2 | 15.1 m |
| `mid1` | -13.95, 0.80 | 0.85 m | 3 | 20.9 m |
| `mid2` | -11.95, 15.30 | 0.46 m | 3 | 20.8 m |
| `mid3` | -13.95, 5.30 | 0.65 m | 2 | 15.6 m |
| `mid4` | -15.45, 12.30 | 0.35 m | 2 | 19.3 m |
| `mid5` | -9.95, 12.80 | 1.42 m | 2 | 20.4 m |

`bigoffice_hc` gets its own six by the same rule (closest pair 3.08 m); they are listed in
`starts_midstart.txt` and `gate_4m.json`.

## Input-worthiness gate, run before the bench

The same measure as the shipped bench's `traverse.py`, per start, at 4 m: geodesic distance
inside the body-passable floor from the start to every reachable cell, then the share of the
visible ceiling that appears **only** from more than one lidar range of walking.

| map | start | ceiling | one revolution from the spawn | share visible only beyond 4 m of walking |
|---|---|---|---|---|
| `bigoffice` | `centre` | 86.7 m2 | 11.3 m2 = 13.0 % | **45.2 %** |
| `bigoffice` | `mid1` | 86.7 m2 | 11.9 m2 = 13.7 % | **71.7 %** |
| `bigoffice` | `mid2` | 86.7 m2 | 20.6 m2 = 23.7 % | **41.7 %** |
| `bigoffice` | `mid3` | 86.7 m2 | 10.4 m2 = 12.0 % | **71.3 %** |
| `bigoffice` | `mid4` | 86.7 m2 | 8.5 m2 = 9.8 % | **49.1 %** |
| `bigoffice` | `mid5` | 86.7 m2 | 23.8 m2 = 27.5 % | **37.4 %** |
| `bigoffice_hc` | `centre` | 85.5 m2 | 10.3 m2 = 12.1 % | **42.0 %** |
| `bigoffice_hc` | `mid1` | 85.5 m2 | 11.4 m2 = 13.4 % | **71.9 %** |
| `bigoffice_hc` | `mid2` | 85.5 m2 | 21.4 m2 = 25.1 % | **42.2 %** |
| `bigoffice_hc` | `mid3` | 85.5 m2 | 9.6 m2 = 11.3 % | **73.2 %** |
| `bigoffice_hc` | `mid4` | 85.5 m2 | 24.0 m2 = 28.0 % | **38.9 %** |
| `bigoffice_hc` | `mid5` | 85.5 m2 | 19.4 m2 = 22.7 % | **37.8 %** |

Reference: the shipped 12 m bench scored **13.8 %** on this map from its own start, and
0.0 to 2.4 % on the four flat maps.

**Every start passes, none is degenerate.** The pre-declared rule was to drop a start whose
first revolution already reveals 50 % or more of its ceiling; the worst here is 28.0 %.
The ceiling itself barely moved (86.7 m2 at 4 m against 87.9 m2 at 12 m), so the change is
in how much walking it takes to collect it, which is the point.

Brackets: at 3 m the share beyond one range is 47.0 to 80.0 %, at 5 m it is 29.5 to 68.3 %.

## The result: does the ping-pong appear, and does #2830 reduce it?

**Yes, it appears under stock. No, #2830 does not reduce the swing count.**

48 runs at 4 m = 2 maps x 6 starts x 2 configs x 2 arms. No start was dropped: on every
one of them both arms drove more than 1 m.

### Cross-map swings, jumps longer than half the floor diagonal

| config | arm | swings, total | runs with at least one | median per run |
|---|---|---|---|---|
| `shipped` | stock | **11** | **7 / 12** | 1.0 |
| `shipped` | #2830 | **11** | **7 / 12** | 1.0 |
| `scoring` | stock | **22** | **9 / 12** | 1.5 |
| `scoring` | #2830 | **20** | **11 / 12** | 1.0 |

The swings are real and they are late: the first one lands on the 7th goal in median (stock,
both configs), and the median swing sits 0.78 of the way through its run. The ten longest
single jumps of the sweep are 19.0 to 20.0 m, which on a floor whose bounding box is
11.6 x 20.6 m is one end to the other. Both arms produce them; six of the ten belong to
#2830 or stock in roughly equal share (5 stock, 5 #2830).

### The round trip, which is what lesh actually described

A swing on its own is one long walk, which can be the right move once the near frontiers are
gone. The complaint is the **return**: cross, then come back. Counted as a swing followed
within the next three jumps by another swing whose direction opposes it:

| config | stock | #2830 |
|---|---|---|
| `shipped` | 2 round trips, in **1 / 12** runs | 3 round trips, in **2 / 12** runs |
| `scoring` | 13 round trips, in **6 / 12** runs | 7 round trips, in **3 / 12** runs |

This is the one place where the PR moves the needle on the behaviour under test, and it moves
it in the right direction: in the `scoring` config, half the stock runs contain a round trip
and a quarter of the #2830 runs do. It is 6 runs against 3 runs. That is a count on 12 pairs,
not a test, and in the `shipped` config, where the 45 s goal timeout cuts a long walk off
after about 6.7 m, the effect is not visible at all (1 run against 2).

### Total goal-to-goal distance

| config | arm | total over 12 runs | median per run | #2830 lower on |
|---|---|---|---|---|
| `shipped` | stock | 538.8 m | 41.87 m | |
| `shipped` | #2830 | **558.6 m** | 39.58 m (-5.5 %) | 5 / 12 pairs |
| `scoring` | stock | 698.5 m | 62.72 m | |
| `scoring` | #2830 | **708.0 m** | 47.03 m (-25.0 %) | 7 / 12 pairs |

The median and the sum disagree, and the disagreement is the finding. #2830's median run has
a shorter goal sequence, but summed over all runs it has a slightly **longer** one, in both
configs. It also issues more goals (121 against 111, 115 against 106). So the median
improvement is a few runs being much tidier, not a systematic reduction.

## The rest of the shipped metrics, at 4 m

Medians over runs, and the paired count of how often #2830 is on the better side. All 12
start pairs per config unless noted.

### Config `shipped` (upstream code as-is, 45 s goal timeout), 24 runs

| metric | stock | #2830 | change | #2830 better on |
|---|---|---|---|---|
| median robot to goal | 3.97 m | 3.91 m | -1.6 % | 10 / 12 |
| max robot to goal | 11.20 m | 9.98 m | -10.9 % | 6 / 12 |
| median goal to goal | 4.14 m | 3.76 m | -9.3 % | 8 / 12 |
| goal to goal, total per run | 41.87 m | 39.58 m | -5.5 % | 5 / 12 |
| cross-map swings | 1.0 | 1.0 | 0.0 % | 4 / 12 |
| path to 50 % of the ceiling | 9.26 m | 7.33 m | -20.8 % | 5 / 12 |
| path to 80 % of the ceiling | 28.84 m | 29.61 m | +2.7 % | 5 / 8 |
| path to 90 % of the ceiling | 38.14 m | 35.07 m | -8.1 % | 2 / 2 |
| total path | 36.79 m | 38.09 m | +3.6 % | 6 / 12 |
| final coverage | 85.5 % | 88.1 % | +2.7 points | 7 / 12 |
| goals reached | 4.0 | 5.0 | | 9 / 12 |
| goals after the last one reached | 3.0 | 2.0 | -33.3 % | 7 / 12 |
| decision time | 10 433 ms | 10 886 ms | +4.3 % | 3 / 12 |

All goals pooled: stock n=111, median 3.98 m, p90 9.29 m, **25.2 % beyond 5 m**;
#2830 n=121, median 3.92 m, p90 8.36 m, **21.5 % beyond 5 m**.

### Config `scoring` (their ranking, no self-stop, no goal timeout), 24 runs

| metric | stock | #2830 | change | #2830 better on |
|---|---|---|---|---|
| median robot to goal | 3.99 m | 3.94 m | -1.2 % | 8 / 12 |
| median goal to goal | 3.92 m | 3.79 m | -3.4 % | 9 / 12 |
| goal to goal, total per run | 62.72 m | 47.03 m | -25.0 % | 7 / 12 |
| cross-map swings | 1.5 | 1.0 | -33.3 % | 5 / 12 |
| path to 50 % of the ceiling | 9.32 m | 7.33 m | -21.4 % | 5 / 12 |
| path to 80 % of the ceiling | 29.36 m | 31.92 m | +8.7 % | 3 / 12 |
| path to 90 % of the ceiling | 52.52 m | 60.46 m | +15.1 % | 2 / 6 |
| total path | 60.03 m | 55.05 m | -8.3 % | 4 / 12 |
| final coverage | 90.4 % | 90.2 % | -0.3 points | 4 / 12 |
| goals after the last one reached | 2.0 | 2.5 | +25.0 % | 2 / 12 |
| decision time | 10 521 ms | 10 719 ms | +1.9 % | 6 / 12 |

All goals pooled: stock n=106, median 3.99 m, p90 13.08 m, **38.7 % beyond 5 m**;
#2830 n=115, median 3.96 m, p90 11.15 m, **32.2 % beyond 5 m**.

### Unreachable-goal churn

| config | arm | goals | reached | timed out | no path | issued after the last one reached | body contacts |
|---|---|---|---|---|---|---|---|
| `shipped` | stock | 111 | 43 | 58 | 10 | 40 | 2 |
| `shipped` | #2830 | 121 | 63 | 48 | 10 | **26** | 1 |
| `scoring` | stock | 106 | 63 | 0 | 43 | **31** | 9 |
| `scoring` | #2830 | 115 | 67 | 0 | 48 | 44 | 8 |

The churn goes the PR's way in `shipped` (26 against 40) and the other way in `scoring`
(44 against 31). It does not point one direction.

### The dispersion result of the shipped bench does not survive the range change

This is the largest single difference from the 12 m volume. There, the PR's headline was that
it picks much nearer frontiers: median robot to goal 8.15 m to 6.20 m, -24 %, and 79.8 % of
goals beyond 5 m falling to 58.3 %. At 4 m the two arms are **1.2 to 1.6 % apart** on that
metric. The reason is mechanical: with a 4 m lidar the frontier ring is never far from the
robot, so there is much less for a distance-aware score to reorder.

## Head to head: same map state, same candidates

One arm drives; at each of its decisions the other is handed the identical frontier
centroids, the identical cluster sizes, the identical costmap and the identical
explored-goal history, and asked what it would have taken.

| | decisions | same choice | when they differ |
|---|---|---|---|
| stock drives | 106 | 80 (75 %) | stock 5.94 m / #2830 3.54 m, #2830 nearer 23 / 26 |
| #2830 drives | 115 | 89 (77 %) | #2830 3.78 m / stock 7.13 m, #2830 nearer 24 / 26 |

**221 decisions, same choice 169 times (76 %); when they diverge, #2830 takes the nearer
frontier in 47 of 52 cases (90 %).** At 12 m the same test gave 57 % agreement and 81 %.
So at short range they agree far more often, and the divergences still go the same way.

The PR's own premise weakens at this range. The real A* path exceeds the straight line by a
median factor of **1.10x** (p90 2.26x), and only **13.3 %** of goals cost more than a 50 %
detour. At 12 m those were 1.27x and 40.9 %. When every candidate is roughly 4 m away in a
room the robot is standing in, the straight line is already a good estimate of the route, so
replacing it with an A* cost has less to correct.

## Lidar-range brackets, 3 / 4 / 5 m

The 4 m sweep of 48 runs took about 19 minutes of wall clock on 10 workers, so three full
sweeps would have been about an hour, over the time budget set for this job. The brackets
were therefore run on **`bigoffice` only** (24 runs each, 6 start pairs per config), and the
bracket figure compares only that map at all three ranges so the bars share a denominator.

| metric (bigoffice, both configs pooled) | 3 m stock / #2830 | 4 m stock / #2830 | 5 m stock / #2830 |
|---|---|---|---|
| median robot to goal | 3.0 / 3.0 m | 4.0 / 3.9 m | 4.9 / 4.4 m |
| share of goals beyond 5 m | 26.8 / 17.4 % | 26.1 / 23.6 % | 35.0 / 27.9 % |
| goal to goal, median total per run | 35.6 / 33.1 m | 41.9 / 47.0 m | 30.2 / 39.5 m |
| **cross-map swings, total over 12 runs** | **3 / 3** | **11 / 11** | **7 / 12** |
| path to 80 % of the ceiling | 37.6 / 33.6 m | 29.0 / 30.7 m | 25.6 / 25.8 m |
| total path | 54.5 / 50.2 m | 44.0 / 44.3 m | 40.7 / 45.1 m |

There is no monotone trend in the swing count across the three ranges, and at 5 m #2830 has
more swings than stock (12 against 7). The counts are small (3 to 12 events over 12 runs),
which is the honest description of that row.

## Verdict, plainly

1. **The behaviour lesh described is present under stock from middle-of-space starts with a
   4 m lidar.** Stock produces 11 cross-map swings in 7 of 12 runs in the shipped config and
   22 in 9 of 12 in the scoring config. In the scoring config those 22 swings contain 13
   there-and-back pairs, spread over 6 of the 12 runs.
2. **#2830 does not reduce the swing count.** 11 against 11 in the shipped config, 20 against
   22 in the scoring config with more runs affected (11 of 12 against 9 of 12). Summed over
   runs it also travels slightly more total goal-to-goal distance than stock, in both
   configs.
3. **The one indicator that moves the PR's way is the round trip in the scoring config**:
   6 of 12 stock runs contain one, 3 of 12 #2830 runs do. That is 6 against 3.
4. **The mid-start, short-range condition does not make the difference between the two arms
   bigger; it makes it smaller.** The nearer-frontier advantage that dominated the 12 m
   volume shrinks from -24 % to -1.6 %, the two arms agree on 76 % of decisions instead of
   57 %, and the A* detour the PR corrects for drops from 1.27x to 1.10x median.

The result is mixed, and where it is not mixed it is null. The stock explorer does swing
across this map from the middle, and this PR is not the change that stops it.

## Caveats, in order of how much they matter

1. **One recording.** `bigoffice` and `bigoffice_hc` are the same recorded dimOS run read by
   two occupancy algorithms, not two offices. The 48 runs are 12 start pairs, not 24.
2. **The bench body is not the dataset's body.** The Go2 that recorded this map is about
   31 cm wide and walks. The simulated rover is 46 cm, rolls, and needs a 60 cm aisle. Two
   thirds of the free floor is closed to it, so the bench measures the exploration of the big
   room plus its corridor, not the whole floor.
3. **This map is not "several hallways".** The body-passable floor is one large room plus one
   20 m corridor and a small area at its far end. That is why the junction criterion had to
   become a tie-break rather than a filter, and it limits how directly this map can answer
   the question lesh asked.
4. **The simulated pose is perfect.** No wheel slip, no map rollback, no relocalization jump.
   A reversal in simulation is a decision, never noise.
5. **Unknown is a wall by construction.** What the Go2 never saw is unreachable here,
   otherwise the simulated rover drives blind into walls it could not have known about.
6. **Counts, not significance tests.** Every "better on N of M" is a count over 12 paired
   starts. The 90 % rows have as few as 2 pairs and carry nothing.
7. **The swing threshold is a choice.** Half the passable-floor bounding-box diagonal gives
   11.80 m on `bigoffice` and 11.83 m on `bigoffice_hc`. The raw goal-to-goal distances are
   in the CSV and JSON; a different threshold gives a different count.
8. **The 45 s goal timeout in config `shipped` cuts a walk at about 6.7 m** at 0.15 m/s. That
   is why the round-trip effect only shows in `scoring`: in `shipped` neither arm is allowed
   to finish a long crossing.
9. **Neither scorer models body width.** The costmap they are handed is inflated 0.25 m while
   the rover's lethal radius is 0.30 m, so both aim at frontiers sitting in pinches the body
   does not fit through. That is unchanged from the shipped bench and it is not the PR's
   fault.
10. **Decision times are measured under 10-way contention**, so the absolute milliseconds are
    not comparable with the shipped bench's isolated `cout_decision.py` measurement. Under
    the same contention the shipped 12 m bench gave 16 410 ms against 16 906 ms (+3.0 %); here
    it is 10 433 ms against 10 886 ms (+4.3 %).
11. **The brackets are one map only** (see above), so they are a range check, not a second
    full volume.

## Files produced, all in this directory

Absolute path of the workspace:
`/tmp/claude-1000/-home-openclaw-lerobot/c23c4729-e991-4dc8-b2d5-9c140c6a8780/scratchpad/sim_2830_midstart/`

| file | what |
|---|---|
| `midstarts.py` | the middle-of-space start selection, new |
| `gate_midstart.py` | the input-worthiness gate at a chosen lidar range, new |
| `plot_midstart.py`, `plot_brackets.py` | the figures, new |
| `bench_2830.py` | shipped bench + `--lidar-range`, `--starts`, `--drop-starts`, and the two ping-pong numbers |
| `shadow_2830.py` | shipped head-to-head + `--lidar-range`, `--starts` |
| `make_outputs.py` | shipped aggregates + the ping-pong rows and columns |
| `results_midstart_4m.json` | 48 runs at 4 m, full poses / goals / coverage curves |
| `results_midstart_3m.json`, `results_midstart_5m.json` | 24 runs each, `bigoffice` only |
| `resultats_midstart_4m.csv`, `..._3m.csv`, `..._5m.csv` | one row per run, 29 columns (French headers, as in the shipped bench) |
| `shadow_midstart_4m.json` | the 221 head-to-head decisions |
| `agregats_midstart_4m.txt`, `..._3m.txt`, `..._5m.txt` | the aggregate tables above |
| `agregats_shadow_midstart_4m.txt` | the head-to-head table |
| `gate_4m.json`, `gate_4m.txt`, `gate_3m.*`, `gate_5m.*` | the worthiness gate, per start |
| `starts_midstart.txt` | the start selection with its diagnostics |
| `rederivation_pingpong.txt` | the two ping-pong numbers recomputed from raw goal coordinates |
| `pingpong_par_depart.txt` | per start, plus the same two numbers on the shipped 12 m volume |
| `pingpong_structure.txt` | where in a run the swings fall, and the round-trip counts |
| `churn_midstart_4m.txt` | the unreachable-goal churn table |
| `goal_sequences_midstart.png` | `bigoffice`, config `shipped`, all 6 starts, stock left / #2830 right |
| `goal_sequences_midstart_hc.png`, `..._scoring.png`, `..._hc_scoring.png` | the other three map/config combinations |
| `coverage_vs_path_midstart.png` | coverage against path, one line per start, both configs |
| `brackets_midstart.png` | the headline metrics at 3 / 4 / 5 m |
| `bench_4m.log`, `bench_3m.log`, `bench_5m.log`, `shadow_4m.log` | the run logs |
| `_from_shipped_bench/` | the shipped bench's own outputs, moved aside untouched |

Nothing was committed, pushed or posted. The shipped workspace
`../sim_2830_bigoffice/` was copied, not modified. `/home/openclaw/vector-dimos` and the
dimos venv were read only.

## Re-derivation of the surprising numbers

The two ping-pong indicators were recomputed from the raw goal coordinates in
`results_midstart_4m.json`, independently of the `summarise()` fields that wrote them
(`rederivation_pingpong.txt`). The first pass, using a flat 11.8 m threshold, disagreed with
the stored counts by exactly one swing in two of the four arm/config cells; using each map's
own threshold (11.80 m and 11.83 m) the disagreement is zero in all four. The totals
538.8 / 558.6 / 698.5 / 708.0 m and the counts 11 / 11 / 22 / 20 are confirmed.

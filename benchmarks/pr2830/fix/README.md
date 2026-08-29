# A fix for the frontier-exploration ping-pong: diagnosis, sweep, result

Offline, no robot, no flight. 2026-08-29. Workspace
`./`,
a copy of the mid-start benchmark workspace `../sim_2830_midstart/`, which was not modified.

**The two upstream selector files were never touched.** At the end of this work
`pr2830/selector_base.py` is md5 `e77c328643c959a49077115e8a341f2c` and
`pr2830/selector_head.py` is md5 `1ccc0c69fe88a72e402565feca988d26`, as required.
`selector_base.py` is also still byte-identical to the file installed in
``.
Everything added here is a harness-side wrapper in `fix_hysteresis.py`, installed on one
selector instance.

## Summary in four lines

1. Of 64 cross-map swings in the mid-start benchmark, **19 are not crossings at all** (the
   goal jumped, the robot did not), **14 are a near frontier losing the ranking by a factor
   of 1.01 to 1.55**, 8 are near frontiers the drive planner had already refused, **2** are a
   frontier cluster blinking out, and **21 are legitimate**: nothing within 6 m and nothing
   comes back.
2. The dominant fixable cause is the near-tie. Every one of the 14 is recoverable with a
   switching margin of k <= 2 at a 6 m vicinity.
3. Five candidates were swept as pre-declared. **All five pass; the winner is candidate B at
   R = 9 m** (finish-the-branch, geodesic radius 9 m), the only one that also stays inside
   the path budget in each configuration taken alone.
4. On the full 4 m volume (2 maps, 6+6 middle starts, both configs): **stock 33 swings to 21
   and 15 round trips to 6; PR #2830 31 swings to 19 and 10 round trips to 3**, with total
   path -3.2 % and -1.3 % and coverage +0.3 and +0.2 points. At 12 m with the shipped
   bench's own starts it does not hurt either: **stock 14 swings to 10, #2830 14 to 8**.

The honest limit is in the diagnosis, not in the result: about half the real crossings on
this map are forced by its shape, and no re-ranking will remove them.

---

## 1. Diagnosis

Full write-up in `diagnostic_swings.md`. What matters here:

The 48 mid-start runs were re-run with a per-decision trace that records, at every
decision, the candidates the arm's own detector produced, the cluster size and the score
the arm's own scorer gave each of them, which were suppressed by the harness failed-goal
filter, and the goal taken; and, on swing decisions, the route length from the robot to
every candidate under the planner's own cost rule. The score is captured by wrapping the
bound method on the instance.

**The traced run reproduces the recorded one exactly**: 48 runs, `n_goals`,
`cross_map_swings`, `path_m`, `area_m2` and `goal_jump_total_m` identical to 1e-6, swing
totals 11 / 11 / 22 / 20 as in `../sim_2830_midstart/`.

| class | what it means | count | share of 64 |
|---|---|---|---|
| **N** | the goal sequence jumped, the robot did not: it was never on the previous goal | **19** | 30 % |
| **A** | a near frontier existed, was available, and lost the ranking | **14** | 22 % |
| **A-blocked** | a near frontier existed but the drive planner had already refused it | **8** | 12 % |
| **B** | nothing near, but the region had a candidate before and one comes back | **2** | 3 % |
| **C** | nothing near, nothing comes back: the area was exhausted | **21** | 33 % |

Of the **45 real crossings** (N removed): C 47 %, A 31 %, A-blocked 18 %, B 4 %.

Per arm and configuration:

| config | arm | swings | N | A | A-blocked | B | C |
|---|---|---|---|---|---|---|---|
| `shipped` | stock | 11 | 4 | 4 | 0 | 0 | 3 |
| `shipped` | #2830 | 11 | 3 | 2 | 0 | 1 | 5 |
| `scoring` | stock | 22 | 6 | 6 | 2 | 1 | 7 |
| `scoring` | #2830 | 20 | 6 | 2 | 6 | 0 | 6 |

**Cause A is the dominant fixable cause and the margin needed is tiny.** In those 14
decisions the near candidate that lost sits at a median of 2.4 m of route length while the
goal taken is at a median of 19.3 m, and the scores are near-ties: score(taken) /
score(best near) has median **1.28** and maximum **1.55**.

    k = 1.5 would have kept the robot near on 13 / 14
    k = 2   would have kept the robot near on 14 / 14

**Cause B is 3 %.** Per the plan, the frontier-persistence stabiliser was therefore not
run: there was nothing for it to fix on these maps, and the compute went to the cause that
carries 31 % instead. The code for it exists in `fix_hysteresis.py` (`Persist`, spec `P1`)
and is documented as untested here.

**Cause C is 47 % of the real crossings and is partly irreducible.** At those decisions the
nearest available alternative is at a median of 19.0 m and 6 of the 21 have no other
candidate at all. This floor is one large room plus one 20 m corridor: a robot that reaches
the far end must come back down it. Any claim that a re-ranking policy removes the
ping-pong here would be false.

**But two thirds of the class-C crossings were set up earlier.** For every swing the trace
was searched backwards for the most recent decision that took a goal beyond 6 m while an
available candidate sat within 6 m. 14 of the 21 class-C swings have one, a median of six
goals earlier. The crossing was forced when it happened; the situation was not. That is why
a local-first policy is worth testing despite C being the largest class: it acts at those
earlier decisions too.

### Why the near-ties happen

Visible in the stock score. `explored_goals_score` is 30 % of the total and equals
`min(distance to the nearest already-issued goal / 10 m, 1)`. Every goal the robot issues
becomes a repeller. A frontier 2.5 m from where the robot is standing has just been
devalued by the goal the robot is standing on; a frontier 20 m away collects the full 1.0.
The 20 % `distance_score`, `1 / (1 + |d - 5 m|)`, does not offset it, because at a 4 m
lidar range nearly every candidate sits within a couple of metres of the 5 m lookahead and
that term is almost flat across the candidate set. Ten of the fourteen class-A swings are
stock; four are #2830, whose `revisit_distance` fade has the same shape at a smaller scale.

---

## 2. The candidates, and what each one cites

Both are harness-side wrappers in `fix_hysteresis.py`. Neither computes a score: they call
the selector's own `detect_frontiers`, which ends with the selector's own `_rank_frontiers`,
and re-order the list that comes back using the selector's own scores. Because both bench
configurations reach the goal through `detect_frontiers`, installing the wrapper there makes
the policy apply to both arms and both configurations with no other change, and the
selector's own bookkeeping (`_update_exploration_direction`, `mark_explored_goal`) follows
the policy's choice exactly as it would have followed its own.

**Candidate H, switch margin on the arm's own score.** Keep the best-scoring candidate
within a 6 m geodesic vicinity unless the best candidate anywhere beats it by a factor k.
Swept k in {1.5, 2, 3}.

> Precedent: **explore_lite** (Jiri Horner, ROS; 1.0.0 released 2016-05-11, algorithm
> unchanged since 2.1.1 in December 2017, shipped in Kinetic through Noetic). Its entire
> frontier cost is
> `potential_scale * min_distance * resolution - gain_scale * size * resolution`
> (`frontier_search.cpp`), sorted ascending, lowest wins, with the shipped launch files
> setting `potential_scale = 3.0` against `gain_scale = 1.0`. Distance from the robot is one
> of only two terms, so a remote frontier has to be a great deal larger before it is worth
> walking to. That is the borrowed element: **distance from the robot as a first-class term
> rather than one of five weighted at 0.2.**
>
> Stated plainly because it affects how this may be cited: **explore_lite itself has no
> hysteresis.** Its only "do not resend" rule is exact equality with the previous goal point
> (a 1 cm tolerance in the community ROS 2 port), which is a de-duplication guard, not an
> incumbent preference; it re-runs the full argmin every `planner_frequency` tick and
> switches target freely. `orientation_scale` is read and never used. The margin k is our way
> of expressing a distance-dominant cost on a scorer we are not allowed to modify.

**Candidate B, finish-the-branch.** While any candidate lies within geodesic radius R of the
robot, candidates outside R are not eligible. Swept R in {6, 9} m. B is H with k = infinity.

> Precedent: **room / segment based coverage, Bormann et al.** (Fraunhofer IPA,
> `ipa_coverage_planning`). `ipa_room_segmentation` cuts the floor plan into rooms,
> `ipa_building_navigation` orders them as a travelling-salesman tour over room centres with
> A* path distances as edge weights, and `ipa_room_exploration` plans a path covering **one**
> room; the robot finishes the room it is in before the sequence moves it to the next.
> *Room Segmentation: Survey, Implementation, and Analysis*, Bormann, Jordan, Li, Hampp,
> Hägele, ICRA 2016, doi 10.1109/ICRA.2016.7487234; *Indoor Coverage Path Planning: Survey,
> Implementation, Analysis*, Bormann, Jordan, Hampp, Hägele, ICRA 2018,
> doi 10.1109/ICRA.2018.8460566. The three stages are provided by those packages; the
> per-room execution loop is left to the application.
>
> The difference to name: **our "room" is a ball of radius R, not a detected room.** No
> segmentation runs here. The mechanism borrowed is "finish where you are before you are
> allowed to leave", not the segmentation itself.

**Candidate P, hold a cluster one extra cycle.** Written, not run, because cause B is 3 %.
Its precedent would be the cross-cycle frontier memory in explore_lite's blacklist
(`goalOnBlacklist` matches a fresh frontier against remembered positions within a 5-cell
box), used in the opposite direction: that code remembers frontiers in order to refuse
them, never to keep them alive. If the deeper fix for cluster stability is ever needed it
belongs in `detect_frontiers` upstream, not in a wrapper.

**The vicinity metric.** Route length from the robot under the planner's own cost rule
(`explore_sim.plan`: blocked at 100, unknown at 80, which is what `RecoveringGlobalPlanner`
passes to `min_cost_astar`), solved as **one Dijkstra wave from the robot per decision** and
read off at each candidate. That is the shape explore_lite uses: navfn spreads one potential
field per cycle and every frontier's cost is read out of it. Blocking at 99 instead, which
is what PR #2830's own frontier A* does, walls off every pinch the Voronoi gradient touches
and makes almost everything look unreachable: measured on one decision, it reported 30.6 m
for a candidate 3.8 m away.

---

## 3. Pre-declared hypothesis and sweep

Written before any sweep was run, in `hypotheses_predeclared.txt`.

> Compared with the **same arm without the wrapper**, on the same paired starts:
> (1) total cross-map swings must drop; (2) total round trips must not rise; (3) median
> total path must not be worse than +5 % relative; (4) median final coverage must not be
> worse than -5 % relative. A candidate passes only if all four hold.
> Winner: largest absolute drop in total swings, ties broken by fewer round trips, then by
> lower median path. No tuning past the declared values.

Sweep volume: `bigoffice` only, 4 m lidar, the 6 middle starts, both configs, the stock arm,
**12 runs per candidate**, against the same 12 stock baseline runs.

| candidate | swings | round trips | median path | coverage | verdict (pooled) | path, `shipped` | path, `scoring` |
|---|---|---|---|---|---|---|---|
| `H` k=1.5, R=6 | 11 -> 8 | 4 -> 1 | -4.4 % | -0.2 % | PASS | -4.0 % | **+6.9 %** |
| `H` k=2, R=6 | 11 -> 8 | 4 -> 1 | -4.4 % | -0.2 % | PASS | -4.0 % | **+10.4 %** |
| `H` k=3, R=6 | 11 -> 8 | 4 -> 1 | -4.4 % | -0.2 % | PASS | -4.0 % | **+10.4 %** |
| `B` R=6 | 11 -> 8 | 4 -> 1 | -4.4 % | -0.2 % | PASS | -4.0 % | **+10.4 %** |
| **`B` R=9** | **11 -> 6** | **4 -> 0** | **-2.5 %** | **+0.1 %** | **PASS** | +1.2 % | -3.7 % |

Three things to report plainly:

- **The k sweep is nearly degenerate at R = 6 on this map.** k = 2, k = 3 and k = infinity
  (which is B at R = 6) produce **identical goal sequences on all 12 runs**; k = 1.5 differs
  from them on exactly one run (`mid5`, `scoring`), and that single run is the whole
  difference between +6.9 % and +10.4 % of path in the `scoring` column. All four change the
  goal sequence against plain stock in 7 of the same 12 runs. The diagnosis predicted this:
  every binding decision has a score ratio below 1.55, so any margin at or above that
  behaves like an infinite one. **The margin does not matter here; the radius does.**
  k = 1.0 would be a different policy and was not in the declared sweep.
- **My pre-declaration did not say whether the +5 % path budget is pooled or per config.**
  Pooled, all five pass. Per configuration, every R = 6 variant exceeds it in `scoring`
  (+6.9 to +10.4 %) and only B at R = 9 stays inside in both (+1.2 % and -3.7 %). Both
  readings are given; the winner is the same under the strict one, which is why it is the
  winner and not just the pooled-verdict winner.
- **Candidate P was not run**, per the plan, because cause B is 2 swings out of 64.

**Winner: `B9`, finish-the-branch with a 9 m geodesic radius.**

---

## 4. The four-arm result, 4 m, both maps

48 baseline runs (`results_trace_4m.json`, identical to the recorded mid-start bench) plus
48 new runs of the two `+B9` arms (`results_fix_4m.json`). 2 maps x 6 middle starts x 2
configs x 4 arms = 96 runs. Every number below was **re-derived from the raw goal
coordinates** with each map's own threshold (11.80 m and 11.83 m), independently of the
`summarise()` fields that wrote them: **zero disagreement on all 8 arm/config cells.**

### Config `shipped` (upstream code as-is, 45 s goal timeout), 12 runs per arm

| arm | swings | runs with >= 1 | round trips | runs with >= 1 | goal to goal, total | total path | median path | median coverage | median path to 80 % |
|---|---|---|---|---|---|---|---|---|---|
| stock | 11 | 7 / 12 | 2 | 1 / 12 | 538.8 m | 444.3 m | 36.79 m | 85.5 % | 27.94 m |
| **stock + B9** | **4** | **4 / 12** | **0** | **0 / 12** | **455.8 m** | 436.0 m | 36.39 m | 85.5 % | 27.34 m |
| #2830 | 11 | 7 / 12 | 3 | 2 / 12 | 558.6 m | 459.3 m | 38.09 m | 88.1 % | 29.61 m |
| **#2830 + B9** | **5** | **4 / 12** | **0** | **0 / 12** | **436.2 m** | 457.5 m | 38.61 m | 89.2 % | 30.25 m |

### Config `scoring` (their ranking, no self-stop, no goal timeout), 12 runs per arm

| arm | swings | runs with >= 1 | round trips | runs with >= 1 | goal to goal, total | total path | median path | median coverage | median path to 80 % |
|---|---|---|---|---|---|---|---|---|---|
| stock | 22 | 9 / 12 | 13 | 6 / 12 | 698.5 m | 747.3 m | 60.03 m | 90.4 % | 29.36 m |
| **stock + B9** | **17** | 9 / 12 | **6** | **4 / 12** | **642.4 m** | 727.4 m | 56.72 m | 91.0 % | 27.30 m |
| #2830 | 20 | 11 / 12 | 7 | 3 / 12 | 708.0 m | 761.2 m | 55.05 m | 90.2 % | 31.92 m |
| **#2830 + B9** | **14** | 11 / 12 | **3** | **2 / 12** | **566.5 m** | 614.0 m | 50.36 m | 90.2 % | 30.06 m |

### The pre-declared test, both configs pooled, 24 paired starts per arm

| | swings | round trips | median total path | median coverage | verdict |
|---|---|---|---|---|---|
| stock + B9 against stock | **33 -> 21** | **15 -> 6** | 44.25 -> 42.83 m, **-3.2 %** | 90.01 -> 90.30 %, **+0.3** | **PASS** |
| #2830 + B9 against #2830 | **31 -> 19** | **10 -> 3** | 44.28 -> 43.72 m, **-1.3 %** | 89.73 -> 89.94 %, **+0.2** | **PASS** |

Both hypotheses hold on both arms. The wrapper works on top of either scorer, which is what
it was built to do.

Read against the diagnosis: the fix removes 12 of the 33 stock swings. The diagnosis found
14 class-A swings across all four arm/config cells, of which 10 belong to stock. So the
policy is doing roughly what its target predicted, plus a few extra through knock-on at the
earlier abandonment decisions, and it is not touching the class-C crossings, which is
correct behaviour: there is nothing near to prefer.

Figures, same visual style as the mid-start bench (black dot = start, thin arrows = the jump
the selector asked for, numbers = goal order, **orange segment = a cross-map swing**), four
columns stock / stock+B9 / #2830 / #2830+B9:

    goal_sequences_fix_bigoffice_shipped.png
    goal_sequences_fix_bigoffice_scoring.png
    goal_sequences_fix_bigoffice_hc_shipped.png
    goal_sequences_fix_bigoffice_hc_scoring.png

---

## 5. Regression: 12 m lidar, the shipped bench's own starts

Does the fix hurt the normal condition? `bigoffice`, 12 m simulated RPLIDAR C1, the shipped
bench's own starts (`centre`, `pose`, `spread1..4`; `origin` is not body-passable and is
dropped, as it was there), both configs. Baseline read straight out of
`../sim_2830_bigoffice/results_bigoffice.json`; only the two `+B9` arms were run, 24 runs.
The two ping-pong numbers do not exist in that older file and were backfilled from its raw
goal coordinates.

| | swings | round trips | median total path | median coverage | verdict |
|---|---|---|---|---|---|
| stock + B9 against stock | **14 -> 10** | **6 -> 2** | 37.39 -> 38.25 m, +2.3 % | 90.14 -> 90.30 %, +0.2 | **PASS** |
| #2830 + B9 against #2830 | **14 -> 8** | **3 -> 0** | 37.09 -> 35.72 m, -3.7 % | 90.36 -> 90.57 %, +0.2 | **PASS** |

It does not hurt the 12 m condition; on this map it improves it. The median robot-to-goal
distance falls hard there (stock 8.77 to 6.67 m in `shipped`, 7.37 to 5.03 m in `scoring`),
which is the same effect PR #2830 was reaching for at that range, reached by a different
route.

---

## 6. What it costs

Measured in isolation on one worker, over 8 decisions of a real run: the added Dijkstra wave
takes a **median of 95 ms** against a **median of 7 881 ms** for `detect_frontiers` and its
ranking, i.e. **+1.2 % of the decision**. The frontier detector's own full-grid BFS in
Python dominates by two orders of magnitude.

The bench tables show +19 to +40 % on `decide_ms_mean`. That number is **not** the cost of
the policy: those runs execute under 12-way contention and, more importantly, they follow
different trajectories, so they are comparing decisions taken on different maps. The
isolated measurement is the one to quote.

---

## 7. Caveats, in order of how much they matter

1. **About half the real crossings on this map are irreducible.** 21 of the 45 have nothing
   within 6 m and nothing comes back. The fix does not touch them and should not be sold as
   if it did.
2. **30 % of the counted swings are not crossings.** The metric is defined on the goal
   sequence, and when a goal times out or is refused the robot is not on it. Any report
   quoting a swing count on this bench should say this. It also means the swing counts of
   the mid-start report slightly overstate the behaviour lesh described.
3. **One recording.** `bigoffice` and `bigoffice_hc` are the same recorded dimOS run read by
   two occupancy algorithms, not two offices. The 96 runs are 12 start pairs, not 24. This
   map is one large room plus one corridor, not "several hallways".
4. **Counts, not significance tests.** Every number here is a count or a median over 12 or 24
   paired starts. Nothing in this report is a statistical claim, and the swing counts are
   small integers: 33 to 21 is twelve events.
5. **R = 9 m was not tuned; it was one of two declared values.** It won by 2 swings over
   R = 6 on the 12-run sweep. On a different map or at a different lidar range the better
   radius may differ, and there is no evidence here that 9 m generalises. A radius is a
   crude stand-in for a room; the Bormann precedent segments the floor plan, and doing that
   properly would be the real version of this idea.
6. **The k sweep returned almost no information.** k = 2, 3 and infinity behave identically
   at R = 6 on this map, and k = 1.5 differs on one run out of twelve. That is a finding
   about the map (all binding score ratios are under 1.55), not evidence that k is a useless
   knob in general.
7. **Candidate P is untested.** Cause B is 2 swings out of 64 here. On a map where clusters
   fragment more, cluster stability could matter, and the fix for it belongs upstream in
   `detect_frontiers`, not in a wrapper.
8. **A-blocked, 8 swings, is out of reach of any re-ranking.** Those are near frontiers the
   drive planner refused because the 0.46 m body does not fit. Neither scorer models body
   width; the costmap they are handed is inflated 0.25 m while the rover's lethal radius is
   0.30 m. That is unchanged from the shipped bench and it is not the PR's fault.
9. **The vicinity metric is a wave over the whole grid.** Cheap here (95 ms on a 738 x 521
   grid at 5 cm). On a much larger map it would need bounding, and the honest version of
   that is explore_lite's: read the navigation stack's existing potential field instead of
   spreading a second one.
10. **The simulated pose is perfect** and **unknown is a wall by construction**, as in the
    mid-start bench. A reversal in simulation is a decision, never noise.
11. **The 45 s goal timeout in config `shipped`** cuts a walk at about 6.7 m at 0.15 m/s.
    That is why the round-trip counts are so much smaller there in both arms.

---

## 8. Files produced

Absolute path of the workspace:
`./`

| file | what |
|---|---|
| `diagnostic_swings.md` | step 1, the full diagnosis, with the 14 class-A decisions in a table |
| `diagnostic_counts.txt` | the console output of the classifier |
| `swing_decisions.json` | one record per swing: every candidate with size, score, route length, suppression flag, and the class |
| `diagnose_swings.py` | the classifier |
| `hypotheses_predeclared.txt` | the hypothesis and the winner rule, written before the sweeps |
| **`fix_hysteresis.py`** | **the harness-side policy wrappers** (H, B, and the unrun P), plus the one-wave geodesic field |
| `bench_2830.py` | the mid-start bench plus `--arms`, `--trace`, and the trace hook (off by default, no behaviour change) |
| `compare_fix.py` | pairwise comparison of any two arms, the pre-declared test, the CSVs |
| `plot_fix.py` | the four-arm goal-sequence figures |
| `results_trace_4m.json` | the 48 traced baseline runs, identical to `../sim_2830_midstart/results_midstart_4m.json` |
| `sweep_stock_4m.json`, `sweep_H_B.csv`, `sweep_H_B.txt` | the 60-run parameter sweep and its tables |
| `results_fix_4m.json`, `final_4m.csv`, `final_4m.txt` | the 48 winning-configuration runs and the four-arm tables |
| `results_fix_12m.json`, `regression_12m.csv`, `regression_12m.txt` | the 12 m regression check |
| `goal_sequences_fix_*.png` | four figures, four arms each, orange swing segments |
| `trace_4m.log`, `sweep_stock_4m.log`, `fix_4m.log`, `fix_12m.log` | the run logs |

**Compute spent**, against a budget of about 90 minutes: 180 runs in four benches, roughly
40 minutes of wall clock. 48 traced baseline runs (12 workers, about 13 min), the 60-run
sweep (16 workers, about 13 min), the 48-run winning configuration and the 24-run 12 m
regression run concurrently (12 and 8 workers, about 14 min). Nothing was re-run to chase a
number, and no parameter outside the declared sweep was tried.

Nothing was committed, pushed or posted. `../sim_2830_midstart/` and
`../sim_2830_bigoffice/` were read, not modified. `` and the
dimos venv were read only. The two upstream selector files are byte-identical to their
starting state, md5 `e77c328643c959a49077115e8a341f2c` and
`1ccc0c69fe88a72e402565feca988d26`.

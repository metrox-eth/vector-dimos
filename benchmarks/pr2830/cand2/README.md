# Two more candidates for the dimOS exploration ping-pong: a signed direction term, and a lazy tour

Offline, no robot, no flight. 2026-08-29. Workspace
`./`,
compute on the rented 192-core box, results synced back here. Nothing was
committed, pushed or posted. No git operation of any kind was run.

The two vendored upstream selector files were never touched. At the end of this
job `pr2830/selector_base.py` is md5 `e77c328643c959a49077115e8a341f2c` and
`pr2830/selector_head.py` is `1ccc0c69fe88a72e402565feca988d26`, the same values
as at the end of the fix job and the fleet job. Everything added here is a
harness-side wrapper: `fix_momentum.py` and `fix_tsp.py`, in the style of
`fix_hysteresis.py`, installed on one selector instance.

## Summary in seven lines

1. **The 15 s correction is the biggest finding of the job, and it is not about
   either candidate.** Config `shipped` re-run at the upstream 15 s goal timeout
   instead of our blueprint's 45 s, same maps, same starts, same two arms, 96
   paired runs: **real crossings 34 to 12, median total path -59.5 %, median
   coverage -18.4 points.** At 4 m lidar, the upstream-faithful shipped
   configuration barely lets the rover get anywhere, and the ping-pong the
   maintainer described is mostly not measurable in it.
2. **Candidate M won its ratio sweep at 4.3, GBPlanner's own value, and the
   control spoils the story**: `M0`, the same wrapper with the direction term
   switched off, reaches the same crossings and the same round trips on
   `bigoffice`. On that map the effect is the distance penalty, not the signed
   direction.
3. **H1 passes on 1 map of the 6 where the metric can discriminate.** Pooled,
   `stock+M4.3` takes real crossings **48 to 22** and real round trips **12 to
   3**. Every failure is on the pre-declared per-configuration path budget, and
   every one of those is in config `shipped`.
4. **In config `scoring`, where the ping-pong actually exists at 4 m, M is
   strong**: **44 to 21 real crossings, 12 to 3 round trips, median path -28.4 %,
   median coverage +0.3 %**, and the path and coverage budgets hold in that
   configuration on every single map.
5. **H2 fails on all 6.** `stock+T30` takes 48 to 35 pooled but makes `bigoffice`
   **worse**, 7 to 10, and overspends path in `shipped` by 27 %.
6. **H3 fails on all 6.** `pr2830+M4.3` takes 40 to 33 and 4 to 2, and misses the
   `shipped` path budget by half a point, +5.5 % against +5 %.
7. **H4 holds pooled and on 2 of the 4 declared large maps.** The clean case is
   `hk_park` in `scoring`: **-36.1 % of median path at +0.7 points of coverage**,
   which is the neighbourhood of P2 Explore's 31 % claim. The other passing map,
   `hk_elevator`, saves path partly by stopping early.

The short version: the signed direction term is the better of the two candidates
and it is not a general fix either; and the configuration we have been reporting
the ping-pong in was running with a goal timeout three times the upstream one.

---

## 1. What this had to answer, and where the design comes from

The design of this job is dictated by the confrontation memo,
`../sim_2830_fleet/confrontation_externe.md`. Four things in it are treated as
settled and are not re-tested here; each is quoted with its place in the memo.

**S1. PR #2830 already deletes `explored_goals_score`.** Memo section (i),
"COLLISION FRONTALE": the verified diff removes the `explored_goals_distance` /
`explored_goals_score` lines and the `+ 0.3 * explored_goals_score` term. So
"delete the repeller" is not a candidate of ours. It is somebody else's open PR,
and the memo's verdict on proposing it again is "a doublon a un mainteneur qui
vient de dire qu'il teste l'autre. A abandonner sous cette forme."

**S2. Deleting it does not kill the short-range swings.** Memo header point 5 and
section (v).4, on our own mid-start aggregates at 4 m lidar: cross-map swings go
**11 to 11** in config `shipped` and **22 to 20** in config `scoring`. Necessary,
not sufficient. That is why this job exists.

**S3. The best-evidenced missing piece is the direction term.** Memo header point
3 and section (v).4:

> dimOS a bien un terme de direction, mais il est inoffensif :
> `momentum_score = max(0.0, dot_product)` (l. 467) a **5 %** de poids. Etant
> tronque a zero, **un demi-tour coute exactement autant qu'un deplacement
> perpendiculaire, c'est-a-dire rien.**

and the established alternative, memo section (iii), form B of "les trois seules
formes que prend la memoire des anciens buts": GBPlanner weights direction 0.3
against 0.07 for path length, a factor of **4.3**; FUEL's `w_dir = 1.5` costs
about 4.7 s equivalent for a U-turn; Umari's `hysteresis_gain = 2.0`.
That is **candidate M**.

**S4. The money is in a global order, not in room segmentation.** Memo header
point 4 and section (iii): P2 Explore, Table II, single robot, metric in metres,
a global TSP ordering of frontiers buys about **31 %** of travel on the large
scenes and room-awareness on top of it only about **4.9 %** more. The memo's own
conclusion: "L'argent est dans 'cesser de decider glouton coup par coup', pas
dans la segmentation." That is **candidate T**.

And one correction from the memo that changes every baseline in this bench:

**S5. The goal timeout is 15 s, not 45 s.** Memo section (v).6, first bullet:

> Le "45 s goal timeout in the shipped config" annonce dans notre commentaire
> public du 2026-08-29 ne se retrouve nulle part dans le depot :
> `WavefrontConfig.goal_timeout` vaut **15.0** (l. 93) [...]. Soit le 45 s vient
> de notre propre config de harnais, soit c'est une erreur publique sur la PR
> d'un tiers.

Verified here independently, in the installed package
``,
md5 `e77c328643c959a49077115e8a341f2c`, which is byte-identical to
`pr2830/selector_base.py`:

    class WavefrontConfig(ModuleConfig):
        ...
        goal_timeout: float = 15.0

**This bench runs config `shipped` at 15.0 s, and every baseline is re-run at
15 s.** No number in this report is ever compared with a 45 s number as if the
two were the same condition. Section 8 measures what the correction changed.

---

## 2. The two candidates

Both are harness-side wrappers installed on one selector INSTANCE by
`fix_hysteresis.install()`, which calls the selector's own `detect_frontiers`
(ending in the selector's own `_rank_frontiers`) and hands the policy the list
that came back plus a tap of the selector's own scores. Neither wrapper invents a
candidate and neither computes a score of its own. Because both bench
configurations reach the goal through `detect_frontiers`, one installation covers
both arms and both configurations, and the selector's own bookkeeping
(`_update_exploration_direction`, `mark_explored_goal`) follows the policy's
choice exactly as it would have followed its own.

### Candidate M, the signed direction penalty (`fix_momentum.py`)

    adjusted = base_score * exp(-0.07 * route_m) * exp(-ratio * 0.07 * dev)

    dev   = (1 - cos(theta)) / 2, in [0, 1]:  0 straight on, 0.5 across,
            1.0 a full U-turn
    theta = angle between the robot's current direction of travel and the
            straight-line bearing to the candidate
    route_m = route length robot -> candidate under the PLANNER's own cost rule,
            one Dijkstra wave per decision (fix_hysteresis.geodesic_field,
            unchanged, so both candidates are measured the same way)
    ratio = the direction-to-distance weight ratio, swept in {1, 2, 4.3}

> Precedent: **GBPlanner** (Dang et al. IROS 2019; Kulkarni et al. ICRA 2022,
> arXiv 2111.06482 Sec. IV-A), read in the memo out of
> `gbplanner/src/rrg.cpp:657-676` and `config/smb/gbplanner_config.yaml`:
> `gain * exp(-0.07 * path_length) * exp(-0.3 * direction_deviation)`, i.e.
> `path_length_penalty = 0.07` against `path_direction_penalty = 0.3`, the 4.3x
> of the memo. The two exponentials, their shape and their coefficients are the
> borrowed elements.

`dev` is the signed reading the memo asks for: it is strictly decreasing in the
dot product over the whole range [-1, +1], where dimOS's `max(0, dot)` is flat
over the entire backwards half. At ratio 4.3 a U-turn costs `exp(-0.301) = 0.740`
of the score, which is what walking 4.3 m further costs. **A U-turn actually
costs.**

The heading is GBPlanner's own recipe, as the memo states it: an exponential
moving average of the **real displacement** between decision poses (position
direction, not body heading), `alpha = 0.3`, refreshed only once the robot is
more than 0.75 m from the pose of the last refresh. Before the first refresh
there is no heading and the direction factor is 1.0.

**What is not claimed.** This is not GBPlanner. GBPlanner penalises a gain
computed on an RRG and adds a hard `kBackward` path rejection on top; here the
two exponentials multiply somebody else's frontier score and there is no hard
rejection.

**The named control.** Spec `M0` is ratio = 0: the distance penalty alone, no
direction term. It is not a candidate and cannot win the sweep. It exists because
the GBPlanner form necessarily carries a distance term as well, so without it
nothing could say whether an effect of M belongs to the direction term at all.
It was run on `bigoffice` only, and section 3 reports what it found, which is not
flattering to candidate M.

### Candidate T, the lazy TSP ordering (`fix_tsp.py`, spec `T30`)

1. one Dijkstra wave from the robot **per replan** gives the route length to
   every current frontier cluster centroid;
2. a tour is built over those centroids: nearest-neighbour from the robot, then
   2-opt to convergence, capped at 20 sweeps;
3. the policy **commits**: at every decision the next un-consumed tour stop is
   moved to the head of the selector's own list. There is no per-decision
   re-optimisation;
4. the tour is re-planned only on one of four declared triggers: the next stop
   vanished without being visited (matched within 1.0 m); the frontier set
   churned by more than the declared fraction **0.30** (symmetric difference over
   union, same 1.0 m matching); the tour is exhausted; or a safety valve, the
   same stop having been put at the head twice in a row, which is counted and
   reported separately and is not part of the precedent.
   A stop that was issued and has since stopped being a frontier is **advanced
   past, not re-planned**: that is the tour being worked through, and it is the
   whole point of the candidate.

> Precedent: **P2 Explore** for the claim (arXiv 2409.10878v4, Table II, single
> robot, metres: about 31 % from a global TSP order on large scenes, about 4.9 %
> more from room-awareness), **FUEL** and **TARE** for the mechanism (FUEL solves
> an ATSP over frontier viewpoints with LKH every cycle,
> `fast_exploration_manager.cpp:345,383`; TARE orders uncovered subspaces with
> OR-Tools). All three via the memo, section (iii), form C.

**Declared approximation, and it is the weakest part of the candidate.** With one
wave per replan, only the robot-to-centroid edges are geodesic; the
centroid-to-centroid edges of the tour are **straight line**. FUEL pays for the
full pairwise A* matrix and then calls LKH. This is a cheap tour and it is not
called a TSP solver anywhere in this report.

**What is not claimed.** No room decomposition, no LKH, no viewpoint sampling, no
coverage path. The borrowed element is exactly one thing: a stable global order
over what is left, committed to across decisions instead of re-argmaxed at every
one.

---

## 3. The M ratio sweep, and the control that spoils it

Pre-declared in `hypotheses_cand2.txt` section 2: `bigoffice` only, 4 m, the 6
middle starts, both configurations, the stock arm, 12 runs per arm, against the
same 12 stock baseline runs. Winner rule, written before the sweep: the largest
drop in real crossings, ties broken by fewer real round trips, then by lower
median total path. No value outside {1, 2, 4.3} was tried.

| arm | real crossings | real round trips | median path, `shipped` | median path, `scoring` | median coverage, `shipped` | median coverage, `scoring` | goal-to-goal total |
|---|---|---|---|---|---|---|---|
| `stock` | 7 | 4 | 12.05 m | 59.97 m | 55.7 % | 90.6 % | 411.5 m |
| `stock+M1` | **4** | **0** | 11.98 m | 57.75 m | 57.4 % | 91.0 % | 379.0 m |
| `stock+M2` | **4** | **0** | 13.56 m | 57.75 m | 58.7 % | 90.6 % | 375.6 m |
| **`stock+M4.3`** | **4** | **0** | **11.64 m** | **54.50 m** | 56.8 % | 91.0 % | **360.2 m** |
| `stock+M0` (control) | **4** | **0** | 12.05 m | 57.75 m | 57.4 % | 91.0 % | 389.5 m |

**Winner: M4.3**, GBPlanner's own ratio. All three declared ratios tie on the
primary metric and on round trips, so the winner was decided by the third
tie-break, median total path, where 4.3 is the lowest under both readings
(pooled and per configuration).

Three things have to be said about that table, and two of them are bad news for
the candidate.

1. **The ratio sweep carries almost no information on this map.** Every declared
   ratio gives the same 4 real crossings and the same 0 round trips. Only the
   path separates them, and the separation is a couple of metres of median. This
   is the same shape of finding as the k sweep in `rapport_fix.md` caveat 6: a
   knob that does not move the primary metric on the map it was swept on.
2. **The control makes it worse. `M0`, the distance penalty alone with the
   direction term switched off, reaches the same 4 crossings and the same 0 round
   trips.** On `bigoffice`, at 4 m, the whole of candidate M's effect on the
   primary metric is attributable to the exponential distance penalty, and
   nothing at all to the signed direction term. The direction term only shows up
   in the path and the goal-to-goal total (360.2 m at ratio 4.3 against 389.5 m
   at ratio 0). Candidate M's own precedent is the direction term; on this map
   the direction term is not what is doing the work. That is reported here and
   carried into the verdicts rather than left out.
3. **The instrument does say the diagnosis was right.** The wrapper counts how
   often the arm's own top-ranked candidate lay more than 90 degrees off the
   robot's current direction of travel: **55 of 102 decisions**, i.e. more than
   half the time, stock's own first choice is a backwards choice. That is the
   `max(0, dot)` truncation of the memo, measured. What the sweep shows is that
   penalising it does not, by itself, remove crossings on this floor.

A note on the pooled medians in the automatic tables: pooling `shipped` and
`scoring` runs into one median mixes two very different populations at 15 s
(coverage about 56 % against about 91 %), so the pooled median coverage can move
several points while both per-configuration medians rise. That is why the
pre-declared budget clauses are tested **per configuration**, and why the pooled
figures are printed as "reported" rather than tested.

---

## 4. What ran, and what it cost

Pre-declared in `hypotheses_cand2.txt` section 2 and launched by
`launch_cand2.sh`, one invocation and one result file per map so that a
pathological map can only lose itself.

| | |
|---|---|
| maps | the 6 that passed the fleet gate, plus `bigoffice` and `bigoffice_hc` |
| starts | 6 per map by the `midstarts.py` rule: `centre` + `mid1..mid5` |
| lidar | 4 m |
| configs | `shipped` at the upstream **15 s** goal timeout, and `scoring`; `hk_allaround` and `hk_entrance` shipped-only from the start |
| arms | `stock`, `stock+M4.3`, `stock+T30`, `pr2830`, `pr2830+M4.3` |
| caps | 600 s per run (`--run-cap-s 600`), 1800 s per invocation, `--jobs` summing to 190 on the 192-core box |

**The known trap was avoided rather than rediscovered.** Config `scoring` is
unobtainable on `hk_allaround` and `hk_entrance`: it freezes deterministically at
the same task index in independent attempts, documented in
`../sim_2830_fleet/rapport_fleet.md` section 5, because `scoring` disables the
goal timeout by design and a single `sim.drive` on a 1172 x 960 grid can replan
for tens of minutes without returning to the decision loop where the wall cap is
checked. Those two maps therefore ran `shipped` only from the first launch and
contribute **6 paired starts each instead of 12**, labelled everywhere.

**What ran:** 420 runs, 8 result files, **zero failed invocations and zero
invocations killed**. 19 runs were wall-capped, all of them `hk_allaround`, all
in config `shipped`, listed individually in `cand_4m.txt`. Nothing was dropped.

**Determinism was checked, not assumed.** The `bigoffice` stock runs of the ratio
sweep and of the main grid are two independent invocations of the same 12 runs;
their goal counts, paths, coverages, goal-to-goal totals and swing counts are
identical line for line (only the wall-clock column differs). The 480 runs of
this job also reproduce every stored `cross_map_swings` field: see the
re-derivation note below.

**Compute:** 480 runs (420 grid + 60 sweep), **25.9 core-hours**, about **25
minutes of box wall clock** (sweep about 2 minutes on 60 workers, grid about 22
minutes on 190), against a budget of 2 hours. Nothing was re-run to chase a
number and no parameter outside the declared sweep was tried. The box is left
running.

**Re-derivation.** Every headline count in this report was recomputed from the
**raw goal coordinates** of each run, with that map's own swing threshold,
independently of the `summarise()` fields the bench wrote: **420 grid runs and 60
sweep runs checked, zero disagreements** with the stored `cross_map_swings` in
all 480. The 45 s files used in section 8 were checked the same way, 240 more
runs, also zero.

---

## 5. The four counts, per map and per arm

Real crossings (the primary metric, class-N filtered), with the raw counts
beside them. `hk_allaround` and `hk_entrance` are 6 paired starts, config
`shipped` only; the others are 12.

### Config `scoring` (their ranking, no self-stop, no goal timeout)

| map | stock | stock+M4.3 | stock+T30 | pr2830 | pr2830+M4.3 |
|---|---|---|---|---|---|
| `bigoffice` | 7 (raw 7) | **4** (4) | 7 (7) | 5 (5) | 5 (5) |
| `bigoffice_hc` | 8 (15) | **4** (9) | **4** (8) | 9 (15) | **3** (9) |
| `go2_short` | 9 (12) | **6** (10) | 7 (11) | 5 (9) | 6 (12) |
| `hk_elevator` | 4 (5) | **1** (1) | 6 (6) | 3 (3) | **2** (2) |
| `hk_office` | 3 (9) | **2** (2) | **2** (3) | 6 (8) | 6 (7) |
| `hk_park` | 13 (15) | **4** (4) | **6** (10) | 4 (6) | 5 (5) |
| **total** | **44** (63) | **21** (30) | **32** (45) | **32** (46) | **27** (40) |

### Config `shipped` (upstream loop, goal timeout 15.0 s)

| map | stock | stock+M4.3 | stock+T30 | pr2830 | pr2830+M4.3 |
|---|---|---|---|---|---|
| `bigoffice` | 0 (0) | 0 (0) | **3** (3) | 3 (3) | 2 (2) |
| `bigoffice_hc` | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| `go2_short` | 3 (10) | 1 (4) | **0** (2) | 2 (3) | 1 (3) |
| `hk_allaround` | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| `hk_elevator` | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| `hk_entrance` | 0 (0) | 0 (0) | 0 (0) | 0 (0) | 0 (0) |
| `hk_office` | 0 (0) | 0 (0) | 0 (0) | 2 (2) | 2 (2) |
| `hk_park` | 1 (2) | 0 (0) | 0 (0) | 1 (1) | 1 (1) |
| **total** | **4** (12) | **1** (4) | **3** (5) | **8** (9) | **6** (8) |

**Read the two tables together and the shape of the job is obvious.** In config
`scoring` the stock arm produces 44 real crossings over 36 paired starts. In
config `shipped` at the upstream 15 s it produces **4 over 48**. The metric that
this whole line of work is built on has almost nothing to count in the
upstream-faithful configuration at 4 m lidar, because a 15 s timeout at 0.15 m/s
cuts every walk at about 2.2 m and the run ends on the info-gain self-stop long
before the rover can cross anything. Section 8 measures that directly.

The full tables, with goal-to-goal totals, paths, coverages, capped counts and
both round-trip counts, are in `cand_4m.txt`; the per-run CSV is `cand_4m.csv`.

---

## 6. H1, H2, H3: the pre-declared verdicts, per map

The clauses, from `hypotheses_cand2.txt` section 4: real crossings must drop,
real round trips must drop, median path within +5 % **in each configuration
taken alone**, median coverage within -5 % **in each configuration taken alone**.
A map where the baseline arm has zero real crossings is reported as NO EVENTS and
set aside; that is `hk_allaround` and `hk_entrance` under every comparison, so
**6 maps can discriminate**.

### H1, `stock+M4.3` against `stock`

| map | real crossings | real round trips | path `scoring` | path `shipped` | coverage `scoring` | coverage `shipped` | verdict |
|---|---|---|---|---|---|---|---|
| `bigoffice` | **7 -> 4** | **4 -> 0** | -9.1 % | -3.3 % | +0.5 % | +2.1 % | **PASS** |
| `bigoffice_hc` | **8 -> 4** | **4 -> 2** | -9.6 % | -9.1 % | +1.1 % | **-6.3 %** | FAIL, coverage |
| `go2_short` | **12 -> 7** | 0 -> **1** | -0.0 % | -0.2 % | -2.0 % | -0.4 % | FAIL, round trips |
| `hk_elevator` | **4 -> 1** | 0 -> 0 | -17.0 % | **+8.7 %** | -0.4 % | +16.3 % | FAIL, path (and vacuous round trips) |
| `hk_office` | **3 -> 2** | 0 -> 0 | -30.7 % | **+55.0 %** | -2.8 % | +82.4 % | FAIL, path (and vacuous round trips) |
| `hk_park` | **14 -> 4** | **4 -> 0** | -34.9 % | **+10.4 %** | +0.6 % | +12.1 % | FAIL, path |
| `hk_allaround` | 0 -> 0 | 0 -> 0 | n/a | +21.1 % | n/a | +20.7 % | NO EVENTS |
| `hk_entrance` | 0 -> 0 | 0 -> 0 | n/a | +114.9 % | n/a | +91.9 % | NO EVENTS |
| **all pooled, 84 starts** | **48 -> 22** | **12 -> 3** | **-28.4 %** | **+12.3 %** | **+0.3 %** | **+12.7 %** | **FAIL**, on `shipped` path |

**H1 passes on one map of the six that can discriminate.** Five statements,
honestly ordered.

1. **The crossing drop is real and it is the largest this workspace has
   produced.** 48 to 22 pooled, and 14 to 4 on `hk_park` with its 4 round trips
   removed. For comparison, the fleet's B9 wrapper took 40 to 24 over its six
   maps and did nothing on `hk_office`.
2. **Every path failure is in config `shipped` and every one of them buys
   coverage.** `hk_office` +55.0 % of path against +82.4 % of coverage,
   `hk_entrance` +114.9 % against +91.9 %, `hk_park` +10.4 % against +12.1 %. The
   robot goes further before the upstream info-gain self-stop fires. The
   pre-declared budget has no way to express "more path, proportionally more
   map", so the verdicts stand as FAIL and this is the explanation, not a rescue.
3. **In `scoring` the same arm passes both budgets on every map**, and by a wide
   margin: -9 to -35 % of median path with coverage flat to +1 %. If the
   pre-declaration had been written on the configuration where the ping-pong is
   measurable, H1 would read very differently. It was not, so it does not.
4. **`bigoffice_hc` fails on coverage, not on path**, -6.3 % in `shipped`. That
   is a genuine loss and it is not explained away by anything.
5. **`go2_short` fails on a single round trip appearing where there were none.**
   One event over 12 paired starts.

### H2, `stock+T30` against `stock`

| map | real crossings | real round trips | path `scoring` | path `shipped` | coverage `scoring` | coverage `shipped` | verdict |
|---|---|---|---|---|---|---|---|
| `bigoffice` | 7 -> **10** | **4 -> 1** | +10.4 % | **+63.6 %** | +0.3 % | +23.0 % | **FAIL**, crossings and path |
| `bigoffice_hc` | **8 -> 4** | **4 -> 0** | -17.9 % | **+6.2 %** | +0.5 % | -3.0 % | FAIL, path |
| `go2_short` | **12 -> 7** | 0 -> 0 | **+11.4 %** | -6.5 % | -1.6 % | -1.2 % | FAIL, path |
| `hk_elevator` | 4 -> **6** | 0 -> **2** | -9.2 % | -16.1 % | -0.1 % | **-15.2 %** | **FAIL**, everything but path |
| `hk_office` | **3 -> 2** | 0 -> 0 | -30.8 % | **+15.9 %** | -1.9 % | +47.0 % | FAIL, path |
| `hk_park` | **14 -> 6** | **4 -> 0** | -36.1 % | -12.2 % | +0.7 % | **-5.8 %** | FAIL, coverage |
| `hk_allaround` | 0 -> 0 | 0 -> 0 | n/a | +14.7 % | n/a | -1.1 % | NO EVENTS |
| `hk_entrance` | 0 -> 0 | 0 -> 0 | n/a | +55.9 % | n/a | +35.7 % | NO EVENTS |
| **all pooled, 84 starts** | **48 -> 35** | **12 -> 3** | **-22.0 %** | **+27.2 %** | +0.3 % | +1.6 % | **FAIL**, on `shipped` path |

**H2 fails on all six.** Three things worth naming.

1. **On `bigoffice` the tour makes the primary metric worse**, 7 real crossings
   to 10, and spends 63.6 % more path in `shipped`. A committed order is a
   commitment to walk to the next stop even when it is across the room, which is
   precisely the behaviour the metric counts. That is the honest failure mode of
   candidate T and it is visible in `goal_sequences_cand2_bigoffice_shipped.png`.
2. **`hk_elevator` goes the wrong way on three counts at once**: crossings 4 to
   6, round trips 0 to 2, and 15.2 % less coverage in `shipped`. Its -16.1 % of
   path there is not efficiency, it is stopping earlier.
3. **The one clean win is `hk_park` in `scoring`**: 13 real crossings to 6, 4
   round trips to 0, **-36.1 % of median path at +0.7 points of coverage**. That
   is candidate T doing exactly what its precedent promises, on one floor.

### H3, `pr2830+M4.3` against `pr2830`

| map | real crossings | real round trips | path `scoring` | path `shipped` | coverage `scoring` | coverage `shipped` | verdict |
|---|---|---|---|---|---|---|---|
| `bigoffice` | **8 -> 7** | 0 -> 0 | **+11.8 %** | +0.2 % | +0.0 % | +0.0 % | FAIL, path |
| `bigoffice_hc` | **9 -> 3** | **3 -> 0** | -20.9 % | **+9.6 %** | -0.0 % | +3.1 % | FAIL, path |
| `go2_short` | 7 -> 7 | 0 -> 0 | -13.5 % | +0.0 % | +0.1 % | -0.4 % | FAIL, crossings |
| `hk_elevator` | **3 -> 2** | 0 -> 0 | -10.3 % | **+10.1 %** | +0.5 % | +5.0 % | FAIL, path |
| `hk_office` | 8 -> 8 | 1 -> **2** | **+10.5 %** | -28.1 % | +0.1 % | **-8.4 %** | **FAIL**, everything |
| `hk_park` | 5 -> **6** | 0 -> 0 | -11.7 % | **+38.1 %** | +0.3 % | +29.1 % | **FAIL**, crossings and path |
| `hk_allaround` | 0 -> 0 | 0 -> 0 | n/a | +26.5 % | n/a | +15.2 % | NO EVENTS |
| `hk_entrance` | 0 -> 0 | 0 -> 0 | n/a | +26.0 % | n/a | +28.3 % | NO EVENTS |
| **all pooled, 84 starts** | **40 -> 33** | **4 -> 2** | **-9.5 %** | **+5.5 %** | +0.0 % | +2.5 % | **FAIL**, on `shipped` path by 0.5 points |

**H3 fails on all six, and the pooled verdict turns on half a percentage point.**
The `shipped` median path rises 5.5 % against a 5 % budget. That is the same kind
of hair-splitting the fleet report flagged when its pooled H1 failed by 0.04
points, and it is reported rather than rounded.

The mechanism is the one the fleet already found for B9 and it is visible in the
counters: PR #2830 already prefers near frontiers, so on top of it the wrapper
has less to override. `stock+M4.3` moved the head of the ranking on **448 of
1035 decisions**; `pr2830+M4.3` on **108 of 1027**, a quarter as often.

### The combined arm was not run, and the gate says why

The pre-declared gate: run `stock+M4.3+T30` only if both M and T pass on at least
half of the maps where the metric can discriminate. M passes on **1 of 6**, T on
**0 of 6**. **The gate did not fire.** The chaining machinery exists in
`bench_2830.py` (`ChainedPolicy`) and was not exercised.

---

## 7. H4, candidate T's own precedent claim

P2 Explore reports about 31 % of travel saved by a global TSP order on large
scenes. LARGE was declared before the run as body-passable floor >= 100 m2.

| map | floor | starts | median path, `stock` | median path, `stock+T30` | change | H4 |
|---|---|---|---|---|---|---|
| `hk_allaround` | 544 m2 | 6 | 22.64 m | 25.96 m | **+14.7 %** | FAIL |
| `hk_entrance` | 229 m2 | 6 | 12.51 m | 19.51 m | **+55.9 %** | FAIL |
| `hk_park` | 157 m2 | 12 | 68.06 m | 62.17 m | **-8.7 %** | **PASS** |
| `hk_elevator` | 140 m2 | 12 | 58.22 m | 39.26 m | **-32.6 %** | **PASS** |
| **pooled, large** | | 36 | 27.56 m | 26.70 m | **-3.1 %** | **PASS** |
| the four small maps | | 48 | 16.14 m | 20.30 m | +25.8 % | n/a |

**H4 holds pooled and on 2 of the 4 large maps, and the two passes are not the
same kind of pass.**

- `hk_park` is the real one: in `scoring`, -36.1 % of path at +0.7 points of
  coverage. That is within reach of the 31 % P2 Explore reports, on one floor.
- `hk_elevator`'s -32.6 % is partly the tour stopping early: in `shipped` it
  saves 16.1 % of path and gives up **15.2 points of coverage**. Path saved by
  not exploring is not the claim P2 Explore makes.
- The two failures are the two floors where `shipped` is the only configuration
  available, so their numbers come from the configuration where the 15 s timeout
  dominates everything.
- On the small maps the tour costs 25.8 % more path. The precedent's claim is
  specifically about large scenes, and that is the direction the numbers go.

### What the wrappers counted about themselves

From `h4_counters.txt`, summed over 84 runs per arm.

| counter | `stock+M4.3` | `pr2830+M4.3` | `stock+T30` |
|---|---|---|---|
| decisions | 1035 | 1027 | 1057 |
| decisions with a heading (after 0.75 m) | 941 | 932 | - |
| **the arm's own top choice was more than 90 degrees off the direction of travel** | **526 (51 %)** | **320 (31 %)** | - |
| the policy moved a different candidate to the head | 448 (43 %) | 108 (11 %) | 796 (75 %) |
| tour replans | - | - | 783 |
| ... of which the churn trigger | - | - | 367 |
| ... of which the next stop had vanished | - | - | 260 |
| ... of which the safety valve | - | - | 72 |
| tour stops worked through without a replan | - | - | 615 |

**Two readings.** First, the memo's diagnosis is confirmed by instrument: on
**more than half** of stock's own decisions the top-ranked candidate lies in the
backwards half-plane, which under `max(0, dot)` costs exactly nothing. Second,
**candidate T is not as lazy as its name**: 783 replans over 1057 decisions. The
frontier set on these floors churns past 30 % most of the time, so the "commit to
the order" property is much weaker in practice than in the design. The 615
advances are the part that worked as intended.

### The regressions the medians hide

Per paired start, the number where the fix arm finished with at least 10 points
less coverage than the arm it is compared with, out of 84:

| comparison | regressions >= 10 points | gains >= 10 points |
|---|---|---|
| `stock+M4.3` vs `stock` | **5** | 13 |
| `stock+T30` vs `stock` | **12** | 11 |
| `pr2830+M4.3` vs `pr2830` | **4** | 7 |

The worst single case is `hk_park` / `mid3` / `scoring`, where stock drives
145.6 m to 90.6 % and `stock+M4.3` stops after 15.7 m at 21.1 %: the info-gain
self-stop fires early because the policy keeps the rover in a corner that is
already known. It is visible as the fourth row of
`goal_sequences_cand2_hk_park_scoring.png`. **Candidate T is close to a coin flip
on this measure**, 12 regressions against 11 gains, which is a stronger objection
to it than any of its verdicts. The full list is in `per_run_regressions.txt`.

---

## 8. What the 15 s correction changed, against the 45 s history

The comparison the pre-declaration promised, and it is descriptive: same maps,
same 6 middle starts, same 4 m lidar, same two upstream arms, same harness,
config `shipped` only, two values of `WavefrontConfig.goal_timeout`. The 45 s
runs are the fleet's and the fix bench's own files, not re-run. 96 paired runs
each side. No hypothesis is attached and no verdict is drawn.

| map | arm | real crossings | raw swings | median goals | median path | median coverage |
|---|---|---|---|---|---|---|
| `bigoffice` | stock | 3 -> **0** | 4 -> 0 | 8.5 -> 7.0 | 37.53 -> 12.05 m, **-67.9 %** | 88.0 -> 55.7 %, **-32.3 pt** |
| `bigoffice` | pr2830 | 4 -> 3 | 6 -> 3 | 11.0 -> 8.5 | 41.08 -> 11.94 m, -70.9 % | 89.5 -> 57.6 %, -31.9 pt |
| `bigoffice_hc` | stock | 4 -> **0** | 7 -> 0 | 9.5 -> 9.5 | 36.29 -> 15.52 m, -57.2 % | 83.2 -> 59.9 %, -23.4 pt |
| `bigoffice_hc` | pr2830 | 4 -> **0** | 5 -> 0 | 9.5 -> 7.5 | 36.44 -> 11.72 m, -67.8 % | 87.6 -> 48.8 %, -38.8 pt |
| `go2_short` | stock | 2 -> **3** | 4 -> **10** | 7.0 -> 8.0 | 16.60 -> 11.10 m, -33.1 % | 46.6 -> 38.1 %, -8.5 pt |
| `go2_short` | pr2830 | 3 -> 2 | 5 -> 3 | 7.0 -> 9.5 | 19.07 -> 14.57 m, -23.6 % | 46.7 -> 46.6 %, -0.1 pt |
| `hk_allaround` | stock | 0 -> 0 | 0 -> 0 | 13.0 -> 13.5 | 52.71 -> 22.64 m, -57.0 % | 27.5 -> 15.1 %, -12.4 pt |
| `hk_allaround` | pr2830 | 0 -> 0 | 0 -> 0 | 14.0 -> 12.0 | 55.98 -> 21.80 m, -61.1 % | 29.5 -> 15.0 %, -14.5 pt |
| `hk_elevator` | stock | 2 -> **0** | 2 -> 0 | 8.5 -> 11.0 | 37.93 -> 17.65 m, -53.5 % | 63.4 -> 49.3 %, -14.1 pt |
| `hk_elevator` | pr2830 | 2 -> **0** | 2 -> 0 | 8.5 -> 9.5 | 37.50 -> 15.33 m, -59.1 % | 63.8 -> 44.8 %, -19.0 pt |
| `hk_entrance` | stock | 2 -> **0** | 2 -> 0 | 10.0 -> 8.0 | 45.42 -> 12.51 m, -72.5 % | 44.4 -> 20.5 %, -23.9 pt |
| `hk_entrance` | pr2830 | 1 -> **0** | 1 -> 0 | 11.0 -> 10.5 | 44.89 -> 15.62 m, -65.2 % | 49.7 -> 23.4 %, -26.3 pt |
| `hk_office` | stock | 3 -> **0** | 3 -> 0 | 7.5 -> 8.0 | 32.17 -> 10.84 m, -66.3 % | 40.6 -> 19.9 %, -20.7 pt |
| `hk_office` | pr2830 | 0 -> **2** | 0 -> 2 | 10.0 -> 13.0 | 31.87 -> 20.69 m, -35.1 % | 47.6 -> 35.4 %, -12.2 pt |
| `hk_park` | stock | 2 -> 1 | 2 -> 2 | 10.5 -> 13.0 | 43.02 -> 22.66 m, -47.3 % | 52.2 -> 40.8 %, -11.4 pt |
| `hk_park` | pr2830 | 2 -> 1 | 2 -> 1 | 8.5 -> 9.0 | 34.17 -> 14.99 m, -56.2 % | 53.8 -> 29.7 %, -24.1 pt |
| **pooled, 96 runs each side** | | **34 -> 12** | **45 -> 21** | | **37.40 -> 15.16 m, -59.5 %** | **56.1 -> 37.7 %, -18.4 pt** |

**What changed, in four statements.**

1. **The shipped configuration became a much weaker instrument.** Median total
   path falls by 59.5 % and median coverage by 18.4 points on the same starts.
   At 0.15 m/s a 15 s timeout cuts a walk at about **2.2 m**, against about
   6.7 m at 45 s. The rover is pulled off almost every goal before it arrives,
   the info-gain self-stop then ends the run early, and there is very little run
   left in which to cross the map.
2. **Real crossings in `shipped` fall from 34 to 12 across 96 paired runs.** On
   five of the eight maps the stock arm's crossing count in `shipped` goes to
   exactly **zero**. Not because the ping-pong was fixed, but because the run
   ends before it can happen.
3. **The raw swing count moves the other way on the small map.** `go2_short`
   stock goes 4 raw swings to 10 while real crossings only go 2 to 3: more goals
   time out, so more swings are class N, the goal sequence jumping while the
   robot stands still. The class-N filter is doing exactly the job it was built
   for and this is the clearest example of it in the workspace.
4. **Every `shipped` number in `rapport_fix.md` and `rapport_fleet.md` was
   measured at 45 s.** They are not wrong, they are a different condition, and
   they should not be quoted beside this report's `shipped` numbers. The
   `scoring` numbers of the earlier reports are unaffected, since that
   configuration disables the goal timeout in both benches.

**And the consequence for the PR conversation, said plainly.** Our public
comment on PR #2830 described a "45 s goal timeout in the shipped config". That
value is ours, not upstream. The correction should be made by us rather than by
someone else, and this table is the material for it. It also means that a bench
that wants to show the ping-pong at short lidar range should either report the
`scoring` configuration or raise the timeout deliberately and say so, because at
the upstream 15 s the shipped loop mostly measures the timeout.

---

## 9. Figures

Same visual style as the fix and fleet benches, with five columns:
`stock`, `stock+M4.3`, `stock+T30`, `pr2830`, `pr2830+M4.3`. Black dot = start,
thin arrows = the jump the selector asked for (robot at issue time to goal),
numbers = goal order, **solid orange = a real crossing**, **pale dashed orange =
a class-N swing**, where the goal sequence jumped and the robot did not.

| figure | why this pair |
|---|---|
| `goal_sequences_cand2_hk_park_scoring.png` | the strongest result in the job: stock 13 real crossings and 4 round trips, `+M4.3` 4 and 0, `+T30` 6 and 0. Row `mid3` is also the worst regression: stock 30 goals and 91 % coverage, `+M4.3` 7 goals and 21 % |
| `goal_sequences_cand2_bigoffice_hc_scoring.png` | the only pair where all four fix arms improve at once, and where `pr2830+M4.3` is at its best, 9 crossings to 3 |
| `goal_sequences_cand2_hk_office_scoring.png` | the multi-hallway floor the maintainer described. Both wrappers take stock 3 to 2 and neither helps PR #2830, 6 to 6 |
| `goal_sequences_cand2_go2_short_scoring.png` | the aisle grid, the map with the most crossings per run |
| `goal_sequences_cand2_bigoffice_shipped.png` | candidate T's failure mode, drawn: stock has zero crossings at 15 s and `+T30` introduces three, by committing to a stop across the room |

---

## 10. Caveats, in order of how much they matter

1. **At the upstream 15 s timeout, config `shipped` barely measures the thing
   under test, and it is what decides every verdict.** The stock arm produces 4
   real crossings over 48 `shipped` paired starts against 44 over 36 `scoring`
   ones. All three hypotheses fail on the `shipped` per-configuration path
   budget, in a configuration whose median run is 15 m long and 38 % covered. The
   pre-declaration was written before that was known and it was not changed
   afterwards; but a budget clause that binds hardest where the metric is
   emptiest is a weak test, and both readings are printed so the difference is
   visible.
2. **The direction term is not what is doing the work, where it was measured.**
   The `M0` control, the same wrapper with the direction term switched off,
   reaches the same real crossings and the same round trips as every declared
   ratio on `bigoffice`. Candidate M's whole precedent is the direction term. The
   control ran on one map only, which is a limitation of this job rather than a
   defence of the candidate.
3. **The medians hide per-run collapses.** 5 paired starts under `stock+M4.3`,
   12 under `stock+T30` and 4 under `pr2830+M4.3` finish with 10 points or more
   less coverage than their baseline, the worst being 90.6 % to 21.1 % on
   `hk_park`/`mid3`/`scoring`. For candidate T that is close to a coin flip, 12
   regressions against 11 gains, and it is a stronger objection to T than any of
   its verdicts.
4. **One recording per floor, no repetition, no variance estimate.** Every number
   is a count or a median over 12 paired starts, and over 6 on `hk_allaround` and
   `hk_entrance`. The crossing counts are small integers. Nothing here is a
   statistical claim, and a difference of one or two events is not evidence of
   anything on its own. H3's pooled verdict turns on half a percentage point of
   median path.
5. **Both candidates cost more per decision than stock, and one map is
   wall-capped.** M spreads a Dijkstra wave per decision, T one per replan. On
   `hk_allaround`, whose 19 capped runs stop at 600 s of real time, a more
   expensive policy explores less inside the same clock. That is a confound on
   that map and only on that map, and it was already the fleet report's objection
   to B9 there.
6. **Candidate T is not a TSP solver, and it is not very lazy either.** With one
   wave per replan only the robot-to-centroid edges are geodesic; the
   centroid-to-centroid edges are straight line, so on a floor where two
   centroids are 3 m apart across a wall the tour is simply wrong. And it
   re-planned on 783 of 1057 decisions, because the frontier set churns past the
   declared 30 % most of the time. The commitment property the precedent relies
   on is much weaker in practice than in the design.
7. **T's safety valve is not part of the precedent.** A stop that has been put at
   the head twice in a row is abandoned, because otherwise a frontier the drive
   planner cannot reach locks the tour for the rest of the run. It fired 72
   times and is counted separately.
8. **The swing threshold scales with the map and breaks on the biggest one.**
   Half the bounding-box diagonal is 7.84 m on `go2_short` and 35.13 m on
   `hk_allaround`. A 4 m explorer never issues two consecutive goals 35 m apart,
   so the metric counts nothing there under any arm. `hk_allaround` and, at 15 s,
   `hk_entrance` are reported and set aside, never averaged in as zeroes.
9. **Config `scoring` is missing on the two biggest floors.** They contribute 6
   paired starts instead of 12, config `shipped` only, because `scoring` freezes
   deterministically on them (`../sim_2830_fleet/rapport_fleet.md` section 5).
   That was known and honoured from the first launch, and it means the two maps
   whose H4 verdict fails are also the two whose only configuration is the weak
   one.
10. **The class-N filter is straight-line here**, geodesic in
    `../sim_2830_fix/diagnose_swings.py`. The two agreed on 63 of the 64 traced
    `bigoffice` swings. On a floor with more walls between robot and goal they
    would part company more often.
11. **The body is not the dataset's body.** The Go2 that recorded these floors is
    about 31 cm wide and walks; the simulated rover is 46 cm, rolls and needs a
    60 cm aisle. The bench explores a skeleton of each floor.
12. **The simulated pose is perfect and unknown is a wall by construction.** A
    reversal in simulation is a decision, never noise.
13. **Neither scorer models body width.** The costmap they are handed is inflated
    0.25 m while the rover's lethal radius is 0.30 m, so both keep aiming at
    frontiers in pinches the body does not fit through. Unchanged from the
    shipped bench, and not the PR's fault.

---

## 11. Files produced

Workspace:
`./`

| file | what |
|---|---|
| `hypotheses_cand2.txt` | the pre-declared grid, candidate definitions, metric definitions, hypotheses and the winner rule, written before any comparative run |
| **`fix_momentum.py`** | **candidate M**, the signed direction penalty, GBPlanner form, with the `M0` control |
| **`fix_tsp.py`** | **candidate T**, the lazy tour with its four declared replan triggers |
| `fix_hysteresis.py` | unchanged from the fleet job; only its `geodesic_field` and `install` are reused |
| `bench_2830.py` | the bench, plus three changes: the policy resolver across three wrapper modules, `--shipped-timeout-s` (default 15.0, the upstream value), and the per-run policy counters in the summary |
| `dimos_selector.py`, `midstarts.py` | unchanged from the fleet job |
| `pr2830/selector_base.py`, `pr2830/selector_head.py` | the two vendored upstream files, byte-identical, never touched |
| `analyse_cand2.py` | the fleet's `analyse_fleet.py` plus the per-configuration budget check and the "no events to remove" state |
| `h4_and_counters.py` | H4 (total path on the large maps) and the policy counters |
| `compare_timeout.py` | the 15 s against 45 s comparison of section 8 |
| `plot_cand2.py`, `figs_cand2.sh` | the five-column goal-sequence figures |
| `launch_cand2.sh`, `finish_cand2.sh` | the per-map launcher and the sync-and-analyse driver |
| `sweep_M.json`, `sweep_M.csv`, `sweep_M.txt`, `sweep_M_ratio_table.txt`, `sweep_M_verdicts.json` | the M ratio sweep and its tables |
| `cand_4m_<map>.json` | the 4 m grid, one file per map |
| `cand_4m.csv`, `cand_4m.txt`, `cand_4m_verdicts.json` | the merged tables, the per-run CSV and the machine-readable verdicts |
| `h4_counters.txt` | H4 and the counters |
| `timeout_15_vs_45.txt` | the 15 s correction, measured |
| `goal_sequences_cand2_<map>_<config>.png` | the goal-sequence figures, five arms each |
| `cand_<map>.log`, `cand_sweep.log` | the run logs, one line per run |

---

## 12. Integrity statement

- **Nothing was committed, pushed or posted.** No `git` command of any kind was
  run in this job.
- **The two vendored upstream files are byte-identical to their starting state**,
  both on this machine and on the box, re-checked after the last run:
  `pr2830/selector_base.py` md5 **`e77c328643c959a49077115e8a341f2c`**,
  `pr2830/selector_head.py` md5 **`1ccc0c69fe88a72e402565feca988d26`**.
  `selector_base.py` is also still byte-identical to the file installed in
  ``,
  which is where the `goal_timeout = 15.0` of section 1 was read.
- **`hypotheses_cand2.txt` was written before any comparative run.** The only
  things that ran before it were mechanical smoke checks of the two new wrappers,
  named in the file itself.
- **Nothing outside the declared grid was run.** No fourth ratio, no second
  churn fraction, no re-run to chase a number. The combined arm was not run
  because its declared gate did not fire.
- **The earlier workspaces were read, not modified.** `../sim_2830_fix/` and
  `../sim_2830_fleet/` were used as inputs to section 8 only.
- **The box is left running**, with `/root/sim_2830_cand2/` and `/root/logs/`
  intact.

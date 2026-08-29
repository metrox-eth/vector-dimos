# The fidelity gate: does G1 survive a faithful loop?

Offline, no robot, no flight. 2026-08-30. Design, verification and the T_sel
measurement on this rig; the run waves on the rented box, results copied back as
they landed. Workspace
`./`.
Nothing was committed, pushed or posted. No git command of any kind was run.

The two vendored upstream selector files were never opened for writing and are
md5-identical to their state in the fix, fleet, cand2, go2 and residual jobs:
`pr2830/selector_base.py` `e77c328643c959a49077115e8a341f2c`,
`pr2830/selector_head.py` `1ccc0c69fe88a72e402565feca988d26`.

An external audit found the harness' temporal semantics unfaithful. This job
re-verifies the audit against the installed package, fixes the harness, and
re-judges **G1 of `../sim_2830_go2/rapport_go2.md`** with the corrected
instrument.

---

## VERDICT, first

**G1 SURVIVES THE FIDELITY GATE, at every swept T_sel, at both ranges. The
frozen harness was UNDER-counting the behaviour, not inventing it.**

| range | config `shipped`, arm `stock` | T_sel = 0 (frozen, the old bench) | T_sel = 5 (faithful) | T_sel = 15 (faithful) |
|---|---|---|---|---|
| 4 m | maps with a real crossing | **2 of 4** | **4 of 4** | **4 of 4** |
| 4 m | real crossings, 24 paired starts | 6 | **11** | **10** |
| 12 m | maps with a real crossing | **4 of 4** | **4 of 4** | **4 of 4** |
| 12 m | real crossings, 24 paired starts | 16 | **22** | **28** |

Declared bar: real crossings on at least 2 of the 4 discriminating maps.
**HOLDS in all six cells.** Not one faithful setting weakens it, and the two
faithful settings agree with each other, so the "not decidable by this bench"
outcome that `hypotheses_fidelity.txt` allowed for does not apply.

**Stage B therefore ran**, at T_sel = 15 s, the value the pre-declared rule
selects from the isolated measurement (median 10.58 s). **Its verdict: the
remedy still FAILS the bar - 2 PASS of 8 (map, range) cells - but only in
config `scoring`, which the fidelity fix does not touch, and on budget clauses.
In config `shipped` it takes real crossings 28 -> 11 and round trips 10 -> 0 at
12 m while spending less path.** Section 5.

Four things carry this, and each is a number.

1. **Class N is not what is holding G1 up.** Goal churn without displacement is
   0 to 2 events per cell across the whole of Stage A. At 12 m it goes 2 -> 0
   under the fix: the faithful loop produces *fewer* churn events and *more*
   real crossings.
2. **The fix moves the robot, a lot.** At 12 m with T_sel = 15, **47.5 % of all
   the path walked in Stage A was walked during a selection compute**, on goals
   the loop had already given up on. At 4 m it is 34.9 %.
3. **A third to a half of the "timed-out" goals are reached a few seconds
   later, while the selector is choosing the next one.** At 12 m, T_sel = 15:
   163 goals timed out, the robot was still driving in 163 selection windows,
   and it arrived at the abandoned goal in **59 of them (36 %)**. At 4 m it is
   69 of 139 (**50 %**). The frozen harness could not see a single one of these.
4. **The goal the robot is handed is stale, and measurably so.** Median
   |d(robot, goal) at publish - at decision| is 1.26 m at 12 m / T_sel = 15.

What this does NOT say: it does not say dimOS' explorer is worse than the go2
bench reported, and it does not re-open G2/G3/G4. It says the instrument that
produced G1 was conservative in the direction that mattered, and G1 stands.

---

## 1. What the audit found, and what the code says

All three findings were re-verified against the installed package before any
code was written. `loop_semantics.md` is the write-up with file and line
references, on dimos 0.0.14b1 at
``, whose
selector file is md5-identical to `pr2830/selector_base.py`, the PR's base.

**1. Temporality: CONFIRMED, and it is worse than "the robot is frozen".**
`_run_exploration_loop` publishes a goal (selector line 812), clears the arrival
event (819), waits `goal_timeout` (823), and on timeout logs a warning and loops
(828). Between that warning and the next publish there are two statements that
read state (794 odometry, 799 costmap) and one that computes for seconds (800),
and **not one of them cancels anything**. `GlobalPlanner._current_goal` is still
the old goal and the local planner's 10 Hz control thread is still following the
old path; the old goal is dropped only when the new one arrives at
`handle_goal_request` (global_planner.py:134-140). So the robot walks toward the
abandoned goal for the whole of the next selection, **and** the goal it is handed
next was chosen for the pose it had when that selection started.

The asymmetry matters and is modelled: on a goal that was *reached*, or that the
planner *refused*, the robot really is standing still while the next selection
runs, because both stop the local planner (global_planner.py:149-170, 268-284).
Only the timeout leaves it driving.

`goal_timeout` is `WavefrontConfig.goal_timeout: float = 15.0` (selector line 93)
and the Go2 "smart" blueprint overrides nothing
(`robot/unitree/go2/blueprints/smart/unitree_go2.py:46`). One nit for upstream
while we are here: the warning string at line 828 says "Goal timeout after 30
seconds" and the value is 15.

**2. Speed: CONFIRMED.** `LocalPlanner._speed: float = 0.55`
(local_planner.py:67), passed to the controller unchanged unless
`global_config.nerf_speed < 1.0`, which defaults to 1.0 and is not overridden.
This job runs 0.55 m/s. The go2 bench's 0.60 was derived from the odom of dimOS'
recordings, which are a human teleoperating; that number is right for what it
measured and wrong for this loop, which never teleoperates. 0.55 is also the
conservative choice for the premise the bench rests on: `15 s x 0.55 = 8.25 m` of
walk before the loop pulls the robot off a goal, against 9.00 m at 0.60.

The real controller is slower still in ways we do not model, and the direction is
worth stating: it scales linear speed down by the heading error
(`controllers.py:74`, floor 0.2 m/s at `controllers.py:43`), rotates in place
instead of translating past 90 degrees of heading error (70-71), and follows a
smoothed resampled path (global_planner.py:343). Our sim walks straight segments
at a flat 0.55 and charges a separate in-place pivot per segment. **Our robot
therefore covers a metre of path in less time than theirs would, so if anything
this bench gives the loop MORE walk inside its 15 s than the real one gets** -
which is conservative for the question being asked.

**3. Vocabulary: adopted.** Class C is written as "locally forced under R = 6 m"
and never as "legitimate". Class N is written as "goal churn without
displacement" and is **published as its own column in every table in this report
and in every table `analyse_fidelity.py` writes**. The word "dropped" is not used
about it.

---

## 2. T_sel: what it is, and where the number comes from

`T_sel` is the selection compute, in simulated seconds, per decision. At
`T_sel = 0` the robot is frozen through the selection and through the 2 s retry
wait: every earlier bench in this workspace, bit for bit, kept as an arm so the
change reads as a difference rather than being swapped in silently.

Isolated measurement (`t_sel_measurement.txt`), this rig, 6 workers on 24
threads, nothing else running, one timed call to `get_exploration_goal` per
decision:

| map | decisions | median | p10 | p90 | max |
|---|---|---|---|---|---|
| `hk_park` | 59 | **7.68 s** | 6.64 | 8.13 | 8.86 |
| `hk_entrance` | 57 | **9.27 s** | 8.57 | 9.90 | 10.53 |
| `hk_office` | 64 | **10.82 s** | 10.37 | 11.21 | 11.68 |
| `hk_elevator` | 88 | **11.79 s** | 11.10 | 12.39 | 13.19 |
| **pooled** | **268** | **10.58 s** | 7.58 | 12.02 | 13.19 |

The job brief anchored on "~8 s median". 8 s sits inside our per-map spread but
is not what we measure, and this report quotes what we measure. Neither figure
changed the sweep, which `hypotheses_fidelity.txt` fixed at {0, 5, 15} before the
measurement ran. Under the pre-declared rule (the swept faithful value closest to
the measured median, ties to the smaller) 10.58 s selects **T_sel = 15 s for
Stage B**; the brief's 8 s would have selected 5 s, and both faithful values are
in Stage A, so the reader can see the verdict at either one and is not asked to
take the choice on trust.

Cross-check already in hand from the recorded go2 bench's own `decide_ms_mean`,
measured on the 384-core box under 24 to 60 concurrent workers and therefore an
upper bound: 10.2 to 16.8 s per decision on these four floors, 34.6 s on
`hk_allaround`.

This is our silicon and our grids. **The robot's onboard compute is not measured
by anything in this workspace**, which is exactly why T_sel is swept and why the
verdict is reported at every swept value rather than at one.

---

## 3. Stage A, per map, per T_sel

240 runs, `stock` only, profile `go2ctrl` (0.55 m/s), the four discriminating HK
floors, 6 middle starts, both ranges. Full tables `stageA.txt`, per-run CSV
`stageA.csv`, machine-readable `stageA_verdicts.json`.

**Re-derivation: every crossing count below is recomputed from the raw published
goal coordinates by a second independent pass and compared against the count the
bench recorded while running. 240 runs checked, ZERO disagreements.**

**0 of 240 runs hit the 900 s wall cap. 0 published zero goals.**

### 4 m, config `shipped`, arm `stock`

| map | T_sel | REAL | N-churn | raw | round trips | med goals | med path | med cov |
|---|---|---|---|---|---|---|---|---|
| `hk_office` | 0 | **0** | 0 | 0 | 0 | 11.5 | 45.04 m | 52.9 % |
| | 5 | **3** | 0 | 3 | 1 | 11.5 | 49.58 | 52.0 |
| | 15 | **1** | 0 | 1 | 0 | 11.0 | 57.53 | 52.2 |
| `hk_park` | 0 | **0** | 0 | 0 | 0 | 11.0 | 38.45 | 47.7 |
| | 5 | **1** | 0 | 1 | 0 | 10.5 | 42.70 | 43.9 |
| | 15 | **3** | 0 | 3 | 0 | 11.5 | 52.85 | 49.3 |
| `hk_elevator` | 0 | **5** | 0 | 5 | 0 | 14.5 | 58.14 | 90.6 |
| | 5 | **4** | 0 | 4 | 0 | 14.5 | 71.31 | 96.7 |
| | 15 | **4** | 0 | 4 | 0 | 12.5 | 59.77 | 89.7 |
| `hk_entrance` | 0 | **1** | 0 | 1 | 0 | 10.0 | 37.72 | 35.9 |
| | 5 | **3** | 2 | 5 | 0 | 16.5 | 75.00 | 59.4 |
| | 15 | **2** | 1 | 3 | 0 | 11.0 | 65.85 | 49.0 |

**G1 at 4 m: 2 of 4 at T_sel 0, 4 of 4 at T_sel 5, 4 of 4 at T_sel 15. HOLDS at
all three.**

### 12 m, config `shipped`, arm `stock`

| map | T_sel | REAL | N-churn | raw | round trips | med goals | med path | med cov |
|---|---|---|---|---|---|---|---|---|
| `hk_office` | 0 | **1** | 1 | 2 | 0 | 10.0 | 37.63 m | 41.8 % |
| | 5 | **2** | 0 | 2 | 0 | 7.5 | 39.21 | 44.4 |
| | 15 | **3** | 0 | 3 | 0 | 8.5 | 51.96 | 58.8 |
| `hk_park` | 0 | **3** | 0 | 3 | 0 | 8.5 | 38.06 | 58.0 |
| | 5 | **5** | 0 | 5 | 0 | 7.5 | 47.20 | 63.3 |
| | 15 | **12** | 0 | 12 | **6** | 9.5 | 78.49 | 66.0 |
| `hk_elevator` | 0 | **9** | 1 | 10 | 3 | 8.0 | 36.23 | 95.7 |
| | 5 | **10** | 0 | 10 | 4 | 7.0 | 44.51 | 96.7 |
| | 15 | **10** | 0 | 10 | 4 | 7.5 | 62.55 | 97.1 |
| `hk_entrance` | 0 | **3** | 0 | 3 | 0 | 8.0 | 38.43 | 69.6 |
| | 5 | **5** | 0 | 5 | 1 | 9.5 | 59.84 | 80.8 |
| | 15 | **3** | 0 | 3 | 0 | 9.0 | 67.40 | 81.1 |

**G1 at 12 m: 4 of 4 at every T_sel. HOLDS.**

### Paired by start, same run with only the loop semantics changed

| range | change | starts | REAL | N-churn | per start up/flat/down | med goals | med path | med cov |
|---|---|---|---|---|---|---|---|---|
| 4 m | T_sel 0 -> 5 | 24 | 6 -> **11** | 0 -> 2 | 5 / 18 / 1 | 12.0 -> 13.5 | 43.9 -> 57.4 m | 52.3 -> 56.5 % |
| 4 m | T_sel 0 -> 15 | 24 | 6 -> **10** | 0 -> 1 | 6 / 16 / 2 | 12.0 -> 11.0 | 43.9 -> 57.5 | 52.3 -> 57.6 |
| 12 m | T_sel 0 -> 5 | 24 | 16 -> **22** | 2 -> 0 | 7 / 15 / 2 | 8.5 -> 8.0 | 36.6 -> 45.3 | 63.7 -> 67.5 |
| 12 m | T_sel 0 -> 15 | 24 | 16 -> **28** | 2 -> 0 | 10 / 11 / 3 | 8.5 -> 8.5 | 36.6 -> 63.8 | 63.7 -> 73.6 |

The direction is the same in all four rows and on 28 of the 96 paired starts
individually, against 8 in the other direction. **The fidelity fix makes the
ping-pong easier to see, not harder.**

---

## 4. What the fix actually does to the robot

Pooled over Stage A's `shipped` cells, arm `stock`, 24 runs per row:

| range | T_sel | goals reached in the 15 s pursuit | goals timed out | selection windows still driving | **arrived at the OLD goal during the compute** | metres walked inside selection windows | total path | share | median staleness |
|---|---|---|---|---|---|---|---|---|---|
| 4 m | 0 | 121 | 136 | 0 | 0 | 0.0 m | 1031 m | 0 % | 0.00 m |
| 4 m | 5 | 122 | 159 | 159 | **42 (26 %)** | 234 m | 1315 m | 17.8 % | 0.00 m |
| 4 m | 15 | 116 | 139 | 139 | **69 (50 %)** | 490 m | 1406 m | 34.9 % | 0.05 m |
| 12 m | 0 | 11 | 169 | 0 | 0 | 0.0 m | 864 m | 0 % | 0.00 m |
| 12 m | 5 | 11 | 163 | 164 | **23 (14 %)** | 273 m | 1075 m | 25.4 % | 0.86 m |
| 12 m | 15 | 24 | 163 | 163 | **59 (36 %)** | 698 m | 1471 m | 47.5 % | 1.26 m |

Three readings, and the middle one is the finding worth handing over.

**The frozen harness said the Go2 almost never reaches a goal at 12 m** - 11
arrivals against 169 timeouts. That reading was an artefact. Once the robot is
allowed to keep walking while the next selection computes, **59 of the 163
"timed-out" goals at T_sel = 15 are reached a few seconds later**. The loop's
own log line calls those goals timeouts; the robot got there anyway. Anyone
reading dimOS' `Goal timeout ... finding next frontier anyway` warnings as
evidence that the goals are unreachable is reading the wrong thing.

**Between a third and a half of all walking happens on a goal the selector has
already abandoned.** That is not a modelling artefact of ours; it is what
`SEL:823 -> 828 -> 787 -> 800 -> 812` does with a selection that costs seconds.

**The staleness is small in metres and it is not the mechanism.** 1.26 m median
at 12 m / T_sel = 15. The behaviour change comes from the extra walking, not from
the goal being chosen for a pose 1 m away.

### The pre-declared `scoring` check, and the one pair that failed it

`hypotheses_fidelity.txt` declared: in config `scoring` the goal timeout is not
applied, so no goal can end on a timeout, so no goal is ever live during a
selection, so T_sel is pure standstill and the T_sel 0 and T_sel 15 runs must be
identical in space.

**47 of 48 pairs are identical**, to the goal coordinate. The one that is not is
`12 m hk_entrance / mid5`, and the mechanism is exactly the one the declaration
named as the exception: our own failed-goal filter's 60 s clock. Goals 0 to 5 are
identical in both runs; at T_sel = 15 enough simulated time has passed for the
suppression of a refused goal to expire, so a 7th goal is issued 0.42 m from the
6th and is refused again. Path 87.19 -> 88.95 m, coverage 92.50 -> 92.70 %.
**That filter is a harness addition to config `scoring`, not dimOS**, and it is
the only thing T_sel touches in that configuration.

### And the fidelity bug this check caught in our own fix

The check failed on that pair for a *different* and worse reason in the first
version of the faithful arm, and it is worth recording because our own
pre-declaration caught it. `explore_sim.Sim.drive` returns `"timeout"` for two
different events: the clock ran out while walking (drive:793), and the path ran
out or a replan produced no motion (drive:801-805). Only the first is SEL:823
timing out; the second is, in dimOS, the local planner reaching the end of its
path and reporting `"arrived"` (LP:277-310), which stops the robot. The first
version treated both as "still pursuing". `bench_2830._still_pursuing` now
separates them by the clock. The 19 runs produced before the fix are parked at
`out_v1_conflated_timeout/` and are used for nothing; the whole Stage A grid was
re-run after it. Across the 240 Stage A runs the new
`goals_path_exhausted` counter fires **0 times, in either configuration**, so on
these four floors the two events never actually collided - but that is something
we now know rather than assumed, and the `scoring` identity check that found it
now passes on 47 of 48 pairs with the 48th explained.

---

## 5. Stage B: does the remedy still help, under the corrected instrument?

96 more runs, `stock+M4.3` against the Stage A `stock` arm, paired start by
start, at **T_sel = 15 s**, the value the pre-declared rule selected from the
measurement. Both configs, both ranges, the same four floors and six starts.
Per map, BOTH configs must hold: strictly fewer real crossings, no more round
trips, median path within +5 %, median coverage within -5 points.

**VERDICT: the remedy FAILS the bar again - 2 PASS of 8 (map, range) cells,
1 of 4 maps at each range. The corrected instrument does not rescue it. But it
fails better than it did frozen, and its best cell is now much stronger.**

| map | range | config | REAL crossings | N-churn | round trips | median path | median coverage | verdict |
|---|---|---|---|---|---|---|---|---|
| `hk_office` | 4 m | shipped | 1 -> 2 | 0 -> 0 | 0 -> 0 | 57.53 -> 37.15 m, -35.4 % | 52.2 -> 49.0 %, -3.2 pt | FAIL, crossings |
| | 4 m | scoring | 0 -> 1 | 0 -> 0 | 0 -> 0 | 102.89 -> 120.77, +17.4 % | 88.0 -> 88.5, +0.5 | FAIL, crossings + path |
| `hk_park` | 4 m | shipped | **3 -> 2** | 0 -> 0 | 0 -> 0 | 52.85 -> 44.21, -16.3 % | 49.3 -> 48.5, -0.7 | **PASS** |
| | 4 m | scoring | 4 -> 4 | 0 -> 0 | 0 -> 0 | 190.44 -> 110.89, -41.8 % | 76.7 -> 76.6, -0.2 | FAIL, crossings flat |
| `hk_elevator` | 4 m | shipped | **4 -> 1** | 0 -> 0 | 0 -> 0 | 59.77 -> 34.10, -42.9 % | 89.7 -> 74.7, **-15.0 pt** | FAIL, coverage |
| | 4 m | scoring | **5 -> 0** | 0 -> 0 | **1 -> 0** | 88.87 -> 81.50, -8.3 % | 97.6 -> 98.1, +0.5 | **PASS** |
| **`hk_entrance`** | 4 m | shipped | **2 -> 0** | 1 -> 0 | 0 -> 0 | 65.85 -> 67.74, +2.9 % | 49.0 -> 59.5, **+10.5 pt** | **PASS** |
| | 4 m | scoring | **3 -> 0** | 0 -> 1 | 0 -> 0 | 161.46 -> 136.58, -15.4 % | 91.1 -> 90.6, -0.5 | **PASS** |
| `hk_office` | 12 m | shipped | **3 -> 2** | 0 -> 0 | 0 -> 0 | 51.96 -> 54.20, +4.3 % | 58.8 -> 57.1, -1.7 | **PASS** |
| | 12 m | scoring | **4 -> 3** | 1 -> 1 | 1 -> 1 | 121.13 -> 160.91, **+32.8 %** | 88.2 -> 88.7, +0.5 | FAIL, path |
| `hk_park` | 12 m | shipped | **12 -> 2** | 0 -> 0 | **6 -> 0** | 78.49 -> 56.85, **-27.6 %** | 66.0 -> 66.9, **+0.9 pt** | **PASS** |
| | 12 m | scoring | 2 -> 3 | 1 -> 0 | 0 -> 1 | 172.12 -> 113.93, -33.8 % | 79.8 -> 79.2, -0.5 | FAIL, crossings + rt |
| **`hk_elevator`** | 12 m | shipped | **10 -> 4** | 0 -> 0 | **4 -> 0** | 62.55 -> 48.99, -21.7 % | 97.1 -> 97.0, -0.1 | **PASS** |
| | 12 m | scoring | **8 -> 3** | 0 -> 0 | **3 -> 0** | 73.00 -> 50.13, -31.3 % | 98.7 -> 97.1, -1.5 | **PASS** |
| `hk_entrance` | 12 m | shipped | 3 -> 3 | 0 -> 0 | 0 -> 0 | 67.40 -> 56.46, -16.2 % | 81.1 -> 71.1, **-10.0 pt** | FAIL, crossings + coverage |
| | 12 m | scoring | 2 -> 4 | 0 -> 0 | 0 -> 0 | 73.04 -> 77.84, +6.6 % | 92.5 -> 92.3, -0.3 | FAIL, crossings + path |

**Map verdicts: `hk_entrance` PASSES at 4 m, `hk_elevator` PASSES at 12 m,
6 of 8 FAIL.**

Three things to read off it rather than the verdict.

**The remedy is now clearly good in config `shipped`, which is the config the
fidelity fix touches.** Pooled over the four maps: at 4 m, real crossings
10 -> 5; at 12 m, **28 -> 11 and round trips 10 -> 0**. Every one of the eight
`shipped` cells either drops crossings or holds them flat, and none of the four
`shipped` round-trip counts rises. Six of the eight `shipped` cells also spend
LESS path.

**`hk_park` at 12 m, shipped, is the strongest single cell produced anywhere in
this workspace**: 12 real crossings to 2, 6 round trips to 0, path DOWN 27.6 %,
coverage UP 0.9 points. The frozen instrument at 0.60 m/s scored the same cell
6 -> 0 and 2 -> 0 at -7.6 % path; the faithful instrument shows the same shape
on four times the events. The map still fails, and it fails on `scoring`.

**Every remaining failure is either in `scoring` or is a budget clause.** Five of
the six failing cells are `scoring`, where T_sel changes nothing at all (section
4), so those five are the go2 bench's own G2 failures re-observed, not new
information. The one `shipped` failure that is not about `scoring` is
`hk_entrance` at 12 m, which loses 10 coverage points for no crossing reduction -
and that is the same map and the same shape the residual job already named as the
weakest result in the whole dossier (`../sim_2830_resid/rapport_resid.md`
section 5).

**Against the frozen instrument, the remedy is judged slightly better, not
worse.** The go2 bench read G2 as 0 of 4 maps at 4 m and 1 of 4 at 12 m. Here it
is 1 of 4 at each range. So the fidelity correction did not rescue the remedy and
did not sink it either: **the remedy's verdict is robust to the instrument, and
G1's is not** - G1 gets stronger.

---

## 6. Caveats, in order of how much they matter

1. **T_sel is a parameter we cannot measure on their hardware.** 10.58 s is our
   silicon on our grids. The verdict is reported at 5 and 15 and holds at both,
   which is the only reason it can be stated at all. If their onboard selection
   is much faster than 5 s, this bench does not cover it - though at
   T_sel -> 0 it degenerates to the frozen arm, where G1 still holds at 12 m and
   at 2 of 4 maps at 4 m.
2. **Small integers.** Every crossing count is a sum over 6 paired starts.
   `hk_office` at 4 m moves 0 -> 3 -> 1 across the sweep; that is three events
   and it is not a trend. The headline rests on the pooled paired rows (24
   starts) and on the direction being the same in all four of them.
3. **Not subtractable from the go2 bench.** Two things changed at once, the
   speed (0.60 -> 0.55) and the loop semantics. The T_sel = 0 arm here is the go2
   bench's semantics at the corrected speed and is the right baseline for the
   faithful arms; `go2_reference_060.txt` re-derives the recorded go2 bench with
   this job's metric for anyone who wants the other comparison.
4. **The 4 m sweep is not monotone.** 6 -> 11 -> 10 real crossings. The 12 m
   sweep is (16 -> 22 -> 28). At 4 m the robot crosses its visible neighbourhood
   inside one goal even when frozen, so extra walking changes which goals get
   issued rather than adding jumps; the go2 bench's section 7 finding that 12 m
   is the stronger instrument at Go2 speed survives here.
5. **Stage B ran at ONE faithful setting.** T_sel = 15 only, because that is
   what the pre-declared rule selected. Stage A shows G1 is stable across
   T_sel = 5 and 15, but nothing here says the remedy's verdict is. Its
   `scoring` half is T_sel-invariant by construction, so at most the `shipped`
   half could move.
6. **`hk_allaround` is absent.** Its 35.33 m swing threshold is not answerable by
   a 4 m explorer and its `scoring` configuration has never finished in any job
   in this workspace. It was declared out before the grid ran and it was not run.
   The verdict turns on four floors.
7. **`hk_entrance/centre` is a dead start** in every cell it appears in: 1 goal,
   0.6 m of path, 5 % coverage, every arm and every T_sel. It is kept in the
   paired set because dropping starts after seeing them is how a bench lies.
8. **Residual infidelities remain and are listed** in `loop_semantics.md`
   section 6: a refused goal costs 1 s here and up to 15 s of standstill there;
   the impact budget resets across a selection window; the selection window can
   overshoot by one cell-step; odometry is perfect; our body margins (0.35 m
   planner width, 0.766 m pivot circle) are stricter than dimOS' own defaults
   (0.3 / 0.6). The first of those is measurably immaterial here: the largest
   simulated run time in Stage A is 1444 s against a 6000 s budget, so the
   simulated clock binds nothing.
9. **Class N is nearly empty in this job** (0 to 2 events per cell), so the
   distinction between raw swings and real crossings does almost no work at
   these settings. That is worth saying because it means the headline does not
   depend on the N rule at all - but it also means this job does not test the N
   rule.
10. **The simulated pose is perfect and unknown is a wall by construction**, so a
   reversal in simulation is a decision, never noise.
11. **One recording per floor, no repetition, no variance estimate**, as in every
    bench in this workspace.

---

## 7. Files

See `README_files.md` for the full table. The ones that matter:
`loop_semantics.md` (the verified sequence with line references),
`hypotheses_fidelity.txt` (the pre-declaration),
`harness_diff_notes.md` (every change and the reproduction check),
`t_sel_measurement.txt` (the anchor), `stageA.txt` / `stageAB.txt` (the tables),
`out/fidA_*.json` and `out/fidB_*.json` (the per-run results).

---

## 8. Integrity statement

- **Nothing was committed, pushed or posted.** No `git` command of any kind was
  run in this job.
- **The two vendored upstream files are byte-identical to their starting state**
  on both machines: `selector_base.py` md5
  **`e77c328643c959a49077115e8a341f2c`**, `selector_head.py`
  **`1ccc0c69fe88a72e402565feca988d26`**.
- **`hypotheses_fidelity.txt` was written before any comparative run.** Three
  things ran before it and all three are named inside it: the isolated T_sel
  measurement, a 2-run pilot on `hk_elevator` that was killed after about seven
  minutes with its output never written and never read, and the reproduction
  check.
- **The reproduction check passed before Stage A, and again after the
  `_still_pursuing` fix**: the modified harness at profile `go2` and T_sel = 0
  reproduces the recorded `../sim_2830_go2/go2_12m_hk_park.json` on 6 runs, 13
  summary fields plus every goal coordinate, **0 disagreements**, with the two
  new pose fields coinciding and no selection window ever firing.
  (`repro_check.py`, `repro_check.txt`.)
- **Every headline count is re-derived from raw goal coordinates** by an
  independent second pass and compared against the field the bench wrote while
  running: **zero disagreements on every run of Stage A and Stage B.**
- **No run was dropped after being seen.** No run hit the wall cap, none
  published zero goals, and every start selected is in the tables including the
  dead one.
- **One grid was thrown away and re-run**, on purpose and on the evidence of our
  own pre-declared check: the 19 runs made before the `_still_pursuing` fix are
  parked at `out_v1_conflated_timeout/` and used for nothing.
- **Nothing outside the declared grid was run.** No second speed, no second
  radius, no re-run to chase a number. Stage B ran at the T_sel the pre-declared
  rule selected, not at the friendlier one.
- **The earlier workspaces were read, not modified.** `../sim_2830_go2/`,
  `../sim_2830_resid/` and `../sim_2830_cand2/` were inputs only.

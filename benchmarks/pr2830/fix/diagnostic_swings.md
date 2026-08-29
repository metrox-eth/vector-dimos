# What actually happens at each cross-map swing

Step 1 of the fix job: reconstruct the decision behind every swing in the mid-start
benchmark before proposing anything. 2026-08-29, offline, no robot.

Source: `results_trace_4m.json`, the same 48 runs as `../sim_2830_midstart/results_midstart_4m.json`
(4 m lidar, 6 middle starts, 2 maps, 2 configs, 2 arms), re-run with a per-decision trace.

**The traced run reproduces the recorded one exactly.** 48 runs, every one of
`n_goals`, `cross_map_swings`, `path_m`, `area_m2`, `goal_jump_total_m` identical to
1e-6; swing totals 11 / 11 / 22 / 20 as before. The trace is a wrapper on the score
method of one selector instance and it changes no behaviour.

## What the trace records

At every decision: the robot pose, the frontier candidates the arm's own detector
produced, the cluster size and the score the arm's own scorer gave each of them, which
of them the harness failed-goal filter suppressed, and the goal chosen. On a decision
whose goal-to-goal jump exceeds the swing threshold, additionally the route length from
the robot to every candidate, under the planner's own cost rule (blocked at 100, unknown
at 80, the rule the simulated rover's own drives obey), solved as one wave from the robot
rather than one A* per candidate.

The score is captured by replacing `_compute_comprehensive_frontier_score` on the
instance with a wrapper that calls the original and records what it returned. The two
upstream files are read-only inputs and are byte-identical at the end of this work
(md5 `e77c328643c959a49077115e8a341f2c` and `1ccc0c69fe88a72e402565feca988d26`).

## The classes, declared before the traces were read

`R_near` = **6.0 m of route length** from the robot. That is 1.5 lidar ranges at the range
under test, and it is the smaller of the two radii candidate B sweeps, so the diagnosis
and the candidate share one definition of "vicinity".

| class | meaning |
|---|---|
| **N** | not a crossing. The swing is defined on the goal sequence, and the robot is not always standing on the previous goal. |
| **A** | a near frontier existed, was available, and lost the ranking. |
| **A-blocked** | a near frontier existed but every one was suppressed by the failed-goal filter: the drive planner had already refused it. Re-ranking cannot reach these. |
| **B** | nothing near at the swing decision, but the region had a candidate earlier and one reappears in the same place later. The cluster blinked out. |
| **C** | nothing near, and nothing comes back in the same place. The area really was exhausted. |

## Counts

64 swings over the 48 runs.

| config | arm | swings | N | A | A-blocked | B | C |
|---|---|---|---|---|---|---|---|
| `shipped` | stock | 11 | 4 | **4** | 0 | 0 | 3 |
| `shipped` | #2830 | 11 | 3 | **2** | 0 | 1 | 5 |
| `scoring` | stock | 22 | 6 | **6** | 2 | 1 | 7 |
| `scoring` | #2830 | 20 | 6 | **2** | 6 | 0 | 6 |
| **total** | | **64** | **19** | **14** | **8** | **2** | **21** |

As shares of all 64: N 30 %, A 22 %, A-blocked 12 %, B 3 %, C 33 %.
As shares of the **45 real crossings** (N removed): **C 47 %, A 31 %, A-blocked 18 %, B 4 %**.

### First finding: 19 of the 64 swings are not crossings

The bench counts a swing on the goal sequence: goal k to goal k+1 more than 11.8 m apart.
The robot is not always on goal k. When a goal timed out (58 of 111 goals in `shipped`)
or was refused by the planner (43 of 106 in `scoring`), the robot is somewhere else, and
the next goal can be 12 to 19 m from the abandoned goal while being 2 to 4 m from the
robot. Six examples, from the file:

| map | start | config | arm | goal | goal-to-goal jump | robot to that goal |
|---|---|---|---|---|---|---|
| bigoffice | centre | shipped | #2830 | 8 | 11.9 m | 4.3 m |
| bigoffice | mid4 | shipped | stock | 7 | 13.1 m | 4.4 m |
| bigoffice | mid4 | shipped | #2830 | 6 | 13.4 m | 4.3 m |
| bigoffice_hc | centre | scoring | stock | 6 | 18.8 m | 2.8 m |
| bigoffice_hc | centre | scoring | #2830 | 7 | 18.8 m | 2.8 m |
| bigoffice_hc | mid1 | shipped | stock | 5 | 12.1 m | 4.3 m |

12 of the 19 are in `scoring`, 7 in `shipped`. Nothing in these decisions is a walk across
the map, and no re-ranking policy can or should touch them. They are set aside and the
remaining 45 are the ones classified.

### Second finding: cause A is real, and the margin needed is tiny

The 14 class-A swings, in full. `k needed` is score(chosen) / score(best near candidate),
on the arm's own score.

| map | start | config | arm | goal | jump | chosen at | best near at | score chosen | score near | k needed |
|---|---|---|---|---|---|---|---|---|---|---|
| bigoffice | mid4 | scoring | stock | 7 | 17.2 m | 19.3 m | 3.3 m | 0.2944 | 0.2832 | **1.04** |
| bigoffice | mid5 | shipped | stock | 9 | 13.3 m | 18.4 m | 3.2 m | 0.3206 | 0.2479 | 1.29 |
| bigoffice_hc | centre | shipped | stock | 7 | 12.4 m | 13.6 m | 5.9 m | 0.3259 | 0.3129 | **1.04** |
| bigoffice_hc | mid1 | shipped | stock | 11 | 19.1 m | 21.6 m | 2.5 m | 0.2278 | 0.1537 | 1.48 |
| bigoffice_hc | mid1 | scoring | stock | 3 | 17.0 m | 9.2 m | 1.7 m | 0.4330 | 0.4141 | **1.05** |
| bigoffice_hc | mid1 | scoring | stock | 4 | 17.9 m | 21.6 m | 1.7 m | 0.2823 | 0.1966 | 1.44 |
| bigoffice_hc | mid3 | shipped | #2830 | 13 | 12.6 m | 20.6 m | 4.7 m | 0.0180 | 0.0178 | **1.01** |
| bigoffice_hc | mid4 | shipped | #2830 | 6 | 14.4 m | 16.0 m | 3.2 m | 0.0587 | 0.0438 | 1.34 |
| bigoffice_hc | mid4 | scoring | #2830 | 6 | 14.4 m | 16.0 m | 3.2 m | 0.0587 | 0.0438 | 1.34 |
| bigoffice_hc | mid5 | shipped | stock | 7 | 13.8 m | 19.3 m | 2.2 m | 0.3188 | 0.2522 | 1.26 |
| bigoffice_hc | mid5 | scoring | stock | 5 | 12.6 m | 19.2 m | 5.6 m | 0.4095 | 0.3043 | 1.35 |
| bigoffice_hc | mid5 | scoring | stock | 10 | 17.8 m | 21.2 m | 2.6 m | 0.1982 | 0.1279 | 1.55 |
| bigoffice_hc | mid5 | scoring | stock | 11 | 16.7 m | 41.3 m | 2.6 m | 0.1422 | 0.1318 | **1.08** |
| bigoffice_hc | mid5 | scoring | #2830 | 5 | 12.6 m | 19.2 m | 2.2 m | 0.0363 | 0.0293 | 1.24 |

The near candidate that lost sits at a **median of 2.4 m of route length** from the robot
(range 1.7 to 5.9 m) while the goal taken is at a median of **19.3 m** (range 9.2 to 41.3 m).
The scores are nearly tied: median k needed **1.28**, maximum **1.55**, and four of the
fourteen are decided by less than 5 %.

    a margin of k = 1.5 would have kept the robot near on 13 / 14
    a margin of k = 2   would have kept the robot near on 14 / 14

That is the finding that decides which fix is worth testing. These are not decisions where
a far frontier is clearly better; they are near-ties resolved in favour of a walk that is
eight times longer.

Ten of the fourteen are stock and four are #2830, in both configs. The mechanism is
visible in the stock score: `explored_goals_score`, worth 30 % of the total, is
`min(distance to the nearest already-issued goal / 10 m, 1)`. Every goal the robot issues
becomes a repeller. A frontier 2.5 m from where the robot is standing has just been
devalued by the goal the robot is standing on, and a frontier 20 m away scores the full
1.0 on that term. The 20 % `distance_score`, `1 / (1 + |d - 5 m|)`, does not offset it,
because at a 4 m lidar range almost every candidate is within a few metres of the 5 m
lookahead and the term is nearly flat across the candidate set.

### Third finding: cause B is not the problem

**2 swings out of 64.** Frontier clusters vanishing between decisions and coming back is
not what drives this behaviour on these maps. Per the plan, the frontier-stability
stabiliser (holding a cluster for one extra decision cycle) was therefore **not run**:
there is nothing for it to fix here, and running it would have spent the compute budget on
a cause worth 3 %.

### Fourth finding: cause C is the largest single class among real crossings, and it is partly irreducible

21 of the 45 real crossings have nothing at all within 6 m of the robot. The nearest
available alternative at those decisions sits at a **median of 19.0 m**, minimum 6.5 m, and
6 of the 21 have no other candidate whatsoever. The robot has finished the end of a
corridor and everything that is left is at the other end of the floor. It has to walk.

**Said plainly: about half the real crossings on this map are not a scoring mistake.**
This floor is one large room plus one 20 m corridor; a robot that reaches the far end of
the corridor must come back down it. Any claim that a re-ranking policy removes the
ping-pong on this map would be false.

A larger vicinity does not rescue many of them: an available candidate exists within
6 m at 33 of the 64 swings, within 9 m at 37, within 12 m at 39. Going from R = 6 m to
R = 12 m buys six more decisions out of 64.

### Fifth finding: a third of the crossings were set up several goals earlier

A swing with nothing near is forced at the moment it happens. It is not necessarily
forced by the map. For each swing, the trace was searched backwards for the most recent
decision that took a goal further than 6 m away while an available candidate sat within
6 m (straight line, since route lengths are only stored on swing decisions; on this map
the two agree to a median factor of 1.10x).

| class | swings with such an earlier decision | median goals before the swing |
|---|---|---|
| N | 15 / 19 | 1.0 |
| A | 6 / 14 | 1.5 |
| A-blocked | 8 / 8 | 3.0 |
| B | 1 / 2 | 2.0 |
| **C** | **14 / 21** | **6.0** |
| all | 44 / 64 | |

Two thirds of the class-C crossings were preceded, a median of six goals earlier, by a
decision that walked away from a frontier within 6 m. The crossing itself was forced; the
situation was not. This is the reason a local-first policy is worth testing even though
C is the largest class: it acts at those earlier decisions, not only at the swing.

It is also a reason not to expect much. Six goals is a long causal chain in a system where
every goal changes the map, and there is no way to promise from the trace that keeping the
robot local at goal 3 changes what it faces at goal 9. That is what the sweep measures.

### A-blocked, 8 swings

A near frontier existed and was suppressed because the drive planner had already refused a
goal within 0.6 m of it inside the last 60 s. Six of the eight are #2830 in the `scoring`
config. These are the pinches the 0.46 m body does not fit through, which caveat 9 of the
mid-start report already named: neither scorer models body width, both aim at frontiers
the body cannot reach. A re-ranking policy cannot fix them, and a policy that promotes
them to the head of the list would only get them filtered again.

## What the diagnosis licenses

1. **The dominant fixable cause is A**, 14 swings, 31 % of the real crossings, every one of
   them recoverable with a switching margin of k <= 2 at a 6 m vicinity. Candidates H and
   B are aimed exactly at it and are worth the sweep.
2. **Cause B is 3 %.** The frontier-persistence stabiliser is not run.
3. **Cause C is 47 % of the real crossings and is partly irreducible on this map.** The
   honest ceiling for any re-ranking fix is well under half the swings, plus whatever
   knock-on it earns by acting on the earlier abandonment decisions.
4. **Class N, 30 % of the counted swings, is a limitation of the metric**, not of the
   scorer. It should be said in any report that quotes the swing count.

## Files

| file | what |
|---|---|
| `results_trace_4m.json` | the 48 traced runs, identical to the recorded bench |
| `swing_decisions.json` | one record per swing: every candidate with its size, score, route length, suppression flag, plus the class |
| `diagnostic_counts.txt` | the console output of `diagnose_swings.py` |
| `diagnose_swings.py` | the classifier |

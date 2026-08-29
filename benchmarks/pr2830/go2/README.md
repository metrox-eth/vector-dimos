# The Go2 profile: re-judging the dimOS exploration question on the maintainer's robot

Offline, no robot, no flight. 2026-08-29. Workspace
`./`,
compute on the rented 384-core box, results synced back here. Nothing was
committed, pushed or posted. No git operation of any kind was run.

The two vendored upstream selector files were never touched. At the end of this
job `pr2830/selector_base.py` is md5 `e77c328643c959a49077115e8a341f2c` and
`pr2830/selector_head.py` is `1ccc0c69fe88a72e402565feca988d26`, the same values
as at the end of the fix, fleet and cand2 jobs. Everything added here is
harness-side: a robot profile (`go2_profile.py`) and one policy wrapper
(`fix_composite.py`).

Maps: the five HK floors and nothing else, per the instruction that the
maintainer's own dataset is what the algorithm should be judged on. No
bigoffice, no go2_short, no rover-era anchor is in the comparative grid.

---

## Summary in eight lines

1. **G1 HOLDS, and it reverses the cand2 headline.** With the Go2 profile at the
   upstream 15 s goal timeout, stock produces real crossings on **3 of the 4
   discriminating maps** at 4 m and on **4 of 4** at 12 m. At our rover's
   0.15 m/s the same configuration was almost empty (cand2 section 8: 34 real
   crossings to 12, five maps at exactly zero). The ping-pong is measurable in
   the shipped loop once the robot walks at the speed the recordings were made
   at.
2. **The composite's third piece is a no-op on this body, and that is the second
   finding.** The reachability filter dropped **10 candidates out of 9 944** at
   4 m and **6 out of 9 357** at 12 m. `stock+CMP` produced an identical run to
   `stock+M4.3` on **53 of 54** paired starts.
3. **The reason is arithmetic, and it is a real result about the PR.** dimOS'
   loop inflates the costmap it hands the scorer by 0.25 m; the Go2's lethal
   radius under the same margin rules is 0.225 m. The body-feasibility gap that
   cand2 caveat 13 identified for OUR 0.46 m rover **does not exist for their
   0.31 m robot**. Upstream's inflation is conservative for a Go2.
4. **G2 FAILS** (0 of 4 maps at 4 m, 1 of 4 at 12 m). **G3 FAILS** (0 of 4).
   **G4 FAILS**, and by construction: with the filter inert there is nothing for
   the composite to add over M alone.
5. **But M/composite is strong where the behaviour is strong.** At 12 m,
   `hk_elevator` goes **12 real crossings to 4 and 4 round trips to 0 at +3.9 %
   path and +0.5 points of coverage**, passing in both configurations, and
   `hk_park` shipped goes **6 to 0 and 2 to 0 at -7.6 % path and +10.3 points**.
6. **At Go2 speed the low lidar range is the WEAKER instrument, inverting the
   maintainer's own 3-5 m advice.** Pooled over the four discriminating maps,
   stock/shipped: **8 real crossings and 1 round trip at 4 m against 23 and 6 at
   12 m.** The 3-5 m suggestion was calibrated on a 0.15 m/s rover; at 0.60 m/s
   the truncation that made short range informative is gone.
7. **The gate moved.** The Go2 body opens **+11 % to +99 %** more passable floor
   on the eight admitted maps, and `china_office` flips **REJECT to PASS**
   (+174 % of floor). It is not an hk_* floor so it is reported, not run.
8. **Plain failures.** `pr2830` and `pr2830+CMP` published **zero goals on 14 of
   270** runs at 4 m. `hk_allaround` scoring never finished and was killed at
   twice its declared cap. `hk_entrance` scoring, which froze in the fleet
   bench, did **not** freeze here.

---

## 1. The Go2 profile, and where every number comes from

Full derivation: `speed_derivation.txt`. Everything below is measured on dimOS'
own recordings, the same SQLite stores the maps were extracted from, or read off
Unitree's published spec. Nothing is assumed.

| constant | rover | Go2 | source |
|---|---|---|---|
| `SPEED_MPS` | 0.15 | **0.60** | derived: median of 18 733 moving 0.5 s windows over the five HK odom traces (p90 1.04) |
| `TURN_RATE` | 0.50 | **0.63** | derived: median of 1 492 pivot windows (turning while not translating), p90 1.30 |
| `SCAN_EVERY_M` | 0.25 | **0.25** | measured at 0.079 m/revolution and deliberately NOT changed, see below |
| `ROBOT_WIDTH_M` | 0.50 | **0.35** | 0.31 m body + the harness' own 4 cm planner margin |
| `BODY_HALF_WIDTH_M` | 0.23 | **0.155** | body / 2 |
| `CONTROL_MARGIN_M` | 0.05 | 0.05 | unchanged, a controller margin not a body dimension |
| `LETHAL_CLEARANCE_M` | 0.30 | **0.225** | the same formula: width/2 + margin |
| `PIVOT_CLEARANCE_M` | 0.39 | **0.383** | the same rule: half the body diagonal |

**Body.** Unitree Go2 published standing size **700 x 310 x 400 mm**, identical
across Air / Pro / Edu, 15 kg
([docs.quadruped.de, Model Variants | Unitree GO2](https://www.docs.quadruped.de/projects/go2/html/Overview_1.html);
[shop.unitree.com](https://shop.unitree.com/products/unitree-go2)). The
datasheet top speeds are 2.5 to 3.7 m/s, four to six times what the surveys were
actually walked at, which is exactly why the recordings and not the datasheet
fix the simulated speed.

**Speed, per recording** (moving windows, standstill excluded at 0.05 m/s):

| recording | source | poses | recorded | path | moving | median | p90 |
|---|---|---|---|---|---|---|---|
| `hk_office` | SQL | 10 437 | 554.7 s | 190.5 m | 83.7 % | 0.428 | 0.597 |
| `hk_allaround` | blob | 4 352 | 233.8 s | 194.3 m | 94.0 % | 0.945 | 1.080 |
| `hk_elevator` | blob | 1 986 | 105.7 s | 78.1 m | 89.3 % | 0.884 | 1.052 |
| `hk_entrance` | blob | 2 331 | 124.7 s | 108.3 m | 95.8 % | 0.944 | 1.064 |
| `hk_park` | blob | 2 121 | 113.2 s | 97.2 m | 93.2 % | 0.941 | 1.121 |
| `bigoffice` | SQL | 5 465 | 292.5 s | 162.0 m | 94.3 % | 0.542 | 0.924 |
| `go2_short` | SQL | 1 122 | 59.9 s | 37.5 m | 93.0 % | 0.626 | 0.957 |
| `china_office` | blob | 2 602 | 138.8 s | 37.3 m | 71.6 % | 0.376 | 0.567 |
| `markers_go2` | blob | 1 936 | 103.5 s | 13.7 m | 38.5 % | 0.234 | 0.584 |
| **pooled, HK five** | | | | | | **0.6029** | **1.0387** |

Five of the nine recordings leave the store's `pose_x`/`pose_y` index columns
zeroed or null; for those the PoseStamped payload is decoded by dimOS' own
`SqliteStore` reader and the `frame_id` is checked to be `world`. On
`go2_bigoffice`, where both paths exist, they are the same trace.

**The choice, and its sensitivity.** Two poolings were available: the
window-weighted pooled median (0.603) and the median of the five per-map medians
(0.941). The pooled figure is used, and it is the **smaller** of the two, so it
is the reading least favourable to the premise this whole bench rests on. The
spread is real: `hk_office`, a narrow desk-aisle office, was walked at 0.43 m/s
while `hk_allaround` was walked at 0.94. One speed is used across the grid so
the timeout truncation distance is the same on every map.

**What the speed buys**: 15.0 s x 0.60 m/s = **9.00 m of walk** before the loop
pulls the robot off a goal, against 2.25 m for our rover. That is the whole
reason the shipped configuration becomes an instrument again.

**The scan-rate check, which the speed change had to survive.** `SPEED_MPS` and
`SCAN_EVERY_M` are coupled: raising the speed while leaving a per-DISTANCE scan
rate alone would be an optimistic discovery bias if the robot could not deliver
a revolution inside every 0.25 m of walk. Measured from the recordings' own
lidar frame timestamps against their own odom, moving pairs only: the Go2
travels a **median 0.0789 m** (p90 0.1457) between consecutive lidar frames, its
stream running at **7.7 Hz**. So 0.25 m of walk contains about **3.2** real
revolutions at 0.60 m/s and about 12.7 at 0.15 m/s. The simulated
one-per-0.25 m is the coarser of the two at both speeds; information per metre
is geometry-limited, not sensor-limited, at either speed. Keeping the step in
distance therefore isolates the robot profile from the sensor model and keeps
coverage numbers comparable with every earlier bench. **The residual, stated
plainly: the sim under-samples the real Go2 lidar by about 3x per metre, which
is conservative about discovery, not optimistic.**

**What was NOT touched**: every dimOS constant. The loop's 0.25 m costmap
inflation, the 15 s goal timeout, the info-gain self-stop, the failed-goal
radius are theirs and stay as shipped.

---

## 2. The gate at 4 m, both bodies, the same two rules

`gate_go2.txt`, full table; figures `figs_go2/gate_<map>.png`.

**The rover pass is a reproduction check on the patch, and it passes.** Run with
`--profile rover`, `gate_fleet.py` reproduces `../sim_2830_fleet/rapport_fleet.md`
section 3 line for line: passable floors 69.4 / 156.7 / 140.0 / 229.2 / 544.0 /
34.1 / 48.0 / 48.1 / 9.6 / 9.3 / 14.3 m2, the same swing thresholds, the same
ways-out vectors, the same classes, the same PASS/REJECT column. The profile
patch is inert on the old profile.

| map | floor rover | floor go2 | growth | swing thr rover -> go2 | gate rover | gate go2 |
|---|---|---|---|---|---|---|
| `hk_office` | 69.4 m2 | 96.6 m2 | +39.2 % | 14.81 -> 15.45 m | PASS | PASS |
| `hk_park` | 156.7 | 218.0 | +39.1 % | 21.53 -> 21.82 | PASS | PASS |
| `hk_elevator` | 140.0 | 155.6 | +11.1 % | 15.03 -> 15.14 | PASS | PASS |
| `hk_entrance` | 229.2 | 279.9 | +22.1 % | 20.70 -> 21.44 | PASS | PASS |
| `hk_allaround` | 544.0 | 619.6 | +13.9 % | 35.13 -> 35.33 | PASS | PASS |
| `go2_short` | 34.1 | 67.8 | +98.8 % | 7.84 -> 10.98 | PASS | PASS |
| `bigoffice` | 48.0 | 60.4 | +25.8 % | 11.80 -> 12.84 | PASS | PASS |
| `bigoffice_hc` | 48.1 | 60.4 | +25.6 % | 11.83 -> 13.18 | PASS | PASS |
| **`china_office`** | 9.6 | 26.3 | **+174.0 %** | 3.91 -> 7.27 | **REJECT** | **PASS** |
| `apartment` | 9.3 | 13.6 | +46.2 % | 4.54 -> 5.53 | REJECT | REJECT |
| `markers_go2` | 14.3 | 17.4 | +21.7 % | 4.18 -> 4.53 | REJECT | REJECT |

Two things worth saying out loud.

**`china_office` was rejected for the wrong robot.** Under our body only 1.6 % of
what that floor can show sits beyond one lidar revolution; under the Go2 body it
is 26.5 %, because the doorways our 46 cm rover could not fit through are open to
a 31 cm one. A floor we called degenerate is a normal small office for them. It
is not an hk_* recording so it stays out of this grid, but the fleet bench's
rejection of it should not be quoted as a property of the floor.

**`bigoffice_hc` changes class**, multi-branch to room+corridor, which is one
more reason not to lean on that classifier: it already flipped on the choice of
occupancy kernel in the fleet report, and now it flips on the body too. The raw
columns are the content; the class is a convenience.

---

## 3. What ran, and what it cost

Pre-declared in `hypotheses_go2.txt` before any comparative run. Five arms:
`stock`, `pr2830`, `stock+CMP`, `pr2830+CMP`, `stock+M4.3`. Six middle starts
per map, recomputed for the Go2 body. Per-map chunking, `--jobs` 24 to 60 per
invocation, never above 190 total.

| wave | maps | configs | arms | cap | runs | outcome |
|---|---|---|---|---|---|---|
| 1 | hk_office, hk_park, hk_elevator | shipped + scoring | 5 | 900 s | 180 | complete |
| 2 | hk_entrance, hk_allaround | shipped | 5 | 900 s | 60 | complete, 12 capped on hk_allaround |
| 3 | hk_entrance, hk_allaround | scoring | 5 | 600 s | 60 attempted | hk_entrance complete (30); **hk_allaround killed at 2x the cap** |
| 4 | hk_office, hk_park, hk_elevator, hk_entrance | shipped + scoring | stock, stock+CMP | 900 s | 96 | complete, 12 m regression |
| 4b | hk_allaround | shipped + scoring | stock, stock+CMP | 900 s | 24 attempted | **killed, 0 finished** |

**270 runs at 4 m and 96 at 12 m are in the tables. 39 attempted runs are not**,
and they are all on `hk_allaround`.

**The re-derivation check.** Every crossing count in this report is recomputed
from the raw published goal coordinates in the results JSON by a second,
independent pass, and compared against the count the bench recorded while
running. **270 runs checked at 4 m and 96 at 12 m: zero disagreements.**

**The wall cap has a hole and it bit exactly once.** The cap is checked at the
top of the decision loop, so a run stuck INSIDE a decision never reaches it
(already `../sim_2830_fleet/rapport_fleet.md` section 4.1). On `hk_allaround`,
whose grid is 1 172 x 960 and whose median decision costs 15 s in the frontier
detector's full-grid Python BFS, the `scoring` configuration - no self-stop, no
goal timeout - ran past its declared 600 s cap. Per the execution rule it was
killed at 1 214 s, twice the cap. Fifteen of its thirty runs had printed a
summary line but the results file is only written at the end of an invocation,
so those fifteen carry no re-derivable goal coordinates and are **not** used.
`hk_allaround` is therefore reported `shipped`-only, which is exactly what the
pre-declaration said would happen if the one attempt failed.

**The freeze did not reproduce on `hk_entrance`.** The fleet bench recorded
`scoring` freezing deterministically on both big floors. Under the Go2 profile
`hk_entrance` finished all 30 `scoring` runs. Only `hk_allaround` is still
unreachable in that configuration, and on the evidence here it is slowness at
15 s per decision rather than a freeze.

**Decision cost.** Median mean-decision time is **15.3 s** across the grid and it
is dominated by the upstream frontier detector, not by any wrapper: `stock`
15 351 ms, `stock+M4.3` 14 190 ms, `stock+CMP` 14 402 ms, `pr2830` 15 313 ms,
`pr2830+CMP` 15 821 ms. The composite's second Dijkstra wave costs about 1.5 %
and is invisible next to the detector. That is worth knowing on its own: on
these floors the thing that makes exploration slow in dimOS is the frontier
scan, not the ranking.

---

## 4. G1: the behaviour IS there, in their conditions

**G1, verbatim**: *with the Go2 profile at 15 s shipped, the cross-map swing
behavior the maintainer described is present under stock (real crossings > 0 on
at least half the discriminating maps).*

### 4 m lidar, config `shipped`, arm `stock`, 6 starts per map

| map | REAL crossings | raw swings | round trips | median goals | median path | median coverage | fires? |
|---|---|---|---|---|---|---|---|
| `hk_office` | **0** | 0 | 0 | 9.5 | 34.70 m | 39.0 % | no |
| `hk_park` | **1** | 1 | 0 | 12.5 | 44.34 m | 48.9 % | YES |
| `hk_elevator` | **4** | 4 | 0 | 13.5 | 58.42 m | 89.0 % | YES |
| `hk_entrance` | **3** | 3 | 1 | 14.0 | 52.99 m | 53.3 % | YES |
| `hk_allaround` | 1 | 1 | 0 | 18.0 | 70.20 m | 30.7 % | set aside (35.33 m threshold) |

**3 of 4. G1 HOLDS.** At 12 m it is 4 of 4, with 2 / 6 / 12 / 3 real crossings.

### Why this matters more than the verdict

The cand2 bench, at our rover's 0.15 m/s, found that switching config `shipped`
from our blueprint's 45 s to the upstream 15 s took real crossings from 34 to 12
over 96 paired runs and left **five of eight maps at exactly zero**, with median
coverage falling 18.4 points. Its conclusion was that "at the upstream 15 s the
shipped loop mostly measures the timeout".

That conclusion was about the rover, not about dimOS. Restore the robot the
timeout was written for and the same 15 s buys **9.0 m of walk instead of
2.25 m**, and the crossings come back: 3 of 4 discriminating maps at 4 m, 4 of 4
at 12 m. **The instrument was broken by our body, not by their constant.**

The two benches are not directly subtractable - different map set, and coverage
is measured against a Go2-passable ceiling here and a rover-passable one there -
so the comparison that carries is the qualitative one: five of eight maps at
exactly zero crossings then, one of four now. If anything goes back to the
maintainer from this job, it is this line and the correction to our public
"45 s" claim that cand2 already found.

---

## 5. G2 and G3: the composite does not hold, per map and per config

**G2, verbatim**: *stock+COMPOSITE reduces real crossings and round trips vs
stock in each config, path within +5 %, coverage within -5 %, per map.*
Coverage is judged in percentage POINTS.

### 4 m, `stock+CMP` against `stock`

| map | config | crossings | round trips | median path | median coverage | verdict |
|---|---|---|---|---|---|---|
| `hk_office` | shipped | 0 -> 2 | 0 -> 0 | 34.70 -> 32.11 m, -7.5 % | 39.0 -> 48.0 %, +9.0 pt | NO EVENTS |
| `hk_office` | scoring | 0 -> 1 | 0 -> 0 | 102.89 -> 120.77 m, +17.4 % | 88.0 -> 88.5 %, +0.5 pt | NO EVENTS |
| `hk_park` | shipped | 1 -> **3** | 0 -> 0 | 44.34 -> 44.72 m, +0.9 % | 48.9 -> 48.2 %, -0.7 pt | **FAIL** crossings rose |
| `hk_park` | scoring | 4 -> 4 | 0 -> 0 | 190.44 -> 110.89 m, **-41.8 %** | 76.7 -> 76.6 %, -0.2 pt | **FAIL** crossings flat |
| `hk_elevator` | shipped | **4 -> 1** | 0 -> 0 | 58.42 -> 33.41 m, -42.8 % | 89.0 -> 74.7 %, **-14.4 pt** | **FAIL** coverage |
| `hk_elevator` | scoring | **5 -> 0** | **1 -> 0** | 88.87 -> 81.50 m, -8.3 % | 97.6 -> 98.1 %, +0.5 pt | **PASS** |
| `hk_entrance` | shipped | **3 -> 0** | **1 -> 0** | 52.99 -> 71.22 m, **+34.4 %** | 53.3 -> 69.8 %, +16.6 pt | **FAIL** path |
| `hk_entrance` | scoring | **3 -> 0** | 0 -> 0 | 161.46 -> 136.58 m, -15.4 % | 91.1 -> 90.6 %, -0.5 pt | **PASS** |
| `hk_allaround` | shipped | 1 -> 0 | 0 -> 0 | 70.20 -> 75.58 m, +7.7 % | 30.7 -> 39.1 %, +8.4 pt | set aside |

Per map, both configurations must hold: `hk_office` NO EVENTS, `hk_park` FAIL,
`hk_elevator` FAIL, `hk_entrance` FAIL. **0 PASS of 4. G2 FAILS.**

Read the failures rather than the verdict and they are not the same failure.
On `hk_entrance` and `hk_elevator`/scoring the candidate does exactly what it
was designed to do - crossings and round trips to zero - and is failed by a
budget clause, path +34 % on one and coverage -14 points on the other. On
`hk_park` it fails on the metric itself: crossings go UP in shipped and stay
flat in scoring while the path halves.

### 4 m, `pr2830+CMP` against `pr2830`

`hk_office` shipped PASS / scoring FAIL, `hk_park` FAIL both, `hk_elevator`
shipped PASS / scoring FAIL (round trips 1 -> 1), `hk_entrance` FAIL both
(path +5.8 % and +19.8 %). **0 PASS of 4. G3 FAILS.** The clearest cell is
`hk_entrance`/scoring, where `pr2830+CMP` takes 2 real crossings to 0 and is
failed on +19.8 % of path. The PR arm also moves much less under the wrapper
than stock does: the direction term changed its top choice on 2 to 13 decisions
per map against 30 to 70 for stock.

### 12 m, `stock+CMP` against `stock`, the regression pass

| map | config | crossings | round trips | median path | median coverage | verdict |
|---|---|---|---|---|---|---|
| `hk_office` | shipped | 2 -> 2 | 0 -> 1 | 36.87 -> 51.02 m, +38.4 % | 46.5 -> 68.4 %, +21.9 pt | FAIL |
| `hk_office` | scoring | 4 -> 3 | 1 -> 1 | 121.13 -> 160.91 m, +32.8 % | 88.2 -> 88.7 %, +0.5 pt | FAIL |
| `hk_park` | shipped | **6 -> 0** | **2 -> 0** | 35.35 -> 32.65 m, -7.6 % | 47.9 -> 58.3 %, +10.3 pt | **PASS** |
| `hk_park` | scoring | 2 -> 3 | 0 -> 1 | 172.12 -> 113.93 m, -33.8 % | 79.8 -> 79.2 %, -0.5 pt | FAIL |
| `hk_elevator` | shipped | **12 -> 4** | **4 -> 0** | 38.89 -> 40.41 m, +3.9 % | 96.2 -> 96.8 %, +0.5 pt | **PASS** |
| `hk_elevator` | scoring | **8 -> 3** | **3 -> 0** | 73.00 -> 50.13 m, -31.3 % | 98.7 -> 97.1 %, -1.5 pt | **PASS** |
| `hk_entrance` | shipped | 3 -> 3 | 0 -> 0 | 48.91 -> 49.16 m, +0.5 % | 80.5 -> 82.0 %, +1.5 pt | FAIL |
| `hk_entrance` | scoring | 2 -> 4 | 0 -> 0 | 73.04 -> 77.84 m, +6.6 % | 92.5 -> 92.3 %, -0.2 pt | FAIL |

**1 PASS of 4 (`hk_elevator`, both configurations). G2 still FAILS at 12 m**, but
this is the best the candidate looks anywhere in the workspace:
`hk_elevator`/shipped is 12 crossings to 4 and 4 round trips to 0 for +3.9 % of
path and +0.5 points of coverage, and `hk_park`/shipped is 6 to 0 and 2 to 0
while the path FALLS 7.6 % and coverage RISES 10.3 points. Those two cells are
the shape a real fix would have.

---

## 6. G4, and the finding hiding inside it: the filter is inert on this body

**G4, verbatim**: *COMPOSITE beats stock+M4.3 alone on at least half the
discriminating maps without extra path beyond the budget.*

**G4 FAILS, 0 of 4** - and the reason is not that the composite is worse. It is
that **on the Go2 body the composite and M4.3 are the same policy.**

What the wrapper counted about itself, 4 m, whole grid:

| arm | decisions | decisions with a drop | candidates seen | dropped | drop % | top choice dropped | valve fired | hook missing |
|---|---|---|---|---|---|---|---|---|
| `stock+CMP` | 683 | 10 | 5 304 | **10** | 0.19 % | 5 | 0 | 0 |
| `pr2830+CMP` | 605 | 0 | 4 640 | **0** | 0.00 % | 0 | 0 | 0 |
| `stock+CMP` (12 m) | 364 | 6 | 9 357 | **6** | 0.06 % | 0 | 0 | 0 |

The `hook missing` column is the one that had to be zero: it counts decisions
where the policy was not bound to a navigation view and the filter therefore did
nothing silently. It is zero in every cell, so the filter really did run.

`stock+CMP` produced an identical run to `stock+M4.3` on **53 of 54** paired
starts at 4 m: same path to the millimetre, same crossings, same coverage.

**Why, and it is a result about the PR rather than about our wrapper.** Piece (3)
exists because cand2's caveat 13 said neither scorer models body width: "the
costmap they are handed is inflated 0.25 m while the rover's lethal radius is
0.30 m, so both keep aiming at frontiers in pinches the body does not fit
through". That is true of a 0.46 m rover. For a 0.31 m Go2 under the same margin
rules the lethal radius is **0.225 m**, which is SMALLER than the 0.25 m the loop
already inflates by. Upstream's inflation over-covers the body it was written
for. There is essentially no frontier the scorer proposes that the Go2 cannot
reach, so the feasibility check has nothing to discard.

**The honest consequence for the ping-pong conversation**: "the scorer aims at
places the robot cannot fit" is OUR bug, produced by running their algorithm
with a body 48 % wider than theirs. It should not be taken to the maintainer as
a defect of dimOS. What remains of the composite on their robot is exactly
candidate M: the signed direction term at GBPlanner's ratio, plus its distance
penalty.

---

## 7. Which lidar range shows the behaviour, at Go2 speed

Full table `range_4m_vs_12m.txt`. Pooled over the four discriminating maps, same
starts, same profile, arm `stock`:

| config | range | runs | REAL crossings | round trips | median goals | median coverage |
|---|---|---|---|---|---|---|
| shipped | 4 m | 24 | **8** | 1 | 12.0 | 52.4 % |
| shipped | 12 m | 24 | **23** | **6** | 8.0 | 61.8 % |
| scoring | 4 m | 24 | 12 | 1 | 11.5 | 88.7 % |
| scoring | 12 m | 24 | **16** | **4** | 5.0 | 89.6 % |

The maintainer's instruction was "spawn the synthetic robot at the middle of the
space, and use a low-range lidar (3-5 m)". On our 0.15 m/s rover that was right:
short range meant the robot could not see across the floor, so the greedy scorer
had to commit and re-commit. On a robot that walks at 0.60 m/s the picture
inverts. At 4 m the Go2 crosses the visible neighbourhood inside one goal, the
run ends in a handful of goals, and there are fewer consecutive-goal pairs for
the metric to catch. At 12 m the robot sees the far end of the floor from the
start, the scorer has many distant frontiers ranked at once, and it ping-pongs
between them: three times the real crossings and six times the round trips.

**This is a testable statement to hand back**: on a Go2, the ping-pong is easier
to reproduce at the sensor's real range than at an artificially short one, and
the 3-5 m recipe was a compensation for a slow robot.

---

## 8. Plain failures

1. **`pr2830` published zero goals on 14 of 270 runs at 4 m** (7 as `pr2830`, 7
   as `pr2830+CMP`), on `hk_office`, `hk_park`, `hk_entrance` and
   `hk_allaround`. Those runs end on the harness idle detector with 2.5 % of the
   ceiling and 0.0 m of path. `stock` never does this. The PR's frontier A*
   blocks at cost 99, which walls off every pinch the Voronoi gradient touches -
   the failure mode `explore_sim.plan`'s own docstring warned about - and on
   floors whose median passable width is around 1 m that is most of the floor.
   It is not caused by the Go2 profile: the same arm, same code, on a body that
   fits MORE places.
2. **`hk_entrance`/`centre` is a dead start for every arm** at 4 m shipped: one
   goal, 0.8 m of path, 3.7 % coverage, all five arms identical. The gate had
   already measured one way out of that start. It is kept in the paired set
   because dropping starts after seeing them is how a bench lies.
3. **12 of 30 `hk_allaround` shipped runs hit the 900 s wall cap** and are marked
   capped in every table. On that map a more expensive policy explores less
   inside the same clock, which is a confound on that map alone.
4. **`hk_allaround` `scoring` produced nothing** (section 3).
5. **The composite lost coverage badly on individual starts even where medians
   held.** `per_run_regressions.txt`: 5 of 54 paired starts lose 10 points or
   more, worst `hk_elevator`/`centre`/shipped at **94.0 % to 53.2 %**, a run that
   was 40 points better under stock. Coverage was better on 29 starts and worse
   on 20. A median that moves +0.5 points is hiding a coin flip.

---

## 9. Figures

Five columns (`stock`, `stock+M4.3`, `stock+CMP`, `pr2830`, `pr2830+CMP`), the
style of the fix, fleet and cand2 benches: black dot = start, thin arrows = the
jump the selector asked for (robot at issue time to goal), numbers = goal order,
**solid orange = a real crossing**, **pale dashed orange = a class-N swing**.

| figure | why this pair |
|---|---|
| `goal_sequences_go2_12m_hk_elevator_shipped.png` | the strongest cell in the job: stock 12 real crossings and 4 round trips, `+CMP` 4 and 0, at +3.9 % path |
| `goal_sequences_go2_12m_hk_park_shipped.png` | 6 crossings and 2 round trips to zero, with path DOWN 7.6 % and coverage UP 10.3 points |
| `goal_sequences_go2_hk_elevator_shipped.png` | the 4 m counterpart, and the coverage failure: 4 crossings to 1, but 89 % to 75 % |
| `goal_sequences_go2_hk_elevator_scoring.png` | the only 4 m PASS on a map that also passes in the other config: 5 crossings and 1 round trip to zero |
| `goal_sequences_go2_hk_entrance_shipped.png` | crossings and round trips to zero bought with +34 % of path, and the dead `centre` start visible in every column |
| `goal_sequences_go2_hk_park_scoring.png` | the path halves, -41.8 %, and the crossings do not move: the clearest case of the candidate changing the route without changing the behaviour |
| `goal_sequences_go2_hk_office_shipped.png` | the composite INTRODUCING crossings, 0 to 2, and `pr2830` columns standing still at the start |
| `figs_go2/gate_china_office.png` | the floor that flips REJECT to PASS on body alone |

---

## 10. Caveats, in order of how much they matter

1. **The composite is not a third candidate on this body.** Sections 6. It is
   M4.3 plus a filter that fires on 0.1 % of candidates. Every G2/G3 number in
   this report is, on the Go2, a number about candidate M, and it should be
   quoted that way. The reachability piece was designed against a real gap that
   turns out to be ours.
2. **One speed for five floors, and the floors were not walked at one speed.**
   `hk_office` was recorded at 0.43 m/s and `hk_allaround` at 0.94. The bench
   runs all five at the pooled 0.603. Using the per-map speed would change the
   truncation distance per map and make maps incomparable; using one speed makes
   `hk_office` faster than it really was and `hk_allaround` slower. The
   direction of that distortion is opposite on those two maps and it is not
   corrected anywhere.
3. **The pivot rate rests on one floor.** 1 334 of the 1 492 pivot windows come
   from `hk_office`; `hk_elevator` contributes 12. The pooled 0.63 rad/s is
   effectively `hk_office`'s median. The direction of the correction (a Go2
   pivots faster than 0.5 rad/s) is not in doubt; the exact value is one floor's.
4. **Small integers.** Every crossing count is a sum over 6 paired starts. G2's
   `hk_park` failure at 4 m is 1 crossing becoming 3. Nothing here is a
   statistical claim and a difference of one or two events is not evidence on
   its own.
5. **`hk_allaround` is half-reported and `hk_office` never fired at 4 m.** Of the
   five HK floors, one contributes shipped-only data, one contributes no
   crossings at all at 4 m, and the verdicts therefore turn on three floors.
6. **The swing threshold still breaks on the biggest floor.** 35.33 m on
   `hk_allaround`; a 4 m explorer does not issue consecutive goals that far
   apart. Reported and set aside, never averaged in as a zero.
7. **The class-N filter is straight-line here**, geodesic in
   `../sim_2830_fix/diagnose_swings.py`; the two agreed on 63 of 64 traced
   swings in the earlier job. On a floor with more walls between robot and goal
   they would part company more often.
8. **The sim under-samples the real lidar by 3x per metre** (section 1), which
   makes coverage conservative under the Go2 profile. A denser scan model would
   raise every coverage number; whether it would change a verdict is untested.
9. **The simulated pose is perfect and unknown is a wall by construction.** A
   reversal in simulation is a decision, never noise. The real recordings had
   slips and relocalization jumps that no arm here pays for.
10. **One recording per floor, no repetition, no variance estimate.**
11. **The 12 m pass ran two arms only**, as declared, so there is no `pr2830` or
    `M4.3` column at 12 m and G3/G4 have no 12 m reading.
12. **Budget clauses decide three of the four G2 failures at 4 m**, in a bench
    whose per-cell samples are 6 starts. A +34 % median path over 6 runs is not
    a stable quantity, and the pre-declaration bound it anyway. Both the metric
    and the budget are printed for every cell so the reader can apply a
    different bar.

---

## 11. Files produced

Workspace:
`./`

| file | what |
|---|---|
| `hypotheses_go2.txt` | the pre-declared grid, profile, maps, metrics, G1-G4 and the reading rules, written before any comparative run |
| **`speed_derivation.txt`** | the three derivations with their per-map tables: walking speed, pivot rate, lidar spacing |
| `derive_speed.py` | the measurement, on the odom and lidar streams of nine dimOS recordings |
| `speed_raw.json`, `speed_windows.npz` | its output and the raw moving windows |
| **`go2_profile.py`** | the two robot profiles and the rule that installs one on explore_sim |
| **`gate_go2.txt`** | the worthiness gate, both bodies, same rules |
| `gate_go2_4m.json`, `gate_rover_4m.json`, `gate_table.py`, `figs_go2/` | the gate's raw output, the table renderer, the per-map figures |
| **`fix_composite.py`** | the COMPOSITE: pieces (1) and (2) imported from `fix_momentum.py`, piece (3) the body reachability filter |
| `bench_2830.py` | cand2's bench plus `--profile`, the composite in the policy resolver, the `bind_nav` hook and `policy.counters()` |
| `fix_momentum.py`, `fix_hysteresis.py`, `fix_tsp.py`, `dimos_selector.py`, `midstarts.py` | unchanged from cand2 |
| `pr2830/selector_base.py`, `pr2830/selector_head.py` | the two vendored upstream files, byte-identical, never touched |
| `analyse_go2.py` | G1-G4 judged per map and per config, plus the re-derivation check |
| `go2_4m_hk_*.json`, `go2_4mscoring_hk_entrance.json`, `go2_12m_hk_*.json` | the results, one file per invocation |
| `go2_4m.txt` / `.csv` / `_verdicts.json` | the 4 m tables, per-run CSV, machine-readable verdicts |
| `go2_12m.txt` / `.csv` / `_verdicts.json` | the 12 m regression pass |
| `range_4m_vs_12m.txt` | section 7 |
| `per_run_regressions.txt` | the per-start collapses the medians hide |
| `goal_sequences_go2_*.png` | the goal-sequence figures |
| `launch_go2.sh`, `figs_go2.sh`, `plot_cand2.py`, `regressions_go2.py` | the launcher, the figure driver, the plotter, the regression lister |
| `go2_4m*.log`, `go2_12m*.log` | the run logs, one line per run |

---

## 12. Integrity statement

- **Nothing was committed, pushed or posted.** No `git` command of any kind was
  run in this job.
- **The two vendored upstream files are byte-identical to their starting state**,
  on both machines, re-checked after the last run: `selector_base.py` md5
  **`e77c328643c959a49077115e8a341f2c`**, `selector_head.py`
  **`1ccc0c69fe88a72e402565feca988d26`**.
- **`hypotheses_go2.txt` was written before any comparative run.** Four things
  ran before it and all four are named inside it: the speed derivation, the two
  gate passes, a two-run mechanical smoke check of the composite, and one launch
  of the grid that was killed after 20 seconds with its partial output deleted,
  before any run finished, when the lidar-spacing question was raised. Nothing
  from that launch survives or is read.
- **The pre-declaration was amended once, and only before any comparative run**:
  the turn rate moved from "unchanged at 0.5 rad/s" to the derived 0.63, and the
  scan-rate paragraph gained its measurement. Both amendments are in the file
  with their reasons.
- **Nothing outside the declared grid was run.** No second ratio, no second
  filter variant, no re-run to chase a number. The `CMP0` and `CMPr` controls
  that `fix_composite.py` defines were never run in the grid.
- **Every per-map failure is printed** and no map is averaged away. The maps that
  could not answer are named as such.
- **The earlier workspaces were read, not modified.** `../sim_2830_fix/`,
  `../sim_2830_fleet/` and `../sim_2830_cand2/` were inputs only.
- **The box is left running**, with `/root/sim_2830_go2/`, `/root/data/` and
  `/root/logs/` intact, and no bench process alive.

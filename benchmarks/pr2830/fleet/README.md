# The B9 exploration fix on other floors: dataset hunt, gate, and a six-map bench

Offline, no robot, no flight. 2026-08-29. Workspace
`./`,
compute on a rented 192-core box, results synced back here. Nothing was committed,
pushed or posted.

The two upstream selector files were never touched: `pr2830/selector_base.py` is md5
`e77c328643c959a49077115e8a341f2c` and `pr2830/selector_head.py` is
`1ccc0c69fe88a72e402565feca988d26`, the same values as at the end of the fix job.

## Summary in six lines

1. **A genuinely multi-hallway floor exists and was found**: `hk_office`, from the dimOS
   public dataset `go2_hongkong_office`, a grid of aisles between desk blocks. `hk_park` is
   also classed multi-branch, though it is an open covered walkway with obstacle islands
   rather than hallways. Six new maps passed the gate out of nine extracted; three were
   rejected and are listed with their numbers.
2. **H1 passes on one map of six.** `stock+B9` removes real crossings on three floors
   (`hk_park` 15 to 7, `go2_short` 11 to 5, `hk_elevator` 6 to 4) and removes all four of
   `hk_park`'s round trips, but it removes nothing on `hk_office` and `hk_entrance` and
   overspends the path budget on both.
3. **H2 fails on every map.** On top of PR #2830 the wrapper changes almost nothing here,
   26 real crossings to 25 pooled, and on `hk_park` it makes the count worse, 6 to 8.
4. **H3 fails, and the answer to caveat 5 is blunt: R = 9 is not the best radius on any
   floor in this fleet where the metric can discriminate.** R = 6 wins on three maps,
   R = 12 wins on `hk_office` by a factor of two over R = 9. The 9 m that won on
   `bigoffice` was a `bigoffice` artefact.
5. **H4 holds pooled and fails on five maps of six.** At 12 m, B9 removes crossings on
   `hk_elevator` and `hk_office` while overspending path by 15 and 26 %, adds round trips on
   `go2_short` and `hk_park`, and is clearly harmful on `hk_allaround`.
6. **Two limitations dominate the rest.** Config `scoring` is unobtainable on the two
   biggest floors, so they contribute 6 paired starts instead of 12; and on `hk_allaround`
   the pre-declared swing metric counts zero events under every arm, because half its
   bounding-box diagonal is 35 m and a 4 m explorer never jumps that far.

The short version: the B9 fix is not a general fix. It helps on some floors, does nothing on
the one that best matches the complaint it was built for, and its radius does not transfer.


---

## 1. What this had to answer

`rapport_fix.md` left two open questions and they are the whole job here.

- **Caveat 5.** R = 9 m was not tuned; it was one of two declared values, and it won by
  2 swings on a 12-run sweep. Nothing in that report says 9 m generalises to another floor.
- **Caveat 3.** `bigoffice` and `bigoffice_hc` are one recording read by two occupancy
  algorithms. That floor is one large room plus one 20 m corridor, not the "middle of a
  space with several hallways" the dimOS maintainer described.

Everything below was pre-declared in `hypotheses_fleet.txt`, written before any arm
comparison was launched. One thing ran before that file and is named in it: a timing
pilot, stock arm only, config `shipped` only, on the four new maps that existed at the
time, to size the grid inside the compute budget. It contains no comparison between arms.

---

## 2. Where the maps came from

### 2.1 The store

The dimOS public LFS store is `https://lfs.dimensionalos.com/dimensionalOS/dimos`. The
repo's own `.lfsconfig` carries `fetchexclude = data/.lfs/*`, so a plain `git lfs pull`
in a clone fetches none of it. The 107 pointer files under `data/.lfs/` of
`dimensionalOS/dimos @ a7d19f761` name **49.0 GB** of compressed archives; the full list
with sizes is in `lfs_inventory.txt`.

Nothing was pulled blindly. The pointer files (7 kB in total) were copied to the box and
`lfs_get.py` asked the LFS batch API for the specific `oid`/`size` pairs of the objects
whose names suggested an indoor lidar + odom recording. **1.9 GB of the 49 GB was
downloaded.** Note for anyone repeating this: the task brief said the clone was at
``; that directory holds only a venv and one `.pc2.lcm` file and
is not a git repo. The clones with the pointer files are ``
and ``.

### 2.2 The extraction chain, and the check that it is the same chain

`extract_fleet.py` is `extract_bigoffice.py` with the source file as an argument:
accumulate every `lidar` frame of the store (and **refuse** the store if the frames are
not already in the `world` frame, rather than inventing pose maths), voxel-dedup at
0.05 m, hand the cloud to their own `dimos.mapping.pointclouds.occupancy` with algorithm
`simple`, write the `.npz` that `explore_sim.load_world` reads. Two differences from the
original, both named: the store class is `dimos.memory2.store.sqlite.SqliteStore` in the
build installed on the box (the local rig has it under `dimos.memory`), and the
accumulation is chunked so a 0.8 GB store is not concatenated twice.

**The chain was certified before it was used.** `go2_bigoffice.db` was pulled to the box
and re-extracted; the resulting `.npz` is **equal array for array** to the
`bigoffice.npz` the earlier benches ran on (`lidar`, `low`, `seen`, `res`, `ox`, `oy`,
`n`, `pose_xy`, `ts`, all `np.array_equal` True). The box was already certified on the
bench side: its 48 bigoffice smoke runs are bit-identical to the local rig.

### 2.3 What was inspected, and what came of it

| dataset | archive | frames | what it gave | outcome |
|---|---|---|---|---|
| `go2_hongkong_office.db` | 773 MB | 4 235 | `hk_office`, 32.8 x 36.4 m | gate PASS |
| `hk_building_all_around.db` | 278 MB | 1 799 | `hk_allaround`, 58.6 x 48.0 m | gate PASS |
| `hk_building_enterance.db` | 145 MB | 958 | `hk_entrance`, 37.5 x 28.0 m | gate PASS |
| `hk_building_park.db` | 124 MB | 884 | `hk_park`, 20.8 x 42.5 m | gate PASS |
| `hk_building_elevator.db` | 114 MB | 817 | `hk_elevator`, 40.8 x 32.3 m | gate PASS |
| `go2_short.db` | 84 MB | 461 | `go2_short`, 14.0 x 22.8 m | gate PASS |
| `go2_china_office.db` | 136 MB | 982 | `china_office`, 12.1 x 15.5 m | **gate REJECT** |
| `markers_go2.db` | 99 MB | 795 | `markers_go2`, 9.3 x 9.4 m | **gate REJECT** |
| `apartment.tar.gz` (`sum.ply`) | 18 MB | 338 local + 1 sum | `apartment`, 9.5 x 11.9 m | **gate REJECT** |
| `go2_bigoffice.db` | 196 MB | 2 251 | `bigoffice` re-extracted | chain check only |
| `go2_sf_office.tar.gz` | 26 MB | 74 pickles | pickled tuples, not a store | not extracted |
| `office_lidar.tar.gz` | 29 MB | 500 pickles | pickled dicts, not a store | not extracted |
| `go2_hongkong_office_twopass_map.pc2.lcm` | 3 MB | one map | a prebuilt cloud, not a recording | not extracted |

The three "not extracted" rows are honest leftovers: they are lidar data in per-frame
pickle or LCM containers rather than the SQLite store the certified chain reads, and
writing a second adapter for them would have been a second chain to certify. The six
gated maps were enough.

---

## 3. The gate: which floors can answer the question at all

`gate_fleet.py`, at 4 m, on the body-passable floor of the 0.46 m body. Two pre-declared
admission rules and one structure measure.

**Worthiness.** Per start, the share of the ceiling that appears only after a walk longer
than one lidar revolution. A start whose first revolution already reveals 50 % or more of
everything it can see is DEGENERATE (the rule of `gate_midstart.py`, unchanged). A map is
admitted when the median of that share over its 6 starts is at least 20 %. The 20 % was
declared against the two references already in the workspace: the shipped 12 m bench
scored 13.8 % on `bigoffice` and called it thin, and 0.0 to 2.4 % on the four flat maps it
then refused to draw conclusions from.

**Structure.** Ways out of each start (`midstarts.open_runs`: 32 rays, a direction open
when the body can drive 1.5 m along it, contiguous angular runs counted; a start with at
least 3 is a junction start), plus a branch count: the connected pieces of the passable
floor left once a geodesic ball of radius R around the graph centre is cut out, keeping a
piece only if it is at least 2 m2 and reaches at least 3 m further. R is swept over
{3, 5, 8, 12} m because the fleet's floors run from 8 m to 70 m across and one ball cannot
be the middle of both. Class: **multi-branch** if at least 3 junction starts AND at least
3 branches at some R; **single-room** if at most 1 branch at every R and at most 1 junction
start; **room+corridor** otherwise.

| map | grid | passable floor | bbox | swing threshold | median width | ways out of the 6 starts | junctions | branches R=3/5/8/12 | class | worst one-turn | beyond one range, median | gate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `hk_office` | 655 x 728 | 69 m2 | 10.5 x 27.7 m | 14.81 m | 0.86 m | [0, 3, 3, 3, 2, 2] | 3 | 1/1/3/2 | multi-branch | 10 % | 75.6 % | **PASS** |
| `hk_park` | 415 x 850 | 157 m2 | 16.0 x 40.0 m | 21.53 m | 1.25 m | [1, 5, 3, 2, 3, 2] | 3 | 2/2/2/3 | multi-branch | 12 % | 72.3 % | **PASS** |
| `hk_entrance` | 750 x 560 | 229 m2 | 34.3 x 23.2 m | 20.70 m | 1.32 m | [1, 3, 4, 2, 3, 5] | 4 | 1/2/2/2 | room+corridor | 7 % | 80.8 % | **PASS** |
| `hk_allaround` | 1172 x 960 | 544 m2 | 53.6 x 45.5 m | 35.13 m | 1.48 m | [0, 3, 4, 3, 5, 2] | 4 | 2/2/2/2 | room+corridor | 4 % | 92.3 % | **PASS** |
| `hk_elevator` | 815 x 646 | 140 m2 | 28.1 x 10.6 m | 15.03 m | 1.50 m | [2, 3, 2, 3, 3, 2] | 3 | 2/2/2/2 | room+corridor | 16 % | 59.3 % | **PASS** |
| `go2_short` | 280 x 455 | 34 m2 | 8.8 x 13.0 m | 7.84 m | 0.85 m | [1, 3, 2, 2, 4, 2] | 2 | 1/1/1/0 | room+corridor | 18 % | 58.3 % | **PASS** |
| `bigoffice` | 521 x 738 | 48 m2 | 11.6 x 20.6 m | 11.80 m | 1.10 m | [2, 3, 3, 2, 2, 2] | 2 | 2/2/2/0 | room+corridor | 27 % | 47.1 % | **PASS** |
| `bigoffice_hc` | 521 x 738 | 48 m2 | 11.6 x 20.7 m | 11.83 m | 1.08 m | [2, 4, 3, 2, 2, 3] | 3 | 2/2/3/0 | multi-branch | 28 % | 42.1 % | **PASS** |
| `china_office` | 241 x 310 | 10 m2 | 4.8 x 6.2 m | 3.91 m | 0.91 m | [2, 3, 2, 2, 4, 2] | 2 | 0/0/0/0 | room+corridor | 44 % | 1.6 % | **REJECT** (worthiness) |
| `apartment` | 190 x 237 | 9 m2 | 5.2 x 7.4 m | 4.54 m | 0.78 m | [1, 3, 2, 3, 2, 2] | 2 | 0/0/0/0 | room+corridor | 44 % | 5.8 % | **REJECT** (worthiness) |
| `markers_go2` | 186 x 188 | 14 m2 | 5.4 x 6.4 m | 4.18 m | 1.14 m | [3, 3, 2, 3, 2, 2] | 3 | 0/0/0/0 | room+corridor | 70 % | 0.8 % | **REJECT** (all 6 degenerate) |

### 3.1 What each floor actually looks like

The figures are `gate_<map>.png`: passable floor in blue, the core ball in grey, each
counted branch in its own colour, the graph centre a star, the 6 starts black dots.

- **`hk_office`** is the prize: a grid of aisles between desk blocks, aisles running
  north-south with cross links. This is the closest thing in the fleet to "the middle of a
  space with several hallways". It is also very narrow: median passable width 0.86 m for a
  0.46 m body.
- **`hk_park`** is a covered walkway 16 x 40 m with islands of obstacles (planters or
  pillars) in it. It classifies multi-branch, but the topology is an open strip with
  islands, not hallways meeting.
- **`hk_entrance`** is a large entrance hall to the west, a lobby in the middle and a long
  arm running south: an L with a big room at one end.
- **`hk_allaround`** is a **closed perimeter corridor loop** around a building core,
  54 x 46 m, median width 1.5 m. Structurally it is the most interesting floor here and
  the one nothing else in this workspace resembles. It is also, as section 5 shows, the
  one the pre-declared swing metric cannot measure.
- **`hk_elevator`** is a dumbbell: two lobbies joined by a narrow waist, with the graph
  centre sitting in the waist.
- **`go2_short`** is a small aisle grid, same kind as `hk_office` but with only 34 m2 of
  passable floor.
- **`bigoffice` / `bigoffice_hc`**, the anchors, are the one large room plus one 20 m
  corridor already described in the mid-start report.

### 3.2 Two things the gate itself says, and they are not flattering

**The class rule is soft, and the anchors prove it.** `bigoffice` and `bigoffice_hc` are
the same recorded floor read by two occupancy algorithms, and the classifier calls one
`room+corridor` and the other `multi-branch`: the height-cost map opens one extra doorway
wide enough for the body, which turns 2 branches at R = 8 m into 3 and 2 junction starts
into 3. A label that flips on the choice of occupancy kernel is not a strong label. The
raw numbers in the table are the content; the class is a convenience.

**Body mismatch is much worse here than on `bigoffice`.** The Go2 that recorded these
floors is about 31 cm wide and walks; the simulated rover is 46 cm, rolls and needs a
60 cm aisle. On `hk_office` the recording observed 351.6 m2 of free floor and the rover can
stand on **69 m2** of it; on `hk_elevator`, 397.0 m2 observed against 140 m2 passable; on
`hk_allaround`, 954.2 m2 against 544 m2. The bench therefore explores a skeleton of each
floor, not the floor. That is visible in every gate figure as the white area the colours do
not reach.


---

## 4. The grid, and how it ran

Pre-declared in `hypotheses_fleet.txt` before any arm comparison.

| | |
|---|---|
| maps | the 6 that passed the gate, plus `bigoffice` and `bigoffice_hc` as anchors |
| starts | 6 per map by the `midstarts.py` rule: `centre` + `mid1..mid5` |
| arms, 4 m | `stock`, `stock+B9`, `pr2830`, `pr2830+B9`, and on the new maps only `stock+B6` and `stock+B12` |
| arms, 12 m | `stock`, `stock+B9`, `pr2830`, `pr2830+B9`, new maps only |
| configs | `shipped` and `scoring`, both, everywhere as declared; section 5 says where that could not be delivered |
| primary metric | real crossings and real round trips; the raw counts are reported alongside |

**Two operational rules, both enforced.** `--maps` was passed explicitly on every
invocation, so no rover map with a mid start could be pulled in by a defaultless launch.
Every invocation ran under a `timeout` wrapper (1800 s for the map chunks, 1500 s for the
later salvage runs), and a **per-run wall cap** was added to the
harness (`--run-cap-s`, default 0 so no earlier bench in the workspace changes behaviour):
it is checked at the top of the decision loop, applies to every arm identically, and a run
that hits it ends with `end_reason` `wall_cap` and is counted as capped in every table
rather than dropped.

**Why the cap was needed, and what the timing pilot showed.** `explore_sim`'s own budgets
are in simulated units (300 goals, 600 m, 6000 s of simulated time). The frontier
detector's full-grid BFS is Python, so its cost scales with the grid: `hk_allaround` is
1 172 x 960 cells against `bigoffice`'s 521 x 738, and in the pilot four of its six runs
were still going at 420 s having issued 11 goals and covered 22 to 28 % of the ceiling.
The fleet therefore ran with a 600 s cap.

### 4.1 The per-run cap has a hole, and it cost one whole invocation

Reported because it is a finding about the harness, not a detail. The cap is checked at
the **top of the decision loop**, so it can only fire between decisions. In config
`scoring` the goal timeout is disabled by design (each arm is made to pay for the travel
its own choices cost), so a single `sim.drive` can spend a very long time of real clock
replanning across a 1 172 x 960 grid without ever returning to the loop.

The first attempt at the 4 m grid was one invocation of 432 runs. **429 of them finished
and 3 were still inside a single operation after 18 minutes**, three times their cap. The
cap did work on the rest: of the 114 runs that had been written out when the count froze,
26 were `hk_allaround` runs stopped by the cap, and their measured wall times are 602 to
635 s against a 600 s cap. `bench_2830.py` writes its result file only when the
whole pool has drained, so those three runs were holding 429 finished ones hostage. That
invocation was killed and the 4 m grid was re-run as **six invocations, one per map**, each
with its own 600 s per-run cap, its own 30 minute invocation cap and its own result file,
so that a pathological map can only lose itself. Everything reported below comes from that
second, chunked run.

Two things worth saying plainly about how that was handled. The monolithic invocation's
30 minute cap was extended twice while it was in its last batch, which was the wrong call:
it stacked timers instead of admitting the structure was wrong, and the right fix was the
chunking, which is what was done in the end. And the hole in the per-run cap is real: a
cap that can only fire between decisions is not a wall-clock cap. Closing it properly means
checking the clock inside `drive` as well, which is a change to the harness's inner loop
and was not made in the middle of a measurement.




---

## 5. What ran, and what could not

| grid | maps | arms | runs | state |
|---|---|---|---|---|
| 4 m, both configs | `hk_office`, `hk_park`, `hk_elevator`, `go2_short` | 6 | 288 | complete, 0 wall-capped |
| 4 m, `shipped` only | `hk_allaround`, `hk_entrance` | 6 | 72 | salvaged after the `scoring` half proved unobtainable, see below |
| 4 m anchors | `bigoffice`, `bigoffice_hc` | 4 | 96 | re-used from `../sim_2830_fix/` |
| 12 m regression | the 6 new maps | 4 | 240 | complete; the same two maps in `shipped` only |

**Config `scoring` is unobtainable on the two largest floors.** On `hk_allaround` and
`hk_entrance` the run set stops advancing at a specific task and never resumes, at the same
task index in two independent attempts each time. The bench yields results in task order,
so the index identifies the run: task 42 on `hk_allaround` is `mid3`/`scoring`/`stock` and
task 20 on `hk_entrance` is `mid1`/`scoring`/`pr2830`, read off the start/config/arm loop
order. It is deterministic, not luck. Config `scoring`
disables the goal timeout on purpose, so a single `sim.drive` on a 1 172 x 960 or 750 x 560
grid can replan for tens of minutes of real time without ever returning to the decision loop
where the wall cap is checked. Those two invocations were killed and re-run with `--configs
shipped` alone, which caps cleanly. So those two maps contribute **6 paired starts each
instead of 12**, and their numbers are labelled `shipped` only everywhere below. Nothing was
dropped for being unfavourable; what is missing is missing for this one reason.

---

## 6. The four-arm result at 4 m

Totals over the 12 paired starts of each map (6 middle starts x 2 configurations), except
where marked. Every count below was **re-derived from the raw goal coordinates**,
independently of the `summarise()` fields that wrote them: **360 runs checked at 4 m, 240 at
12 m, 96 anchor runs, zero disagreements** with the stored `cross_map_swings` in all 696.

| map | arm | runs | real crossings | runs with >= 1 | real round trips | raw swings | raw round trips | median path | median coverage | capped |
|---|---|---|---|---|---|---|---|---|---|---|
| `hk_office` | `stock` | 12 | 6 | 6/12 | 0 | 12 | 1 | 36.22 m | 51.7 % | 0 |
| `hk_office` | `stock+B9` | 12 | **6** | 5/12 | 0 | 9 | 0 | 40.72 m | 64.3 % | 0 |
| `hk_office` | `pr2830` | 12 | 6 | 4/12 | 1 | 8 | 2 | 44.18 m | 69.1 % | 0 |
| `hk_office` | `pr2830+B9` | 12 | **6** | 3/12 | 1 | 6 | 1 | 44.18 m | 69.1 % | 0 |
| `hk_park` | `stock` | 12 | 15 | 7/12 | 4 | 17 | 6 | 91.55 m | 82.9 % | 0 |
| `hk_park` | `stock+B9` | 12 | **7** | 6/12 | 0 | 8 | 1 | 87.62 m | 86.9 % | 0 |
| `hk_park` | `pr2830` | 12 | 6 | 5/12 | 0 | 8 | 1 | 41.98 m | 56.4 % | 0 |
| `hk_park` | `pr2830+B9` | 12 | **8** | 7/12 | 0 | 9 | 1 | 45.97 m | 68.9 % | 0 |
| `hk_entrance` | `stock` | 6 | 2 | 2/6 | 0 | 2 | 0 | 45.42 m | 44.4 % | 0 |
| `hk_entrance` | `stock+B9` | 6 | **2** | 2/6 | 0 | 2 | 0 | 48.69 m | 53.2 % | 0 |
| `hk_entrance` | `pr2830` | 6 | 1 | 1/6 | 0 | 1 | 0 | 44.89 m | 49.7 % | 0 |
| `hk_entrance` | `pr2830+B9` | 6 | **1** | 1/6 | 0 | 1 | 0 | 50.83 m | 54.5 % | 0 |
| `hk_allaround` | `stock` | 6 | 0 | 0/6 | 0 | 0 | 0 | 52.71 m | 27.5 % | 3 |
| `hk_allaround` | `stock+B9` | 6 | **0** | 0/6 | 0 | 0 | 0 | 52.55 m | 25.8 % | 4 |
| `hk_allaround` | `pr2830` | 6 | 0 | 0/6 | 0 | 0 | 0 | 55.98 m | 29.5 % | 4 |
| `hk_allaround` | `pr2830+B9` | 6 | **0** | 0/6 | 0 | 0 | 0 | 56.02 m | 30.4 % | 5 |
| `hk_elevator` | `stock` | 12 | 6 | 6/12 | 0 | 7 | 0 | 71.11 m | 95.9 % | 0 |
| `hk_elevator` | `stock+B9` | 12 | **4** | 4/12 | 0 | 5 | 0 | 72.63 m | 96.2 % | 0 |
| `hk_elevator` | `pr2830` | 12 | 5 | 5/12 | 0 | 5 | 0 | 60.55 m | 96.6 % | 0 |
| `hk_elevator` | `pr2830+B9` | 12 | **4** | 4/12 | 0 | 4 | 0 | 60.55 m | 96.6 % | 0 |
| `go2_short` | `stock` | 12 | 11 | 8/12 | 0 | 16 | 4 | 16.28 m | 46.9 % | 0 |
| `go2_short` | `stock+B9` | 12 | **5** | 4/12 | 0 | 14 | 5 | 16.31 m | 46.3 % | 0 |
| `go2_short` | `pr2830` | 12 | 8 | 8/12 | 0 | 14 | 5 | 18.92 m | 46.3 % | 0 |
| `go2_short` | `pr2830+B9` | 12 | **6** | 6/12 | 0 | 13 | 5 | 18.92 m | 46.3 % | 0 |

`hk_entrance` and `hk_allaround` are 6 paired starts (config `shipped` only), the others 12.
The 24 capped runs are all `hk_allaround`/`shipped`, and all of them stopped at exactly 14
goals, which is what the cap looks like when it works.

**`hk_allaround` produces zero cross-map swings under every one of the six arms.** Its
passable floor is a 54 x 46 m perimeter loop, so half its bounding-box diagonal is 35.13 m,
and a 4 m explorer that is still on its 14th goal after 600 s of real time never issues two
consecutive goals that far apart. The pre-declared metric has nothing to count there. That
is a property of the metric on a big floor, not a result about the arms, and the map is
reported and then set aside for H1 and H2 rather than averaged in as four zeroes.

The two configurations behind those totals separate cleanly and are in `fleet_4m.txt`. Two
rows worth naming here: on `hk_office` in config `shipped`, stock takes 3 real crossings and
`stock+B9` takes 1, at 40.6 % against 55.8 % of median coverage; on `hk_park` in config
`scoring`, stock takes 13 real crossings and 4 round trips against `stock+B9`'s 5 and 0.

### 6.1 H1, `stock+B9` against `stock`, judged per map

| map | class | starts | real crossings | real round trips | raw swings | median path | median coverage | verdict |
|---|---|---|---|---|---|---|---|---|
| `hk_park` | multi-branch | 12 | **15 -> 7** | **4 -> 0** | 17 -> 8 | 91.55 -> 87.62 m, -4.3 % | 82.9 -> 86.9 %, +4.8 % | **PASS** |
| `go2_short` | room+corridor | 12 | **11 -> 5** | 0 -> 0 | 16 -> 14 | 16.28 -> 16.31 m, +0.2 % | 46.9 -> 46.3 %, -1.3 % | FAIL, round trips only |
| `hk_elevator` | room+corridor | 12 | **6 -> 4** | 0 -> 0 | 7 -> 5 | 71.11 -> 72.63 m, +2.1 % | 95.9 -> 96.2 %, +0.3 % | FAIL, round trips only |
| `hk_office` | multi-branch | 12 | 6 -> 6 | 0 -> 0 | 12 -> 9 | 36.22 -> 40.72 m, **+12.4 %** | 51.7 -> 64.3 %, +24.4 % | **FAIL** |
| `hk_entrance` | room+corridor | 6 | 2 -> 2 | 0 -> 0 | 2 -> 2 | 45.42 -> 48.69 m, **+7.2 %** | 44.4 -> 53.2 %, +19.7 % | **FAIL** |
| `hk_allaround` | room+corridor | 6 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 52.71 -> 52.55 m, -0.3 % | 27.5 -> 25.8 %, **-6.1 %** | **FAIL** |
| all six pooled | | 48 | **40 -> 24** | **4 -> 0** | 54 -> 38 | **+5.04 %** | +2.5 % | **FAIL**, on path by 0.04 points |
| the four complete maps | | 48 | **38 -> 22** | **4 -> 0** | 52 -> 36 | +0.4 % | +4.7 % | PASS |

**H1 passes on one map of six.** Read honestly, in five statements.

1. **On three maps B9 removes real crossings, and on `hk_park` it removes the round trips
   too**: 15 crossings to 7 and 4 there-and-back pairs to none, at 4 % less path and 5 points
   more coverage. That is the clean win of this fleet, and it is on a map the classifier
   calls multi-branch.
2. **`go2_short` and `hk_elevator` fail only on a vacuous clause.** Neither arm produces a
   single round trip on either map, so the literal pre-declared "fewer round trips" cannot
   hold. Under the weaker rule the earlier fix bench used, "must not rise", both pass. The
   crossing drops there, 11 to 5 and 6 to 4, are real.
3. **`hk_office` is a genuine failure and it is the map that matters most**, the only floor
   in this fleet that looks like the several-hallways case the maintainer described. B9
   removes no real crossings at all, 6 to 6, and spends 12.4 % more path against a 5 %
   budget.
4. **`hk_office` and `hk_entrance` both buy coverage with that path.** `hk_office` gains 24.4
   points of median coverage, `hk_entrance` 19.7; in config `shipped` on `hk_office` the
   crossings do fall, 3 to 1, at 40.6 % against 55.8 % coverage. The extra path is more of
   the floor being explored before the upstream self-stop fires, not wasted motion. The
   pre-declared budget has no way to say that, so the verdicts stand as FAIL and this is the
   explanation, not a rescue.
5. **The pooled verdict turns on 0.04 of a percentage point.** Over all six maps the median
   path rises 5.044 % against a 5 % budget, so the pooled test fails; over the four maps that
   produced both configurations it rises 0.4 % and passes. A test that flips on the fourth
   decimal of a median is not measuring much, and both readings are printed rather than the
   convenient one.

---

### 6.2 H2, `pr2830+B9` against `pr2830`, judged per map

| map | starts | real crossings | real round trips | raw swings | median path | median coverage | verdict |
|---|---|---|---|---|---|---|---|
| `go2_short` | 12 | **8 -> 6** | 0 -> 0 | 14 -> 13 | +0.0 % | +0.0 % | FAIL, round trips only |
| `hk_elevator` | 12 | **5 -> 4** | 0 -> 0 | 5 -> 4 | +0.0 % | +0.0 % | FAIL, round trips only |
| `hk_office` | 12 | 6 -> 6 | 1 -> 1 | 8 -> 6 | +0.0 % | +0.0 % | **FAIL** |
| `hk_entrance` | 6 | 1 -> 1 | 0 -> 0 | 1 -> 1 | 44.89 -> 50.83 m, **+13.2 %** | +9.6 % | **FAIL** |
| `hk_allaround` | 6 | 0 -> 0 | 0 -> 0 | 0 -> 0 | +0.1 % | +2.9 % | **FAIL** |
| `hk_park` | 12 | **6 -> 8** | 0 -> 0 | 8 -> 9 | 41.98 -> 45.97 m, **+9.5 %** | 56.4 -> 68.9 %, +22.1 % | **FAIL** |
| all six pooled | 48 | 26 -> 25 | 1 -> 1 | 36 -> 33 | +0.1 % | +0.7 % | **FAIL** |

**H2 does not survive the fleet, on any map.** On the anchors, B9 on top of PR #2830 was the
stronger of the two results, 22 real crossings to 11 pooled. Here it moves almost nothing:
three maps where the median run is bit-identical between the two arms on path and coverage,
one where it does nothing, and one, `hk_park`, where it makes the crossing count **worse**,
6 to 8, while spending 9.5 % more path. Pooled, 26 to 25 is not a result.

The mechanism is visible in the per-map tables: PR #2830 already picks near frontiers, so on
these floors the vicinity rule usually has nothing to override. That is the same observation
the mid-start report made at 4 m from the other direction, where the two arms agreed on 76 %
of decisions, and it is stronger here.


### 6.3 The anchors, for continuity

`bigoffice` and `bigoffice_hc` re-analysed with this report's metrics, from
`../sim_2830_fix/results_trace_4m.json` and `results_fix_4m.json`, not re-run.

| map | comparison | real crossings | real round trips | raw swings | median path | median coverage | verdict |
|---|---|---|---|---|---|---|---|
| `bigoffice` | `stock+B9` vs `stock` | **10 -> 6** | **4 -> 0** | 11 -> 6 | -2.5 % | +0.1 % | **PASS** |
| `bigoffice_hc` | `stock+B9` vs `stock` | **12 -> 10** | **4 -> 3** | 22 -> 15 | -7.6 % | -0.2 % | **PASS** |
| `bigoffice` | `pr2830+B9` vs `pr2830` | **9 -> 8** | 0 -> 0 | 11 -> 8 | -1.7 % | -0.0 % | FAIL, round trips only |
| `bigoffice_hc` | `pr2830+B9` vs `pr2830` | **13 -> 3** | **4 -> 1** | 20 -> 9 | +3.4 % | +0.9 % | **PASS** |

The anchors reproduce the fix report exactly (33 to 21 raw swings pooled, median path
-3.2 %) and, on the primary metric, they are the strongest results in this whole document.
That is the contrast the fleet exists to draw: the wrapper looks good on the floor it was
built and tuned on, and much weaker on five others.

---

## 7. H3, the radius: R = 9 is never the best value on any new floor

R in {6, 9, 12} on the `stock` arm, real crossings totalled over the paired starts of each
map. Full table with paths, coverages and capped counts in `radius_4m.txt`.

| map | class | starts | stock | R = 6 | **R = 9** | R = 12 | best R | gap of R = 9 | H3 |
|---|---|---|---|---|---|---|---|---|
| `hk_park` | multi-branch | 12 | 15 | **6** | 7 | 7 | 6 | 1 | PASS |
| `go2_short` | room+corridor | 12 | 11 | **4** | 5 | 11 | 6 | 1 | PASS |
| `hk_elevator` | room+corridor | 12 | 6 | **3** | 4 | 4 | 6 | 1 | PASS |
| `hk_entrance` | room+corridor | 6 | 2 | 3 | **2** | **2** | 9 or 12 | 0 | PASS |
| `hk_allaround` | room+corridor | 6 | 0 | 1 | **0** | **0** | 9 or 12 | 0 | PASS, vacuously |
| `hk_office` | multi-branch | 12 | 6 | 11 | 6 | **3** | 12 | **3** | **FAIL** |

**H3 fails, on `hk_office`, and the shape of the failure is worse than the verdict.**
R = 12 takes 3 real crossings there against R = 9's 6 and R = 6's 11: a factor of nearly
four across the declared sweep on one map, with the ordering **reversed** against every
other map in the fleet, where R = 6 is the best value.

The flat statement, which is the answer to caveat 5 of `rapport_fix.md`:

> **R = 9 is not the best radius on any floor in this fleet where the metric can
> discriminate.** R = 6 wins on `hk_park`, `go2_short` and `hk_elevator`; R = 12 wins on
> `hk_office` by a factor of two over R = 9. R = 9 is best only on the two maps whose
> crossing counts are 0 and 2, where there is nothing to be best at.

R = 9 won on `bigoffice` by 2 swings in a 12-run sweep, and that ranking does not reproduce
anywhere here. The radius is a real knob with a real per-floor optimum, and 9 m was a
`bigoffice` artefact. It is also worth saying that a fixed radius is the wrong **shape** of
parameter: the Bormann precedent the candidate cites segments the floor plan into rooms and
finishes a room, and `hk_office` is exactly the floor where a ball of the wrong size cannot
stand in for one.

---

## 8. H4, the 12 m regression

H4 asks only that B9 is **not worse** than stock at the normal 12 m lidar range, inside the
same budgets: crossings must not rise, round trips must not rise, median path within +5 %,
median coverage within -5 %. 240 runs, four arms, the six new maps (`hk_allaround` and
`hk_entrance` in config `shipped` only, as at 4 m). 14 runs wall-capped, all
`hk_allaround`/`shipped`, all at 14 goals.

| map | real crossings | real round trips | raw swings | median path | median coverage | not worse? |
|---|---|---|---|---|---|---|
| `hk_entrance` | 3 -> 3 | 0 -> 0 | 3 -> 3 | -3.0 % | +4.2 % | **yes** |
| `hk_elevator` | **19 -> 12** | **6 -> 0** | 19 -> 12 | **+15.2 %** | +0.9 % | no, path |
| `hk_office` | **13 -> 11** | **3 -> 2** | 17 -> 15 | **+26.3 %** | +26.2 % | no, path |
| `hk_park` | **17 -> 13** | 2 -> **4** | 18 -> 14 | -13.4 % | -2.6 % | no, round trips |
| `go2_short` | 12 -> **13** | 1 -> **3** | 28 -> 21 | +0.0 % | +0.0 % | no, both counts |
| `hk_allaround` | 0 -> **2** | 0 -> **1** | 0 -> 2 | -41.3 % | **-20.6 %** | no, everything |
| all six pooled | **64 -> 54** | **12 -> 10** | 85 -> 67 | -6.7 % | +8.3 % | **yes** |

**H4 holds pooled and fails on five maps of six.** That is a different answer from the fix
report, which found at 12 m on `bigoffice` that B9 did not hurt and in fact improved both
arms. Three things to name.

1. **The two maps where B9 removes the most crossings are the two where it blows the path
   budget.** `hk_elevator` 19 to 12 crossings and 6 round trips to 0, at +15.2 % path;
   `hk_office` 13 to 11 and 3 to 2, at +26.3 % path against +26.2 % coverage. Same trade as
   at 4 m, larger.
2. **`go2_short` and `hk_park` go the wrong way on round trips**, 1 to 3 and 2 to 4, while
   `hk_park`'s crossings fall 17 to 13. A policy that cuts crossings and adds round trips is
   not doing what it was built to do on that floor.
3. **`hk_allaround` is where it is clearly harmful**: from 0 crossings and 0 round trips to
   2 and 1, with 20.6 % less coverage on a map where coverage was already only about 30 %.
   Those runs are wall-capped, so what the number says is that in the same 600 s of real
   time the B9 arm got less far. On the biggest floor in the fleet, the vicinity rule holds
   the rover in a neighbourhood it should be leaving.

The pooled row passes because `hk_elevator` and `hk_office` between them contribute most of
the crossings, and their drops outweigh the small rises elsewhere. Per the pre-declaration,
a failure on any map is reported as a failure on that map and not averaged away.



---

## 9. Figures

Same visual style as the fix bench, with one addition. Black dot = start, thin arrows = the
jump the selector asked for (robot at issue time to goal), numbers = goal order,
**solid orange = a real crossing**, **pale dashed orange = a class-N swing**, where the goal
sequence jumped more than the map's threshold and the robot did not. Four columns:
`stock`, `stock+B9`, `pr2830`, `pr2830+B9`.

| figure | what |
|---|---|
| `goal_sequences_fleet_hk_office_scoring.png` | the multi-branch office, the scoring config |
| `goal_sequences_fleet_hk_park_scoring.png` | the walkway, where H1 passes |
| `goal_sequences_fleet_hk_elevator_scoring.png` | the dumbbell |
| `goal_sequences_fleet_go2_short_scoring.png` | the small aisle grid |
| `goal_sequences_fleet_<map>_shipped.png` | the same six maps in the shipped config, including the two that have only that one |
| `summary_fleet_4m.png` | per-map bars: both crossing counts, both round-trip counts, path, coverage, goal-to-goal distance |
| `gate_<map>.png` | 11 gate figures: passable floor, core ball, counted branches, graph centre, the 6 starts |

### Re-derivation

Every headline count in this report was recomputed from the **raw goal coordinates** of each
run, with that map's own swing threshold, independently of the `summarise()` fields the
bench wrote: **360 runs at 4 m, 240 at 12 m, 96 anchor runs, and zero disagreements** with
the stored `cross_map_swings` in all 696. The class-N filter used here is straight-line and
`diagnose_swings.py`'s is geodesic; on the 64 traced swings of the bigoffice bench where
both exist they agree on 63, the one exception being a decision whose straight line is
2.86 m and whose route is 9.15 m.

---

## 10. Caveats, in order of how much they matter

1. **One recording per floor.** Every map here is a single dimOS walk read by one
   occupancy algorithm. `bigoffice` and `bigoffice_hc` remain the same walk read twice.
   There is no repetition and no variance estimate anywhere in this report.
2. **Counts, not significance tests.** Every number is a count or a median over 12 paired
   starts per map, and over 6 on `hk_allaround` and `hk_entrance`. The crossing counts are small integers, and on several maps they are
   single digits over 12 runs. Nothing here is a statistical claim, and a difference of
   one or two events is not evidence of anything on its own.
3. **The body is not the dataset's body.** The Go2 that recorded these floors is about
   31 cm wide and walks; the simulated rover is 46 cm, rolls and needs a 60 cm aisle. On
   `hk_office` the rover can stand on 69 m2 of the 352 m2 the Go2 observed as free. The
   bench explores a skeleton of each floor. This was already caveat 2 of the mid-start
   report and it is much larger here.
4. **The structure class is soft.** It flips between `bigoffice` and `bigoffice_hc`, which
   are the same floor. Read the raw columns, not the label.
5. **The swing threshold scales with the map and that breaks on the biggest one.** Half
   the bounding-box diagonal is 7.84 m on `go2_short` and 35.13 m on `hk_allaround`. A
   metric whose threshold varies by a factor of 4.5 across the fleet is not one number.
6. **Wall-capped runs are truncated, not failed.** A run that hit the cap was still going;
   its coverage and path are lower bounds and its crossing count is a count over a shorter
   run. Capped runs are listed individually in `fleet_4m.txt` and are never dropped.
7. **The class-N filter is straight-line here, geodesic in `diagnose_swings.py`.** They
   agree on 63 of the 64 traced bigoffice swings; the one disagreement is a decision where
   the straight line is 2.86 m and the route is 9.15 m. On a floor with more walls between
   robot and goal the two would part company more often.
8. **The simulated pose is perfect and unknown is a wall by construction**, as in every
   earlier bench in this workspace. A reversal in simulation is a decision, never noise.
9. **The anchors were not re-run.** Their 4 m numbers come from `../sim_2830_fix/`. The box
   was certified bit-identical to the local rig on 48 bigoffice runs before this job, and
   the extraction chain was re-certified here, so re-running them would have bought
   nothing but compute.
10. **Neither scorer models body width.** The costmap they are handed is inflated 0.25 m
    while the rover's lethal radius is 0.30 m, so both keep aiming at frontiers in pinches
    the body does not fit through. Unchanged from the shipped bench, and not the PR's
    fault.
11. **Config `scoring` is missing on the two biggest floors**, so `hk_allaround` and
    `hk_entrance` contribute 6 paired starts instead of 12 and only the configuration with
    a 45 s goal timeout. Whatever the scoring alone does on those two floors is unmeasured
    here.
12. **The pre-declared swing metric measures nothing on `hk_allaround`**: zero events under
    all six arms, because half the bounding-box diagonal of a 54 x 46 m perimeter loop is
    35.13 m. That map is reported and then set aside for H1 and H2 rather than averaged in
    as zeroes.
13. **`hk_allaround` is also the only map whose runs are truncated**, 24 of 36 at 4 m and 14
    of 24 at 12 m, all at exactly 14 goals and 25 to 37 % of the ceiling. Every conclusion
    on that map is a conclusion about the first 600 s of real time, not about a finished
    run.
14. **The `centre` start of `hk_allaround` is a start the body cannot leave.** Its ways-out
    count is 0 (no direction open for 1.5 m at 0.46 m of width) and every arm drives 0.0 m
    from it. It was kept because the pre-declared rule says 6 starts by the `midstarts.py`
    rule, and it is identical for every arm, but that start pair carries no information.
15. **The pooled H1 verdict turns on 0.04 of a percentage point** of median path (5.044 %
    against a 5 % budget). Both the six-map and the four-complete-map readings are printed.



---

## 11. Files produced

Workspace:
`./`

| file | what |
|---|---|
| `hypotheses_fleet.txt` | the pre-declared grid, gate rules, metric definitions and hypotheses |
| `lfs_inventory.txt` | the 107 objects of the dimOS public LFS store with their sizes |
| `lfs_get.py` | fetches named objects from the LFS batch API, overriding the repo's `fetchexclude` |
| `probe_store.py` | what is inside one recording (streams, counts, lidar frame_id, extent) before extracting it |
| `extract_fleet.py` | recording to `.npz`, the certified `extract_bigoffice.py` chain with the source as an argument |
| `ply_to_cloud.py` | a binary PLY (the apartment `sum.ply`) into the same cloud cache |
| `gate_fleet.py` | the worthiness gate plus the structure measure and the per-map figure |
| `gate_fleet_4m.json`, `gate_fleet_4m_extra.json` | the gate output, every start, every ring |
| `gate_<map>.png` | 11 figures: passable floor, core ball, counted branches, graph centre, the 6 starts |
| `bench_2830.py` | the bench, plus six map data lines and the per-run wall cap (`--run-cap-s`) |
| `analyse_fleet.py` | the tables: both swing counts, per map, per arm, and the pre-declared tests |
| `radius_table.py` | H3, the radius question |
| `plot_fleet.py`, `plot_summary.py` | the goal-sequence panels and the summary bars |
| `fleet_4m.csv`, `fleet_4m.txt`, `fleet_4m_verdicts.json` | the merged 4 m tables, per-run CSV and machine-readable verdicts |
| `fleet_4m_<map>.json` | the 4 m grid, one file per map (`hk_allaround` and `hk_entrance` as `..._shipped.json`) |
| `fleet_12m_<map>.json` | the 12 m regression, one file per map |
| `pilot_4m.json`, `fleet_4m_partial.*` | the timing pilot and the four-map interim analysis |
| `radius_4m.txt`, `radius_4m_partial.txt` | the R sweep and the H3 verdict |
| `fleet_12m.csv`, `fleet_12m.txt`, `fleet_12m_verdicts.json` | the merged 12 m tables and verdicts |
| `anchors_4m.csv`, `anchors_4m.txt` | the anchors, re-analysed from `../sim_2830_fix/` with the fleet's metrics |
| `goal_sequences_fleet_<map>_<config>.png` | the goal-sequence panels, four arms |
| `summary_fleet_4m.png` | the per-map summary bars |
| `fleet_4m_<map>.log`, `fleet_12m_<map>.log`, `ex_<map>.log` | the bench run logs, one line per run, and the extraction logs |
| `<map>.npz` (in the parent scratchpad) | the nine extracted maps |



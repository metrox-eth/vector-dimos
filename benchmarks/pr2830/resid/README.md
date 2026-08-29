# The residual crossings: is there another half?

Offline, no robot, no flight. 2026-08-29. Workspace
`./`,
compute on the rented 384-core box, results synced back here. Nothing was
committed, pushed or posted. No git operation of any kind was run.

The two vendored upstream selector files were never touched: at the end of this
job `pr2830/selector_base.py` is md5 `e77c328643c959a49077115e8a341f2c` and
`pr2830/selector_head.py` is `1ccc0c69fe88a72e402565feca988d26`, the same values
as at the end of the fix, fleet, cand2 and Go2 jobs.

This job dissects `../sim_2830_go2/rapport_go2.md` and reuses the taxonomy of
`../sim_2830_fix/diagnostic_swings.md` unchanged. Both earlier workspaces were
read, not modified.

---

## The verdict, first

**No. There is no other half.**

Under `stock` in the Go2's own conditions, 51 % of the real crossings are
fixable by re-ranking (class A + A-blocked + B) and 49 % are legitimate walking
(class C). Under the remedy `stock+M4.3`, **class A goes to exactly zero**, the
residual is **85 % class C**, and the 15 % that is not C is 5 class-B events
whose nearest available alternative was 23 to 33 m away. The ping-pong is
explained end to end: the fixable half was one mechanism, the remedy takes all
of it, and what is left is a robot that has finished a region and has to walk.

| pooled, 4 maps, both configs, both ranges | real crossings | A | A-blocked | B | C | fixable | legitimate |
|---|---|---|---|---|---|---|---|
| `stock` | **59** | 24 | 0 | 6 | 29 | **30 (51 %)** | **29 (49 %)** |
| `stock+M4.3` | **33** | **0** | 0 | 5 | 28 | **5 (15 %)** | **28 (85 %)** |

Six things carry this, and each of them is a number rather than a reading.

1. **Class A, the near-tie that loses, goes 24 to 0 on paired starts.** Not
   reduced, removed. Every one of the 24 stock class-A decisions is dumped with
   its candidate list in `resid_A_dumps.txt`.
2. **At the 33 residual crossings there is nothing anywhere near.** The nearest
   available candidate other than the one taken sits at a **median 27.7 m of
   route length, minimum 12.0 m**. Under stock the same figure is a median
   6.7 m, minimum 1.7 m. Widening the vicinity from 6 m to 12 m finds an
   available candidate at 41 of the 60 stock swings and at **1 of the 34**
   remedy swings. A bigger radius does not uncover a hidden class A.
3. **The remedy did not suppress legitimate long walks.** Cell by cell it
   removed 16 class-C crossings and introduced 15: **net -1 out of 29**. Total
   path over the 64 paired cells FALLS from 6 051 m to 5 096 m and median
   coverage moves +0.1 points.
4. **A-blocked is zero in both arms**, where it was 18 % of the fix job's rover
   crossings. That is section 6 of `rapport_go2.md` showing up in the
   behaviour: on a 0.31 m body the planner stops refusing what the scorer aims
   at, so the class that no re-ranking could reach has disappeared with the
   body.
5. **The mechanism behind class A is not the one the rover bench named.** On
   these floors at Go2 speed the near candidate loses on the **info-gain term
   on 16 of 24 decisions** and on the explored-goals repeller on 6. The frontier
   taken is a median 148-cell cluster, the near one that lost is a median
   34-cell cluster, the median info-gain gap is 0.183 of the score, and the
   20 % distance term claws back a median 0.08 of it.
6. **The maps where the remedy broke its budgets did not break them fighting
   legitimate crossings.** On the 21 paired cells where it spent more than 5 %
   extra path, it removed 8 class-A crossings and coverage rose on 18 of 21, by
   a median +7.4 points and up to +42.5.

What this does NOT say: it does not say the remedy is good. `rapport_go2.md`
failed it on G2, G3 and G4 and nothing here reverses that. It says the thing
the remedy is aimed at is the whole of the fixable behaviour, and a second
re-ranking fix has nothing left to aim at on these floors.

---

## 1. What ran, and the reproduction statement

**128 traced runs, 0 wall-capped, about 7 minutes of wall clock** on the box at
128 concurrent workers. Arms `stock` and `stock+M4.3`; configs `shipped` and
`scoring`; ranges 4 m and 12 m; maps `hk_office`, `hk_park`, `hk_elevator`,
`hk_entrance`. `hk_allaround` is out for the reason the Go2 bench gave: its
`scoring` configuration never finished there and its 35.33 m swing threshold is
not answerable by a 4 m explorer.

Starts: the cells where the recorded Go2 bench produced a real crossing under
`stock`, `stock+M4.3` or `stock+CMP`, unioned over the two configs so that no
config is decided by a start selected on the other one. Both arms ran on every
selected start, so **every comparison below is paired**: 64 cells, 128 runs.

**Trace mode reproduces the recorded runs exactly, and this was verified before
scaling and again at scale.**

- Before scaling, on one bare-arm run (`12m hk_park shipped centre stock`) and
  one policy-arm run (`4m hk_office shipped mid4 stock+M4.3`): every summary
  field and every goal coordinate identical to the recorded Go2 bench.
- At scale: **128 runs re-derived from their raw published goal coordinates by
  a second independent pass; 92 of them matched against the recorded Go2 bench
  on 12 summary fields plus every goal coordinate. Zero disagreements.** The
  36 that have no recorded counterpart are the 12 m `stock+M4.3` runs, an arm
  the Go2 bench never ran (its caveat 11).
- Those 36 are covered another way: **all 64 traced `stock+M4.3` runs are the
  same run as the recorded `stock+CMP` on the same cell**, identical on nine
  summary fields (path, coverage, area, goals, crossings, sim time, reached,
  timed out, jump total) and on every goal coordinate (`cmp_m43_cmp.py`). That
  re-checks `rapport_go2.md` section 6 (the reachability filter is inert on the
  Go2 body) run by run, and extends it to 12 m, where the Go2 bench could not
  check it.
- The classifier is the fix job's. `crosscheck_classifier.py` runs
  `../sim_2830_fix/diagnose_swings.py` and this job's classifier over the same
  traces: **94 of 94 swings get the same class from both.**
- The two class-N filters agree: the bench's straight-line rule and the
  geodesic rule used here both keep **92 of the 94** raw swings as real, and
  they agree run by run, not just in total.

The trace adds two report-only fields and changes no returned value: the five
weighted terms of the upstream score per candidate (four recomputed from the
selector's own config and own read-only helpers, the obstacles term derived from
the total so the expensive square search never runs twice), and, on a policy
arm, the policy's own adjusted score with the route length and deviation it
used. The pre-declaration is `hypotheses_resid.txt`.

---

## 2. Under stock: is their explorer dumb, in their own conditions?

Half of it is. Pooled over the four maps and both configs, class N removed:

| range | real crossings | A | A-blocked | B | C | fixable A+Abl+B | legitimate C |
|---|---|---|---|---|---|---|---|
| 4 m | 20 | 9 | 0 | 4 | 7 | **13 (65 %)** | 7 (35 %) |
| 12 m | 39 | 15 | 0 | 2 | 22 | **17 (44 %)** | 22 (56 %) |
| both | **59** | 24 | 0 | 6 | 29 | **30 (51 %)** | 29 (49 %) |

For comparison, the fix job's rover on `bigoffice` read C 47 %, A 31 %,
A-blocked 18 %, B 4 % over 45 real crossings. The Go2 on the HK floors reads
C 49 %, A 41 %, A-blocked 0 %, B 10 % over 59. The legitimate share is
essentially the same on two different bodies, two different map sets and two
different speeds; what moved is that **A-blocked turned into A**. On the rover,
18 % of the crossings were the planner refusing a pinch the 0.46 m body could
not enter. On the Go2 that class is empty and those decisions are simply
scoring losses.

### The 24 class-A decisions, in full

`k` is the arm's own score of the goal taken over the best near candidate's.

| range | map | start | config | goal | jump | chosen at | best near at | k | loses on |
|---|---|---|---|---|---|---|---|---|---|
| 12 m | hk_elevator | centre | shipped | 3 | 23.5 m | 21.4 m | 5.2 m | 1.87 | info_gain |
| 12 m | hk_elevator | mid1 | shipped | 2 | 16.3 | 13.5 | 6.0 | 1.32 | explored_goals |
| 12 m | hk_elevator | mid1 | scoring | 2 | 16.5 | 21.4 | 2.8 | 2.58 | explored_goals |
| 12 m | hk_elevator | mid4 | shipped | 4 | 26.0 | 20.4 | 5.1 | 2.58 | info_gain |
| 12 m | hk_elevator | mid4 | scoring | 2 | 19.8 | 24.0 | 5.8 | 1.38 | info_gain |
| 12 m | hk_elevator | mid5 | shipped | 2 | 18.2 | 9.3 | 6.0 | 1.62 | info_gain |
| 12 m | hk_elevator | mid5 | shipped | 3 | 19.2 | 16.0 | 3.5 | 1.16 | obstacles |
| 12 m | hk_elevator | mid5 | scoring | 2 | 18.3 | 20.2 | 5.7 | 1.32 | info_gain |
| 12 m | hk_entrance | mid3 | shipped | 4 | 27.7 | 31.4 | 4.0 | 1.77 | explored_goals |
| 12 m | hk_office | mid4 | shipped | 6 | 22.2 | 21.7 | 4.1 | 1.44 | info_gain |
| 12 m | hk_office | mid4 | scoring | 2 | 15.6 | 18.2 | 4.5 | 1.26 | info_gain |
| 12 m | hk_park | centre | shipped | 4 | 32.8 | 26.1 | 5.2 | 1.31 | info_gain |
| 12 m | hk_park | mid2 | shipped | 5 | 24.0 | 22.5 | 4.2 | 1.65 | info_gain |
| 12 m | hk_park | mid4 | shipped | 3 | 26.9 | 26.7 | 4.2 | 1.59 | info_gain |
| 12 m | hk_park | mid4 | scoring | 3 | 29.1 | 36.9 | 4.1 | 1.22 | explored_goals |
| 4 m | hk_elevator | mid1 | shipped | 13 | 22.7 | 23.5 | 4.0 | **1.05** | explored_goals |
| 4 m | hk_elevator | mid3 | shipped | 10 | 21.0 | 24.4 | 3.9 | **1.08** | info_gain |
| 4 m | hk_elevator | mid3 | scoring | 8 | 20.9 | 24.2 | 3.4 | 2.24 | info_gain |
| 4 m | hk_entrance | mid1 | shipped | 14 | 22.9 | 30.7 | 4.0 | **1.01** | explored_goals |
| 4 m | hk_entrance | mid1 | scoring | 17 | 28.7 | 34.6 | 4.3 | 1.14 | info_gain |
| 4 m | hk_entrance | mid3 | scoring | 14 | 23.8 | 26.9 | 4.5 | 1.30 | info_gain |
| 4 m | hk_park | centre | scoring | 6 | 23.4 | 23.7 | 4.0 | 1.10 | obstacles |
| 4 m | hk_park | mid2 | scoring | 12 | 27.2 | 45.6 | 5.5 | 1.16 | info_gain |
| 4 m | hk_park | mid3 | scoring | 9 | 27.8 | 30.7 | 1.7 | 3.58 | info_gain |

The shape the fix job found on the rover holds on the Go2, with one term
swapped. The near candidate that loses sits at a **median 4.2 m of route length**
(1.7 to 6.0) while the goal taken is at a **median 23.9 m** (9.3 to 45.6): a walk
about six times longer. The scores are near-ties, **median k 1.32**, and a
switching margin of k = 1.5 would have kept the robot near on 15 of 24, k = 2 on
20 of 24, k = 3 on 23 of 24.

**The losing term is `info_gain` on 16 of 24, `explored_goals` on 6, `obstacles`
on 2.** That is a different mechanism from the one the rover bench named, and it
is worth handing over as such. `info_gain_score` is
`min(frontier_size / (min_frontier_perimeter / resolution * 10), 1)` at a 30 %
weight. Over the 24 class-A decisions the frontier taken has a **median cluster
size of 148 cells** (69 to 886, median 250 at 12 m) and the near one that lost
has a **median of 34** (10 to 139). The big cluster saturates the term at its
full 0.3 and the small one gets a fraction of it: the median info-gain gap is
**0.183 of the total score**, against 0.084 for the explored-goals repeller,
while the 20 % distance term only claws back a median 0.08 in the other
direction. The scorer is not really trading distance against information here;
it is buying whichever frontier is biggest, and on these floors the biggest one
is across the building.

---

## 3. Under the remedy: what is left

| range | real crossings | A | A-blocked | B | C | fixable | legitimate |
|---|---|---|---|---|---|---|---|
| 4 m | 11 | **0** | 0 | 2 | 9 | 2 (18 %) | **9 (82 %)** |
| 12 m | 22 | **0** | 0 | 3 | 19 | 3 (14 %) | **19 (86 %)** |
| both | **33** | **0** | 0 | 5 | 28 | 5 (15 %) | **28 (85 %)** |

The pre-declared bar was: A alone above 20 % of the residual means an other half
exists. **A is 0 %.** The fixable half is covered.

The 15 % that is not class C is five class-B decisions, and they are not
near-misses either:

| range | map | config | start | goal | jump | nearest OTHER available candidate |
|---|---|---|---|---|---|---|
| 12 m | hk_entrance | shipped | mid5 | 7 | 22.2 m | 32.6 m |
| 12 m | hk_office | shipped | centre | 11 | 23.1 m | 30.2 m |
| 12 m | hk_office | scoring | mid2 | 8 | 24.5 m | 23.0 m |
| 4 m | hk_park | scoring | mid3 | 11 | 23.4 m | 30.5 m |
| 4 m | hk_park | scoring | mid4 | 9 | 30.8 m | 30.4 m |

Class B says a cluster in the region behind the robot blinked out and comes back
later. It does not say a better goal was available at the moment. At all five
of these decisions the closest available alternative anywhere on the floor was
23 m or more, so a frontier-persistence stabiliser would have had nothing near
to hold. The fix job declined to run that stabiliser on 2 swings out of 64; on
this evidence that decision still stands.

**The isolation number is the one to quote.** At the 33 residual crossings the
nearest available candidate other than the one taken is at a median 27.7 m of
route, minimum 12.0 m, and one decision had no other candidate at all. Widening
the vicinity radius does not help:

| arm | traced swings | available candidate within 6 m | within 9 m | within 12 m |
|---|---|---|---|---|
| `stock` | 60 | 26 | 37 | 41 |
| `stock+M4.3` | 34 | **1** | **1** | **1** |

That is what "legitimate" means, measured rather than asserted.

One thing the remedy does not fully clean up: the fix job's fifth finding, that
a crossing forced at the moment it happens may have been set up several goals
earlier. Under stock, 28 of the 29 class-C crossings were preceded by a decision
that walked away from a candidate within 6 m (median 1.5 goals before). Under
the remedy that falls to **18 of 28** (median 2.0 goals before). The remedy acts
at those earlier decisions too, and it does not act on all of them. That is
where a next candidate would have to look, and it is a lookahead question, not a
re-ranking one.

---

## 4. Did the remedy suppress any legitimate long walk?

Cell by cell over the 64 paired starts: **16 class-C crossings removed, 15
introduced, net -1 out of 29.** The remedy is not trading legitimate walking for
its class-A reduction; it changes which legitimate walks happen.

| paired totals, 64 cells | real | A | A-blocked | B | C |
|---|---|---|---|---|---|
| `stock` | 59 | 24 | 0 | 6 | 29 |
| `stock+M4.3` | 33 | 0 | 0 | 5 | 28 |

The cost side, on the same 64 cells: total path **6 051 m to 5 096 m (-16 %)**,
median per-cell path **-8.9 %**, median coverage **+0.1 points**. On this
crossing-bearing subset the remedy is cheaper, not dearer. (That is a subset
selected because it contains crossings, so it is not the grid-wide budget
answer; `rapport_go2.md` section 5 remains the budget answer.)

The 15 cells where a class-C crossing disappeared, with what they cost:

| range | map | config | start | C | path | coverage |
|---|---|---|---|---|---|---|
| 12 m | hk_elevator | scoring | mid3 | 1 to 0 | 66.2 to 30.0 m (-54.7 %) | 98.7 to 96.7 % (-2.0 pt) |
| 12 m | hk_elevator | scoring | mid4 | 1 to 0 | 85.1 to 48.1 m (-43.5 %) | 98.8 to 97.0 % (-1.8 pt) |
| 12 m | hk_elevator | shipped | mid3 | 1 to 0 | 39.0 to 28.1 m (-27.9 %) | 97.1 to 96.5 % (-0.6 pt) |
| 12 m | hk_elevator | shipped | mid4 | 1 to 0 | 49.2 to 43.6 m (-11.4 %) | 96.0 to 96.9 % (+0.9 pt) |
| 12 m | hk_entrance | shipped | mid1 | 1 to 0 | 47.1 to 55.4 m (+17.5 %) | 88.5 to 88.5 % (0.0 pt) |
| 12 m | hk_entrance | shipped | mid5 | 1 to 0 | 50.7 to 40.1 m (-20.8 %) | 72.5 to 64.1 % (**-8.4 pt**) |
| 12 m | hk_office | scoring | mid2 | 2 to 0 | 139.0 to 167.4 m (+20.4 %) | 88.9 to 89.6 % (+0.7 pt) |
| 12 m | hk_office | shipped | mid2 | 1 to 0 | 41.8 to 57.6 m (+38.0 %) | 49.2 to 72.5 % (+23.4 pt) |
| 12 m | hk_park | shipped | centre | 1 to 0 | 29.7 to 26.3 m (-11.6 %) | 44.1 to 51.5 % (+7.4 pt) |
| 12 m | hk_park | shipped | mid4 | 1 to 0 | 42.0 to 22.9 m (-45.5 %) | 50.6 to 44.3 % (-6.3 pt) |
| 4 m | hk_elevator | scoring | centre | 1 to 0 | 118.2 to 71.3 m (-39.7 %) | 98.4 to 97.9 % (-0.5 pt) |
| 4 m | hk_elevator | scoring | mid2 | 1 to 0 | 97.5 to 93.1 m (-4.5 %) | 98.4 to 98.6 % (+0.2 pt) |
| 4 m | hk_elevator | scoring | mid3 | 1 to 0 | 96.1 to 86.2 m (-10.3 %) | 97.1 to 98.6 % (+1.4 pt) |
| 4 m | hk_elevator | shipped | centre | 1 to 0 | 66.6 to 28.8 m (-56.7 %) | 94.0 to 53.2 % (**-40.8 pt**) |
| 4 m | hk_entrance | shipped | mid1 | 1 to 0 | 59.0 to 82.7 m (+40.2 %) | 57.1 to 81.1 % (+24.0 pt) |

Two of the fifteen are a real loss and they should be said plainly.
`4m hk_elevator/shipped/centre` is the collapse `rapport_go2.md` section 8.5
already named, 94.0 % to 53.2 %, and this classification adds what happened:
the remedy kept the robot local, the run ended 38 m of path earlier on the
explorer's own info-gain self-stop, and one legitimate return walk never
happened because the run was already over. `12m hk_entrance/shipped/mid5` is
the same shape, smaller, -8.4 points. **That is the failure mode to watch: not
a suppressed crossing, an early self-stop.** The other thirteen cells either
gained coverage or lost under 2 points.

---

## 5. The maps where the remedy broke its budgets

Per (range, map) over the traced paired cells. `real` is class-N-free.

| range | map | cells | real | A | B | C | median path | median coverage | cells with path up |
|---|---|---|---|---|---|---|---|---|---|
| 4 m | hk_office | 4 | 0 to 3 | 0 to 0 | 0 to 0 | 0 to 3 | +5.1 % | +0.3 pt | 2 / 4 |
| 4 m | hk_park | 10 | 5 to 7 | 3 to 0 | 0 to 2 | 2 to 5 | -13.2 % | +0.1 pt | 1 / 10 |
| 4 m | hk_elevator | 8 | 9 to 1 | 3 to 0 | 2 to 0 | 4 to 1 | -7.4 % | +0.7 pt | 2 / 8 |
| 4 m | hk_entrance | 6 | 6 to 0 | 3 to 0 | 2 to 0 | 1 to 0 | +13.4 % | +3.4 pt | 3 / 6 |
| 12 m | hk_office | 6 | 6 to 5 | 2 to 0 | 0 to 2 | 4 to 3 | +29.2 % | +12.0 pt | 5 / 6 |
| 12 m | hk_park | 10 | 8 to 3 | 4 to 0 | 1 to 0 | 3 to 3 | -17.1 % | +3.8 pt | 3 / 10 |
| 12 m | hk_elevator | 12 | 20 to 7 | 8 to 0 | 1 to 0 | 11 to 7 | -12.4 % | +0.1 pt | 2 / 12 |
| 12 m | hk_entrance | 8 | 5 to 7 | 1 to 0 | 0 to 1 | 4 to 6 | -5.1 % | -0.3 pt | 3 / 8 |

Across the 21 paired cells where the remedy spent more than 5 % extra path, it
removed 8 class-A crossings, added 1 class-B and 3 class-C, and coverage rose on
18 of the 21 by a median +7.4 points, up to +42.5. **The extra path is not spent
fighting legitimate crossings. It is spent covering floor the stock run had left
unseen.**

**`hk_park`, 4 m (G2 FAIL: crossings rose 1 to 3 in `shipped`, flat 4 to 4 in
`scoring` while path halved).** This is the failure that looked worst in
`rapport_go2.md` because it failed on the metric itself rather than on a budget
clause, and the classification turns it around. On the ten traced cells the real
crossings go 5 to 7, but the composition goes A 3 to 0, B 0 to 2, C 2 to 5.
Every crossing the remedy removed was a near-tie it should have removed, and
every crossing it added is a decision where the nearest available alternative
was 30 m away. Median path on these cells falls 13.2 % and median coverage moves
+0.1 points. The count went up and the behaviour got better; the metric cannot
see the difference, which is exactly why this classification exists. `hk_park`
is a park: long open runs with frontier clusters at both ends, and a robot that
stops ping-ponging finishes one end and then legitimately walks to the other.

**`hk_elevator`, 4 m (G2 FAIL on coverage, 89.0 % to 74.7 % in `shipped`).**
The remedy does its job on this map more clearly than anywhere else: 9 real
crossings to 1, A 3 to 0, B 2 to 0, C 4 to 1, median path -7.4 % and median
coverage +0.7 points across the eight cells. The budget failure is concentrated
in one cell, `centre`/`shipped`, where coverage falls 40.8 points, and it is not
a crossing story at all: the run ends early on the explorer's own info-gain
self-stop after 28.8 m of path instead of 66.6 m. Five of the eight traced cells
gain coverage, and among the four traced `shipped` cells the other three move
+12.5, +4.9 and -6.0 points. The recorded -14.4 point median is over six starts,
two of which are not in this traced set; what this classification can say is
that the collapse is a self-stop interacting with a locality bias, not a
suppressed legitimate walk. At 12 m the same map is the strongest cell in the
whole workspace and passes both configurations.

**`hk_entrance`, 4 m (G2 FAIL on path, +34.4 % in `shipped`) and 12 m (FAIL both
configs on the metric).** At 4 m the remedy takes all six real crossings to
zero, A 3 to 0 and B 2 to 0, and the path bill is real: median +13.4 % over the
six cells, +157 % on `mid3`/`shipped` alone. What that cell bought is +32.6
points of coverage, and `mid1`/`shipped` bought +24.0 points for +40 %. The map
is a large entrance hall whose far side stock never reached inside its self-stop;
the remedy walks it. At 12 m the picture inverts and the map is the one place
where the residual grows: real 5 to 7, with A 1 to 0 and C 4 to 6, median path
-5.1 % and median coverage -0.3 points. So at 12 m `hk_entrance` is a genuine
no-improvement: the remedy removes its one fixable crossing and picks up
legitimate ones for no coverage gain. The worst cell is `shipped`/`mid2`, where
it walks **27.0 % less, covers 16.9 points less, and still gains a class-C
crossing**: that is a run that stopped early in a different place, not one that
explored better. `hk_entrance` at 12 m is the weakest result in this report and
it is not hidden by anything.

**`hk_office`, 12 m (G2 FAIL both configs on path, +38.4 % and +32.8 %).** The
clearest case that the budget clause is measuring the wrong thing here. Five of
the six traced cells spend more path and the map's median coverage rises **12.0
points**, with three cells at +42.5, +41.8 and +23.4. Real crossings go 6 to 5,
A 2 to 0, and the biggest single addition is `shipped`/`centre`, which goes 0 to
2 (one B and one C) on the cell that gained **+42.5 points of coverage** for
+83.9 % of path: a robot that explores twice as much floor issues more goals and
eventually has to cross the floor it opened.
The stock runs on this map were not better behaved, they were shorter. At 4 m
the same map is `NO EVENTS` under stock, and the three crossings the remedy
shows there are all class C for the same reason.

**`hk_park`, 12 m (`scoring` FAIL, crossings 2 to 3 and round trips 0 to 1).**
Same reading as `hk_park` at 4 m, on fewer events. Over the five `scoring` cells
the composition goes A 1 to 0 and C 1 to 3 while median path falls sharply
(-17.1 % over the map's ten cells, with -59.1 %, -53.7 % and -23.0 % on three
`scoring` starts) and median coverage rises 3.8 points. The `shipped`
configuration on the same map is the cell `rapport_go2.md` calls the shape a
real fix would have: 8 real crossings to 3, A 4 to 0, path down, coverage up.

---

## 6. Caveats, in order of how much they matter

1. **Small integers, again.** 59 crossings under stock and 33 under the remedy,
   over 64 paired starts. The headline (A 24 to 0) is large enough to survive
   a few misclassifications; the per-map rows are not. `hk_office` at 4 m rests
   on 3 events, `hk_entrance` at 12 m on 12.
2. **The selection is the crossing-bearing subset.** Starts were chosen because
   the recorded bench found a crossing there. That is the right sample for
   classifying crossings and the wrong sample for judging budgets: the path and
   coverage figures in section 4 describe these 64 cells, not the grid.
   `rapport_go2.md` section 5 is still the budget answer.
3. **R_near is 6.0 m at both ranges.** It was justified at 4 m (1.5 lidar
   ranges) and is 0.5 lidar ranges at 12 m. Keeping it fixed is what makes the
   two ranges comparable, and the sensitivity table in section 3 is there so the
   reader can see that the verdict does not depend on it: at R = 12 m the remedy
   still has an available candidate at 1 of 34 swings.
4. **Class A on a policy arm is measured on the policy's score.** That is the
   right comparison and it makes no difference to this verdict: the class labels
   are assignment on geometry and suppression only, and both classifiers agree
   on all 94 swings including the 34 policy-arm ones. With A at zero there is
   nothing for the score choice to change.
5. **A-blocked being zero is a property of the Go2 body, not of dimOS.** On our
   0.46 m rover it was 18 % of crossings. Anyone re-running this with a wider
   robot will see that class come back, and it is unreachable by re-ranking.
6. **The `info_gain` finding is a reading of 24 decisions on four floors.** The
   term decomposition is exact (the five weighted terms sum to the score the
   selector returned, to 1e-6), but "the near candidate loses most on info gain"
   is a per-decision argmax over a delta, not a sensitivity analysis. It says
   where to look, not what to change.
7. **The simulated pose is perfect and unknown is a wall by construction**, so a
   crossing in simulation is always a decision and never noise. The real
   recordings had slips and relocalisation jumps that no arm here pays for.
8. **One recording per floor, no repetition, no variance estimate**, as in every
   bench in this workspace.
9. **`hk_allaround` is absent.** Of the five HK floors, the verdict turns on
   four.
10. **Nothing here re-opens G2/G3/G4.** The remedy failed them and this report
    does not re-judge them; it explains what the failures are made of.

---

## 7. Files produced

Workspace:
`./`
(mirrored on the box at `/root/sim_2830_resid/`, logs in `/root/logs/resid_*.log`).

| file | what |
|---|---|
| **`rapport_resid.md`** | this report |
| `hypotheses_resid.txt` | the taxonomy, the radii, the arms and the reading rules, declared before the traces were read |
| **`resid_classification.json`** | every traced swing with its class, its full candidate list (size, upstream score, five weighted terms, policy route and deviation, policy score, geodesic and straight-line distance, suppression flag), plus the per-run and paired tables and the reproduction record |
| **`resid_A_dumps.txt`** | per-decision dumps: part 1, the class-A residuals under the remedy (none); part 2, the 24 class-A decisions under stock the remedy removed, with every candidate |
| `resid_counts.txt` | the console output of `diagnose_resid.py`, all tables |
| `diagnose_resid.py` | the classifier |
| `diagnose_swings_ref.py` | the fix job's classifier, unmodified, kept as the reference |
| `crosscheck_classifier.py` | runs both classifiers on the same traces and prints any class that differs (94 of 94 agree) |
| `cmp_m43_cmp.py` | compares every traced `stock+M4.3` run to the recorded `stock+CMP` on the same cell (64 of 64 identical) |
| `select_runs.py`, `go2_per_run_crossings.json`, `trace_cells.json` | which recorded runs contain a real crossing, and the cell list the wave ran |
| `launch_resid.sh` | the wave, one invocation per (range, map) |
| `bench_2830.py` | the Go2 bench's own file plus the two report-only trace fields, documented in place |
| `resid_4m_hk_*.json`, `resid_12m_hk_*.json` | the 128 traced runs |
| `verify_one.json`, `verify_two.json`, `verify_pol.json` | the pre-scaling reproduction checks: before instrumentation, after it, and on a policy arm |
| `go2_profile.py`, `fix_*.py`, `dimos_selector.py`, `midstarts.py`, `pr2830/` | copied unchanged from `../sim_2830_go2/` |

---

## 8. Integrity statement

- **Nothing was committed, pushed or posted.** No `git` command of any kind was
  run in this job.
- **The two vendored upstream files are byte-identical to their starting state**
  on both machines: `selector_base.py` md5
  **`e77c328643c959a49077115e8a341f2c`**, `selector_head.py`
  **`1ccc0c69fe88a72e402565feca988d26`**.
- **`hypotheses_resid.txt` was written before the traces were read.** The three
  reproduction runs ran before it and are named inside it.
- **The trace changes no returned value.** Proven twice before scaling, on a
  bare arm and on a policy arm, and then on 92 of the 128 runs at scale against
  the recorded bench, with zero disagreements.
- **No run was dropped after being seen.** All 128 finished, none hit the 900 s
  wall cap, and every cell selected is in the tables including the ones with
  zero crossings in both arms.
- **Nothing outside the declared grid was run.** No second radius, no second
  ratio, no re-run to chase a number.
- **The earlier workspaces were read, not modified.** `../sim_2830_fix/` and
  `../sim_2830_go2/` were inputs only.
- **The box is left running**, with `/root/sim_2830_resid/`, `/root/data/` and
  `/root/logs/` intact, and no bench process alive.

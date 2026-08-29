# Correction: the vicinity sensitivity table of `rapport_resid.md`

2026-08-30. Local correction, offline, no simulation re-run.

An external audit found a predicate bug in `diagnose_resid.py`. It is real, it
is confirmed, and it changes the published vicinity sensitivity table. It does
not change the headline 51/49 split, and the reason it cannot is structural
rather than lucky. A separate claim, that the split is robust to the vicinity
radius, turns out not to be supported by the corrected table nor by the buggy
one, and that is stated plainly below.

---

## 1. The bug

`diagnose_resid.py` computed two vicinity fields per traced swing decision:

| field | excluded from the search | correct? |
|---|---|---|
| `nearest_other_geo_m` | the chosen candidate AND the suppressed ones | yes |
| `nearest_other_available_geo_m` | the suppressed ones ONLY | **no** |

Both are meant to answer "how far away is the nearest OTHER thing the robot
could have gone to". The second one forgot the word "other": because a chosen
candidate is never a suppressed one in this data (verified, 0 of 94 decisions),
the goal the robot had just taken was always in its own candidate pool. So

```
nearest_other_available_geo_m (buggy) = min(d_geo_chosen_m, nearest_other_geo_m)
```

and whenever the robot's own goal was the closest thing on the floor, the field
reported the distance to the goal it had taken, labelled as the distance to an
alternative it had passed up.

The fix, applied to `diagnose_resid.py`, adds `not is_chosen(c)` to the second
predicate so that the two fields now agree by construction. The pre-correction
value is still emitted per decision, under `nearest_other_available_geo_m_buggy`,
so the audit trail is preserved.

## 2. How much of the data was affected

19 of the 94 traced decisions had their value changed. Every one of those 19 had
the chosen goal as its own "nearest alternative", so the bug fired exactly where
predicted and nowhere else.

Of the 19:

- **2 are class N** (the goal jumped but the robot did not; the goal taken sits
  2.17 m and 2.83 m of route away). These are the only two whose buggy value
  falls inside any tested radius, and they are the entire difference in the
  published table.
- **17 are real crossings** where the goal taken was 7.7 m or further away. For
  these the correction only nudges the value upward, from the goal taken to the
  true nearest other candidate, and never across a 4 / 6 / 9 / 12 m threshold.
- **7 decisions turn out to have no other candidate at all** (their corrected
  value is null, where the buggy field always found "itself"): 1 class N and 4
  class C under `stock`, 1 class N and 1 class C under `stock+M4.3`.

## 3. Corrected vicinity sensitivity table

Traced swings with an available candidate **other than the one taken** within R
metres of route length. Denominator is traced swings, as published.

### `stock` (n = 60 traced)

| scope | n | R = 4 | R = 6 | R = 9 | R = 12 |
|---|---|---|---|---|---|
| 4 m range | 20 | 5 -> 5 | 9 -> 9 | 15 -> 15 | 15 -> 15 |
| 12 m range | 40 | 6 -> **5** | 17 -> **16** | 22 -> **21** | 26 -> **25** |
| **pooled** | 60 | 11 -> **10** | 26 -> **25** | 37 -> **36** | **41 -> 40** |

### `stock+M4.3` (n = 34 traced)

| scope | n | R = 4 | R = 6 | R = 9 | R = 12 |
|---|---|---|---|---|---|
| 4 m range | 12 | 1 -> **0** | 1 -> **0** | 1 -> **0** | 1 -> **0** |
| 12 m range | 22 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 |
| **pooled** | 34 | 1 -> **0** | 1 -> **0** | 1 -> **0** | **1 -> 0** |

Values shown as `old -> corrected`; bold marks a cell that moved. The published
table (R = 6 / 9 / 12: `stock` 26 / 37 / 41, `stock+M4.3` 1 / 1 / 1) is
reproduced exactly by the old column, which confirms the recomputation is
measuring the same thing the report published.

Taking the class-N decisions out of the denominator makes old and corrected
identical at every radius (`stock` 10 / 25 / 36 / 40 of 59, `stock+M4.3`
0 / 0 / 0 / 0 of 33). That is the whole of the effect: the bug counted two
decisions in which the robot was already standing next to its own goal as
decisions in which it had walked away from a nearby alternative.

## 4. Headline split at R = 6, cross-check

**The split does not move.** Verified, and the audit's expectation is confirmed.

Recomputing the class of all 94 decisions from the corrected fields against the
labels stored by the original run gives **0 class changes** when the test is run
at the original full precision. This is a hard check, not an assertion: the
original run stored `n_near` and `n_near_blocked` per decision, both computed
with the correct predicate at full precision, and every recomputed label matches.

One decision (`12m hk_elevator shipped mid1 stock`, goal 6) appears to flip from
B to A when the test is run on the candidate `geo_m` values as stored, because
those are rounded to two decimals and this candidate's route length rounds to
exactly 6.00 while its raw value is fractionally above 6.0 (the original run
recorded `n_near = 0` for it). It is a display-rounding artefact, not a
reclassification. It would not move the split in any case, since A and B are
both on the fixable side.

The reason the split is immune is structural. Class N is tested before class A:
a decision whose goal taken is within R is labelled N and never reaches the
class-A test. The buggy field can only differ from the correct one by reporting
`d_geo_chosen_m`, and any decision for which `d_geo_chosen_m <= R` is already N.
So the corrupted value could never be read by the classifier. The classifier
used `nearest_other_geo_m`, the field that was already right, and the corrupted
field was only ever read by the sensitivity table.

## 5. Corrected subgroup splits

Real crossings only (class N removed), from the corrected data. These are
unchanged from the published values, for the reason given in section 4.

| scope | arm | real | A | A-blocked | B | C | fixable | legitimate |
|---|---|---|---|---|---|---|---|---|
| config `shipped` | `stock` | 31 | 13 | 0 | 4 | 14 | 17 (55 %) | 14 (45 %) |
| config `shipped` | `stock+M4.3` | 15 | 0 | 0 | 2 | 13 | 2 (13 %) | 13 (87 %) |
| config `scoring` | `stock` | 28 | 11 | 0 | 2 | 15 | 13 (46 %) | 15 (54 %) |
| config `scoring` | `stock+M4.3` | 18 | 0 | 0 | 3 | 15 | 3 (17 %) | 15 (83 %) |
| range 4 m | `stock` | 20 | 9 | 0 | 4 | 7 | 13 (65 %) | 7 (35 %) |
| range 4 m | `stock+M4.3` | 11 | 0 | 0 | 2 | 9 | 2 (18 %) | 9 (82 %) |
| range 12 m | `stock` | 39 | 15 | 0 | 2 | 22 | 17 (44 %) | 22 (56 %) |
| range 12 m | `stock+M4.3` | 22 | 0 | 0 | 3 | 19 | 3 (14 %) | 19 (86 %) |
| **pooled** | **`stock`** | **59** | **24** | **0** | **6** | **29** | **30 (51 %)** | **29 (49 %)** |
| **pooled** | **`stock+M4.3`** | **33** | **0** | **0** | **5** | **28** | **5 (15 %)** | **28 (85 %)** |

The spread the audit pointed at is real and it is wide. The pooled 51/49 is an
average over subgroups running from 44 % fixable (12 m range) to 65 % fixable
(4 m range), on 20 and 39 crossings respectively. `shipped` versus `scoring`
splits less far apart, 55 % versus 46 %. The remedy arm is much flatter, 13 % to
18 % fixable across all four subgroups, which is the one place where quoting a
pooled figure is defensible.

## 6. The split as a function of the vicinity radius

This is what "robust to the vicinity radius" has to mean, and it is worth
computing directly rather than reading off the vicinity counts. It is derivable
from the stored data without re-simulation, because the B/C test uses
`REGION_M = 6 m`, a constant separate from `R_near`, and so does not move with R.

| arm | R | real | A | B | C | fixable | legitimate |
|---|---|---|---|---|---|---|---|
| `stock` | 4 m | 59 | 10 | 6 | 43 | 16 (27 %) | 43 (73 %) |
| `stock` | **6 m** | **59** | **24** | **6** | **29** | **30 (51 %)** | **29 (49 %)** |
| `stock` | 9 m | 58 | 35 | 0 | 23 | 35 (60 %) | 23 (40 %) |
| `stock` | 12 m | 57 | 38 | 0 | 19 | 38 (67 %) | 19 (33 %) |
| `stock+M4.3` | 4 m | 33 | 0 | 5 | 28 | 5 (15 %) | 28 (85 %) |
| `stock+M4.3` | **6 m** | **33** | **0** | **5** | **28** | **5 (15 %)** | **28 (85 %)** |
| `stock+M4.3` | 9 m | 33 | 0 | 5 | 28 | 5 (15 %) | 28 (85 %) |
| `stock+M4.3` | 12 m | 33 | 0 | 5 | 28 | 5 (15 %) | 28 (85 %) |

A-blocked is 0 in every cell. The R = 6 m rows use the full-precision labels;
the boundary decision of section 4 shifts one crossing between A and B at that
radius and does not change any fixable or legitimate figure.

**This table is identical whether it is computed with the buggy field or the
corrected one**, for the structural reason in section 4. The bug is therefore
not what makes the robustness claim fail. What makes it fail is the table
itself: the `stock` split moves from 27/73 at R = 4 m to 67/33 at R = 12 m. It
passes through 51/49 at the declared R = 6 m, and the number is an artefact of
that choice as much as of the data.

## 7. What changes and what survives

**Claims that change:**

1. Section 2, sixth sentence: "Widening the vicinity from 6 m to 12 m finds an
   available candidate at 41 of the 60 stock swings and at 1 of the 34 remedy
   swings" becomes **40 of 60** and **0 of 34**.
2. The section 3 table `stock` 26 / 37 / 41 becomes **25 / 36 / 40**, and
   `stock+M4.3` 1 / 1 / 1 becomes **0 / 0 / 0**.
3. Caveat 3: "at R = 12 m the remedy still has an available candidate at 1 of 34
   swings" becomes **0 of 34**. The correction strengthens this claim rather
   than weakening it: the remedy's single apparent near-miss was the goal it had
   just taken, so there is no radius up to 12 m at which any residual crossing
   under `stock+M4.3` had an alternative available.

**Claims that do not survive, and did not before the correction either:**

4. Any reading of the section 3 table as showing that the fixable/legitimate
   split is robust to the vicinity radius. The `stock` split runs 27/73, 51/49,
   60/40, 67/33 across R = 4 / 6 / 9 / 12 m. The 51/49 headline is specific to
   R = 6 m and should be quoted with that radius attached. The `stock+M4.3`
   15/85 IS flat across all four radii, and the caveat's actual sentence, which
   is about the remedy arm, holds.
5. Quoting the pooled 51/49 without its subgroup spread. Section 5 gives 65 %
   fixable at the 4 m range against 44 % at 12 m.

**Claims that survive intact:**

6. The headline pooled split, `stock` 30 fixable (51 %) / 29 legitimate (49 %)
   and `stock+M4.3` 5 (15 %) / 28 (85 %), at R = 6 m. Verified above, 0 class
   changes.
7. Class A going 24 to 0 under the remedy. Untouched: it is computed from
   `nearest_other_geo_m`, which was always correct.
8. Every subgroup split in section 5.
9. The isolation figures of section 2 and of the "isolation number" paragraph.
   These were computed from the correct field, and the recomputation reproduces
   them: over the 33 residual crossings the nearest available candidate other
   than the one taken is at a **median 27.66 m, minimum 12.02 m**, with **one
   decision having no other candidate at all** (published: median 27.7 m,
   minimum 12.0 m, one decision). Under `stock` over 59 real crossings it is a
   **median 6.65 m, minimum 1.70 m** (published: 6.7 m and 1.7 m). For the
   record, the buggy field would have given `stock` a median of 7.19 m and
   `stock+M4.3` a median of 27.33 m with no null decisions, so the report did
   not use it here.
10. A-blocked at 0 in both arms, the paired class-C result, the info-gain
    finding. None of them reads the corrupted field.

## 8. Method and limits

Recomputed from `resid_classification.json`, which stores every candidate of
every traced decision with `geo_m`, `suppressed` and `is_chosen`. Both fields
were recomputed from those lists and reproduce the values the buggy run stored
with **0 mismatches on 94 decisions**, which is what licenses the rest of the
arithmetic. No simulation, no re-tracing, no re-planning.

Limits:

- `geo_m` is stored rounded to two decimals, so a candidate whose raw route
  length sits within 0.005 m of R can fall on the wrong side of a threshold. One
  decision does, at R = 6 m only, and it is resolved above using the
  full-precision counts the original run stored. At R = 4, 9 and 12 m no
  decision sits on a boundary.
- The section 6 table assumes the B/C test does not move with R. That is true of
  this code: `REGION_M` is a separate constant fixed at 6 m. A reader who wants
  `REGION_M` to track `R_near` would need a re-run, and that re-run would not be
  a re-simulation either, only a re-trace.
- Everything else in `rapport_resid.md` is out of scope here. The sample is
  still 64 crossing-bearing paired cells on four floors, one recording each.

## 9. Files

- `diagnose_resid.py` predicate corrected, and the printed sensitivity table now
  covers R = 4 / 6 / 9 / 12 m per range and pooled. Not re-run, because re-running
  it overwrites `resid_classification.json`.
- `recompute_sensitivity.py` the recomputation, `recompute_sensitivity.out` its
  full output.
- `resid_classification_v2.json` corrected records plus
  `nearest_other_available_geo_m_buggy` per decision, and the new
  `correction`, `vicinity_sensitivity_v2`,
  `vicinity_sensitivity_real_crossings_v2`, `subgroup_splits_v2`,
  `split_by_radius_v2` and `isolation_v2` blocks.
- `resid_classification.json` untouched, as published.
- `rapport_resid.md` untouched. The three edits it needs are listed in section 7.

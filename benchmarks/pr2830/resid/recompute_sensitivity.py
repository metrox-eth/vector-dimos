#!/usr/bin/env python3
"""Recompute the vicinity fields and the radius sensitivity table from the
STORED per-decision candidate lists of resid_classification.json.

No simulation and no re-tracing: every candidate of every traced decision is
already stored with geo_m, suppressed and is_chosen, which is all the two
fields need.

The bug being corrected (diagnose_resid.py, the second predicate of the pair at
lines 191-197): nearest_other_available_geo_m excluded only the SUPPRESSED
candidates, so the chosen goal was allowed to be its own nearest alternative.
nearest_other_geo_m, the first predicate, excluded the chosen goal as well and
was already right.

Writes resid_classification_v2.json. The original file is not touched.
"""
from __future__ import annotations

import json
import os
import statistics
from collections import Counter

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "resid_classification.json")
DST = os.path.join(HERE, "resid_classification_v2.json")

R_NEAR_M = 6.0
RADII = (4.0, 6.0, 9.0, 12.0)
ARMS = ("stock", "stock+M4.3")
RANGES = ("4m", "12m")
CONFIGS = ("shipped", "scoring")


def nearest(cands, pred):
    v = [c["geo_m"] for c in cands if pred(c) and c["geo_m"] is not None]
    return round(min(v), 2) if v else None


def le(value, R):
    return value is not None and value <= R


def main():
    with open(SRC) as fh:
        d = json.load(fh)
    swings = d["swings"]

    # ---- 1. recompute both fields, keep the buggy one for the audit trail ---
    mismatch_correct = mismatch_buggy = 0
    for s in swings:
        cands = s["cands"]
        correct = nearest(cands, lambda c: not c["is_chosen"] and not c["suppressed"])
        buggy = nearest(cands, lambda c: not c["suppressed"])

        # integrity: the recomputation must reproduce what the buggy run stored
        if s["nearest_other_geo_m"] != correct:
            mismatch_correct += 1
        if s["nearest_other_available_geo_m"] != buggy:
            mismatch_buggy += 1

        s["nearest_other_available_geo_m_buggy"] = s["nearest_other_available_geo_m"]
        s["nearest_other_geo_m"] = correct
        s["nearest_other_available_geo_m"] = correct

    print(f"recomputed {len(swings)} traced decisions from the stored candidate "
          f"lists")
    print(f"  integrity check vs the values the buggy run stored: "
          f"nearest_other_geo_m {mismatch_correct} mismatches, "
          f"nearest_other_available_geo_m {mismatch_buggy} mismatches")

    # how many decisions were actually corrupted by the bug
    changed = [s for s in swings
               if s["nearest_other_available_geo_m_buggy"]
               != s["nearest_other_available_geo_m"]]
    self_counted = [s for s in swings
                    if s["nearest_other_available_geo_m_buggy"] is not None
                    and s["d_geo_chosen_m"] is not None
                    and abs(s["nearest_other_available_geo_m_buggy"]
                            - s["d_geo_chosen_m"]) < 1e-9]
    print(f"  decisions whose value changes: {len(changed)} of {len(swings)}; "
          f"of those, {len(self_counted)} had the CHOSEN goal as their own "
          f"'nearest alternative'")

    # ---- 2. radius sensitivity, old vs corrected ---------------------------
    def subset(arm, rng=None, cfg=None):
        return [s for s in swings if s["arm"] == arm
                and (rng is None or s["range"] == rng)
                and (cfg is None or s["config"] == cfg)]

    sens = {}
    print("\nvicinity sensitivity: traced swings with an AVAILABLE candidate "
          "other than the one taken within R metres of route length")
    hdr = (f"{'scope':10s} {'arm':11s} {'n':>4s} | " +
           " ".join(f"{'R=' + format(R, 'g'):>16s}" for R in RADII))
    print(hdr)
    print(f"{'':10s} {'':11s} {'':4s} | " +
          " ".join(f"{'old -> corrected':>16s}" for R in RADII))
    print("-" * len(hdr))
    for scope, rng in (("4m", "4m"), ("12m", "12m"), ("pooled", None)):
        for arm in ARMS:
            sub = subset(arm, rng)
            if not sub:
                continue
            row = {"n": len(sub)}
            cells = []
            for R in RADII:
                old = sum(1 for s in sub
                          if le(s["nearest_other_available_geo_m_buggy"], R))
                new = sum(1 for s in sub
                          if le(s["nearest_other_available_geo_m"], R))
                row[f"R{R:g}"] = {"old": old, "corrected": new,
                                  "delta": new - old, "n": len(sub)}
                cells.append(f"{old:6d} -> {new:<7d}")
            sens[f"{scope}|{arm}"] = row
            print(f"{scope:10s} {arm:11s} {len(sub):4d} | " + " ".join(
                f"{c:>16s}" for c in cells))

    # same table with the class-N decisions taken out of the denominator, so a
    # reader can see the two ways of counting do not disagree here
    print("\nsame, over the REAL crossings only (class N removed)")
    sens_real = {}
    for scope, rng in (("4m", "4m"), ("12m", "12m"), ("pooled", None)):
        for arm in ARMS:
            sub = [s for s in subset(arm, rng) if s["cls"] != "N"]
            if not sub:
                continue
            row = {"n": len(sub)}
            cells = []
            for R in RADII:
                old = sum(1 for s in sub
                          if le(s["nearest_other_available_geo_m_buggy"], R))
                new = sum(1 for s in sub
                          if le(s["nearest_other_available_geo_m"], R))
                row[f"R{R:g}"] = {"old": old, "corrected": new,
                                  "delta": new - old, "n": len(sub)}
                cells.append(f"{old:6d} -> {new:<7d}")
            sens_real[f"{scope}|{arm}"] = row
            print(f"{scope:10s} {arm:11s} {len(sub):4d} | " + " ".join(
                f"{c:>16s}" for c in cells))

    # ---- 3. headline split at R = 6 recomputed from the CORRECTED fields ----
    # class rule of diagnose_resid.classify_run, re-expressed on stored fields:
    #   N          the goal taken is itself within R of route length
    #   A          an available candidate other than the one taken is within R
    #   A-blocked  a candidate within R exists but every one is suppressed
    #   B / C      nothing within R; B if the region's cluster came back
    # The B/C test uses REGION_M = 6 m, a separate constant from R_near, so it
    # does not move with R and the split is derivable at every R below.
    # CAVEAT: geo_m is stored rounded to 2 decimals, so a candidate whose raw
    # route length sits within 0.005 m of R can fall on the wrong side of the
    # test. Those decisions are counted and listed rather than glossed over.
    EPS = 0.005

    def nearest_blocked(s):
        return nearest(s["cands"], lambda c: c["suppressed"])

    def cls_at(s, R):
        if le(s["d_geo_chosen_m"], R):
            return "N"
        if le(s["nearest_other_available_geo_m"], R):
            return "A"
        if le(nearest_blocked(s), R):
            return "A-blocked"
        return "B" if s.get("n_returned", 0) else "C"

    def boundary(s, R):
        vals = [s["d_geo_chosen_m"], s["nearest_other_available_geo_m"],
                nearest_blocked(s)]
        return any(v is not None and abs(v - R) <= EPS for v in vals)

    disagree = []
    for s in swings:
        c = cls_at(s, R_NEAR_M)
        if c != s["cls"]:
            disagree.append({"cell": [s["range"], s["map"], s["config"],
                                      s["start"], s["arm"], s["goal_index"]],
                             "stored": s["cls"], "recomputed": c,
                             "on_rounding_boundary": boundary(s, R_NEAR_M),
                             "n_near_raw_precision": s["n_near"]})
    print(f"\ncross-check of the headline split at R = {R_NEAR_M:g} m, "
          f"recomputed from the corrected fields against the stored labels:")
    print(f"  {len(disagree)} of {len(swings)} decisions differ")
    for x in disagree:
        print("   " + json.dumps(x))
    # authoritative check at R = 6: n_near / n_near_blocked were computed by the
    # original run at full precision with the CORRECT predicate (chosen and
    # suppressed both excluded), so they settle every boundary case.
    hard = []
    for s in swings:
        if le(s["d_geo_chosen_m"], R_NEAR_M):
            c = "N"
        elif s["n_near"] > 0:
            c = "A"
        elif s["n_near_blocked"] > 0:
            c = "A-blocked"
        else:
            c = "B" if s.get("n_returned", 0) else "C"
        if c != s["cls"]:
            hard.append([s["range"], s["map"], s["config"], s["start"],
                         s["arm"], s["goal_index"], s["cls"], c])
    print(f"  using the full-precision n_near / n_near_blocked the original run "
          f"stored (same corrected predicate): {len(hard)} decisions differ")
    for x in hard:
        print("   DISAGREEMENT " + repr(x))

    def split(sub):
        c = Counter(x["cls"] for x in sub)
        real = len(sub) - c["N"]
        fix = c["A"] + c["A-blocked"] + c["B"]
        return {"traced": len(sub), "N": c["N"], "real": real, "A": c["A"],
                "A-blocked": c["A-blocked"], "B": c["B"], "C": c["C"],
                "fixable": fix, "legit": c["C"],
                "fixable_pct": (100.0 * fix / real) if real else None,
                "legit_pct": (100.0 * c["C"] / real) if real else None}

    # relabel on the recomputed class so the subgroup tables are the corrected ones
    for s in swings:
        s["cls_recomputed_R6"] = cls_at(s, R_NEAR_M)

    subgroups = {}
    print("\nsubgroup splits from the corrected data, real crossings only "
          "(class N removed)")
    hdr2 = (f"{'scope':16s} {'arm':11s} {'real':>5s} {'A':>4s} {'Abl':>4s} "
            f"{'B':>4s} {'C':>4s} | {'fixable':>14s} {'legitimate':>14s}")
    print(hdr2)
    print("-" * len(hdr2))
    scopes = ([(f"config {c}", lambda s, c=c: s["config"] == c) for c in CONFIGS]
              + [(f"range {r}", lambda s, r=r: s["range"] == r) for r in RANGES]
              + [("pooled", lambda s: True)])
    for name, sel in scopes:
        for arm in ARMS:
            sub = [s for s in swings if s["arm"] == arm and sel(s)]
            if not sub:
                continue
            v = split(sub)
            subgroups[f"{name}|{arm}"] = v
            fp = "-" if v["fixable_pct"] is None else f"{v['fixable_pct']:.0f} %"
            lp = "-" if v["legit_pct"] is None else f"{v['legit_pct']:.0f} %"
            print(f"{name:16s} {arm:11s} {v['real']:5d} {v['A']:4d} "
                  f"{v['A-blocked']:4d} {v['B']:4d} {v['C']:4d} | "
                  f"{v['fixable']:4d} ({fp:>5s}) {v['legit']:6d} ({lp:>5s})")

    # ---- 3b. the fixable/legitimate split AS A FUNCTION OF R ---------------
    # This is what "robust to the vicinity radius" has to mean. Derivable from
    # the stored data because B/C does not depend on R (REGION_M is fixed).
    print("\nfixable / legitimate split as a function of the vicinity radius R, "
          "old field -> corrected field")
    hdr4 = (f"{'arm':11s} {'R':>4s} | {'real':>9s} {'A':>9s} {'B':>7s} "
            f"{'C':>9s} | {'fixable %':>19s} {'legit %':>19s}  bnd")
    print(hdr4)
    print("-" * len(hdr4))
    split_by_R = {}
    for arm in ARMS:
        sub = [s for s in swings if s["arm"] == arm]
        for R in RADII:
            row = {}
            for key, label in (("nearest_other_available_geo_m_buggy", "old"),
                               ("nearest_other_available_geo_m", "corrected")):
                c = Counter()
                for s in sub:
                    if le(s["d_geo_chosen_m"], R):
                        k = "N"
                    elif le(s[key], R):
                        k = "A"
                    elif le(nearest_blocked(s), R):
                        k = "A-blocked"
                    else:
                        k = "B" if s.get("n_returned", 0) else "C"
                    c[k] += 1
                real = len(sub) - c["N"]
                fix = c["A"] + c["A-blocked"] + c["B"]
                row[label] = {"traced": len(sub), "N": c["N"], "real": real,
                              "A": c["A"], "A-blocked": c["A-blocked"],
                              "B": c["B"], "C": c["C"], "fixable": fix,
                              "legit": c["C"],
                              "fixable_pct": (100.0 * fix / real) if real else None,
                              "legit_pct": (100.0 * c["C"] / real) if real else None}
            nb = sum(1 for s in sub if boundary(s, R))
            row["decisions_on_rounding_boundary"] = nb
            split_by_R[f"{arm}|R{R:g}"] = row
            o, n = row["old"], row["corrected"]
            print(f"{arm:11s} {R:4.0f} | {o['real']:4d}->{n['real']:<4d} "
                  f"{o['A']:4d}->{n['A']:<4d} {o['B']:3d}->{n['B']:<3d} "
                  f"{o['C']:4d}->{n['C']:<4d} | "
                  f"{(o['fixable_pct'] or 0):5.0f} -> {(n['fixable_pct'] or 0):<5.0f}"
                  f"{'':7s}"
                  f"{(o['legit_pct'] or 0):5.0f} -> {(n['legit_pct'] or 0):<5.0f}"
                  f"{'':4s}{nb:3d}")

    # ---- 4. the isolation number, old vs corrected -------------------------
    print("\nisolation: nearest AVAILABLE candidate other than the one taken, "
          "over the REAL crossings only (class N removed), as the report quotes it")
    iso = {}
    for scope, sel in (("real_crossings", lambda s: s["cls"] != "N"),
                       ("all_traced", lambda s: True)):
        for arm in ARMS:
            for key, label in (("nearest_other_available_geo_m_buggy", "old"),
                               ("nearest_other_available_geo_m", "corrected")):
                pool = [s for s in swings if s["arm"] == arm and sel(s)]
                v = sorted(s[key] for s in pool if s[key] is not None)
                n_none = sum(1 for s in pool if s[key] is None)
                iso[f"{scope}|{arm}|{label}"] = {
                    "n_decisions": len(pool), "n_with_value": len(v),
                    "n_no_other_candidate": n_none,
                    "median": round(statistics.median(v), 2) if v else None,
                    "min": v[0] if v else None, "max": v[-1] if v else None}
                if scope == "real_crossings":
                    print(f"  {arm:11s} {label:9s} n={len(pool):3d} median "
                          f"{(statistics.median(v) if v else float('nan')):6.2f} m  "
                          f"min {(v[0] if v else float('nan')):6.2f} m  "
                          f"max {(v[-1] if v else float('nan')):6.2f} m  "
                          f"(no other candidate at all: {n_none})")

    # ---- 5. write v2 -------------------------------------------------------
    d["correction"] = {
        "what": "nearest_other_available_geo_m now excludes the chosen "
                "candidate as well as the suppressed ones; the pre-correction "
                "value is kept per decision under "
                "nearest_other_available_geo_m_buggy",
        "source": "recomputed from the stored per-decision candidate lists "
                  "(geo_m / suppressed / is_chosen); no simulation was re-run",
        "decisions_recomputed": len(swings),
        "decisions_changed": len(changed),
        "decisions_where_chosen_counted_as_its_own_alternative":
            len(self_counted),
        "integrity_mismatch_nearest_other_geo_m": mismatch_correct,
        "integrity_mismatch_nearest_other_available_geo_m": mismatch_buggy,
        "headline_split_R6_class_changes_on_rounded_geo_m": len(disagree),
        "headline_split_R6_class_changes_on_rounded_geo_m_note":
            "artefact of geo_m being stored rounded to 2 decimals, not a "
            "reclassification; see the full-precision figure below",
        "radii_m": list(RADII),
    }
    d["correction"]["headline_split_R6_class_changes_full_precision"] = len(hard)
    d["correction"]["headline_split_R6_moved"] = bool(hard)
    d["vicinity_sensitivity_v2"] = sens
    d["vicinity_sensitivity_real_crossings_v2"] = sens_real
    d["subgroup_splits_v2"] = subgroups
    d["split_by_radius_v2"] = split_by_R
    d["isolation_v2"] = iso
    with open(DST, "w") as fh:
        json.dump(d, fh, indent=1, default=str)
    print(f"\n-> {DST}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

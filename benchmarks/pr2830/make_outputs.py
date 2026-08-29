#!/usr/bin/env python3
"""results.json -> resultats.csv + the aggregate numbers the report quotes.

    make_outputs.py results.json
"""

from __future__ import annotations

import csv
import json
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

COLUMNS = [
    ("map", "carte"),
    ("arm", "strategie"),
    ("config", "config"),
    ("start", "depart"),
    ("n_goals", "n_buts"),
    ("goals_reached", "buts_atteints"),
    ("goals_no_path", "buts_sans_chemin"),
    ("dead_tail_goals", "buts_apres_dernier_atteint"),
    ("d_goal_to_goal_median_m", "d_mediane_but_a_but_m"),
    ("d_goal_to_goal_max_m", "d_max_but_a_but_m"),
    ("d_robot_to_goal_median_m", "d_mediane_robot_a_but_m"),
    ("d_robot_to_goal_max_m", "d_max_robot_a_but_m"),
    ("d_robot_to_goal_median_reached_m", "d_mediane_robot_a_but_atteints_m"),
    ("path_m", "chemin_total_m"),
    ("path_to_50pct_m", "chemin_a_50pct_m"),
    ("path_to_80pct_m", "chemin_a_80pct_m"),
    ("path_to_90pct_m", "chemin_a_90pct_m"),
    ("area_m2", "couverture_m2"),
    ("ceiling_m2", "plafond_visible_m2"),
    ("coverage_pct", "couverture_pct"),
    ("reversals", "inversions_de_sens"),
    ("impacts", "contacts_corps"),
    ("sim_s", "duree_sim_s"),
    ("decide_ms_mean", "decision_ms_moy"),
    ("end_reason", "fin_de_run"),
]


def fmt(v):
    if isinstance(v, float):
        if math.isnan(v):
            return ""
        return f"{v:.3f}"
    if v is None:
        return ""
    return v


def write_csv(rows, path):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([fr for _, fr in COLUMNS])
        for s in rows:
            w.writerow([fmt(s.get(k)) for k, _ in COLUMNS])
    print(f"-> {path}  ({len(rows)} lignes)")


# A start where NEITHER arm could drive anywhere measures the flat's geometry,
# not the strategies. Both arms are excluded together, by a rule that cannot
# prefer one of them, and the excluded starts are listed in the report.
DEGENERATE_PATH_M = 1.0


def degenerate_starts(rows):
    out = set()
    for cfg in {r["config"] for r in rows}:
        keys = {(r["map"], r["start"]) for r in rows if r["config"] == cfg}
        for m, st in keys:
            paths = [r["path_m"] for r in rows
                     if (r["map"], r["start"], r["config"]) == (m, st, cfg)]
            if paths and max(paths) < DEGENERATE_PATH_M:
                out.add((m, st, cfg))
    return out


def paired(rows, config, key):
    """Per (map, start) pairs of (stock, pr) for `key`, both present and finite."""
    dead = degenerate_starts(rows)
    idx = {(r["map"], r["start"], r["arm"]): r for r in rows
           if r["config"] == config and (r["map"], r["start"], config) not in dead}
    out = []
    for (m, st, arm), r in idx.items():
        if arm != "stock":
            continue
        p = idx.get((m, st, "pr2830"))
        if p is None:
            continue
        a, b = r.get(key), p.get(key)
        if a is None or b is None:
            continue
        if isinstance(a, float) and math.isnan(a):
            continue
        if isinstance(b, float) and math.isnan(b):
            continue
        out.append((m, st, a, b))
    return out


def sign_test(pairs):
    """How often the PR is strictly lower, higher, equal. No distributional claim
    is made on 8-10 pairs; this is a count, printed as a count."""
    lower = sum(1 for *_, a, b in pairs if b < a)
    higher = sum(1 for *_, a, b in pairs if b > a)
    same = len(pairs) - lower - higher
    return lower, higher, same


def block(rows, config, key, label, lower_is_better=True):
    pairs = paired(rows, config, key)
    if not pairs:
        return f"{label:<44} (aucune paire)"
    stock = [a for *_, a, b in pairs]
    pr = [b for *_, a, b in pairs]
    lo, hi, eq = sign_test(pairs)
    ms, mp = statistics.median(stock), statistics.median(pr)
    delta = 100 * (mp - ms) / ms if ms else float("nan")
    good = lo if lower_is_better else hi
    return (f"{label:<44} stock {ms:7.2f}   #2830 {mp:7.2f}   "
            f"{delta:+6.1f} %   #2830 meilleur sur {good}/{len(pairs)} paires")


def pooled_goal_distances(data, config, dead):
    out = {}
    for arm in ("stock", "pr2830"):
        d = []
        for r in data["results"]:
            s = r["summary"]
            if s["config"] != config or s["arm"] != arm:
                continue
            if (s["map"], s["start"], config) in dead:
                continue
            d += [g["d_robot"] for g in r["goals"]]
        out[arm] = d
    return out


def main(argv=None):
    args = argv or sys.argv[1:]
    path = args[0] if args else os.path.join(HERE, "results.json")
    with open(path) as fh:
        data = json.load(fh)
    rows = [r["summary"] for r in data["results"]]
    write_csv(rows, os.path.join(HERE, "resultats.csv"))

    dead = degenerate_starts(rows)
    lines = []
    if dead:
        lines.append(f"departs ecartes (AUCUN des deux bras n'a roule "
                     f"{DEGENERATE_PATH_M} m - la geometrie du lieu, pas la strategie) :")
        for m, st, cfg in sorted(dead):
            lines.append(f"  {m:16s} {st:8s} config {cfg}")
    for config in ("shipped", "scoring"):
        n = len([r for r in rows if r["config"] == config])
        if not n:
            continue
        lines.append("")
        lines.append(f"=== config « {config} » ({n} runs) " + "=" * 40)
        lines.append(f"{'metrique':<44} {'mediane des runs':>32}   {'ecart':>7}   comparaisons appariees")
        for key, label, lib in [
            ("d_robot_to_goal_median_m", "d mediane robot->but (m)", True),
            ("d_robot_to_goal_max_m", "d max robot->but (m)", True),
            ("d_goal_to_goal_median_m", "d mediane but->but (m)", True),
            ("d_goal_to_goal_max_m", "d max but->but (m)", True),
            ("path_to_50pct_m", "chemin pour 50 % du plafond (m)", True),
            ("path_to_80pct_m", "chemin pour 80 % du plafond (m)", True),
            ("path_to_90pct_m", "chemin pour 90 % du plafond (m)", True),
            ("path_m", "chemin total (m)", True),
            ("coverage_pct", "couverture finale (%)", False),
            ("n_goals", "buts publies", True),
            ("goals_reached", "buts atteints", False),
            ("dead_tail_goals", "buts apres le dernier atteint", True),
            ("reversals", "inversions de sens", True),
            ("decide_ms_mean", "decision (ms)", True),
        ]:
            lines.append(block(rows, config, key, label, lib))

        pool = pooled_goal_distances(data, config, dead)
        for arm, d in pool.items():
            if not d:
                continue
            d = sorted(d)
            lines.append(f"  tous les buts confondus, {arm:7s}: n={len(d):4d}  "
                         f"med {statistics.median(d):5.2f} m  "
                         f"p90 {d[int(0.9 * (len(d) - 1))]:5.2f} m  max {max(d):5.2f} m  "
                         f"part > 5 m: {100 * sum(1 for v in d if v > 5) / len(d):4.1f} %")

    text = "\n".join(lines)
    print(text)
    with open(os.path.join(HERE, "agregats.txt"), "w") as fh:
        fh.write(text + "\n")
    print(f"\n-> {os.path.join(HERE, 'agregats.txt')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

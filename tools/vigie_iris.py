#!/usr/bin/env python3
"""Iris's vigil on the organ panel - runs on the RIG, prints ONE line per real
state transition and nothing else.

Owner's rule (27/08 15h57): the panel is Iris's instrument too - she must know
what she is doing. Owner's correction (27/08 16h15): the first vigil compared a
string containing the raw load number, so every 33->38 flutter fired an event
and flooded the conversation. This one fingerprints only STATES: organ
alive/dead, stack, map frame, watchdog, viewer, load BUCKET (with hysteresis so
the boundary cannot flap). Each poll carries ?watcher=iris so the panel's
"Software > Monitoring" row can prove somebody is actually watching.
"""
import json
import time
import urllib.request

URL = "http://192.168.0.56:8900/metrics?watcher=iris"
PERIOD_S = 15
# hysteresis: enter a bucket at its floor, leave it under the leave value.
# Elargie 27/08 21h04: la respiration checkpoint (5<->8) faisait japper la
# vigie toutes les dix minutes - le pouls normal ne doit produire AUCUN event.
BUCKETS = [(14.0, 10.0, "charge HAUTE"), (9.0, 6.0, "charge elevee"), (0.0, 0.0, "charge ok")]


def bucket(load: float, prev: str) -> str:
    for enter, leave, name in BUCKETS:
        if load >= enter or (prev == name and load >= leave):
            return name
    return BUCKETS[-1][2]


_mem_prev = ["ok"]


def _mem_bucket(mem: float) -> str:
    prev = _mem_prev[0]
    if mem > 92 or (prev == "CRITIQUE" and mem > 89):
        b = "CRITIQUE"
    elif mem > 87 or (prev in ("haute", "CRITIQUE") and mem > 83):
        b = "haute"
    else:
        b = "ok"
    _mem_prev[0] = b
    return b


def snapshot(prev_bucket: str):
    try:
        with urllib.request.urlopen(URL, timeout=8) as r:
            d = json.load(r)
    except Exception:
        return {"panneau": "INJOIGNABLE"}, None
    s = d.get("sensors", {})
    sw = s.get("software", {})
    load = d.get("load_1m", 0.0)
    mem = d.get("ram_percent") or 0.0
    state = {
        "stack": "up" if sw.get("stack_running") else "OFF",
        "frame": str(sw.get("reloc_state")),
        "garde": "armee" if sw.get("garde_vitesse") else "off",
        "viewer": "connecte" if sw.get("rerun_connected") else "OFF",
        "charge": bucket(load, prev_bucket),
        # la RAM du Jetson est UNIFIEE (le GPU y vit aussi): >92 % = risque OOM.
        # Hysteresis (27/08 22h38: la base du stack vit PILE a 85 %, la
        # frontiere jappait a chaque respiration - meme remede que la charge)
        "memoire": _mem_bucket(mem),
    }
    for k, v in sorted(s.items()):
        if isinstance(v, dict) and "alive" in v:
            state[k] = "OK" if v["alive"] else "MORT"
    return state, load


def main() -> None:
    prev: dict = {}
    prev_bucket = "charge ok"
    while True:
        cur, load = snapshot(prev_bucket)
        prev_bucket = cur.get("charge", prev_bucket)
        if prev and cur != prev:
            deltas = [f"{k}: {prev.get(k, '?')} -> {cur[k]}" for k in cur if cur[k] != prev.get(k)]
            extra = f" (load {load:.0f})" if load is not None and any(k.startswith("charge") for k in deltas) else ""
            print("; ".join(deltas) + extra, flush=True)
        elif not prev:
            print("vigie armee - etat initial: " + " ".join(f"{k}={v}" for k, v in cur.items()), flush=True)
        prev = cur
        time.sleep(PERIOD_S)


if __name__ == "__main__":
    main()

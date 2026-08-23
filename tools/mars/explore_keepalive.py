"""Keep the frontier explorer alive: re-send explore_cmd whenever it quits.

dimOS's WavefrontFrontierExplorer stops itself after 10 "no frontier" retries
or 2 rounds without information gain. On a cluttered floor that happens after
a minute, and the rover then idles until a human notices (metrox, 23/08:
"il fonctionne pendant une minute, puis t'attends neuf minutes"). This loop
tails the run's log and re-triggers exploration RETRIGGER_S after each stop,
up to MAX_RESTARTS. A human stop (stop_explore_cmd / dimos stop) ends it.
"""

import glob
import json
import os
import sys
import time

from dimos.core.transport_factory import make_transport
from dimos_lcm.std_msgs import Bool

RETRIGGER_S = 5.0
MAX_RESTARTS = int(sys.argv[1]) if len(sys.argv) > 1 else 30
STOP_MARKERS = ("Exploration complete", "Stopped autonomous frontier exploration")

log_dir = sorted(glob.glob(os.path.expanduser("~/.local/state/dimos/logs/*vector-dimos-explore*")), key=os.path.getmtime)[-1]
files = {}
cmd = make_transport("explore_cmd", Bool)
time.sleep(0.5)
restarts = 0
last_trigger = time.monotonic()
print(f"keepalive on {os.path.basename(log_dir)}, max {MAX_RESTARTS} restarts", flush=True)
while restarts < MAX_RESTARTS:
    stopped_at = None
    for path in glob.glob(log_dir + "/*.jsonl"):
        f = files.get(path)
        if f is None:
            f = files[path] = open(path)
            f.seek(0, 2)  # only what happens from now on
        for line in f:
            if any(m in line for m in STOP_MARKERS):
                try:
                    stopped_at = json.loads(line).get("timestamp", "?")
                except ValueError:
                    stopped_at = "?"
    if stopped_at and time.monotonic() - last_trigger > RETRIGGER_S:
        time.sleep(RETRIGGER_S)
        cmd.broadcast(None, Bool(True))
        restarts += 1
        last_trigger = time.monotonic()
        print(f"{time.strftime('%H:%M:%S')} explorer stopped at {stopped_at} -> explore_cmd re-sent ({restarts}/{MAX_RESTARTS})", flush=True)
    time.sleep(2.0)
print("keepalive: restart budget exhausted", flush=True)

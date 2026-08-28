"""Cold bench for tools/explore_ctl.py: the explore start/stop must ride the RUN's bus.

Rule #2: known input -> known output, here in bytes on a named channel (what a
bus actually carries). Groups:

  A. topics    - each bus's exact topic for start and stop, TRANSPORT read the
                 way the stack reads it (unset = lcm, case-insensitive)
  B. payload   - Bool(True) is the same 9 known bytes on both buses (8-byte LCM
                 fingerprint of std_msgs.Bool + 0x01), and decodes back to True
  C. the stack - those topics are the ones dimOS's own factory derives for the
                 logical channels the explorer subscribes to (needs dimOS)
  D. lcm       - a real publish lands on /explore_cmd#std_msgs.Bool byte for byte
                 (the behaviour before the zenoh switch, unchanged)
  E. zenoh     - a real publish lands on dimos/stop_explore_cmd/std_msgs.Bool in a
                 local subscriber session, byte for byte

E dials ZENOH_ENDPOINT: the subscriber here plays the rover's rendezvous peer
(tools/zenoh_rendezvous.py). End to end into a live VectorExplorer stays a rover
check.

Run:  PYTHONPATH=. .venv/bin/python3 tests/test_explore_ctl_cold.py
"""

import json
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import lcm  # noqa: E402
import zenoh  # noqa: E402
from dimos_lcm.std_msgs import Bool  # noqa: E402

import explore_ctl  # noqa: E402

try:
    from dimos.core.global_config import global_config
    from dimos.core.transport_factory import transport_topic
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic as LCMTopic
    from dimos.protocol.pubsub.impl.zenohpubsub import Topic as ZenohTopic
    HAVE_DIMOS = True
except Exception:  # noqa: BLE001 - no dimOS on a laptop
    HAVE_DIMOS = False

TRUE_BYTES = b"\x1e\xbe\xf0jP\x95\x0e>\x01"   # std_msgs.Bool fingerprint + data=True
LCM_START = "/explore_cmd#std_msgs.Bool"
LCM_STOP = "/stop_explore_cmd#std_msgs.Bool"
ZENOH_START = "dimos/explore_cmd/std_msgs.Bool"
ZENOH_STOP = "dimos/stop_explore_cmd/std_msgs.Bool"
OK = 0
KO = 0


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def set_transport(value):
    if value is None:
        os.environ.pop("TRANSPORT", None)
    else:
        os.environ["TRANSPORT"] = value


saved_transport = os.environ.get("TRANSPORT")
os.environ.pop("DIMOS_TRANSPORT", None)   # the bench drives TRANSPORT alone

try:
    print("A. topics")
    set_transport(None)
    check("TRANSPORT unset -> lcm", explore_ctl.transport() == "lcm", explore_ctl.transport())
    check(f"unset: start -> {LCM_START}", explore_ctl.topic("start") == LCM_START, explore_ctl.topic("start"))
    check(f"unset: stop  -> {LCM_STOP}", explore_ctl.topic("stop") == LCM_STOP, explore_ctl.topic("stop"))
    set_transport("lcm")
    check(f"lcm: start -> {LCM_START}", explore_ctl.topic("start") == LCM_START, explore_ctl.topic("start"))
    check(f"lcm: stop  -> {LCM_STOP}", explore_ctl.topic("stop") == LCM_STOP, explore_ctl.topic("stop"))
    set_transport("zenoh")
    check(f"zenoh: start -> {ZENOH_START}", explore_ctl.topic("start") == ZENOH_START, explore_ctl.topic("start"))
    check(f"zenoh: stop  -> {ZENOH_STOP}", explore_ctl.topic("stop") == ZENOH_STOP, explore_ctl.topic("stop"))
    set_transport("ZENOH")
    check("TRANSPORT=ZENOH -> zenoh (the config binds it case-insensitively)",
          explore_ctl.topic("stop") == ZENOH_STOP, explore_ctl.topic("stop"))

    print("B. payload")
    payload = Bool(data=True).lcm_encode()
    check(f"Bool(True) -> {TRUE_BYTES!r}", payload == TRUE_BYTES, repr(payload))
    check("decodes back to data=True", Bool.lcm_decode(payload).data is True)

    print("C. the stack's own topics" if HAVE_DIMOS else "C. the stack's own topics - SKIPPED (no dimOS)")
    if HAVE_DIMOS:
        global_config.update(transport="lcm")
        stack_lcm = {c: str(LCMTopic(transport_topic(f"/{n}"), Bool))
                     for c, n in explore_ctl.CHANNELS.items()}
        global_config.update(transport="zenoh")
        stack_zenoh = {c: ZenohTopic(transport_topic(f"/{n}"), Bool).key_expr
                       for c, n in explore_ctl.CHANNELS.items()}
        check("lcm channels match dimOS's factory",
              stack_lcm == {"start": LCM_START, "stop": LCM_STOP}, str(stack_lcm))
        check("zenoh keys match dimOS's factory",
              stack_zenoh == {"start": ZENOH_START, "stop": ZENOH_STOP}, str(stack_zenoh))

    print("D. lcm delivery")
    set_transport("lcm")
    bus = lcm.LCM()
    heard = []
    bus.subscribe(LCM_START, lambda ch, data: heard.append((ch, data)))
    used = explore_ctl.publish("start")
    deadline = time.monotonic() + 2.0
    while not heard and time.monotonic() < deadline:
        bus.handle_timeout(100)
    check(f"publish('start') says {LCM_START}", used == LCM_START, used)
    check("a subscriber on that channel got exactly the known bytes",
          heard == [(LCM_START, TRUE_BYTES)], str(heard))

    print("E. zenoh delivery")
    set_transport("zenoh")
    config = zenoh.Config()
    try:
        config.insert_json5("listen/endpoints", json.dumps([explore_ctl.ZENOH_ENDPOINT]))
        session = zenoh.open(config)   # plays the rover's rendezvous peer
        role = f"listening on {explore_ctl.ZENOH_ENDPOINT}"
    except Exception as e:  # noqa: BLE001 - port already held (a real rendezvous): scout instead
        session = zenoh.open(zenoh.Config())
        role = f"plain peer, {explore_ctl.ZENOH_ENDPOINT} busy ({e})"
    samples = []   # every explore key under dimos/**, so a publish on a NEARBY key still shows up

    def on_sample(s):
        key = str(s.key_expr)
        if "explore_cmd" in key:   # ignore a live stack's other traffic on the same bus
            samples.append((key, bytes(s.payload)))

    session.declare_subscriber("dimos/**", on_sample)
    time.sleep(0.5)   # let the subscriber declaration settle before the publisher dials in
    used = explore_ctl.publish("stop")
    deadline = time.monotonic() + 3.0
    while not samples and time.monotonic() < deadline:
        time.sleep(0.05)
    session.close()
    check(f"subscriber {role}", True)
    check(f"publish('stop') says {ZENOH_STOP}", used == ZENOH_STOP, used)
    check("the zenoh subscriber got exactly the known bytes on that key",
          samples == [(ZENOH_STOP, TRUE_BYTES)], str(samples))
    check("and it decodes to a stop command (data=True)",
          bool(samples) and Bool.lcm_decode(samples[0][1]).data is True)
finally:
    set_transport(saved_transport)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)

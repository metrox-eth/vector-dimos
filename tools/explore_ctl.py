#!/usr/bin/env python3
"""Start/stop autonomous exploration on the run's transport (usage: explore_ctl.py start|stop).

The stack builds `explore_cmd` / `stop_explore_cmd` through dimOS's transport
factory, so under TRANSPORT=zenoh nobody is subscribed on the LCM bus: the
LCM-only publish this used to do went into the void - exploration never started
(fly.sh gate 7/8 reported success anyway), and the speed watchdog's stop
(tools/garde_vitesse.py) reached nobody while the rover kept driving.

Same switch as the stack, same topics dimOS derives (transport_topic + the
zenoh Topic.key_expr: the type is a '#' suffix on LCM, a trailing key segment
on zenoh), same lcm_encode payload on both buses:

    lcm    /explore_cmd#std_msgs.Bool
    zenoh  dimos/explore_cmd/std_msgs.Bool

Importable: the speed watchdog calls publish("stop"). Other out-of-stack
publishers (tools/moss_cockpit.py: goals, soft stop, episode markers) use
publish_msg(name, msg), which lands any dimos_lcm message on the same bus with
the same naming rule.
"""
import json
import os
import sys
import time

from dimos_lcm.std_msgs import Bool

CHANNELS = {"start": "explore_cmd", "stop": "stop_explore_cmd"}
ZENOH_ENDPOINT = "tcp/127.0.0.1:7447"  # the rendezvous peer (tools/zenoh_rendezvous.py); scouting covers the rest
LINK_TIMEOUT_S = 2.0                   # bounded wait for a link: a put before one is up goes nowhere
FLUSH_S = 0.3                          # let zenoh push the sample out before the session closes


def transport() -> str:
    """The run's bus, read the way GlobalConfig reads it (TRANSPORT, case-insensitive)."""
    return (os.environ.get("TRANSPORT") or os.environ.get("DIMOS_TRANSPORT") or "lcm").strip().lower()


def channel(name: str, msg_name: str) -> str:
    """The LCM channel or zenoh key expression of stream `name` carrying `msg_name`."""
    if transport() == "zenoh":
        return f"dimos/{name}/{msg_name}"
    return f"/{name}#{msg_name}"


def topic(cmd: str) -> str:
    """The LCM channel or zenoh key expression this command lands on."""
    return channel(CHANNELS[cmd], Bool.msg_name)


def publish_msg(name: str, msg, msg_name: str = None, bus=None) -> str:
    """Publish any dimos_lcm message on stream `name` of the run's bus; returns the topic used.

    `msg_name` is the type suffix dimOS puts in the topic. It defaults to the
    message's own msg_name, but pass it for types whose dimos_lcm class carries a
    bare name: dimos_lcm's PoseStamped says "PoseStamped" while dimOS's wrapper
    (and so every stack topic) says "geometry_msgs.PoseStamped". The payload is
    the same lcm_encode bytes either way. `bus`: an already open lcm.LCM to
    publish through (a long-running caller keeps one); None opens one per call.
    """
    where = channel(name, msg_name or msg.msg_name)
    payload = msg.lcm_encode()
    if transport() == "zenoh":
        _zenoh_put(where, payload)
    else:
        if bus is None:
            import lcm  # only the active bus gets imported

            bus = lcm.LCM()
        bus.publish(where, payload)
    return where


def publish(cmd: str) -> str:
    """Publish Bool(True) for 'start'|'stop' on the run's bus; returns the topic used."""
    return publish_msg(CHANNELS[cmd], Bool(data=True))


def _zenoh_put(key: str, payload: bytes) -> None:
    """One-shot put from a throwaway peer session."""
    import zenoh

    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([ZENOH_ENDPOINT]))
    session = zenoh.open(config)
    try:
        deadline = time.monotonic() + LINK_TIMEOUT_S
        while not session.info.links() and time.monotonic() < deadline:
            time.sleep(0.05)
        session.put(key, payload)
        time.sleep(FLUSH_S)
    finally:
        session.close()


def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "start"
    if cmd not in CHANNELS:
        sys.exit("usage: explore_ctl.py start|stop")
    print(f"published {publish(cmd)} on {transport()}")


if __name__ == "__main__":
    main()

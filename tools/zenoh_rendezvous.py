"""Zenoh rendezvous peer - the robot's well-known meeting point (port 7447).

The stack's zenoh sessions listen on EPHEMERAL ports (visible with ss) and
find each other on-host via loopback multicast; a remote machine has nothing
fixed to dial. This peer holds tcp/0.0.0.0:7447 (their ROBOT_ZENOH_PORT
convention) with gossip on: the rig dials it (ROBOT_IP=<rover>), gossip hands
out every session's real endpoint, and the mesh completes both ways. Proven
pattern: the 27/08 20h30 heartbeat test crossed the LAN exactly this way.

Publishes nothing, subscribes to nothing - a pure introduction service.
"""
import time

import zenoh


def main() -> None:
    c = zenoh.Config()
    c.insert_json5("listen/endpoints", '["tcp/0.0.0.0:7447"]')
    c.insert_json5("scouting/gossip/enabled", "true")
    s = zenoh.open(c)
    print("zenoh rendezvous: tcp/0.0.0.0:7447, gossip on", flush=True)
    try:
        while True:
            time.sleep(60)
    finally:
        s.close()


if __name__ == "__main__":
    main()

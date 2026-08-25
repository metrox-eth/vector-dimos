#!/usr/bin/env python3
"""Start/stop autonomous exploration via LCM (usage: explore_ctl.py start|stop)."""
import sys
import lcm
from dimos_lcm.std_msgs import Bool

def main() -> None:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "start"
    topic = "/explore_cmd#std_msgs.Bool" if cmd == "start" else "/stop_explore_cmd#std_msgs.Bool"
    msg = Bool()
    msg.data = True
    lcm.LCM().publish(topic, msg.lcm_encode())
    print(f"published {topic}")

if __name__ == "__main__":
    main()

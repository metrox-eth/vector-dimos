"""Live bench (needs dimos + LCM multicast, i.e. the Jetson): a slow subscriber
on a command channel must never accumulate a backlog once lcm_latest is
installed. Known in: 100 twists published in 1 s to a handler that takes
100 ms each. Known out: with a 1-deep queue the handler sees ~10 messages and
at most ONE arrives after the publisher stopped; with dimOS's 10000-deep queue
it would grind through all 100 for 10 s (the 2026-08-22 failure).

    $ python tests/test_lcm_latest_jetson.py            # patched (default)
    $ python tests/test_lcm_latest_jetson.py --stock    # dimOS default, for comparison
"""
import sys, time, threading
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent.parent))
stock = "--stock" in sys.argv
if not stock:
    from vector_dimos.lcm_latest import install
    assert install(), "lcm_latest.install() found no dimos"
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Twist import Twist

TOPIC = "cmd_vel_latest_bench"           # contains "cmd_vel" -> clamped when patched
handled = []
def slow_handler(msg):
    handled.append(time.monotonic()); time.sleep(0.1)
sub = make_transport(TOPIC, Twist); sub.subscribe(slow_handler)
pub = make_transport(TOPIC, Twist)
time.sleep(0.5)
t0 = time.monotonic()
for i in range(100):
    pub.broadcast(None, Twist(linear=[0.1, 0, 0], angular=[0, 0, 0])); time.sleep(0.01)
t_stop = time.monotonic()
time.sleep(3.0)
late = [t for t in handled if t > t_stop + 0.15]
print(f"{'STOCK' if stock else 'PATCHED'}: published 100 in {t_stop-t0:.2f}s; handled {len(handled)}; "
      f"handled after the publisher stopped (+150 ms): {len(late)}; last handled {max(handled)-t_stop:+.2f}s after stop")
ok = (len(late) <= 1) if not stock else True
print("TEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

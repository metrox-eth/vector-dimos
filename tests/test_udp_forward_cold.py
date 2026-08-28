"""Cold bench: a refused send drops ONE datagram, never the relay.

P3 of the 28/08 audit (tools/udp_forward.py:29): the receive path caught OSError
but neither sendto did, and nothing supervises the process - fly.sh backgrounds
it with nohup on both sides and never looks again. One ENETUNREACH on a WiFi/AP
blip, or one ENOBUFS on a full qdisc under video load, killed the forwarder
mid-flight: the cockpit went black for the rest of the flight even after the
link recovered, with only a traceback in /tmp/udp_forward_rig.log that no gate
reads.

The relay is a plain script (no import surface), so the bench EXECUTES it with a
fake socket/select: the scripted network refuses exactly one datagram, in one
direction, and the plan is the rig-side flight of fly.sh:178 - the cockpit's
QUIC datagrams 127.0.0.1:$WT_PORT -> 192.168.0.56:$RELAY_EXT and back. Physical
units: 1200-byte datagrams in, 1200-byte datagrams out, counted and ordered.

Sections:
  A. rig -> rover: 5 datagrams, the 3rd refused ENETUNREACH. 4 arrive, in order,
     the relay is still running, and the client keeps its ONE upstream socket.
  B. rover -> rig: the reply path, the 2nd reply refused ENOBUFS. 2 of 3 arrive.
  C. what the pre-fix relay did: the same two plans against the audited loop,
     frozen below - it dies on the refused datagram, everything after is lost.
  D. no bare sendto is left outside the guard.

No rover, no real socket: the live check (a real ICMP/route failure on the rig's
WiFi) is deferred - the rover is not reachable from here.
"""
import errno
import importlib.util
import sys
import tempfile
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
RELAY = REPO_ROOT / "tools" / "udp_forward.py"

# tools/udp_forward.py as the audit found it (commit b679354), verbatim: the
# reference the fix has to beat. Frozen here so section C keeps biting once the
# fix is committed and HEAD no longer holds the bug.
PRE_FIX = '''import socket, select, sys, time

port = int(sys.argv[1]); remote = (sys.argv[2], int(sys.argv[3])); bind = sys.argv[4] if len(sys.argv) > 4 else "127.0.0.1"
lsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); lsock.bind((bind, port)); lsock.setblocking(False)
clients = {}   # client addr -> upstream socket
upstream = {}  # upstream socket -> client addr
last = {}
print(f"udp forward {bind}:{port} -> {remote[0]}:{remote[1]}", flush=True)
while True:
    rl, _, _ = select.select([lsock] + list(upstream), [], [], 5.0)
    now = time.time()
    for s in rl:
        if s is lsock:
            data, addr = lsock.recvfrom(65535)
            u = clients.get(addr)
            if u is None:
                u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); u.setblocking(False)
                clients[addr] = u; upstream[u] = addr
            u.sendto(data, remote); last[addr] = now
        else:
            try: data, _ = s.recvfrom(65535)
            except OSError: continue
            lsock.sendto(data, upstream[s]); last[upstream[s]] = now
    for addr in [a for a, t in last.items() if now - t > 120]:
        u = clients.pop(addr); upstream.pop(u, None); last.pop(addr, None); u.close()
'''

ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and bool(cond)


# The rig side of fly.sh:178, with one run's ports.
WT_PORT = 47231
ROVER = ("192.168.0.56", 45817)
CLIENT = ("127.0.0.1", 51000)
ARGV = ["udp_forward.py", str(WT_PORT), ROVER[0], str(ROVER[1]), "127.0.0.1"]
SIZE = 1200                                        # one QUIC datagram, bytes


def frame(n):
    return f"frame{n}".encode().ljust(SIZE, b"\0")


class EndOfPlan(Exception):
    """The scripted flight is over and the relay is still in its loop."""


class FakeUDP:
    """One datagram socket: the bench feeds recvfrom and records sendto."""

    def __init__(self, net):
        self.net = net; self.inbox = []; self.closed = False; self.bound = None

    def bind(self, addr): self.bound = addr
    def setblocking(self, flag): pass
    def recvfrom(self, n): return self.inbox.pop(0)
    def close(self): self.closed = True

    def sendto(self, data, dest):
        err = self.net.refuse.pop(data[:16], None)
        self.net.attempted.append((data, dest))
        if err is not None:
            raise err
        self.net.sent.append((data, dest)); return len(data)


class Net:
    def __init__(self, refuse):
        self.socks = []; self.sent = []; self.attempted = []; self.refuse = dict(refuse)

    def socket(self, family, type_):
        s = FakeUDP(self); self.socks.append(s); return s


class Plan:
    """select() drives the flight: each step wakes one socket with one datagram.

    ('up', n)   the cockpit's datagram n arrives on the bound socket
    ('down', n) the rover's reply n arrives on the client's upstream socket
    """

    def __init__(self, steps): self.steps = list(steps); self.i = 0

    def select(self, rlist, wlist, xlist, timeout=None):
        if self.i >= len(self.steps):
            raise EndOfPlan
        role, n = self.steps[self.i]; self.i += 1
        if role == "up":
            rlist[0].inbox.append((frame(n), CLIENT)); return [rlist[0]], [], []
        assert len(rlist) > 1, "no upstream socket yet"
        rlist[1].inbox.append((frame(n), ROVER)); return [rlist[1]], [], []


def fly(path, steps, refuse):
    """Run the relay at `path` through `steps`. Returns (alive, net)."""
    net = Net(refuse)
    plan = Plan(steps)
    fake_socket = types.ModuleType("socket")
    fake_socket.AF_INET = 2; fake_socket.SOCK_DGRAM = 2
    fake_socket.socket = net.socket
    fake_select = types.ModuleType("select"); fake_select.select = plan.select
    saved = {k: sys.modules.get(k) for k in ("socket", "select")}
    saved_argv = sys.argv
    sys.modules["socket"] = fake_socket; sys.modules["select"] = fake_select
    sys.argv = list(ARGV)
    alive = False
    try:
        spec = importlib.util.spec_from_file_location("relay_under_test", path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    except EndOfPlan:
        alive = True                                # still looping when the flight ended
    except OSError:
        alive = False                               # the refused send killed it
    finally:
        for k, v in saved.items():
            if v is None: sys.modules.pop(k, None)
            else: sys.modules[k] = v
        sys.argv = saved_argv
        sys.modules.pop("relay_under_test", None)
    return alive, net


UP = [("up", n) for n in (1, 2, 3, 4, 5)]
DOWN = [("up", 1), ("down", 1), ("down", 2), ("down", 3)]
REFUSE_UP = {frame(3)[:16]: OSError(errno.ENETUNREACH, "Network is unreachable")}
REFUSE_DOWN = {frame(2)[:16]: OSError(errno.ENOBUFS, "No buffer space available")}


def payloads(net, dest):
    return [d.rstrip(b"\0").decode() for d, a in net.sent if a == dest]


print(f"A. rig -> rover: 5 x {SIZE} B, the 3rd refused ENETUNREACH")
alive, net = fly(RELAY, UP, REFUSE_UP)
check(alive, "the relay is still in its loop when the flight ends")
got = payloads(net, ROVER)
check(got == ["frame1", "frame2", "frame4", "frame5"],
      f"forwarded to {ROVER[0]}:{ROVER[1]}: {got} (the refused one dropped, the rest in order)")
check(len(net.sent) == 4 and len(net.attempted) == 5,
      f"{len(net.attempted)} datagrams attempted, {len(net.sent)} = {len(net.sent) * SIZE} B delivered")
check(len(net.socks) == 2, f"one upstream socket for the client, reused ({len(net.socks) - 1})")
check(not any(s.closed for s in net.socks), "no socket torn down by the drop")

print(f"\nB. rover -> rig: 3 replies, the 2nd refused ENOBUFS")
alive, net = fly(RELAY, DOWN, REFUSE_DOWN)
check(alive, "the relay is still in its loop when the flight ends")
back = payloads(net, CLIENT)
check(back == ["frame1", "frame3"],
      f"returned to the cockpit {CLIENT[0]}:{CLIENT[1]}: {back} (2 of 3, the refused one dropped)")

print("\nC. what the audited relay did: the same two flights, pre-fix")
with tempfile.TemporaryDirectory() as tmp:
    old = Path(tmp) / "udp_forward_prefix.py"
    old.write_text(PRE_FIX)
    check(PRE_FIX.count("except OSError") == 1 and PRE_FIX.count(".sendto(") == 2,
          "the pre-fix source guards recv only: 2 sendto, 1 except")
    alive, net = fly(old, UP, REFUSE_UP)
    got = payloads(net, ROVER)
    check(not alive, "pre-fix: the relay exits on the refused datagram")
    check(got == ["frame1", "frame2"],
          f"pre-fix: {got} - datagrams 4 and 5 never leave, the cockpit is black for the rest of the flight")
    alive, net = fly(old, DOWN, REFUSE_DOWN)
    check(not alive and payloads(net, CLIENT) == ["frame1"],
          f"pre-fix reply path: {payloads(net, CLIENT)}, relay alive={alive}")

print("\nD. no bare sendto outside the guard")
src = RELAY.read_text()
bare = [l.strip() for l in src.splitlines()
        if ".sendto(" in l and "sock.sendto" not in l and "send(" not in l]
check(not bare, f"every sendto goes through send(): {bare}")
check(src.count("send(") >= 3, "the guard is used on both directions")

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)

"""Tiny UDP forwarder: BIND:PORT -> REMOTE_HOST:REMOTE_PORT, one socket per client (QUIC-safe).

Usage: udp_forward.py PORT REMOTE_HOST REMOTE_PORT [BIND=127.0.0.1]

Born ~/mars/udp_forward.py on the rover (24/08, first live RealSense feed in
the cockpit); adopted into the repo 26/08 - the cockpit video needs it on BOTH
sides, and 26/08 midday it was wired on the wrong side (bound on the rover on
the very port deno holds -> Address already in use -> a whole day of "the page
is up but the video never came"). The working recipe lives in fly.sh.
"""
import socket, select, sys, time

port = int(sys.argv[1]); remote = (sys.argv[2], int(sys.argv[3])); bind = sys.argv[4] if len(sys.argv) > 4 else "127.0.0.1"
lsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); lsock.bind((bind, port)); lsock.setblocking(False)
clients = {}   # client addr -> upstream socket
upstream = {}  # upstream socket -> client addr
last = {}
drops = 0; last_drop_log = 0.0


def send(sock, data, dest, tag):
    """A refused send costs ONE datagram, never the relay.

    Audit 28/08: only recv was guarded. A WiFi/AP blip (ENETUNREACH) or a full
    qdisc under video load (ENOBUFS) raised out of the loop, the process exited,
    and nothing restarts it - fly.sh backgrounds it with nohup and never looks
    again. The cockpit then stayed black for the rest of the flight, even after
    the link came back. Drops are counted and logged at most every 5 s, so the
    log shows a blip instead of one traceback and silence.
    """
    global drops, last_drop_log
    try:
        sock.sendto(data, dest); return True
    except OSError as e:
        drops += 1; now = time.time()
        if now - last_drop_log > 5.0:
            last_drop_log = now; print(f"drop {drops} {tag} {dest[0]}:{dest[1]}: {e}", flush=True)
        return False


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
            send(u, data, remote, "->"); last[addr] = now   # live either way: it just sent us one
        else:
            try: data, _ = s.recvfrom(65535)
            except OSError: continue
            send(lsock, data, upstream[s], "<-"); last[upstream[s]] = now
    for addr in [a for a, t in last.items() if now - t > 120]:
        u = clients.pop(addr); upstream.pop(u, None); last.pop(addr, None); u.close()

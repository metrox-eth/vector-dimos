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

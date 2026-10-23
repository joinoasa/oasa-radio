"""Helpers for scripts/serial-e2e.sh: send marked UDP packets, check what
comes out, scan captured serial bytes, read JSON log lines. Test tool only."""
import json
import os
import socket
import sys
import threading
import time

LOCAL = "127.0.0.1"
MARKER = b"OASA-E2E-MARKER"


def udp(port=0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((LOCAL, port))
    return s


def listen(sock, store, stop):
    sock.settimeout(0.2)
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(70000)
        except socket.timeout:
            continue
        store.append(data)


def traffic(args):
    """traffic N INTERVAL A_IN A_OUT B_IN B_OUT: both sides send N marked packets
    (sizes up to 200 bytes, with zeros); each must arrive exactly once, unchanged."""
    n, interval = int(args[0]), float(args[1])
    a_in, a_out, b_in, b_out = (int(x) for x in args[2:6])
    got_a, got_b = [], []
    stop = threading.Event()
    ra, rb = udp(a_out), udp(b_out)
    for sock, store in ((ra, got_a), (rb, got_b)):
        threading.Thread(target=listen, args=(sock, store, stop), daemon=True).start()
    tx = udp()
    run = os.urandom(4).hex().encode()

    def make(tag, i):
        body = MARKER + b" " + tag + b" " + run + b" %04d " % i
        pad = os.urandom(8 + (i * 7) % 150)
        return body + pad.replace(b"\x01", b"\x00")  # plenty of zero bytes

    sent_a = [make(b"A->B", i) for i in range(n)]
    sent_b = [make(b"B->A", i) for i in range(n)]
    for i in range(n):
        tx.sendto(sent_a[i], (LOCAL, a_in))
        tx.sendto(sent_b[i], (LOCAL, b_in))
        time.sleep(interval)
    time.sleep(1.5)
    stop.set()
    time.sleep(0.3)
    ok = True
    for name, sent, got in (("A->B", sent_a, got_b), ("B->A", sent_b, got_a)):
        missing = [p for p in sent if got.count(p) == 0]
        dups = [p for p in sent if got.count(p) > 1]
        foreign = [p for p in got if p not in sent]
        print(f"{name}: sent {len(sent)} delivered {len(sent) - len(missing)} "
              f"duplicates {len(dups)} foreign {len(foreign)}")
        ok = ok and not missing and not dups and not foreign
    return 0 if ok else 2


def inject(args):
    """inject PORT TEXT: one raw UDP packet straight at a bridge (the plaintext control)."""
    udp().sendto(args[1].encode(), (LOCAL, int(args[0])))
    return 0


def scan(args):
    """scan CAPTURE [MARKER]: exit 0 if the marker is in the file, 1 if not.
    Also exits 3 if the file is too small to mean anything."""
    data = open(args[0], "rb").read()
    marker = args[1].encode() if len(args) > 1 else MARKER
    print(f"{args[0]}: {len(data)} bytes, marker {'FOUND' if marker in data else 'absent'}")
    if len(data) < 500:
        return 3
    return 0 if marker in data else 1


def last_stats(path):
    stats = None
    for line in open(path):
        obj = json.loads(line)  # every line must be JSON with ts and event
        assert "ts" in obj and "event" in obj, line
        if obj["event"] == "stats":
            stats = obj
    return stats


def stat(args):
    """stat FILE KEY: a number from the last stats line (KEY like drops.bad_frame; 0 if absent)."""
    cur = last_stats(args[0])
    if cur is None:
        print("no stats line", file=sys.stderr)
        return 2
    for part in args[1].split("."):
        cur = cur.get(part, 0) if isinstance(cur, dict) else 0
    print(cur)
    return 0


def count(args):
    """count FILE EVENT: how many lines have this event."""
    print(sum(1 for line in open(args[0]) if json.loads(line)["event"] == args[1]))
    return 0


def main():
    cmd = {"traffic": traffic, "inject": inject, "scan": scan, "stat": stat, "count": count}
    sys.exit(cmd[sys.argv[1]](sys.argv[2:]))


if __name__ == "__main__":
    main()

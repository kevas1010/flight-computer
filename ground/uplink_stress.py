#!/usr/bin/env python3
"""Uplink reliability measurement: sends N PING commands through the reliable
uplink to a running flight computer and reports delivery and latency.

    python3 ground/uplink_stress.py --count 1000 --json uplink.json

Run it with scripts/netem.sh applying loss to measure delivery under loss.
PING is accepted in every flight state, so this can run alongside a flight.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from link import ReliableUplink  # noqa: E402
from protocol import Ack, DecodeError, MsgType, Opcode, decode_packet  # noqa: E402


def percentile(sorted_vals: list[float], p: float) -> float:
    idx = max(0, min(len(sorted_vals) - 1, -(-int(p * len(sorted_vals)) // 100) - 1))
    return sorted_vals[idx]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fc", default="127.0.0.1:5601")
    ap.add_argument("--count", type=int, default=1000)
    ap.add_argument("--rto", type=float, default=0.05)
    ap.add_argument("--max-attempts", type=int, default=40)
    ap.add_argument("--json", help="write results here")
    args = ap.parse_args()

    host, port = args.fc.rsplit(":", 1)
    dest = (host, int(port))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.002)
    uplink = ReliableUplink(rto=args.rto, max_attempts=args.max_attempts)
    decode_errors = 0

    start = time.monotonic()
    for _ in range(args.count):
        uplink.submit(Opcode.PING, now=time.monotonic())
    while uplink.in_flight:
        now = time.monotonic()
        for pkt in uplink.poll(now):
            sock.sendto(pkt, dest)
        try:
            data, _ = sock.recvfrom(512)
        except (socket.timeout, ConnectionRefusedError):
            continue
        try:
            header, payload = decode_packet(data)
            if header.type != MsgType.ACK:
                raise DecodeError("bad_type")
            uplink.on_ack(Ack.parse(payload), time.monotonic())
        except DecodeError:
            decode_errors += 1
    elapsed = time.monotonic() - start

    done = uplink.completed
    acked = [p for p in done if p.result is not None]
    lat = sorted(p.latency * 1e3 for p in acked)
    attempts = [p.attempts for p in done]
    hist: dict[int, int] = {}
    for a in attempts:
        hist[a] = hist.get(a, 0) + 1
    result = {
        "commands": len(done),
        "acked": len(acked),
        "failed": sum(p.failed for p in done),
        "delivery_pct": round(100.0 * len(acked) / len(done), 3),
        "packets_sent": uplink.packets_sent,
        "retransmissions": uplink.retransmissions,
        "first_try_pct": round(100.0 * hist.get(1, 0) / len(done), 2),
        "mean_attempts": round(sum(attempts) / len(attempts), 3),
        "max_attempts": max(attempts),
        "attempt_histogram": dict(sorted(hist.items())),
        "latency_ms": {
            "p50": round(percentile(lat, 50), 2),
            "p99": round(percentile(lat, 99), 2),
            "max": round(lat[-1], 2),
        }
        if lat
        else None,
        "stray_acks": uplink.stray_acks,
        "ack_decode_errors": decode_errors,
        "elapsed_s": round(elapsed, 2),
    }
    print(json.dumps(result, indent=2))
    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=2) + "\n")
    return 0 if result["failed"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())

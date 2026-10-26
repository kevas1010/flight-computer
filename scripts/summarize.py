#!/usr/bin/env python3
"""Turns a scripts/measure.sh output directory into the Markdown summary used in README.md."""

from __future__ import annotations

import json
import sys
from pathlib import Path


def load(p: Path) -> dict | None:
    try:
        return json.loads(p.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def main(out: Path) -> None:
    print(f"# Measurements ({out})\n")

    print("## Loop timing, 100 Hz loop, nominal flights\n")
    print("| run | SCHED_FIFO | spin | frames | wake p50 us | wake p99 us | wake max us | jitter p50 us | jitter p99 us | jitter max us | overruns |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for d in sorted(out.glob("timing_*")):
        s = load(d / "fc_stats.json")
        if not s:
            continue
        w, j = s["wake_latency"], s["period_jitter"]
        print(f"| {d.name} | {s['sched_fifo']} | {s.get('spin_us', 0)} | {w['count']} | {w['p50_us']:.1f} | {w['p99_us']:.1f} "
              f"| {w['max_us']:.1f} | {j['p50_us']:.1f} | {j['p99_us']:.1f} | {j['max_us']:.1f} | {s['overruns']} |")

    print("\n## Flights (real-time SITL, three processes over UDP)\n")
    print("| run | outcome | touchdown m/s | apogee m | FC final state | abort reason | telemetry delivered | uplink acked |")
    print("|---|---|---|---|---|---|---|---|")
    for d in sorted(p for p in out.iterdir() if p.is_dir() and (p / "sim_summary.json").exists()):
        sim, fc, gs = load(d / "sim_summary.json"), load(d / "fc_stats.json"), load(d / "gs_stats.json")
        if not (sim and fc and gs):
            continue
        t, u = gs["telemetry"], gs["uplink"]
        print(f"| {d.name} | {sim['outcome']} | {sim['touchdown_speed_mps']:.2f} | {sim['apogee_m']:.0f} | {fc['final_state']} "
              f"| {fc['abort_reason']} | {t['delivery_pct']}% ({t['dropped']} lost, {t['corrupt']} corrupt) "
              f"| {u['acked']}/{u['commands']} ({u['retransmissions']} retx) |")

    print("\n## Uplink (stop-and-wait ARQ), 2000 PING commands\n")
    print("| link | delivered | failed | first-try | mean tx/cmd | retransmissions | latency p50 ms | p99 ms | max ms "
          "| corrupt ACKs rejected (gs) | corrupt commands rejected (fc) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    fc_corrupt = load(out / "uplink_corrupt_fc_stats.json")
    for name in ("uplink_clean", "uplink_netem20", "uplink_netem20_corrupt1"):
        s = load(out / f"{name}.json")
        if not s:
            continue
        lat = s["latency_ms"] or {}
        fc_rej = fc_corrupt["uplink_crc_errors"] if fc_corrupt and "corrupt" in name else "-"
        print(f"| {name} | {s['acked']}/{s['commands']} ({s['delivery_pct']}%) | {s['failed']} | {s['first_try_pct']}% "
              f"| {s['mean_attempts']} | {s['retransmissions']} | {lat.get('p50')} | {lat.get('p99')} | {lat.get('max')} "
              f"| {s['ack_decode_errors']} | {fc_rej} |")

    for name in ("replay.txt", "ctest.txt", "pytest.txt"):
        p = out / name
        if p.exists():
            print(f"\n## {name}\n\n```\n{p.read_text().strip()}\n```")


if __name__ == "__main__":
    main(Path(sys.argv[1] if len(sys.argv) > 1 else "out/measurements"))

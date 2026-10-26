#!/usr/bin/env python3
"""Plots a scripts/sitl.sh run: truth vs. flight-computer estimate, throttle, state and faults.

    python3 scripts/plot_flight.py out/sitl docs/flight.png [--title "..."]
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ground"))
from protocol import STATES, fault_names  # noqa: E402

STATE_COLORS = ["#dddddd", "#f4a261", "#e9c46a", "#8ecae6", "#e76f51", "#2a9d8f", "#d62828"]


def read_csv(path: Path) -> list[dict[str, float]]:
    with path.open() as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--title", default=None)
    args = ap.parse_args()

    tlm = read_csv(args.run_dir / "telemetry.csv")
    truth = read_csv(args.run_dir / "truth.csv")
    if not tlm:
        sys.exit("no telemetry")

    # Align sim time with flight-computer time using the ascent start in both.
    t_fc = [r["tick"] * 0.01 for r in tlm]
    fc_liftoff = next((r["tick"] * 0.01 for r in tlm if r["state"] >= 1), 0.0)
    sim_liftoff = next((r["t"] for r in truth if r["thrust"] > 100.0), 0.0)
    offset = fc_liftoff - sim_liftoff
    t_truth = [r["t"] + offset for r in truth]

    fig, ax = plt.subplots(4, 1, figsize=(10, 9), sharex=True, gridspec_kw={"height_ratios": [3, 2, 1.3, 0.5]})
    ax[0].plot(t_truth, [r["h"] for r in truth], color="black", lw=2.5, alpha=0.35, label="truth (sim)")
    ax[0].plot(t_fc, [r["h_est"] for r in tlm], color="tab:blue", lw=1.2, label="Kalman estimate")
    alt = [(t, r["alt_meas"]) for t, r in zip(t_fc, tlm) if int(r["sensor_flags"]) & 2]
    ax[0].plot([a[0] for a in alt], [a[1] for a in alt], ".", ms=1.5, color="tab:green", alpha=0.5, label="altimeter")
    ax[0].set_ylabel("altitude [m]")
    ax[0].legend(loc="upper right", fontsize=8)

    ax[1].plot(t_truth, [r["v"] for r in truth], color="black", lw=2.5, alpha=0.35, label="truth")
    ax[1].plot(t_fc, [r["v_est"] for r in tlm], color="tab:orange", lw=1.2, label="estimate")
    ax[1].axhline(0, color="grey", lw=0.5)
    ax[1].set_ylabel("velocity [m/s]")
    ax[1].legend(loc="lower left", fontsize=8)

    ax[2].plot(t_fc, [r["throttle_cmd"] for r in tlm], color="tab:red", lw=1.2)
    ax[2].set_ylabel("throttle")
    ax[2].set_ylim(-0.05, 1.05)

    # State timeline with fault overlay.
    start = 0
    for i in range(1, len(tlm) + 1):
        if i == len(tlm) or tlm[i]["state"] != tlm[start]["state"]:
            s = int(tlm[start]["state"])
            ax[3].axvspan(t_fc[start], t_fc[i - 1], color=STATE_COLORS[s], lw=0)
            if t_fc[i - 1] - t_fc[start] > 2.0:
                ax[3].text((t_fc[start] + t_fc[i - 1]) / 2, 0.5, STATES[s], ha="center", va="center", fontsize=7)
            start = i
    faulted = [(t, r) for t, r in zip(t_fc, tlm) if int(r["faults"]) & ~0x8 and t > fc_liftoff - 3]
    for t, _ in faulted:
        ax[3].plot([t, t], [0.0, 0.18], color="crimson", lw=0.6)
    ax[3].set_ylim(0.0, 1.0)
    ax[3].set_yticks([])
    ax[3].set_xlabel("flight computer time [s]   (red ticks: active fault)")

    names = sorted({n for _, r in faulted for n in fault_names(int(r["faults"])) if n != "ALT_OUTLIER"})
    title = args.title or args.run_dir.name
    if names:
        title += "  —  faults: " + ", ".join(names)
    ax[0].set_title(title)
    for a in ax[:3]:
        a.grid(alpha=0.3)
    first = min(t_fc[0], fc_liftoff - 3)
    ax[3].set_xlim(max(first, 0), t_fc[-1])
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=110)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Ground station: live telemetry plots and the acknowledged command uplink.

    python3 ground/ground_station.py                    # GUI (matplotlib)
    python3 ground/ground_station.py --headless --auto-arm 2 --stats gs.json

Console commands (type into the terminal, GUI or headless):
    arm | abort | ping | set <param> <value> | status | quit
    params: ascent_burn_s landing_throttle landing_kp terminal_velocity ignition_margin_m
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from link import ReliableUplink, SequenceTracker  # noqa: E402
from protocol import (  # noqa: E402
    Ack,
    AckResult,
    DecodeError,
    MsgType,
    Opcode,
    ParamId,
    Telemetry,
    decode_packet,
    fault_names,
)

TERMINAL_STATES = {"LANDED", "ABORT"}


class GroundStation:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        self.tlm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.tlm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tlm_sock.bind((args.bind, args.tlm_port))
        self.tlm_sock.settimeout(0.1)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.settimeout(0.005)
        self.fc_addr = (args.fc_host, args.cmd_port)

        self.tracker = SequenceTracker()
        self.uplink = ReliableUplink(rto=args.rto, max_attempts=args.max_attempts)
        self.ack_decode_errors = 0
        self.history: deque[tuple[float, Telemetry]] = deque(maxlen=args.history)
        self.latest: Telemetry | None = None
        self.first_tlm_at: float | None = None
        self.terminal_since: float | None = None
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.messages: deque[str] = deque(maxlen=6)

        self.csv_file = open(args.csv, "w", newline="") if args.csv else None
        self.csv = csv.writer(self.csv_file) if self.csv_file else None
        if self.csv:
            self.csv.writerow(["t_recv", *Telemetry.__dataclass_fields__])

    # --- threads ---------------------------------------------------------------

    def telemetry_loop(self) -> None:
        while not self.stop.is_set():
            try:
                data, _ = self.tlm_sock.recvfrom(512)
            except socket.timeout:
                continue
            now = time.monotonic()
            try:
                header, payload = decode_packet(data)
                if header.type != MsgType.TELEMETRY:
                    raise DecodeError("bad_type")
                tlm = Telemetry.parse(payload)
            except DecodeError:
                self.tracker.on_corrupt()
                continue
            self.tracker.on_packet(header.seq)
            with self.lock:
                self.latest = tlm
                self.history.append((now, tlm))
                if self.first_tlm_at is None:
                    self.first_tlm_at = now
                if tlm.state_name in TERMINAL_STATES:
                    self.terminal_since = self.terminal_since or now
            if self.csv:
                self.csv.writerow([f"{now:.4f}", *(getattr(tlm, f) for f in Telemetry.__dataclass_fields__)])

    def uplink_loop(self) -> None:
        while not self.stop.is_set():
            now = time.monotonic()
            for pkt in self.uplink.poll(now):
                self.cmd_sock.sendto(pkt, self.fc_addr)
            try:
                data, _ = self.cmd_sock.recvfrom(512)
            except (socket.timeout, ConnectionRefusedError):
                continue
            try:
                header, payload = decode_packet(data)
                if header.type != MsgType.ACK:
                    raise DecodeError("bad_type")
                ack = Ack.parse(payload)
            except DecodeError:
                self.ack_decode_errors += 1
                continue
            done = self.uplink.on_ack(ack, time.monotonic())
            if done:
                self.log(
                    f"ACK {Opcode(done.cmd.opcode).name} #{done.cmd.cmd_id}: {done.result.name} "
                    f"({done.attempts} tx, {done.latency * 1e3:.1f} ms)"
                )

    def automation_loop(self) -> None:
        armed = False
        while not self.stop.wait(0.05):
            now = time.monotonic()
            with self.lock:
                first, latest, terminal = self.first_tlm_at, self.latest, self.terminal_since
            if self.args.auto_arm is not None and not armed and first is not None and now - first >= self.args.auto_arm:
                if latest is not None and latest.state_name == "PAD" and latest.faults == 0:
                    self.send(Opcode.ARM)
                    armed = True
            if self.args.auto_abort is not None and first is not None and now - first >= self.args.auto_abort:
                self.send(Opcode.ABORT)
                self.args.auto_abort = None
            if self.args.exit_on_terminal and terminal is not None and now - terminal >= self.args.exit_on_terminal:
                self.stop.set()
            if self.args.duration and first is not None and now - first >= self.args.duration:
                self.stop.set()

    def console_loop(self) -> None:
        for line in sys.stdin:
            if self.stop.is_set():
                break
            self.handle_console(line.strip())

    # --- commands --------------------------------------------------------------

    def send(self, opcode: Opcode, param: int = 0, value: float = 0.0):
        pending = self.uplink.submit(opcode, param, value, time.monotonic())
        self.log(f"TX {opcode.name} #{pending.cmd.cmd_id}")
        return pending

    def handle_console(self, line: str) -> None:
        parts = line.split()
        if not parts:
            return
        cmd = parts[0].lower()
        if cmd == "arm":
            self.send(Opcode.ARM)
        elif cmd == "abort":
            self.send(Opcode.ABORT)
        elif cmd == "ping":
            self.send(Opcode.PING)
        elif cmd == "set" and len(parts) == 3:
            try:
                param = ParamId[parts[1].upper()]
                self.send(Opcode.SET_PARAM, param, float(parts[2]))
            except (KeyError, ValueError):
                print(f"unknown param or value; params: {', '.join(p.name.lower() for p in ParamId)}")
        elif cmd == "status":
            print(self.status_line())
        elif cmd in ("quit", "exit"):
            self.stop.set()
        else:
            print("commands: arm | abort | ping | set <param> <value> | status | quit")

    def log(self, msg: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        self.messages.append(f"{stamp} {msg}")
        print(f"[gs] {msg}", flush=True)

    # --- reporting -------------------------------------------------------------

    def status_line(self) -> str:
        with self.lock:
            t = self.latest
        if t is None:
            return "no telemetry yet"
        faults = ",".join(fault_names(t.faults)) or "none"
        return (
            f"{t.state_name:<12} h={t.h_est:8.1f} m  v={t.v_est:7.1f} m/s  thr={t.throttle_cmd:4.2f}  "
            f"faults={faults}  tlm {self.tracker.delivery_ratio * 100:5.1f}% "
            f"(lost {self.tracker.dropped}, corrupt {self.tracker.corrupt})  loop p99 {t.loop_p99_us:.0f} us"
        )

    def stats(self) -> dict:
        done = self.uplink.completed
        acked = [p for p in done if p.result is not None]
        latencies = sorted(p.latency for p in acked)

        def pct(p: float) -> float | None:
            if not latencies:
                return None
            return latencies[min(len(latencies) - 1, max(0, int(round(p / 100 * len(latencies) + 0.5)) - 1))] * 1e3

        with self.lock:
            last = self.latest
        return {
            "telemetry": {
                "received": self.tracker.received,
                "dropped": self.tracker.dropped,
                "reordered": self.tracker.reordered,
                "corrupt": self.tracker.corrupt,
                "delivery_pct": round(self.tracker.delivery_ratio * 100, 3),
            },
            "uplink": {
                "commands": len(done) + self.uplink.in_flight,
                "acked": len(acked),
                "failed": sum(p.failed for p in done),
                "delivery_pct": round(100 * len(acked) / len(done), 3) if done else None,
                "packets_sent": self.uplink.packets_sent,
                "retransmissions": self.uplink.retransmissions,
                "stray_acks": self.uplink.stray_acks,
                "ack_decode_errors": self.ack_decode_errors,
                "latency_ms_p50": pct(50),
                "latency_ms_p99": pct(99),
                "results": {r.name: sum(p.result == r for p in acked) for r in AckResult},
            },
            "final_state": last.state_name if last else None,
            "final_faults": fault_names(last.faults) if last else [],
            "fc_cmds_duplicate": last.cmds_duplicate if last else None,
        }

    # --- main ------------------------------------------------------------------

    def run(self) -> None:
        threads = [
            threading.Thread(target=self.telemetry_loop, daemon=True),
            threading.Thread(target=self.uplink_loop, daemon=True),
            threading.Thread(target=self.automation_loop, daemon=True),
        ]
        if not self.args.no_console:
            threads.append(threading.Thread(target=self.console_loop, daemon=True))
        for th in threads:
            th.start()
        print(f"[gs] telemetry on :{self.args.tlm_port}, uplink -> {self.fc_addr[0]}:{self.fc_addr[1]}, "
              f"session {self.uplink.session:#010x}", flush=True)
        try:
            if self.args.headless:
                self.run_headless()
            else:
                self.run_gui()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()
            time.sleep(0.15)
            self.finish()

    def run_headless(self) -> None:
        last_print = 0.0
        while not self.stop.is_set():
            time.sleep(0.05)
            if time.monotonic() - last_print >= 1.0:
                last_print = time.monotonic()
                print(f"[gs] {self.status_line()}", flush=True)

    def run_gui(self) -> None:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from matplotlib.widgets import Button

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        fig.canvas.manager.set_window_title("Flight Computer Ground Station")
        fig.subplots_adjust(left=0.08, right=0.72, top=0.95, bottom=0.07, hspace=0.12)
        (alt_line,) = axes[0].plot([], [], lw=1.5, label="h estimate")
        (alt_meas,) = axes[0].plot([], [], ".", ms=2, alpha=0.4, label="altimeter")
        axes[0].set_ylabel("altitude [m]")
        axes[0].legend(loc="upper right", fontsize=8)
        (vel_line,) = axes[1].plot([], [], lw=1.5, color="tab:orange")
        axes[1].axhline(0, color="grey", lw=0.5)
        axes[1].set_ylabel("velocity [m/s]")
        (thr_line,) = axes[2].plot([], [], lw=1.5, color="tab:red")
        axes[2].set_ylim(-0.05, 1.05)
        axes[2].set_ylabel("throttle")
        axes[2].set_xlabel("flight computer time [s]")
        for ax in axes:
            ax.grid(alpha=0.3)
        panel = fig.text(0.74, 0.93, "", family="monospace", fontsize=9, va="top")

        def make_button(y: float, label: str, opcode: Opcode, color: str):
            ax = fig.add_axes([0.76, y, 0.2, 0.06])
            b = Button(ax, label, color=color, hovercolor="white")
            b.on_clicked(lambda _evt: self.send(opcode))
            return b

        buttons = [  # noqa: F841 - keep references alive
            make_button(0.18, "ARM", Opcode.ARM, "#9fd89f"),
            make_button(0.10, "ABORT", Opcode.ABORT, "#f29b9b"),
        ]

        def update(_frame):
            with self.lock:
                hist = list(self.history)
                t = self.latest
            if self.stop.is_set():
                plt.close(fig)
                return []
            if hist:
                xs = [h.tick * 0.01 for _, h in hist]
                alt_line.set_data(xs, [h.h_est for _, h in hist])
                pairs = [(h.tick * 0.01, h.alt_meas) for _, h in hist if h.sensor_flags & 2]
                alt_meas.set_data([p[0] for p in pairs], [p[1] for p in pairs])
                vel_line.set_data(xs, [h.v_est for _, h in hist])
                thr_line.set_data(xs, [h.throttle_cmd for _, h in hist])
                for ax in axes:
                    ax.relim()
                    ax.autoscale_view(scalex=True, scaley=ax is not axes[2])
            if t is not None:
                faults = "\n  ".join(fault_names(t.faults)) or "none"
                panel.set_text(
                    f"STATE  {t.state_name}\n\n"
                    f"h      {t.h_est:9.1f} m\n"
                    f"v      {t.v_est:9.1f} m/s\n"
                    f"sigma  {t.h_sigma:9.2f} m\n"
                    f"thr    {t.throttle_cmd:9.2f}\n"
                    f"mass   {t.mass_est:9.1f} kg\n"
                    f"ign h  {t.ignition_alt:9.1f} m\n\n"
                    f"faults\n  {faults}\n\n"
                    f"downlink {self.tracker.delivery_ratio * 100:6.2f}%\n"
                    f"  lost {self.tracker.dropped}  bad {self.tracker.corrupt}\n"
                    f"uplink  acked {len([p for p in self.uplink.completed if p.result is not None])}"
                    f"  retx {self.uplink.retransmissions}\n"
                    f"loop p99 {t.loop_p99_us:6.0f} us\n"
                    f"overruns {t.loop_overruns}\n\n" + "\n".join(m[9:][:34] for m in self.messages)
                )
            return []

        self._anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()

    def finish(self) -> None:
        if self.csv_file:
            self.csv_file.close()
        stats = self.stats()
        print("[gs] " + json.dumps(stats), flush=True)
        if self.args.stats:
            Path(self.args.stats).write_text(json.dumps(stats, indent=2) + "\n")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--tlm-port", type=int, default=5600)
    p.add_argument("--fc-host", default="127.0.0.1")
    p.add_argument("--cmd-port", type=int, default=5601)
    p.add_argument("--headless", action="store_true", help="no GUI; print a status line every second")
    p.add_argument("--no-console", action="store_true", help="do not read commands from stdin")
    p.add_argument("--auto-arm", type=float, metavar="S", help="send ARM S seconds after telemetry starts")
    p.add_argument("--auto-abort", type=float, metavar="S", help="send ABORT S seconds after telemetry starts")
    p.add_argument("--exit-on-terminal", type=float, metavar="S", help="exit S seconds after LANDED/ABORT")
    p.add_argument("--duration", type=float, metavar="S", help="exit after S seconds of telemetry")
    p.add_argument("--rto", type=float, default=0.05, help="uplink retransmission timeout [s]")
    p.add_argument("--max-attempts", type=int, default=40)
    p.add_argument("--history", type=int, default=20000, help="telemetry frames kept for plotting")
    p.add_argument("--csv", help="write every telemetry frame to this CSV")
    p.add_argument("--stats", help="write link statistics JSON here on exit")
    return p.parse_args(argv)


if __name__ == "__main__":
    GroundStation(parse_args()).run()

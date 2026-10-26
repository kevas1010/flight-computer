"""Unit tests for the ground-station protocol and link layer.

    python3 -m unittest discover -s ground -v
"""

from __future__ import annotations

import random
import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from link import ReliableUplink, SequenceTracker  # noqa: E402
from protocol import (  # noqa: E402
    Ack,
    AckResult,
    Command,
    DecodeError,
    MsgType,
    Opcode,
    Telemetry,
    decode_packet,
    encode_packet,
    fault_names,
)


def telemetry(**over) -> Telemetry:
    base = dict(
        # Values exactly representable as float32 so a round trip compares equal.
        tick=1, state=0, faults=0, h_est=1.0, v_est=2.0, bias_est=0.0, h_sigma=0.125, accel_meas=9.75,
        alt_meas=1.0, sensor_flags=3, throttle_cmd=0.5, mass_est=900.0, ignition_alt=0.0, cmds_accepted=0,
        cmds_duplicate=0, alt_rejects=0, uplink_crc_errors=0, loop_overruns=0, loop_p99_us=50.0, loop_max_us=90.0,
    )
    base.update(over)
    return Telemetry(**base)


class ProtocolTest(unittest.TestCase):
    def test_crc_matches_standard_check_value(self):
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)

    def test_telemetry_round_trip(self):
        t = telemetry(tick=99, state=4, faults=0x3, h_est=12.5)
        header, payload = decode_packet(encode_packet(MsgType.TELEMETRY, 7, 123, t.pack()))
        self.assertEqual(header.seq, 7)
        self.assertEqual(Telemetry.parse(payload), t)

    def test_every_bit_flip_detected(self):
        pkt = bytearray(encode_packet(MsgType.TELEMETRY, 1, 2, telemetry().pack()))
        for i in range(len(pkt) * 8):
            bad = bytearray(pkt)
            bad[i // 8] ^= 1 << (i % 8)
            with self.assertRaises(DecodeError):
                decode_packet(bytes(bad))

    def test_truncated_and_padded(self):
        pkt = encode_packet(MsgType.ACK, 1, 2, Ack(1, 1, Opcode.PING, AckResult.OK).pack())
        for n in range(len(pkt)):
            with self.assertRaises(DecodeError):
                decode_packet(pkt[:n])
        with self.assertRaises(DecodeError) as cm:
            decode_packet(pkt + b"\x00")
        self.assertEqual(cm.exception.reason, "bad_length")

    def test_matches_cpp_layout(self):
        # Same bytes as tests/test_protocol.cpp HeaderIsLittleEndianOnTheWire.
        pkt = encode_packet(MsgType.ACTUATOR, 0x01020304, 0x1122334455667788, b"\x00" * 6)
        self.assertEqual(pkt[:4], bytes([0x5A, 0xFC, 1, 2]))
        self.assertEqual(pkt[4], 0x04)
        self.assertEqual(pkt[8], 0x88)
        self.assertEqual(pkt[16], 6)

    def test_fault_names(self):
        self.assertEqual(fault_names(0b11), ["ALT_STALE", "IMU_STUCK"])


class SequenceTrackerTest(unittest.TestCase):
    def test_counts_gaps_and_reorders(self):
        tr = SequenceTracker()
        for s in [0, 1, 2, 5, 6, 4, 10]:
            tr.on_packet(s)
        self.assertEqual(tr.dropped, 2 + 3)  # 3,4 then 7,8,9
        self.assertEqual(tr.reordered, 1)    # 4 arrived late
        self.assertEqual(tr.received, 7)

    def test_handles_wraparound(self):
        tr = SequenceTracker()
        for s in [0xFFFFFFFE, 0xFFFFFFFF, 0, 2]:
            tr.on_packet(s)
        self.assertEqual(tr.dropped, 1)
        self.assertAlmostEqual(tr.delivery_ratio, 4 / 5)


class FakeFlightComputer:
    """Receiver side: dedups by (session, cmd_id) like fc::CommandReceiver."""

    def __init__(self):
        self.executed: list[int] = []
        self.last_id = 0

    def handle(self, pkt: bytes) -> bytes:
        header, payload = decode_packet(pkt)
        assert header.type == MsgType.COMMAND
        cmd = Command.parse(payload)
        if cmd.cmd_id > self.last_id:
            self.executed.append(cmd.cmd_id)
            self.last_id = cmd.cmd_id
        return encode_packet(MsgType.ACK, 0, 0, Ack(cmd.session, cmd.cmd_id, cmd.opcode, AckResult.OK).pack())


def run_lossy(loss: float, count: int, seed: int = 1, rto: float = 0.05):
    """Simulated time: 1 ms steps, 2 ms one-way delay, independent loss each way."""
    rng = random.Random(seed)
    up = ReliableUplink(session=1234, rto=rto, max_attempts=100)
    fc = FakeFlightComputer()
    for _ in range(count):
        up.submit(Opcode.PING, now=0.0)
    wire: list[tuple[float, str, bytes]] = []
    now = 0.0
    while up.in_flight and now < 1000.0:
        for pkt in up.poll(now):
            if rng.random() >= loss:
                wire.append((now + 0.002, "up", pkt))
        due = [w for w in wire if w[0] <= now]
        wire = [w for w in wire if w[0] > now]
        for _, direction, pkt in due:
            if direction == "up":
                ack = fc.handle(pkt)
                if rng.random() >= loss:
                    wire.append((now + 0.002, "down", ack))
            else:
                _, payload = decode_packet(pkt)
                up.on_ack(Ack.parse(payload), now)
        now += 0.001
    return up, fc


class ReliableUplinkTest(unittest.TestCase):
    def test_lossless_single_attempt(self):
        up, fc = run_lossy(0.0, 50)
        self.assertEqual(len(up.completed), 50)
        self.assertTrue(all(p.attempts == 1 for p in up.completed))
        self.assertEqual(up.retransmissions, 0)
        # Latency is per command (one ~4 ms round trip), not time spent queued behind the others.
        self.assertLess(max(p.latency for p in up.completed), 0.006)

    def test_twenty_percent_loss_delivers_everything_exactly_once(self):
        up, fc = run_lossy(0.2, 500, seed=7)
        self.assertEqual(sum(p.result == AckResult.OK for p in up.completed), 500)
        self.assertEqual(fc.executed, list(range(1, 501)))  # each executed once, in order
        self.assertGreater(up.retransmissions, 100)          # ~36% of round trips lose a packet

    def test_gives_up_after_max_attempts(self):
        up = ReliableUplink(session=1, rto=0.01, max_attempts=3)
        p = up.submit(Opcode.ARM, now=0.0)
        sent = 0
        t = 0.0
        while up.in_flight:
            sent += len(up.poll(t))
            t += 0.01
        self.assertTrue(p.failed)
        self.assertEqual(sent, 3)
        self.assertTrue(p.done.is_set())

    def test_ignores_stale_and_foreign_acks(self):
        up = ReliableUplink(session=5, rto=0.05)
        up.submit(Opcode.PING, now=0.0)
        up.poll(0.0)
        self.assertIsNone(up.on_ack(Ack(6, 1, Opcode.PING, AckResult.OK), 0.01))  # other session
        self.assertIsNone(up.on_ack(Ack(5, 9, Opcode.PING, AckResult.OK), 0.01))  # wrong id
        self.assertEqual(up.stray_acks, 2)
        self.assertIsNotNone(up.on_ack(Ack(5, 1, Opcode.PING, AckResult.OK), 0.02))

    def test_retransmits_on_timeout_only(self):
        up = ReliableUplink(session=1, rto=0.05)
        up.submit(Opcode.PING, now=0.0)
        self.assertEqual(len(up.poll(0.0)), 1)
        self.assertEqual(len(up.poll(0.049)), 0)
        self.assertEqual(len(up.poll(0.051)), 1)
        self.assertEqual(up.retransmissions, 1)


if __name__ == "__main__":
    unittest.main()

"""Link-layer logic for the ground station: downlink loss accounting and the
reliable (acknowledged, retransmitted) command uplink.

Both classes are pure state machines driven by an explicit `now` so they can be
tested deterministically without sockets or sleeps.
"""

from __future__ import annotations

import random
import threading
from collections import deque
from dataclasses import dataclass, field

from protocol import Ack, AckResult, Command, MsgType, Opcode, encode_packet


class SequenceTracker:
    """Counts received, dropped (sequence gaps), duplicate/reordered and corrupt packets."""

    def __init__(self) -> None:
        self.received = 0
        self.dropped = 0
        self.reordered = 0
        self.corrupt = 0
        self._last: int | None = None

    def on_packet(self, seq: int) -> None:
        self.received += 1
        if self._last is None:
            self._last = seq
            return
        delta = (seq - self._last) & 0xFFFFFFFF
        if delta == 0 or delta > 0x7FFFFFFF:
            self.reordered += 1  # duplicate, or older than the newest seen
            return
        self.dropped += delta - 1
        self._last = seq

    def on_corrupt(self) -> None:
        self.corrupt += 1

    @property
    def expected(self) -> int:
        return self.received - self.reordered + self.dropped

    @property
    def delivery_ratio(self) -> float:
        return 1.0 if self.expected == 0 else (self.received - self.reordered) / self.expected


@dataclass
class PendingCommand:
    cmd: Command
    submitted_at: float
    attempts: int = 0
    first_sent: float | None = None
    last_sent: float | None = None
    acked_at: float | None = None
    result: AckResult | None = None
    failed: bool = False
    done: threading.Event = field(default_factory=threading.Event, repr=False)

    @property
    def latency(self) -> float | None:
        """First transmission to ACK, including retransmissions; excludes time queued behind other commands."""
        if self.acked_at is None or self.first_sent is None:
            return None
        return self.acked_at - self.first_sent


class ReliableUplink:
    """Stop-and-wait ARQ over UDP.

    One command is in flight at a time and is retransmitted every `rto` seconds
    until its ACK arrives or `max_attempts` is reached. Each ground-station run
    picks a random 32-bit session id; command ids increase by one per command, so
    the flight computer can recognise retransmissions and re-ACK them without
    executing the command twice.
    """

    def __init__(self, session: int | None = None, rto: float = 0.05, max_attempts: int = 40):
        self.session = session if session is not None else random.getrandbits(32)
        self.rto = rto
        self.max_attempts = max_attempts
        self._next_id = 1
        self._seq = 0
        self._queue: deque[PendingCommand] = deque()
        self._lock = threading.Lock()
        self.completed: list[PendingCommand] = []
        self.packets_sent = 0
        self.retransmissions = 0
        self.stray_acks = 0

    def submit(self, opcode: Opcode, param: int = 0, value: float = 0.0, now: float = 0.0) -> PendingCommand:
        with self._lock:
            cmd = Command(self.session, self._next_id, int(opcode), int(param), float(value))
            self._next_id += 1
            pending = PendingCommand(cmd, now)
            self._queue.append(pending)
            return pending

    def poll(self, now: float) -> list[bytes]:
        """Returns the packets to put on the wire right now (zero or one)."""
        with self._lock:
            while self._queue:
                head = self._queue[0]
                if head.last_sent is not None and now - head.last_sent < self.rto:
                    return []
                if head.attempts >= self.max_attempts:
                    head.failed = True
                    self._finish(head)
                    continue  # give up on this one, move to the next
                if head.attempts > 0:
                    self.retransmissions += 1
                head.attempts += 1
                if head.first_sent is None:
                    head.first_sent = now
                head.last_sent = now
                self.packets_sent += 1
                self._seq += 1
                return [encode_packet(MsgType.COMMAND, self._seq, int(now * 1e6), head.cmd.pack())]
            return []

    def on_ack(self, ack: Ack, now: float) -> PendingCommand | None:
        with self._lock:
            if not self._queue or ack.session != self.session:
                self.stray_acks += 1
                return None
            head = self._queue[0]
            if ack.cmd_id != head.cmd.cmd_id:
                self.stray_acks += 1  # late ACK for an earlier retransmission
                return None
            head.acked_at = now
            head.result = AckResult(ack.result)
            self._finish(head)
            return head

    def _finish(self, pending: PendingCommand) -> None:
        self._queue.popleft()
        self.completed.append(pending)
        pending.done.set()

    @property
    def in_flight(self) -> int:
        with self._lock:
            return len(self._queue)

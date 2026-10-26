"""Wire protocol shared with the flight computer (mirrors include/fc/protocol.hpp).

Packet layout, little-endian:
    magic u16 | version u8 | type u8 | seq u32 | timestamp_us u64 | payload_len u16
    payload[payload_len]
    crc32 u32   (zlib CRC-32 over header + payload)
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum

MAGIC = 0xFC5A
VERSION = 1

HEADER = struct.Struct("<HBBIQH")
CRC = struct.Struct("<I")
assert HEADER.size == 18


class MsgType(IntEnum):
    SENSOR = 1
    ACTUATOR = 2
    TELEMETRY = 3
    COMMAND = 4
    ACK = 5


class Opcode(IntEnum):
    ARM = 1
    ABORT = 2
    SET_PARAM = 3
    PING = 4


class ParamId(IntEnum):
    ASCENT_BURN_S = 1
    LANDING_THROTTLE = 2
    LANDING_KP = 3
    TERMINAL_VELOCITY = 4
    IGNITION_MARGIN_M = 5


class AckResult(IntEnum):
    OK = 0
    REJECTED = 1
    BAD_PARAM = 2
    BAD_OPCODE = 3


STATES = ["PAD", "ASCENT", "COAST", "DESCENT", "LANDING_BURN", "LANDED", "ABORT"]

FAULTS = {
    1 << 0: "ALT_STALE",
    1 << 1: "IMU_STUCK",
    1 << 2: "SENSOR_LINK",
    1 << 3: "ALT_OUTLIER",
    1 << 4: "EST_DIVERGED",
    1 << 5: "IMU_INVALID",
}


def fault_names(bits: int) -> list[str]:
    return [name for bit, name in FAULTS.items() if bits & bit]


class DecodeError(Exception):
    """Raised for any malformed packet; `reason` matches fc::proto::DecodeError names."""

    def __init__(self, reason: str):
        super().__init__(reason)
        self.reason = reason


@dataclass(frozen=True)
class Header:
    type: int
    seq: int
    timestamp_us: int


def encode_packet(msg_type: int, seq: int, timestamp_us: int, payload: bytes) -> bytes:
    body = HEADER.pack(MAGIC, VERSION, int(msg_type), seq & 0xFFFFFFFF, timestamp_us, len(payload)) + payload
    return body + CRC.pack(zlib.crc32(body))


def decode_packet(data: bytes) -> tuple[Header, bytes]:
    if len(data) < HEADER.size + CRC.size:
        raise DecodeError("too_short")
    magic, version, msg_type, seq, ts, length = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise DecodeError("bad_magic")
    if version != VERSION:
        raise DecodeError("bad_version")
    body_len = HEADER.size + length
    if body_len + CRC.size != len(data):
        raise DecodeError("bad_length")
    (crc,) = CRC.unpack_from(data, body_len)
    if crc != zlib.crc32(data[:body_len]):
        raise DecodeError("bad_crc")
    if msg_type not in MsgType._value2member_map_:
        raise DecodeError("bad_type")
    return Header(msg_type, seq, ts), data[HEADER.size:body_len]


# --- Messages -----------------------------------------------------------------

TELEMETRY = struct.Struct("<IBH4f2fB3f3I2I2f")
COMMAND = struct.Struct("<IIBBf")
ACK = struct.Struct("<IIBB")
assert TELEMETRY.size == 72 and COMMAND.size == 14 and ACK.size == 10


@dataclass(frozen=True)
class Telemetry:
    tick: int
    state: int
    faults: int
    h_est: float
    v_est: float
    bias_est: float
    h_sigma: float
    accel_meas: float
    alt_meas: float
    sensor_flags: int
    throttle_cmd: float
    mass_est: float
    ignition_alt: float
    cmds_accepted: int
    cmds_duplicate: int
    alt_rejects: int
    uplink_crc_errors: int
    loop_overruns: int
    loop_p99_us: float
    loop_max_us: float

    @property
    def state_name(self) -> str:
        return STATES[self.state] if self.state < len(STATES) else f"?{self.state}"

    @classmethod
    def parse(cls, payload: bytes) -> "Telemetry":
        if len(payload) != TELEMETRY.size:
            raise DecodeError("bad_payload")
        return cls(*TELEMETRY.unpack(payload))

    def pack(self) -> bytes:
        return TELEMETRY.pack(*(getattr(self, f) for f in self.__dataclass_fields__))


@dataclass(frozen=True)
class Command:
    session: int
    cmd_id: int
    opcode: int
    param: int = 0
    value: float = 0.0

    def pack(self) -> bytes:
        return COMMAND.pack(self.session, self.cmd_id, int(self.opcode), int(self.param), float(self.value))

    @classmethod
    def parse(cls, payload: bytes) -> "Command":
        if len(payload) != COMMAND.size:
            raise DecodeError("bad_payload")
        return cls(*COMMAND.unpack(payload))


@dataclass(frozen=True)
class Ack:
    session: int
    cmd_id: int
    opcode: int
    result: int

    def pack(self) -> bytes:
        return ACK.pack(self.session, self.cmd_id, int(self.opcode), int(self.result))

    @classmethod
    def parse(cls, payload: bytes) -> "Ack":
        if len(payload) != ACK.size:
            raise DecodeError("bad_payload")
        return cls(*ACK.unpack(payload))

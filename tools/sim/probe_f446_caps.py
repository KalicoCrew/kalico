#!/usr/bin/env python3
"""F446 sim smoke probe: send QueryRuntimeCaps over the kalico-native channel,
verify the response carries the SMALL-profile sizing constants.

Runs against the booted F446 Renode sim (USART2 → tcp://localhost:3334).
Hand-frames the kalico-native packet directly so this stays decoupled from
the higher-level kalico-host-rt Rust code.

Wire format (per src/kalico_dispatch.c + rust/kalico-protocol/src/messages.rs):

  Outer frame:
    sync         u8   = 0x55
    len_field    u16  little-endian; counts [len .. crc] inclusive
                       = 2 + 1 + payload_len + 2
    channel      u8   = 0x01 (control channel)
    payload      ...  per-message body, see below
    crc16_ccitt  u16  little-endian, over [len .. payload]

  Per-message header (inside payload):
    kind         u16  little-endian
    version      u8
    correlation_id u32 little-endian

  QueryRuntimeCaps body: empty (kind 0x0040)
  RuntimeCapsResponse body (11 bytes):
    max_control_points  u32_le
    max_knot_vector_len u32_le
    max_degree          u8
    curve_pool_n        u16_le
"""

from __future__ import annotations

import socket
import struct
import sys
import time

KALICO_SYNC = 0x55
KLIPPER_SYNC = 0x7E
KIND_QUERY_RUNTIME_CAPS = 0x0040
KIND_RUNTIME_CAPS_RESPONSE = 0x0041
CHANNEL_CONTROL = 0x00

EXPECTED_SMALL = {
    "max_control_points": 512,
    "max_knot_vector_len": 524,
    "max_degree": 10,
    "curve_pool_n": 4,
}


def crc16_ccitt(data: bytes) -> int:
    """CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflection, no final xor) —
    byte-at-a-time variant. Matches src/generic/crc16_ccitt.c (firmware) and
    rust/kalico-native-transport/src/frame.rs::crc16_ccitt (host)."""
    crc = 0xFFFF
    for byte in data:
        d = byte ^ (crc & 0x00FF)
        d = (d ^ ((d << 4) & 0x00FF)) & 0xFF
        crc = ((crc >> 8) ^ (d << 8) ^ (d << 3) ^ (d >> 4)) & 0xFFFF
    return crc


def build_kalico_frame(
    kind: int, version: int, correlation_id: int, body: bytes
) -> bytes:
    msg = struct.pack("<HBI", kind, version, correlation_id) + body
    len_field = 2 + 1 + len(msg) + 2
    header = (
        struct.pack("<BH", KALICO_SYNC, len_field)
        + bytes([CHANNEL_CONTROL])
        + msg
    )
    crc = crc16_ccitt(header[1:])
    return header + struct.pack("<H", crc)


def parse_kalico_frame(buf: bytes) -> tuple[int, int, int, bytes] | None:
    """Walk `buf` looking for a valid kalico frame. Returns (kind, version,
    correlation_id, body) or None if no complete frame found."""
    i = 0
    while i < len(buf):
        if buf[i] != KALICO_SYNC:
            i += 1
            continue
        if i + 3 > len(buf):
            return None
        len_field = struct.unpack_from("<H", buf, i + 1)[0]
        total = 1 + len_field
        if i + total > len(buf):
            return None
        frame = buf[i : i + total]
        crc_actual = crc16_ccitt(frame[1:-2])
        crc_expected = struct.unpack_from("<H", frame, total - 2)[0]
        if crc_actual != crc_expected:
            i += 1
            continue
        if frame[3] != CHANNEL_CONTROL:
            i += total
            continue
        kind, version, correlation_id = struct.unpack_from("<HBI", frame, 4)
        body = bytes(frame[11 : total - 2])
        return kind, version, correlation_id, body
    return None


def main() -> int:
    host, port = "localhost", 3334
    print(f"[probe] connecting to {host}:{port}")
    s = socket.create_connection((host, port), timeout=10.0)
    s.settimeout(5.0)

    drained = bytearray()
    try:
        while True:
            chunk = s.recv(4096)
            if not chunk:
                break
            drained.extend(chunk)
            if not chunk:
                break
    except socket.timeout:
        pass
    if drained:
        print(
            f"[probe] drained {len(drained)} bytes of pre-existing UART traffic"
        )

    correlation_id = 0xCAFEBABE
    frame = build_kalico_frame(KIND_QUERY_RUNTIME_CAPS, 0, correlation_id, b"")
    print(
        f"[probe] sending QueryRuntimeCaps ({len(frame)} bytes): {frame.hex()}"
    )
    s.sendall(frame)

    deadline = time.monotonic() + 8.0
    rxbuf = bytearray()
    while time.monotonic() < deadline:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        rxbuf.extend(chunk)
        result = parse_kalico_frame(bytes(rxbuf))
        if result is None:
            continue
        kind, version, corr, body = result
        if kind != KIND_RUNTIME_CAPS_RESPONSE:
            print(
                f"[probe] received non-target frame kind=0x{kind:04x}; continuing"
            )
            continue
        if corr != correlation_id:
            print(
                f"[probe] correlation mismatch: got 0x{corr:08x}, expected 0x{correlation_id:08x}"
            )
            return 2
        if len(body) != 11:
            print(
                f"[probe] response body wrong length: {len(body)} (expected 11)"
            )
            return 3
        mcp, mkv = struct.unpack_from("<II", body, 0)
        mdeg = body[8]
        cpn = struct.unpack_from("<H", body, 9)[0]
        got = {
            "max_control_points": mcp,
            "max_knot_vector_len": mkv,
            "max_degree": mdeg,
            "curve_pool_n": cpn,
        }
        print(f"[probe] RuntimeCapsResponse: {got}")
        if got == EXPECTED_SMALL:
            print("[probe] PASS — caps match RUNTIME_TARGET_SMALL profile.")
            return 0
        print(f"[probe] FAIL — expected {EXPECTED_SMALL}")
        return 4

    print("[probe] FAIL — no RuntimeCapsResponse received within timeout")
    print(
        f"[probe] received {len(rxbuf)} bytes total: {bytes(rxbuf).hex()[:200]}"
    )
    return 5


if __name__ == "__main__":
    sys.exit(main())

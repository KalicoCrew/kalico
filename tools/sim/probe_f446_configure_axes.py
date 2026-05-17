#!/usr/bin/env python3
"""F446 sim: send ConfigureAxes via kalico-protocol, then watch for crash.

The hypothesis: configure_axes registers step_time_event timers for
StepTime motors. step_time_event fires every 100 ms post-config, calling
into kalico_runtime_step_ring_peek_head + arm_producer_timer_if_kicked_inline.
If any of those FFIs / arm logic has a bug that's only reachable post-
configure_axes, this is when F4 starts crashing on the bench.

Test plan:
  1. Boot F4 sim.
  2. Drain UART.
  3. Send ConfigureAxes (kalico-protocol kind=0x0030, body 20 bytes).
  4. Watch for ConfigureAxesResponse + any subsequent shutdown / silence.
  5. Pump for 10 seconds; emit timestamps for every received frame.

If the sim crashes (sim process dies, or sim stops emitting any frames for
> 1 s of wall-clock after the ConfigureAxesResponse), we've reproduced the
bench bug in sim.
"""
from __future__ import annotations

import socket
import struct
import sys
import time

KALICO_SYNC = 0x55
CHANNEL_CONTROL = 0x00
KIND_CONFIGURE_AXES = 0x0030
KIND_CONFIGURE_AXES_RESPONSE = 0x0031


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        d = (byte ^ (crc & 0x00FF))
        d = (d ^ ((d << 4) & 0x00FF)) & 0xFF
        crc = ((crc >> 8) ^ (d << 8) ^ (d << 3) ^ (d >> 4)) & 0xFFFF
    return crc


def build_kalico_frame(kind: int, version: int, correlation_id: int, body: bytes) -> bytes:
    msg = struct.pack("<HBI", kind, version, correlation_id) + body
    len_field = 2 + 1 + len(msg) + 2
    header = struct.pack("<BH", KALICO_SYNC, len_field) + bytes([CHANNEL_CONTROL]) + msg
    crc = crc16_ccitt(header[1:])
    return header + struct.pack("<H", crc)


def parse_one_kalico_frame(buf: bytes, start: int):
    """Try to parse a kalico frame starting at `start`. Returns (frame_len,
    (kind, version, correlation_id, body)) on success, or (1, None) to advance
    by one byte to skip junk."""
    if start + 3 > len(buf):
        return 0, None
    if buf[start] != KALICO_SYNC:
        return 1, None
    len_field = struct.unpack_from("<H", buf, start + 1)[0]
    total = 1 + len_field
    if start + total > len(buf):
        return 0, None
    frame = buf[start : start + total]
    crc_actual = crc16_ccitt(frame[1:-2])
    crc_expected = struct.unpack_from("<H", frame, total - 2)[0]
    if crc_actual != crc_expected:
        return 1, None
    if frame[3] != CHANNEL_CONTROL:
        return total, None
    kind, version, correlation_id = struct.unpack_from("<HBI", frame, 4)
    body = bytes(frame[11 : total - 2])
    return total, (kind, version, correlation_id, body)


def collect_frames(buf: bytes):
    """Yield (kind, version, correlation_id, body, raw_offset) tuples."""
    out = []
    i = 0
    while i < len(buf):
        consumed, parsed = parse_one_kalico_frame(buf, i)
        if consumed == 0:
            break
        if parsed is not None:
            out.append((parsed, i, consumed))
        i += consumed
    return out, i


def main():
    print("[probe] connecting to localhost:3334")
    s = socket.create_connection(("localhost", 3334), timeout=5.0)
    s.settimeout(1.0)

    # Drain pre-existing UART noise (status_drain emits at 10 Hz).
    pre = bytearray()
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            chunk = b""
        if chunk:
            pre.extend(chunk)
        else:
            break
    print(f"[probe] drained {len(pre)} bytes pre-existing UART")

    # Build ConfigureAxes blob: Cartesian, Z only, 400 steps/mm on Z.
    kinematics = 1  # CartesianXyzAndE
    present_mask = 0x04  # bit 2 = Z
    awd_mask = 0x00
    invert_mask = 0x00
    steps_per_mm = [0.0, 0.0, 400.0, 0.0]
    blob = struct.pack("<BBBB", kinematics, present_mask, awd_mask, invert_mask)
    for s_per_mm in steps_per_mm:
        blob += struct.pack("<f", s_per_mm)
    assert len(blob) == 20, f"blob len {len(blob)}"

    correlation_id = 0xCAFEBABE
    frame = build_kalico_frame(KIND_CONFIGURE_AXES, 0, correlation_id, blob)
    print(f"[probe] -> ConfigureAxes ({len(frame)}B): {frame.hex()}")
    s.sendall(frame)

    # Watch for response + any subsequent traffic
    rx_total = bytearray()
    last_byte_t = time.monotonic()
    last_emit_t = time.monotonic()
    silence_warned = False
    start = time.monotonic()
    end = start + 30.0
    saw_config_response = False
    while time.monotonic() < end:
        try:
            s.settimeout(0.2)
            chunk = s.recv(4096)
        except socket.timeout:
            chunk = b""
        if chunk:
            now = time.monotonic()
            if now - last_byte_t > 1.0:
                print(f"[probe] t={now-start:5.2f}s RECOVERY  silence broken after {now-last_byte_t:.2f}s")
            last_byte_t = now
            rx_total.extend(chunk)
        # Periodic-status report
        if time.monotonic() - last_emit_t > 1.0:
            silence = time.monotonic() - last_byte_t
            print(f"[probe] t={time.monotonic()-start:5.2f}s rx_total={len(rx_total)}B silence={silence:.2f}s")
            last_emit_t = time.monotonic()
            if silence > 5.0 and not silence_warned:
                print("[probe] *** F4 sim has been silent >5s after ConfigureAxes — possible crash ***")
                silence_warned = True

        # Parse frames and look for ConfigureAxesResponse
        frames, _ = collect_frames(bytes(rx_total))
        for (kind, ver, corr, body), off, n in frames:
            if kind == KIND_CONFIGURE_AXES_RESPONSE and corr == correlation_id and not saw_config_response:
                saw_config_response = True
                result = struct.unpack("<i", body[:4])[0]
                print(f"[probe] +++ ConfigureAxesResponse result={result} (at offset {off}) +++")

    # Final summary
    print("")
    print("=" * 60)
    print(f"[probe] Total RX: {len(rx_total)} bytes over {time.monotonic()-start:.1f}s")
    print(f"[probe] Saw ConfigureAxesResponse: {saw_config_response}")
    print(f"[probe] Final silence: {time.monotonic()-last_byte_t:.2f}s")

    # Decode all kalico frames we saw, count by kind
    frames, _ = collect_frames(bytes(rx_total))
    by_kind = {}
    for (kind, ver, corr, body), off, n in frames:
        by_kind[kind] = by_kind.get(kind, 0) + 1
    print(f"[probe] Frame kinds seen: {dict(sorted(by_kind.items()))}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

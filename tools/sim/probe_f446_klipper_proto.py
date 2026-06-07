#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import socket
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from klippy import msgproto  # noqa: E402

MESSAGE_SYNC = 0x7E
MESSAGE_DEST = 0x10
MESSAGE_MIN = 5
MESSAGE_MAX = 64


def encode_vli(val):
    out = bytearray()
    msgproto.PT_uint32().encode(out, val)
    return bytes(out)


def encode_string(val):
    if isinstance(val, str):
        val = val.encode()
    return bytes([len(val)]) + val


def build_frame(seq, payload):
    msglen = MESSAGE_MIN + len(payload)
    if msglen > MESSAGE_MAX:
        raise ValueError(f"frame too large: {msglen}")
    header = bytes([msglen, MESSAGE_DEST | (seq & 0x0F)])
    crc = msgproto.crc16_ccitt(header + payload)
    return header + payload + bytes(crc) + bytes([MESSAGE_SYNC])


def parse_frames(buf):
    frames = []
    i = 0
    while i < len(buf):
        if i + MESSAGE_MIN > len(buf):
            break
        msglen = buf[i]
        if msglen < MESSAGE_MIN or msglen > MESSAGE_MAX:
            i += 1
            continue
        if i + msglen > len(buf):
            break
        if buf[i + msglen - 1] != MESSAGE_SYNC:
            i += 1
            continue
        crc = list(msgproto.crc16_ccitt(buf[i : i + msglen - 3]))
        expected = [buf[i + msglen - 3], buf[i + msglen - 2]]
        if crc != expected:
            i += 1
            continue
        seq_byte = buf[i + 1]
        payload = bytes(buf[i + 2 : i + msglen - 3])
        frames.append((seq_byte, payload, bytes(buf[i : i + msglen])))
        i += msglen
    return frames, buf[i:]


def send_identify(s, seq, offset, count):
    body = encode_vli(1) + encode_vli(offset) + encode_vli(count)
    frame = build_frame(seq, body)
    print(
        f"[probe] -> identify offset={offset} count={count} ({len(frame)}B): {frame.hex()}"
    )
    s.sendall(frame)


def recv_with_timeout(s, timeout_s, accumulator):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            s.settimeout(max(0.05, deadline - time.monotonic()))
            chunk = s.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        accumulator.extend(chunk)
    return accumulator


def recv_until_frames_present(s, min_frames, max_wait_s):
    accumulator = bytearray()
    deadline = time.monotonic() + max_wait_s
    while time.monotonic() < deadline:
        try:
            s.settimeout(0.2)
            chunk = s.recv(4096)
        except socket.timeout:
            chunk = b""
        if chunk:
            accumulator.extend(chunk)
            frames, _ = parse_frames(accumulator)
            if len(frames) >= min_frames:
                try:
                    s.settimeout(0.01)
                    while True:
                        more = s.recv(4096)
                        if not more:
                            break
                        accumulator.extend(more)
                except socket.timeout:
                    pass
                break
    return accumulator


def fetch_data_dict(s, verbose=False):
    blob = bytearray()
    offset = 0
    seq = 0
    nak_retries = 0
    last_progress_offset = -1
    last_seq_used = seq
    while True:
        send_identify(s, seq, offset, 40)
        last_seq_used = seq

        rx = recv_until_frames_present(s, min_frames=2, max_wait_s=1.5)
        if verbose:
            print(f"[probe]   <- {len(rx)} bytes raw: {bytes(rx).hex()}")
        frames, leftover = parse_frames(rx)
        if verbose:
            print(
                f"[probe]   parsed {len(frames)} frames, {len(leftover)} bytes leftover"
            )
            for fi, (sb, pl, _) in enumerate(frames):
                print(
                    f"[probe]     frame[{fi}] seq=0x{sb:02x} payload={pl.hex()}"
                )

        nak_seq = None
        for sb, pl, _ in frames:
            if len(pl) == 0:
                nak_seq = sb & 0x0F
                break
        if nak_seq is not None and not any(len(pl) > 0 for sb, pl, _ in frames):
            if nak_retries < 3:
                print(f"[probe]   MCU NAK, resyncing seq to 0x{nak_seq:02x}")
                seq = nak_seq
                nak_retries += 1
                continue
        progress = False
        for _seq, payload, _raw in frames:
            if len(payload) < 1 or payload[0] != 0:
                continue
            pos = 1
            r_offset, pos = msgproto.PT_uint32().parse(payload, pos)
            data, pos = msgproto.PT_progmem_buffer().parse(payload, pos)
            if r_offset != offset:
                continue
            blob.extend(data)
            offset += len(data)
            progress = True
            if not data:
                return bytes(blob), last_seq_used
        if not progress:
            print(f"[probe] no progress at offset {offset}; bailing")
            return bytes(blob), last_seq_used
        nak_retries = 0
        seq = (seq + 1) & 0x0F


def parse_dict(blob):
    decompressed = zlib.decompress(blob)
    return json.loads(decompressed)


def find_cmd(data_dict, prefix):
    for fmt, cid in data_dict["commands"].items():
        if fmt.startswith(prefix + " ") or fmt == prefix:
            return cid, fmt
    return None, None


def main():
    s = socket.create_connection(("localhost", 3334), timeout=5.0)
    s.settimeout(2.0)

    drain = bytearray()
    recv_with_timeout(s, 0.5, drain)
    if drain:
        print(f"[probe] drained {len(drain)} bytes pre-existing UART traffic")

    blob, last_dict_seq = fetch_data_dict(s)
    print(
        f"[probe] data dictionary: {len(blob)} compressed bytes (last seq used: 0x{last_dict_seq:02x})"
    )
    if not blob:
        print("[probe] FAIL — no data dictionary received")
        return 1
    try:
        data_dict = parse_dict(blob)
    except Exception as e:
        print(f"[probe] dict parse failed: {e!r}")
        print(f"[probe]   blob[:64]={blob[:64].hex()}")
        return 2

    print(
        f"[probe] mcu='{data_dict.get('mcu')}' version='{data_dict.get('version')}'"
    )
    cmds_by_name = sorted(data_dict["commands"].keys())
    print(f"[probe] {len(cmds_by_name)} commands available")
    for name in cmds_by_name:
        if "tmcuart" in name or "shutdown" in name:
            print(f"[probe]   cmd: {name}")

    cfg_id, cfg_fmt = find_cmd(data_dict, "config_tmcuart")
    snd_id, snd_fmt = find_cmd(data_dict, "tmcuart_send")
    fin_id, fin_fmt = find_cmd(data_dict, "finalize_config")
    aoid_id, aoid_fmt = find_cmd(data_dict, "allocate_oids")
    print(f"[probe] config_tmcuart cmd id={cfg_id}, fmt={cfg_fmt}")
    print(f"[probe] tmcuart_send cmd id={snd_id}, fmt={snd_fmt}")
    print(f"[probe] finalize_config cmd id={fin_id}, fmt={fin_fmt}")
    print(f"[probe] allocate_oids cmd id={aoid_id}, fmt={aoid_fmt}")

    responses_by_id = {cid: fmt for fmt, cid in data_dict["responses"].items()}
    print(f"[probe] {len(responses_by_id)} responses in dict")
    for k in sorted(responses_by_id)[:20]:
        print(f"[probe]   response[{k}]: {responses_by_id[k]}")
    if not snd_id:
        print("[probe] FAIL — no tmcuart_send command in dictionary")
        return 3

    print("[probe] === BOOT SEQUENCE ===")

    seq_counter = [(last_dict_seq + 1) & 0x0F]

    def next_seq():
        v = seq_counter[0]
        seq_counter[0] = (v + 1) & 0x0F
        return v

    def send_cmd(cmd_id, *args, label=""):
        body = encode_vli(cmd_id)
        for a in args:
            if isinstance(a, (bytes, bytearray)):
                body += encode_string(a)
            else:
                body += encode_vli(a)
        frame = build_frame(next_seq(), body)
        print(f"[probe] -> {label} ({len(frame)}B) {frame.hex()}")
        s.sendall(frame)

    send_cmd(aoid_id, 1, label="allocate_oids count=1")
    rx = bytearray()
    recv_with_timeout(s, 1.0, rx)
    if rx:
        print(f"[probe]   <- {len(rx)} bytes: {bytes(rx).hex()[:200]}")
    # gpio encoding: (port - 'A') * 16 + pin; PA10 = 10, PA9 = 9
    send_cmd(cfg_id, 0, 10, 1, 9, 27500, label="config_tmcuart")
    rx = bytearray()
    recv_with_timeout(s, 1.0, rx)
    if rx:
        print(f"[probe]   <- {len(rx)} bytes: {bytes(rx).hex()[:200]}")

    if fin_id is not None:
        send_cmd(fin_id, 0, label="finalize_config crc=0")
        rx = bytearray()
        recv_with_timeout(s, 1.0, rx)
        if rx:
            print(f"[probe]   <- {len(rx)} bytes: {bytes(rx).hex()[:200]}")

    write_bytes = bytes([0x05, 0x00, 0x00, 0xFF])
    print(f"[probe] -> tmcuart_send oid=0 write={write_bytes.hex()} read=8")
    send_cmd(snd_id, 0, write_bytes, 8, label="tmcuart_send oid=0 write=...")

    print("[probe] waiting 4s for response / crash...")
    rx = bytearray()
    recv_with_timeout(s, 4.0, rx)
    print(f"[probe]   <- {len(rx)} bytes: {bytes(rx).hex()[:400]}")

    frames, leftover = parse_frames(rx)
    print(
        f"[probe] parsed {len(frames)} frames, {len(leftover)} bytes leftover"
    )
    for i, (sb, pl, _) in enumerate(frames):
        print(f"[probe]   frame[{i}]: seq=0x{sb:02x} payload={pl.hex()}")

    return 0


if __name__ == "__main__":
    sys.exit(main())

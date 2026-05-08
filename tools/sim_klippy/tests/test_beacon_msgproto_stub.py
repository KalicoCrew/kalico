"""Unit tests for the faithful BeaconMcuStub.

These tests open the PTY directly (no klippy process) and exercise the
msgproto wire handshake so we can iterate on the stub's framing /
identify / NVM / streaming surface without spinning up the full sim.

Each test acts like a tiny mock-klippy: it encodes a known msgproto
command, frames it with crc16-ccitt + sync-byte trailer, writes to the
PTY slave, then reads frames back from the PTY and parses them via the
same MessageParser.
"""

from __future__ import annotations

import json
import os
import struct
import time
import zlib

import pytest

from klippy import msgproto

from tools.sim_klippy.orchestrator.beacon_serial_stub import (
    BeaconMcuStub,
    NVM_IMAGE,
)
from tools.sim_klippy.orchestrator.beacon_identify_dict import (
    BEACON_COMMANDS,
    BEACON_RESPONSES,
    IDENTIFY_BLOB,
)


# ---------------------------------------------------------------------------
# Wire helpers
# ---------------------------------------------------------------------------

def _frame(parser: msgproto.MessageParser, seq: int, msgformat: str,
           **kwargs) -> bytes:
    """Encode + frame a command for sending to the stub.

    Mirrors `BeaconMcuStub._send_msg` but with a caller-supplied seq so
    tests can simulate klippy's serialqueue.c sender side.
    """
    cmd = parser.lookup_command(msgformat).encode_by_name(**kwargs)
    seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
    payload = [msgproto.MESSAGE_MIN + len(cmd), seq_byte] + list(cmd)
    crc = msgproto.crc16_ccitt(payload)
    payload.extend(crc)
    payload.append(msgproto.MESSAGE_SYNC)
    return bytes(payload)


def _parser_with_default_messages() -> msgproto.MessageParser:
    """Return a parser that knows only the default identify messages.

    Used for the very first identify exchange before we've loaded the
    full dictionary. msgproto.DefaultMessages already includes
    `identify offset=%u count=%c` / `identify_response offset=%u data=%.*s`.
    """
    return msgproto.MessageParser(warn_prefix="test: ")


def _open_pty_writer(pty_path: str) -> int:
    """Open the PTY symlinked at *pty_path* in read/write mode."""
    return os.open(pty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)


def _read_frames(fd: int, parser: msgproto.MessageParser,
                 expected_name: str, timeout: float = 2.0) -> dict:
    """Read PTY bytes until a frame whose ``#name`` == expected_name.

    Returns the parsed parameter dict. Times out after `timeout`.
    """
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = os.read(fd, 4096)
            if chunk:
                buf.extend(chunk)
        except (BlockingIOError, OSError):
            time.sleep(0.005)
        # Drain whatever framed messages have arrived.
        while True:
            msglen = parser.check_packet(buf)
            if msglen == 0:
                break
            if msglen < 0:
                idx = buf.find(msgproto.MESSAGE_SYNC)
                if idx < 0:
                    buf.clear()
                    break
                del buf[: idx + 1]
                continue
            frame = list(buf[:msglen])
            del buf[:msglen]
            params = parser.parse(frame)
            if params.get("#name") == expected_name:
                return params
    raise AssertionError(
        f"timed out waiting for {expected_name!r}; "
        f"residual buffer: {bytes(buf)[:64].hex()}"
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@pytest.fixture
def stub(tmp_path):
    pty_path = str(tmp_path / "beacon_pty")
    log_path = str(tmp_path / "beacon_traffic.log")
    s = BeaconMcuStub(pty_path, log_path=log_path)
    s.start()
    # Wait for symlink to appear.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if os.path.exists(pty_path):
            break
        time.sleep(0.01)
    assert os.path.exists(pty_path), "PTY symlink did not appear"
    try:
        yield s, pty_path
    finally:
        s.stop()


def test_identify_chunked_returns_full_dict(stub):
    """Drive identify offset=N count=K until the stub returns empty data,
    assemble the chunks, zlib-decompress, and verify the dictionary keys.
    """
    s, pty_path = stub
    parser = _parser_with_default_messages()
    fd = _open_pty_writer(pty_path)
    try:
        identify_data = bytearray()
        seq = 1
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = _frame(parser, seq, "identify offset=%u count=%c",
                           offset=len(identify_data), count=40)
            os.write(fd, frame)
            seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK or 1
            params = _read_frames(fd, parser, "identify_response", 2.0)
            assert params["offset"] == len(identify_data), (
                f"unexpected offset {params['offset']} vs "
                f"{len(identify_data)}"
            )
            data = params["data"]
            if not data:
                break
            identify_data.extend(data)
        else:
            pytest.fail("identify chunked exchange did not terminate")

        assert bytes(identify_data) == IDENTIFY_BLOB
        decoded = json.loads(zlib.decompress(bytes(identify_data)).decode())
        # Every command beacon.py looks up must be present byte-for-byte.
        for cmd in BEACON_COMMANDS:
            assert cmd in decoded["commands"], (
                f"missing command in identify dict: {cmd!r}"
            )
        for resp in BEACON_RESPONSES:
            assert resp in decoded["responses"], (
                f"missing response in identify dict: {resp!r}"
            )
        assert decoded["app"] == "BeaconStub"
        assert decoded["config"]["BEACON_HAS_ACCEL"] == 0
    finally:
        os.close(fd)


def test_msgproto_frame_roundtrip_get_uptime(stub):
    """get_uptime → uptime reply with sane 64-bit clock components."""
    s, pty_path = stub
    parser = msgproto.MessageParser()
    parser.process_identify(IDENTIFY_BLOB, decompress=True)
    fd = _open_pty_writer(pty_path)
    try:
        os.write(fd, _frame(parser, 1, "get_uptime"))
        params = _read_frames(fd, parser, "uptime", 2.0)
        # Some elapsed time has passed, so clock should be > 0; high may
        # still be zero on a freshly-started stub. Both must be uint32.
        assert 0 <= params["high"] <= 0xFFFFFFFF
        assert 0 <= params["clock"] <= 0xFFFFFFFF
    finally:
        os.close(fd)


def test_get_config_flips_after_finalize_config(stub):
    """get_config returns is_config=0 first, is_config=1 after finalize."""
    s, pty_path = stub
    parser = msgproto.MessageParser()
    parser.process_identify(IDENTIFY_BLOB, decompress=True)
    fd = _open_pty_writer(pty_path)
    try:
        os.write(fd, _frame(parser, 1, "get_config"))
        params = _read_frames(fd, parser, "config", 2.0)
        assert params["is_config"] == 0
        assert params["crc"] == 0
        # Send finalize_config crc=...
        os.write(fd, _frame(parser, 2, "finalize_config crc=%u", crc=0xDEADBEEF))
        # Allow the stub to process the command.
        time.sleep(0.05)
        os.write(fd, _frame(parser, 3, "get_config"))
        params = _read_frames(fd, parser, "config", 2.0)
        assert params["is_config"] == 1
        assert params["crc"] == 0xDEADBEEF
    finally:
        os.close(fd)


def test_beacon_nvm_read_returns_image_bytes(stub):
    """A 20-byte read at offset=0 returns the model-region sentinel image."""
    s, pty_path = stub
    parser = msgproto.MessageParser()
    parser.process_identify(IDENTIFY_BLOB, decompress=True)
    fd = _open_pty_writer(pty_path)
    try:
        os.write(fd, _frame(parser, 1, "beacon_nvm_read len=%c offset=%hu",
                            len=20, offset=0))
        params = _read_frames(fd, parser, "beacon_nvm_data", 2.0)
        assert params["offset"] == 0
        assert params["bytes"] == NVM_IMAGE[:20]
        # Decoded by beacon: f_count and adc_count sentinels.
        f_count, adc_count = struct.unpack("<IH", params["bytes"][:6])
        assert f_count == 0xFFFFFFFF
        assert adc_count == 0xFFFF
    finally:
        os.close(fd)


def test_identify_dict_exposes_trsync_command_surface(stub):
    """The identify dict must include every trsync format string klippy's
    `MCU_trsync._build_config` looks up — drift breaks lookup_command at
    klippy connect time."""
    s, pty_path = stub
    parser = _parser_with_default_messages()
    fd = _open_pty_writer(pty_path)
    try:
        identify_data = bytearray()
        seq = 1
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            os.write(fd, _frame(parser, seq, "identify offset=%u count=%c",
                                offset=len(identify_data), count=40))
            seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK or 1
            params = _read_frames(fd, parser, "identify_response", 2.0)
            data = params["data"]
            if not data:
                break
            identify_data.extend(data)
        decoded = json.loads(zlib.decompress(bytes(identify_data)).decode())

        for fmt in [
            "config_trsync oid=%c",
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c",
            "trsync_set_timeout oid=%c clock=%u",
            "trsync_trigger oid=%c reason=%c",
            "stepper_stop_on_trigger oid=%c trsync_oid=%c",
        ]:
            assert fmt in decoded["commands"], (
                f"missing trsync command in identify dict: {fmt!r}"
            )
        assert (
            "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u"
            in decoded["responses"]
        )
    finally:
        os.close(fd)


def test_trsync_trigger_emits_state_with_can_trigger_zero(stub):
    """trsync_trigger oid reason → trsync_state oid can_trigger=0
    trigger_reason=reason. Mirrors src/trsync.c command_trsync_trigger."""
    s, pty_path = stub
    parser = msgproto.MessageParser()
    parser.process_identify(IDENTIFY_BLOB, decompress=True)
    fd = _open_pty_writer(pty_path)
    try:
        # Configure a trsync OID first (klippy does this via send_config
        # before any trsync_start / trsync_trigger).
        os.write(fd, _frame(parser, 1, "config_trsync oid=%c", oid=3))
        os.write(fd, _frame(parser, 2,
                            "trsync_trigger oid=%c reason=%c", oid=3, reason=2))
        params = _read_frames(fd, parser, "trsync_state", 2.0)
        assert params["oid"] == 3
        assert params["can_trigger"] == 0
        assert params["trigger_reason"] == 2
    finally:
        os.close(fd)


def test_beacon_stream_starts_emitting_status_frames(stub):
    """beacon_stream en=1 → beacon_status frames begin arriving."""
    s, pty_path = stub
    parser = msgproto.MessageParser()
    parser.process_identify(IDENTIFY_BLOB, decompress=True)
    fd = _open_pty_writer(pty_path)
    try:
        os.write(fd, _frame(parser, 1, "beacon_stream en=%u", en=1))
        params = _read_frames(fd, parser, "beacon_status", 2.0)
        assert params["frequency"] > 0
        assert params["temp"] != 0
        # And we should see the counter incrementing on a second sample.
        params2 = _read_frames(fd, parser, "beacon_status", 2.0)
        assert params2["sample"] != params["sample"]
    finally:
        os.close(fd)

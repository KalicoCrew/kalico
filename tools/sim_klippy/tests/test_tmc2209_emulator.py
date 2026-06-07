"""TMC2209 single-wire UART protocol emulator.

Logical TMC2209 datagrams (per datasheet):
  Read request:  4 bytes — 0x05 sync, slave_addr, reg_addr, CRC8
  Read response: 8 bytes — 0x05 sync, 0xFF master, reg_addr, data×4, CRC8
  Write request: 8 bytes — 0x05 sync, slave_addr, reg_addr|0x80, data×4, CRC8
                (no reply)

Wire-level reality: klippy's tmc_uart driver wraps each logical byte with
UART start (0) and stop (1) bits before sending to the firmware, so
the on-wire frame sizes are 5 (read req) and 10 (write req / read reply)
bytes. The emulator's `handle()` decodes inbound wire-format frames
back to logical bytes and re-encodes replies before returning.

The test feeds wire-format input and decodes the wire-format reply for
assertion, mirroring exactly what the firmware-side bit-bang path sees.

CRC8: polynomial 0x07, init 0, LSB-first within each byte."""

import pytest

from tools.sim_klippy.orchestrator.tmc2209_emulator import (
    TMC2209Emulator,
    _decode_uart_bits,
    _encode_uart_bits,
    crc8,
)

pytestmark = pytest.mark.sim_unit


def _wire(logical: bytes) -> bytes:
    return _encode_uart_bits(logical)


def _logical(wire: bytes) -> bytes:
    return _decode_uart_bits(wire)


def test_crc8_known_vector():
    body = bytes([0x05, 0x00, 0x00])
    logical_req = body + bytes([crc8(body)])
    chip = TMC2209Emulator(slave_addr=0)
    reply_wire = chip.handle(_wire(logical_req))
    assert len(reply_wire) == 10


def test_write_then_read_roundtrip_gconf():
    chip = TMC2209Emulator(slave_addr=0)
    write_body = bytes([0x05, 0x00, 0x00 | 0x80, 0x00, 0x00, 0x00, 0x05])
    write_logical = write_body + bytes([crc8(write_body)])
    assert chip.handle(_wire(write_logical)) == b""

    read_body = bytes([0x05, 0x00, 0x00])
    read_logical = read_body + bytes([crc8(read_body)])
    reply = _logical(chip.handle(_wire(read_logical)))
    assert len(reply) == 8
    assert reply[0] == 0x05
    assert reply[1] == 0xFF
    assert reply[2] == 0x00
    assert reply[3:7] == bytes([0, 0, 0, 5])
    assert reply[7] == crc8(reply[:7])


def test_gstat_clears_on_read():
    chip = TMC2209Emulator(slave_addr=0)
    chip._registers[0x01] = 0x07
    body = bytes([0x05, 0x00, 0x01])
    msg = _wire(body + bytes([crc8(body)]))
    first = _logical(chip.handle(msg))
    assert first[3:7] == bytes([0, 0, 0, 0x07])
    second = _logical(chip.handle(msg))
    assert second[3:7] == bytes([0, 0, 0, 0x00])


def test_wrong_slave_ignored():
    chip = TMC2209Emulator(slave_addr=0)
    body = bytes([0x05, 0x07, 0x00])
    msg = _wire(body + bytes([crc8(body)]))
    assert chip.handle(msg) == b""


def test_bad_crc_raises():
    chip = TMC2209Emulator(slave_addr=0)
    body = bytes([0x05, 0x00, 0x00])
    bad = _wire(body + bytes([0xFF]))
    with pytest.raises(ValueError, match="CRC"):
        chip.handle(bad)

"""TMC5160 5-byte SPI datagram framer + side-effects.

Datasheet §5.1: each transfer is 5 bytes. byte0 = R/W bit (MSB) | reg
addr (low 7 bits). Bytes 1-4 = data (big-endian). Read returns:
status byte | previous-read data — i.e., the data from the PREVIOUS
read, not the current one. So a single read needs two transfers."""

import pytest

from tools.sim_klippy.orchestrator.tmc5160_emulator import TMC5160Emulator

pytestmark = pytest.mark.sim_unit


def test_write_then_double_read_returns_value():
    chip = TMC5160Emulator()
    write_req = bytes([0x80 | 0x0B, 0, 0, 0, 128])
    write_reply = chip.transfer(write_req)
    assert len(write_reply) == 5

    chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    second_reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    assert second_reply[1:5] == bytes([0, 0, 0, 128])


def test_globalscaler_clamps_low():
    chip = TMC5160Emulator()
    chip.transfer(bytes([0x80 | 0x0B, 0, 0, 0, 10]))
    chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    assert reply[4] == 32


def test_globalscaler_clamps_high():
    chip = TMC5160Emulator()
    chip.transfer(bytes([0x80 | 0x0B, 0, 0, 0xFF, 0xFF]))
    chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))
    assert reply[3:5] == bytes([0, 255])


def test_gstat_clear_on_read():
    chip = TMC5160Emulator()
    chip._registers[0x01] = 0x07
    chip.transfer(bytes([0x01, 0, 0, 0, 0]))
    first = chip.transfer(bytes([0x01, 0, 0, 0, 0]))
    assert first[1:5] == bytes([0, 0, 0, 0x07])
    chip.transfer(bytes([0x01, 0, 0, 0, 0]))
    second = chip.transfer(bytes([0x01, 0, 0, 0, 0]))
    assert second[1:5] == bytes([0, 0, 0, 0])


def test_drv_status_sg_result_from_load_hook():
    chip = TMC5160Emulator()
    chip.set_load(120)
    chip.transfer(bytes([0x6F, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x6F, 0, 0, 0, 0]))
    sg = (
        reply[3] << 8 | reply[4]
    ) & 0x3FF  # 10-bit SG_RESULT lives in low bits
    assert sg == 120


def test_diag_callback_fires_on_threshold_cross():
    chip = TMC5160Emulator()
    fires = []
    chip.set_diag_callback(lambda high: fires.append(high))
    chip.set_load(200)
    chip.maybe_trigger_diag(sg_threshold=100)
    chip.set_load(50)
    chip.maybe_trigger_diag(sg_threshold=100)
    chip.set_load(200)
    chip.maybe_trigger_diag(sg_threshold=100)
    assert fires == [True, False]


def test_ihold_irun_clamps_fields():
    chip = TMC5160Emulator()
    chip.transfer(bytes([0x80 | 0x10, 0, 0, 0x40, 0x3F]))
    chip.transfer(bytes([0x10, 0, 0, 0, 0]))
    reply = chip.transfer(bytes([0x10, 0, 0, 0, 0]))
    value = reply[1] << 24 | reply[2] << 16 | reply[3] << 8 | reply[4]
    ihold = value & 0x1F
    irun = (value >> 8) & 0x1F
    assert ihold == 31
    assert irun == 31

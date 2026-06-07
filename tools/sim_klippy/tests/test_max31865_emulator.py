import pytest

from tools.sim_klippy.orchestrator.max31865_emulator import (
    CONFIG_REG,
    DEFAULT_RTD_REGISTER,
    RTD_MSB_REG,
    MAX31865Emulator,
)

pytestmark = pytest.mark.sim_unit


def test_config_register_default():
    chip = MAX31865Emulator()
    # Read 1 byte from CONFIG (addr=0x00, read = bit 7 clear).
    reply = chip.transfer(bytes([CONFIG_REG, 0x00]))
    assert reply[1] == 0x00


def test_rtd_register_read_returns_default_25c():
    chip = MAX31865Emulator()
    # 3-byte read from RTD MSB: matches the firmware's
    # thermocouple_handle_max31865 transfer layout.
    reply = chip.transfer(bytes([RTD_MSB_REG, 0x00, 0x00]))
    msb, lsb = reply[1], reply[2]
    val = (msb << 8) | lsb
    assert val == DEFAULT_RTD_REGISTER


def test_config_write_then_read_round_trips():
    chip = MAX31865Emulator()
    # Write 0xC2 to CONFIG (bias on, autoconvert, fault clear).
    chip.transfer(bytes([0x80 | CONFIG_REG, 0xC2]))
    reply = chip.transfer(bytes([CONFIG_REG, 0x00]))
    assert reply[1] == 0xC2


def test_address_auto_increments_across_payload():
    chip = MAX31865Emulator()
    chip.set_rtd_register(0xABCD)
    reply = chip.transfer(bytes([RTD_MSB_REG, 0x00, 0x00, 0x00]))
    # reply[0] = status; reply[1] = RTD_MSB, reply[2] = RTD_LSB,
    # reply[3] = HFAULT_MSB (default 0xFF).
    assert reply[1] == 0xAB
    assert reply[2] == 0xCD
    assert reply[3] == 0xFF

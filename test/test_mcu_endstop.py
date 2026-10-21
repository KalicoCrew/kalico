import pytest

from klippy import mcu as mcu_mod


class _CommandError(Exception):
    pass


class _FakePrinter:
    command_error = _CommandError


class _FakeMCU:
    def __init__(self):
        self.non_critical_disconnected = True

    def get_printer(self):
        return _FakePrinter()

    def get_name(self):
        return "toolboard"


def _make_endstop():
    endstop = mcu_mod.MCU_endstop.__new__(mcu_mod.MCU_endstop)
    endstop._mcu = _FakeMCU()
    endstop._home_cmd = endstop._query_cmd = None
    return endstop


def test_query_endstop_on_disconnected_mcu_raises_command_error():
    with pytest.raises(_CommandError, match="disconnected MCU 'toolboard'"):
        _make_endstop().query_endstop(0.0)


def test_home_start_on_disconnected_mcu_raises_command_error():
    with pytest.raises(_CommandError, match="disconnected MCU 'toolboard'"):
        _make_endstop().home_start(0.0, 0.0, 1, 0.0)

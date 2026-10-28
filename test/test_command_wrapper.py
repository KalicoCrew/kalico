import pytest

from klippy import gcode, mcu, serialhdl


class _FakeCmd:
    def encode(self, data):
        return b""


class _DisconnectedSerial:
    def raw_send(self, *args):
        raise serialhdl.error("mcu 'adxl': non-critical MCU is disconnected")

    raw_send_wait_ack = raw_send


@pytest.mark.parametrize("method", ["send", "send_wait_ack"])
def test_send_errors_are_command_errors(method):
    # A send to a disconnected non-critical mcu must raise a command error
    # so callers (e.g. bulk_sensor stop callbacks) can handle it
    cw = mcu.CommandWrapper.__new__(mcu.CommandWrapper)
    cw._serial = _DisconnectedSerial()
    cw._error = gcode.CommandError
    cw._cmd = _FakeCmd()
    cw._cmd_queue = None
    with pytest.raises(gcode.CommandError):
        getattr(cw, method)()

import pytest

from klippy.extras import tools_calibrate


class _GCodeError(Exception):
    pass


class _FakeGCode:
    error = _GCodeError


class _FakeReactor:
    def monotonic(self):
        return 0.0


class _FakeKinematics:
    def __init__(self, status):
        self._status = status

    def get_status(self, eventtime):
        return self._status


class _FakeToolhead:
    def __init__(self, kin_status):
        self._kin = _FakeKinematics(kin_status)

    def get_status(self, eventtime):
        return {"homed_axes": "xyz"}

    def get_position(self):
        return [10.0, 10.0, 10.0, 0.0]

    def get_kinematics(self):
        return self._kin


class _FakePrinter:
    command_error = _GCodeError

    def __init__(self, toolhead):
        self._toolhead = toolhead

    def lookup_object(self, name):
        assert name == "toolhead"
        return self._toolhead

    def get_reactor(self):
        return _FakeReactor()


def _make_probe(kin_status):
    probe = tools_calibrate.PrinterProbeMultiAxis.__new__(
        tools_calibrate.PrinterProbeMultiAxis
    )
    probe.printer = _FakePrinter(_FakeToolhead(kin_status))
    probe.gcode = _FakeGCode()
    return probe


def test_kinematics_without_axis_maximum_raises_friendly_error():
    probe = _make_probe({"axis_minimum": [0.0, 0.0, 0.0]})
    with pytest.raises(_GCodeError, match="cartesian"):
        probe._get_target_position(0, 1, 5.0)


def test_kinematics_without_axis_minimum_raises_friendly_error():
    probe = _make_probe({"axis_maximum": [200.0, 200.0, 200.0]})
    with pytest.raises(_GCodeError, match="cartesian"):
        probe._get_target_position(0, -1, 5.0)

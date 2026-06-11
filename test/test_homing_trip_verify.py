import pytest

from klippy.extras.homing import (
    _homed_axis_position,
    _no_trigger_error_message,
    _verify_latched_trip,
)
from klippy.extras.sim_remote_endstop import trip_to_stop_travel


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


class FakeLatchEndstop:
    def __init__(self, tripped, trip_clock):
        self._state = {"tripped": tripped, "trip_clock": trip_clock}

    def query_trip_state(self):
        return dict(self._state)


class FakeRemoteEndstop:
    pass


def test_verify_passes_on_matching_low32():
    es = FakeLatchEndstop(True, 0xDEADBEEF)
    _verify_latched_trip(FakeGcmd(), 2, es, 0x1_DEAD_BEEF)


def test_verify_raises_on_clock_mismatch():
    es = FakeLatchEndstop(True, 0x1111)
    with pytest.raises(RuntimeError, match="latch/doorbell clock mismatch"):
        _verify_latched_trip(FakeGcmd(), 2, es, 0x2222)


def test_verify_raises_when_latch_not_tripped():
    es = FakeLatchEndstop(False, 0)
    with pytest.raises(RuntimeError, match="latch shows no trip"):
        _verify_latched_trip(FakeGcmd(), 2, es, 0x2222)


def test_verify_skips_endstops_without_latch():
    _verify_latched_trip(FakeGcmd(), 2, FakeRemoteEndstop(), 0x2222)


def test_no_trigger_message_plain():
    msg = _no_trigger_error_message(2, FakeLatchEndstop(False, 0), 40.0)
    assert "did not trigger within 40.0mm" in msg
    assert "doorbell" not in msg


def test_no_trigger_message_reports_lost_doorbell():
    msg = _no_trigger_error_message(2, FakeLatchEndstop(True, 1234), 40.0)
    assert "trip event was lost" in msg
    assert "1234" in msg


def test_no_trigger_message_remote_endstop():
    msg = _no_trigger_error_message(2, FakeRemoteEndstop(), 40.0)
    assert "did not trigger within 40.0mm" in msg


class FakeProviderNoHook:
    pass


class FakeProviderMeasures:
    def measured_trip_position(self, axis, trip_pos, final_pos):
        return 3.25


class FakeProviderDeclines:
    def measured_trip_position(self, axis, trip_pos, final_pos):
        return None


def test_homed_position_default_is_trigger_height_plus_overshoot():
    pos = _homed_axis_position(
        FakeProviderNoHook(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == pytest.approx(0.5 + (0.9 - 1.0))


def test_homed_position_none_provider_uses_default():
    pos = _homed_axis_position(None, 2, [0, 0, 1.0], [0, 0, 0.9], 0.5)
    assert pos == pytest.approx(0.4)


def test_homed_position_uses_provider_measurement():
    pos = _homed_axis_position(
        FakeProviderMeasures(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == 3.25


def test_homed_position_provider_declining_falls_back():
    pos = _homed_axis_position(
        FakeProviderDeclines(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == pytest.approx(0.4)


def test_trip_to_stop_travel_homing_down():
    travel = trip_to_stop_travel(2, [0, 0, 10.0], [0, 0, 3.3], [0, 0, 3.2])
    assert travel == pytest.approx(0.1)


def test_trip_to_stop_travel_homing_up():
    travel = trip_to_stop_travel(2, [0, 0, 0.0], [0, 0, 4.9], [0, 0, 5.0])
    assert travel == pytest.approx(0.1)


def test_trip_to_stop_travel_negative_when_trip_lands_past_stop():
    travel = trip_to_stop_travel(2, [0, 0, 10.0], [0, 0, 3.1], [0, 0, 3.2])
    assert travel == pytest.approx(-0.1)


def test_trip_to_stop_travel_zero_when_trip_equals_stop():
    travel = trip_to_stop_travel(2, [0, 0, 10.0], [0, 0, 3.2], [0, 0, 3.2])
    assert travel == 0.0

import pytest

from klippy.extras.homing import (
    _no_trigger_error_message,
    _verify_latched_trip,
)


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

import logging

import pytest

from klippy import structured_log
from klippy.extras import log_observability as lo


class CaptureHandler(logging.Handler):
    def __init__(self):
        super().__init__()
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture(autouse=True)
def _reset():
    structured_log.clear_print()
    structured_log.bind_session("k-test-1")
    yield
    structured_log.clear_session()
    structured_log.clear_print()


def test_heartbeat_emits_observability_event():
    cap = CaptureHandler()
    evlog = logging.getLogger("kalico.event")
    # In a bare test env the root logger defaults to WARNING, which would
    # filter the INFO heartbeat before it reaches the handler. klippy sets the
    # level at startup; here we lower it explicitly to observe the record.
    prev_level = evlog.level
    evlog.setLevel(logging.DEBUG)
    evlog.addHandler(cap)
    try:
        lo.emit_heartbeat()
    finally:
        evlog.removeHandler(cap)
        evlog.setLevel(prev_level)
    rec = next(
        r for r in cap.records if getattr(r, "event", None) == "heartbeat"
    )
    assert rec.subsystem == "observability"


def test_lag_within_threshold_is_ok():
    assert lo.check_lag(bytes_behind=1024, threshold=1_048_576) is False


def test_lag_over_threshold_is_flagged():
    assert lo.check_lag(bytes_behind=5_000_000, threshold=1_048_576) is True


def test_lag_at_threshold_is_not_stale():
    # boundary: exactly at threshold is not yet stale (strictly greater)
    assert lo.check_lag(bytes_behind=1_048_576, threshold=1_048_576) is False


if __name__ == "__main__":
    import sys

    sys.exit(pytest.main([__file__, "-v"]))

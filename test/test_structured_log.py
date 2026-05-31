# test/test_structured_log.py
import logging
import sys

import pytest

from klippy import structured_log as sl


@pytest.fixture(autouse=True)
def _reset_log_context():
    # The session/print contextvars are module-level globals; reset them
    # before AND after every test so the suite is order-independent.
    sl.clear_session()
    sl.clear_print()
    yield
    sl.clear_session()
    sl.clear_print()


def test_level_name_maps_stdlib_levels():
    assert sl.level_name(logging.DEBUG) == "debug"
    assert sl.level_name(logging.INFO) == "info"
    assert sl.level_name(logging.WARNING) == "warn"
    assert sl.level_name(logging.ERROR) == "error"
    assert sl.level_name(logging.CRITICAL) == "error"
    assert sl.level_name(sl.TRACE_LEVEL) == "trace"


def test_format_time_is_rfc3339_utc_millis_z():
    # 2026-05-31T00:00:00Z == 1780185600 (UTC)
    out = sl.format_time(1780185600.0)
    assert out == "2026-05-31T00:00:00.000Z"


def test_session_bind_and_get():
    sl.bind_session("k-1779840000-4242")
    assert sl.get_session() == "k-1779840000-4242"


def test_print_bind_clear_default_empty():
    sl.clear_print()
    assert sl.get_print() == ""
    sl.bind_print("print-123")
    assert sl.get_print() == "print-123"
    sl.clear_print()
    assert sl.get_print() == ""


def test_make_session_id_shape():
    sid = sl.make_session_id()
    parts = sid.split("-")
    assert parts[0] == "k"
    assert parts[1].isdigit() and parts[2].isdigit()


def test_get_session_unbound_is_sentinel():
    # The autouse fixture has cleared the session var, so an unbound read
    # returns the queryable sentinel rather than crashing.
    sl.clear_session()
    assert sl.get_session() == sl.UNBOUND_SESSION

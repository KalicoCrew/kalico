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


def _make_record(msg="hello", level=logging.INFO, name="mod.Cls", **extra):
    rec = logging.LogRecord(
        name=name, level=level, pathname=__file__, lineno=1,
        msg=msg, args=(), exc_info=None,
    )
    rec.created = 1780185600.0
    rec.session_id = "k-1779840000-1"
    rec.print_id = ""
    rec.source = sl.SOURCE_HOST_PY
    for k, v in extra.items():
        setattr(rec, k, v)
    return rec


def test_record_to_dict_core_fields():
    rec = _make_record()
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert d["_time"] == "2026-05-31T00:00:00.000Z"
    assert d["_msg"] == "hello"
    assert d["level"] == "info"
    assert d["source"] == "host-py"
    assert d["session_id"] == "k-1779840000-1"
    assert d["target"] == "mod.Cls"
    assert d["print_id"] == ""  # empty allowed


def test_record_to_dict_promotes_extra_fields():
    rec = _make_record(subsystem="homing", event="homing.trip", axis="z")
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert d["subsystem"] == "homing"
    assert d["event"] == "homing.trip"
    assert d["axis"] == "z"


def test_record_to_dict_captures_exception_traceback():
    # logging.exception() / exc_info populates record.exc_text during format();
    # the traceback must survive into the JSONL schema, not be dropped.
    try:
        raise ValueError("boom")
    except ValueError:
        rec = logging.LogRecord(
            "mod.Cls", logging.ERROR, __file__, 1,
            "handler failed", (), sys.exc_info(),
        )
    rec.created = 1780185600.0
    rec.session_id = "k-1779840000-1"
    rec.print_id = ""
    rec.source = sl.SOURCE_HOST_PY
    # Formatter.format() is what sets exc_text; emulate the QueueHandler path.
    logging.Formatter().format(rec)
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert "ValueError: boom" in d["exception"]
    assert "Traceback" in d["exception"]


import json


def test_serialize_is_single_line_and_round_trips():
    rec = _make_record(msg="line1\nline2\twith \"quote\" and \x01 ctrl")
    rec.message = rec.getMessage()
    line = sl.serialize_record(sl.record_to_dict(rec))
    # Exactly one physical line (trailing newline only).
    assert line.endswith("\n")
    assert line.count("\n") == 1
    obj = json.loads(line)
    assert obj["_msg"] == "line1\nline2\twith \"quote\" and \x01 ctrl"


def test_serialize_handles_nonjson_value():
    rec = _make_record(weird=object())
    rec.message = rec.getMessage()
    # Must not raise; non-serializable value is stringified.
    line = sl.serialize_record(sl.record_to_dict(rec))
    assert "weird" in json.loads(line)

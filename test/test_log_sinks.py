# test/test_log_sinks.py
import logging

import pytest

from klippy import log_sinks


class _RecordingSink(log_sinks.Sink):
    def __init__(self):
        self.records = []
        self.closed = False

    def emit_record(self, record):
        self.records.append(record)

    def close(self):
        self.closed = True


def _rec(msg="m"):
    r = logging.LogRecord("t", logging.INFO, __file__, 1, msg, (), None)
    r.message = msg
    return r


def test_registry_fans_out_to_all_sinks():
    a, b = _RecordingSink(), _RecordingSink()
    reg = log_sinks.SinkRegistry([a, b])
    reg.emit(_rec("hi"))
    assert len(a.records) == 1 and len(b.records) == 1


def test_registry_close_closes_all():
    a, b = _RecordingSink(), _RecordingSink()
    reg = log_sinks.SinkRegistry([a, b])
    reg.close()
    assert a.closed and b.closed


class _BoomSink(log_sinks.Sink):
    def emit_record(self, record):
        raise OSError("disk full")


def test_registry_emit_failure_is_loud():
    # Per spec §12 a write failure is a hard error, not silently swallowed.
    reg = log_sinks.SinkRegistry([_BoomSink()])
    with pytest.raises(OSError):
        reg.emit(_rec())


def test_text_sink_writes_stock_message_only(tmp_path):
    path = str(tmp_path / "klippy.log")
    sink = log_sinks.TextSink(path, rotate_log_at_restart=False)
    sink.emit_record(_rec("Stats 1.0: a=1 b=2"))
    sink.close()
    with open(path) as f:
        content = f.read()
    assert content == "Stats 1.0: a=1 b=2\n"  # no level/time prefix


def test_text_sink_rollover_info_header(tmp_path):
    path = str(tmp_path / "klippy.log")
    sink = log_sinks.TextSink(path, rotate_log_at_restart=False)
    sink.set_rollover_info("versions", "Git version: 'abc'")
    sink.do_rollover()
    sink.emit_record(_rec("after"))
    sink.close()
    with open(path) as f:
        content = f.read()
    assert "Git version: 'abc'" in content
    assert "Log rollover at" in content

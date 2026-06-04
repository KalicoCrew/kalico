# test/test_log_sinks.py
import json
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


def test_jsonl_sink_writes_valid_json_line(tmp_path):
    path = str(tmp_path / "host-py.jsonl")
    sink = log_sinks.JsonlSink(path)
    r = _rec("endstop trip")
    r.session_id = "k-1-2"
    r.print_id = ""
    r.source = "host-py"
    r.subsystem = "homing"
    sink.emit_record(r)
    sink.close()
    with open(path) as f:
        lines = f.readlines()
    assert len(lines) == 1
    obj = json.loads(lines[0])
    assert obj["_msg"] == "endstop trip"
    assert obj["subsystem"] == "homing"
    assert obj["source"] == "host-py"


def test_jsonl_sink_flushes_each_record(tmp_path):
    # Relaxed durability: each record is flushed to the OS immediately so it
    # is readable without closing the file (spec §3/§7 durability contract).
    path = str(tmp_path / "host-py.jsonl")
    sink = log_sinks.JsonlSink(path)
    sink.emit_record(_rec("one"))
    with open(path) as f:
        assert f.read().count("\n") == 1  # visible before close
    sink.close()


def test_jsonl_sink_periodic_fsync_backstop(tmp_path, monkeypatch):
    # Spec §3/§7: relaxed default = flush-per-record + a periodic fsync
    # backstop. With interval 0 every emit fsyncs; close() always fsyncs.
    calls = []
    monkeypatch.setattr(log_sinks.os, "fsync", lambda fd: calls.append(fd))
    path = str(tmp_path / "host-py.jsonl")
    sink = log_sinks.JsonlSink(path, fsync_interval=0.0)
    sink.emit_record(_rec("a"))
    sink.emit_record(_rec("b"))
    assert len(calls) >= 2  # fsynced on each emit when interval elapsed
    pre_close = len(calls)
    sink.close()
    assert len(calls) == pre_close + 1  # final fsync on close


def test_jsonl_sink_default_interval_does_not_fsync_every_record(
    tmp_path, monkeypatch
):
    calls = []
    monkeypatch.setattr(log_sinks.os, "fsync", lambda fd: calls.append(fd))
    path = str(tmp_path / "host-py.jsonl")
    sink = log_sinks.JsonlSink(path)  # default interval (>0)
    sink.emit_record(_rec("a"))
    sink.emit_record(_rec("b"))
    # The two back-to-back emits are within the interval: no per-record fsync.
    assert calls == []
    sink.close()  # close still fsyncs once
    assert len(calls) == 1

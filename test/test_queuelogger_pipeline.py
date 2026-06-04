# test/test_queuelogger_pipeline.py
import json
import logging
import os
import queue

import pytest

from klippy import queuelogger, structured_log


def test_queue_handler_raises_on_overflow():
    q = queue.Queue(maxsize=1)
    h = queuelogger.QueueHandler(q)
    r1 = logging.LogRecord("t", logging.INFO, __file__, 1, "a", (), None)
    r2 = logging.LogRecord("t", logging.INFO, __file__, 1, "b", (), None)
    h.emit(r1)  # fills the queue
    with pytest.raises(queuelogger.LogQueueOverflow):
        h.emit(r2)  # overflow must be loud, not silently dropped


def test_setup_bg_logging_writes_both_files(tmp_path):
    structured_log.bind_session("k-1779840000-99")
    structured_log.clear_print()
    klippy_log = str(tmp_path / "klippy.log")
    events_dir = str(tmp_path / "events")
    ql = queuelogger.setup_bg_logging(
        filename=klippy_log,
        debuglevel=logging.INFO,
        rotate_log_at_restart=False,
        events_dir=events_dir,
    )
    try:
        log = logging.getLogger("test.pipeline")
        log.info("hello structured world")
    finally:
        # flush the bg thread
        ql.stop()
        queuelogger.clear_bg_logging()

    # stock text log has the raw message
    with open(klippy_log) as f:
        assert "hello structured world" in f.read()
    # structured jsonl has a schema record
    jsonl = os.path.join(events_dir, "host-py.jsonl")
    with open(jsonl) as f:
        lines = [json.loads(x) for x in f if x.strip()]
    rec = [r for r in lines if r["_msg"] == "hello structured world"][0]
    assert rec["source"] == "host-py"
    assert rec["session_id"] == "k-1779840000-99"
    assert rec["target"] == "test.pipeline"


def test_stop_is_clean_on_normal_path(tmp_path):
    structured_log.bind_session("k-1779840000-99")
    structured_log.clear_print()
    klippy_log = str(tmp_path / "klippy.log")
    events_dir = str(tmp_path / "events")
    ql = queuelogger.setup_bg_logging(
        filename=klippy_log,
        debuglevel=logging.INFO,
        rotate_log_at_restart=False,
        events_dir=events_dir,
    )
    try:
        logging.getLogger("test.pipeline").info("clean stop record")
        # A normal stop() must flush and return without raising.
        ql.stop()
    finally:
        queuelogger.clear_bg_logging()
    with open(klippy_log) as f:
        assert "clean stop record" in f.read()


class _RaisingSink:
    # Minimal failing sink: emit_record blows up (e.g. disk full), close()
    # is a durable-flush no-op that must still run during stop().
    def __init__(self):
        self.closed = False

    def emit_record(self, record):
        raise RuntimeError("sink exploded")

    def close(self):
        self.closed = True


def _wait_thread_dead(ql, timeout=2.0):
    ql.bg_thread.join(timeout=timeout)


def test_stop_resurfaces_bg_sink_failure():
    from klippy import log_sinks

    sink = _RaisingSink()
    ql = queuelogger.QueueListener.__new__(queuelogger.QueueListener)
    ql.registry = log_sinks.SinkRegistry([sink])
    ql.bg_queue = queue.Queue(maxsize=queuelogger.LOG_QUEUE_MAXSIZE)
    ql._bg_exc = None
    import threading

    ql.bg_thread = threading.Thread(target=ql._bg_thread)
    ql.bg_thread.start()

    r = logging.LogRecord("t", logging.INFO, __file__, 1, "boom", (), None)
    ql.bg_queue.put(r)
    # The sink raises; the bg thread must store the cause and exit.
    _wait_thread_dead(ql)
    assert not ql.bg_thread.is_alive()

    with pytest.raises(RuntimeError, match="sink exploded"):
        ql.stop()
    # registry.close() must run even when re-raising the stored cause.
    assert sink.closed


def test_stop_does_not_hang_when_thread_dead_and_queue_full():
    from klippy import log_sinks

    sink = _RaisingSink()
    ql = queuelogger.QueueListener.__new__(queuelogger.QueueListener)
    ql.registry = log_sinks.SinkRegistry([sink])
    # Tiny queue so we can saturate it after the thread has died.
    ql.bg_queue = queue.Queue(maxsize=2)
    ql._bg_exc = None
    import threading

    ql.bg_thread = threading.Thread(target=ql._bg_thread)
    ql.bg_thread.start()

    r = logging.LogRecord("t", logging.INFO, __file__, 1, "boom", (), None)
    ql.bg_queue.put(r)
    _wait_thread_dead(ql)
    assert not ql.bg_thread.is_alive()
    # Saturate the now-undrained queue so a blocking put(None) would hang.
    ql.bg_queue.put_nowait(
        logging.LogRecord("t", logging.INFO, __file__, 1, "x", (), None)
    )
    ql.bg_queue.put_nowait(
        logging.LogRecord("t", logging.INFO, __file__, 1, "y", (), None)
    )
    # stop() must not hang; it surfaces the stored sink failure instead.
    with pytest.raises(RuntimeError, match="sink exploded"):
        ql.stop()
    assert sink.closed


def test_bg_sink_failure_emits_last_gasp(capsys, tmp_path):
    # A mid-run sink failure must be surfaced PROACTIVELY (spec §16 item 11):
    # a last-gasp to stderr (journald-captured on the Pi) plus the human text
    # log, not only at the next enqueue / at shutdown.
    structured_log.bind_session("k-1-1")
    structured_log.clear_print()
    klippy_log = str(tmp_path / "klippy.log")
    ql = queuelogger.QueueListener(
        filename=klippy_log,
        rotate_log_at_restart=False,
        events_dir=None,
    )
    # Register a sink that blows up; the healthy text sink is still first, so it
    # both writes the record AND receives the last-gasp.
    ql.registry.register(_RaisingSink())
    r = logging.LogRecord("t", logging.INFO, __file__, 1, "boom", (), None)
    ql.bg_queue.put(r)
    _wait_thread_dead(ql)
    assert isinstance(ql._bg_exc, RuntimeError)

    with pytest.raises(RuntimeError, match="sink exploded"):
        ql.stop()

    # last-gasp reached stderr (the reliable operator channel)...
    assert "kalico log sink failed" in capsys.readouterr().err
    # ...and the human-readable klippy.log (the failed sink was not the text one)
    with open(klippy_log) as fh:
        text = fh.read()
    assert "kalico log sink failed" in text

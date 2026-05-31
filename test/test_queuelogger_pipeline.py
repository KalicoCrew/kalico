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

# test/test_queuelogger_pipeline.py
import logging
import queue

import pytest

from klippy import queuelogger


def test_queue_handler_raises_on_overflow():
    q = queue.Queue(maxsize=1)
    h = queuelogger.QueueHandler(q)
    r1 = logging.LogRecord("t", logging.INFO, __file__, 1, "a", (), None)
    r2 = logging.LogRecord("t", logging.INFO, __file__, 1, "b", (), None)
    h.emit(r1)  # fills the queue
    with pytest.raises(queuelogger.LogQueueOverflow):
        h.emit(r2)  # overflow must be loud, not silently dropped

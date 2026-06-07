# Code to implement asynchronous logging from a background thread
#
# Copyright (C) 2016-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import queue
import sys
import threading

from . import log_sinks, structured_log

LOG_QUEUE_MAXSIZE = 100000


class LogQueueOverflow(Exception):
    pass


# Class to forward all messages through a queue to a background thread
class QueueHandler(logging.Handler):
    def __init__(self, queue):
        logging.Handler.__init__(self)
        self.queue = queue

    def emit(self, record):
        try:
            self.format(record)
            record.msg = record.message
            record.args = None
            record.exc_info = None
        except Exception:
            self.handleError(record)
            return
        try:
            self.queue.put_nowait(record)
        except queue.Full:
            raise LogQueueOverflow(
                "klippy log queue overflow; logging cannot keep up"
            )


class QueueListener:
    def __init__(self, filename, rotate_log_at_restart, events_dir):
        self._text = log_sinks.TextSink(filename, rotate_log_at_restart)
        sinks = [self._text]
        self._jsonl = None
        if events_dir:
            os.makedirs(events_dir, exist_ok=True)
            self._jsonl = log_sinks.JsonlSink(
                os.path.join(events_dir, "host-py.jsonl")
            )
            sinks.append(self._jsonl)
        self.registry = log_sinks.SinkRegistry(sinks)
        self.bg_queue = queue.Queue(maxsize=LOG_QUEUE_MAXSIZE)
        self._bg_exc = None
        self.bg_thread = threading.Thread(target=self._bg_thread)
        self.bg_thread.start()

    def _bg_thread(self):
        while True:
            record = self.bg_queue.get(True)
            if record is None:
                break
            try:
                self.registry.emit(record)
            except Exception as e:
                self._bg_exc = e
                self._last_gasp(e)
                break

    def _last_gasp(self, exc):
        msg = (
            "FATAL: kalico log sink failed; structured logging stopped: %r"
            % (exc,)
        )
        try:
            sys.stderr.write(msg + "\n")
            sys.stderr.flush()
        except Exception:
            pass
        try:
            rec = logging.makeLogRecord(
                {
                    "msg": msg,
                    "levelno": logging.CRITICAL,
                    "levelname": "CRITICAL",
                    "name": "kalico.observability",
                }
            )
            self._text.emit_record(rec)
        except Exception:
            pass

    def stop(self):
        if self.bg_thread.is_alive():
            try:
                self.bg_queue.put(None, timeout=5.0)
            except queue.Full:
                # bg thread is wedged/dead and the queue is saturated;
                # don't block shutdown forever.
                pass
            self.bg_thread.join(timeout=5.0)
        self.registry.close()
        if self._bg_exc is not None:
            raise self._bg_exc

    def set_rollover_info(self, name, info):
        self._text.set_rollover_info(name, info)

    def clear_rollover_info(self):
        self._text.clear_rollover_info()

    def doRollover(self):
        self._text.do_rollover()


MainQueueHandler = None


def setup_bg_logging(
    filename, debuglevel, rotate_log_at_restart, events_dir=None
):
    global MainQueueHandler
    ql = QueueListener(
        filename=filename,
        rotate_log_at_restart=rotate_log_at_restart,
        events_dir=events_dir,
    )
    MainQueueHandler = QueueHandler(ql.bg_queue)
    MainQueueHandler.addFilter(structured_log.ContextFilter())
    root = logging.getLogger()
    root.addHandler(MainQueueHandler)
    root.setLevel(debuglevel)
    return ql


def clear_bg_logging():
    global MainQueueHandler
    if MainQueueHandler is not None:
        root = logging.getLogger()
        root.removeHandler(MainQueueHandler)
        root.setLevel(logging.WARNING)
        MainQueueHandler = None

import logging
import logging.handlers
import os
import time

from . import structured_log


class Sink:
    def emit_record(self, record):
        raise NotImplementedError

    def close(self):
        pass


class SinkRegistry:
    def __init__(self, sinks=None):
        self._sinks = list(sinks or [])

    def register(self, sink):
        self._sinks.append(sink)

    def emit(self, record):
        for sink in self._sinks:
            sink.emit_record(record)

    def close(self):
        for sink in self._sinks:
            sink.close()


class TextSink(Sink):
    def __init__(self, filename, rotate_log_at_restart):
        if rotate_log_at_restart:
            self._handler = logging.handlers.TimedRotatingFileHandler(
                filename, when="S", interval=60 * 60 * 24, backupCount=5
            )
        else:
            self._handler = logging.handlers.TimedRotatingFileHandler(
                filename, when="midnight", backupCount=5
            )
        self._rollover_info = {}

    def emit_record(self, record):
        self._handler.emit(record)

    def set_rollover_info(self, name, info):
        if info is None:
            self._rollover_info.pop(name, None)
        else:
            self._rollover_info[name] = info

    def clear_rollover_info(self):
        self._rollover_info.clear()

    def do_rollover(self):
        self._handler.doRollover()
        lines = [
            self._rollover_info[name] for name in sorted(self._rollover_info)
        ]
        lines.append(
            "=============== Log rollover at %s ==============="
            % (time.asctime(),)
        )
        self._handler.emit(
            logging.makeLogRecord(
                {"msg": "\n".join(lines), "levelno": logging.INFO}
            )
        )

    def close(self):
        self._handler.close()


JSONL_MAX_BYTES = 32 * 1024 * 1024
JSONL_BACKUP_COUNT = 5
JSONL_FSYNC_INTERVAL = 15.0


class JsonlSink(Sink):
    def __init__(
        self,
        filename,
        max_bytes=JSONL_MAX_BYTES,
        backup_count=JSONL_BACKUP_COUNT,
        fsync_interval=JSONL_FSYNC_INTERVAL,
    ):
        os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
        self._handler = logging.handlers.RotatingFileHandler(
            filename,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
            delay=False,
        )
        self._fsync_interval = fsync_interval
        self._last_fsync = time.monotonic()

    def emit_record(self, record):
        line = structured_log.serialize_record(
            structured_log.record_to_dict(record)
        )
        stream = self._handler.stream
        if self._handler.shouldRollover(record):
            self._handler.doRollover()
            stream = self._handler.stream
            self._last_fsync = time.monotonic()
        stream.write(line)
        stream.flush()
        now = time.monotonic()
        if now - self._last_fsync >= self._fsync_interval:
            os.fsync(stream.fileno())
            self._last_fsync = now

    def close(self):
        stream = self._handler.stream
        if stream is not None:
            stream.flush()
            os.fsync(stream.fileno())
        self._handler.close()

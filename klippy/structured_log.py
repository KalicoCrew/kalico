# Structured logging schema, context, and forward helper for the klippy host.
#
# This module never imports heavy klippy objects so it can be used from the
# earliest point in startup (before the reactor/printer exist).
import datetime
import json
import logging
import os
import shutil
import time

# A "trace" level below DEBUG (stdlib has no TRACE).
TRACE_LEVEL = 5
logging.addLevelName(TRACE_LEVEL, "TRACE")

SOURCE_HOST_PY = "host-py"

_LEVEL_MAP = {
    TRACE_LEVEL: "trace",
    logging.DEBUG: "debug",
    logging.INFO: "info",
    logging.WARNING: "warn",
    logging.ERROR: "error",
    logging.CRITICAL: "error",
}


def level_name(levelno):
    # Map an arbitrary numeric level to the nearest schema level name.
    if levelno in _LEVEL_MAP:
        return _LEVEL_MAP[levelno]
    if levelno >= logging.ERROR:
        return "error"
    if levelno >= logging.WARNING:
        return "warn"
    if levelno >= logging.INFO:
        return "info"
    if levelno >= logging.DEBUG:
        return "debug"
    return "trace"


def format_time(created):
    # RFC3339 UTC with millisecond precision and a trailing 'Z'.
    dt = datetime.datetime.fromtimestamp(created, tz=datetime.timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (
        dt.microsecond // 1000,
    )


UNBOUND_SESSION = "__unbound__"

# Session/print correlation is process-wide: the session is bound once at
# startup, the print id is set/cleared per print — never concurrent. We use
# plain module globals rather than `contextvars`, because klippy's reactor runs
# callbacks in greenlets and `greenlet` gives each greenlet its OWN
# `contextvars.Context`. A ContextVar bound in main() is therefore invisible to
# records emitted from reactor greenlets (observed live: the bound session
# reached only main()-context records; everything from the reactor carried the
# UNBOUND sentinel). A module global is visible across all greenlets/threads —
# which is exactly what this correlation needs. Simple assignment + read is
# GIL-atomic; no lock required.
_session_id = None
_print_id = ""


def make_session_id():
    return "k-%d-%d" % (int(time.time()), os.getpid())


def bind_session(session_id):
    global _session_id
    _session_id = session_id


def clear_session():
    global _session_id
    _session_id = None


def get_session():
    return UNBOUND_SESSION if _session_id is None else _session_id


def bind_print(print_id):
    global _print_id
    _print_id = print_id


def clear_print():
    global _print_id
    _print_id = ""


def get_print():
    return _print_id


# LogRecord attributes that are stdlib bookkeeping, not schema payload.
_STD_ATTRS = frozenset(
    logging.LogRecord("x", logging.INFO, "x", 0, "x", (), None).__dict__.keys()
) | {"message", "asctime", "session_id", "print_id", "source", "taskName"}

# Schema fields that get a dedicated slot (everything else is free payload).
_RESERVED_OUT = frozenset(
    ["_time", "_msg", "level", "source", "session_id", "target", "print_id"]
)


def record_to_dict(record):
    # `record.message` must already be set (QueueHandler does this).
    msg = getattr(record, "message", None)
    if msg is None:
        msg = record.getMessage()
    out = {
        "_time": format_time(record.created),
        "_msg": msg,
        "level": level_name(record.levelno),
        "source": getattr(record, "source", SOURCE_HOST_PY),
        "session_id": getattr(record, "session_id", UNBOUND_SESSION),
        "target": getattr(record, "target", record.name),
    }
    print_id = getattr(record, "print_id", "")
    out["print_id"] = print_id if print_id else ""
    # Capture the formatted exception traceback if present. QueueHandler clears
    # record.exc_info but the Formatter has already rendered record.exc_text,
    # which would otherwise be dropped (it lives in stdlib's _STD_ATTRS).
    exc_text = getattr(record, "exc_text", None)
    if exc_text:
        out["exception"] = exc_text
    # Promote any non-standard attribute (from logging `extra=`) to a field.
    for key, val in record.__dict__.items():
        if key in _STD_ATTRS or key in _RESERVED_OUT:
            continue
        if key.startswith("_"):
            continue
        out[key] = val
    return out


def serialize_record(record_dict):
    # json.dumps escapes embedded newlines/quotes/control chars, guaranteeing
    # exactly one physical line per record (NDJSON-safe; injection-safe for
    # user-controlled values such as gcode comments / M117 text).
    line = json.dumps(
        record_dict, ensure_ascii=False, separators=(",", ":"), default=repr
    )
    return line + "\n"


class ContextFilter(logging.Filter):
    # Injected on the root logger; reads the process-global session/print, so
    # it is correct from any thread or reactor greenlet. Never raises (raising
    # inside logging is unsafe); an unbound session shows up as the queryable
    # UNBOUND_SESSION sentinel, which the startup ordering invariant
    # (printer.main) keeps us off in normal operation.
    def filter(self, record):
        if not hasattr(record, "session_id"):
            record.session_id = get_session()
        if not hasattr(record, "print_id"):
            record.print_id = get_print()
        if not hasattr(record, "source"):
            record.source = SOURCE_HOST_PY
        if not hasattr(record, "target"):
            record.target = record.name
        return True


def make_print_id():
    return "print-%d" % (int(time.time()),)


_event_logger = logging.getLogger("kalico.event")


def event(subsystem, event, *, level=logging.INFO, msg=None, **fields):
    # Forward helper for NEW / hot-path code: emits a fully structured record.
    # Fail loudly (per project policy) if the required fields are missing.
    if not subsystem or not event:
        raise ValueError(
            "structured_log.event requires non-empty subsystem and event"
        )
    extra = {"subsystem": subsystem, "event": event}
    extra.update(fields)
    _event_logger.log(level, msg if msg is not None else event, extra=extra)


# Default reserve below which we refuse to start (spec §16 tunable).
LOG_SPACE_RESERVE_BYTES = 64 * 1024 * 1024


class LogSpaceError(Exception):
    pass


def check_log_space(path, reserve_bytes=LOG_SPACE_RESERVE_BYTES):
    # Pure check: never creates directories (that is the caller's job). Probe
    # the nearest existing ancestor of `path` for free space, so the preflight
    # works even when the logs directory has not been created yet.
    probe = os.path.abspath(path)
    while not os.path.isdir(probe):
        parent = os.path.dirname(probe)
        if parent == probe:
            break
        probe = parent
    free = shutil.disk_usage(probe).free
    if free < reserve_bytes:
        raise LogSpaceError(
            "insufficient free space for logs at %s: %d < %d"
            % (path, free, reserve_bytes)
        )
    return free

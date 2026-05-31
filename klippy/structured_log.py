# klippy/structured_log.py
# Structured logging schema, context, and forward helper for the klippy host.
#
# This module never imports heavy klippy objects so it can be used from the
# earliest point in startup (before the reactor/printer exist).
import contextvars
import datetime
import logging

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
    dt = datetime.datetime.fromtimestamp(
        created, tz=datetime.timezone.utc
    )
    return dt.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (
        dt.microsecond // 1000,
    )


import os
import time

UNBOUND_SESSION = "__unbound__"

_session_var = contextvars.ContextVar("kalico_session_id", default=None)
_print_var = contextvars.ContextVar("kalico_print_id", default="")


def make_session_id():
    return "k-%d-%d" % (int(time.time()), os.getpid())


def bind_session(session_id):
    _session_var.set(session_id)


def clear_session():
    _session_var.set(None)


def get_session():
    val = _session_var.get()
    return UNBOUND_SESSION if val is None else val


def bind_print(print_id):
    _print_var.set(print_id)


def clear_print():
    _print_var.set("")


def get_print():
    return _print_var.get()


# LogRecord attributes that are stdlib bookkeeping, not schema payload.
_STD_ATTRS = frozenset(
    logging.LogRecord(
        "x", logging.INFO, "x", 0, "x", (), None
    ).__dict__.keys()
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
        "target": record.name,
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

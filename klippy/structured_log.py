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

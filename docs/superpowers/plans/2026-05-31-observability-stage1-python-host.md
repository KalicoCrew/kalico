# Observability Logging — Stage 1 (Python Host) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the klippy Python host one structured logging pipeline — every log record becomes a schema-conformant JSON line in a durable `events/host-py.jsonl` file, while a stock-format `klippy.log` is preserved for back-compat — without editing any of the ~480 existing `logging.*` call sites.

**Architecture:** Keep klippy's existing async design (`QueueHandler` on the calling thread → bounded queue → one background `QueueListener` thread). Behind that thread, replace the single file handler with a **`SinkRegistry`** that fans each record out to a **`TextSink`** (stock `klippy.log`) and a **`JsonlSink`** (durable structured JSONL). A `ContextFilter` on the root logger injects `session_id` / `print_id` / `source` / `target` into every record at emit time (correct for `contextvars`). A small `structured_log` module owns the schema, the context vars, the serializer, and a forward `event()` helper for new code.

**Tech Stack:** Python 3 (project `requires-python >=3.9`; this dev host runs 3.14), stdlib `logging` + `logging.handlers` + `contextvars` + `json`, pytest, ruff (lint+format, line-length 80, configured in `pyproject.toml`). No new third-party dependencies.

**Spec:** `docs/superpowers/specs/2026-05-31-observability-logging-pipeline-design.md` (this plan implements §17 Stage 1).

---

## Scope

This is Stage 1 of three (spec §17). It delivers standalone value: queryable JSONL + stock text log with **zero external services**. Out of scope here (later stages): Rust host `tracing` swap (Stage 2), VictoriaLogs + Vector + the `query-logs` skill (Stage 3), MCU endpoint (spec #2), UI (spec #3), and the user-facing sink-selection / runtime-level config (deferred follow-on). Stage 1 hard-wires the active sink set to `{text, jsonl}`.

## File structure

| File | Responsibility |
|---|---|
| `klippy/structured_log.py` (new) | Schema: level map, `_time` formatting, `record_to_dict`, `serialize_record` (sanitizing, one-line JSON); context vars + `bind_session`/`bind_print`/`clear_print`/getters + `make_session_id`; `ContextFilter`; forward `event()` helper. |
| `klippy/log_sinks.py` (new) | `Sink` base, `TextSink` (stock klippy.log + rollover), `JsonlSink` (durable size-rotated JSONL, flush-per-record), `SinkRegistry` (fan-out, fail-loud). |
| `klippy/queuelogger.py` (modify) | Bounded fail-loud `QueueHandler`; `QueueListener` owns a `SinkRegistry` and delegates rollover; `setup_bg_logging` builds the registry, installs `ContextFilter`, creates the events dir. |
| `klippy/printer.py` (modify) | Generate + bind `session_id` before the first log line; pass `events_dir` to `setup_bg_logging`; bind on the no-logfile path too. |
| `klippy/extras/print_stats.py` (modify) | Bind/clear `print_id` on print start/complete/cancel/error (print correlation). |
| `test/test_structured_log.py` (new) | Unit tests for schema, serializer/sanitization, context + filter, `event()`. |
| `test/test_log_sinks.py` (new) | Unit tests for sinks + registry fan-out + fail-loud. |
| `test/test_queuelogger_pipeline.py` (new) | Integration: emit → bounded queue → registry → both files; rollover; overflow. |

## Prerequisites

- [ ] **Confirm a green baseline** before starting.

Run: `cd /Users/daniladergachev/Developer/kalico/.worktrees/observability && python3 -m pytest test/ -q`
Expected: existing suite passes (collection succeeds, 0 failures). If pytest is missing, run `python3 -m pip install pytest` first. Record the baseline pass count.

---

## Task 1: Schema — level map and `_time` formatting

**Files:**
- Create: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# test/test_structured_log.py
import logging
import sys

import pytest

from klippy import structured_log as sl


@pytest.fixture(autouse=True)
def _reset_log_context():
    # The session/print contextvars are module-level globals; reset them
    # before AND after every test so the suite is order-independent.
    sl.clear_session()
    sl.clear_print()
    yield
    sl.clear_session()
    sl.clear_print()


def test_level_name_maps_stdlib_levels():
    assert sl.level_name(logging.DEBUG) == "debug"
    assert sl.level_name(logging.INFO) == "info"
    assert sl.level_name(logging.WARNING) == "warn"
    assert sl.level_name(logging.ERROR) == "error"
    assert sl.level_name(logging.CRITICAL) == "error"
    assert sl.level_name(sl.TRACE_LEVEL) == "trace"


def test_format_time_is_rfc3339_utc_millis_z():
    # 2026-05-31T00:00:00Z == 1780185600 (UTC)
    out = sl.format_time(1780185600.0)
    assert out == "2026-05-31T00:00:00.000Z"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `ModuleNotFoundError`/`AttributeError` (module/functions not defined).

- [ ] **Step 3: Write minimal implementation**

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): structured-log schema level map and RFC3339 time"
```

---

## Task 2: Schema — context vars and session id

**Files:**
- Modify: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
def test_session_bind_and_get():
    sl.bind_session("k-1779840000-4242")
    assert sl.get_session() == "k-1779840000-4242"


def test_print_bind_clear_default_empty():
    sl.clear_print()
    assert sl.get_print() == ""
    sl.bind_print("print-123")
    assert sl.get_print() == "print-123"
    sl.clear_print()
    assert sl.get_print() == ""


def test_make_session_id_shape():
    sid = sl.make_session_id()
    parts = sid.split("-")
    assert parts[0] == "k"
    assert parts[1].isdigit() and parts[2].isdigit()


def test_get_session_unbound_is_sentinel():
    # The autouse fixture has cleared the session var, so an unbound read
    # returns the queryable sentinel rather than crashing.
    sl.clear_session()
    assert sl.get_session() == sl.UNBOUND_SESSION
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError` (bind_session/get_session/etc. not defined).

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (6 tests total).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): session/print contextvars and session id"
```

---

## Task 3: Schema — `record_to_dict`

**Files:**
- Modify: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
def _make_record(msg="hello", level=logging.INFO, name="mod.Cls", **extra):
    rec = logging.LogRecord(
        name=name, level=level, pathname=__file__, lineno=1,
        msg=msg, args=(), exc_info=None,
    )
    rec.created = 1780185600.0
    rec.session_id = "k-1779840000-1"
    rec.print_id = ""
    rec.source = sl.SOURCE_HOST_PY
    for k, v in extra.items():
        setattr(rec, k, v)
    return rec


def test_record_to_dict_core_fields():
    rec = _make_record()
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert d["_time"] == "2026-05-31T00:00:00.000Z"
    assert d["_msg"] == "hello"
    assert d["level"] == "info"
    assert d["source"] == "host-py"
    assert d["session_id"] == "k-1779840000-1"
    assert d["target"] == "mod.Cls"
    assert d["print_id"] == ""  # empty allowed


def test_record_to_dict_promotes_extra_fields():
    rec = _make_record(subsystem="homing", event="homing.trip", axis="z")
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert d["subsystem"] == "homing"
    assert d["event"] == "homing.trip"
    assert d["axis"] == "z"


def test_record_to_dict_captures_exception_traceback():
    # logging.exception() / exc_info populates record.exc_text during format();
    # the traceback must survive into the JSONL schema, not be dropped.
    try:
        raise ValueError("boom")
    except ValueError:
        rec = logging.LogRecord(
            "mod.Cls", logging.ERROR, __file__, 1,
            "handler failed", (), sys.exc_info(),
        )
    rec.created = 1780185600.0
    rec.session_id = "k-1779840000-1"
    rec.print_id = ""
    rec.source = sl.SOURCE_HOST_PY
    # Formatter.format() is what sets exc_text; emulate the QueueHandler path.
    logging.Formatter().format(rec)
    rec.message = rec.getMessage()
    d = sl.record_to_dict(rec)
    assert "ValueError: boom" in d["exception"]
    assert "Traceback" in d["exception"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError: module ... has no attribute 'record_to_dict'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py

# LogRecord attributes that are stdlib bookkeeping, not schema payload.
_STD_ATTRS = frozenset(
    logging.LogRecord(
        "x", logging.INFO, "x", 0, "x", (), None
    ).__dict__.keys()
) | {"message", "asctime", "session_id", "print_id", "source", "taskName"}

# Schema fields that get a dedicated slot (everything else is free payload).
_RESERVED_OUT = frozenset(
    ["_time", "_msg", "level", "source", "subsystem", "session_id",
     "target", "print_id"]
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (8 tests total).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): record_to_dict maps LogRecord to schema"
```

---

## Task 4: Schema — `serialize_record` (sanitizing, one physical line)

**Files:**
- Modify: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
import json


def test_serialize_is_single_line_and_round_trips():
    rec = _make_record(msg="line1\nline2\twith \"quote\" and \x01 ctrl")
    rec.message = rec.getMessage()
    line = sl.serialize_record(sl.record_to_dict(rec))
    # Exactly one physical line (trailing newline only).
    assert line.endswith("\n")
    assert line.count("\n") == 1
    obj = json.loads(line)
    assert obj["_msg"] == "line1\nline2\twith \"quote\" and \x01 ctrl"


def test_serialize_handles_nonjson_value():
    rec = _make_record(weird=object())
    rec.message = rec.getMessage()
    # Must not raise; non-serializable value is stringified.
    line = sl.serialize_record(sl.record_to_dict(rec))
    assert "weird" in json.loads(line)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError: ... 'serialize_record'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py
import json


def serialize_record(record_dict):
    # json.dumps escapes embedded newlines/quotes/control chars, guaranteeing
    # exactly one physical line per record (NDJSON-safe; injection-safe for
    # user-controlled values such as gcode comments / M117 text).
    line = json.dumps(
        record_dict, ensure_ascii=False, separators=(",", ":"), default=repr
    )
    return line + "\n"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (10 tests total).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): serialize_record sanitizes to one JSON line"
```

---

## Task 5: `ContextFilter`

**Files:**
- Modify: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
def test_context_filter_injects_bound_context():
    sl.bind_session("k-1779840000-7")
    sl.clear_print()
    sl.bind_print("print-77")
    f = sl.ContextFilter()
    rec = logging.LogRecord(
        "some.logger", logging.INFO, __file__, 1, "m", (), None
    )
    assert f.filter(rec) is True
    assert rec.session_id == "k-1779840000-7"
    assert rec.print_id == "print-77"
    assert rec.source == "host-py"
    assert rec.target == "some.logger"
    sl.clear_print()


def test_context_filter_does_not_overwrite_existing_source():
    f = sl.ContextFilter()
    rec = logging.LogRecord(
        "x", logging.INFO, __file__, 1, "m", (), None
    )
    rec.source = "sim"
    f.filter(rec)
    assert rec.source == "sim"  # re-emitted records keep their source
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError: ... 'ContextFilter'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py
class ContextFilter(logging.Filter):
    # Injected on the root logger; runs on the CALLING thread, so contextvars
    # are read correctly. Never raises (raising inside logging is unsafe);
    # an unbound session shows up as the queryable UNBOUND_SESSION sentinel,
    # which the startup ordering invariant (printer.main) guarantees we never
    # actually hit in normal operation.
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (12 tests total).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): ContextFilter injects session/print/source/target"
```

---

## Task 6: Forward `event()` helper (fail-loud on missing required fields)

**Files:**
- Modify: `klippy/structured_log.py`
- Test: `test/test_structured_log.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
def test_event_emits_with_required_fields(caplog):
    with caplog.at_level(logging.INFO):
        sl.event("homing", "homing.endstop_trip", axis="z", trigger_mm=12.4)
    rec = caplog.records[-1]
    assert rec.subsystem == "homing"
    assert rec.event == "homing.endstop_trip"
    assert rec.axis == "z"
    assert rec.trigger_mm == 12.4


def test_event_requires_subsystem_and_event():
    import pytest as _pytest

    with _pytest.raises(ValueError):
        sl.event("", "x")
    with _pytest.raises(ValueError):
        sl.event("motion", "")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError: ... 'event'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (14 tests total).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py test/test_structured_log.py
git commit -m "feat(logging): structured_log.event forward helper"
```

---

## Task 7: `Sink` base + `SinkRegistry` (fan-out, fail-loud)

**Files:**
- Create: `klippy/log_sinks.py`
- Test: `test/test_log_sinks.py`

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: FAIL — `ModuleNotFoundError: klippy.log_sinks`.

- [ ] **Step 3: Write minimal implementation**

```python
# klippy/log_sinks.py
# Pluggable log sinks and the registry that fans records out to them.
#
# A Sink consumes a (context-enriched, message-formatted) logging.LogRecord.
# The registry runs on the single QueueListener background thread, so sinks
# need not be thread-safe among themselves.
import logging


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
        # Fail loudly: a sink raising (e.g. disk full) propagates so the
        # background thread surfaces it rather than silently degrading the
        # durable store (spec §12, fail-loudly).
        for sink in self._sinks:
            sink.emit_record(record)

    def close(self):
        for sink in self._sinks:
            sink.close()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/log_sinks.py test/test_log_sinks.py
git commit -m "feat(logging): Sink base and fail-loud SinkRegistry"
```

---

## Task 8: `TextSink` — stock `klippy.log` + rollover

**Files:**
- Modify: `klippy/log_sinks.py`
- Test: `test/test_log_sinks.py`

The `TextSink` wraps a `TimedRotatingFileHandler` with **no formatter** so output is exactly `%(message)s` — byte-identical to today's stock `klippy.log` (preserves logextract/graphstats). It also owns the rollover-info header that `queuelogger.QueueListener` emits today.

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_log_sinks.py
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: FAIL — `AttributeError: ... 'TextSink'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/log_sinks.py
import logging.handlers
import time


class TextSink(Sink):
    # Stock klippy.log: a TimedRotatingFileHandler with NO formatter, so the
    # emitted line is the raw message (identical to legacy klippy behavior).
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
            self._rollover_info[name]
            for name in sorted(self._rollover_info)
        ]
        lines.append(
            "=============== Log rollover at %s ==============="
            % (time.asctime(),)
        )
        self._handler.emit(
            logging.makeLogRecord(
                {"msg": "\n".join(lines), "level": logging.INFO}
            )
        )

    def close(self):
        self._handler.close()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/log_sinks.py test/test_log_sinks.py
git commit -m "feat(logging): TextSink preserves stock klippy.log + rollover"
```

---

## Task 9: `JsonlSink` — durable size-rotated JSONL

**Files:**
- Modify: `klippy/log_sinks.py`
- Test: `test/test_log_sinks.py`

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_log_sinks.py
import json


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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: FAIL — `AttributeError: ... 'JsonlSink'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/log_sinks.py
import os
import time

from . import structured_log

# Rotated files are kept UNCOMPRESSED (spec §8): the Stage-3 shipper (Vector)
# cannot resume reading a gzipped file.
JSONL_MAX_BYTES = 32 * 1024 * 1024
JSONL_BACKUP_COUNT = 5
# Periodic fsync backstop interval (spec §3/§7 relaxed-durability contract).
JSONL_FSYNC_INTERVAL = 15.0


class JsonlSink(Sink):
    def __init__(self, filename, max_bytes=JSONL_MAX_BYTES,
                 backup_count=JSONL_BACKUP_COUNT,
                 fsync_interval=JSONL_FSYNC_INTERVAL):
        os.makedirs(os.path.dirname(filename) or ".", exist_ok=True)
        self._handler = logging.handlers.RotatingFileHandler(
            filename, maxBytes=max_bytes, backupCount=backup_count,
            encoding="utf-8", delay=False,
        )
        self._fsync_interval = fsync_interval
        self._last_fsync = time.monotonic()

    def emit_record(self, record):
        line = structured_log.serialize_record(
            structured_log.record_to_dict(record)
        )
        # Write the already-serialized line verbatim; flush immediately for
        # the relaxed-durability contract. A write/flush failure propagates
        # (fail-loud, spec §12).
        stream = self._handler.stream
        if self._handler.shouldRollover(record):
            self._handler.doRollover()
            stream = self._handler.stream
            self._last_fsync = time.monotonic()
        stream.write(line)
        stream.flush()
        # Periodic fsync backstop: bound power-loss exposure to ~interval
        # without paying an fsync on every record. Runs on the single
        # QueueListener bg thread, so no extra thread or lock is needed.
        now = time.monotonic()
        if now - self._last_fsync >= self._fsync_interval:
            os.fsync(stream.fileno())
            self._last_fsync = now

    def close(self):
        # Final fsync so a clean shutdown is fully durable, then close.
        stream = self._handler.stream
        if stream is not None:
            stream.flush()
            os.fsync(stream.fileno())
        self._handler.close()
```

> Note: `RotatingFileHandler.shouldRollover` calls `self.format(record)` to size the message; since we write our own line, size-based rotation is approximate (good enough — exact byte caps are a §16 tunable). This keeps us on the stdlib rotation machinery rather than hand-rolling it. The fsync is on the handler's file descriptor; `flush()` first guarantees Python's buffer is in the OS before the fd is fsync'd.

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_log_sinks.py -q`
Expected: PASS (9 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/log_sinks.py test/test_log_sinks.py
git commit -m "feat(logging): JsonlSink writes durable size-rotated JSONL"
```

---

## Task 10: Bounded, fail-loud `QueueHandler`

**Files:**
- Modify: `klippy/queuelogger.py:14-27` (the `QueueHandler` class)
- Test: `test/test_queuelogger_pipeline.py`

Replace today's silent `put_nowait` + `handleError` (which drops records on overflow) with a bounded queue that fails loudly on overflow (spec §7.1).

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_queuelogger_pipeline.py -q`
Expected: FAIL — `AttributeError: ... 'LogQueueOverflow'` (and no raise today).

- [ ] **Step 3: Write minimal implementation**

Replace the `QueueHandler` class body in `klippy/queuelogger.py` (lines 14-27) with:

```python
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
            # Fail loudly (spec §7.1): never silently drop a durable record.
            raise LogQueueOverflow(
                "klippy log queue overflow; logging cannot keep up"
            )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_queuelogger_pipeline.py -q`
Expected: PASS (1 test).

- [ ] **Step 5: Commit**

```bash
git add klippy/queuelogger.py test/test_queuelogger_pipeline.py
git commit -m "feat(logging): bounded fail-loud log queue handler"
```

---

## Task 11: `QueueListener` owns a `SinkRegistry`

**Files:**
- Modify: `klippy/queuelogger.py:31-94` (`QueueListener` + `setup_bg_logging`)
- Test: `test/test_queuelogger_pipeline.py`

Rework `QueueListener` so it is **no longer** a `TimedRotatingFileHandler` subclass; instead it owns a `SinkRegistry` (TextSink + JsonlSink) and exposes the same `set_rollover_info` / `clear_rollover_info` / `doRollover` / `stop` surface that `printer.py` already calls. `setup_bg_logging` gains an `events_dir` and installs the `ContextFilter`.

- [ ] **Step 1: Write the failing integration test**

```python
# append to test/test_queuelogger_pipeline.py
import json
import logging as _logging
import os

from klippy import structured_log


def test_setup_bg_logging_writes_both_files(tmp_path):
    structured_log.bind_session("k-1779840000-99")
    structured_log.clear_print()
    klippy_log = str(tmp_path / "klippy.log")
    events_dir = str(tmp_path / "events")
    ql = queuelogger.setup_bg_logging(
        filename=klippy_log,
        debuglevel=_logging.INFO,
        rotate_log_at_restart=False,
        events_dir=events_dir,
    )
    try:
        log = _logging.getLogger("test.pipeline")
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_queuelogger_pipeline.py -q`
Expected: FAIL — `TypeError`/`AttributeError` (`setup_bg_logging` has no `events_dir`; no JSONL produced).

- [ ] **Step 3: Write minimal implementation**

Replace `QueueListener` (lines 31-79) and `setup_bg_logging` (lines 85-94) in `klippy/queuelogger.py` with:

```python
import os

from . import log_sinks
from . import structured_log


# Polls the queue in a background thread and fans each record out to sinks.
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
        self.bg_thread = threading.Thread(target=self._bg_thread)
        self.bg_thread.start()

    def _bg_thread(self):
        while True:
            record = self.bg_queue.get(True)
            if record is None:
                break
            self.registry.emit(record)

    def stop(self):
        self.bg_queue.put(None)
        self.bg_thread.join()
        self.registry.close()

    # --- back-compat surface used by printer.py / configfile / mcu / etc. ---
    def set_rollover_info(self, name, info):
        self._text.set_rollover_info(name, info)

    def clear_rollover_info(self):
        self._text.clear_rollover_info()

    def doRollover(self):
        self._text.do_rollover()


MainQueueHandler = None
LOG_QUEUE_MAXSIZE = 100000


def setup_bg_logging(filename, debuglevel, rotate_log_at_restart,
                     events_dir=None):
    global MainQueueHandler
    ql = QueueListener(
        filename=filename,
        rotate_log_at_restart=rotate_log_at_restart,
        events_dir=events_dir,
    )
    MainQueueHandler = QueueHandler(ql.bg_queue)
    root = logging.getLogger()
    root.addFilter(structured_log.ContextFilter())
    root.addHandler(MainQueueHandler)
    root.setLevel(debuglevel)
    return ql
```

Also update the module imports near the top of `klippy/queuelogger.py` to ensure `queue` and `threading` remain imported (they already are) and remove the now-unused `import logging.handlers` only if nothing else uses it (TextSink does, in log_sinks). Keep `import time` removal optional — leave it; it is harmless.

> **Filter placement note:** `root.addFilter(...)` only filters records that pass through a handler *attached to the root logger*. Because klippy logs via the root logger and `MainQueueHandler` is on the root, add the filter to the **handler** instead if any child logger has `propagate=False`. Safer: `MainQueueHandler.addFilter(structured_log.ContextFilter())`. Use the handler form.

Apply the handler-filter form:

```python
    MainQueueHandler = QueueHandler(ql.bg_queue)
    MainQueueHandler.addFilter(structured_log.ContextFilter())
    root = logging.getLogger()
    root.addHandler(MainQueueHandler)
    root.setLevel(debuglevel)
    return ql
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_queuelogger_pipeline.py -q`
Expected: PASS. Then run the whole suite: `python3 -m pytest test/ -q` — existing tests still green (queuelogger import test still passes).

- [ ] **Step 5: Commit**

```bash
git add klippy/queuelogger.py test/test_queuelogger_pipeline.py
git commit -m "feat(logging): QueueListener fans records to text+jsonl sinks"
```

---

## Task 12: Bind `session_id` and wire `events_dir` in `printer.main`

**Files:**
- Modify: `klippy/printer.py` — `main()` around lines 678-710 (logging setup) and the no-logfile branch.
- Test: `test/test_session_binding.py` (new)

The binding-timing invariant (spec §6): `session_id` must be bound **before the first log line** (`"Starting Klippy..."`, currently line 705). The events dir is derived from the logfile path (`<logdir>/events/`).

- [ ] **Step 1: Write the failing test**

```python
# test/test_session_binding.py
from klippy import structured_log


def test_session_id_format_round_trip():
    sid = structured_log.make_session_id()
    structured_log.bind_session(sid)
    assert structured_log.get_session() == sid
    assert sid.startswith("k-")


def test_events_dir_derivation():
    from klippy import printer
    assert printer.events_dir_for("/home/pi/printer_data/logs/klippy.log") == (
        "/home/pi/printer_data/logs/events"
    )
    assert printer.events_dir_for(None) is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_session_binding.py -q`
Expected: FAIL — `AttributeError: module 'klippy.printer' has no attribute 'events_dir_for'`.

- [ ] **Step 3: Write minimal implementation**

Add a helper near the top of `klippy/printer.py` (after imports), and add `structured_log` to the existing relative-import block (the one that currently imports `queuelogger` at line 33):

```python
# in the existing "from . import (...)" block alongside queuelogger:
    structured_log,
```

```python
# module-level helper in klippy/printer.py
import os


def events_dir_for(logfile):
    if not logfile:
        return None
    return os.path.join(os.path.dirname(os.path.abspath(logfile)), "events")
```

Then in `main()`, **before** `logging.info("=======================")` (line 704), bind the session and pass `events_dir`. Change the logfile block (current lines 692-703, which start at `bglogger = None`) to the following. Note `edir` is computed **once** here so Task 13 can insert its preflight cleanly with no second edit of these lines:

```python
    # Bind the session id BEFORE the first log line (spec §6 binding-timing).
    session_id = structured_log.make_session_id()
    structured_log.bind_session(session_id)
    start_args["session_id"] = session_id

    edir = events_dir_for(options.logfile)
    bglogger = None
    if options.logfile:
        start_args["log_file"] = options.logfile
        bglogger = queuelogger.setup_bg_logging(
            filename=options.logfile,
            debuglevel=debuglevel,
            rotate_log_at_restart=options.rotate_log_at_restart,
            events_dir=edir,
        )
        if options.rotate_log_at_restart:
            bglogger.doRollover()
    else:
        logging.getLogger().setLevel(debuglevel)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_session_binding.py -q`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/printer.py test/test_session_binding.py
git commit -m "feat(logging): bind session id before first log, wire events dir"
```

---

## Task 13: Disk-space preflight for the logs directory

**Files:**
- Modify: `klippy/structured_log.py`
- Modify: `klippy/printer.py` (`main()`, after `events_dir` is known)
- Test: `test/test_structured_log.py`

Spec §12: a pre-start check refuses to proceed if free space under the logs dir is below a reserve, so a disk-full logging failure is caught up front (fail-loudly) rather than mid-print.

- [ ] **Step 1: Write the failing test**

```python
# append to test/test_structured_log.py
def test_check_log_space_ok_for_tmp(tmp_path):
    # Plenty of space in tmp; returns free bytes, does not raise.
    free = sl.check_log_space(str(tmp_path), reserve_bytes=1)
    assert free > 1


def test_check_log_space_raises_when_below_reserve(tmp_path):
    import pytest as _pytest
    huge = 10 ** 18  # 1 EB reserve cannot be satisfied
    with _pytest.raises(sl.LogSpaceError):
        sl.check_log_space(str(tmp_path), reserve_bytes=huge)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: FAIL — `AttributeError: ... 'check_log_space'`.

- [ ] **Step 3: Write minimal implementation**

```python
# append to klippy/structured_log.py
import shutil

# Default reserve below which we refuse to start (spec §16 tunable).
LOG_SPACE_RESERVE_BYTES = 64 * 1024 * 1024


class LogSpaceError(Exception):
    pass


def check_log_space(path, reserve_bytes=LOG_SPACE_RESERVE_BYTES):
    os.makedirs(path, exist_ok=True)
    free = shutil.disk_usage(path).free
    if free < reserve_bytes:
        raise LogSpaceError(
            "insufficient free space for logs at %s: %d < %d"
            % (path, free, reserve_bytes)
        )
    return free
```

Then call it in `klippy/printer.py:main()`. Task 12 already introduced the `edir = events_dir_for(options.logfile)` line; insert the preflight immediately **after** that line and **before** `bglogger = None` (do not recompute `edir`):

```python
    edir = events_dir_for(options.logfile)
    if edir is not None:
        structured_log.check_log_space(edir)
    bglogger = None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest test/test_structured_log.py -q`
Expected: PASS (16 tests total in this file).

- [ ] **Step 5: Commit**

```bash
git add klippy/structured_log.py klippy/printer.py test/test_structured_log.py
git commit -m "feat(logging): disk-space preflight for logs directory"
```

---

## Task 14: Bind `print_id` to the print lifecycle

**Files:**
- Modify: `klippy/extras/print_stats.py` (`note_start`, `note_complete`, `note_cancel`, `note_error`, `reset`)
- Test: `test/test_print_id_binding.py` (new)

Spec §6 print_id lifecycle: set at print start, cleared at complete/cancel/error. Format `print-<unix>`.

- [ ] **Step 1: Read the current methods**

Run: `grep -n "def note_start\|def note_complete\|def note_cancel\|def note_error\|def reset" klippy/extras/print_stats.py`
Note the exact bodies so the structured_log calls are added without removing existing behavior.

- [ ] **Step 2: Write the failing test**

```python
# test/test_print_id_binding.py
from klippy import structured_log


def test_print_id_set_and_cleared_helpers():
    # The print lifecycle uses these two helpers; verify the contract here.
    structured_log.clear_print()
    assert structured_log.get_print() == ""
    pid = structured_log.make_print_id()
    structured_log.bind_print(pid)
    assert structured_log.get_print() == pid
    assert pid.startswith("print-")
    structured_log.clear_print()
    assert structured_log.get_print() == ""
```

- [ ] **Step 3: Run test to verify it fails**

Run: `python3 -m pytest test/test_print_id_binding.py -q`
Expected: FAIL — `AttributeError: ... 'make_print_id'`.

- [ ] **Step 4: Add `make_print_id` to `structured_log.py`**

```python
# append to klippy/structured_log.py
def make_print_id():
    return "print-%d" % (int(time.time()),)
```

- [ ] **Step 5: Wire into `print_stats.py`**

Add `from .. import structured_log` to the imports, then:
- In `note_start`: `structured_log.bind_print(structured_log.make_print_id())`
- In `note_complete`, `note_cancel`, `note_error`: `structured_log.clear_print()`
- Leave `note_pause` unchanged (a paused print retains its `print_id`, per spec §6).

Add each call as the **last** line of the respective method so existing behavior is untouched.

- [ ] **Step 6: Run tests to verify pass**

Run: `python3 -m pytest test/test_print_id_binding.py -q && python3 -m pytest test/ -q`
Expected: PASS (new test green; full suite still green).

- [ ] **Step 7: Commit**

```bash
git add klippy/structured_log.py klippy/extras/print_stats.py test/test_print_id_binding.py
git commit -m "feat(logging): bind print_id across the print lifecycle"
```

---

## Task 15: Full-suite regression + ruff (lint + format)

**Files:** none (verification task)

The project uses **ruff** (configured in `pyproject.toml`: `line-length = 80`, import-sort rules `I001`/`I002`). There is no black/isort here.

- [ ] **Step 1: Lint + format to project style**

Run (auto-fix import sorting and other fixable lints, then format):
```bash
python3 -m ruff check --fix klippy/structured_log.py klippy/log_sinks.py klippy/queuelogger.py klippy/printer.py klippy/extras/print_stats.py test/test_structured_log.py test/test_log_sinks.py test/test_queuelogger_pipeline.py test/test_session_binding.py test/test_print_id_binding.py
python3 -m ruff format klippy/structured_log.py klippy/log_sinks.py klippy/queuelogger.py klippy/printer.py klippy/extras/print_stats.py test/test_structured_log.py test/test_log_sinks.py test/test_queuelogger_pipeline.py test/test_session_binding.py test/test_print_id_binding.py
```
Expected: files reformatted to line-length 80 with sorted imports (or "N files left unchanged"). If `ruff` is not installed, run `python3 -m pip install ruff` first.

- [ ] **Step 2: Lint check passes clean**

Run: `python3 -m ruff check klippy/structured_log.py klippy/log_sinks.py klippy/queuelogger.py klippy/printer.py klippy/extras/print_stats.py test/`
Expected: `All checks passed!`

- [ ] **Step 3: Run the full test suite**

Run: `python3 -m pytest test/ -q`
Expected: all green, including the new files and the pre-existing baseline.

- [ ] **Step 4: Smoke-check the import graph**

Run: `python3 -c "import klippy.queuelogger, klippy.structured_log, klippy.log_sinks, klippy.printer"`
Expected: no output (clean import).

- [ ] **Step 5: Commit any formatting changes**

```bash
git add -A
git commit -m "chore(logging): ruff lint+format for stage 1"
```

---

## Self-review (completed during authoring)

**Spec coverage (Stage 1 scope, §17):** SinkRegistry → Task 7; text + jsonl sinks → Tasks 8, 9; ContextFilter → Task 5; structured_log.event → Task 6; session binding + binding-timing invariant → Task 12; print_id lifecycle → Task 14; sanitization/one-line → Task 4; exception-traceback capture → Task 3; bounded fail-loud queue → Task 10; write-failure fail-loud → Tasks 7, 9 (propagate) + §12; relaxed durability (flush-per-record **+ periodic fsync backstop + fsync-on-close**) → Task 9; disk preflight → Task 13; stock-format text view → Task 8; events dir under printer_data/logs → Tasks 9, 11, 12.

**Review fixes applied (post adversarial review, 0 blockers / 6 confirmed findings):** corrected the test epoch to `1780185600` (= 2026-05-31T00:00:00Z) in Tasks 1 & 3; added exception-traceback capture (`record.exc_text` → `exception` field) in Task 3; added an autouse contextvars-reset fixture + `clear_session()` for test isolation (Tasks 2, 5, 6); implemented the spec-locked periodic-fsync backstop + fsync-on-close in Task 9; removed the stray "Step 2b"; de-duplicated the `edir` computation across Tasks 12/13; switched Task 15 from black/isort to the project's **ruff**; cleaned the Task 11 filter-placement guidance to the single correct handler form.

**Deferred-correctly (NOT in this plan):** Rust `tracing` (Stage 2), Vector/VL/skill (Stage 3), runtime `SET_LOG_LEVEL` + sink-selection config (follow-on), tmpfs index (§16), per-subsystem default-level table + Pi profiling (a Stage-1-adjacent task that needs hardware — see "Open" below).

**Type/name consistency:** `record_to_dict`, `serialize_record`, `level_name`, `format_time`, `bind_session`/`get_session`, `bind_print`/`get_print`/`clear_print`, `make_session_id`/`make_print_id`, `Sink.emit_record`/`close`, `SinkRegistry.emit`/`register`/`close`, `TextSink.set_rollover_info`/`clear_rollover_info`/`do_rollover`, `JsonlSink`, `LogQueueOverflow`, `LogSpaceError`/`check_log_space`, `events_dir_for` — all defined once and referenced consistently.

## Open (carry to review, not blockers for Task 1)

- **Per-subsystem default levels + Pi 3B/4 profiling** (spec §7.1, §9): the concrete default-level table and the high-speed-print profiling pass need target hardware; sequence them after Task 15 as a measurement task, defaulting noisy subsystems (`clocksync`, heartbeat) to `warn` if profiling shows volume pressure. Not required to land the pipeline.
- **`subsystem` for legacy call sites:** existing `logging.*` records have no `subsystem` until enriched (spec §7.1 step 9, incremental). Records without `subsystem` are valid (it is optional in the schema); the `_stream` grouping in Stage 3 tolerates its absence.

#!/usr/bin/env python3
from __future__ import annotations

import inspect

import pytest

from klippy.motion_toolhead import MotionToolhead

pytestmark = pytest.mark.sim_unit


class _FakeMcu:
    def __init__(self, est):
        self._est = est

    def estimated_print_time(self, eventtime):
        return self._est


class _FakeReactor:
    def __init__(self, now):
        self._now = now

    def monotonic(self):
        return self._now


class _FakePrinter:
    def __init__(self):
        self.events = []

    def send_event(self, name, *args):
        self.events.append((name, *args))
        return []


class _Stub:
    def __init__(self, pending_end, est, now=1000.0):
        self._mcu_pending_end_time = pending_end
        self.mcu = _FakeMcu(est)
        self.reactor = _FakeReactor(now)
        self.printer = _FakePrinter()


def test_check_busy_reports_idle_when_motion_has_drained():
    stub = _Stub(pending_end=10.0, est=70.0)
    print_time, est_print_time, lookahead_empty = MotionToolhead.check_busy(
        stub, eventtime=123.0
    )
    assert print_time == 10.0
    assert est_print_time == 70.0
    assert lookahead_empty is True
    assert est_print_time - print_time == 60.0


def test_check_busy_reports_busy_while_motion_queued():
    stub = _Stub(pending_end=80.0, est=70.0)
    _, _, lookahead_empty = MotionToolhead.check_busy(stub, eventtime=123.0)
    assert lookahead_empty is False


def test_sync_print_time_emits_event_with_stock_arg_order():
    stub = _Stub(pending_end=80.0, est=70.0, now=1000.0)
    MotionToolhead._sync_print_time(stub)
    assert stub.printer.events == [
        ("toolhead:sync_print_time", 1000.0, 70.0, 80.0)
    ]


def test_move_and_dwell_start_the_idle_timeout_clock():
    for name in ("move", "dwell"):
        src = inspect.getsource(MotionToolhead.__dict__[name])
        assert "_sync_print_time" in src, (
            "%s must emit toolhead:sync_print_time, or idle_timeout never "
            "starts and motors never disable" % name
        )

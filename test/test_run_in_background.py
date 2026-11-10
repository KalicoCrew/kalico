import multiprocessing
import os
import threading
import time

import pytest

from klippy import util


class FakeReactor:
    def monotonic(self):
        return time.monotonic()

    def pause(self, waketime):
        time.sleep(max(0.0, waketime - time.monotonic()))
        return time.monotonic()


class FakeGCode:
    def __init__(self):
        self.messages = []

    def respond_info(self, msg, log=True):
        self.messages.append(msg)


class FakePrinter:
    def __init__(self):
        self.gcode = FakeGCode()

    def get_reactor(self):
        return FakeReactor()

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode


@pytest.fixture
def printer():
    yield FakePrinter()
    assert not multiprocessing.active_children()


def test_returns_result_of_closure(printer):
    offset = 10
    assert util.run_in_background(printer, lambda: 5 + offset) == 15


def test_reports_progress(printer):
    util.run_in_background(
        printer,
        lambda: time.sleep(0.5),
        progress_msg="working",
        progress_interval=0.1,
    )
    assert printer.gcode.messages
    assert set(printer.gcode.messages) == {"working"}


def test_exception_carries_child_traceback(printer):
    def fail():
        raise ValueError("bad data")

    with pytest.raises(util.BackgroundError, match="ValueError: bad data"):
        util.run_in_background(printer, fail)


def test_child_exit_without_result(printer):
    with pytest.raises(util.BackgroundError, match="exit code 3"):
        util.run_in_background(printer, lambda: os._exit(3), timeout=10.0)


def test_timeout_kills_child(printer):
    start = time.monotonic()
    with pytest.raises(util.BackgroundTimeout):
        util.run_in_background(printer, lambda: time.sleep(3600), timeout=0.5)
    assert time.monotonic() - start < 5.0


def test_result_larger_than_pipe_buffer(printer):
    size = 8 * 1024 * 1024
    result = util.run_in_background(printer, lambda: b"x" * size, timeout=10.0)
    assert len(result) == size


def test_unpicklable_result(printer):
    with pytest.raises(util.BackgroundError, match="pickle"):
        util.run_in_background(printer, threading.Lock, timeout=10.0)

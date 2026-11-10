import multiprocessing
import os
import threading
import time

import pytest

from klippy import util
from klippy.gcode import InterruptToken, WaitInterruption
from klippy.reactor import Reactor


class FakeGCode:
    def __init__(self, reactor):
        self.messages = []
        self.token = InterruptToken(reactor, lambda: False)

    def respond_info(self, msg, log=True):
        self.messages.append(msg)

    def get_interrupt_token(self):
        return self.token


class FakePrinter:
    def __init__(self):
        self.reactor = Reactor()
        self.gcode = FakeGCode(self.reactor)

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name):
        assert name == "gcode"
        return self.gcode

    def run(self, func, *, watchdog=15.0, **kwargs):
        """Call run_in_background from inside the running reactor."""
        reactor = self.reactor
        outcome = []

        def invoke(eventtime):
            try:
                outcome.append(
                    (True, util.run_in_background(self, func, **kwargs))
                )
            except Exception as e:  # noqa: BLE001
                outcome.append((False, e))
            finally:
                reactor.end()

        reactor.register_callback(invoke)
        reactor.register_callback(
            lambda eventtime: reactor.end(), reactor.monotonic() + watchdog
        )
        reactor.run()
        assert outcome, "run_in_background did not return before the watchdog"
        ok, value = outcome[0]
        if not ok:
            raise value
        return value


@pytest.fixture
def printer():
    printer = FakePrinter()
    try:
        yield printer
    finally:
        printer.reactor.finalize()
    assert not multiprocessing.active_children()


def test_returns_result_of_closure(printer):
    offset = 10
    assert printer.run(lambda: 5 + offset) == 15


def test_reports_progress_while_reactor_runs(printer):
    printer.run(
        lambda: time.sleep(0.5),
        progress_msg="working",
        progress_interval=0.1,
    )
    assert len(printer.gcode.messages) >= 2
    assert set(printer.gcode.messages) == {"working"}


def test_exception_carries_child_traceback(printer):
    def fail():
        raise ValueError("bad data")

    with pytest.raises(util.BackgroundError, match="ValueError: bad data"):
        printer.run(fail)


def test_child_exit_without_result(printer):
    with pytest.raises(util.BackgroundError, match="exit code 3"):
        printer.run(lambda: os._exit(3))


def test_timeout_kills_child(printer):
    start = time.monotonic()
    with pytest.raises(util.BackgroundTimeout):
        printer.run(lambda: time.sleep(3600), timeout=0.5)
    assert time.monotonic() - start < 5.0


def test_result_larger_than_pipe_buffer(printer):
    size = 8 * 1024 * 1024
    assert len(printer.run(lambda: b"x" * size)) == size


def test_unpicklable_result(printer):
    with pytest.raises(util.BackgroundError, match="pickle"):
        printer.run(threading.Lock)


def test_interrupt_kills_child(printer):
    reactor = printer.reactor
    token = printer.gcode.token
    reactor.register_callback(
        lambda eventtime: token.interrupt(), reactor.monotonic() + 0.3
    )
    start = time.monotonic()
    with pytest.raises(WaitInterruption, match="Command interrupted"):
        printer.run(lambda: time.sleep(3600))
    assert time.monotonic() - start < 5.0
    assert not token.callbacks


def test_interrupt_raises_token_error(printer):
    reactor = printer.reactor
    token = InterruptToken(reactor, lambda: False)
    reactor.register_callback(
        lambda eventtime: token.interrupt(
            WaitInterruption("Printer is shutdown")
        ),
        reactor.monotonic() + 0.3,
    )
    with pytest.raises(WaitInterruption, match="Printer is shutdown"):
        printer.run(lambda: time.sleep(3600), interrupt_token=token)


def test_already_interrupted_token(printer):
    printer.gcode.token.interrupt()
    with pytest.raises(WaitInterruption):
        printer.run(lambda: time.sleep(3600))


def test_completion_callbacks():
    reactor = Reactor()
    completion = reactor.completion()
    results = []

    def removed(result):
        results.append(("removed", result))

    completion.add_callback(lambda result: results.append(("first", result)))
    completion.add_callback(removed)
    completion.remove_callback(removed)
    completion.complete(7)
    completion.add_callback(lambda result: results.append(("late", result)))
    assert results == [("first", 7), ("late", 7)]
    assert not completion.callbacks

import sys
import types

import pytest

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")
_fake_native_mod.MotionBridge = object
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy import structured_log  # noqa: E402
from klippy.motion_bridge import attach_structured_logging  # noqa: E402


class FakeNative:
    def __init__(self):
        self.init_calls = []
        self.ctx_calls = []

    def init_logging(self, events_dir):
        self.init_calls.append(events_dir)

    def set_session_context(self, session_id, print_id=""):
        self.ctx_calls.append((session_id, print_id))


class FakePrinter:
    def __init__(self):
        self.handlers = {}

    def register_event_handler(self, name, cb):
        self.handlers.setdefault(name, []).append(cb)

    def fire(self, name):
        for cb in self.handlers.get(name, []):
            cb()


@pytest.fixture(autouse=True)
def _reset_context():
    structured_log.clear_session()
    structured_log.clear_print()
    yield
    structured_log.clear_session()
    structured_log.clear_print()


def test_init_and_initial_context_pushed():
    native = FakeNative()
    printer = FakePrinter()
    structured_log.bind_session("k-1-2")
    attach_structured_logging(
        native, printer, "/home/x/printer_data/logs/events"
    )
    assert native.init_calls == ["/home/x/printer_data/logs/events"]
    assert native.ctx_calls[0] == ("k-1-2", "")


def test_none_events_dir_skips_init_but_still_sets_context():
    native = FakeNative()
    printer = FakePrinter()
    structured_log.bind_session("k-1-2")
    attach_structured_logging(native, printer, None)
    assert native.init_calls == []
    assert native.ctx_calls[0] == ("k-1-2", "")


def test_print_start_and_end_propagate():
    native = FakeNative()
    printer = FakePrinter()
    structured_log.bind_session("k-1-2")
    attach_structured_logging(native, printer, "/x/events")
    # print_stats binds the print id BEFORE firing start_printing (Stage 1
    # ordering), so the start handler observes the new id.
    structured_log.bind_print("print-9")
    printer.fire("print_stats:start_printing")
    assert native.ctx_calls[-1] == ("k-1-2", "print-9")
    # print_stats fires the finish event BEFORE clearing the print id, so the
    # finish handlers must push an explicit empty print id regardless of the
    # current contextvar value.
    printer.fire("print_stats:complete_printing")
    assert native.ctx_calls[-1] == ("k-1-2", "")


def test_pause_retains_print_id():
    native = FakeNative()
    printer = FakePrinter()
    structured_log.bind_session("k-1-2")
    attach_structured_logging(native, printer, "/x/events")
    structured_log.bind_print("print-9")
    printer.fire("print_stats:paused_printing")
    assert native.ctx_calls[-1] == ("k-1-2", "print-9")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))

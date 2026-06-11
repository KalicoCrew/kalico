import contextlib

import klippy.printer


class FakeReactor:
    NOW = 0.0
    NEVER = 9999999999.0

    def register_callback(self, callback, waketime=NOW):
        return None

    def register_async_callback(self, callback, waketime=NOW):
        return None

    def register_fd(self, fd, read_callback, write_callback=None):
        return object()

    def unregister_fd(self, handle):
        return None

    def register_timer(self, callback, waketime=NEVER):
        return object()

    def unregister_timer(self, handle):
        return None

    def mutex(self, is_locked=False):
        return contextlib.nullcontext()

    def monotonic(self):
        return 0.0

    def run(self):
        return None

    def get_gc_stats(self):
        return (0, 0, 0)


def make_failed_connect_printer(monkeypatch):
    printer = klippy.printer.Printer(FakeReactor(), None, {})
    disconnects = []
    printer.register_event_handler(
        "klippy:disconnect", lambda: disconnects.append(1)
    )

    def raise_config_error():
        raise printer.config_error(
            "Option 'velocity_ff' is not valid in section 'servo_x'"
        )

    monkeypatch.setattr(printer, "_read_config", raise_config_error)
    return printer, disconnects


def test_config_error_keeps_printer_alive_and_reporting(monkeypatch):
    printer, disconnects = make_failed_connect_printer(monkeypatch)

    printer._connect(0.0)

    assert "velocity_ff" in printer.state_message
    assert not disconnects, (
        "a config error must not dispatch klippy:disconnect — webhooks must "
        "keep serving the error state to moonraker"
    )


def test_exit_after_failed_connect_dispatches_disconnect_once(monkeypatch):
    printer, disconnects = make_failed_connect_printer(monkeypatch)
    printer._connect(0.0)

    run_result = printer.run()

    assert run_result is None
    assert disconnects == [1]

    printer._dispatch_disconnect()
    assert disconnects == [1]

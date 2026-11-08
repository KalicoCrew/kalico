"""Interactive submissions hand off without advancing their nested scripts."""

import socket
from types import SimpleNamespace

import pytest

from klippy.extras.display.menu import MenuManager
from klippy.gcode import CommandError, GCodeDispatch, GCodeIO
from klippy.reactor import Reactor


@pytest.fixture
def dispatch():
    reactor = Reactor()
    messages, shutdowns = [], []
    printer = SimpleNamespace(
        get_start_args=lambda: {},
        is_shutdown=lambda: False,
        get_reactor=lambda: reactor,
        register_event_handler=lambda *args: None,
        lookup_object=lambda name, default=None: default,
        send_event=lambda event: None,
        invoke_shutdown=shutdowns.append,
    )
    gcode = GCodeDispatch(printer)
    printer.lookup_object = lambda name, default=None: (
        gcode if name == "gcode" else default
    )
    gcode._handle_ready()
    gcode.register_output_handler(messages.append)
    try:
        yield gcode, reactor, messages, shutdowns
    finally:
        reactor.finalize()


def run(reactor, callback):
    errors, completed = [], []

    def invoke(eventtime):
        try:
            callback()
            completed.append(True)
        except BaseException as error:
            errors.append(error)
        finally:
            reactor.end()

    reactor.register_callback(invoke)
    reactor.register_callback(
        lambda eventtime: reactor.end(), reactor.monotonic() + 1
    )
    reactor.run()
    if errors:
        raise errors[0]
    assert completed, "interactive dispatch did not finish"


@pytest.mark.parametrize("source", ["pty", "menu"])
def test_interactive_input_unblocks_nested_handler(dispatch, source):
    gcode, reactor, messages, shutdowns = dispatch
    accepted, finished = reactor.completion(), reactor.completion()
    events, captured = [], []

    def wait(gcmd):
        captured.append(gcmd)
        events.append("waiting")
        with gcmd.without_lock():
            accepted.wait()
        assert gcmd._context.owns_mutex()
        # Repeated releases must neither hand off again nor acknowledge again.
        with gcmd.without_lock():
            pass
        events.append("resumed")

    def outer(gcmd):
        gcmd.run_script("WAIT_TEST\nINNER_TEST")
        events.append("outer")
        finished.complete(None)

    def accept(gcmd):
        assert gcmd._context.owns_mutex()
        assert not captured[0]._context.owns_mutex()
        assert gcmd._context.get_root() is not captured[0]._context.get_root()
        assert events == ["waiting"]
        if source == "pty":
            assert messages == ["ok"]
        events.append("accept")
        accepted.complete(None)

    gcode.register_command("WAIT_TEST", wait)
    gcode.register_command("OUTER_TEST", outer)
    gcode.register_command("INNER_TEST", lambda gcmd: events.append("inner"))
    gcode.register_command("ACCEPT_TEST", accept)
    gcode.register_command("SCRIPT_TEST", lambda gcmd: events.append("script"))
    server, client = socket.socketpair()
    try:
        if source == "pty":
            gcode.printer.get_start_args = lambda: {"gcode_fd": server.fileno()}
            io = GCodeIO(gcode.printer)
            # Both commands arrive in one read, as well as the next partial line.
            client.sendall(b"OUTER_TEST\nACCEPT_TEST\nPARTIAL")

            def submit():
                io._process_data(reactor.monotonic())
                assert io.partial_input == "PARTIAL"
                assert not io.is_processing_data
                finished.wait()

        else:
            menu = MenuManager.__new__(MenuManager)
            menu.gcode = gcode
            menu.gcode_queue = ["OUTER_TEST\nSCRIPT_TEST", "ACCEPT_TEST"]

            def submit():
                menu.dispatch_gcode(reactor.monotonic())
                assert not menu.gcode_queue
                finished.wait()

        run(reactor, submit)
    finally:
        server.close()
        client.close()
    expected = ["waiting", "accept", "resumed", "inner", "outer"]
    assert events == expected + (["script"] if source == "menu" else [])
    assert messages == (["ok", "ok"] if source == "pty" else [])
    assert not shutdowns
    assert not gcode.get_mutex().test()
    with pytest.raises(CommandError, match="no longer valid"):
        captured[0].run_script("")


@pytest.mark.parametrize("after_handoff", [False, True])
@pytest.mark.parametrize("internal_error", [False, True])
def test_interactive_errors_are_reported_and_workers_finish(
    dispatch, after_handoff, internal_error
):
    gcode, reactor, messages, shutdowns = dispatch
    released, finished = reactor.completion(), reactor.completion()
    error = ValueError("bug") if internal_error else CommandError("bad command")

    def fail(gcmd):
        try:
            if after_handoff:
                with gcmd.without_lock():
                    released.wait()
            raise error
        finally:
            finished.complete(None)

    gcode.register_command("FAIL_TEST", fail)

    def submit():
        if after_handoff:
            gcode.run_interactive_script("FAIL_TEST")
            assert not messages
            released.complete(None)
            finished.wait()
        else:
            with pytest.raises(type(error)) as exc:
                gcode.run_interactive_script("FAIL_TEST")
            assert exc.value is error
        assert finished.test()

    run(reactor, submit)
    assert len(messages) == 1
    assert messages[0].startswith("!!")
    assert len(shutdowns) == int(internal_error)
    assert not gcode.get_mutex().test()


def test_pty_acknowledgements_and_error_recovery(dispatch):
    gcode, reactor, messages, shutdowns = dispatch
    server, client = socket.socketpair()
    gcode.printer.get_start_args = lambda: {"gcode_fd": server.fileno()}
    io = GCodeIO(gcode.printer)
    stored = []

    def explicit_ack(gcmd):
        stored.append(gcmd)
        assert gcmd.ack("T:25")
        assert not gcmd.ack()
        # Subcommands must not inherit the PTY acknowledgement callback.
        gcmd.run_script("SUB_TEST")
        reactor.pause(reactor.NOW)
        stored.append("done")

    def fail(gcmd):
        assert stored[-1] == "done"
        raise gcmd.error("bad command")

    gcode.register_command("ACK_TEST", explicit_ack)
    gcode.register_command("SUB_TEST", lambda gcmd: stored.append(gcmd.ack()))
    gcode.register_command("FAIL_TEST", fail)
    client.sendall(b"ACK_TEST\nFAIL_TEST\n; comment\n")
    try:
        run(reactor, lambda: io._process_data(reactor.monotonic()))
    finally:
        server.close()
        client.close()
    assert messages == ["ok T:25", "!! bad command", "ok", "ok"]
    assert stored[1] is False
    assert not stored[0].ack()
    assert not shutdowns
    assert not gcode.get_mutex().test()

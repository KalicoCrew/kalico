"""Root and nested contexts preserve mutex ownership across dispatch."""

from types import SimpleNamespace

import pytest

from klippy.gcode import (
    CommandError,
    GCodeContextLegacy,
    GCodeContextRoot,
    GCodeDispatch,
)
from klippy.reactor import Reactor


@pytest.fixture
def dispatch():
    reactor = Reactor()
    printer = SimpleNamespace(
        get_start_args=lambda: {},
        is_shutdown=lambda: False,
        get_reactor=lambda: reactor,
        register_event_handler=lambda *args: None,
        lookup_object=lambda name, default=None: default,
        send_event=lambda event: None,
        invoke_shutdown=lambda message: None,
    )
    gcode = GCodeDispatch(printer)
    gcode._handle_ready()
    try:
        yield gcode, reactor
    finally:
        reactor.finalize()


def test_run_without_gcode_lock_restores_lock_and_keeps_the_root(dispatch):
    gcode, _ = dispatch
    mutex = gcode.get_mutex()
    captured = []
    deferred = []

    def callback(value, *, extra):
        assert not mutex.test()
        assert not captured[0]._context.is_nested()
        if value < 0:
            raise ValueError("wait failed")
        return value + extra

    def handler(command):
        captured.append(command)
        deferred.append(command.without_lock())
        assert command.run_without_gcode_lock(callback, 5, extra=2) == 7
        with pytest.raises(ValueError, match="wait failed"):
            command.run_without_gcode_lock(callback, -1, extra=2)
        assert mutex.test()
        assert not command._context.is_nested()
        # Reacquiring the lock enables another scope within this handler.
        assert command.run_without_gcode_lock(lambda: 9) == 9

    def expired(command):
        with pytest.raises(CommandError, match="no longer valid"):
            with deferred[0]:
                pytest.fail("expired scope released the mutex")
        assert mutex.test()

    gcode.register_command("WAIT_TEST", handler)
    gcode.register_command("EXPIRED_TEST", expired)
    gcode.run_script("WAIT_TEST\nEXPIRED_TEST")
    assert not mutex.test()
    with (
        mutex,
        pytest.raises(CommandError, match="dispatched handler|no longer valid"),
    ):
        captured[0].run_without_gcode_lock(lambda: None)
    assert not mutex.test()


def test_nested_commands_share_the_root_context(dispatch):
    gcode, _ = dispatch
    commands = []

    def leaf(command):
        commands.append(command)
        assert command._context.get_root() is commands[0]._context.get_root()
        assert command.run_without_gcode_lock(lambda: 7) == 7

    def child(command):
        commands.append(command)
        assert command._context.get_root() is commands[0]._context.get_root()
        gcode.run_script_from_command("LEAF_TEST", command)

    def root(command):
        commands.append(command)
        command.run_script("CHILD_TEST\nRETURN\nUNREACHABLE")

    gcode.register_command("ROOT_TEST", root)
    gcode.register_command("CHILD_TEST", child)
    gcode.register_command("LEAF_TEST", leaf)
    gcode.run_script("ROOT_TEST")
    assert len(commands) == 3


def test_legacy_scripts_and_their_new_api_descendants_cannot_unlock(dispatch):
    gcode, _ = dispatch
    executed = []

    def child(command):
        executed.append(command.get_command())
        command.wait_while(lambda eventtime: False)
        assert gcode.get_mutex().test()
        with pytest.raises(
            CommandError, match="dispatched handler|no longer valid"
        ):
            command.run_without_gcode_lock(lambda: pytest.fail("legacy unlock"))
        if command.get_command() == "CHILD_TEST":
            command.run_script("LEAF_TEST\nRETURN\nUNREACHABLE")

    def root(command):
        gcode.run_script_from_command("CHILD_TEST\nRETURN\nUNREACHABLE")
        assert command.run_without_gcode_lock(lambda: 9) == 9

    gcode.register_command("ROOT_TEST", root)
    gcode.register_command("CHILD_TEST", child)
    gcode.register_command("LEAF_TEST", child)
    gcode.run_script("ROOT_TEST")
    assert executed == ["CHILD_TEST", "LEAF_TEST"]


class RecordingMutex:
    def __init__(self, mutex):
        self.mutex = mutex
        self.enters = self.exits = 0

    def test(self):
        return self.mutex.test()

    def __enter__(self):
        self.mutex.__enter__()
        self.enters += 1

    def __exit__(self, exc_type, exc_value, traceback):
        self.exits += 1
        return self.mutex.__exit__(exc_type, exc_value, traceback)


def record_mutex(gcode):
    mutex = RecordingMutex(gcode.get_mutex())
    gcode.mutex = mutex
    return mutex


def test_dispatch_acquires_once_at_first_ordinary_command(dispatch):
    gcode, _ = dispatch
    mutex = record_mutex(gcode)
    seen = []

    def handler(command):
        seen.append(
            (
                command.get_command(),
                mutex.test(),
                command._context.get_mutex()
                if command._context.owns_mutex()
                else None,
            )
        )

    gcode.register_command("FREE_TEST", handler, lockfree=True)
    gcode.register_command("NORMAL_TEST", handler)
    gcode.run_script("FREE_TEST\nNORMAL_TEST\nFREE_TEST\nNORMAL_TEST")
    assert seen == [
        ("FREE_TEST", False, None),
        ("NORMAL_TEST", True, mutex),
        ("FREE_TEST", True, mutex),
        ("NORMAL_TEST", True, mutex),
    ]
    assert (mutex.enters, mutex.exits) == (1, 1)
    assert not mutex.test()
    # Re-registering a formerly lockfree command must acquire the mutex.
    gcode.register_command("FREE_TEST", None)
    assert "FREE_TEST" not in gcode.gcode_lockfree
    gcode.register_command("FREE_TEST", handler)
    gcode.run_script("FREE_TEST")
    assert seen[-1] == ("FREE_TEST", True, mutex)
    assert (mutex.enters, mutex.exits) == (2, 2)


def test_lockfree_command_runs_while_another_invocation_holds_mutex(dispatch):
    gcode, reactor = dispatch
    mutex = record_mutex(gcode)
    events = []
    completed = []

    def owner(command):
        events.append("owner start")
        reactor.pause(reactor.monotonic() + 0.01)
        events.append("owner end")

    def free(command):
        assert mutex.test()
        assert not command._context.owns_mutex()
        events.append("lockfree")

    def ordinary(command):
        assert command._context.get_mutex() is mutex
        events.append("ordinary")

    def contender(eventtime):
        gcode.run_script("FREE_TEST\nNORMAL_TEST")
        completed.append(True)
        reactor.end()

    gcode.register_command("OWNER_TEST", owner)
    gcode.register_command("FREE_TEST", free, lockfree=True)
    gcode.register_command("NORMAL_TEST", ordinary)
    reactor.register_callback(lambda time: gcode.run_script("OWNER_TEST"))
    reactor.register_callback(contender)
    reactor.register_callback(
        lambda time: reactor.end(), reactor.monotonic() + 0.5
    )
    reactor.run()
    assert completed
    assert events == ["owner start", "lockfree", "owner end", "ordinary"]
    assert (mutex.enters, mutex.exits) == (2, 2)
    assert not mutex.test()


def test_serial_error_keeps_the_mutex_until_the_batch_finishes(dispatch):
    gcode, _ = dispatch
    mutex = record_mutex(gcode)
    seen = []
    defaults = []
    original_default = gcode.cmd_default

    def default(command):
        assert mutex.test()
        defaults.append(command.get_commandline())
        original_default(command)

    gcode.cmd_default = default

    def fail(command):
        assert mutex.test()
        raise CommandError("handler failed")

    def after(command):
        seen.append(command._context.get_mutex())
        assert mutex.test()

    gcode.register_command("FAIL_TEST", fail)
    gcode.register_command("AFTER_TEST", after)
    gcode._process_commands(
        ["", "; comment", "FAIL_TEST", "AFTER_TEST"],
        need_ack=True,
        context=GCodeContextRoot(gcode.get_mutex()),
    )
    assert defaults == ["", "; comment"]
    assert seen == [mutex]
    assert (mutex.enters, mutex.exits) == (1, 1)
    assert not mutex.test()


def test_dispatch_uses_only_the_mutex_in_its_context(dispatch):
    gcode, reactor = dispatch
    dispatcher_mutex = record_mutex(gcode)
    provided_mutex = RecordingMutex(reactor.mutex())
    seen = []

    def handler(command):
        seen.append(command._context.get_mutex())
        assert provided_mutex.test() == (borrowed or not lockfree)

    for lockfree, borrowed in [(False, False), (True, True)]:
        gcode.register_command("CONTEXT_TEST", handler, lockfree=lockfree)
        if borrowed:
            with provided_mutex:
                gcode._process_commands(
                    ["CONTEXT_TEST"],
                    need_ack=False,
                    context=GCodeContextLegacy(provided_mutex),
                )
                assert provided_mutex.test()
        else:
            gcode._process_commands(
                ["CONTEXT_TEST"],
                need_ack=False,
                context=GCodeContextRoot(provided_mutex),
            )
        gcode.register_command("CONTEXT_TEST", None)
        assert not provided_mutex.test()
    assert seen == [provided_mutex, provided_mutex]
    assert (dispatcher_mutex.enters, dispatcher_mutex.exits) == (0, 0)
    assert (provided_mutex.enters, provided_mutex.exits) == (2, 2)


def test_nested_acquisition_releases_after_error_and_can_be_retried(dispatch):
    gcode, _ = dispatch
    mutex = record_mutex(gcode)
    events = []

    def fail(command):
        assert mutex.test()
        raise CommandError("nested failure")

    def normal(command):
        assert mutex.test()
        events.append("retried")

    def free(command):
        with pytest.raises(CommandError, match="nested failure"):
            command.run_script("FAIL_TEST")
        assert not command._context.owns_mutex()
        assert not mutex.test()
        command.run_script("NORMAL_TEST\nRETURN\nUNREACHABLE")
        assert not command._context.owns_mutex()
        assert not mutex.test()

    gcode.register_command("FAIL_TEST", fail)
    gcode.register_command("NORMAL_TEST", normal)
    gcode.register_command("FREE_TEST", free, lockfree=True)
    gcode.run_script("FREE_TEST")
    assert events == ["retried"]
    assert (mutex.enters, mutex.exits) == (2, 2)


def test_direct_subcommands_share_scope_until_dispatched_handler_returns(
    dispatch,
):
    gcode, _ = dispatch
    captured = []

    def direct(child):
        grandchild = child.create_subcommand("LEAF_TEST", "LEAF_TEST", {})
        captured.extend((child, grandchild))
        assert grandchild._context is child._context
        assert grandchild.run_without_gcode_lock(lambda: 7) == 7
        raise ValueError("direct handler failed")

    def handler(command):
        child = command.create_subcommand("CHILD_TEST", "CHILD_TEST", {})
        assert child._context is command._context
        with pytest.raises(ValueError, match="direct handler failed"):
            direct(child)
        for subcommand in captured:
            assert subcommand._context.owns_mutex()
            assert subcommand.run_without_gcode_lock(lambda: 9) == 9
        assert command._context.owns_mutex()

    gcode.register_command("PARENT_TEST", handler)
    gcode.run_script("PARENT_TEST")
    with gcode.get_mutex():
        for expired in captured:
            with pytest.raises(CommandError, match="no longer valid"):
                expired.run_without_gcode_lock(lambda: pytest.fail("ran"))
            with pytest.raises(CommandError, match="no longer valid"):
                expired.run_script("FREE_TEST")


def test_guard_error_acknowledges_serial_command_without_calling_handler(
    dispatch,
):
    gcode, _ = dispatch
    with pytest.raises(CommandError, match="requires the G-code mutex"):
        gcode.run_script_from_command("M112")
    standalone = gcode.create_gcode_command("WAIT_TEST", "WAIT_TEST", {})
    with gcode.get_mutex():
        with pytest.raises(CommandError, match="dispatched handler"):
            standalone.run_without_gcode_lock(
                lambda: pytest.fail("callback ran")
            )
    messages = []
    gcode.register_output_handler(messages.append)
    gcode.register_command(
        "NORMAL_TEST", lambda command: pytest.fail("handler ran")
    )
    context = GCodeContextLegacy(gcode.printer.get_reactor().mutex())
    with gcode.get_mutex():
        with pytest.raises(CommandError, match="cannot acquire"):
            gcode._process_commands(
                ["NORMAL_TEST"], need_ack=False, context=context
            )
        gcode._process_commands(["NORMAL_TEST"], need_ack=True, context=context)
        assert gcode.get_mutex().test()
    assert not gcode.get_mutex().test()
    assert messages[0].startswith(
        "!! A legacy G-code context cannot acquire the mutex"
    )
    assert messages[-1] == "ok"


def test_with_new_context_replays_expired_command_in_live_scope(dispatch):
    gcode, _ = dispatch
    originals = []
    copies = []
    donors = []
    gcode.register_command("ORIGINAL", originals.append)

    def replay(gcmd):
        donors.append(gcmd)
        original = originals[0]
        assert gcmd._context.get_root() is original._context.get_root()
        assert gcmd._context.owns_mutex()
        with pytest.raises(CommandError, match="no longer valid"):
            original._context.owns_mutex()
        duplicate = original.with_new_context(gcmd)
        copies.append(duplicate)
        assert duplicate.get_command() == original.get_command()
        assert duplicate.get_commandline() == original.get_commandline()
        assert (
            duplicate.get_raw_command_parameters() == 'VALUE="original value"'
        )
        assert duplicate.get("VALUE") == "original value"
        assert duplicate._context is gcmd._context
        assert not duplicate.ack()
        duplicate.get_command_parameters()["VALUE"] = "changed"
        assert original.get("VALUE") == "original value"
        assert gcmd.get("VALUE") == "resume value"
        assert duplicate.run_without_gcode_lock(lambda: 7) == 7

    gcode.register_command("REPLAY", replay)
    gcode.run_script(
        'ORIGINAL VALUE="original value"\nREPLAY VALUE="resume value"'
    )
    original = originals[0]
    with pytest.raises(CommandError, match="no longer valid"):
        copies[0].run_script("")
    with pytest.raises(CommandError, match="no longer valid"):
        original.with_new_context(donors[0])

"""Command waits preserve mutex ownership and wake on interrupts."""

import threading
from types import SimpleNamespace

import pytest

from klippy.extras import control_mpc, gcode_macro, heaters, pid_calibrate
from klippy.gcode import (
    CommandError,
    GCodeDispatch,
    WaitInterruption,
)
from klippy.printer import Printer
from klippy.reactor import Reactor


@pytest.fixture
def dispatch():
    reactor = Reactor()
    objects = {}
    handlers = {}
    shutdown = []

    def send_event(event):
        for callback in handlers.get(event, []):
            callback()

    def invoke_shutdown(message):
        shutdown.append(message)
        send_event("klippy:shutdown")

    printer = SimpleNamespace(
        command_error=CommandError,
        get_start_args=lambda: {},
        get_reactor=lambda: reactor,
        is_shutdown=lambda: bool(shutdown),
        register_event_handler=lambda name, callback: handlers.setdefault(
            name, []
        ).append(callback),
        lookup_object=lambda name, default=None: objects.get(name, default),
        send_event=send_event,
        invoke_shutdown=invoke_shutdown,
    )
    gcode = GCodeDispatch(printer)
    objects["gcode"] = gcode
    gcode._handle_ready()
    try:
        yield gcode, reactor, printer, objects
    finally:
        reactor.finalize()


def run_in_reactor(reactor, action):
    errors = []
    finished = []

    def invoke(eventtime):
        try:
            action()
        except BaseException as error:
            errors.append(error)
        finally:
            finished.append(True)
            reactor.end()

    reactor.register_callback(invoke)
    reactor.register_callback(
        lambda eventtime: reactor.end(), reactor.monotonic() + 0.5
    )
    reactor.run()
    if errors:
        raise errors[0]
    assert finished, "wait did not wake before the watchdog"


def test_wait_after_shutdown_does_not_evaluate_predicate(dispatch):
    gcode, _, printer, _ = dispatch
    printer.invoke_shutdown("test shutdown")

    def predicate(eventtime):
        pytest.fail("predicate ran")

    def handler(command):
        for error_on_cancel in (True, False):
            with pytest.raises(WaitInterruption, match="Printer is shutdown"):
                command.wait_while(predicate, error_on_cancel)
            with pytest.raises(WaitInterruption, match="Printer is shutdown"):
                command.sleep(1.0, error_on_cancel)

    gcode.register_command("WAIT_TEST", handler, when_not_ready=True)
    gcode.run_script("WAIT_TEST")
    for error_on_cancel in (True, False):
        with pytest.raises(WaitInterruption, match="Printer is shutdown"):
            Printer.wait_while(printer, predicate, error_on_cancel)


@pytest.mark.parametrize("source", ["command", "shutdown", "disconnect"])
def test_interrupt_wakes_wait_immediately_and_reacquires_mutex(
    dispatch, source
):
    gcode, reactor, printer, _ = dispatch
    events = []

    def mark(command):
        assert command._context.owns_mutex()
        events.append("other command")

    def interrupt(eventtime):
        assert not gcode.get_mutex().test()
        gcode.run_script("MARK")
        if source == "command":
            gcode.run_script("HEATER_INTERRUPT")
        elif source == "shutdown":
            gcode.interrupt()
            printer.invoke_shutdown("test shutdown")
        else:
            printer.send_event("klippy:disconnect")

    def handler(command):
        def condition(eventtime):
            assert not command._context.owns_mutex()
            assert not gcode.get_mutex().test()
            events.append("condition")
            reactor.register_callback(interrupt, eventtime + 0.005)
            return True

        try:
            message = (
                "Printer is shutdown"
                if source == "shutdown"
                else "Command interrupted"
            )
            with (
                pytest.raises(WaitInterruption, match=message),
                command.without_lock(),
            ):
                command.wait_while(
                    condition,
                    error_on_cancel=source != "shutdown",
                    interval=30.0,
                )
        finally:
            assert command._context.owns_mutex()
            events.append("cleanup")

    gcode.register_command("WAIT_TEST", handler)
    gcode.register_command("MARK", mark)
    run_in_reactor(reactor, lambda: gcode.run_script("WAIT_TEST"))
    assert events == ["condition", "other command", "cleanup"]
    assert not gcode.get_mutex().test()


def test_tokens_are_captured_by_command_scope_not_at_wait_entry(dispatch):
    gcode, _, _, _ = dispatch
    captured = []

    def child(command):
        captured.append(command._context.get_interrupt_token())
        assert not captured[-1].test()
        command.wait_while(lambda eventtime: False)

    def parent(command):
        token = command._context.get_interrupt_token()
        direct = command.create_subcommand("DIRECT", "DIRECT", {})
        assert direct._context.get_interrupt_token() is token
        gcode.interrupt()
        for stale in (command, direct):
            with pytest.raises(WaitInterruption):
                stale.wait_while(
                    lambda eventtime: pytest.fail("stale predicate ran")
                )
            stale.wait_while(
                lambda eventtime: pytest.fail("stale predicate ran"), False
            )
        command.run_script("CHILD_TEST")
        assert captured[-1] is not token
        assert token.test()

    gcode.register_command("PARENT_TEST", parent)
    gcode.register_command("CHILD_TEST", child)
    gcode.run_script("PARENT_TEST\nCHILD_TEST")
    assert len(captured) == 2


def test_one_token_wakes_multiple_independent_waiters(dispatch):
    gcode, reactor, _, _ = dispatch
    tokens = []
    completed = []

    def handler(command):
        tokens.append(command._context.get_interrupt_token())
        with pytest.raises(WaitInterruption), command.without_lock():
            command.wait_while(lambda eventtime: True, interval=30.0)
        assert command._context.owns_mutex()
        completed.append(command)

    gcode.register_command("WAIT_TEST", handler)

    def action():
        reactor.register_callback(
            lambda eventtime: gcode.run_script("WAIT_TEST")
        )
        reactor.register_callback(
            lambda eventtime: gcode.run_script("HEATER_INTERRUPT"),
            reactor.monotonic() + 0.005,
        )
        gcode.run_script("WAIT_TEST")
        # Let the other awakened waiter reacquire the mutex and finish.
        reactor.pause(reactor.monotonic() + 0.005)

    run_in_reactor(reactor, action)
    assert len(completed) == 2
    assert tokens[0] is tokens[1]


def heater_manager(dispatch):
    gcode, _, printer, objects = dispatch
    manager = heaters.PrinterHeaters.__new__(heaters.PrinterHeaters)
    manager.printer = printer
    manager.gcode = gcode
    manager._get_temp = lambda eventtime: "T:20 /200"
    objects["toolhead"] = SimpleNamespace(
        register_lookahead_callback=lambda callback: None,
        get_last_move_time=lambda: 0.0,
    )
    return manager


def test_heater_setter_wait_is_interruptible(dispatch):
    gcode, reactor, _, _ = dispatch
    manager = heater_manager(dispatch)
    targets = []
    heater = SimpleNamespace(
        set_temp=targets.append, check_busy=lambda eventtime: True
    )

    manager.set_temperature(heater, 150.0)

    def handler(command):
        manager.set_temperature(
            heater,
            200.0,
            True,
            command if command.get_int("WITH_COMMAND") else None,
        )

    def action():
        for with_command in (0, 1):
            reactor.register_callback(
                lambda eventtime: gcode.run_script("HEATER_INTERRUPT"),
                reactor.monotonic() + 0.005,
            )
            with pytest.raises(WaitInterruption):
                gcode.run_script("HEAT_TEST WITH_COMMAND=%d" % with_command)

    gcode.register_command("HEAT_TEST", handler)
    run_in_reactor(reactor, action)
    assert targets == [150.0, 200.0, 200.0]


def test_temperature_wait_preserves_allow_cancel(dispatch):
    gcode, reactor, _, _ = dispatch
    manager = heater_manager(dispatch)
    manager.available_sensors = ["heater"]
    manager.heaters = {
        "heater": SimpleNamespace(
            get_temp=lambda eventtime: (20.0, 200.0),
            check_not_calibrating=lambda: None,
        )
    }
    gcode.register_command("TEMPERATURE_WAIT", manager.cmd_TEMPERATURE_WAIT)

    def action():
        reactor.register_callback(
            lambda eventtime: gcode.run_script("HEATER_INTERRUPT"),
            reactor.monotonic() + 0.005,
        )
        script = "TEMPERATURE_WAIT SENSOR=heater MINIMUM=100"
        gcode.run_script(script + " ALLOW_CANCEL=1")

    run_in_reactor(reactor, action)


def calibration_heater(dispatch):
    _, reactor, printer, _ = dispatch
    heater = heaters.Heater.__new__(heaters.Heater)
    heater.printer = printer
    heater.reactor = reactor
    heater.name = "extruder"
    heater.min_temp = 0.0
    heater.max_temp = 300.0
    heater.max_power = 1.0
    heater.lock = threading.Lock()
    heater._calibration = None
    heater.smoothed_temp = 20.0
    heater.target_temp = 0.0
    heater.last_pwm_value = 0.0
    heater.get_temp = lambda eventtime: (20.0, heater.target_temp)
    pwm = []
    heater.set_pwm = lambda time, value: pwm.append(value)
    heater.control = SimpleNamespace(
        ambient_sensor=SimpleNamespace(get_temp=lambda eventtime: (20, 0)),
        temperature_update=lambda time, temp, target: heater.set_pwm(time, 0.0),
    )
    return heater, pwm


def test_calibration_rejects_competitors_and_restores_controller(dispatch):
    gcode, reactor, printer, objects = dispatch
    manager = heater_manager(dispatch)
    heater, pwm = calibration_heater(dispatch)
    original = heater.control
    objects["heaters"] = manager
    manager.lookup_heater = lambda name: heater
    pid = pid_calibrate.PIDCalibrate.__new__(pid_calibrate.PIDCalibrate)
    pid.printer = printer
    mpc = control_mpc.MpcCalibrate(printer, heater, original)

    def calibrate(gcmd, callback):
        wait = gcmd.wait_while
        gcmd.wait_while = lambda *args, **kwargs: gcmd.run_without_gcode_lock(
            wait, *args, **kwargs
        )
        callback(gcmd)

    gcode.register_command(
        "PID_CALIBRATE", lambda gcmd: calibrate(gcmd, pid.cmd_PID_CALIBRATE)
    )
    gcode.register_command(
        "MPC_CALIBRATE", lambda gcmd: calibrate(gcmd, mpc.run)
    )
    attempts = []

    def compete(eventtime, active):
        assert not gcode.get_mutex().test()
        installed = heater.control
        assert installed is not original
        competitors = ("PID", "MPC") if active == "PID" else ("PID",)
        for competing in competitors:
            with pytest.raises(CommandError, match="is being calibrated"):
                gcode.run_script(
                    competing + "_CALIBRATE HEATER=extruder TARGET=200"
                )
        assert heater.control is installed
        other, _ = calibration_heater(dispatch)
        with other.calibration() as calibration:
            calibration.set_control(original)
        attempts.append(active)
        gcode.run_script("HEATER_INTERRUPT")

    def action():
        for active in ("PID", "MPC"):
            reactor.register_callback(
                lambda eventtime, active=active: compete(eventtime, active),
                reactor.monotonic() + 0.005,
            )
            with pytest.raises(CommandError, match="Command interrupted"):
                gcode.run_script(
                    active + "_CALIBRATE HEATER=extruder TARGET=200"
                )
            assert heater.control is original
            assert heater.target_temp == 0.0
            assert heater._calibration is None
            heater.control.temperature_update(
                reactor.monotonic(), 20.0, heater.target_temp
            )
            assert heater.target_temp == 0.0
            assert pwm[-1] == 0.0
            # Calibration ownership has been released, even after interruption.
            assert heater.set_control(original) is original

    run_in_reactor(reactor, action)
    assert attempts == ["PID", "MPC"]


def python_template(dispatch, script):
    _, _, printer, objects = dispatch
    macro = gcode_macro.PrinterGCodeMacro.__new__(gcode_macro.PrinterGCodeMacro)
    macro.printer = printer
    objects["gcode_macro"] = macro
    return gcode_macro.TemplateWrapperPython(printer, None, "wait-test", script)


def test_python_wait_helpers_use_the_entry_points_token(dispatch):
    gcode, reactor, _, _ = dispatch
    reached_sleep = []
    template = python_template(
        dispatch,
        "wait_while(lambda: False)\nwait_until(lambda: True)\nsleep(0)\n"
        "emit('REACHED_SLEEP')\nsleep(30)",
    )
    gcode.register_command(
        "REACHED_SLEEP", lambda command: reached_sleep.append(True)
    )

    def handler(command):
        # The command retains this cancelled token; the dispatch has a fresh one.
        gcode.interrupt()
        if command.get_int("LEGACY"):
            reactor.register_callback(
                lambda eventtime: gcode.run_script("HEATER_INTERRUPT"),
                reactor.monotonic() + 0.005,
            )
            template.run_gcode_from_command()
        else:
            template.run_gcode(command)

    gcode.register_command("PY_TEST", handler)

    def action():
        for legacy in (0, 1):
            with pytest.raises(WaitInterruption):
                gcode.run_script("PY_TEST LEGACY=%d" % legacy)
            assert reached_sleep == ([True] if legacy else [])
            assert not gcode.get_mutex().test()

    run_in_reactor(reactor, action)


def test_printer_waits_follow_token_replacement_and_shutdown(dispatch):
    gcode, reactor, printer, _ = dispatch
    old = gcode.get_interrupt_token()
    gcode.interrupt()
    old.interrupt()
    assert old.wait() is True
    checked = []

    def action():
        for shutdown in (False, True):
            token = gcode.get_interrupt_token()
            assert token is not old and not token.test()

            def condition(eventtime):
                assert not gcode.get_mutex().test()
                checked.append(shutdown)
                if shutdown:

                    def stop(eventtime):
                        gcode.interrupt()
                        printer.invoke_shutdown("test shutdown")

                    reactor.register_callback(stop, eventtime + 0.005)
                else:
                    reactor.register_callback(
                        lambda eventtime: gcode.interrupt(), eventtime + 0.005
                    )
                return True

            if shutdown:
                with pytest.raises(
                    WaitInterruption, match="Printer is shutdown"
                ):
                    Printer.wait_while(printer, condition, False, 30.0)
            else:
                assert (
                    Printer.wait_while(printer, condition, False, 30.0) is None
                )
            assert token.test() and not gcode.get_interrupt_token().test()

    run_in_reactor(reactor, action)
    assert checked == [False, True]


@pytest.mark.parametrize("with_command", [False, True])
def test_sleep_waits_until_deadline_preserving_mutex(dispatch, with_command):
    gcode, reactor, _, _ = dispatch
    marks = []
    gcode.register_command(
        "MARK", lambda command: marks.append(True), lockfree=True
    )

    def handler(command):
        calls = []

        def waiting(eventtime):
            assert command._context.owns_mutex()
            assert gcode.get_mutex().test()
            calls.append(eventtime)
            return len(calls) < 2

        command.wait_while(waiting, interval=0.001)
        assert len(calls) == 2
        start = reactor.monotonic()
        command.sleep(0.005)
        assert reactor.monotonic() >= start + 0.005
        assert command._context.owns_mutex()

    gcode.register_command("SLEEP_TEST", handler)

    def mark(eventtime):
        assert gcode.get_mutex().test() == with_command
        gcode.run_script("MARK")

    def action():
        start = reactor.monotonic()
        reactor.register_callback(mark, start + 0.001)
        if with_command:
            gcode.run_script("SLEEP_TEST")
        else:
            gcode.get_interrupt_token().sleep(0.005)
        assert reactor.monotonic() >= start + 0.005

    run_in_reactor(reactor, action)
    assert marks == [True]
    assert not gcode.get_mutex().test()


def test_token_sleep_can_ignore_interruption(dispatch):
    gcode, reactor, _, _ = dispatch

    def action():
        token = gcode.get_interrupt_token()
        reactor.register_callback(lambda eventtime: gcode.interrupt())
        token.sleep(30.0, False)
        assert token.test()
        assert gcode.get_interrupt_token() is not token

    run_in_reactor(reactor, action)


def test_sleep_can_explicitly_release_the_mutex(dispatch):
    gcode, reactor, _, _ = dispatch
    marks = []
    gcode.register_command("MARK", lambda command: marks.append(True))

    def handler(command):
        reactor.register_callback(lambda eventtime: gcode.run_script("MARK"))
        with command.without_lock():
            command.sleep(0.005)
        assert command._context.owns_mutex()

    gcode.register_command("SLEEP_TEST", handler)
    run_in_reactor(reactor, lambda: gcode.run_script("SLEEP_TEST"))
    assert marks == [True]


def test_heater_commands_reject_only_calibrating_heaters(dispatch):
    from klippy.extras import heated_fan, heater_bed, homing_heaters
    from klippy.kinematics.extruder import PrinterExtruder

    gcode, _, printer, objects = dispatch
    manager = heater_manager(dispatch)
    hotend, _ = calibration_heater(dispatch)
    bed, _ = calibration_heater(dispatch)
    bed.name = "heater_bed"
    hotend.target_temp, bed.target_temp = 200.0, 60.0
    manager.heaters = {"heater_bed": bed, "extruder": hotend}
    manager.available_sensors = list(manager.heaters) + [
        "heater_generic hotend"
    ]
    objects["heater_generic hotend"] = hotend
    objects["heaters"] = manager
    extruder = PrinterExtruder.__new__(PrinterExtruder)
    extruder.printer, extruder.heater = printer, hotend
    objects["toolhead"].get_extruder = lambda: extruder
    bed_commands = heater_bed.PrinterHeaterBed.__new__(
        heater_bed.PrinterHeaterBed
    )
    bed_commands.printer, bed_commands.heater = printer, bed
    fan = heated_fan.HeatedFan.__new__(heated_fan.HeatedFan)
    fan.heater, fan.heater_temp, fan.set_speed = hotend, 50.0, 0.25
    homing = homing_heaters.HomingHeaters.__new__(homing_heaters.HomingHeaters)
    homing.flaky_steppers = None
    homing.disable_heaters = list(manager.heaters)
    homing.pheaters, homing.target_save = manager, {}
    pmgr = heaters.Heater.ProfileManager.__new__(heaters.Heater.ProfileManager)
    pmgr.outer_instance = hotend
    handlers = {
        "PID_PROFILE": pmgr.cmd_PID_PROFILE,
        "SET_HEATER_PID": hotend.cmd_SET_HEATER_PID,
        "SET_SMOOTH_TIME": hotend.cmd_SET_SMOOTH_TIME,
        "M104": extruder.cmd_M104,
        "M109": extruder.cmd_M109,
        "M302": extruder.cmd_M302,
        "M140": bed_commands.cmd_M140,
        "M190": bed_commands.cmd_M190,
        "SET_HEATER_TEMPERATURE": hotend.cmd_SET_HEATER_TEMPERATURE,
        "COLD_EXTRUDE": hotend.cmd_COLD_EXTRUDE,
        "TEMPERATURE_WAIT": manager.cmd_TEMPERATURE_WAIT,
        "TURN_OFF_HEATERS": manager.cmd_TURN_OFF_HEATERS,
        "SET_HEATED_FAN_TARGET": fan.cmd_SET_HEATED_FAN_TARGET,
        "M106": fan.cmd_M106,
        "M107": fan.cmd_M107,
        "HOME_GUARD": lambda command: homing.handle_homing_move_begin(
            SimpleNamespace(get_mcu_endstops=lambda: [])
        ),
    }
    for name, handler in handlers.items():
        gcode.register_command(name, handler)

    def handler(command):
        with hotend.calibration():
            with pytest.raises(CommandError, match="is being calibrated"):
                hotend.set_control(hotend.control)
            for script in (
                "PID_PROFILE LOAD=default",
                "SET_HEATER_PID KP=10",
                "SET_SMOOTH_TIME SMOOTH_TIME=1",
                "M104 S0",
                "M109 S0",
                "M302 P1",
                "SET_HEATER_TEMPERATURE TARGET=0",
                "COLD_EXTRUDE ENABLE=1",
                "TEMPERATURE_WAIT SENSOR=extruder MINIMUM=100",
                'TEMPERATURE_WAIT SENSOR="heater_generic hotend" MINIMUM=100',
                "TURN_OFF_HEATERS",
                "M106 S0",
                "M107",
                "SET_HEATED_FAN_TARGET TARGET=0",
                "HOME_GUARD",
            ):
                with pytest.raises(CommandError, match="is being calibrated"):
                    command.run_script(script)
                assert hotend.target_temp == 200.0
                assert bed.target_temp == 60.0
            assert homing.target_save == {}
            assert (fan.heater_temp, fan.set_speed) == (50.0, 0.25)
            command.run_script("M140 S65")
            assert bed.target_temp == 65.0
        with bed.calibration():
            for script in ("M140 S0", "M190 S0"):
                with pytest.raises(CommandError, match="is being calibrated"):
                    command.run_script(script)
            command.run_script("M104 S210")
            assert hotend.target_temp == 210.0
            assert bed.target_temp == 65.0
        command.run_script("TURN_OFF_HEATERS")
        assert hotend.target_temp == bed.target_temp == 0.0

    gcode.register_command("GUARD_TEST", handler)
    gcode.run_script("GUARD_TEST")

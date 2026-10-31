"""Manual probes use normal selection and pause their caller until ACCEPT."""

import configparser
from types import SimpleNamespace

import pytest

from klippy import pins
from klippy.configfile import ConfigWrapper
from klippy.extras import manual_probe as manual_probe_module
from klippy.extras.manual_probe import ManualProbe
from klippy.extras.probe import PrinterProbe, ProbeList, ProbePointsHelper
from klippy.extras.probe_eddy_current import EddyCalibration
from klippy.gcode import CommandError, GCodeDispatch
from klippy.reactor import Reactor


class Toolhead:
    def __init__(self):
        self.position = [0.0, 0.0, 5.0, 0.0]

    def get_status(self, eventtime):
        return {"homed_axes": "xyz"}

    def get_position(self):
        return self.position[:]

    def manual_move(self, position, speed):
        for axis, value in enumerate(position):
            if value is not None:
                self.position[axis] = value

    def flush_step_generation(self):
        pass

    def get_last_move_time(self):
        return 0.0

    def get_kinematics(self):
        return SimpleNamespace(
            get_steppers=lambda: [],
            calc_position=lambda positions: self.get_position(),
        )


@pytest.fixture
def context():
    reactor = Reactor()
    objects = {
        "toolhead": Toolhead(),
        "pins": SimpleNamespace(register_chip=lambda *args: None),
        "gcode_move": SimpleNamespace(),
        "gcode_macro": SimpleNamespace(load_template=lambda *args: None),
    }
    handlers = {}

    def send_event(event):
        for callback in handlers.get(event, []):
            callback()

    printer = SimpleNamespace(
        get_start_args=lambda: {},
        is_shutdown=lambda: False,
        get_reactor=lambda: reactor,
        lookup_object=lambda name, default=None: objects.get(name, default),
        load_object=lambda config, name: objects[name],
        add_object=lambda name, obj: objects.__setitem__(name, obj),
        register_event_handler=lambda event, callback: handlers.setdefault(
            event, []
        ).append(callback),
        send_event=send_event,
        config_error=configparser.Error,
        command_error=CommandError,
        invoke_shutdown=lambda message: pytest.fail(message),
    )
    gcode = GCodeDispatch(printer)
    objects["gcode"] = gcode
    parser = configparser.RawConfigParser()
    parser.read_dict(
        {"manual_probe": {}, "printer": {"kinematics": "cartesian"}}
    )
    config = ConfigWrapper(printer, parser, {}, "manual_probe")
    manual = ManualProbe(config)
    objects["manual_probe"] = manual
    gcode._handle_ready()
    try:
        yield printer, config, manual, gcode, reactor
    finally:
        reactor.finalize()


@pytest.mark.parametrize("default", [False, True])
def test_manual_fallback_respects_explicit_hardware_default(context, default):
    printer, config, manual, gcode, reactor = context
    probe_list = ProbeList.get_list(printer)
    wrapped = probe_list.get_probe("manual")
    assert isinstance(wrapped, PrinterProbe)
    assert wrapped.mcu_probe is manual
    assert wrapped.sample_count == 1 and wrapped.get_offsets() == (0, 0, 0)
    with pytest.raises(CommandError, match="no endstop state"):
        wrapped.mcu_probe.query_endstop(0.0)
    with pytest.raises(pins.error, match="virtual endstop"):
        wrapped.setup_pin(
            "endstop",
            {"pin": "z_virtual_endstop", "invert": False, "pullup": False},
        )
    assert probe_list.get_default_probe() is None
    hardware = SimpleNamespace(probe_name="hardware", is_default_probe=default)
    probe_list.probes["hardware"] = hardware
    if default:
        probe_list.default_probe = ("hardware", hardware)
    printer.send_event("klippy:connect")
    assert probe_list.get_default_probe() is (hardware if default else wrapped)
    assert wrapped.is_default_probe == (not default)
    assert probe_list.get_config_probe(config) is (
        hardware if default else wrapped
    )
    unselected = gcode.create_gcode_command("PROBE", "PROBE", {})
    assert probe_list.get_command_probe(unselected) is (
        hardware if default else wrapped
    )
    assert (
        probe_list.get_command_probe(unselected, default_probe_name="manual")
        is wrapped
    )
    named = gcode.create_gcode_command("PROBE", "PROBE", {"PROBE": "hardware"})
    assert (
        probe_list.get_command_probe(named, default_probe_name="manual")
        is hardware
    )
    command = gcode.create_gcode_command("PROBE", "PROBE", {"METHOD": "manual"})
    assert probe_list.get_command_probe(command) is wrapped
    command = gcode.create_gcode_command("PROBE", "PROBE", {"PROBE": "manual"})
    assert probe_list.get_command_probe(command) is wrapped


def run_interactive(context, script, action):
    printer, config, manual, gcode, reactor = context
    finished = []
    errors = []

    def begin(eventtime):
        try:
            gcode.run_script(script)
        except CommandError as error:
            errors.append(str(error))
        finally:
            finished.append(True)
            reactor.end()

    def interact(eventtime):
        if manual.get_status(eventtime)["is_active"]:
            action()
        return eventtime + 0.001

    def stop(eventtime):
        reactor.end()
        return reactor.NEVER

    reactor.register_callback(begin)
    interaction = reactor.register_timer(interact, reactor.NOW)
    watchdog = reactor.register_timer(stop, reactor.monotonic() + 1)
    try:
        reactor.run()
    finally:
        reactor.unregister_timer(interaction)
        reactor.unregister_timer(watchdog)
    assert finished, "manual probe did not return after user input"
    assert not manual.status["is_active"]
    assert manual._helper is None
    assert not gcode.get_mutex().test()
    return errors


def test_probe_points_waits_at_each_point_and_then_resumes_the_macro(context):
    printer, config, manual, gcode, reactor = context
    printer.send_event("klippy:connect")
    results = []
    events = []
    accepted = []
    points = [(20, 20), (40, 20)]
    helper = ProbePointsHelper(
        config, lambda offsets, positions: results.append(positions[:]), points
    )
    gcode.register_command("MEASURE", helper.start_probe)
    gcode.register_command("AFTER", lambda command: events.append("after"))
    gcode.register_command(
        "MACRO",
        lambda command: command.run_script("MEASURE METHOD=manual\nAFTER"),
    )

    def accept():
        assert not events and not results
        assert not gcode.get_mutex().test()
        accepted.append(True)
        delta = 1.0 - printer.lookup_object("toolhead").get_position()[2]
        gcode.run_script("TESTZ Z=%s\nACCEPT" % delta)

    assert not run_interactive(context, "MACRO", accept)
    assert results == [[[20, 20, 1.0], [40, 20, 1.0]]]
    assert events == ["after"]
    assert len(accepted) == 2
    assert helper.retry_session._gcmd is None


def test_shutdown_wakes_manual_waiter_and_stops_followup_commands(context):
    printer, config, manual, gcode, reactor = context
    events = []
    gcode.register_command("AFTER", lambda command: events.append("after"))

    def abort():
        printer.send_event("klippy:shutdown")

    assert run_interactive(context, "MANUAL_PROBE\nAFTER", abort) == [
        "Manual probe aborted"
    ]
    assert not events
    assert (
        not {"ACCEPT", "NEXT", "ABORT", "TESTZ"}
        & gcode.ready_gcode_handlers.keys()
    )


def test_invalid_lock_context_cleans_up_the_manual_helper(context):
    printer, config, manual, gcode, reactor = context
    command = gcode.create_gcode_command("MANUAL_PROBE", "MANUAL_PROBE", {})
    with gcode.get_mutex():
        with pytest.raises(CommandError, match="dispatched handler"):
            manual.run_probe(command)
    assert manual._helper is None
    assert not manual.status["is_active"]
    assert (
        not {"ACCEPT", "NEXT", "ABORT", "TESTZ"}
        & gcode.ready_gcode_handlers.keys()
    )


@pytest.mark.parametrize("finish", [None, "ABORT", "ACCEPT"])
def test_new_manual_probe_rejected_until_previous_call_returns(
    context, monkeypatch, finish
):
    printer, config, manual, gcode, reactor = context
    printer.send_event("klippy:connect")

    events = []
    gcode.register_command("AFTER", lambda command: events.append("after"))

    gcode.register_command(
        "DIRECT_PROBE",
        ProbeList.get_list(printer).get_probe("manual").run_probe,
    )

    def attempt_new_probe():
        helper = manual._helper
        if finish is not None:
            gcode.run_script("TESTZ Z=-1\n" + finish)
        wrapped = ProbeList.get_list(printer).get_probe("manual")
        state = wrapped.retry_session._retry_state
        with monkeypatch.context() as patch:
            patch.setattr(
                manual_probe_module,
                "ManualProbeHelper",
                lambda *args, **kwargs: pytest.fail(
                    "constructed overlapping helper"
                ),
            )
            for script in ("DIRECT_PROBE", "PROBE_ACCURACY PROBE=manual"):
                with pytest.raises(
                    CommandError, match="Already in a manual Z probe"
                ):
                    gcode.run_script(script)
                assert wrapped.retry_session._retry_state is state
        assert manual._helper is helper

    errors = run_interactive(context, "MANUAL_PROBE\nAFTER", attempt_new_probe)
    assert errors == ([] if finish == "ACCEPT" else ["Manual probe aborted"])
    assert events == (["after"] if finish == "ACCEPT" else [])
    events.clear()
    assert not run_interactive(
        context, "PROBE\nAFTER", lambda: gcode.run_script("TESTZ Z=-1\nACCEPT")
    )

    assert events == ["after"]


@pytest.mark.parametrize("accept_first", [False, True])
def test_probe_points_cancel_stops_the_selected_probe(context, accept_first):
    printer, config, manual, gcode, reactor = context
    completion = reactor.completion()
    helper = ProbePointsHelper(
        config, default_points=[(20, 20)], completion=completion
    )
    gcode.register_command("MEASURE", helper.start_probe)

    def cancel():
        if accept_first:
            gcode.run_script("TESTZ Z=-1\nACCEPT")
        state = helper.retry_session._retry_state
        with pytest.raises(CommandError, match="Already in a manual Z probe"):
            gcode.run_script("MEASURE METHOD=manual")
        assert helper.retry_session._retry_state is state
        assert not completion.test()
        helper.cancel()
        assert not completion.test()
        assert helper.retry_session._gcmd is not None

    assert run_interactive(context, "MEASURE METHOD=manual", cancel) == [
        "Probing cancelled" if accept_first else "Manual probe aborted"
    ]
    assert completion.test() and completion.wait() is None
    assert helper.retry_session._gcmd is None


@pytest.mark.parametrize("kind", ["endstop", "eddy"])
def test_calibration_selects_manual_by_default_and_normalizes_reference(
    context, kind
):
    printer, config, manual, gcode, reactor = context
    probe_list = ProbeList.get_list(printer)
    hardware = SimpleNamespace(
        probe_name="hardware",
        is_default_probe=True,
        run_probe=lambda command: [10.0, 20.0, 2.5],
        get_offsets=lambda: (3.0, -2.0, 0.5),
    )
    probe_list.probes["hardware"] = hardware
    probe_list.default_probe = ("hardware", hardware)
    results = []
    if kind == "endstop":
        manual.z_position_endstop = 0.0
        printer.add_object(
            "configfile",
            SimpleNamespace(set=lambda *args: results.append(args)),
        )
        handler = manual.cmd_Z_ENDSTOP_CALIBRATE
    else:
        eddy = EddyCalibration.__new__(EddyCalibration)
        eddy.printer = printer
        eddy.post_manual_probe = results.append
        handler = eddy.cmd_EDDY_CALIBRATE
    gcode.register_command("CALIBRATE_TEST", handler)
    assert not run_interactive(
        context,
        "CALIBRATE_TEST",
        lambda: gcode.run_script("TESTZ Z=-4\nACCEPT"),
    )
    gcode.run_script("CALIBRATE_TEST PROBE=hardware")
    if kind == "endstop":
        assert results == [
            ("stepper_z", "position_endstop", "-1.000"),
            ("stepper_z", "position_endstop", "-2.000"),
        ]
    else:
        assert results == [[0.0, 0.0, 1.0], [13.0, 18.0, 2.0]]


def test_manual_accuracy_abort_and_invalid_xy_clear_retry_state(context):
    printer, _, manual, gcode, _ = context
    toolhead = printer.lookup_object("toolhead")
    wrapped = ProbeList.get_list(printer).get_probe("manual")
    assert run_interactive(
        context,
        "PROBE_ACCURACY PROBE=manual",
        lambda: gcode.run_script("ABORT"),
    ) == ["Manual probe aborted"]
    assert wrapped.retry_session._gcmd is None
    assert not wrapped.retry_session._point_lookup

    for axis in (0, 1):
        # Remain in the old retry cache's quantization bucket.
        requested = toolhead.get_position()
        requested[axis] += 0.01
        toolhead.manual_move(requested, 5.0)

        def move_then_accept():
            assert toolhead.get_position()[:2] == requested[:2]
            gcode.run_script("TESTZ Z=-1")
            changed = toolhead.get_position()
            changed[axis] += 0.01
            toolhead.manual_move(changed, 5.0)
            gcode.run_script("ACCEPT")

        assert run_interactive(context, "MANUAL_PROBE", move_then_accept) == [
            "Manual probe aborted"
        ]
        assert wrapped.retry_session._gcmd is None
        assert not wrapped.retry_session._point_lookup


def test_probe_calibration_uses_registered_reference_probe(context):
    printer, config, _, gcode, _ = context
    saved = []
    printer.add_object(
        "configfile", SimpleNamespace(set=lambda *args: saved.append(args))
    )
    probe_list = ProbeList.get_list(printer)
    toolhead = printer.lookup_object("toolhead")
    toolhead.position[:] = [100.0, 50.0, 10.0, 0.0]
    position = toolhead.position
    contacts = []
    target_offsets = (10.0, 5.0, 2.0)
    reference_offsets = (20.0, -3.0, 0.25)

    def run_probe(offsets):
        x, y = position[0] + offsets[0], position[1] + offsets[1]
        contacts.append((x, y))
        position[2] = 0.01 * x + 0.02 * y + offsets[2]
        return position[:3]

    reference = SimpleNamespace(
        probe_name="touch_reference",
        is_default_probe=False,
        get_offsets=lambda: reference_offsets,
        run_probe=lambda command: run_probe(reference_offsets),
    )
    probe_list.add_probe_object(reference, config)
    target = PrinterProbe.__new__(PrinterProbe)
    target.printer, target.gcode, target.name = printer, gcode, "probe target"
    target.lift_speed = target.speed = 5.0
    target.x_offset, target.y_offset, target.z_offset = target_offsets
    target.run_probe = lambda command: run_probe(target_offsets)
    target._move = toolhead.manual_move

    gcode.register_command("CALIBRATE_TEST", target.cmd_PROBE_CALIBRATE)
    gcode.run_script("CALIBRATE_TEST REFERENCE_PROBE=touch_reference")
    assert contacts == [(110.0, 55.0), (110.0, 55.0)]
    assert saved == [("probe target", "z_offset", "2.000")]

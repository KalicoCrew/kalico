"""Probe sessions publish results after cleanup and complete on cancellation."""

import configparser
from types import SimpleNamespace

import pytest
import test_parent_command as commands

from klippy.configfile import ConfigWrapper
from klippy.extras import manual_probe, probe
from klippy.gcode import CommandError

command_context = commands.context


@pytest.fixture
def probing(command_context):
    printer, gcode, objects = command_context
    printer.config_error = configparser.Error
    printer.command_error = CommandError
    printer.add_object = objects.__setitem__
    printer.load_object = lambda config, name: objects[name]
    objects["gcode_move"] = SimpleNamespace()
    objects["pins"] = SimpleNamespace(register_chip=lambda *args: None)
    objects["gcode_macro"] = SimpleNamespace(load_template=lambda *args: None)
    position = [0.0, 0.0, 5.0, 0.0]
    clock = [0.0]
    captured = []

    def move(coords, speed):
        for index, value in enumerate(coords):
            if value is not None:
                position[index] = value
        clock[0] += 1.0

    def capture(callback):
        captured.append(clock[0])
        callback(clock[0])

    objects["toolhead"] = SimpleNamespace(
        manual_move=move,
        get_status=lambda time: {"homed_axes": "xyz"},
        get_position=lambda: position[:],
        get_last_move_time=lambda: clock[0],
        get_max_velocity=lambda: (300.0, 3000.0),
        register_lookahead_callback=capture,
        wait_moves=lambda: None,
        dwell=lambda duration: None,
    )
    parser = configparser.RawConfigParser()
    parser.read_dict(
        {
            "test_probe": {},
            "manual_probe": {},
            "printer": {"kinematics": "cartesian"},
        }
    )
    config = ConfigWrapper(printer, parser, {}, "test_probe")
    objects["manual_probe"] = manual_probe.ManualProbe(
        config.getsection("manual_probe")
    )
    sessions = []
    hardware = SimpleNamespace(
        probe_name="hardware",
        speed=5.0,
        get_lift_speed=lambda command: 5.0,
        get_offsets=lambda: (0.0, 0.0, 0.0),
        multi_probe_begin=lambda context: sessions.append("begin"),
        multi_probe_end=lambda command: sessions.append("end"),
        run_probe=lambda command, retry: position[:2] + [0.25],
    )
    hardware.mcu_probe = SimpleNamespace(
        begin_collect_distance=lambda: SimpleNamespace(
            sample_rate=lambda: 1000,
            finish=lambda time: [
                SimpleNamespace(time=t, distance=4.75) for t in captured
            ],
        )
    )
    plist = probe.ProbeList.get_list(printer)
    plist.probes["hardware"] = hardware
    plist.default_probe = ("hardware", hardware)
    return config, hardware, sessions


@pytest.mark.parametrize(
    "method, with_callback",
    [("automatic", False), ("automatic", True), ("rapid_scan", True)],
)
def test_result_completion_preserves_callback_retry_and_snapshot(
    probing, method, with_callback
):
    config, hardware, sessions = probing
    printer = config.get_printer()
    completion = printer.get_reactor().completion()
    calls = []

    def end(command):
        assert not completion.test()
        sessions.append("end")

    hardware.multi_probe_end = end

    def finalize(offsets, positions):
        assert not completion.test()
        calls.append(positions[:])
        if len(calls) == 1:
            return "retry"
        positions.pop()

    helper = probe.ProbePointsHelper(
        config,
        finalize if with_callback else None,
        default_points=[(20, 20)],
        completion=completion,
    )
    gcode = printer.lookup_object("gcode")
    gcode.register_command("MEASURE", helper.start_probe)
    gcode.run_script("MEASURE METHOD=" + method)
    offsets, positions = completion.wait()
    assert offsets == (0.0, 0.0, 0.0)
    assert [list(position) for position in positions] == [[20, 20, 0.25]]
    assert sessions == ["begin", "end"]
    assert len(calls) == (2 if with_callback else 0)
    assert helper.retry_session._gcmd is None


@pytest.mark.parametrize(
    "script, message, expected_sessions",
    [
        ("MEASURE", "probe failed", ["begin", "end"]),
        ("MEASURE CANCEL_ON_END=1", "Probing cancelled", ["begin", "end"]),
        (
            "MEASURE METHOD=rapid_scan CANCEL_ON_END=1",
            "Probing cancelled",
            ["begin", "end"],
        ),
        ("MEASURE PROBE=missing", "Unknown requested", []),
        (
            "MEASURE METHOD=rapid_scan HORIZONTAL_MOVE_Z=-1",
            "horizontal_move_z",
            [],
        ),
    ],
)
def test_failed_probe_completes_after_session_cleanup(
    probing, script, message, expected_sessions
):
    config, hardware, sessions = probing
    printer = config.get_printer()
    completion = printer.get_reactor().completion()

    def fail(command, retry):
        if command.get_int("CANCEL_ON_END", 0):
            return [20, 20, 0.25]
        raise CommandError("probe failed")

    def end(context):
        sessions.append("end")
        if context.gcmd.get_int("CANCEL_ON_END", 0):
            helper.cancel()
        assert not completion.test()

    hardware.multi_probe_end = end

    hardware.run_probe = fail
    helper = probe.ProbePointsHelper(
        config, default_points=[(20, 20)], completion=completion
    )
    gcode = printer.lookup_object("gcode")
    gcode.register_command("MEASURE", helper.start_probe)
    with pytest.raises(CommandError, match=message):
        gcode.run_script(script)
    assert completion.test() and completion.wait() is None
    assert sessions == expected_sessions
    assert helper.retry_session._gcmd is None

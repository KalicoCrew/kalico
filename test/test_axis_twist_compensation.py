"""Exercise compensation using real config parsing, ZMesh, and probe helpers."""

import configparser
import socket
from contextlib import nullcontext
from types import SimpleNamespace

import pytest
import test_manual_probe as manual_tests

from klippy.configfile import ConfigWrapper
from klippy.extras import axis_twist_compensation as twist
from klippy.extras import gcode_macro, manual_probe, probe
from klippy.gcode import (
    CommandError,
    GCodeDispatch,
    GCodeIO,
)
from klippy.reactor import Reactor, ReactorCompletion

manual_context = manual_tests.context


class TestReactor:
    __test__ = False

    def monotonic(self):
        return 0.0

    def completion(self):
        return ReactorCompletion(self)


class GCode:
    def __init__(self):
        self.messages = []
        self.commands = {}
        self.mux_commands = {}
        self.mutex = None
        self._interrupt_token = None

    def get_interrupt_token(self):
        if self._interrupt_token is None:
            self._interrupt_token = self.printer.get_reactor().completion()
        return self._interrupt_token

    def get_mutex(self):
        return self.mutex if self.mutex is not None else nullcontext()

    def respond_info(self, message):
        self.messages.append(message)

    respond_raw = respond_info
    error = CommandError

    def register_command(self, name, handler, **kwargs):
        previous = self.commands.pop(name, None)
        if handler is not None:
            if previous is not None:
                raise configparser.Error("Command already registered")
            self.commands[name] = handler
        return previous

    def register_mux_command(self, name, key, value, handler, **kwargs):
        if name not in self.mux_commands:
            self.mux_commands[name] = {}
            self.register_command(
                name,
                lambda command: self.mux_commands[name][command.get(key, None)](
                    command
                ),
            )
        self.mux_commands[name][value] = handler

    def create_gcode_command(self, command, commandline, params, **kwargs):
        return GCodeDispatch.create_gcode_command(
            self, command, commandline, params, **kwargs
        )


class Toolhead:
    def __init__(self):
        self.position = [0.0, 0.0, 5.0, 0.0]
        self.time = 0.0
        self.captured = []

    def manual_move(self, coords, speed):
        for i, value in enumerate(coords):
            if value is not None:
                self.position[i] = value
        self.time += 1.0

    def get_status(self, eventtime):
        return {"homed_axes": "xyz"}

    def get_position(self):
        return list(self.position)

    def flush_step_generation(self):
        pass

    def get_kinematics(self):
        return SimpleNamespace(
            get_steppers=lambda: [],
            calc_position=lambda positions: self.get_position(),
            get_status=lambda eventtime: {
                "axis_minimum": (-100.0, -50.0, 0.0),
                "axis_maximum": (200.0, 250.0, 200.0),
            },
        )

    def get_last_move_time(self):
        return self.time

    def get_max_velocity(self):
        return 300.0, 3000.0

    def register_lookahead_callback(self, callback):
        self.captured.append(self.time)
        callback(self.time)

    def wait_moves(self):
        pass

    def dwell(self, duration):
        self.time += duration


class Printer:
    config_error = configparser.Error
    command_error = CommandError

    def __init__(self):
        self.events = {}
        self.reactor = TestReactor()
        self.saved = {}
        self.objects = {
            "gcode": GCode(),
            "pins": SimpleNamespace(register_chip=lambda *args: None),
            "toolhead": Toolhead(),
            "gcode_move": SimpleNamespace(),
            "configfile": SimpleNamespace(set=self.save),
            "gcode_macro": SimpleNamespace(load_template=lambda *args: None),
        }
        self.objects["gcode"].printer = self
        self.sessions = []

    def get_start_args(self):
        return {}

    def is_shutdown(self):
        return False

    def get_reactor(self):
        return self.reactor

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def add_object(self, name, value):
        self.objects[name] = value

    def register_event_handler(self, event, callback):
        self.events.setdefault(event, []).append(callback)

    def send_event(self, event):
        for callback in self.events.get(event, []):
            callback()

    def load_object(self, config, name):
        if name not in self.objects:
            assert name == "axis_twist_compensation"
            self.objects[name] = twist.load_config(config.getsection(name))
        return self.objects[name]

    def save(self, section, key, value):
        self.saved.setdefault(section, {})[key] = str(value)

    def command(self, **params):
        return self.objects["gcode"].create_gcode_command(
            "AXIS_TWIST_COMPENSATION_CALIBRATE", "calibrate", params
        )


class Probe:
    def __init__(self, printer, name, offsets, bias=lambda x, y: 0.0):
        self.printer = printer
        self.gcode = printer.objects["gcode"]
        self.probe_name = name
        self.offsets = offsets
        self.bias = bias
        self.speed = 5.0
        self.fail = False
        self.in_session = False
        self.measured = []

    def get_offsets(self):
        return self.offsets

    def get_lift_speed(self, gcmd=None):
        return 5.0

    def multi_probe_begin(self, context):
        assert not self.in_session
        self.in_session = True
        self.printer.sessions.append((self.probe_name, "begin"))

    def multi_probe_end(self, gcmd=None):
        if self.in_session:
            self.printer.sessions.append((self.probe_name, "end"))
        self.in_session = False

    def run_probe(self, gcmd, retry_session):
        if self.fail:
            raise CommandError("Probe failed")
        pos = self.printer.objects["toolhead"].get_position()
        x, y = pos[0] + self.offsets[0], pos[1] + self.offsets[1]
        self.measured.append((x, y))
        z = bed(x, y) + self.offsets[2] + self.bias(x, y)
        z += self.printer.objects[
            "axis_twist_compensation"
        ].get_z_compensation_value(pos, self)
        return [pos[0], pos[1], z]


def bed(x, y):
    return 0.001 * x + 0.002 * y


def config(printer, options=None, name="axis_twist_compensation target"):
    parser = configparser.RawConfigParser()
    parser.read_dict(
        {
            name: {
                "mesh_min": "20, 20",
                "mesh_max": "100, 100",
                **(options or {}),
            }
        }
    )
    return ConfigWrapper(printer, parser, {}, name)


def _make_probe_setup():
    printer = Printer()
    probes = probe.ProbeList.get_list(printer)
    target = Probe(
        printer, "target", (7.0, -3.0, 1.5), lambda x, y: 0.0001 * x * y + 2.0
    )
    reference = Probe(printer, "reference", (-5.0, 4.0, 0.8))
    probes.probes.update(target=target, reference=reference)
    probes.default_probe = ("target", target)

    parser = configparser.RawConfigParser()
    parser.read_dict(
        {"manual_probe": {}, "printer": {"kinematics": "cartesian"}}
    )
    printer.objects["manual_probe"] = manual_probe.ManualProbe(
        ConfigWrapper(printer, parser, {}, "manual_probe")
    )

    return printer, target, reference


@pytest.fixture
def setup():
    return _make_probe_setup()


def load(printer, options=None, name="axis_twist_compensation target"):
    compensation = twist.load_config_prefix(config(printer, options, name))
    compensation.handle_connect()
    return compensation


def constant_map(compensation, value):
    params = compensation.calibration.layout.params
    compensation.mesh = twist.CompensationMesh.build(
        params,
        [[value] * params["x_count"] for _ in range(params["y_count"])],
        compensation.name,
        CommandError,
    )
    return compensation.mesh


def test_automatic_two_pass_calibration_and_reference_map(setup):
    printer, target, reference = setup
    compensation = load(printer, {"reference_probe": "reference"})
    old = constant_map(compensation, 99.0)
    ref = load(printer, name="axis_twist_compensation reference")
    # A reference correction with spatial variation must affect the result.
    ref.mesh = twist.CompensationMesh.build(
        ref.calibration.layout.params,
        [[-0.2, 0.0, 0.2]] * 3,
        ref.name,
        CommandError,
    )
    bed_mesh = object()
    printer.objects["bed_mesh"] = bed_mesh
    compensation.manager.cmd_CALIBRATE(printer.command(PROBE="target"))
    assert compensation.mesh is not old
    assert not compensation.calibration.is_calibrating()
    assert (
        target.measured
        == reference.measured
        == compensation.calibration.layout.points
    )
    assert printer.sessions == [
        ("target", "begin"),
        ("target", "end"),
        ("reference", "begin"),
        ("reference", "end"),
    ]
    assert printer.objects["bed_mesh"] is bed_mesh
    assert compensation.mesh.calc_z(20, 20) == pytest.approx(0.12)
    assert compensation.mesh.calc_z(100, 100) == pytest.approx(-0.44)
    assert sum(map(sum, compensation.mesh.probed_matrix)) == pytest.approx(0.0)
    assert list(printer.saved) == [compensation.name]


def test_probe_failure_releases_session_and_preserves_map(setup):
    printer, target, reference = setup
    compensation = load(printer, {"reference_probe": "reference"})
    old = constant_map(compensation, 0.5)
    for failing_probe in (target, reference):
        failing_probe.fail = True
        with pytest.raises(CommandError, match="Probe failed"):
            compensation.manager.cmd_CALIBRATE(printer.command(PROBE="target"))
        assert not compensation.calibration.is_calibrating()
        assert compensation.mesh is old
        assert printer.saved == {}
        assert not target.in_session and not reference.in_session

        failing_probe.fail = False


def test_probe_dispatch_offsets_and_boundary_clamping(setup):
    printer, target, reference = setup
    compensation = load(printer)
    layout = compensation.calibration.layout
    compensation.mesh = layout.build_difference(
        [0] * 9, [x * y * 0.001 for x, y in layout.points], "test", CommandError
    )
    manager = compensation.manager
    assert manager.get_z_compensation_value((13, 23), target) == pytest.approx(
        -3.2
    )
    assert manager.get_z_compensation_value((13, 23)) == pytest.approx(-3.2)
    assert manager.get_z_compensation_value(
        (-1000, -1000), target
    ) == pytest.approx(-3.2)
    assert manager.get_z_compensation_value(
        (1000, 1000), target
    ) == pytest.approx(6.4)
    assert manager.get_z_compensation_value((50, 50), reference) == 0.0

    # Exercise the ordinary probe caller against the same spatial map.
    target.probing_move = lambda speed, gcmd: ([13, 23, 2.0], True)
    reference.probing_move = target.probing_move
    printer.objects["toolhead"].position = [13, 23, 5, 0]
    pos, _ = probe.PrinterProbe._probe(target, 5, printer.command())
    assert pos[2] == pytest.approx(-1.2)
    pos, _ = probe.PrinterProbe._probe(reference, 5, printer.command())
    assert pos[2] == 2.0

    printer.objects["probe_list"].default_probe = None
    assert manager.get_z_compensation_value((50, 50)) == 0.0


def test_rapid_scan_uses_probe_identity_and_physical_coordinates(setup):
    printer, target, reference = setup
    compensation = load(printer)
    layout = compensation.calibration.layout
    compensation.mesh = layout.build_difference(
        [0] * 9, [x * y * 0.001 for x, y in layout.points], "test", CommandError
    )
    toolhead = printer.objects["toolhead"]
    session = SimpleNamespace(
        sample_rate=lambda: 1000,
        finish=lambda time: [
            SimpleNamespace(time=t, distance=1.0) for t in toolhead.captured
        ],
    )
    target.mcu_probe = SimpleNamespace(begin_collect_distance=lambda: session)
    reference.mcu_probe = target.mcu_probe
    for selected, expected in [(target, -3.2), (reference, 0.0)]:
        toolhead.captured.clear()
        results = []
        helper = probe.ProbePointsHelper(
            config(printer),
            lambda offsets, positions: results.extend(positions),
            default_points=[(20, 20)],
            use_offsets=True,
        )
        helper.start_probe(
            printer.command(PROBE=selected.probe_name, METHOD="rapid_scan")
        )
        assert len(results) == 1
        assert results[0][2] == pytest.approx(
            5 - 1 + selected.offsets[2] + expected
        )


def test_invalid_config(setup):
    printer, _, _ = setup
    for options, message in [
        ({"mesh_min": "100, 100"}, "bounds"),
        ({"mesh_radius": "40", "round_probe_count": "4"}, "odd"),
        ({"points": "1, 2, 3", "version": "999"}, "version"),
    ]:
        with pytest.raises(configparser.Error, match=message):
            load(printer, options)


def test_bad_saved_points_and_incomplete_pass_are_rejected(setup):
    printer, _, _ = setup
    compensation = load(printer)
    old = constant_map(compensation, 0.5)
    compensation.save(old, "manual", compensation.calibration.layout)
    saved = printer.saved[compensation.name]
    for points in ["1, 2, 3", "1, 2, nan\n1, 2, 3\n1, 2, 3"]:
        with pytest.raises(
            configparser.Error, match="Invalid compensation mesh points"
        ):
            load(printer, {**saved, "points": points})
    with pytest.raises(CommandError, match="Incomplete"):
        compensation.calibration.layout.build_difference(
            [], [], "test", CommandError
        )


def test_unknown_target_and_invalid_command_options(setup):
    printer, _, _ = setup
    compensation = load(printer)
    for params, message in [
        ({"PROBE": "missing"}, "Unknown requested"),
        ({"REFERENCE_PROBE": "missing"}, "Unknown requested"),
        ({"REFERENCE_PROBE": "target"}, "itself"),
        ({"AXIS": "X"}, "AXIS"),
        ({"SAMPLE_COUNT": "1"}, "SAMPLE_COUNT"),
    ]:
        with pytest.raises(CommandError, match=message):
            compensation.manager.cmd_CALIBRATE(printer.command(**params))
    assert printer.sessions == []


@pytest.mark.parametrize(
    "options",
    [
        {},
        {"mesh_radius": "40", "mesh_origin": "75, 85"},
        {"algorithm": "bicubic", "probe_count": "5, 5"},
    ],
)
def test_layout_and_interpolation_match_existing_bed_mesh(setup, options):
    printer, _, _ = setup
    compensation = load(printer, options)
    layout = compensation.calibration.layout
    meshes = []
    bedmesh = SimpleNamespace(
        set_mesh=meshes.append, save_profile=lambda name: None
    )
    bed_calibration = twist.bed_mesh.BedMeshCalibrate(
        config(printer, options, "bed_mesh"), bedmesh
    )
    assert layout.points == bed_calibration.points
    differences = [x * y * 0.001 for x, y in layout.points]
    mean = sum(differences) / len(differences)
    bed_calibration.probe_finalize(
        (0, 0, 0),
        [
            [x, y, difference - mean]
            for (x, y), difference in zip(layout.points, differences)
        ],
    )
    mesh = layout.build_difference(
        [0] * len(differences), differences, "test", CommandError
    )
    compensation.save(mesh, "manual", layout)
    reloaded = twist.load_config_prefix(
        legacy_config(
            printer, printer.saved[compensation.name], compensation.name
        )
    )
    reloaded.handle_connect()
    assert reloaded.calibration.layout.points == layout.points
    for x, y in [(20, 20), (40, 45), (70, 80), (110, 120)]:
        assert mesh.calc_z(x, y) == pytest.approx(meshes[0].calc_z(x, y))
        assert reloaded.mesh.calc_z(x, y) == pytest.approx(
            mesh.calc_z(x, y), abs=1e-6
        )


LEGACY_CALIBRATION = {
    "calibrate_start_x": "20",
    "calibrate_end_x": "100",
    "calibrate_y": "40",
    "calibrate_start_y": "10",
    "calibrate_end_y": "90",
    "calibrate_x": "60",
}


def legacy_config(printer, options, name="axis_twist_compensation"):
    parser = configparser.RawConfigParser()
    parser.read_dict({name: options})
    return ConfigWrapper(printer, parser, {}, name)


def load_legacy(printer, options):
    manager = twist.load_config(legacy_config(printer, options))
    printer.objects["axis_twist_compensation"] = manager
    manager._handle_connect()
    return manager


def legacy_interpolate(value, start, end, samples):
    t = (value - start) / (end - start) * (len(samples) - 1)
    index = max(0, min(int(t // 1), len(samples) - 2))
    return samples[index] + (t - index) * (samples[index + 1] - samples[index])


@pytest.mark.parametrize("axes", ["X", "XY"])
def test_legacy_import_covers_work_area_with_standard_mesh(setup, axes):
    printer, target, reference = setup
    options = dict(LEGACY_CALIBRATION)
    if "X" in axes:
        options.update(
            z_compensations="0.4, -0.1, 0.2",
            compensation_start_x="100",
            compensation_end_x="20",
        )
    if "Y" in axes:
        options.update(
            zy_compensations="0.05, -0.03, 0.08",
            compensation_start_y="10",
            compensation_end_y="90",
        )
    manager = load_legacy(printer, options)
    imported = manager.get_compensation(target).mesh
    assert type(imported) is twist.bed_mesh.ZMesh
    params = imported.get_mesh_params()
    assert params["min_x"] <= -100 + target.offsets[0]
    assert params["max_x"] >= 200 + target.offsets[0]
    assert params["min_y"] <= -50 + target.offsets[1]
    assert params["max_y"] >= 250 + target.offsets[1]
    for x, y in [
        (-100, -50),
        (20, 10),
        (35, 35),
        (60, 50),
        (90, 75),
        (100, 90),
        (200, 250),
    ]:
        expected = 0.0
        if "X" in axes:
            expected += legacy_interpolate(x, 20, 100, [0.2, -0.1, 0.4])
        if "Y" in axes:
            expected += legacy_interpolate(y, 10, 90, [0.05, -0.03, 0.08])
        assert manager.get_z_compensation_value(
            (x, y), target
        ) == pytest.approx(expected)
        assert manager.get_z_compensation_value((x, y)) == pytest.approx(
            expected
        )
        assert manager.get_z_compensation_value((x, y), reference) == 0.0
    same_name = Probe(printer, "target", target.offsets)
    assert manager.get_z_compensation_value((60, 50), same_name) == 0.0
    assert printer.saved == {}
    # Legacy calibration settings never activate line calibration.
    for command in ({"AXIS": "X"}, {"SAMPLE_COUNT": "3"}):
        with pytest.raises(CommandError, match="no longer supported"):
            manager.cmd_CALIBRATE(printer.command(**command))
    assert not printer.sessions


def test_saved_mesh_takes_precedence_over_legacy_data(setup):
    printer, target, reference = setup
    manager = twist.load_config(
        legacy_config(
            printer, {**LEGACY_CALIBRATION, "z_compensations": "invalid"}
        )
    )
    printer.objects["axis_twist_compensation"] = manager
    compensation = load(printer)
    old = constant_map(compensation, 0.75)
    manager._handle_connect()
    assert manager.get_compensation(target).mesh is old
    assert manager.get_z_compensation_value((20, 20), target) == pytest.approx(
        0.75
    )
    assert manager.get_z_compensation_value((20, 20), reference) == 0.0


def test_invalid_legacy_curve_configuration():
    for options, message in [
        ({"z_compensations": "0.1, 0.2"}, "requires compensation_start_x"),
        (
            {
                "z_compensations": "0.1",
                "compensation_start_x": "0",
                "compensation_end_x": "100",
            },
            "Invalid z_compensations",
        ),
        (
            {
                "z_compensations": "0.1, 0.2",
                "compensation_start_x": "0",
                "compensation_end_x": "0",
            },
            "Invalid z_compensations",
        ),
    ]:
        printer, _, _ = _make_probe_setup()
        with pytest.raises(configparser.Error, match=message):
            load_legacy(printer, {**LEGACY_CALIBRATION, **options})


def test_command_probe_selection_and_self_contained_saved_config(setup):
    printer, target, reference = setup
    alternate = Probe(
        printer, "alternate", (3.0, -2.0, 0.25), lambda x, y: 0.001 * x
    )
    probe_list = probe.ProbeList.get_list(printer)
    probe_list.probes["alternate"] = alternate
    command = printer.command(
        PROBE="target", REFERENCE_PROBE="alternate", METHOD="manual"
    )
    assert (
        probe_list.get_command_probe(
            command,
            default_probe_name="reference",
            param_name="REFERENCE_PROBE",
        )
        is alternate
    )
    manager = twist.load_config(
        config(
            printer,
            {
                "reference_probe": "reference",
                "speed": "32",
                "horizontal_move_z": "7",
            },
            "axis_twist_compensation",
        )
    )
    printer.objects["axis_twist_compensation"] = manager
    manager.cmd_CALIBRATE(
        printer.command(PROBE="target", REFERENCE_PROBE="alternate")
    )
    compensation = manager.get_compensation(target)
    assert printer.lookup_object(compensation.name) is compensation
    assert len(target.measured) == len(alternate.measured) == 9
    assert not reference.measured
    saved = printer.saved[compensation.name].copy()
    assert compensation.name == "axis_twist_compensation target"
    assert saved["reference_probe"] == "alternate"
    assert saved["speed"] == "32" and saved["horizontal_move_z"] == "7"
    assert (
        "axis_twist_compensation",
        "mesh_min",
    ) in manager.config.access_tracking
    assert (
        "mesh_min" in saved and "mesh_max" in saved and "probe_count" in saved
    )
    restarted, _, _ = _make_probe_setup()
    probe.ProbeList.get_list(restarted).probes["alternate"] = Probe(
        restarted, "alternate", alternate.offsets
    )
    reloaded = twist.load_config_prefix(
        legacy_config(restarted, saved, compensation.name)
    )
    reloaded.handle_connect()
    assert reloaded.calibration.helper.speed == 32
    assert reloaded.calibration.helper.horizontal_move_z == 7
    assert (
        reloaded.mesh.get_mesh_params() == compensation.mesh.get_mesh_params()
    )
    assert reloaded.mesh.calc_z(65, 75) == pytest.approx(
        compensation.mesh.calc_z(65, 75), abs=1e-6
    )
    # The stored mesh remains independent of the next calibration's layout.
    changed_layout = twist.load_config_prefix(
        legacy_config(
            restarted,
            {
                **saved,
                "mesh_min": "30, 30",
                "mesh_max": "130, 130",
                "probe_count": "4, 4",
            },
            compensation.name,
        )
    )
    assert (
        changed_layout.mesh.get_mesh_params()
        == compensation.mesh.get_mesh_params()
    )
    assert changed_layout.calibration.layout.params["x_count"] == 4
    assert changed_layout.mesh.calc_z(65, 75) == pytest.approx(
        compensation.mesh.calc_z(65, 75), abs=1e-6
    )
    # A command override does not replace the configured default in this session.
    manager.cmd_CALIBRATE(printer.command(PROBE="target"))
    assert len(reference.measured) == 9
    assert len(alternate.measured) == 9
    assert printer.saved[compensation.name]["reference_probe"] == "reference"
    # Named settings override inherited base settings, including reference choice.
    other = twist.ProbeCompensation(
        legacy_config(
            printer,
            {"probe_count": "4, 4", "reference_probe": "target"},
            "axis_twist_compensation alternate",
        ),
        manager,
    )
    other.handle_connect()
    assert other.calibration.layout.params["x_count"] == 4
    manager.cmd_CALIBRATE(printer.command(PROBE="alternate"))
    assert printer.saved[other.name]["reference_probe"] == "target"


@pytest.mark.parametrize(
    "macro_kind, source, queued_input, outcome",
    [
        ("gcode", "serial", True, "success"),
        ("python", "api", False, "abort"),
        ("gcode", "api", False, "error"),
    ],
)
def test_nested_macro_waits_for_manual_calibration_and_then_continues(
    setup, monkeypatch, source, queued_input, outcome, macro_kind
):
    printer, target, reference = setup
    reactor = Reactor()
    printer.reactor = reactor
    server, client = socket.socketpair()
    printer.get_start_args = lambda: {"gcode_fd": server.fileno()}
    shutdowns = []
    printer.invoke_shutdown = shutdowns.append
    dispatch = GCodeDispatch(printer)
    printer.objects["gcode"] = dispatch
    target.gcode = reference.gcode = dispatch
    messages = []
    dispatch.register_output_handler(messages.append)
    macro_config = legacy_config(printer, {}, "gcode_macro")
    printer.objects["gcode_macro"] = gcode_macro.PrinterGCodeMacro(macro_config)
    compensation = load(printer)
    previous_mesh = constant_map(compensation, 0.5)
    io = GCodeIO(printer)
    printer.objects["gcode_io"] = io
    events = []
    sent = set()
    completed_events = ["before", "calibrated", "after", "batch_after"]
    failed_events = (
        ["before", "batch_after"] if source == "serial" else ["before"]
    )
    if queued_input:
        completed_events.append("queued_after")
        failed_events.append("queued_after")
    if source == "serial":
        completed_events.append("stream_after")
        failed_events.append("stream_after")

    def mark(gcmd):
        value = gcmd.get("VALUE")
        assert dispatch.get_mutex().test()
        if value == "calibrated":
            assert compensation.mesh is not None
            assert gcmd._context.is_nested()
        events.append(value)

    dispatch.register_command("MARK_EVENT", mark)

    def check_context(gcmd):
        assert not gcmd._context.is_nested()

    dispatch.register_command("CHECK_CONTEXT", check_context)

    def macro_script(script):
        return "!emit(%r)" % script if macro_kind == "python" else script

    inner = gcode_macro.GCodeMacro(
        legacy_config(
            printer,
            {
                "gcode": macro_script(
                    "AXIS_TWIST_COMPENSATION_CALIBRATE PROBE=target\n"
                    "MARK_EVENT VALUE=calibrated\nRETURN\nMARK_EVENT VALUE=unreachable"
                )
            },
            "gcode_macro INNER",
        )
    )
    outer = gcode_macro.GCodeMacro(
        legacy_config(
            printer,
            {
                "gcode": macro_script(
                    "MARK_EVENT VALUE=before\nINNER\nMARK_EVENT VALUE=after"
                )
            },
            "gcode_macro OUTER",
        )
    )
    printer.objects.update(
        {"gcode_macro INNER": inner, "gcode_macro OUTER": outer}
    )
    dispatch._handle_ready()
    io._handle_ready()

    if queued_input:
        run_probe = target.run_probe

        def queue_before_wait(gcmd, retry_session):
            result = run_probe(gcmd, retry_session)
            if len(target.measured) == len(
                compensation.calibration.layout.points
            ):
                # Input accumulated during probing and disabled further reads.
                io.pending_commands.extend(
                    ["MARK_EVENT VALUE=queued_after"] + [""] * 20
                )
                reactor.unregister_fd(io.fd_handle)
                io.fd_handle = None
            return result

        monkeypatch.setattr(target, "run_probe", queue_before_wait)

    def user_input(eventtime):
        if events == completed_events:
            reactor.end()
            return reactor.NEVER
        if (
            outcome != "success"
            and any(message.startswith("!!") for message in messages)
            and not compensation.calibration.is_calibrating()
            and events == failed_events
        ):
            reactor.end()
            return reactor.NEVER
        if printer.objects["manual_probe"].status["is_active"]:
            assert events == ["before"]
            if not sent:
                with pytest.raises(CommandError, match="already in progress"):
                    compensation.manager.cmd_CALIBRATE(printer.command())
            assert inner.in_script and outer.in_script
            if source == "serial":
                assert io.is_processing_data
                if queued_input:
                    assert (
                        "MARK_EVENT VALUE=queued_after" in io.pending_commands
                    )
                    assert io.fd_handle is None
            if source == "serial" and not sent:
                client.sendall(b"MARK_EVENT VALUE=stream_after\n")
            x, y, z = printer.objects["toolhead"].get_position()[:3]
            if (x, y) not in sent:
                sent.add((x, y))
                if outcome == "abort":
                    interactive_script = "ABORT"
                elif outcome == "error":
                    interactive_script = "TESTZ Z=invalid"
                else:
                    interactive_script = "TESTZ Z=%.6f\nACCEPT" % (
                        bed(x, y) - z
                    )
                # Interactive API commands are independent of the waiting stream.
                try:
                    dispatch.run_script("CHECK_CONTEXT\n" + interactive_script)
                except CommandError:
                    assert outcome == "error"
        return eventtime + 0.001

    script = "OUTER\nMARK_EVENT VALUE=batch_after\n"
    if source == "serial":
        client.sendall(script.encode())
    else:

        def run_api(eventtime):
            try:
                dispatch.run_script(script)
            except CommandError:
                assert outcome != "success"

        reactor.register_callback(run_api)
    reactor.register_timer(user_input, reactor.NOW)
    reactor.register_callback(
        lambda eventtime: reactor.end(), reactor.monotonic() + 1.0
    )
    try:
        reactor.run()
    finally:
        reactor.finalize()
        server.close()
        client.close()
    if outcome == "success":
        assert events == completed_events
        assert len(sent) == 9
        assert not any(message.startswith("!!") for message in messages)
        assert compensation.mesh.calc_z(20, 20) == pytest.approx(0.32)
    else:
        assert events == failed_events
        assert any(message.startswith("!!") for message in messages)
        assert compensation.mesh is previous_mesh
        assert not printer.saved
    assert not inner.in_script and not outer.in_script
    assert not dispatch.get_mutex().test()
    assert not io.is_processing_data
    assert not shutdowns


def test_manual_wrapper_applies_its_map_exactly_once(manual_context):
    printer, _, manual, gcode, _ = manual_context
    probe_list = probe.ProbeList.get_list(printer)
    probe_list.probes["reference"] = SimpleNamespace(probe_name="reference")
    manager = twist.AxisTwistCompensation(legacy_config(printer, {}))
    printer.add_object("axis_twist_compensation", manager)
    compensation = twist.ProbeCompensation(
        config(
            printer,
            {"reference_probe": "reference"},
            "axis_twist_compensation manual",
        ),
        manager,
    )
    compensation.handle_connect()
    constant_map(compensation, 0.25)
    results = []
    gcode.register_command(
        "MEASURE", lambda command: results.append(manual.run_probe(command))
    )
    assert not manual_tests.run_interactive(
        manual_context,
        "MEASURE",
        lambda: gcode.run_script("TESTZ Z=-4\nACCEPT"),
    )
    assert results == [[0.0, 0.0, 1.25]]
    assert printer.lookup_object("toolhead").get_position()[2] == 1.0

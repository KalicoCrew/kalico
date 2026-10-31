"""Nested scripts preserve command context or use the legacy movement API."""

from collections import deque
from types import SimpleNamespace

import pytest

from klippy.extras import (
    bed_tilt,
    dockable_probe,
    gcode_macro,
    gcode_move,
    homing,
    probe,
    smart_effector,
    trad_rack,
    tuning_tower,
)
from klippy.gcode import GCodeDispatch
from klippy.reactor import Reactor


@pytest.fixture
def context():
    reactor = Reactor()
    objects = {
        "manual_probe": SimpleNamespace(verify_no_manual_probe=lambda: None)
    }
    handlers = {}

    def send_event(event, *args):
        return [callback(*args) for callback in handlers.get(event, [])]

    printer = SimpleNamespace(
        get_start_args=lambda: {},
        get_reactor=lambda: reactor,
        lookup_object=lambda name, default=None: objects.get(name, default),
        register_event_handler=lambda event, callback: handlers.setdefault(
            event, []
        ).append(callback),
        send_event=send_event,
    )
    gcode = GCodeDispatch(printer)
    objects["gcode"] = gcode
    gcode._handle_ready()
    try:
        yield printer, gcode, objects
    finally:
        reactor.finalize()


def test_tuning_tower_uses_legacy_scripts_without_changing_move_api(context):
    printer, gcode, objects = context
    seen = []
    terminal = SimpleNamespace(
        move=lambda position, speed: seen.append(("move", position, speed))
    )
    tilt = bed_tilt.BedTilt.__new__(bed_tilt.BedTilt)
    tilt.toolhead = terminal
    tilt.x_adjust = tilt.y_adjust = tilt.z_adjust = 0.0
    tower = tuning_tower.TuningTower.__new__(tuning_tower.TuningTower)
    tower.normal_transform = tilt
    tower.gcode = gcode
    tower.last_position = [0.0] * 4
    tower.last_z = -999.0
    tower.last_command_value = None
    tower.start = tower.band = tower.step_height = tower.skip = 0.0
    tower.factor = 0.1
    tower.command_fmt = "MARK_TUNE VALUE=%.9f"
    tower.gcode_move = SimpleNamespace(
        get_status=lambda: {"gcode_position": SimpleNamespace(z=2.0)}
    )
    move = gcode_move.GCodeMove.__new__(gcode_move.GCodeMove)
    move.axis_map = {"X": 0, "Y": 1, "Z": 2, "E": 3}
    move.last_position = [0.0] * 4
    move.base_position = [0.0] * 4
    move.absolute_coord = move.absolute_extrude = True
    move.extrude_factor = 1.0
    move.speed = 50.0
    move.speed_factor = 1.0 / 60.0
    move.move_with_transform = tower.move

    def root(gcmd):
        gcmd.run_script("G1 X10 Y5 Z2 E1")

    gcode.register_command("ROOT_MOVE", root)
    gcode.register_command("G1", move.cmd_G1)
    gcode.register_command(
        "MARK_TUNE",
        lambda gcmd: seen.append(("script", gcmd.get_command())),
    )
    gcode.run_script("ROOT_MOVE")
    assert seen == [
        ("script", "MARK_TUNE"),
        ("move", [10.0, 5.0, 2.0, 1.0], 50.0),
    ]


def test_probe_homing_callbacks_use_the_probing_command(context, monkeypatch):
    printer, gcode, objects = context
    seen = []
    roots = []
    objects["toolhead"] = SimpleNamespace(
        get_status=lambda time: {"max_accel": 3000.0}
    )
    effector = smart_effector.SmartEffectorEndstopWrapper.__new__(
        smart_effector.SmartEffectorEndstopWrapper
    )
    effector.printer = printer
    effector.gcode = gcode
    effector.probe_wrapper = SimpleNamespace(
        probe_prepare=lambda move: None, probe_finish=lambda move: None
    )
    effector.probe_accel = 1000.0
    effector.recovery_time = 0.0
    printer_probe = probe.PrinterProbe.__new__(probe.PrinterProbe)
    printer_probe.mcu_probe = effector
    printer.register_event_handler(
        "homing:homing_move_begin", printer_probe._handle_homing_move_begin
    )
    printer.register_event_handler(
        "homing:homing_move_end", printer_probe._handle_homing_move_end
    )
    printer_homing = homing.PrinterHoming.__new__(homing.PrinterHoming)
    printer_homing.printer = printer
    objects["homing"] = printer_homing

    def homing_move(move, position, speed, probe_pos=False):
        assert move.gcmd is roots[0]
        assert move.toolhead is objects["toolhead"]
        printer.send_event("homing:homing_move_begin", move)
        printer.send_event("homing:homing_move_end", move)
        return position

    monkeypatch.setattr(homing.HomingMove, "homing_move", homing_move)
    monkeypatch.setattr(
        homing.HomingMove, "check_no_movement", lambda move: None
    )

    def root(gcmd):
        roots.append(gcmd)
        effector.probing_move([0, 0, 0], 5.0, gcmd)

    gcode.register_command("ROOT_PROBE", root)
    gcode.register_command(
        "M204",
        lambda gcmd: seen.append(
            (gcmd.get_float("S"), gcmd._context.get_root())
        ),
    )
    gcode.run_script("ROOT_PROBE")
    assert seen == [
        (1000.0, roots[0]._context.get_root()),
        (3000.0, roots[0]._context.get_root()),
    ]


def test_trad_rack_next_uses_the_resuming_command(context):
    printer, gcode, objects = context
    rack = trad_rack.TradRack.__new__(trad_rack.TradRack)
    rack.lane_count = 2
    rack.lane_positions = [0.0, 10.0]
    rack.tr_kinematics = SimpleNamespace(
        get_selector_rail=lambda: SimpleNamespace(
            get_homing_info=lambda: SimpleNamespace(position_endstop=0.0)
        )
    )
    seen = []
    rack._prompt_selector_calibration = lambda lane, gcmd=None: seen.append(
        (lane, gcmd)
    )
    rack._measure_selector_to_endstop = lambda distance, gcmd=None: distance
    gcode.register_command(
        "TR_CALIBRATE_SELECTOR", rack.cmd_TR_CALIBRATE_SELECTOR
    )
    gcode.register_command("TR_NEXT", rack.cmd_TR_NEXT)
    gcode.run_script("TR_CALIBRATE_SELECTOR\nTR_NEXT")
    assert [lane for lane, command in seen] == [0, 1]
    assert seen[0][1].get_command() == "TR_CALIBRATE_SELECTOR"
    assert seen[1][1].get_command() == "TR_NEXT"
    assert seen[0][1] is not seen[1][1]
    rack.tr_next_generator.close()


@pytest.mark.parametrize("kind", ["jinja", "python"])
def test_template_command_and_compatibility_entry_points(context, kind):
    printer, gcode, objects = context
    macro = gcode_macro.PrinterGCodeMacro.__new__(gcode_macro.PrinterGCodeMacro)
    macro.printer = printer
    objects["gcode_macro"] = macro
    if kind == "jinja":
        template = gcode_macro.TemplateWrapperJinja.__new__(
            gcode_macro.TemplateWrapperJinja
        )
        template.gcode = gcode
        template.render = lambda context: "MARK_TEMPLATE"
    else:
        template = gcode_macro.TemplateWrapperPython(
            printer, None, "compat", "emit('MARK_TEMPLATE')"
        )
    commands = []
    parents = []

    def parent(gcmd):
        parents.append(gcmd)
        if with_command:
            template.run_gcode(gcmd)
        else:
            template.run_gcode_from_command()

    gcode.register_command("TEMPLATE_TEST", parent)
    gcode.register_command("MARK_TEMPLATE", commands.append)
    for with_command in (True, False):
        commands.clear()
        parents.clear()
        gcode.run_script("TEMPLATE_TEST")
        assert len(commands) == 1
        assert (
            commands[0]._context.get_root() is parents[0]._context.get_root()
        ) == with_command


def test_multi_probe_context_reaches_dockable_probe(context):
    restore_toolhead = True
    printer, gcode, _ = context
    dock = dockable_probe.DockableProbe.__new__(dockable_probe.DockableProbe)
    position = [10.0, 20.0, 5.0, 0.0]
    dock.toolhead = SimpleNamespace(
        get_position=lambda: position, manual_move=lambda *args: None
    )
    dock.lift_speed = 5
    dock.auto_detach_probe = lambda gcmd, return_pos=None: gcmd.run_script(
        "MARK_ATTACH"
    )
    attached = []

    def attach(gcmd, return_pos=None, always_restore_toolhead=False):
        attached.append((return_pos, always_restore_toolhead, gcmd))
        gcmd.run_script("MARK_ATTACH")

    dock.auto_attach_probe = attach
    wrapper = probe.PrinterProbe.__new__(probe.PrinterProbe)
    wrapper.mcu_probe = dock
    wrapper.multi_probe_count = 0
    marks = []
    gcode.register_command("MARK_ATTACH", marks.append)

    def handler(command):
        probing_context = probe.ProbingContext(
            always_restore_toolhead=restore_toolhead, gcmd=command
        )
        with probe.multi_probe_session(wrapper, probing_context):
            assert attached == [(position, restore_toolhead, command)]
            assert wrapper.multi_probe_count == 1
        assert all(
            mark._context.get_root() is command._context.get_root()
            for mark in marks
        )
        assert wrapper.multi_probe_count == 0

    gcode.register_command("PROBE_TEST", handler)
    gcode.run_script("PROBE_TEST")
    assert len(marks) == 2


def test_probe_tolerance_failure_stows_with_live_command(context):
    printer, gcode, objects = context
    wrapper = probe.PrinterProbe.__new__(probe.PrinterProbe)
    wrapper.printer = printer
    wrapper.multi_probe_count = 0
    wrapper.speed = 5
    wrapper.sample_count = 2
    wrapper.samples_tolerance = 0.1
    wrapper.samples_retries = 0
    wrapper.samples_result = "average"
    wrapper.retry_session = SimpleNamespace(
        start=lambda gcmd: None,
        set_position=lambda pos: None,
        end=lambda: None,
    )
    wrapper._discard_first_result = lambda *args: None
    wrapper._retract = lambda *args: None
    positions = iter([[0, 0, 0], [0, 0, 1]])
    wrapper._run_probe_with_retries = lambda *args: next(positions)
    objects["toolhead"] = SimpleNamespace(get_position=lambda: [0, 0, 5, 0])
    backend = probe.ProbeEndstopWrapper.__new__(probe.ProbeEndstopWrapper)
    backend.stow_on_each_sample = False
    cleanup = []
    backend._raise_probe = lambda gcmd: gcmd.run_script("STOW_TEST")
    wrapper.mcu_probe = backend
    gcode.register_command("STOW_TEST", cleanup.append)

    def handler(gcmd):
        with probe.multi_probe_session(
            wrapper, probe.ProbingContext(gcmd=gcmd)
        ):
            with pytest.raises(gcode.error, match="samples_tolerance"):
                wrapper.run_probe(gcmd)
            assert wrapper.multi_probe_count == 1
            assert backend.multi == "FIRST"
            assert not cleanup
            raise gcmd.error("Probe samples exceed samples_tolerance")

    gcode.register_command("PROBE_TEST", handler)
    with pytest.raises(gcode.error, match="samples_tolerance"):
        gcode.run_script("PROBE_TEST")
    assert len(cleanup) == 1
    assert backend.multi == "OFF"
    assert wrapper.multi_probe_count == 0


def test_homing_events_preserve_probe_context(context):
    printer, gcode, objects = context
    wrapper = probe.PrinterProbe.__new__(probe.PrinterProbe)
    wrapper.multi_probe_count = 0
    calls = []
    wrapper.mcu_probe = SimpleNamespace(
        multi_probe_begin=lambda ctx: calls.append(("begin", ctx)),
        multi_probe_end=lambda ctx: ctx.gcmd.run_script("STOW_TEST"),
    )
    printer.register_event_handler(
        "homing:home_rails_begin", wrapper._handle_home_rails_begin
    )
    printer.register_event_handler(
        "homing:home_rails_end", wrapper._handle_home_rails_end
    )
    cleanup = []
    gcode.register_command("STOW_TEST", cleanup.append)
    rail = SimpleNamespace(get_endstops=lambda: [(wrapper.mcu_probe, "probe")])

    def handler(gcmd):
        # Existing event producers need only supply the homing command.
        state = SimpleNamespace(gcmd=gcmd)
        printer.send_event("homing:home_rails_begin", state, [rail])
        assert wrapper.multi_probe_count == 1
        assert calls == [
            (
                "begin",
                probe.ProbingContext(always_restore_toolhead=True, gcmd=gcmd),
            )
        ]
        printer.send_event("homing:home_rails_end", state, [rail])
        assert wrapper.multi_probe_count == 0

    gcode.register_command("HOME_TEST", handler)
    gcode.run_script("HOME_TEST")
    assert len(cleanup) == 1


@pytest.mark.parametrize(
    "load_command, tool_override, missing_lane, expected_lane, expected_tool",
    [
        ("TR_LOAD_TOOLHEAD TOOL=1", None, True, 2, 1),
        ("T1", 1, False, 2, 1),
        ("TR_LOAD_TOOLHEAD LANE=0", None, False, 0, None),
    ],
)
def test_trad_rack_load_restarts_preserve_load_command_in_resume_context(
    context,
    load_command,
    tool_override,
    missing_lane,
    expected_lane,
    expected_tool,
):
    printer, gcode, objects = context
    printer.command_error = gcode.error
    rack = trad_rack.TradRack.__new__(trad_rack.TradRack)
    rack.printer = printer
    rack.gcode = gcode
    rack.reactor = printer.get_reactor()
    rack.active_lane = None
    rack.lane_count = 3
    rack.default_lanes = [0, None if missing_lane else 2, 1]
    rack.resume_stack = deque()
    rack.resume_callbacks = {"check condition": rack._resume_check_condition}
    paused = [False]
    homed = [False]
    objects["pause_resume"] = SimpleNamespace(
        get_status=lambda time: {"is_paused": paused[0]}
    )
    rack._send_pause = lambda gcmd: paused.__setitem__(0, True)
    rack._is_selector_homed = lambda: homed[0]
    resumed = []
    rack._send_resume = lambda message, *, gcmd: resumed.append(gcmd)
    initial = []
    resumes = []
    attempts = []

    def load(*args, gcmd):
        attempts.append((args, gcmd))
        assert gcmd.get_commandline() == initial[0].get_commandline()
        assert (
            gcmd.get_command_parameters() == initial[0].get_command_parameters()
        )
        current = resumes[-1] if resumes else initial[0]
        assert gcmd._context is current._context
        gcmd.run_without_gcode_lock(lambda: None)
        if not homed[0]:
            raise trad_rack.SelectorNotHomedError("Must home selector first")

    rack._load_toolhead = load

    def start(gcmd):
        if tool_override is not None:
            gcmd = gcode._get_extended_params(gcmd)
        initial.append(gcmd)
        rack.cmd_TR_LOAD_TOOLHEAD(gcmd, tool_override)

    def resume(gcmd):
        resumes.append(gcmd)
        rack.cmd_TR_RESUME(gcmd)

    gcode.register_command(load_command.split()[0], start)
    gcode.register_command("TR_RESUME", resume)
    options = (
        " MIN_TEMP=190 EXACT_TEMP=210 BOWDEN_LENGTH=500"
        " EXTRUDER_LOAD_LENGTH=30 HOTEND_LOAD_LENGTH=12"
    )
    gcode.run_script(load_command + options)
    if missing_lane:
        assert not attempts
        rack.default_lanes[1] = expected_lane
        gcode.run_script("TR_RESUME")
    assert len(attempts) == 1
    assert rack.resume_stack
    assert not resumed
    homed[0] = True
    gcode.run_script("TR_RESUME")
    assert len(attempts) == 2
    assert attempts[-1][0] == (
        expected_lane,
        expected_tool,
        190.0,
        210.0,
        500.0,
        30.0,
        12.0,
    )
    assert not rack.resume_stack
    assert resumed == [resumes[-1]]
    for _, command in attempts:
        with pytest.raises(gcode.error, match="no longer valid"):
            command.run_script("")

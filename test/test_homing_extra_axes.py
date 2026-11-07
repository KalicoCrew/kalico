"""Extra-axis homing uses the moving motor's rate and motion limits."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from klippy import chelper, stepper, toolhead
from klippy.extras import homing
from klippy.kinematics.delta import DeltaKinematics
from klippy.kinematics.extruder import PrinterExtruder
from klippy.kinematics.polar import PolarKinematics


@pytest.fixture
def steppers():
    ffi, lib = chelper.get_ffi()

    def make(name, step_dist, axis=None):
        motor = stepper.MCU_stepper.__new__(stepper.MCU_stepper)
        motor._name = name
        motor._step_dist = step_dist
        if axis is None:
            motor._stepper_kinematics = ffi.gc(
                lib.extruder_stepper_alloc(), lib.extruder_stepper_free
            )
        else:
            motor._stepper_kinematics = ffi.gc(
                lib.cartesian_stepper_alloc(axis), lib.free
            )
        return motor

    return {
        "x": make("stepper_x", 0.01, b"x"),
        "e": make("extruder", 0.001),
        "extra": make("extra", 0.002),
    }


def endstop_rate(steppers, delta, names, speed):
    start = [100.0, 100.0, 10.0, 20.0, 30.0]
    end = [pos + dist for pos, dist in zip(start, delta)]
    hmove = homing.HomingMove(
        None, [], SimpleNamespace(get_position=lambda: start), None
    )
    hmove.extra_steppers = {
        "extruder": (3, steppers["e"]),
        "extra": (4, steppers["extra"]),
    }
    endstop = SimpleNamespace(
        get_steppers=lambda: [steppers[name] for name in names]
    )
    return hmove._calc_endstop_rate(endstop, end, speed)


@pytest.mark.parametrize(
    "delta,names,speed,expected",
    [
        ([10, 0, 0, 0, 0], ["x"], 5, 0.002),
        ([0, 0, 0, 10, 0], ["e"], 5, 0.0002),
        ([0, 0, 0, -10, 0], ["e"], 5, 0.0002),
        ([0, 0, 0, 10, 0], ["e"], 20, 0.00005),
        ([0, 0, 0, 0, 10], ["extra"], 5, 0.0004),
        ([0, 0, 0, 10, 30], ["e", "extra"], 5, 0.0004),
        ([10, 0, 0, 2, 0], ["x", "e"], 5, 0.001),
        ([1.0e-10, 0, 0, 10, 0], ["e"], 5, 0.0002),
        ([0, 0, 0, 0, 0], ["e"], 5, 0.001),
        ([10, 0, 0, 0, 0], ["e"], 5, 0.001),
    ],
)
def test_endstop_rate(steppers, delta, names, speed, expected):
    assert endstop_rate(steppers, delta, names, speed) == pytest.approx(
        expected
    )


@pytest.fixture
def extruder_toolhead():
    extruder = PrinterExtruder.__new__(PrinterExtruder)
    extruder.printer = SimpleNamespace(command_error=RuntimeError)
    extruder.heater = SimpleNamespace(can_extrude=True)
    extruder.max_e_dist = 1000.0
    extruder.max_e_velocity = 5.0
    extruder.max_e_accel = 500.0
    extruder.trapq = object()
    th = toolhead.ToolHead.__new__(toolhead.ToolHead)
    th.max_accel = 3000.0
    th.max_velocity = 300.0
    th.max_accel_to_decel = 1500.0
    th.junction_deviation = 0.02
    th.commanded_pos = [0.0, 0.0, 0.0, 0.0]
    th.extra_axes = [extruder]
    th.kin = Mock()
    th.motion_queuing = Mock()
    th.trapq = object()
    th.dwell = Mock()
    th._process_lookahead = Mock()
    th._drip_load_trapq = Mock(return_value=(0.0, 1.0))
    return th, extruder


def test_drip_move_applies_extruder_limits(extruder_toolhead):
    th, extruder = extruder_toolhead
    th.drip_move([0.0, 0.0, 0.0, 10.0], 10.0, object())
    move = th._drip_load_trapq.call_args.args[0]
    assert move.accel == extruder.max_e_accel
    assert move.max_cruise_v2 == extruder.max_e_velocity**2


def test_drip_move_checks_cold_extrusion(extruder_toolhead):
    th, extruder = extruder_toolhead
    extruder.heater.can_extrude = False
    with pytest.raises(RuntimeError, match="Extrude below minimum temp"):
        th.drip_move([0.0, 0.0, 0.0, 10.0], 5.0, object())
    th._drip_load_trapq.assert_not_called()


@pytest.mark.parametrize("kinematics", [DeltaKinematics, PolarKinematics])
def test_extrude_only_drip_move_does_not_require_xyz_homing(
    extruder_toolhead, kinematics
):
    th, extruder = extruder_toolhead
    th.kin = kinematics.__new__(kinematics)
    th.kin.limit_xy2 = -1.0
    th.kin.need_home = True
    th.printer = SimpleNamespace(command_error=RuntimeError)
    th.drip_move([0.0, 0.0, 0.0, 10.0], 5.0, object())
    th._drip_load_trapq.assert_called_once()


def test_drip_move_skips_stationary_extruder(extruder_toolhead):
    th, extruder = extruder_toolhead
    extruder.heater.can_extrude = False
    end = [10.0, 0.0, 0.0, 0.0]
    th.drip_move(end, 5.0, object())
    move = th._drip_load_trapq.call_args.args[0]
    assert move.axes_d[3] == 0.0
    th.kin.check_move.assert_called_once_with(move)


@pytest.mark.parametrize(
    "kinematic_axes,extra_axes,expected",
    [
        (True, (), [11, 12, 13, 4, 5]),
        (False, (3,), [1, 2, 3, 14, 5]),
        (False, (4,), [1, 2, 3, 4, 15]),
        (False, (3, 4), [1, 2, 3, 14, 15]),
        (True, (3,), [11, 12, 13, 14, 5]),
        (False, (), [1, 2, 3, 4, 5]),
    ],
)
def test_set_position_only_updates_selected_axes(
    monkeypatch, kinematic_axes, extra_axes, expected
):
    ffi = Mock()
    monkeypatch.setattr(chelper, "get_ffi", lambda: (None, ffi))
    th = toolhead.ToolHead.__new__(toolhead.ToolHead)
    th.flush_step_generation = Mock()
    th.commanded_pos = [1, 2, 3, 4, 5]
    th.print_time = 1.0
    th.trapq = object()
    th.kin = Mock()
    th.extra_axes = [Mock(), Mock()]
    th.printer = Mock()
    th.set_position(
        [11, 12, 13, 14, 15],
        extra_axes=extra_axes,
        kinematic_axes=kinematic_axes,
    )
    assert th.commanded_pos == expected
    if kinematic_axes:
        th.kin.set_position.assert_called_once_with([11, 12, 13, 14, 15], "")
    else:
        th.kin.set_position.assert_not_called()
    written_trapqs = {
        call.args[0] for call in ffi.trapq_set_position.call_args_list
    }
    expected_trapqs = {th.trapq} if kinematic_axes else set()
    for axis, ea in enumerate(th.extra_axes, 3):
        if axis in extra_axes:
            ea.set_position.assert_called_once_with([expected[axis], 0.0, 0.0])
            expected_trapqs.add(ea.get_trapq())
        else:
            ea.set_position.assert_not_called()
    assert written_trapqs == expected_trapqs


@pytest.mark.parametrize(
    "names,axes",
    [
        (["x"], set()),
        (["e"], {3}),
        (["extra"], {4}),
        (["x", "e"], {3}),
        (["e", "extra"], {3, 4}),
        ([], set()),
    ],
)
@pytest.mark.parametrize(
    "probe_pos,overshoot", [(True, 0), (True, 2), (False, 0), (False, 2)]
)
def test_homing_only_resets_participating_axes(
    monkeypatch, steppers, names, axes, probe_pos, overshoot
):
    start = [100.0, 100.0, 10.0, 20.0, 30.0]
    end = list(start)
    for name, axis in (("x", 0), ("e", 3), ("extra", 4)):
        steppers[name].get_commanded_position = Mock(return_value=start[axis])
        steppers[name].get_trapq = Mock(return_value=object())
        steppers[name].set_position = Mock()
        if name in names:
            end[axis] += 10.0
    kin = Mock()
    kin.get_steppers.return_value = [steppers["x"]]
    kin.calc_position.return_value = [101.0, 102.0, 103.0]
    th = Mock()
    th.get_position.side_effect = lambda: list(start)
    th.get_kinematics.return_value = kin
    th.get_extra_axes.return_value = [None, None, None] + [
        SimpleNamespace(get_trapq=motor.get_trapq)
        for motor in (steppers["e"], steppers["extra"])
    ]

    def position(motor, endstop_name):
        return SimpleNamespace(
            stepper=motor,
            stepper_name=motor.get_name(),
            endstop_name=endstop_name,
            start_pos=0,
            start_cmd_pos=motor.get_commanded_position(),
            trig_pos=10,
            halt_pos=10 + overshoot,
            note_home_end=Mock(),
            verify_no_probe_skew=Mock(),
        )

    monkeypatch.setattr(homing, "StepperPosition", position)
    endstop = Mock()
    endstop.get_steppers.return_value = [steppers[name] for name in names]
    endstop.home_wait.return_value = 1.0
    printer = Mock(command_error=RuntimeError)
    endstops = [(endstop, "endstop")] if names else []
    hmove = homing.HomingMove(printer, endstops, th, None)
    hmove.homing_move(end, 5.0, probe_pos=probe_pos)

    assert hmove.kinematic_axes == ("x" in names)
    calls = th.set_position.call_args_list
    if not names:
        assert not calls
    else:
        assert len(calls) == (2 if not probe_pos and overshoot else 1)
    for call in calls:
        if axes:
            assert call.kwargs == {
                "extra_axes": axes,
                "kinematic_axes": "x" in names,
            }
        else:
            assert call.kwargs == {}
        if "x" not in names:
            assert call.args[0][:3] == start[:3]
    if "x" not in names:
        kin.calc_position.assert_not_called()
    for name, axis in (("e", 3), ("extra", 4)):
        assert steppers[name].set_position.call_count == (
            len(calls) if axis in axes else 0
        )

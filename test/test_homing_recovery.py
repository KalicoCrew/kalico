"""Homing and probing recover the participating extra-axis motors."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from klippy import chelper, toolhead
from klippy.extras.homing import HomingMove
from klippy.extras.manual_stepper import ManualStepper
from klippy.kinematics.extruder import PrinterExtruder


class Motor:
    def __init__(self, name, axis, start, trapq):
        self.name, self.axis, self.trapq = name, axis, trapq
        self.offset = start
        self.position = self.trigger_position = 0
        self.past_times = []

    def get_name(self):
        return self.name

    def get_step_dist(self):
        return 1.0

    def get_trapq(self):
        return self.trapq

    def get_commanded_position(self):
        return self.position + self.offset

    def get_mcu_position(self, position=None):
        return self.position if position is None else position - self.offset

    def mcu_to_commanded_position(self, position):
        return position + self.offset

    def get_past_mcu_position(self, print_time):
        self.past_times.append(print_time)
        return self.trigger_position if print_time == 1.0 else self.position

    def calc_position_from_coord(self, coord):
        return coord[self.axis]

    def set_position(self, coord):
        local_axis = self.axis if self.axis < 3 else 0
        self.offset = coord[local_axis] - self.position


@pytest.fixture
def recovery_move(monkeypatch):
    ffi = Mock()
    monkeypatch.setattr(chelper, "get_ffi", lambda: (None, ffi))
    start = [100.0, 200.0, 300.0, 20.0, 30.0]
    queues = [object(), object(), object()]
    motors = {
        name: Motor(name, axis, start[axis], queues[max(axis - 2, 0)])
        for name, axis in (("x", 0), ("y", 1), ("z", 2), ("e", 3), ("a", 4))
    }
    kin_motors = [motors[name] for name in ("x", "y", "z")]
    kin = Mock()
    kin.get_steppers.return_value = kin_motors
    kin.calc_position.side_effect = lambda positions: [
        positions[motor.name] for motor in kin_motors
    ]
    kin.set_position.side_effect = lambda pos, axes: [
        motor.set_position(pos) for motor in kin_motors
    ]
    th = toolhead.ToolHead.__new__(toolhead.ToolHead)
    th.commanded_pos = list(start)
    th.print_time = 2.0
    th.kin, th.trapq = kin, queues[0]
    th.flush_step_generation = Mock()
    th.get_last_move_time = lambda: 2.0
    th.dwell = Mock()
    th.extra_axes = []
    for axis, queue in ((3, queues[1]), (4, queues[2])):
        group = [motor for motor in motors.values() if motor.axis == axis]
        th.extra_axes.append(
            SimpleNamespace(
                get_trapq=lambda queue=queue: queue,
                set_position=lambda pos, group=group: [
                    motor.set_position(pos) for motor in group
                ],
            )
        )
    printer = Mock(command_error=RuntimeError)
    th.printer = printer

    def run(watched, moving, probe_pos, overshoot=1, trigger_steps=2):
        end = list(start)
        for axis in moving:
            end[axis] += 10.0
        stop = Mock()
        stop.get_steppers.return_value = [motors[name] for name in watched]
        stop.home_wait.return_value = 1.0

        def drip(pos, speed, completion):
            th.commanded_pos[:] = pos
            for motor in motors.values():
                if motor.axis in moving:
                    motor.trigger_position = trigger_steps if watched else 10
                    motor.position = motor.trigger_position + (
                        overshoot if watched else 0
                    )

        th.drip_move = drip
        hmove = HomingMove(
            printer, [(stop, "stop")] if watched else [], th, None
        )
        result = hmove.homing_move(end, 5.0, probe_pos=probe_pos)
        return result, hmove

    return run, th, motors, ffi


@pytest.mark.parametrize("watched", [["e"], ["a"], ["e", "a"], ["x", "e"]])
@pytest.mark.parametrize("probe_pos", [False, True])
@pytest.mark.parametrize("overshoot", [0, 1])
def test_recovers_homed_axes(recovery_move, watched, probe_pos, overshoot):
    run, th, motors, ffi = recovery_move
    result, hmove = run(
        watched, {motors[name].axis for name in watched}, probe_pos, overshoot
    )
    for name, axis, start in (("x", 0, 100.0), ("e", 3, 20.0), ("a", 4, 30.0)):
        halt = start
        if name in watched:
            halt += (2 if probe_pos else 10) + overshoot
        assert th.get_position()[axis] == halt
        if probe_pos:
            assert result[axis] == start + (2 if name in watched else 0)
        if name not in watched:
            assert motors[name].past_times == []
    assert th.get_position()[1:3] == [200.0, 300.0]
    if not probe_pos:
        assert hmove.distance_elapsed == [
            2 + overshoot if "x" in watched else 0.0,
            0.0,
            0.0,
        ]


@pytest.mark.parametrize("probe_pos", [False, True])
def test_extra_axis_moves_without_endstops(recovery_move, probe_pos):
    run, th, motors, ffi = recovery_move
    run([], {3}, probe_pos)
    assert th.get_position() == [100.0, 200.0, 300.0, 30.0, 30.0]
    assert motors["x"].past_times == []
    assert motors["e"].past_times == []


def test_stationary_endstop_axes_are_not_reset(recovery_move):
    run, th, motors, ffi = recovery_move
    run(["z", "e"], {3}, True)
    th.kin.set_position.assert_not_called()
    assert th.get_position() == [100.0, 200.0, 300.0, 23.0, 30.0]
    written_queues = {
        call.args[0] for call in ffi.trapq_set_position.call_args_list
    }
    assert written_queues == {motors["e"].get_trapq()}


@pytest.mark.parametrize("moving_axis", [2, 3, 4])
@pytest.mark.parametrize("probe_pos", [False, True])
@pytest.mark.parametrize("trigger_steps", [0, 2])
def test_shared_endstop_checks_only_participating_axes(
    recovery_move, moving_axis, probe_pos, trigger_steps
):
    run, th, motors, ffi = recovery_move
    th.printer.get_start_args.return_value = {}
    _, hmove = run(
        ["z", "e", "a"],
        {moving_axis},
        probe_pos,
        overshoot=0,
        trigger_steps=trigger_steps,
    )
    assert hmove.check_no_movement() == (None if trigger_steps else "stop")
    for name in ("z", "e", "a"):
        if motors[name].axis != moving_axis:
            assert motors[name].past_times == []


@pytest.mark.parametrize("primary_name", ["e", "a", None])
def test_extruder_position_reset_respects_motor_queue(
    recovery_move, primary_name
):
    run, th, motors, ffi = recovery_move
    extruder = PrinterExtruder.__new__(PrinterExtruder)
    extruder.trapq = motors["e"].get_trapq()
    extruder.last_position = 20.0
    extruder.extruder_stepper = (
        SimpleNamespace(stepper=motors[primary_name])
        if primary_name is not None
        else None
    )
    th.extra_axes[0] = extruder
    run(["e"], {3}, True)
    assert extruder.last_position == th.get_position()[3] == 23.0
    assert motors["e"].get_commanded_position() == 23.0
    assert motors["a"].get_commanded_position() == th.get_position()[4] == 30.0


def test_manual_stepper_homing_uses_three_argument_drip_interface():
    motor = Motor("manual", 0, 0.0, object())
    manual = ManualStepper.__new__(ManualStepper)
    manual.steppers, manual.rail = [motor], motor
    manual.commanded_pos = manual.next_cmd_time = 0.0
    manual.homing_accel = 1.0
    manual.sync_print_time = Mock()
    manual.flush_step_generation = Mock()
    manual.get_last_move_time = lambda: 2.0
    manual.motion_queuing = Mock()
    manual.trapq = motor.trapq
    manual.printer = Mock(command_error=RuntimeError)

    def submit(start, position, speed, accel):
        manual.commanded_pos = position
        motor.position = motor.trigger_position = 2
        return 2.0

    manual._submit_move = submit
    stop = Mock()
    stop.get_steppers.return_value = [motor]
    stop.home_wait.return_value = 1.0
    HomingMove(manual.printer, [(stop, "stop")], manual, None).homing_move(
        [10.0, 0.0, 0.0, 0.0], 5.0
    )
    stop.home_wait.assert_called_once()
    manual.motion_queuing.drip_update_time.assert_called_once()

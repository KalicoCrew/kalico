"""Homing end listeners handle both completed and partially started homes."""

from types import SimpleNamespace

import pytest

from klippy.extras import (
    endstop_phase,
    gcode_move,
    homing,
    trad_rack,
    z_calibration,
    z_thermal_adjust,
)


@pytest.mark.parametrize(
    "homing_class", [homing.Homing, trad_rack.TradRackHoming]
)
@pytest.mark.parametrize("failure", ["begin", "move"])
def test_homing_end_after_failure(monkeypatch, homing_class, failure):
    events = []
    fail = False
    error = RuntimeError("homing failed")

    def begin(state, rails):
        if fail and failure == "begin":
            raise error

    def move(*args):
        if fail and failure == "move":
            raise error

    stepper = SimpleNamespace(
        get_name=lambda: "stepper_z", is_active_axis=lambda axis: axis == "z"
    )
    info = SimpleNamespace(accel=None, speed=5, retract_dist=0, min_home_dist=0)
    rail = SimpleNamespace(
        get_endstops=lambda: [],
        get_steppers=lambda: [stepper],
        get_homing_info=lambda: info,
        position_endstop=10,
        homing_speed=5,
        second_homing_speed=2,
        homing_retract_dist=0,
        position_min=0,
    )
    toolhead = SimpleNamespace(
        get_position=lambda: [0, 0, 0, 0],
        set_position=lambda *args, **kwargs: None,
        flush_step_generation=lambda: None,
    )
    phase = endstop_phase.EndstopPhase.__new__(endstop_phase.EndstopPhase)
    phase.name = "stepper_z"
    phase.align_endstop = lambda rail: 0
    phase.get_homed_offset = lambda stepper, pos: 0
    phases = endstop_phase.EndstopPhases.__new__(endstop_phase.EndstopPhases)
    phases.update_stepper = lambda *args: events.append("phase")
    calibration = z_calibration.ZCalibrationHelper.__new__(
        z_calibration.ZCalibrationHelper
    )
    calibration.probing_speed = calibration.second_speed = None
    calibration.retract_dist = calibration.position_min = None
    thermal = z_thermal_adjust.ZThermalAdjuster.__new__(
        z_thermal_adjust.ZThermalAdjuster
    )
    thermal.smoothed_temp = 25
    gmove = gcode_move.GCodeMove.__new__(gcode_move.GCodeMove)
    gmove.reset_last_position = lambda: events.append("position")
    gmove.base_position = [0, 0, 0, 0]
    gmove.homing_position = [0, 0, 10, 0]
    listeners = [
        phase.handle_home_rails_end,
        phases.handle_home_rails_end,
        calibration.handle_home_rails_end,
        thermal.handle_homing_move_end,
        gmove._handle_home_rails_end,
        lambda state, rails: events.append("cleanup"),
    ]

    def send_event(event, state, rails):
        callbacks = [begin] if event.endswith("begin") else listeners
        for callback in callbacks:
            callback(state, rails)

    printer = SimpleNamespace(
        lookup_object=lambda name: toolhead, send_event=send_event
    )
    hmove = SimpleNamespace(
        homing_move=move,
        moved_less_than_dist=lambda *args: False,
        stepper_positions=[
            SimpleNamespace(stepper_name="stepper_z", trig_pos=100)
        ],
    )
    monkeypatch.setattr(homing, "HomingMove", lambda *args: hmove)
    monkeypatch.setattr(trad_rack, "HomingMove", lambda *args: hmove)
    state = homing_class.__new__(homing_class)
    homing.Homing.__init__(state, printer, None)
    state.set_axes([2])
    state._set_homing_accel = state._set_homing_current = lambda *a, **kw: None
    state._reset_endstop_states = lambda *args: None

    # First home succeeds; a later failed home must not reuse its trigger data.
    state.home_rails([rail], [None, None, 0], [None, None, 10])
    assert events == ["phase", "position", "cleanup"]
    assert calibration.z_homing == 10
    assert thermal.ref_temperature == 25
    assert gmove.base_position[2] == 10

    events.clear()
    fail = True
    calibration.z_homing = None
    thermal.smoothed_temp = 50
    gmove.base_position[2] = 20
    with pytest.raises(RuntimeError) as exc:
        state.home_rails([rail], [None, None, 0], [None, None, 10])
    assert exc.value is error
    assert events == ["position", "cleanup"]
    assert calibration.z_homing is None
    assert thermal.ref_temperature == 25
    assert gmove.base_position[2] == 20

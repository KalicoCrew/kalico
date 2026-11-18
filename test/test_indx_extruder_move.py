"""INDX move status reflects the extruder rather than other endstop motors."""

import contextlib
from dataclasses import dataclass
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import test_homing_recovery as homing_tests

from klippy.extras.heaters import Heater
from klippy.extras.indx import heater as indx_heater

recovery_move = homing_tests.recovery_move


@dataclass
class TareParams:
    trigger_force: float = 0.0
    drift_safety_limit: float = 1.0


@pytest.mark.parametrize("requested_speed", [2.0, 100.0])
@pytest.mark.parametrize(
    "force,distance,steps",
    [
        (None, -10.0, -10),
        (None, 0.0, 0),
        (None, 10.0, 10),
        (100, -10.0, -2),
        (100, 0.0, 0),
        (100, 10.0, 2),
        (100, 10.0, 0),
    ],
)
def test_extruder_move_reports_its_own_movement(
    recovery_move, monkeypatch, force, requested_speed, distance, steps
):
    _, th, motors, ffi = recovery_move
    th.printer.lookup_object.side_effect = lambda name: {"toolhead": th}[name]
    th.printer.get_start_args.return_value = {}
    th.wait_moves = Mock()
    heater = Heater.__new__(Heater)
    heater._calibration = None
    heater.cold_extrude = heater.can_extrude = False
    heater.min_extrude_temp, heater.smoothed_temp = 170.0, 20.0
    extruder = SimpleNamespace(
        get_heater=lambda: heater,
        extruder_stepper=SimpleNamespace(stepper=motors["e"]),
        max_e_velocity=5.0,
    )
    th.get_extruder = lambda: extruder
    current_helper = SimpleNamespace(
        get_current=lambda: (1.0, 0.3, None),
        push_current=lambda *args: contextlib.nullcontext(),
    )
    monkeypatch.setattr(
        indx_heater.compat, "get_tmc_current_helper", lambda s: current_helper
    )
    stop = Mock()
    stop.get_steppers.return_value = [motors["z"], motors["e"]]
    stop.home_wait.return_value = 1.0
    stop.build_tare_params.return_value = TareParams()
    stop.with_tare_params.side_effect = lambda params: contextlib.nullcontext()
    indx = indx_heater.IndxToolboardHeater.__new__(
        indx_heater.IndxToolboardHeater
    )
    indx.heater = heater
    indx.toolboard = SimpleNamespace(printer=th.printer)
    indx.extruder_force_endstop = stop

    def drip(pos, speed, completion):
        assert heater.can_extrude
        assert speed == min(requested_speed, extruder.max_e_velocity)
        th.commanded_pos[:] = pos
        motors["e"].position = motors["e"].trigger_position = steps

    th.drip_move = drip
    params = {
        "DISTANCE": distance,
        "SPEED": requested_speed,
        "CURRENT": 0.5,
        "FORCE": force,
    }
    gcmd = SimpleNamespace(
        get_float=lambda name, **kwargs: params[name],
        get_int=lambda name, **kwargs: params.get(name, kwargs.get("default")),
        get=lambda name, default=None: params.get(name, default),
        error=RuntimeError,
        respond_info=Mock(),
    )
    indx.cmd_EXTRUDER_MOVE(gcmd)
    assert indx._last_extruder_move_info["movement"] == bool(steps)
    assert indx._last_extruder_move_info["halt_position"] == 20.0 + steps
    assert motors["z"].position == 0
    assert not heater.cold_extrude

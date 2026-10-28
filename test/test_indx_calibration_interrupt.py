"""INDX host acquisition waits cancel without publishing calibration data."""

from collections import namedtuple
from types import SimpleNamespace

import pytest
import test_gcode_interrupt as commands

from klippy.extras.indx import calibration
from klippy.extras.indx.heater import IndxToolboardHeater
from klippy.gcode import WaitInterruption

dispatch = commands.dispatch


@pytest.mark.parametrize("phase", ["heatup", "fan"])
def test_indx_acquisition_interrupt_restores_heater_and_callbacks(
    dispatch, monkeypatch, phase
):
    gcode, reactor, printer, objects = dispatch
    heater, _ = commands.calibration_heater(dispatch)
    indx = IndxToolboardHeater.__new__(IndxToolboardHeater)
    indx.heater = heater
    indx.heaters = [heater]
    indx.heater_raw_set_temp = heater.set_temp
    indx.toolboard = SimpleNamespace(printer=printer)
    indx.max_model_error = 5.0
    indx.max_temp_nozzle = 300.0
    indx.coil_timings = (1.0, 1.0, 1.0)
    indx.temperature_callbacks = []
    indx._blend_ambient_temp = lambda temperature: temperature
    params = namedtuple(
        "Params", "max_power max_power_temp_coeff thermal_capacity to_ambient_r"
    )
    indx.thermal_model = SimpleNamespace(
        params=params(1.0, 0.0, 10.0, 2.0), is_valid=lambda: True
    )
    # MCU-side tuning remains synchronous and is outside this regression test.
    indx._calibrate_coil_driver = lambda gcmd: None
    motor_states = []
    motor = SimpleNamespace(
        is_motor_enabled=lambda: True,
        motor_enable=lambda time: motor_states.append(True),
        motor_disable=lambda time: motor_states.append(False),
    )
    indx.fan = SimpleNamespace(steppers=[motor])
    speeds = []
    indx.part_cooling_fan = SimpleNamespace(set_speed=speeds.append)
    objects["toolhead"] = SimpleNamespace(
        dwell=lambda duration: None, get_last_move_time=lambda: 0.0
    )
    monkeypatch.setattr(calibration, "MODEL_CAL_BASELINE_TIME", 0.01)
    monkeypatch.setattr(calibration, "MODEL_CAL_POLL_INTERVAL", 0.001)
    monkeypatch.setattr(
        calibration,
        "run_fit_in_background",
        lambda *args: pytest.fail("fit started"),
    )
    indx._apply_model_calibration = lambda *args: pytest.fail(
        "results published"
    )
    cancelled = []
    messages = {
        "heatup": "INDX: heating to",
        "fan": "INDX: fan",
    }

    def report(message):
        if messages[phase] in message and not cancelled:
            cancelled.append(True)
            reactor.register_callback(
                lambda eventtime: gcode.run_script("HEATER_INTERRUPT")
            )

    gcode.register_output_handler(report)

    def reading(eventtime):
        value = heater.target_temp if phase == "fan" else 20.0
        sample = SimpleNamespace(
            nozzle_temperature=value,
            sensor_temperature=20.0,
            delta_time=0.1,
            delta_energy=0.1,
            delta_pwm_energy=0.1,
            pwm_avg=0.5,
        )
        for callback in indx.temperature_callbacks[:]:
            callback(sample)
        if not cancelled:
            reactor.register_callback(reading, eventtime + 0.001)

    gcode.register_command("INDX_CALIBRATE", indx.cmd_CALIBRATE)
    gcode.register_command("INDX_FAN_CALIBRATE", indx.cmd_FAN_CALIBRATE)

    def action():
        reactor.register_callback(reading)
        script = (
            "INDX_FAN_CALIBRATE SETTLE_TIME=0"
            if phase == "fan"
            else "INDX_CALIBRATE"
        )
        with pytest.raises(WaitInterruption):
            gcode.run_script(script)

    commands.run_in_reactor(reactor, action)
    assert cancelled
    assert heater.target_temp == 0.0
    assert heater._calibration is None
    assert indx.temperature_callbacks == []
    assert indx.max_model_error == 5.0
    if phase == "fan":
        assert speeds[0] == speeds[-1] == 0.0
        assert any(speed > 0.0 for speed in speeds)
    else:
        assert motor_states == [False, True]


def test_indx_commands_reject_a_calibrating_heater(dispatch):
    from klippy.extras.indx.heater import IndxBracketTempSensor

    gcode, _, _, _ = dispatch
    heater, _ = commands.calibration_heater(dispatch)
    indx = IndxToolboardHeater.__new__(IndxToolboardHeater)
    indx.heater = heater
    bracket = IndxBracketTempSensor.__new__(IndxBracketTempSensor)
    bracket.heater = indx
    handlers = {
        "INDX_SET_PID": indx.cmd_SET_PID,
        "INDX_SET_CYCLE_LIMIT": indx.cmd_SET_CYCLE_LIMIT,
        "INDX_SET_IR_SENSOR_PARAMS": indx.cmd_SET_IR_SENSOR_PARAMS,
        "INDX_SET_MODEL_PARAMS": indx.cmd_SET_MODEL_PARAMS,
        "INDX_FORCE_BRACKET_TEMP": bracket.cmd_FORCE_BRACKET_TEMP,
        "INDX_MEASURE_POWER": indx.cmd_MEASURE_POWER,
        "INDX_LOAD_FILAMENT": indx.cmd_LOAD_FILAMENT,
        "INDX_CLEAR_FILAMENT": indx.cmd_CLEAR_FILAMENT,
        "INDX_EXTRUDER_MOVE": indx.cmd_EXTRUDER_MOVE,
        "INDX_CALIBRATE": indx.cmd_CALIBRATE,
        "INDX_FAN_CALIBRATE": indx.cmd_FAN_CALIBRATE,
    }
    for name, callback in handlers.items():
        gcode.register_command(name, callback)

    def handler(gcmd):
        with heater.calibration():
            for name in handlers:
                with pytest.raises(gcmd.error, match="is being calibrated"):
                    gcmd.run_script(name)

    gcode.register_command("GUARD_TEST", handler)
    gcode.run_script("GUARD_TEST")


def test_calibration_preflight_heating_behavior(dispatch):
    gcode, _, printer, _ = dispatch
    heater, _ = commands.calibration_heater(dispatch)
    heater.set_temp(210.0)
    original = heater.control
    indx = IndxToolboardHeater.__new__(IndxToolboardHeater)
    indx.heater = heater
    indx.heaters = [heater]
    indx.toolboard = SimpleNamespace(printer=printer)
    indx._calibrate = lambda command: pytest.fail("calibration started")
    gcode.register_command("INDX_CALIBRATE", indx.cmd_CALIBRATE)
    gcode.register_command("INDX_FAN_CALIBRATE", indx.cmd_FAN_CALIBRATE)

    with pytest.raises(gcode.error, match="Turn the heater off first"):
        gcode.run_script("INDX_CALIBRATE")

    assert heater.target_temp == 210.0
    assert heater.control is original
    assert heater._calibration is None

    # FAN_CALIBRATE intentionally turns heating off even when preflight rejects it.
    indx._heater_config_valid = lambda: True
    for fan in (None, SimpleNamespace()):
        heater.set_temp(210.0)
        indx.part_cooling_fan = fan
        message = (
            "part cooling fan is not configured" if fan is None else "BREAKS"
        )
        with pytest.raises(gcode.error, match=message):
            gcode.run_script("INDX_FAN_CALIBRATE BREAKS=2")
        assert heater.target_temp == 0.0
        assert heater.control is original
        assert heater._calibration is None

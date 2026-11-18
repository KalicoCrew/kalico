"""MPC reads the extruder step history on the reactor thread only."""

from types import SimpleNamespace

import pytest

from klippy.extras import control_mpc

PROFILE = {
    "block_heat_capacity": 20.0,
    "ambient_transfer": 0.1,
    "target_reach_time": 2.0,
    "heater_power": 40.0,
    "smoothing": 0.83,
    "sensor_responsiveness": 0.2,
    "min_ambient_change": 1.0,
    "steady_state_rate": 0.5,
    "filament_diameter": 1.75,
    "filament_density": 1.2,
    "filament_heat_capacity": 1.8,
    "maximum_retract": 2.0,
    "filament_temp_src": (control_mpc.FILAMENT_TEMP_SRC_AMBIENT,),
    "ambient_temp_sensor": None,
    "cooling_fan": None,
    "fan_ambient_transfer": [],
}
EXTRUDE_SPEED = 5.0


def test_extruder_lookup_runs_on_reactor():
    lookups = []
    callbacks = []

    def find_past_position(print_time):
        lookups.append(print_time)
        return EXTRUDE_SPEED * print_time

    heater = SimpleNamespace(
        get_max_power=lambda: 1.0, set_pwm=lambda read_time, value: None
    )
    extruder = SimpleNamespace(
        get_heater=lambda: heater, find_past_position=find_past_position
    )
    toolhead = SimpleNamespace(get_extruder=lambda: extruder)
    reactor = SimpleNamespace(register_async_callback=callbacks.append)
    heater.printer = SimpleNamespace(
        get_reactor=lambda: reactor, lookup_object=lambda name: toolhead
    )
    mpc = control_mpc.ControlMPC(
        PROFILE, heater, load_clean=True, register=False
    )

    # The sensor callback (serial thread) only queues the lookup
    mpc.temperature_update(10.0, 200.0, 210.0)
    assert lookups == []
    assert mpc.last_loss_filament == 0.0

    # The reactor runs it, and the next sample uses the result
    callbacks.pop()(0.0)
    assert mpc.extrude_speed == pytest.approx(EXTRUDE_SPEED)
    mpc.temperature_update(10.3, 200.0, 210.0)
    assert mpc.last_loss_filament > 0.0

    # Heater off: no stale speed left for the next heat-up
    mpc.temperature_update(10.6, 200.0, 0.0)
    assert mpc.extrude_speed == 0.0

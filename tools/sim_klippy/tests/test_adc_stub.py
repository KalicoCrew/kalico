"""HeaterModel + thermistor curve. Used by the orchestrator to feed
realistic thermistor readings back into the firmware."""
import pytest
from tools.sim_klippy.orchestrator.adc_stub import HeaterModel, temp_to_adc


def test_bed_ramps_toward_target():
    h = HeaterModel(initial_temp_c=25, ramp_rate_c_per_s=0.5)
    h.set_target(60)
    for _ in range(120):
        h.step(dt_s=1.0)
    assert h.temp_c == pytest.approx(60.0, abs=0.5)


def test_does_not_overshoot():
    h = HeaterModel(initial_temp_c=25, ramp_rate_c_per_s=0.5)
    h.set_target(30)
    for _ in range(100):
        h.step(dt_s=1.0)
    assert 29.5 <= h.temp_c <= 30.5


def test_can_cool_down():
    h = HeaterModel(initial_temp_c=200, ramp_rate_c_per_s=2.0)
    h.set_target(25)
    for _ in range(200):
        h.step(dt_s=1.0)
    assert h.temp_c == pytest.approx(25.0, abs=1.0)


def test_temp_to_adc_monotone():
    """Higher temp → lower thermistor resistance → lower ADC reading
    (with our 4700Ω pull-up to 3.3V). Just assert monotonicity."""
    assert temp_to_adc(25) > temp_to_adc(60)
    assert temp_to_adc(60) > temp_to_adc(200)
    assert temp_to_adc(0) > temp_to_adc(25)


def test_temp_to_adc_in_range():
    assert 0 < temp_to_adc(25) < 4096
    assert 0 < temp_to_adc(200) < 4096

import pytest

from klippy import pins
from klippy.extras.probe import (
    calc_probe_z_result,
    validate_virtual_endstop_request,
)


def _pin_params(pin="z_virtual_endstop", invert=0, pullup=0):
    return {
        "chip": object(),
        "chip_name": "probe",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def test_average():
    assert calc_probe_z_result([1.0, 2.0, 6.0], "average") == pytest.approx(3.0)


def test_median_odd():
    assert calc_probe_z_result([5.0, 1.0, 2.0], "median") == 2.0


def test_median_even_averages_middle_pair():
    assert calc_probe_z_result([4.0, 1.0, 2.0, 3.0], "median") == pytest.approx(
        2.5
    )


def test_unknown_method_raises():
    with pytest.raises(ValueError):
        calc_probe_z_result([1.0], "mode")


def test_valid_virtual_endstop_request_passes():
    validate_virtual_endstop_request(_pin_params(), 2)


def test_wrong_pin_name_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pin="virtual_endstop"), 2)


def test_modifiers_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pullup=1), 2)
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(invert=1), 2)


def test_non_z_axis_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(), 0)

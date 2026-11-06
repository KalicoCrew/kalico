import pathlib
import typing

import pytest
from klippy_testing import PrinterShim, Restart

from klippy.extras.control_mpc import ControlMPC, MpcCalibrate


class FakeFan:
    def __init__(self, speed=0.0):
        self.speed = speed

    def get_status(self, eventtime):
        return {"speed": self.speed}

    def set_speed(self, value, print_time=None):
        self.speed = value


def make_control(fans, transfers):
    control = ControlMPC.__new__(ControlMPC)
    control.const_ambient_transfer = 0.1
    control.cooling_fans = fans
    control.const_fan_ambient_transfer = transfers
    return control


def test_no_fans_uses_base_transfer():
    assert make_control([], [])._ambient_transfer(0.0) == 0.1


def test_uncalibrated_fan_uses_base_transfer():
    assert make_control([FakeFan(1.0)], [])._ambient_transfer(0.0) == 0.1


def test_single_fan_interpolates():
    control = make_control([FakeFan(0.25)], [[0.1, 0.2, 0.4]])
    assert control._ambient_transfer(0.0) == pytest.approx(0.15)


def test_independent_fans_add_up():
    control = make_control(
        [FakeFan(1.0), FakeFan(0.5)],
        [[0.1, 0.2, 0.3], [0.1, 0.15, 0.2]],
    )
    assert control._ambient_transfer(0.0) == pytest.approx(0.1 + 0.2 + 0.05)


def test_stopped_fans_add_nothing():
    control = make_control(
        [FakeFan(0.0), FakeFan(0.0)],
        [[0.1, 0.2, 0.3], [0.12, 0.15, 0.2]],
    )
    assert control._ambient_transfer(0.0) == pytest.approx(0.1)


def test_calibration_measures_one_fan_at_a_time():
    fans = [FakeFan(1.0), FakeFan(1.0)]
    measured = []

    class Heater:
        def set_temp(self, temp):
            pass

    class Gcmd:
        def respond_info(self, msg):
            pass

    def measure_power(max_time, sample_time):
        measured.append((fans[0].speed, fans[1].speed))
        return 10.0 + fans[0].speed * 10.0 + fans[1].speed * 20.0

    calibrate = MpcCalibrate.__new__(MpcCalibrate)
    calibrate.heater = Heater()
    calibrate.orig_control = make_control(fans, [])
    calibrate.wait_stable = lambda cycles=5: None
    calibrate.measure_power = measure_power

    res = calibrate.transfer_test(
        Gcmd(), 20.0, 5.0, 2, {"post_block_temp": 200.0}
    )

    assert measured == [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    assert res["fan_powers"] == [
        [(0.0, 10.0), (1.0, 20.0)],
        [(0.0, 10.0), (1.0, 30.0)],
    ]
    assert res["base_power"] == 10.0
    assert fans[0].speed == fans[1].speed == 0.0


def test_second_pass_keeps_one_list_per_fan():
    calibrate = MpcCalibrate.__new__(MpcCalibrate)
    res = calibrate.process_second_pass(
        {
            "t1": 100.0,
            "t2": 150.0,
            "dt": 10.0,
            "start_temp": 25.0,
            "t1_time": 5.0,
        },
        {
            "target_temp": 120.0,
            "base_power": 10.0,
            "fan_powers": [
                [(0.0, 10.0), (1.0, 20.0)],
                [(0.0, 10.0), (1.0, 30.0)],
            ],
        },
        20.0,
        50.0,
    )
    assert res["fan_ambient_transfer"] == [[0.1, 0.2], [0.1, 0.3]]


def test_multiline_transfer_survives_save_config(
    config_root: typing.Annotated[pathlib.Path, "test_configs/autosave"],
):
    start_args = {"config_file": str(config_root / "printer.cfg")}
    with PrinterShim(start_args) as printer:
        pconfig = printer.lookup_object("configfile")
        printer.load_config()
        pconfig.set("extruder", "fan_ambient_transfer", "1, 2\n3, 4")
        with pytest.raises(Restart):
            printer.call("SAVE_CONFIG")

    with PrinterShim(start_args) as printer:
        section = printer.load_config().getsection("extruder")
        assert section.getlists(
            "fan_ambient_transfer", seps=(",", "\n"), parser=float
        ) == ((1.0, 2.0), (3.0, 4.0))

import pytest

from klippy.extras import homing as homing_mod
from klippy.extras import servo_axis


class FakeErrConfig:
    error = RuntimeError


def test_infer_positive_dir_at_min_is_negative():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", -6.0, -6.0, 235.0) is False


def test_infer_positive_dir_at_max_is_positive():
    cfg = FakeErrConfig()
    assert servo_axis.infer_positive_dir(cfg, "x", 235.0, -6.0, 235.0) is True


def test_infer_positive_dir_mid_range_is_config_error():
    cfg = FakeErrConfig()
    with pytest.raises(RuntimeError, match="position_endstop"):
        servo_axis.infer_positive_dir(cfg, "x", 100.0, -6.0, 235.0)


def test_get_homing_info_reflects_homing_config():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.position_endstop = -6.0
    rail.homing_speed = 50.0
    rail.homing_positive_dir = False
    hi = rail.get_homing_info()
    assert hi.speed == 50.0
    assert hi.position_endstop == -6.0
    assert hi.positive_dir is False


class FakeSectionsConfig:
    def __init__(self, sections):
        self._sections = sections

    def has_section(self, name):
        return name in self._sections


def test_endstop_section_finds_stepper():
    cfg = FakeSectionsConfig({"stepper_x"})
    assert homing_mod._endstop_section(cfg, "x") == "stepper_x"


def test_endstop_section_finds_servo():
    cfg = FakeSectionsConfig({"servo_x"})
    assert homing_mod._endstop_section(cfg, "x") == "servo_x"


def test_endstop_section_none_when_axis_absent():
    cfg = FakeSectionsConfig({"stepper_y"})
    assert homing_mod._endstop_section(cfg, "x") is None


class FakeStepperEnable:
    def __init__(self):
        self.calls = []

    def motor_debug_enable(self, name, enable):
        self.calls.append((name, enable))


class FakeStepper:
    def __init__(self, name):
        self._name = name

    def get_name(self):
        return self._name


class FakeRail:
    def __init__(self, steppers, name):
        self._steppers = steppers
        self._name = name

    def get_steppers(self):
        return self._steppers

    def get_name(self, short=False):
        return self._name


def test_enable_homing_motors_enables_each_stepper():
    se = FakeStepperEnable()
    rail = FakeRail(
        [FakeStepper("stepper_x"), FakeStepper("stepper_x1")], "stepper_x"
    )
    homing_mod._enable_homing_motors(se, rail)
    assert se.calls == [("stepper_x", True), ("stepper_x1", True)]


def test_enable_homing_motors_enables_servo_rail_by_name():
    se = FakeStepperEnable()
    rail = FakeRail([], "servo_x")
    homing_mod._enable_homing_motors(se, rail)
    assert se.calls == [("servo_x", True)]

import pytest

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

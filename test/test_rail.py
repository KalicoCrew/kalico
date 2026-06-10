from klippy import rail, stepper
from klippy.extras import servo_axis


def make_homing_configured_rail(cls):
    r = cls.__new__(cls)
    r.homing_speed = 40.0
    r.position_endstop = -6.0
    r.homing_retract_speed = 20.0
    r.homing_retract_dist = 3.0
    r.homing_positive_dir = False
    r.second_homing_speed = 10.0
    r.use_sensorless_homing = False
    r.min_home_dist = 3.0
    r.homing_accel = None
    r.position_min = -6.0
    r.position_max = 235.0
    return r


def test_printer_rail_homing_info_is_shared_type():
    hi = make_homing_configured_rail(stepper.PrinterRail).get_homing_info()
    assert isinstance(hi, rail.HomingInfo)
    assert hi.speed == 40.0
    assert hi.retract_dist == 3.0
    assert hi.retract_speed == 20.0


def test_servo_rail_homing_info_is_shared_type():
    hi = make_homing_configured_rail(servo_axis.ServoRail).get_homing_info()
    assert isinstance(hi, rail.HomingInfo)
    assert hi.speed == 40.0
    assert hi.retract_dist == 3.0
    assert hi.retract_speed == 20.0


def test_get_range_is_shared():
    for cls in (stepper.PrinterRail, servo_axis.ServoRail):
        r = make_homing_configured_rail(cls)
        assert r.get_range() == (-6.0, 235.0)


def test_base_rail_defaults_legacy_homing_fields():
    base = rail.BaseRail()
    assert base.second_homing_speed == 0.0
    assert base.use_sensorless_homing is False
    assert base.min_home_dist == 0.0
    assert base.homing_accel is None

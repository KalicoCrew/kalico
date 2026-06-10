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


class FakeRailConfig:
    error = RuntimeError
    _UNSET = object()

    def __init__(self, name, options):
        self._name = name
        self._options = dict(options)

    def get_printer(self):
        return None

    def get_name(self):
        return self._name

    def get(self, option, default=_UNSET):
        return self._lookup(option, default)

    def getfloat(
        self, option, default=_UNSET, above=None, minval=None, maxval=None
    ):
        return self._lookup(option, default)

    def getint(self, option, default=_UNSET, minval=None, maxval=None):
        return self._lookup(option, default)

    def _lookup(self, option, default):
        if option in self._options:
            return self._options[option]
        if default is FakeRailConfig._UNSET:
            raise RuntimeError("missing required option %r" % (option,))
        return default


SERVO_Z_OPTIONS = {
    "protocol": "ethercat",
    "node": "z_drive",
    "rotation_distance": 40.0,
    "encoder_counts_per_rev": 131072,
    "position_min": -6.0,
    "position_max": 235.0,
    "endstop_pin": "ec_z:endstop",
    "position_endstop": -6.0,
}


def make_servo_rail(extra=(), drop=()):
    options = dict(SERVO_Z_OPTIONS)
    options.update(extra)
    for key in drop:
        options.pop(key)
    return servo_axis.ServoRail(FakeRailConfig("servo_z", options))


def test_get_homing_info_reflects_homing_config():
    rail = make_servo_rail(extra={"homing_speed": 50.0})
    hi = rail.get_homing_info()
    assert hi.speed == 50.0
    assert hi.position_endstop == -6.0
    assert hi.positive_dir is False


def test_homing_info_reflects_retract_config():
    rail = make_servo_rail(
        extra={"homing_retract_dist": 3.0, "homing_retract_speed": 10.0}
    )
    hi = rail.get_homing_info()
    assert hi.retract_dist == 3.0
    assert hi.retract_speed == 10.0


def test_retract_defaults_match_stepper_rail():
    rail = make_servo_rail(extra={"homing_speed": 50.0})
    hi = rail.get_homing_info()
    assert hi.retract_dist == 5.0
    assert hi.retract_speed == 50.0


def test_no_endstop_pin_means_zero_retract():
    rail = make_servo_rail(drop=("endstop_pin", "position_endstop"))
    hi = rail.get_homing_info()
    assert hi.retract_dist == 0.0
    assert rail.second_homing_speed == 0.0


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


def make_homing_servo_rail():
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.axis = "x"
    rail.name = "servo_x"
    rail.rotation_distance = 40.0
    rail.encoder_counts_per_rev = 131072
    rail.homing_following_error = 2.5
    rail.homing_max_torque = 50.0
    rail.following_error = None
    rail.max_torque = None
    return rail


def test_homing_drive_limits_convert_units():
    rail = make_homing_servo_rail()
    counts, tenth_pct = rail.get_homing_drive_limits()
    assert counts == 8192
    assert tenth_pct == 500


def test_session_drive_limits_none_when_unconfigured():
    rail = make_homing_servo_rail()
    assert rail.get_session_drive_limits() == (None, None)


def test_session_drive_limits_convert_units():
    rail = make_homing_servo_rail()
    rail.following_error = 5.0
    rail.max_torque = 120.0
    assert rail.get_session_drive_limits() == (16384, 1200)


class FakeLimitsBridge:
    def __init__(self):
        self.calls = []

    def set_drive_limits(self, handle, counts, tenth_pct):
        self.calls.append(("set", handle, counts, tenth_pct))

    def restore_drive_limits(self, handle):
        self.calls.append(("restore", handle))


def test_homing_limits_guard_sets_and_restores():
    bridge = FakeLimitsBridge()
    with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
        assert bridge.calls == [("set", 7, 8192, 500)]
    assert bridge.calls == [("set", 7, 8192, 500), ("restore", 7)]


def test_homing_limits_guard_restores_on_error():
    bridge = FakeLimitsBridge()
    try:
        with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
            raise RuntimeError("trip move failed")
    except RuntimeError:
        pass
    assert bridge.calls[-1] == ("restore", 7)


def test_homing_limits_guard_noop_without_limits():
    bridge = FakeLimitsBridge()
    with homing_mod._servo_drive_limits(bridge, None, None):
        pass
    assert bridge.calls == []


class FailingRestoreBridge(FakeLimitsBridge):
    def restore_drive_limits(self, handle):
        raise OSError("endpoint gone")


def test_homing_limits_guard_restore_failure_raises_on_success_path():
    bridge = FailingRestoreBridge()
    with pytest.raises(OSError, match="endpoint gone"):
        with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
            pass


def test_homing_limits_guard_restore_failure_does_not_mask_body_error():
    bridge = FailingRestoreBridge()
    with pytest.raises(RuntimeError, match="trip move failed"):
        with homing_mod._servo_drive_limits(bridge, 7, (8192, 500)):
            raise RuntimeError("trip move failed")


class FakeGcmd:
    error = RuntimeError


class FakeFaultBridge:
    def __init__(self, fault):
        self._fault = fault
        self.taken = []

    def take_drive_fault(self, handle):
        self.taken.append(handle)
        return self._fault


def test_post_trip_fault_check_raises_on_fault():
    bridge = FakeFaultBridge(0x8611)
    with pytest.raises(RuntimeError, match="drive fault 0x8611"):
        homing_mod._check_servo_drive_fault(FakeGcmd(), bridge, 0, 7)
    assert bridge.taken == [7]


def test_post_trip_fault_check_passes_without_fault():
    bridge = FakeFaultBridge(None)
    homing_mod._check_servo_drive_fault(FakeGcmd(), bridge, 0, 7)
    assert bridge.taken == [7]


def test_post_trip_fault_check_skips_non_servo():
    bridge = FakeFaultBridge(0x8611)
    homing_mod._check_servo_drive_fault(FakeGcmd(), bridge, 0, None)
    assert bridge.taken == []


class FakeServoBridge(FakeLimitsBridge):
    def __init__(self, fault=None):
        super().__init__()
        self._fault = fault

    def take_drive_fault(self, handle):
        self.calls.append(("take_fault", handle))
        return self._fault


def run_guarded_trip(bridge, se, servo_handle, servo_limits, trip):
    rail = FakeRail([], "servo_x")
    return homing_mod._run_servo_guarded_trip(
        FakeGcmd(), bridge, 0, se, rail, servo_handle, servo_limits, trip
    )


def test_guarded_trip_failure_disables_servo_motor_and_reraises():
    bridge = FakeServoBridge()
    se = FakeStepperEnable()

    def trip():
        raise RuntimeError("trip move failed")

    with pytest.raises(RuntimeError, match="trip move failed"):
        run_guarded_trip(bridge, se, 7, (8192, 500), trip)
    assert se.calls == [("servo_x", False)]


def test_guarded_trip_latched_fault_disables_servo_motor():
    bridge = FakeServoBridge(fault=0x8611)
    se = FakeStepperEnable()
    with pytest.raises(RuntimeError, match="drive fault 0x8611"):
        run_guarded_trip(bridge, se, 7, (8192, 500), lambda: (1.0, 2.0))
    assert se.calls == [("servo_x", False)]


def test_guarded_trip_success_keeps_servo_motor_enabled():
    bridge = FakeServoBridge()
    se = FakeStepperEnable()
    result = run_guarded_trip(bridge, se, 7, (8192, 500), lambda: (1.0, 2.0))
    assert result == (1.0, 2.0)
    assert se.calls == []


def test_guarded_trip_stepper_rail_failure_skips_servo_disable():
    bridge = FakeServoBridge()
    se = FakeStepperEnable()

    def trip():
        raise RuntimeError("trip move failed")

    with pytest.raises(RuntimeError, match="trip move failed"):
        run_guarded_trip(bridge, se, None, None, trip)
    assert se.calls == []

# Unit tests for FanFloorRegistry (klippy/extras/fan.py).
#
# These tests cover the pure Python speed-combine logic that backs
# `[heater_fan]` delegate mode. They deliberately avoid instantiating
# `fan.Fan` because that requires MCU setup.
import pytest

from klippy.extras.fan import FanFloorRegistry


def test_no_floors_returns_user_speed():
    r = FanFloorRegistry()
    assert r.set_user_speed(0.0) == 0.0
    assert r.set_user_speed(0.5) == 0.5
    assert r.set_user_speed(1.0) == 1.0


def test_single_floor_below_user_speed():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.set_user_speed(0.8)
    assert r.update_floor("hotend", 0.4) == 0.8


def test_single_floor_above_user_speed():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.set_user_speed(0.2)
    assert r.update_floor("hotend", 0.4) == 0.4


def test_m107_while_floor_active():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.update_floor("hotend", 0.4)
    assert r.set_user_speed(0.0) == 0.4


def test_floor_drops_back_to_user_speed():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.set_user_speed(0.0)
    r.update_floor("hotend", 0.4)
    assert r.update_floor("hotend", 0.0) == 0.0


def test_multiple_floors_take_max():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.register_floor("bed")
    r.set_user_speed(0.1)
    r.update_floor("hotend", 0.4)
    assert r.update_floor("bed", 0.3) == 0.4
    assert r.update_floor("bed", 0.5) == 0.5
    assert r.update_floor("hotend", 0.0) == 0.5
    assert r.update_floor("bed", 0.0) == 0.1


def test_duplicate_register_raises():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    with pytest.raises(ValueError):
        r.register_floor("hotend")


def test_update_unknown_floor_raises():
    r = FanFloorRegistry()
    with pytest.raises(KeyError):
        r.update_floor("hotend", 0.4)


def test_set_user_speed_returns_effective_with_existing_floor():
    r = FanFloorRegistry()
    r.register_floor("hotend")
    r.update_floor("hotend", 0.4)
    # User bumps above floor
    assert r.set_user_speed(0.9) == 0.9
    # User drops below floor
    assert r.set_user_speed(0.1) == 0.4

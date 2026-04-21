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


class _StubGCRQ:
    def __init__(self):
        self.async_calls = []
        self.gcode_calls = []

    def send_async_request(self, value, print_time=None):
        self.async_calls.append((value, print_time))

    def queue_gcode_request(self, value):
        self.gcode_calls.append(value)


def _make_fan_with_stub():
    # Bypass Fan.__init__ — it needs printer/MCU plumbing we don't have.
    # Instead construct a bare object with just the attributes the
    # speed-dispatch methods touch, then bind the methods off the real
    # class.
    from klippy.extras.fan import Fan, FanFloorRegistry

    stub = _StubGCRQ()

    class _FanLike:
        pass

    f = _FanLike()
    f._floor_registry = FanFloorRegistry()
    f.gcrq = stub
    # Bind the real methods unchanged
    f.set_speed = Fan.set_speed.__get__(f, _FanLike)
    f.set_speed_from_command = Fan.set_speed_from_command.__get__(f, _FanLike)
    f.register_floor = Fan.register_floor.__get__(f, _FanLike)
    f.update_floor = Fan.update_floor.__get__(f, _FanLike)
    return f, stub


def test_fan_set_speed_dispatches_effective_async():
    f, stub = _make_fan_with_stub()
    f.register_floor("hotend")
    f.update_floor("hotend", 0.4)
    assert stub.async_calls[-1] == (0.4, None)

    f.set_speed(0.2)
    # user 0.2 < floor 0.4 => effective 0.4
    assert stub.async_calls[-1] == (0.4, None)

    f.set_speed(0.9, print_time=12.5)
    assert stub.async_calls[-1] == (0.9, 12.5)


def test_fan_set_speed_from_command_dispatches_effective_gcode():
    f, stub = _make_fan_with_stub()
    f.register_floor("hotend")
    f.update_floor("hotend", 0.3)

    f.set_speed_from_command(0.1)
    assert stub.gcode_calls[-1] == 0.3

    f.set_speed_from_command(0.8)
    assert stub.gcode_calls[-1] == 0.8

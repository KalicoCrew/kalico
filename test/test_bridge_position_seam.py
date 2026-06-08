# Tests for the 5 host seam methods repointed at the Rust eval/kinematics:
#   stepper.get_mcu_position / get_past_mcu_position
#   BridgeKinematics.calc_position / set_position
#   MotionToolhead._fire_active_callbacks
#
# motion_bridge.py imports the native .so at load; inject a stand-in whose
# MotionBridge records calls and returns scripted values so the host arithmetic
# (mm + offset)/step_dist, rail averaging, and slot filtering can be asserted.
import sys
import types

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")


class _FakeNativeBridge:
    def __init__(self):
        self.calls = []
        self.now_mm = {}
        self.at_clock_mm = {}
        self.inverse_xy = [0.0, 0.0]
        self.forward_slots = []
        self.delta_slots = []

    def set_position(self, x, y, z):
        self.calls.append(("set_position", x, y, z))

    def eval_motor_position_now(self, mcu, oid):
        self.calls.append(("now", mcu, oid))
        return self.now_mm[(mcu, oid)]

    def eval_motor_position_at_clock(self, mcu, oid, clock):
        self.calls.append(("at_clock", mcu, oid, clock))
        return self.at_clock_mm[(mcu, oid, clock)]

    def motor_positions_to_toolhead(self, mcu, a, b):
        self.calls.append(("inverse", mcu, a, b))
        return list(self.inverse_xy)

    def forward_motor_positions(self, mcu, x, y, z):
        self.calls.append(("forward", mcu, x, y, z))
        return list(self.forward_slots)

    def toolhead_delta_to_motor_slots(self, mcu, dx, dy, dz):
        self.calls.append(("delta", mcu, dx, dy, dz))
        return list(self.delta_slots)


_fake_native_mod.MotionBridge = _FakeNativeBridge
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy import motion_toolhead as mth  # noqa: E402
from klippy import stepper as stepper_mod  # noqa: E402
from klippy.motion_bridge import MotionBridgeWrapper  # noqa: E402


def _wrapper_around(native):
    w = MotionBridgeWrapper.__new__(MotionBridgeWrapper)
    w._bridge = native
    return w


class _FakeMcu:
    def __init__(self, wrapper, bridge_handle=7):
        self._motion_bridge = wrapper
        self._bridge_handle = bridge_handle
        self._clock = {}

    def print_time_to_clock(self, print_time):
        return self._clock[print_time]


def _make_stepper(mcu, oid, step_dist, offset=0.0):
    s = stepper_mod.MCU_stepper.__new__(stepper_mod.MCU_stepper)
    s._mcu = mcu
    s._oid = oid
    s._step_dist = step_dist
    s._mcu_position_offset = offset
    s._name = "stepper_%d" % oid
    return s


# ---------------------------------------------------------------- get_mcu_position


def test_get_mcu_position_now_evals_and_rounds():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)
    s = _make_stepper(mcu, oid=11, step_dist=0.01, offset=0.5)
    native.now_mm[(3, 11)] = 1.234

    pos = s.get_mcu_position()

    assert native.calls == [("now", 3, 11)]
    # (1.234 + 0.5) / 0.01 = 173.4 -> 173
    assert pos == 173


def test_get_mcu_position_negative_rounds_toward_zero_half():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)
    s = _make_stepper(mcu, oid=12, step_dist=0.01, offset=0.0)
    native.now_mm[(3, 12)] = -1.236

    pos = s.get_mcu_position()
    # -123.6 -> -124
    assert pos == -124


def test_get_mcu_position_with_cmd_pos_skips_rust():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper)
    s = _make_stepper(mcu, oid=13, step_dist=0.01, offset=0.25)

    pos = s.get_mcu_position(cmd_pos=1.0)

    assert native.calls == []
    # (1.0 + 0.25) / 0.01 = 125
    assert pos == 125


def test_get_past_mcu_position_uses_clock_and_at_clock_eval():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=5)
    mcu._clock[2.0] = 999
    s = _make_stepper(mcu, oid=21, step_dist=0.02, offset=1.0)
    native.at_clock_mm[(5, 21, 999)] = 3.0

    pos = s.get_past_mcu_position(2.0)

    assert native.calls == [("at_clock", 5, 21, 999)]
    # (3.0 + 1.0) / 0.02 = 200
    assert pos == 200


# ---------------------------------------------------------------- kinematics fakes


class _FakeRail:
    def __init__(self, steppers):
        self._steppers = steppers
        self._range = (0.0, 100.0)

    def get_steppers(self):
        return list(self._steppers)

    def get_range(self):
        return self._range


class _FakeToolhead:
    def __init__(self, bridge):
        self.bridge = bridge


def _make_kin(rails_by_axis, bridge):
    k = mth.BridgeKinematics.__new__(mth.BridgeKinematics)
    k._toolhead = _FakeToolhead(bridge)
    k.limits = [(1.0, -1.0)] * 3
    k._rails_by_axis = rails_by_axis
    k._axis_rails = lambda: dict(rails_by_axis)
    return k


# ---------------------------------------------------------------- calc_position


def test_calc_position_averages_rails_and_calls_inverse_with_mm():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=9)
    sx = _make_stepper(mcu, oid=1, step_dist=0.01)
    sy = _make_stepper(mcu, oid=2, step_dist=0.01)
    sz = _make_stepper(mcu, oid=3, step_dist=0.01)
    x_rail = _FakeRail([sx])
    y_rail = _FakeRail([sy])
    z_rail = _FakeRail([sz])
    kin = _make_kin({0: x_rail, 1: y_rail, 2: z_rail}, wrapper)
    native.inverse_xy = [12.5, -4.0]

    # values are already MM (homing.py converts before calc_position)
    out = kin.calc_position(
        {"stepper_1": 10.0, "stepper_2": 6.0, "stepper_3": 2.5}
    )

    assert native.calls == [("inverse", 9, 10.0, 6.0)]
    assert out == [12.5, -4.0, 2.5]


def test_calc_position_rail_pos_is_mean_no_double_stepdist():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=9)
    # X rail has two steppers; rail_pos must be their mean, in mm, unscaled.
    sx0 = _make_stepper(mcu, oid=1, step_dist=0.005)
    sx1 = _make_stepper(mcu, oid=2, step_dist=0.005)
    x_rail = _FakeRail([sx0, sx1])
    kin = _make_kin({0: x_rail}, wrapper)
    native.inverse_xy = [0.0, 0.0]

    kin.calc_position({"stepper_1": 4.0, "stepper_2": 8.0})

    # mean(4,8)=6 passed straight through; NOT divided by step_dist.
    assert native.calls == [("inverse", 9, 6.0, 0.0)]


# ---------------------------------------------------------------- set_position


def test_set_position_grounds_steppers_from_forward_kinematics():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=4)
    sx = _make_stepper(mcu, oid=1, step_dist=0.01)
    sy = _make_stepper(mcu, oid=2, step_dist=0.02)
    x_rail = _FakeRail([sx])
    y_rail = _FakeRail([sy])
    kin = _make_kin({0: x_rail, 1: y_rail}, wrapper)
    native.forward_slots = [(0, 5.0), (1, 3.0)]

    captured = {}

    def cap(stepper, val):
        captured[stepper] = val

    sx._set_mcu_position = lambda v: cap(sx, v)
    sy._set_mcu_position = lambda v: cap(sy, v)

    kin.set_position([1.0, 2.0, 0.0], homing_axes=(0,))

    assert ("set_position", 1.0, 2.0, 0.0) in native.calls
    assert ("forward", 4, 1.0, 2.0, 0.0) in native.calls
    # 5.0 / 0.01 = 500 ; 3.0 / 0.02 = 150
    assert captured[sx] == 500
    assert captured[sy] == 150
    assert kin.limits[0] == (0.0, 100.0)


def test_set_position_ignores_slot_without_rail():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=4)
    sx = _make_stepper(mcu, oid=1, step_dist=0.01)
    x_rail = _FakeRail([sx])
    kin = _make_kin({0: x_rail}, wrapper)
    native.forward_slots = [(0, 5.0), (2, 9.0)]  # slot 2 has no rail

    captured = []
    sx._set_mcu_position = lambda v: captured.append(v)

    kin.set_position([1.0, 0.0, 0.0])

    assert captured == [500]


# ---------------------------------------------------------------- _fire_active_callbacks


def _make_toolhead(kin, bridge):
    th = mth.MotionToolhead.__new__(mth.MotionToolhead)
    th.kin = kin
    th.bridge = bridge
    th._last_move_time = 1.0
    th.get_last_move_time = lambda: 1.0
    return th


def test_fire_active_callbacks_fires_only_moved_slots():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=8)
    sx = _make_stepper(mcu, oid=1, step_dist=0.01)
    sy = _make_stepper(mcu, oid=2, step_dist=0.01)
    sx._name = "stepper_x"
    sy._name = "stepper_y"
    x_rail = _FakeRail([sx])
    y_rail = _FakeRail([sy])
    kin = _make_kin({0: x_rail, 1: y_rail}, wrapper)
    kin.rails = [x_rail, y_rail]
    kin.get_steppers = lambda: [sx, sy]

    fired = []
    y_cb = lambda pt: fired.append(("y", pt))  # noqa: E731
    sx._active_callbacks = [lambda pt: fired.append(("x", pt))]
    sy._active_callbacks = [y_cb]

    th = _make_toolhead(kin, wrapper)
    # only motor slot 0 moved
    native.delta_slots = [(0, 1.0)]

    result = th._fire_active_callbacks(1.0, 0.0, 0.0, 0.0, print_time=2.0)

    assert ("delta", 8, 1.0, 0.0, 0.0) in native.calls
    assert result is True
    assert fired == [("x", 2.0)]
    assert sx._active_callbacks == []
    assert sy._active_callbacks == [y_cb]  # y untouched


def test_fire_active_callbacks_early_out_when_nothing_moved():
    native = _FakeNativeBridge()
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=8)
    sx = _make_stepper(mcu, oid=1, step_dist=0.01)
    sx._name = "stepper_x"
    x_rail = _FakeRail([sx])
    kin = _make_kin({0: x_rail}, wrapper)
    kin.rails = [x_rail]
    kin.get_steppers = lambda: [sx]
    sx._active_callbacks = [lambda pt: None]

    th = _make_toolhead(kin, wrapper)
    native.delta_slots = []

    result = th._fire_active_callbacks(0.0, 0.0, 0.0, 0.0)

    assert result is False
    # callbacks left intact, not fired
    assert len(sx._active_callbacks) == 1

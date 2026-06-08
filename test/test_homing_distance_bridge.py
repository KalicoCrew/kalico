# Homing-distance correctness across the faithful position-at-time seam.
#
# Pins three invariants of the new eval-based homing path:
#   1. distance_elapsed = inverse(motor delta) is the true toolhead distance.
#   2. moved_less_than_dist gates the second approach on that distance.
#   3. Under trip-truncation, eval_now == eval_at_clock(trip) so halt == trig
#      and (halt - start) is identical from either read.
#
# The real classes under test are homing.StepperPosition,
# homing.HomingMove.moved_less_than_dist, and
# motion_toolhead.BridgeKinematics.calc_position. The Rust .so is replaced by a
# scripted fake native bridge (same pattern as test_bridge_position_seam.py and
# test_motion_bridge_trip_routing.py) so we drive a known move + trip without
# hardware, with the host arithmetic and the decision predicate fully real.
import sys
import types

_fake_native_mod = types.ModuleType("klippy.motion_bridge_native")


class _FakeNativeBridge:
    def __init__(self):
        self.now_mm = {}
        self.at_clock_mm = {}
        self.inverse_fn = None

    def set_position(self, x, y, z):
        pass

    def eval_motor_position_now(self, mcu, oid):
        return self.now_mm[(mcu, oid)]

    def eval_motor_position_at_clock(self, mcu, oid, clock):
        return self.at_clock_mm[(mcu, oid, clock)]

    def motor_positions_to_toolhead(self, mcu, a, b):
        return list(self.inverse_fn(a, b))


_fake_native_mod.MotionBridge = _FakeNativeBridge
sys.modules.setdefault("klippy.motion_bridge_native", _fake_native_mod)

from klippy import motion_toolhead as mth  # noqa: E402
from klippy import stepper as stepper_mod  # noqa: E402
from klippy.extras import danger_options as danger_mod  # noqa: E402
from klippy.extras import homing  # noqa: E402
from klippy.motion_bridge import MotionBridgeWrapper  # noqa: E402


class _DangerStub:
    # moved_less_than_dist reads only this one field.
    homing_elapsed_distance_tolerance = 0.5


danger_mod.DANGER_OPTIONS = _DangerStub()


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


def _make_stepper(mcu, oid, name, step_dist, offset=0.0):
    s = stepper_mod.MCU_stepper.__new__(stepper_mod.MCU_stepper)
    s._mcu = mcu
    s._oid = oid
    s._step_dist = step_dist
    s._mcu_position_offset = offset
    s._name = name
    return s


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
    k.rails = list(rails_by_axis.values())
    k._axis_rails = lambda: dict(rails_by_axis)
    return k


# CoreXY: motor A = X + Y, motor B = X - Y.
# inverse: X = (A + B) / 2, Y = (A - B) / 2.
def _corexy_inverse(a, b):
    return [(a + b) / 2.0, (a - b) / 2.0]


class _FakeEndstop:
    def __init__(self, steppers):
        self._steppers = steppers

    def get_steppers(self):
        return list(self._steppers)


def _make_homing_move(kin):
    # HomingMove.moved_less_than_dist only reads self.distance_elapsed; bypass
    # __init__ (which would resolve a real toolhead) and exercise the predicate.
    hm = homing.HomingMove.__new__(homing.HomingMove)
    hm.distance_elapsed = []
    return hm


# ------------------------------------------------------ 1. distance_elapsed


def test_distance_elapsed_is_corexy_inverse_of_motor_delta():
    # CoreXY, step_dist = 0.01 on both motors.
    # start motor mm: A=10.0, B=10.0   -> toolhead (10, 0)
    # trip  motor mm: A=14.0, B=6.0    -> toolhead (10, 4)
    # deltas (mm):    dA=+4.0, dB=-4.0
    # steps_moved passes mm deltas into calc_position; inverse of (4, -4):
    #   X = (4 + -4)/2 = 0.0
    #   Y = (4 - -4)/2 = 4.0
    # So the toolhead moved 0 in X and 4 mm in Y.
    native = _FakeNativeBridge()
    native.inverse_fn = _corexy_inverse
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)

    sa = _make_stepper(mcu, oid=1, name="stepper_x", step_dist=0.01)
    sb = _make_stepper(mcu, oid=2, name="stepper_y", step_dist=0.01)
    a_rail = _FakeRail([sa])
    b_rail = _FakeRail([sb])
    kin = _make_kin({0: a_rail, 1: b_rail}, wrapper)

    # Capture start at construction (eval_now must return S here).
    native.now_mm[(3, 1)] = 10.0
    native.now_mm[(3, 2)] = 10.0
    spa = homing.StepperPosition(sa, "endstop_a")
    spb = homing.StepperPosition(sb, "endstop_b")
    assert spa.start_pos == 1000  # 10.0 / 0.01
    assert spb.start_pos == 1000

    # Trip-truncated curve: now and at_clock both read the trip position T.
    trigger_time = 2.0
    mcu._clock[trigger_time] = 555
    native.now_mm[(3, 1)] = 14.0
    native.now_mm[(3, 2)] = 6.0
    native.at_clock_mm[(3, 1, 555)] = 14.0
    native.at_clock_mm[(3, 2, 555)] = 6.0

    spa.note_home_end(trigger_time)
    spb.note_home_end(trigger_time)
    assert spa.halt_pos == 1400  # 14.0 / 0.01
    assert spb.halt_pos == 600  # 6.0 / 0.01

    steps_moved = {
        spa.stepper_name: (spa.halt_pos - spa.start_pos) * sa.get_step_dist(),
        spb.stepper_name: (spb.halt_pos - spb.start_pos) * sb.get_step_dist(),
    }
    # (1400 - 1000) * 0.01 = 4.0 ; (600 - 1000) * 0.01 = -4.0
    assert steps_moved == {"stepper_x": 4.0, "stepper_y": -4.0}

    distance_elapsed = kin.calc_position(steps_moved)
    # inverse(4, -4) = (0, 4); z rail absent -> 0
    assert distance_elapsed == [0.0, 4.0, 0.0]


# ------------------------------------------------------ 2. min_home_dist gate


def _distance_elapsed_for_motor_delta(da_mm, db_mm):
    # Drive the real StepperPosition + calc_position for a CoreXY motor delta,
    # returning distance_elapsed (toolhead frame).
    native = _FakeNativeBridge()
    native.inverse_fn = _corexy_inverse
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)
    sa = _make_stepper(mcu, oid=1, name="stepper_x", step_dist=0.01)
    sb = _make_stepper(mcu, oid=2, name="stepper_y", step_dist=0.01)
    kin = _make_kin({0: _FakeRail([sa]), 1: _FakeRail([sb])}, wrapper)

    native.now_mm[(3, 1)] = 0.0
    native.now_mm[(3, 2)] = 0.0
    spa = homing.StepperPosition(sa, "ea")
    spb = homing.StepperPosition(sb, "eb")

    tt = 2.0
    mcu._clock[tt] = 555
    native.now_mm[(3, 1)] = da_mm
    native.now_mm[(3, 2)] = db_mm
    native.at_clock_mm[(3, 1, 555)] = da_mm
    native.at_clock_mm[(3, 2, 555)] = db_mm
    spa.note_home_end(tt)
    spb.note_home_end(tt)

    steps_moved = {
        spa.stepper_name: (spa.halt_pos - spa.start_pos) * sa.get_step_dist(),
        spb.stepper_name: (spb.halt_pos - spb.start_pos) * sb.get_step_dist(),
    }
    return kin.calc_position(steps_moved)


def test_short_trip_arms_second_approach():
    # Y-axis homing. min_home_dist = 2.0 mm.
    # motor delta dA=+0.5, dB=-0.5 -> inverse Y = (0.5 - -0.5)/2 = 0.5 mm < 2.0.
    de = _distance_elapsed_for_motor_delta(0.5, -0.5)
    assert de == [0.0, 0.5, 0.0]

    hm = _make_homing_move(None)
    hm.distance_elapsed = de
    # homing axis = Y (index 1); 0.5 < 2.0 -> rehome.
    assert hm.moved_less_than_dist(2.0, [1]) is True


def test_sufficient_trip_does_not_arm_second_approach():
    # motor delta dA=+5.0, dB=-5.0 -> inverse Y = 5.0 mm >= 2.0.
    de = _distance_elapsed_for_motor_delta(5.0, -5.0)
    assert de == [0.0, 5.0, 0.0]

    hm = _make_homing_move(None)
    hm.distance_elapsed = de
    assert hm.moved_less_than_dist(2.0, [1]) is False


# ------------------------------------------------------ 3. halt == trig


def test_halt_equals_trig_under_trip_truncation():
    # The retained curve is truncated AT the trip, so eval_now (endpoint) and
    # eval_at_clock(trip_clock) read the same motor mm. halt_pos and trig_pos
    # must therefore be equal, making (halt - start) == (trig - start).
    native = _FakeNativeBridge()
    native.inverse_fn = _corexy_inverse
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)
    s = _make_stepper(mcu, oid=1, name="stepper_x", step_dist=0.01)

    native.now_mm[(3, 1)] = 0.0
    sp = homing.StepperPosition(s, "ea")
    assert sp.start_pos == 0

    tt = 2.0
    mcu._clock[tt] = 555
    trip_mm = 7.0
    native.now_mm[(3, 1)] = trip_mm
    native.at_clock_mm[(3, 1, 555)] = trip_mm

    sp.note_home_end(tt)
    assert sp.halt_pos == sp.trig_pos == 700
    assert (sp.halt_pos - sp.start_pos) == (sp.trig_pos - sp.start_pos) == 700


def test_disagreeing_now_vs_at_clock_would_be_caught():
    # NEGATIVE / regression guard: the OLD broken behavior had get_mcu_position
    # (now) and get_past_mcu_position (at_clock) backed by two different
    # functions whose values could diverge — that is the bug class (distance
    # always 0, halt != trig). Here we deliberately script them to disagree and
    # show StepperPosition surfaces the inequality, so a future regression that
    # re-splits the seam fails this assertion.
    native = _FakeNativeBridge()
    native.inverse_fn = _corexy_inverse
    wrapper = _wrapper_around(native)
    mcu = _FakeMcu(wrapper, bridge_handle=3)
    s = _make_stepper(mcu, oid=1, name="stepper_x", step_dist=0.01)

    native.now_mm[(3, 1)] = 0.0
    sp = homing.StepperPosition(s, "ea")

    tt = 2.0
    mcu._clock[tt] = 555
    native.now_mm[(3, 1)] = 9.0  # halt reads 900 steps
    native.at_clock_mm[(3, 1, 555)] = 3.0  # trig reads 300 steps (divergent)

    sp.note_home_end(tt)
    assert sp.halt_pos == 900
    assert sp.trig_pos == 300
    # Inequality here == the bug class. The truncated-curve seam guarantees they
    # match (see test_halt_equals_trig_under_trip_truncation); this test pins
    # that StepperPosition does NOT mask a divergence if one ever reappears.
    assert sp.halt_pos != sp.trig_pos

from klippy.motion_toolhead import BridgeKinematics, MotionToolhead


class FakeStepper:
    def __init__(self, name):
        self._name = name
        self._active_callbacks = []

    def add_active_callback(self, cb):
        self._active_callbacks.append(cb)

    def get_name(self, short=False):
        return self._name


class FakeRail:
    def __init__(self, name, steppers):
        self._name = name
        self._steppers = steppers

    def get_name(self, short=False):
        return self._name if short else "stepper_" + self._name

    def get_steppers(self):
        return self._steppers


class FakeKin:
    active_rails = BridgeKinematics.active_rails

    def __init__(self, kinematics, rails):
        self.kinematics = kinematics
        self.rails = rails

    def get_steppers(self):
        return [s for rail in self.rails for s in rail.get_steppers()]


def make_kin(kinematics):
    rails = [
        FakeRail("x", [FakeStepper("stepper_x"), FakeStepper("stepper_x1")]),
        FakeRail("y", [FakeStepper("stepper_y"), FakeStepper("stepper_y1")]),
        FakeRail("z", [FakeStepper("stepper_z"), FakeStepper("stepper_z1")]),
    ]
    return FakeKin(kinematics, rails)


def rail_names(rails):
    return [r.get_name(short=True) for r in rails]


def test_corexy_x_move_couples_both_gantry_rails_not_z():
    kin = make_kin("corexy")
    assert rail_names(kin.active_rails(5.0, 0.0, 0.0)) == ["x", "y"]
    assert rail_names(kin.active_rails(0.0, 5.0, 0.0)) == ["x", "y"]
    assert rail_names(kin.active_rails(0.0, 0.0, 5.0)) == ["z"]
    assert rail_names(kin.active_rails(0.0, 0.0, 0.0)) == []


def test_cartesian_rails_are_independent():
    kin = make_kin("cartesian")
    assert rail_names(kin.active_rails(5.0, 0.0, 0.0)) == ["x"]
    assert rail_names(kin.active_rails(0.0, 5.0, 0.0)) == ["y"]
    assert rail_names(kin.active_rails(0.0, 0.0, 5.0)) == ["z"]


def test_hybrid_corexy_y_move_couples_x_motor():
    kin = make_kin("hybrid_corexy")
    assert rail_names(kin.active_rails(5.0, 0.0, 0.0)) == ["x"]
    assert rail_names(kin.active_rails(0.0, 5.0, 0.0)) == ["x", "y"]
    assert rail_names(kin.active_rails(0.0, 0.0, 5.0)) == ["z"]


class FakeToolhead:
    _fire_active_callbacks = MotionToolhead._fire_active_callbacks

    def __init__(self, kin):
        self.kin = kin
        self._clock = 100.0

    def get_last_move_time(self):
        self._clock += 0.090
        return self._clock


def test_each_enable_callback_gets_fresh_print_time():
    kin = make_kin("corexy")
    fired = []
    for rail in kin.rails:
        for s in rail.get_steppers():
            s.add_active_callback(fired.append)
    th = FakeToolhead(kin)
    th._fire_active_callbacks()
    assert len(fired) == 6
    assert len(set(fired)) == 6, "print_time must be recomputed per callback"
    assert fired == sorted(fired)

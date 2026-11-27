import pytest

from klippy import toolhead
from klippy.kinematics import (
    cartesian,
    corexz,
    limited_cartesian,
    limited_corexz,
)

CONFIG = {
    "max_x_velocity": 300.0,
    "max_y_velocity": 250.0,
    "max_z_velocity": 25.0,
    "max_x_accel": 1200.0,
    "max_y_accel": 900.0,
    "max_z_accel": 30.0,
}


class _FakeConfig:
    class _Printer:
        class _GCode:
            def register_command(self, *_, **__):
                pass

        def lookup_object(self, name):
            return self._GCode()

    def getfloat(self, name, default=None, **_):
        return CONFIG.get(name, default)

    def getboolean(self, name, default=None):
        return default

    def get_printer(self):
        return self._Printer()


class _FakeGCmd:
    def __init__(self, **params):
        self.params = params

    def get_float(self, name, default=None, **_):
        return float(self.params[name]) if name in self.params else default

    def respond_info(self, *_, **__):
        pass


class _FakeToolhead:
    def __init__(self, kin):
        self.kin = kin
        self.orig_cfg = {
            name: getattr(kin, name) for name in CONFIG if hasattr(kin, name)
        }
        self.orig_cfg.update(
            max_velocity=500.0,
            max_accel=1500.0,
            square_corner_velocity=5.0,
            min_cruise_ratio=0.5,
        )

    def get_kinematics(self):
        return self.kin

    def get_max_velocity(self):
        return 500.0, 1500.0

    def set_max_velocities(self, *_):
        return 500.0, 1500.0, 5.0, 0.5


class _FakeMove:
    is_kinematic_move = True

    def __init__(self, axes_r):
        self.axes_r = axes_r
        self.limits = None

    def limit_speed(self, max_v, max_a):
        self.limits = (max_v, max_a)


def _base_init(self, toolhead, config):
    self.max_z_velocity = config.getfloat("max_z_velocity")
    self.max_z_accel = config.getfloat("max_z_accel")


@pytest.fixture(
    params=[
        (limited_cartesian.LimitedCartKinematics, cartesian.CartKinematics),
        (limited_corexz.LimitedCoreXZKinematics, corexz.CoreXZKinematics),
    ],
    ids=["limited_cartesian", "limited_corexz"],
)
def kinematics(request, monkeypatch):
    kin_class, base_class = request.param
    monkeypatch.setattr(
        toolhead,
        "get_danger_options",
        lambda: type("_Options", (), {"log_velocity_limit_changes": False}),
    )
    monkeypatch.setattr(base_class, "__init__", _base_init)
    monkeypatch.setattr(
        base_class, "_check_endstops", lambda self, move: None, raising=False
    )
    return kin_class(_FakeToolhead(None), _FakeConfig())


def _limits(kin, axes_r):
    move = _FakeMove(axes_r)
    kin.check_move(move)
    return move.limits


def test_set_velocity_limit_changes_enforced_limits(kinematics):
    helper = toolhead.ToolHeadCommandHelper.__new__(
        toolhead.ToolHeadCommandHelper
    )
    helper.toolhead = _FakeToolhead(kinematics)
    assert _limits(kinematics, (0.0, 0.0, 1.0)) == (25.0, 30.0)
    assert _limits(kinematics, (1.0, 0.0, 0.0)) == (300.0, 1200.0)

    helper.cmd_SET_VELOCITY_LIMIT(
        _FakeGCmd(Z_VELOCITY=10, Z_ACCEL=15, X_VELOCITY=200, X_ACCEL=1100)
    )
    assert _limits(kinematics, (0.0, 0.0, 1.0)) == (10.0, 15.0)
    assert _limits(kinematics, (1.0, 0.0, 0.0)) == (200.0, 1100.0)

    helper.cmd_RESET_VELOCITY_LIMIT(_FakeGCmd())
    assert _limits(kinematics, (0.0, 0.0, 1.0)) == (25.0, 30.0)
    assert _limits(kinematics, (1.0, 0.0, 0.0)) == (300.0, 1200.0)

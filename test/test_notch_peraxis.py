# Unit test for the per-axis notch: config resolution/validation
# (ToolHead._resolve_notch) and per-move target selection
# (ToolHead._move_notch_pair / _move_notch_freq / _notch_on). Imports the REAL
# toolhead module with runtime-only dependencies stubbed so it can run
# standalone.
#
# Naming two different modes selects a two-zero (trapezoidal) ramp that nulls
# BOTH on every axis, so the target no longer depends on heading -- see
# test_notch_dual.py for the law itself. This file checks that the toolhead
# resolves the config to the right pair, degenerates to the single-zero case
# when f_x == f_y, and gets the Z-coupling cases right.
import importlib
import math
import os
import sys
import types

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
KLIPPY = os.path.join(ROOT, "klippy")
STUB_MODULES = [
    "klippy",
    "klippy.chelper",
    "klippy.kinematics",
    "klippy.kinematics.extruder",
    "klippy.toolhead",
]
saved_modules = {name: sys.modules.get(name) for name in STUB_MODULES}
pkg = types.ModuleType("klippy")
pkg.__path__ = [KLIPPY]
sys.modules["klippy"] = pkg
sys.modules.setdefault("klippy.chelper", types.ModuleType("klippy.chelper"))
kin_pkg = types.ModuleType("klippy.kinematics")
kin_pkg.__path__ = [os.path.join(KLIPPY, "kinematics")]
sys.modules["klippy.kinematics"] = kin_pkg
extruder = types.ModuleType("klippy.kinematics.extruder")
extruder.DummyExtruder = object
extruder.add_printer_objects = lambda config: None
sys.modules["klippy.kinematics.extruder"] = extruder
toolhead = importlib.import_module("klippy.toolhead")
for name in STUB_MODULES:
    if saved_modules[name] is None:
        sys.modules.pop(name, None)
    else:
        sys.modules[name] = saved_modules[name]

TH = toolhead.ToolHead


def check(failures, name, got, want, tol=1e-9):
    ok = abs(got - want) <= tol
    print(
        "  %-52s %s (got %.4f want %.4f)"
        % (name, "OK" if ok else "FAIL", got, want)
    )
    if not ok:
        failures.append(name)


def check_true(failures, name, cond):
    print("  %-52s %s" % (name, "OK" if cond else "FAIL"))
    if not cond:
        failures.append(name)


class FakeMove:
    def __init__(self, rx, ry, rz=0.0):
        # axes_r is the move's unit direction (x, y, z, e).
        self.axes_r = [rx, ry, rz, 0.0]


class FakeStepper:
    def __init__(self, axes):
        self.axes = axes

    def is_active_axis(self, axis):
        return axis in self.axes


class FakeRail:
    def __init__(self, steppers):
        self.steppers = steppers

    def get_steppers(self):
        return self.steppers


# Real itersolve active-axis flags: a cartesian/CoreXY Z stepper is AF_Z only,
# while a CoreXZ stepper is AF_X|AF_Z and a delta tower is AF_X|AF_Y|AF_Z.
DECOUPLED_RAILS = [
    FakeRail([FakeStepper("xy")]),
    FakeRail([FakeStepper("xy")]),
    FakeRail([FakeStepper("z")]),
]
COREXZ_RAILS = [
    FakeRail([FakeStepper("xz")]),
    FakeRail([FakeStepper("y")]),
    FakeRail([FakeStepper("xz")]),
]


def mk(fx, fy, z_coupled=False, rails=DECOUPLED_RAILS):
    # Bare instance with the (already resolved) per-axis freqs set.
    o = object.__new__(TH)
    o.unified_notch_freq_x = fx
    o.unified_notch_freq_y = fy
    if z_coupled:
        rails = COREXZ_RAILS
    o.kin = types.SimpleNamespace(rails=rails) if rails is not None else None
    return o


def f(o, rx, ry, rz=0.0):
    return TH._move_notch_freq(o, FakeMove(rx, ry, rz))


def pair(o, rx, ry, rz=0.0):
    return TH._move_notch_pair(o, FakeMove(rx, ry, rz))


def f_eq(f_lo, f_hi):
    # The equivalent (harmonic-mean) frequency of a two-zero pair -- what
    # governs ramp distance and runway.
    return 2.0 / (1.0 / f_lo + 1.0 / f_hi)


def raises(shared, x, y):
    try:
        TH._resolve_notch(shared, x, y)
        return False
    except ValueError:
        return True


def run_checks():
    failures = []
    s2 = math.sqrt(0.5)  # 45-degree unit component

    print("== _resolve_notch: config resolution + validation ==")
    check_true(
        failures,
        "shared alone -> both axes",
        TH._resolve_notch(55.0, None, None) == (55.0, 55.0),
    )
    check_true(
        failures,
        "off (0, none, none) -> (0,0)",
        TH._resolve_notch(0.0, None, None) == (0.0, 0.0),
    )
    check_true(
        failures,
        "per-axis pair -> (x,y)",
        TH._resolve_notch(0.0, 70.0, 60.0) == (70.0, 60.0),
    )
    check_true(
        failures,
        "per-axis overrides shared",
        TH._resolve_notch(55.0, 70.0, 60.0) == (70.0, 60.0),
    )
    check_true(
        failures,
        "both equal pair ok",
        TH._resolve_notch(0.0, 55.0, 55.0) == (55.0, 55.0),
    )
    check_true(failures, "x without y -> ValueError", raises(55.0, 70.0, None))
    check_true(failures, "y without x -> ValueError", raises(0.0, None, 60.0))
    check_true(
        failures,
        "x set, y unset, no shared -> ValueError",
        raises(0.0, 70.0, None),
    )

    print("== off ==")
    o = mk(0.0, 0.0)
    check(failures, "both 0 -> 0", f(o, 1, 0), 0.0)
    check_true(failures, "_notch_on() false", not TH._notch_on(o))

    print(
        "== both equal (shared folded in) -> that freq for EVERY direction =="
    )
    o = mk(55.0, 55.0)
    check(failures, "pure-X", f(o, 1, 0), 55.0)
    check(failures, "pure-Y", f(o, 0, 1), 55.0)
    check(failures, "45deg ", f(o, s2, s2), 55.0)
    check(failures, "X+Z   ", f(o, 0.6, 0.0, 0.8), 55.0)
    check_true(failures, "_notch_on() true", TH._notch_on(o))

    print("== no XY motion, Z decoupled -> no notch (nothing to cancel) ==")
    # a_x = rx*a(t) and a_y = ry*a(t) with rx = ry = 0, so no a(t) excites the
    # XY modes. Shaping anyway costs the whole 2/f_n ramp and its runway for
    # zero benefit -- and a Z hop has nowhere near that runway.
    o = mk(55.0, 55.0)
    check(failures, "Z-only  -> off", f(o, 0, 0, 1), 0.0)
    check(failures, "E-only  -> off", f(o, 0, 0, 0), 0.0)
    o = mk(60.0, 55.0)
    check(failures, "Z-only per-axis -> off", f(o, 0, 0, 1), 0.0)
    # A Z move with ANY real XY component is still shaped: that component does
    # excite the modes. With two modes named it gets both zeros.
    check(
        failures, "tiny X + Z -> pair", f(o, 1e-6, 0.0, 1.0), f_eq(55.0, 60.0)
    )

    print("== no XY motion, Z COUPLED (CoreXZ/delta) -> keep shaping ==")
    # On CoreXZ (A = x + z, B = x - z), delta and polar, a Z-only move drives
    # the very belts the notch targets. Commanded X is zero only to first
    # order, so the shaping must stay.
    o = mk(55.0, 55.0, z_coupled=True)
    check(failures, "Z-only coupled  -> on", f(o, 0, 0, 1), 55.0)
    o = mk(60.0, 55.0, z_coupled=True)
    check(
        failures,
        "Z-only coupled per-axis",
        f(o, 0, 0, 1),
        f_eq(55.0, 60.0),
    )
    # Unknown kinematics must fall on the safe side: keep shaping.
    o = mk(55.0, 55.0, rails=None)
    check(failures, "unknown kin -> on", f(o, 0, 0, 1), 55.0)

    print("== per-axis: fx=60, fy=55 -> BOTH zeros, on every heading ==")
    # Naming two modes asks for a trapezoidal ramp rect(1/60) * rect(1/55),
    # whose spectrum nulls 55 AND 60. Both zeros land on both axes (a_i = r_i*a),
    # so unlike the old single-zero blend the target does not depend on heading.
    o = mk(60.0, 55.0)
    check_true(failures, "pure-X -> pair", pair(o, 1, 0) == (55.0, 60.0))
    check_true(failures, "pure-Y -> pair", pair(o, 0, 1) == (55.0, 60.0))
    check_true(failures, "45deg  -> pair", pair(o, s2, s2) == (55.0, 60.0))
    check_true(
        failures, "X+Z    -> pair", pair(o, 0.6, 0.0, 0.8) == (55.0, 60.0)
    )
    # The scalar the planner uses is the pair's harmonic mean: a ramp costs
    # 0.5*(v0+v1)*(1/f_lo + 1/f_hi) = (v0+v1)/f_eq, the single-notch formula.
    check(failures, "f_eq is the harmonic mean", f(o, 1, 0), f_eq(55.0, 60.0))

    print(
        "== the pair is heading-independent (this is what lets spans link) =="
    )
    o = mk(60.0, 55.0)
    vals = set()
    bad = None
    for deg in range(0, 360, 5):
        r = math.radians(deg)
        v = f(o, math.cos(r), math.sin(r))
        vals.add(v)
        if v < 55.0 - 1e-9 or v > 60.0 + 1e-9:
            bad = (deg, v)
    check_true(failures, "one target for all 72 headings", len(vals) == 1)
    check_true(failures, "f_eq inside [55,60]", bad is None)

    print("== f_x == f_y degenerates to the single-zero triangle ==")
    o = mk(55.0, 55.0)
    check_true(failures, "equal pair", pair(o, s2, s2) == (55.0, 55.0))
    # Exactly f, not 2/(1/f+1/f): span linking compares this between moves.
    check_true(failures, "f_eq is exactly f", f(o, s2, s2) == 55.0)
    return failures


def test_per_axis_notch_resolution():
    failures = run_checks()
    assert not failures, failures


def main():
    failures = run_checks()
    print("ALL PASS" if not failures else "FAILURES: %s" % failures)
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

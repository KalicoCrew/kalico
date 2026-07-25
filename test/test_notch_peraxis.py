# Unit test for the per-axis notch: config resolution/validation
# (ToolHead._resolve_notch) and the direction-weighted blend
# (ToolHead._move_notch_freq / _notch_on). Imports the REAL toolhead module
# with runtime-only dependencies stubbed so it can run standalone.
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


def mk(fx, fy):
    # Bare instance with the (already resolved) per-axis freqs set.
    o = object.__new__(TH)
    o.unified_notch_freq_x = fx
    o.unified_notch_freq_y = fy
    return o


def f(o, rx, ry, rz=0.0):
    return TH._move_notch_freq(o, FakeMove(rx, ry, rz))


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
    check(failures, "Z-only", f(o, 0, 0, 1), 55.0)
    check_true(failures, "_notch_on() true", TH._notch_on(o))

    print("== per-axis: fx=60, fy=55 ==")
    o = mk(60.0, 55.0)
    check(failures, "pure-X -> fx", f(o, 1, 0), 60.0)
    check(failures, "pure-Y -> fy", f(o, 0, 1), 55.0)
    check(failures, "45deg  -> mean", f(o, s2, s2), 57.5)
    check(
        failures,
        "X-dominant blend",
        f(o, 0.8, 0.6),
        (60 * 0.8 + 55 * 0.6) / 1.4,
    )
    check(failures, "X+Z pure-X-in-plane -> fx", f(o, 0.6, 0.0, 0.8), 60.0)

    print("== diagonal within [min,max] and monotonic toward pure-X ==")
    o = mk(60.0, 55.0)
    bad = None
    prev = None
    mono = True
    for deg in range(90, -1, -5):  # 90 (pure Y) -> 0 (pure X)
        r = math.radians(deg)
        v = f(o, math.cos(r), math.sin(r))
        if v < 55.0 - 1e-9 or v > 60.0 + 1e-9:
            bad = (deg, v)
        if prev is not None and v < prev - 1e-9:
            mono = False
        prev = v
    check_true(failures, "all angles in [55,60]", bad is None)
    check_true(failures, "f increases toward pure-X", mono)
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

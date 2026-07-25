# Standalone regression tests for unified-planner ToolHead integration.
#
# Run: klippy-env/bin/python test/test_toolhead_unified.py
import importlib
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


class FakePrinter:
    def command_error(self, msg):
        return RuntimeError(msg)


class FakeAxis:
    def __init__(self, name="A", segmented=False):
        self.name = name
        if segmented:
            self.process_move_segment = lambda *args: None

    def get_axis_gcode_id(self):
        return self.name


def make_toolhead_without_unified_fields():
    th = types.SimpleNamespace()
    th.max_accel = 3000.0
    th.max_velocity = 200.0
    th.junction_deviation = 0.01
    th.max_accel_to_decel = 1500.0
    th.extra_axes = []
    th._move_notch_freq = lambda move: 0.0
    th._uses_unified_reach = toolhead.ToolHead._uses_unified_reach.__get__(
        th, object
    )
    th._pathplan_cons = toolhead.ToolHead._pathplan_cons.__get__(th, object)
    th._move_reach_v2 = toolhead.ToolHead._move_reach_v2.__get__(th, object)
    return th


def test_subclass_without_unified_fields():
    th = make_toolhead_without_unified_fields()
    prev = toolhead.Move(th, [0.0, 0.0, 0.0, 0.0], [10.0, 0.0, 0.0, 0.0], 100.0)
    move = toolhead.Move(
        th, [10.0, 0.0, 0.0, 0.0], [10.0, 10.0, 0.0, 0.0], 100.0
    )
    move.calc_junction(prev)
    assert move.max_start_v2 >= 0.0
    reach = th._move_reach_v2(prev, prev.max_start_v2)
    assert reach == prev.max_start_v2 + prev.delta_v2
    print("  Move.calc_junction tolerates missing unified fields OK")


def test_unified_rejects_unsegmented_extra_axis():
    th = object.__new__(toolhead.ToolHead)
    th.unified_emit = True
    th.extra_axes = [FakeAxis("A")]
    th.printer = FakePrinter()
    try:
        th._check_unified_extra_axis_support()
    except RuntimeError as e:
        assert "process_move_segment" in str(e)
        assert "'A'" in str(e)
    else:
        raise AssertionError("unsupported extra axis was accepted")
    print("  unified planner rejects unsegmented extra axes OK")


class FakeGCmd:
    def error(self, msg):
        return RuntimeError(msg)


def make_unified_toolhead(**over):
    th = object.__new__(toolhead.ToolHead)
    th.max_accel = 100000.0
    th.max_velocity = 650.0
    th.junction_deviation = 0.01
    th.max_accel_to_decel = 50000.0
    th.unified_emit = True
    th.unified_max_jerk = 0.0
    th.unified_jerk_dt = 0.001
    th.unified_max_da = 0.0
    th.unified_notch_freq = 55.0
    th.unified_notch_freq_x = 55.0
    th.unified_notch_freq_y = 55.0
    th.unified_notch_max_freq = 0.0
    th.extra_axes = []
    th.printer = FakePrinter()
    for k, v in over.items():
        setattr(th, k, v)
    return th


def test_notch_freq_floor():
    err = lambda msg: RuntimeError(msg)  # noqa: E731
    # 0 / None mean "notch disabled" and must stay allowed.
    toolhead.ToolHead._check_notch_freq(0.0, "unified_notch_freq", err)
    toolhead.ToolHead._check_notch_freq(None, "unified_notch_freq", err)
    # The classic typo: 0.55 instead of 55. Every ramp would take 3.6 s.
    for bad in (0.55, 1.0, 4.9):
        try:
            toolhead.ToolHead._check_notch_freq(bad, "unified_notch_freq", err)
        except RuntimeError as e:
            assert "at least" in str(e), e
        else:
            raise AssertionError("accepted stalling notch freq %r" % (bad,))
    for good in (5.0, 30.0, 55.0):
        toolhead.ToolHead._check_notch_freq(good, "unified_notch_freq", err)
    print("  notch frequency floor rejects stalling values OK")


def test_set_unified_pair_rule_and_rollback():
    # Setting only NOTCH_FREQ_X used to slip past the X/Y pair rule that is a
    # hard config error at startup. It must be rejected, and rejection must
    # leave every unified field untouched.
    th = make_unified_toolhead(
        unified_notch_freq_x=0.0,
        unified_notch_freq_y=0.0,
        unified_notch_freq=0.0,
    )
    th.flush_step_generation = lambda: None
    before = th._unified_state()
    gcmd = FakeGCmd()
    gcmd.get_int = lambda n, d, **kw: d
    vals = {"NOTCH_FREQ_X": 55.0}
    gcmd.get_float = lambda n, d, **kw: vals.get(n, d)
    gcmd.respond_info = lambda msg: None
    try:
        th.cmd_SET_UNIFIED(gcmd)
    except RuntimeError as e:
        assert "both" in str(e), e
    else:
        raise AssertionError("one-sided NOTCH_FREQ_X was accepted")
    assert th._unified_state() == before, (th._unified_state(), before)
    print("  SET_UNIFIED enforces the X/Y pair rule and rolls back OK")


def test_reach_cache_tracks_start_v2():
    # The per-move reach memo must key on start_v2: flush() probes the same
    # move at several speeds, and a stale hit would silently pin the toolhead.
    th = make_unified_toolhead()
    th._move_notch_freq = lambda move: 55.0
    # 8 mm is runway-limited at 55 Hz (max_velocity is not the binding limit),
    # so the two start speeds give genuinely different answers.
    move = toolhead.Move(th, [0.0, 0.0, 0.0, 0.0], [8.0, 0.0, 0.0, 0.0], 650.0)
    a = th._move_reach_v2(move, 0.0)
    b = th._move_reach_v2(move, 100.0 * 100.0)
    assert a != b, (a, b)
    # Re-probing must replay each answer, not serve the other one from a
    # single-slot cache that ignored the key.
    assert th._move_reach_v2(move, 0.0) == a, (a, b)
    assert th._move_reach_v2(move, 100.0 * 100.0) == b, (a, b)
    assert th._move_reach_v2(move, 0.0) == a, (a, b)
    print("  per-move reach cache keys on start_v2 OK")


def main():
    test_subclass_without_unified_fields()
    test_unified_rejects_unsegmented_extra_axis()
    test_notch_freq_floor()
    test_set_unified_pair_rule_and_rollback()
    test_reach_cache_tracks_start_v2()
    print("ALL PASS")


if __name__ == "__main__":
    main()

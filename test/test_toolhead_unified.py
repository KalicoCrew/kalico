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
            self.process_move_cubic = lambda *args: None

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
        assert "process_move_cubic" in str(e)
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
    th.unified_notch_freq = 55.0
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


def test_unified_settings_require_notch():
    err = lambda msg: RuntimeError(msg)  # noqa: E731
    th = make_unified_toolhead(unified_notch_freq=0.0)
    try:
        th._check_unified_settings(err)
    except RuntimeError as e:
        assert "requires unified_notch_freq" in str(e), e
    else:
        raise AssertionError("enabled planner without a notch was accepted")
    th = make_unified_toolhead()
    th._check_unified_settings(err)
    th.unified_emit = False
    th.unified_notch_freq = 0.0
    th._check_unified_settings(err)
    print("  enabled planner requires a notch OK")


def test_low_max_accel_is_accepted_and_renders():
    # A saturated plateau is one segment however long it is, so any
    # max_accel -- including what M204 may lower it to mid-print -- is just a
    # slow ramp.
    err = lambda msg: RuntimeError(msg)  # noqa: E731
    th = make_unified_toolhead(max_accel=50.0, max_velocity=800.0)
    th._check_unified_settings(err)
    cons = toolhead.pathplan.Constraints(
        a_const=th.max_accel,
        v_ceil=th.max_velocity + 1.0,
        notch_freq=th.unified_notch_freq,
    )
    toolhead.pathplan.validate_cubic_profile(0.0, 400.0, 0.0, 1e6, cons)
    segs = toolhead.pathplan.render_cubic_profile(0.0, 400.0, 0.0, 1e6, cons)
    assert len(segs) == 7, segs  # 3 up + cruise + 3 down
    print("  a very low max_accel is accepted and renders OK")


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


def test_no_velocity_step_at_move_boundaries():
    # The stock planner gets boundary continuity for free: with constant accel
    # reachable_start_v2 == next_end_v2 + delta_v2 and start_v2 == next_end_v2 -
    # delta_v2 on a full-accel move, so the accel-to-decel midpoint lands
    # exactly on next_end_v2. The notch law breaks that algebra, and without
    # the lift the midpoint lands BELOW next_end_v2 -- leaving this move ending
    # slower than the next one starts, i.e. a hard velocity step (80 mm/s on
    # 5 mm segments at 55 Hz) which is an infinite-accel impulse in the very
    # band the notch exists to keep quiet.
    for seg in (0.2, 1.0, 2.0, 5.0, 10.0):
        th = make_unified_toolhead(
            max_accel=20000.0,
            max_velocity=600.0,
            max_accel_to_decel=10000.0,
            junction_deviation=0.5 * 5.0**2 / 20000.0,
        )
        laq = toolhead.LookAheadQueue()
        pos = [0.0, 0.0, 0.0, 0.0]
        for _ in range(12):
            nxt = [pos[0] + seg, 0.0, 0.0, 0.0]
            laq.add_move(toolhead.Move(th, pos, nxt, 300.0))
            pos = nxt
        moves = laq.flush()
        for i in range(1, len(moves)):
            gap = moves[i].start_v - moves[i - 1].end_v
            assert abs(gap) <= 1e-9, (
                "velocity step at move boundary",
                seg,
                i,
                gap,
                moves[i - 1].end_v,
                moves[i].start_v,
            )
    print("  notch plan leaves no velocity step at move boundaries OK")


def test_infeasible_batch_fails_before_emission():
    th = make_unified_toolhead()
    first = toolhead.Move(
        th,
        [0.0, 0.0, 0.0, 0.0],
        [100.0, 0.0, 0.0, 1.0],
        200.0,
    )
    first.start_v = 0.0
    first.cruise_v = first.end_v = 100.0
    move_d = (
        toolhead.pathplan.notch_dist(
            200.0, 0.0, th.max_accel, th.unified_notch_freq
        )
        * 0.8
    )
    second = toolhead.Move(
        th,
        [100.0, 0.0, 0.0, 1.0],
        [100.0 + move_d, 0.0, 0.0, 2.0],
        200.0,
    )
    second.start_v = second.cruise_v = 200.0
    second.end_v = 0.0

    emitted = []

    class CountingAxis:
        def process_move_cubic(self, *args):
            emitted.append(("axis", args))

    th.extra_axes = [CountingAxis()]
    th.lookahead = types.SimpleNamespace(
        flush=lambda lazy=False: [first, second]
    )
    th.special_queuing_state = "NeedPrime"
    th.print_time = 123.0
    th.trapq = object()
    th.trapq_append = lambda *args: emitted.append(("trapq", args))
    th.trapq_append_jerk = lambda *args: emitted.append(("trapq", args))
    th._calc_print_time = lambda: emitted.append(("calc_print_time", ()))

    try:
        th._process_lookahead()
    except toolhead.pathplan.InfeasibleProfile as e:
        assert "does not fit move" in str(e)
    else:
        raise AssertionError("infeasible profile reached trapq emission")

    assert emitted == [], emitted
    assert th.print_time == 123.0
    assert th.special_queuing_state == "NeedPrime"
    print("  infeasible batch fails before timing or motion-queue mutation OK")


def main():
    test_subclass_without_unified_fields()
    test_unified_rejects_unsegmented_extra_axis()
    test_notch_freq_floor()
    test_unified_settings_require_notch()
    test_low_max_accel_is_accepted_and_renders()
    test_reach_cache_tracks_start_v2()
    test_no_velocity_step_at_move_boundaries()
    test_infeasible_batch_fails_before_emission()
    print("ALL PASS")


if __name__ == "__main__":
    main()

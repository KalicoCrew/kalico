# Standalone regression tests for unified-planner ToolHead integration.
#
# Run: klippy-env/bin/python test/test_toolhead_unified.py
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
    th._span_reach_v2 = toolhead.ToolHead._span_reach_v2.__get__(th, object)
    th._span_link_ok = toolhead.ToolHead._span_link_ok.__get__(th, object)
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
    th.unified_jerk_dt = 0.001
    th.unified_max_da = 0.0
    th.unified_notch_freq = 55.0
    th.unified_notch_freq_x = 55.0
    th.unified_notch_freq_y = 55.0
    th.unified_span_ramps = True
    th.unified_span_max_angle = toolhead.SPAN_MAX_ANGLE
    th.unified_span_min_cos = math.cos(math.radians(th.unified_span_max_angle))
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


def test_unified_settings_require_notch_and_resolved_timestep():
    err = lambda msg: RuntimeError(msg)  # noqa: E731
    th = make_unified_toolhead(
        unified_notch_freq=0.0,
        unified_notch_freq_x=0.0,
        unified_notch_freq_y=0.0,
    )
    try:
        th._check_unified_settings(err)
    except RuntimeError as e:
        assert "requires unified_notch_freq" in str(e), e
    else:
        raise AssertionError("enabled planner without a notch was accepted")

    th = make_unified_toolhead(
        unified_jerk_dt=0.002,
        unified_notch_freq_x=55.0,
        unified_notch_freq_y=75.0,
    )
    try:
        th._check_unified_settings(err)
    except RuntimeError as e:
        assert "at least 10 slices" in str(e), e
    else:
        raise AssertionError("under-resolved notch timestep was accepted")

    th.unified_jerk_dt = 0.001
    th._check_unified_settings(err)
    th.unified_emit = False
    th.unified_jerk_dt = 1.0
    th._check_unified_settings(err)
    print("  enabled planner requires a notch resolved by jerk_dt OK")


def test_unified_settings_bound_the_ramp_slice_budget():
    # jerk_dt bounds slices per ramp from BELOW (resolve the notch edge). This
    # is the bound from above, and it has to live at config time: the emitter
    # reports an over-resolved ramp by raising InfeasibleProfile, nothing in
    # klippy catches that, and it lands as a shutdown on some G1 mid-print.
    err = lambda msg: RuntimeError(msg)  # noqa: E731

    # unified_max_da shrinks the integration step directly and, unlike
    # jerk_dt, has no floor -- it is the setting that gets there first.
    th = make_unified_toolhead(unified_max_da=20.0, max_velocity=400.0)
    th._check_unified_settings(err)
    th.unified_max_da = 10.0
    try:
        th._check_unified_settings(err)
    except RuntimeError as e:
        assert "more than 4096 slices" in str(e), e
        assert "unified_max_da" in str(e), e
    else:
        raise AssertionError("over-resolved max_da was accepted")

    # A low max_accel gets there the other way, by stretching the saturated
    # plateau rather than by shrinking the step.
    th = make_unified_toolhead(max_accel=100.0, max_velocity=400.0)
    th._check_unified_settings(err)
    th.max_accel = 50.0
    try:
        th._check_unified_settings(err)
    except RuntimeError as e:
        assert "more than 4096 slices" in str(e), e
    else:
        raise AssertionError("over-long saturated plateau was accepted")
    print("  config rejects an over-resolved ramp slice budget OK")


def test_over_budget_settings_still_render_at_runtime():
    # The config bound must NOT be the runtime bound. max_accel is not static
    # -- M204 lowers it per feature, and failing there would be precisely the
    # mid-print shutdown this is avoiding -- so a ramp past the config budget
    # still has to emit. Only non-convergence may go fatal.
    err = lambda msg: RuntimeError(msg)  # noqa: E731
    th = make_unified_toolhead(max_accel=20000.0, max_velocity=400.0)
    th._check_unified_settings(err)  # the config a print starts with
    th.max_accel = 50.0  # what an M204 may do to it, unchecked
    cons = toolhead.pathplan.Constraints(
        a_const=th.max_accel,
        v_ceil=th.max_velocity + 1.0,
        jerk_dt=th.unified_jerk_dt,
        notch_freq=th.unified_notch_freq_x,
    )
    assert not toolhead.pathplan.ramp_fits_slice_budget(0.0, 400.0, cons)
    segs = toolhead.pathplan.emit_profile(0.0, 400.0, 0.0, 1e6, cons)
    assert segs, "a ramp past the config budget must still emit"
    print("  a ramp past the config budget still renders OK")


def test_set_velocity_limit_rolls_back_an_unemittable_limit():
    # Raising max_velocity raises the dv of the worst ramp, which raises J,
    # which shrinks a max_da-limited step -- so VELOCITY alone can push a valid
    # unified config past its slice budget. The command must refuse it and
    # leave BOTH limits as they were; a half-applied limit is worse than the
    # error, because the next G1 plans against it.
    th = make_unified_toolhead(unified_max_da=20.0, max_velocity=400.0)
    th._check_unified_settings(th.printer.command_error)
    gcmd = FakeGCmd()
    asked = {"VELOCITY": 800.0}
    gcmd.get_float = lambda n, d, **kw: asked.get(n, d)
    try:
        th.cmd_SET_VELOCITY_LIMIT(gcmd)
    except RuntimeError as e:
        assert "more than 4096 slices" in str(e), e
    else:
        raise AssertionError("unemittable VELOCITY was accepted")
    assert th.max_velocity == 400.0, th.max_velocity
    assert th.max_accel == 100000.0, th.max_accel
    print("  SET_VELOCITY_LIMIT refuses and rolls back an unemittable limit OK")


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


def test_set_unified_rearms_profile_warnings():
    th = make_unified_toolhead()
    th.flush_step_generation = lambda: None
    th._unified_warned = set(toolhead.pathplan.LOSS_REASONS)
    th._unified_all_warned = True
    gcmd = FakeGCmd()
    vals = {"SPAN_MAX_ANGLE": 1.5}
    gcmd.get_int = lambda n, d, **kw: d
    gcmd.get_float = lambda n, d, **kw: vals.get(n, d)
    gcmd.respond_info = lambda msg: None
    th.cmd_SET_UNIFIED(gcmd)
    assert th._unified_warned == set()
    assert not th._unified_all_warned
    print("  SET_UNIFIED re-arms profile-loss diagnostics OK")


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
    # exactly on next_end_v2. The notch law breaks that algebra, and the
    # midpoint used to land BELOW next_end_v2 -- leaving this move ending
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


def test_reset_velocity_limit_resyncs_derived_span_angle():
    # unified_span_min_cos is DERIVED from unified_span_max_angle. Restoring
    # the angle without re-deriving it leaves RESET_VELOCITY_LIMIT reporting
    # the config value while the planner keeps honouring the live one -- it
    # would report 2 degrees and go on accepting 18 degree turns.
    th = make_unified_toolhead(min_cruise_ratio=0.5, square_corner_velocity=5.0)
    th.kin = types.SimpleNamespace()
    th.flush_step_generation = lambda: None
    th.orig_cfg = {
        "max_velocity": th.max_velocity,
        "max_accel": th.max_accel,
        "square_corner_velocity": th.square_corner_velocity,
        "min_cruise_ratio": th.min_cruise_ratio,
    }
    for name in toolhead.ToolHead._UNIFIED_FIELDS:
        th.orig_cfg[name] = getattr(th, name)

    # Widen the angle live, the way SET_UNIFIED SPAN_MAX_ANGLE=18 would.
    th.unified_span_max_angle = 18.0
    th._sync_span_cos()
    prev = toolhead.Move(th, [0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], 100.0)
    ang = math.radians(18.0) * 0.5  # 9 degrees: inside 18, outside 2
    move = toolhead.Move(
        th,
        [1.0, 0.0, 0.0, 0.0],
        [1.0 + math.cos(ang), math.sin(ang), 0.0, 0.0],
        100.0,
    )
    cos_theta = -(math.cos(ang))
    assert th._span_link_ok(prev, move, cos_theta), "18 deg should span at 18"

    gcmd = FakeGCmd()
    gcmd.respond_info = lambda msg: None
    saved_danger = toolhead.get_danger_options
    toolhead.get_danger_options = lambda: types.SimpleNamespace(
        log_velocity_limit_changes=False
    )
    try:
        th.cmd_RESET_VELOCITY_LIMIT(gcmd)
    finally:
        toolhead.get_danger_options = saved_danger

    assert th.unified_span_max_angle == toolhead.SPAN_MAX_ANGLE, (
        th.unified_span_max_angle,
    )
    want = math.cos(math.radians(toolhead.SPAN_MAX_ANGLE))
    assert abs(th.unified_span_min_cos - want) < 1e-12, (
        "derived cos went stale after reset",
        th.unified_span_min_cos,
        want,
    )
    assert not th._span_link_ok(prev, move, cos_theta), (
        "planner still accepts a 9 deg turn after resetting to"
        " %.1f deg" % (toolhead.SPAN_MAX_ANGLE,)
    )
    print("  RESET_VELOCITY_LIMIT re-derives the span angle OK")


def test_infeasible_batch_fails_before_emission():
    th = make_unified_toolhead(unified_span_ramps=False)
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
        def process_move_segment(self, *args):
            emitted.append(("axis", args))

    th.extra_axes = [CountingAxis()]
    th.lookahead = types.SimpleNamespace(
        flush=lambda lazy=False: [first, second]
    )
    th.special_queuing_state = "NeedPrime"
    th.print_time = 123.0
    th.trapq = object()
    th.trapq_append = lambda *args: emitted.append(("trapq", args))
    th._calc_print_time = lambda: emitted.append(("calc_print_time", ()))
    th._unified_all_warned = False
    th._unified_warned = set()

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
    test_unified_settings_require_notch_and_resolved_timestep()
    test_unified_settings_bound_the_ramp_slice_budget()
    test_over_budget_settings_still_render_at_runtime()
    test_set_velocity_limit_rolls_back_an_unemittable_limit()
    test_set_unified_pair_rule_and_rollback()
    test_set_unified_rearms_profile_warnings()
    test_reach_cache_tracks_start_v2()
    test_no_velocity_step_at_move_boundaries()
    test_reset_velocity_limit_resyncs_derived_span_angle()
    test_infeasible_batch_fails_before_emission()
    print("ALL PASS")


if __name__ == "__main__":
    main()

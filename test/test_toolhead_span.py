# Tests for cross-move ramp spanning in the unified planner.
#
# A notched accel ramp lasts 2/f_n seconds no matter how small the speed change
# is, so it needs (v0+v1)/f_n of travel. Requiring that runway INSIDE every move
# floors the reachable speed of a chain of L-mm segments at about f_n*L -- 11
# mm/s on 0.2 mm segments at 55 Hz. Spanning lets one ramp cover a run of
# near-collinear moves, so the runway is the run's length. The ramp keeps its
# shape, so its spectral zero stays exactly on f_n; only the demand that
# acceleration return to zero at every move boundary is dropped.
#
# Verifies: a chain of short segments actually accelerates with spanning on and
# is pinned at ~f_n*L with it off; the spanning profile's acceleration spectrum
# still has its null at f_n; per-move buckets conserve distance and velocity;
# and a real corner still breaks the run.
#
# Run: klippy-env/bin/python test/test_toolhead_span.py
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

FN = 55.0  # notch frequency (Hz) -- red's configured Y mode
SEG = 0.2  # segment length (mm): a curve sliced by a slicer
FEED = 300.0  # requested feedrate (mm/s)


class FakePrinter:
    def command_error(self, msg):
        return RuntimeError(msg)


def make_toolhead(span=True, notch=FN):
    th = object.__new__(toolhead.ToolHead)
    th.max_accel = 20000.0
    th.max_velocity = 600.0
    th.max_accel_to_decel = 10000.0
    th.square_corner_velocity = 5.0
    th.junction_deviation = 0.5 * th.square_corner_velocity**2 / th.max_accel
    th.unified_emit = True
    th.unified_jerk_dt = 0.0005
    th.unified_max_da = 0.0
    th.unified_notch_freq = notch
    th.unified_notch_freq_x = notch
    th.unified_notch_freq_y = notch
    th.unified_span_ramps = span
    th.unified_span_max_angle = toolhead.SPAN_MAX_ANGLE
    th.unified_span_min_cos = math.cos(math.radians(th.unified_span_max_angle))
    th.extra_axes = []
    th.printer = FakePrinter()
    th._unified_warned = set()
    th._unified_all_warned = True  # skip the diagnostic in these tests
    return th


def plan_chain(th, n, seg=SEG, feed=FEED, turn_at=None, turn_deg=0.0):
    """Queue n collinear seg-mm moves (optionally with one corner) and plan."""
    laq = toolhead.LookAheadQueue()
    pos = [0.0, 0.0, 0.0, 0.0]
    ang = 0.0
    for i in range(n):
        if turn_at is not None and i == turn_at:
            ang = math.radians(turn_deg)
        nxt = [
            pos[0] + seg * math.cos(ang),
            pos[1] + seg * math.sin(ang),
            0.0,
            0.0,
        ]
        move = toolhead.Move(th, pos, nxt, feed)
        laq.add_move(move)
        pos = nxt
    return laq.flush()


def span_buckets(th, moves):
    """Materialize span plans for tests; production renders one group at a time."""
    out = {}
    plans = th._span_plans(moves)
    for move in moves:
        item = plans.get(id(move))
        if item is None or item[1] != 0:
            continue
        group, vs, vc, ve, total_d, cons = item[0]
        segs = toolhead.pathplan.emit_profile(vs, vc, ve, total_d, cons)
        buckets = toolhead.pathplan.split_segments(
            segs, [m.move_d for m in group]
        )
        out.update(
            (id(member), bucket) for member, bucket in zip(group, buckets)
        )
    return out


def span_profile(th, moves):
    """Concatenate the spanning buckets covering `moves`, or None."""
    buckets = span_buckets(th, moves)
    if not buckets:
        return None
    segs = []
    for move in moves:
        b = buckets.get(id(move))
        if b is None:
            continue
        segs.extend(b)
    return segs or None


def axis_spectrum(items, freq, axis):
    """|FFT| of a_axis(t) = r[axis] * a(t) at `freq`, over (axes_r, slices).

    This is the measurement that matters. The scalar a(t) keeps its null by
    construction, but the AXES see a(t) scaled by the move's direction, so a
    heading change part-way through a ramp truncates the pulse on the turning
    axis and destroys the null there. A scalar-only spectrum cannot see it.

    freq is None returns the L1 norm of the SCALAR a(t), the common normaliser
    for both axes so the two are compared in absolute terms.
    """
    if freq is None:
        return math.fsum(
            abs(a) * (at + dt)
            for _r, segs in items
            for at, _ct, dt, _sv, _cv, a, _d in segs
        )
    w = 2.0 * math.pi * freq
    re = im = t = 0.0
    for r, segs in items:
        ra = r[axis]
        for at, ct, dt, _sv, _cv, a, _d in segs:
            for dur, acc in ((at, ra * a), (ct, 0.0), (dt, -ra * a)):
                if dur <= 0.0:
                    continue
                nt = t + dur
                if acc:
                    re += acc * (math.sin(w * nt) - math.sin(w * t)) / w
                    im += acc * (math.cos(w * nt) - math.cos(w * t)) / w
                t = nt
    return math.hypot(re, im)


def accel_spectrum(segs, freq):
    """|FFT| of the emitted a(t) staircase at `freq` (ZOH slices).

    freq <= 0 returns the L1 norm of a(t), which is the natural scale for a
    profile whose accel and decel pulses cancel at DC.
    """
    if freq <= 0.0:
        return math.fsum(
            abs(a) * (at + dt) for at, _ct, dt, _sv, _cv, a, _d in segs
        )
    w = 2.0 * math.pi * freq
    re = im = t = 0.0
    for at, ct, dt, _sv, _cv, a, _d in segs:
        for dur, acc in ((at, a), (ct, 0.0), (dt, -a)):
            if dur <= 0.0:
                continue
            nt = t + dur
            if acc:
                re += acc * (math.sin(w * nt) - math.sin(w * t)) / w
                im += acc * (math.cos(w * nt) - math.cos(w * t)) / w
            t = nt
    return math.hypot(re, im)


def test_span_beats_the_runway_floor():
    # 40 * 0.2 mm = 8 mm of runway. Per-move ramps cannot use it: each move can
    # only change speed by what fits in 0.2 mm, which is nothing.
    n = 40
    off = plan_chain(make_toolhead(span=False), n)
    on = plan_chain(make_toolhead(span=True), n)
    v_off = max(m.cruise_v for m in off)
    v_on = max(m.cruise_v for m in on)
    floor = FN * SEG  # 11 mm/s
    assert v_off <= 3.0 * floor, ("no-span chain should be pinned", v_off)
    assert v_on > 8.0 * floor, ("spanning did not lift the floor", v_on, v_off)
    print(
        "  span lifts a %d x %.1f mm chain from %.1f to %.1f mm/s"
        " (per-move floor ~%.1f) OK" % (n, SEG, v_off, v_on, floor)
    )


def test_span_keeps_the_zero_on_fn():
    th = make_toolhead(span=True)
    moves = plan_chain(th, 40)
    segs = span_profile(th, moves)
    assert segs is not None, "no spanning profile was emitted"
    # Isolate the accel pulse: it is the shaper, and it must be ONE triangle
    # spanning many moves rather than one triangle per move.
    pulse = []
    for seg in segs:
        if seg[0] <= 0.0:
            break
        pulse.append(seg)
    assert len(pulse) > 1, "no spanning accel pulse"
    dc = accel_spectrum(pulse, 0.0)
    assert dc > 1e-6, ("degenerate pulse", dc)
    at_fn = accel_spectrum(pulse, FN) / dc
    # Off-notch reference: the same pulse is NOT quiet away from the mode.
    off_notch = accel_spectrum(pulse, FN * 0.5) / dc
    assert at_fn < 0.02, ("zero moved off f_n", at_fn)
    assert off_notch > 10.0 * at_fn, ("null is not selective", at_fn, off_notch)
    # And the pulse really did span: its rise time is 1/f_n, far longer than
    # any single 0.2 mm move lasts.
    # (the few-percent shortfall is the known ZOH discretization of the ramp,
    # not a shift of the law -- see test_zoh_notch_error_scales_with_dt)
    rise = math.fsum(s[0] for s in pulse)
    assert abs(rise - 2.0 / FN) < 0.1 / FN, ("ramp is not 2/f_n", rise)
    print(
        "  spanning accel pulse: rise %.4f s (2/f_n = %.4f),"
        " |A(f_n)|/|A(0)| = %.5f vs %.4f at f_n/2 OK"
        % (rise, 2.0 / FN, at_fn, off_notch)
    )


def test_span_keeps_BOTH_zeros_with_per_axis_modes():
    # The toolhead -> pathplan seam, end to end: per-axis modes must reach the
    # emitter as a PAIR (_move_notch_pair -> Constraints.notch_freq2) and the
    # spanning ramp must come back with a null on each of them.
    #
    # Spanning and two zeros interact, so this is worth checking together rather
    # than trusting each in isolation: the span is emitted as ONE profile over
    # the whole run and then split into per-move buckets, and it is that single
    # long pulse -- not any per-move fragment -- that has to carry both nulls.
    f_x, f_y = 55.0, 70.0
    th = make_toolhead(span=True)
    th.unified_notch_freq = 0.0
    th.unified_notch_freq_x = f_x
    th.unified_notch_freq_y = f_y
    moves = plan_chain(th, 60)

    cons = th._pathplan_cons(moves[0])
    assert (cons.notch_lo, cons.notch_hi) == (f_x, f_y), (
        "per-axis modes did not reach the emitter as a pair",
        cons.notch_lo,
        cons.notch_hi,
    )

    segs = span_profile(th, moves)
    assert segs is not None, "no spanning profile was emitted"
    pulse = []
    for seg in segs:
        if seg[0] <= 0.0:
            break
        pulse.append(seg)
    assert len(pulse) > 1, "no spanning accel pulse"
    dc = accel_spectrum(pulse, 0.0)
    assert dc > 1e-6, ("degenerate pulse", dc)
    at_x = accel_spectrum(pulse, f_x) / dc
    at_y = accel_spectrum(pulse, f_y) / dc
    assert at_x < 0.02, ("no null at f_x", at_x)
    assert at_y < 0.02, ("no null at f_y", at_y)
    # Not just a long quiet pulse: well below the pair, the same pulse carries
    # most of its energy.
    #
    # The probe is deliberately at f_x/2 and not between the two zeros. 55 and
    # 70 Hz are close enough that |sinc(f/55)*sinc(f/70)| stays small across the
    # whole gap -- about 0.014 at the midpoint against a 0.007-0.009 ZOH floor
    # at the zeros themselves. That shallow ratio is the pair working, not a
    # missing null, so asserting on it would only measure the discretization.
    off_notch = accel_spectrum(pulse, 0.5 * f_x) / dc
    assert off_notch > 20.0 * max(at_x, at_y), (off_notch, at_x, at_y)
    # The ramp lasts 1/f_x + 1/f_y, not 2/f_n for either mode alone.
    rise = math.fsum(s[0] for s in pulse)
    want = 1.0 / f_x + 1.0 / f_y
    assert abs(rise - want) < 0.1 * want, ("ramp is not 1/f_x + 1/f_y", rise)
    print(
        "  spanning pulse over %d moves: rise %.4f s (1/f_x+1/f_y = %.4f),"
        " |A|/|A(0)| = %.5f at %g Hz and %.5f at %g Hz OK"
        % (len(moves), rise, want, at_x, f_x, at_y, f_y)
    )


def test_span_buckets_conserve_distance_and_velocity():
    th = make_toolhead(span=True)
    moves = plan_chain(th, 40)
    buckets = span_buckets(th, moves)
    assert buckets, "expected a spanning run"
    v = None
    for move in moves:
        b = buckets.get(id(move))
        if b is None:
            continue
        d = math.fsum(s[6] for s in b)
        assert abs(d - move.move_d) <= 1e-9 * max(1.0, move.move_d), (
            "bucket distance != move distance",
            d,
            move.move_d,
        )
        for at, ct, dt, sv, cv, a, dist in b:
            # Per-slice trapq-implied distance must equal the declared dist.
            implied = (
                (sv * at + 0.5 * a * at * at)
                + cv * ct
                + (cv * dt - 0.5 * a * dt * dt)
            )
            assert abs(implied - dist) <= 1e-6 * max(1.0, dist), (
                "trapq invariant broken",
                implied,
                dist,
            )
            if v is not None:
                assert abs(sv - v) <= 1e-6, ("velocity discontinuity", v, sv)
            v = cv - a * dt if dt > 0.0 else (sv + a * at if at > 0.0 else cv)
    print("  spanning buckets conserve distance and stay continuous OK")


def accel_pulse_items(th, moves):
    """(axes_r, slices) for the span's leading ACCEL pulse, in path order."""
    buckets = span_buckets(th, moves)
    items = []
    for move in moves:
        b = buckets.get(id(move))
        if b is None:
            if items:
                break
            continue
        keep = [s for s in b if s[0] > 0.0]
        if keep:
            items.append((move.axes_r, keep))
        if len(keep) != len(b):
            break
    return items


def turn_residual(deg, frac):
    """Residual at f_n on each axis when ONE notch ramp turns by `deg`.

    Built from the emitter directly rather than from a planned chain: a real
    corner also drags in the centripetal junction limit, which restructures the
    whole plan and stops the measurement from isolating the spanning effect.
    `frac` is where along the ramp the heading changes.
    """
    pathplan = toolhead.pathplan
    cons = pathplan.Constraints(
        a_const=20000.0,
        v_ceil=400.0,
        jerk_dt=0.0005,
        notch_freq=FN,
    )
    v = 300.0
    segs = pathplan.emit_profile(0.0, v, v, v / FN, cons)
    pulse = [seg for seg in segs if seg[0] > 0.0]
    assert len(pulse) > 4, "no accel pulse to split"
    d = math.fsum(seg[6] for seg in pulse)
    first, second = pathplan.split_segments(pulse, [d * frac, d * (1.0 - frac)])
    th = math.radians(deg)
    items = [
        ((1.0, 0.0, 0.0, 0.0), first),
        ((math.cos(th), math.sin(th), 0.0, 0.0), second),
    ]
    norm = axis_spectrum(items, None, 0)
    return (
        axis_spectrum(items, FN, 0) / norm,
        axis_spectrum(items, FN, 1) / norm,
    )


def worst_turn_residual(deg):
    # Scan where the turn lands; the truncated-pulse residual peaks near the
    # acceleration peak, not at either end.
    return max(
        turn_residual(deg, f)[1] for f in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8)
    )


def test_turning_axis_keeps_its_null():
    # The claim "the zero stays on f_n" has to hold on the AXES, not just on
    # the scalar path speed. Turning part-way through a ramp leaves the turning
    # axis a TRUNCATED pulse, which has no null; the residual scales as
    # |r2 - r1| * 0.159 (see SPAN_MAX_ANGLE). Budget it against the ~0.0066
    # the emitter's own discretization already leaves on a straight run.
    floor = turn_residual(0.0, 0.5)[0]
    assert 0.001 < floor < 0.02, ("unexpected collinear residual", floor)
    limit = make_toolhead(span=True).unified_span_max_angle
    at_limit = worst_turn_residual(limit)
    assert at_limit <= floor, (
        "the allowed heading change exceeds the emitter's own floor",
        limit,
        at_limit,
        floor,
    )
    # Sensitivity: the measurement must actually react. 18 degrees was the
    # original constant and has to come out clearly over budget, or this test
    # proves nothing.
    coarse = worst_turn_residual(18.0)
    assert coarse > 4.0 * floor, ("measurement is not sensitive", coarse, floor)
    print(
        "  turning axis: %.5f at the %.1f deg limit, %.5f at 18 deg,"
        " straight-run floor %.5f OK" % (at_limit, limit, coarse, floor)
    )


def test_span_angle_limit_is_enforced():
    th = make_toolhead(span=True)
    lim = th.unified_span_max_angle
    for deg, want in ((0.0, True), (lim * 0.5, True), (lim * 3.0, False)):
        m = plan_chain(make_toolhead(span=True), 12, turn_at=6, turn_deg=deg)
        assert m[6].span_link is want, (deg, want, m[6].span_link)
    # 18 degrees was the original constant; it must no longer span.
    m = plan_chain(make_toolhead(span=True), 12, turn_at=6, turn_deg=18.0)
    assert not m[6].span_link, "18 deg still spans"
    print("  heading-change limit is enforced at %.1f deg OK" % lim)


def test_lazy_flush_keeps_velocity_continuous():
    # Production never calls flush() once over the whole queue: add_move()
    # signals when to flush(lazy=True), which emits a prefix and REPLANS the
    # retained suffix. The split is only sound at a move whose start speed can
    # no longer change, and the constant-accel smoothed chain that picks it
    # under-estimates how far back a notched decel reaches.
    for span in (False, True):
        th = make_toolhead(span=span)
        laq = toolhead.LookAheadQueue()
        pos = [0.0, 0.0, 0.0, 0.0]
        batches = []
        for _ in range(500):
            nxt = [pos[0] + SEG, 0.0, 0.0, 0.0]
            move = toolhead.Move(th, pos, nxt, FEED)
            pos = nxt
            if laq.add_move(move):
                got = laq.flush(lazy=True)
                if got:
                    batches.append(got)
        got = laq.flush()
        if got:
            batches.append(got)
        assert len(batches) > 1, ("no lazy split happened", span)
        for bi, batch in enumerate(batches):
            for i in range(1, len(batch)):
                gap = batch[i].start_v - batch[i - 1].end_v
                assert abs(gap) <= 1e-6, ("gap inside batch", span, bi, i, gap)
            if bi:
                gap = batch[0].start_v - batches[bi - 1][-1].end_v
                assert abs(gap) <= 1e-6, (
                    "velocity step across a lazy flush boundary",
                    span,
                    bi,
                    batches[bi - 1][-1].end_v,
                    batch[0].start_v,
                )
        print(
            "  lazy flush span=%d: %d batches, all boundaries continuous OK"
            % (span, len(batches))
        )


def test_run_ended_by_a_nonkinematic_move_is_still_spanned():
    # Regression: red shut down mid-print with
    #   InfeasibleProfile: start=275.009495 cruise=400.000000 end=400.000000
    #                      distance=5.000081
    # on a G1 F24000 travel that bed_mesh had split into 5 mm segments.
    #
    # _span_groups() closed a run ONLY on the "usable" path, so a run ended by
    # a move that is not kinematic at all was dropped on the floor instead of
    # being yielded. Travels are bookended by G10/G11 -- extruder-only, hence
    # not kinematic -- so EVERY bed-mesh-split travel hit it. The lookahead had
    # already clamped those moves against the whole run's runway, so losing the
    # span left one 5 mm segment carrying 275 -> 400 mm/s, which needs
    # 0.5*(275+400)*(2/55) = 12.3 mm of path and has no notched profile at all.
    th = make_toolhead(span=True)
    moves = plan_chain(th, 12)
    assert all(m.is_kinematic_move for m in moves)
    spanned = sum(len(g) for g, _, _ in th._span_groups(moves))
    assert spanned, "no span formed at all; test proves nothing"

    # Same chain, but the run is now terminated by a non-kinematic move --
    # exactly what a retract does at the end of a travel.
    tail = moves[-1]
    tail.is_kinematic_move = False
    ended = sum(len(g) for g, _, _ in th._span_groups(moves))
    assert ended >= spanned - 1, (
        "a run ended by a non-kinematic move was discarded instead of "
        "yielded: %d moves spanned, %d after the run is closed by a retract"
        % (spanned, ended)
    )
    print(
        "  run closed by a non-kinematic move still spans"
        " (%d moves, %d with a trailing retract) OK" % (spanned, ended)
    )


def test_lazy_flush_never_emits_an_unrealisable_move():
    # Regression: red shut down mid-print with
    #   InfeasibleProfile: start=275.009495 cruise=400.000000 end=400.000000
    #                      distance=5.000081
    # on a G1 F24000 travel that bed_mesh had split into 5 mm segments.
    #
    # The clamp had let cruise reach 400 mm/s because the RUN had 38 mm of
    # runway, then a lazy flush split the run and handed the emitter one 5 mm
    # segment carrying the whole 275 -> 400 change. A notched ramp lasts
    # 1/f_lo + 1/f_hi no matter what, so that needs 0.5*(275+400)*(2/55) =
    # 12.3 mm of path -- no acceleration limit can buy it back, and with
    # max_accel this high the notch is the ONLY thing limiting accel.
    #
    # The invariant: whatever a lazy flush emits, _process_lookahead must be
    # able to validate. Here that is asserted directly against the batches.
    pathplan = toolhead.pathplan
    th = make_toolhead(span=True)
    th.max_accel = 100000.0  # red's config: notch-limited, not accel-limited
    th.max_velocity = 650.0
    th.junction_deviation = 0.5 * th.square_corner_velocity**2 / th.max_accel
    seg, feed = 5.0, 400.0

    laq = toolhead.LookAheadQueue()
    pos = [0.0, 0.0, 0.0, 0.0]
    batches = []
    for _ in range(400):
        nxt = [pos[0] + seg, 0.0, 0.0, 0.0]
        move = toolhead.Move(th, pos, nxt, feed)
        pos = nxt
        if laq.add_move(move):
            got = laq.flush(lazy=True)
            if got:
                batches.append(got)
    got = laq.flush()
    if got:
        batches.append(got)
    assert len(batches) > 1, "no lazy split happened; test proves nothing"

    checked = 0
    for bi, batch in enumerate(batches):
        # Exactly what _process_lookahead does before touching print_time.
        plans = th._span_plans(batch)
        for move in batch:
            if id(move) in plans or not move.is_kinematic_move:
                continue
            try:
                pathplan.validate_profile(
                    move.start_v,
                    move.cruise_v,
                    move.end_v,
                    move.move_d,
                    th._pathplan_cons(move),
                )
            except pathplan.InfeasibleProfile as e:
                raise AssertionError(
                    "lazy flush emitted a move the emitter cannot render "
                    "(batch %d, span_start_d=%.3f): %s"
                    % (bi, move.span_start_d, e)
                )
            checked += 1
    print(
        "  lazy flush emits only realisable moves: %d batches, %d validated OK"
        % (len(batches), checked)
    )


def test_corner_breaks_the_span():
    # A 60 degree turn is far outside unified_span_max_angle: the run must
    # not carry the ramp through it, so the moves on each side belong to
    # different spans.
    th = make_toolhead(span=True)
    moves = plan_chain(th, 40, turn_at=20, turn_deg=60.0)
    assert not moves[20].span_link, "span carried a ramp through a 60 deg turn"
    groups = list(th._span_groups(moves))
    for group, _peak, _cap in groups:
        assert moves[20] not in group[1:], "corner move joined a run"
    assert len(groups) >= 2, ("corner did not split the run", len(groups))
    print("  a real corner breaks the run into separate spans OK")


def test_span_follows_a_curve():
    # Every other span test walks a straight X line, where the notch target is
    # bit-identical between moves. On a CURVE the direction-weighted target
    # drifts, and an exact != comparison rejected every pair -- silently
    # disabling spanning on exactly the geometry it exists for. This walks an
    # arc whose chord angle is inside the heading limit and checks the run
    # really does span it.
    th = make_toolhead(span=True)
    radius = 40.0
    step = SEG / radius  # ~0.29 deg per segment, inside the 2 deg limit
    assert math.degrees(step) < th.unified_span_max_angle
    laq = toolhead.LookAheadQueue()
    pos = [radius, 0.0, 0.0, 0.0]
    for i in range(1, 60):
        a = i * step
        nxt = [radius * math.cos(a), radius * math.sin(a), 0.0, 0.0]
        laq.add_move(toolhead.Move(th, pos, nxt, FEED))
        pos = nxt
    moves = laq.flush()
    linked = sum(1 for m in moves if m.span_link)
    assert linked > len(moves) - 3, ("curve did not link", linked, len(moves))
    groups = list(th._span_groups(moves))
    assert groups, "no span formed on a curve"
    biggest = max(len(g) for g, _p, _c in groups)
    assert biggest > 20, ("curve span is too fragmented", biggest)
    # And it must actually beat the per-move floor on the same arc.
    off = make_toolhead(span=False)
    laq2 = toolhead.LookAheadQueue()
    pos = [radius, 0.0, 0.0, 0.0]
    for i in range(1, 60):
        a = i * step
        nxt = [radius * math.cos(a), radius * math.sin(a), 0.0, 0.0]
        laq2.add_move(toolhead.Move(off, pos, nxt, FEED))
        pos = nxt
    v_off = max(m.cruise_v for m in laq2.flush())
    v_on = max(m.cruise_v for m in moves)
    assert v_on > 4.0 * v_off, ("no speedup on a curve", v_off, v_on)
    print(
        "  curve of %d segments spans in runs of %d: %.1f -> %.1f mm/s OK"
        % (len(moves), biggest, v_off, v_on)
    )


def test_span_notch_target_is_constant_along_a_curve():
    # The single-zero blend placed the zero at a direction-weighted mean, so on
    # a curve the target crept at EVERY segment; spans then had to bound that
    # drift against the run's first move or it accumulated without limit.
    #
    # Two zeros remove the cause rather than bounding it. rect(1/f_hi) *
    # rect(1/f_lo) nulls both modes on both axes, so the ramp shape -- and the
    # target the span compares -- no longer depends on heading at all. The drift
    # bound is asserted here as ZERO, not merely within a tolerance.
    th = make_toolhead(span=True)
    th.unified_notch_freq = 0.0
    th.unified_notch_freq_x = 55.0
    th.unified_notch_freq_y = 70.0
    radius = 40.0
    step = SEG / radius
    laq = toolhead.LookAheadQueue()
    pos = [radius, 0.0, 0.0, 0.0]
    for i in range(1, 300):
        a = i * step
        nxt = [radius * math.cos(a), radius * math.sin(a), 0.0, 0.0]
        laq.add_move(toolhead.Move(th, pos, nxt, FEED))
        pos = nxt
    moves = laq.flush()
    targets = {th._move_notch_freq(m) for m in moves}
    assert len(targets) == 1, ("notch target varies along the curve", targets)
    pairs = {th._move_notch_pair(m) for m in moves}
    assert pairs == {(55.0, 70.0)}, pairs
    # And so the curve is not split by the notch target at all: every move
    # still ends up in a span, and every split is a RUN boundary rather than a
    # notch decision.
    #
    # This used to assert one group for the whole curve. Grouping now follows
    # the run the lookahead actually planned (span_start_d), so the constant-
    # cruise middle -- where there is no ramp to carry and spanning buys
    # nothing -- is not welded into the accel run. Demanding one group here
    # would be demanding exactly the over-grouping that let the clamp size a
    # move against a run the emitter then rendered differently.
    groups = list(th._span_groups(moves))
    covered = sum(len(g) for g, _peak, _cap in groups)
    assert covered == len(moves), (
        "spanning left moves behind on the curve",
        covered,
        len(moves),
    )
    for group, _peak, _cap in groups:
        assert group[0].span_start_d == 0.0, (
            "a span started mid-run, so the group is not the planner's run",
            group[0].span_start_d,
        )
        for member in group[1:]:
            assert member.span_start_d > 0.0, (
                "a move with no inherited runway was welded into a run",
                member.span_start_d,
            )
    # Drift is not merely bounded, it is ZERO -- which is why _span_link_ok no
    # longer carries a notch-target comparison at all. Asserted exactly, so
    # reintroducing any heading dependence fails here rather than silently
    # eating into a tolerance.
    for group, _peak, _cap in groups:
        first_f = th._move_notch_freq(group[0])
        for move in group:
            move_f = th._move_notch_freq(move)
            assert move_f == first_f, (
                "span accumulated notch drift",
                first_f,
                move_f,
            )
    print(
        "  per-axis notch target constant over %d curve segments (%.4f Hz) OK"
        % (len(moves), targets.pop())
    )


def test_span_survives_direction_dependent_accel():
    # limited_cartesian (and friends) set the accel limit per move as
    # min(x_max_a/|rx|, y_max_a/|ry|), so move.accel changes at EVERY segment
    # of a curve. Requiring equal accel to link rejected every pair and
    # disabled spanning entirely on such a machine -- invisible to any test
    # that uses a constant accel, and it cost a full hardware run to find.
    radius = 40.0
    step = SEG / radius

    def arc(th, apply_limit):
        laq = toolhead.LookAheadQueue()
        pos = [radius, 0.0, 0.0, 0.0]
        for i in range(1, 300):
            a = i * step
            nxt = [radius * math.cos(a), radius * math.sin(a), 0.0, 0.0]
            move = toolhead.Move(th, pos, nxt, FEED)
            if apply_limit:
                rx = max(abs(move.axes_r[0]), 1e-9)
                ry = max(abs(move.axes_r[1]), 1e-9)
                # Axis caps below the hypot limit, the way scale_xy_accel
                # leaves them, so the per-move value actually varies BELOW
                # max_accel instead of being clamped away by limit_speed.
                axis_a = th.max_accel * 0.5
                move.limit_speed(th.max_velocity, min(axis_a / rx, axis_a / ry))
            laq.add_move(move)
            pos = nxt
        return laq.flush()

    th = make_toolhead(span=True)
    moves = arc(th, True)
    accels = {m.accel for m in moves}
    assert len(accels) > 50, ("accel should vary per move", len(accels))
    linked = sum(1 for m in moves if m.span_link)
    assert linked > len(moves) - 3, (
        "per-move accel limits broke every span link",
        linked,
        len(moves),
    )
    groups = list(th._span_groups(moves))
    assert groups, "no span formed under a direction-dependent accel limit"
    # The emitted run must respect the TIGHTEST member's accel, not the first's.
    for group, _peak, _cap in groups:
        a_span = min(m.accel for m in group)
        assert a_span <= group[0].accel + 1e-9
    v_on = max(m.cruise_v for m in moves)
    v_off = max(m.cruise_v for m in arc(make_toolhead(span=False), True))
    assert v_on > 4.0 * v_off, ("no speedup under per-move accel", v_off, v_on)
    print(
        "  spans survive direction-dependent accel (%d distinct):"
        " %.1f -> %.1f mm/s OK" % (len(accels), v_off, v_on)
    )


def test_span_off_is_a_noop():
    th = make_toolhead(span=False)
    moves = plan_chain(th, 20)
    assert th._span_plans(moves) == {}, "spanning ran while disabled"
    for move in moves:
        assert not move.span_link
    print("  unified_span_ramps=0 leaves the per-move path untouched OK")


def main():
    test_span_beats_the_runway_floor()
    test_span_keeps_the_zero_on_fn()
    test_span_keeps_BOTH_zeros_with_per_axis_modes()
    test_span_buckets_conserve_distance_and_velocity()
    test_turning_axis_keeps_its_null()
    test_span_angle_limit_is_enforced()
    test_lazy_flush_keeps_velocity_continuous()
    test_run_ended_by_a_nonkinematic_move_is_still_spanned()
    test_lazy_flush_never_emits_an_unrealisable_move()
    test_corner_breaks_the_span()
    test_span_follows_a_curve()
    test_span_notch_target_is_constant_along_a_curve()
    test_span_survives_direction_dependent_accel()
    test_span_off_is_a_noop()
    print("ALL PASS")


if __name__ == "__main__":
    main()

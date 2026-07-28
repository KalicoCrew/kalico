# Tests for the per-ramp notch jerk law (J = dv * f_n^2) in pathplan.
#
# A jerk-limited ramp whose acceleration never saturates is TRIANGULAR in a(t),
# so it acts as a shaper with a zero at f = 1/T_rise where T_rise = sqrt(dv/J).
# With a FIXED jerk that zero slides as sqrt(J/dv) and cannot cancel a mode at a
# fixed frequency. Setting J = dv*f_n^2 per ramp pins T_rise at 1/f_n.
#
# Verifies: the zero is parked (rise time constant across dv, total ramp = 2/f_n);
# a_peak scales linearly with dv; ramp distance is (v0+v1)/f_n; the LOOKAHEAD
# twin notch_dist agrees with the emitter's integrated distance;
# notch_reach_v2 stays monotone; saturated ramps use a designed max-accel
# plateau; the stepguard invariants hold; and a full chain remains feasible.
#
# Run: klippy-env/bin/python test/test_pathplan_notch.py
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
import pathplan  # noqa: E402
from test_pathplan import check_segs  # noqa: E402

FN = 44.0  # mode frequency to park the zero on (Hz)
A_CONST = 30000.0  # high enough that a_peak = dv*f_n never saturates below
DVS = [50.0, 100.0, 150.0, 200.0, 300.0, 400.0]


def make_cons(notch=FN, a_const=A_CONST, jerk_dt=0.0002):
    return pathplan.Constraints(
        a_const=a_const,
        v_ceil=600.0,
        jerk_dt=jerk_dt,
        notch_freq=notch,
    )


def ramp_stats(v0, v1, cons):
    slices, dist = pathplan._ramp_up_jerk(v0, v1, cons)
    assert slices is not None, ("ramp did not converge", v0, v1)
    dur = sum(s[0] for s in slices)
    a_peak = max(s[5] for s in slices)
    return dur, a_peak, dist


def zoh_accel_spectrum(slices, freq):
    if freq <= 0.0:
        return sum(s[5] * s[0] for s in slices)
    w = 2.0 * math.pi * freq
    re = im = t = 0.0
    for s in slices:
        dt = s[0]
        a = s[5]
        nt = t + dt
        re += a * (math.sin(w * nt) - math.sin(w * t)) / w
        im += a * (math.cos(w * nt) - math.cos(w * t)) / w
        t = nt
    return math.hypot(re, im)


def test_zero_is_parked():
    # The accel pulse is a triangle: rise 0->a_peak over T_j, fall over T_j.
    # The shaper zero sits at 1/T_j, so it is the RISE time that must equal
    # 1/f_n -- the total ramp is 2*T_j = 2/f_n. _ramp_up_jerk integrates the
    # total, so check that.
    cons = make_cons()
    want_total = 2.0 / FN
    for dv in DVS:
        dur, _, _ = ramp_stats(10.0, 10.0 + dv, cons)
        assert abs(dur - want_total) <= 0.06 * want_total, (
            "zero not parked",
            dv,
            dur,
            want_total,
        )
    print("  ramp duration == 2/f_n across dv=%s OK" % (DVS,))


def test_apeak_linear_in_dv():
    cons = make_cons()
    for dv in DVS:
        _, a_peak, _ = ramp_stats(10.0, 10.0 + dv, cons)
        want = dv * FN
        assert abs(a_peak - want) <= 0.10 * want, (
            "a_peak not dv*f_n",
            dv,
            a_peak,
            want,
        )
    print("  a_peak == dv*f_n OK")


def test_ramp_distance_law():
    cons = make_cons()
    for dv in DVS:
        v0, v1 = 10.0, 10.0 + dv
        _, _, dist = ramp_stats(v0, v1, cons)
        want = (v0 + v1) / FN
        assert abs(dist - want) <= 0.06 * want, (
            "distance not (v0+v1)/f_n",
            dv,
            dist,
            want,
        )
    print("  ramp distance == (v0+v1)/f_n OK")


def test_lookahead_matches_emitter():
    # notch_dist() is what the lookahead plans with; _ramp_up_jerk() is what the
    # emitter renders. The analytic model is conservative against the
    # zero-order-held slices: it may under-use available runway, but it must not
    # approve a distance shorter than the emitted slices need.
    cons = make_cons()
    for dv in DVS:
        v0, v1 = 10.0, 10.0 + dv
        _, _, dist = ramp_stats(v0, v1, cons)
        pred = pathplan.notch_dist(v0, v1, A_CONST, notch_freq=FN)
        assert pred >= dist - 1e-9, (
            "lookahead not conservative",
            dv,
            pred,
            dist,
        )
        assert abs(pred - dist) <= 0.06 * max(pred, 1e-9), (
            "lookahead/emitter gap too large",
            dv,
            pred,
            dist,
        )
    print("  notch_dist conservatively bounds integrated ramp OK")


def test_lookahead_conservative_across_dt_and_saturation():
    cases = []
    for a_const in (3000.0, 8000.0, A_CONST):
        for jerk_dt in (0.001, 0.0005, 0.0002):
            cases.append(
                make_cons(notch=55.0, a_const=a_const, jerk_dt=jerk_dt)
            )
    for cons in cases:
        for v0 in (0.0, 10.0, 50.0, 150.0):
            for dv in (5.0, 20.0, 50.0, 100.0, 200.0):
                slices, dist = pathplan._ramp_up_jerk(v0, v0 + dv, cons)
                assert slices is not None
                pred = pathplan.notch_dist(
                    v0,
                    v0 + dv,
                    cons.a_const,
                    notch_freq=cons.notch_freq,
                )
                assert pred >= dist - 1e-8, (
                    "lookahead underestimates emitted distance",
                    cons.a_const,
                    cons.jerk_dt,
                    v0,
                    dv,
                    pred,
                    dist,
                )
    print("  notch_dist stays conservative across dt/saturation cases OK")


def test_reach_is_monotone_and_finite():
    prev = -1.0
    for d in (1.0, 2.0, 5.0, 10.0, 25.0, 60.0, 150.0):
        u = pathplan.notch_reach_v2(0.0, d, A_CONST, 500.0, notch_freq=FN)
        assert u >= prev - 1e-6, ("reach not monotone", d, u, prev)
        assert math.isfinite(u), ("reach not finite", d, u)
        prev = u
    print("  notch_reach_v2 monotone under the notch law OK")


def test_emitted_zoh_notch_response():
    cons = make_cons(jerk_dt=0.0002)
    slices, _ = pathplan._ramp_up_jerk(10.0, 110.0, cons)
    dc = zoh_accel_spectrum(slices, 0.0)
    at_notch = zoh_accel_spectrum(slices, FN) / dc
    below = zoh_accel_spectrum(slices, FN * 0.75) / dc
    above = zoh_accel_spectrum(slices, FN * 1.25) / dc
    assert at_notch < 0.01, ("actual emitted notch too shallow", at_notch)
    assert at_notch < below * 0.1, (
        "notch not below lower neighbor",
        at_notch,
        below,
    )
    assert at_notch < above * 0.1, (
        "notch not below upper neighbor",
        at_notch,
        above,
    )
    print("  emitted zero-order-hold spectrum has a near-zero at f_n OK")


def test_zoh_notch_error_scales_with_dt():
    vals = []
    for dt in (0.001, 0.0005, 0.0002):
        cons = make_cons(jerk_dt=dt)
        slices, _ = pathplan._ramp_up_jerk(10.0, 110.0, cons)
        vals.append(
            zoh_accel_spectrum(slices, FN) / zoh_accel_spectrum(slices, 0.0)
        )
    assert vals[0] > vals[1] > vals[2], vals
    print("  emitted notch residual shrinks with jerk_dt OK")


def test_notch_loss_reasons():
    clean = make_cons()
    # A feasible, uncapped move reports NOTHING. Zero-order-hold discretization
    # is inherent to slice emission and is deliberately not a reason, or every
    # ordinary print would log a warning on its first move.
    assert pathplan.notch_loss_reasons(0.0, 100.0, 0.0, clean) == []
    saturated = make_cons(notch=55.0, a_const=3000.0)
    assert pathplan.notch_loss_reasons(0.0, 100.0, 0.0, saturated) == []
    print("  notch loss diagnostics report emitted-profile limits OK")


def test_notch_saturated_trapezoid_reaches_requested_peak():
    cons = make_cons(notch=55.0, a_const=3000.0, jerk_dt=0.0002)
    segs = pathplan.emit_profile(0.0, 200.0, 0.0, 30.0, cons)
    assert segs
    peak_v = max(max(s[3], s[4]) for s in segs)
    peak_a = max(s[5] for s in segs)
    assert peak_v >= 199.0, peak_v
    assert peak_a <= cons.a_const + 1e-6, (peak_a, cons.a_const)
    assert pathplan.notch_loss_reasons(0.0, 200.0, 0.0, cons) == []
    print("  saturated notch reaches requested peak while capping accel OK")


def test_saturated_emitted_zoh_notch_response():
    cons = make_cons(notch=55.0, a_const=3000.0, jerk_dt=0.0002)
    slices, _ = pathplan._ramp_up_jerk(0.0, 200.0, cons)
    assert slices is not None
    dc = zoh_accel_spectrum(slices, 0.0)
    at_notch = zoh_accel_spectrum(slices, 55.0) / dc
    below = zoh_accel_spectrum(slices, 55.0 * 0.75) / dc
    above = zoh_accel_spectrum(slices, 55.0 * 1.25) / dc
    assert at_notch < 0.01, ("saturated emitted notch too shallow", at_notch)
    assert at_notch < below * 0.1, (at_notch, below)
    assert at_notch < above * 0.1, (at_notch, above)
    print("  saturated emitted ZOH spectrum has a near-zero at f_n OK")


def test_notch_reach_uses_saturated_plateau():
    accel = 3000.0
    notch = 55.0
    u = pathplan.notch_reach_v2(0.0, 1000.0, accel, 500.0, notch_freq=notch)
    v = math.sqrt(u)
    assert v > accel / notch + 1.0, (v, accel / notch)
    print("  notch reach uses designed saturated plateau OK")


def test_invariants_hold():
    cons = make_cons()
    cases = [
        (0.0, 200.0, 0.0, 60.0),
        (0.0, 250.0, 120.0, 50.0),
        (80.0, 300.0, 80.0, 90.0),
        (10.0, 210.0, 10.0, 30.0),
        (150.0, 150.0, 150.0, 20.0),
    ]
    for vs, vc, ve, d in cases:
        segs = pathplan.emit_profile(vs, vc, ve, d, cons)
        assert segs, ("empty", vs, vc, ve, d)
        check_segs(segs, d, vs, ve, "notch(%s,%s,%s,%s)" % (vs, vc, ve, d))
    print("  emit_profile invariants hold under the notch law OK")


def _notch_lookahead(move_d, v_cap, v_junction, accel, notch, v_ceil=600.0):
    # Mirror the toolhead lookahead with the notch-aware reach: backward decel
    # pass, forward accel pass, then the per-move jerk peak clamp.
    n = len(move_d)
    vb = [0.0] * (n + 1)
    for k in range(n + 1):
        cap = v_junction[k]
        if k > 0:
            cap = min(cap, v_cap[k - 1])
        if k < n:
            cap = min(cap, v_cap[k])
        vb[k] = cap
    for i in range(n - 1, -1, -1):
        r = pathplan.notch_reach_v2(
            vb[i + 1] ** 2, move_d[i], accel, v_ceil, notch_freq=notch
        )
        vb[i] = min(vb[i], math.sqrt(r))
    for i in range(n):
        r = pathplan.notch_reach_v2(
            vb[i] ** 2, move_d[i], accel, v_ceil, notch_freq=notch
        )
        vb[i + 1] = min(vb[i + 1], math.sqrt(r))
    vs = [vb[i] for i in range(n)]
    ve = [vb[i + 1] for i in range(n)]
    vc = []
    for i in range(n):
        pk = min(
            pathplan.notch_reach_v2(
                vs[i] ** 2, move_d[i], accel, v_ceil, notch_freq=notch
            ),
            pathplan.notch_reach_v2(
                ve[i] ** 2, move_d[i], accel, v_ceil, notch_freq=notch
            ),
        )
        vc.append(min(v_cap[i], math.sqrt(pk)))
    return vs, vc, ve


def test_chain_no_sharp_fallback():
    # Moves planned by the notch-aware lookahead must be renderable by the
    # notch-aware emitter WITHOUT the sharp fallback.
    cons = make_cons()
    n = 24
    move_d = [12.0] * n
    v_cap = [200.0] * n
    v_junction = [0.0] + [200.0] * (n - 1) + [0.0]
    vs, vc, ve = _notch_lookahead(move_d, v_cap, v_junction, A_CONST, FN)
    fell_back = 0
    for i in range(n):
        if (
            pathplan._emit_jerk_core(vs[i], vc[i], ve[i], move_d[i], cons)
            is None
        ):
            fell_back += 1
        segs = pathplan.emit_profile(vs[i], vc[i], ve[i], move_d[i], cons)
        check_segs(segs, move_d[i], vs[i], ve[i], "chain[%d]" % i)
    assert fell_back == 0, "%d/%d moves fell back to sharp" % (fell_back, n)
    print(
        "  24-move chain: peak cruise=%.1f mm/s, 0 sharp fallbacks OK" % max(vc)
    )


def test_short_chain_degrades_cleanly():
    cons = make_cons()
    n = 12
    move_d = [1.5] * n
    v_cap = [200.0] * n
    v_junction = [0.0] + [200.0] * (n - 1) + [0.0]
    vs, vc, ve = _notch_lookahead(move_d, v_cap, v_junction, A_CONST, FN)
    for i in range(n):
        segs = pathplan.emit_profile(vs[i], vc[i], ve[i], move_d[i], cons)
        assert segs, ("empty on short chain", i)
        check_segs(segs, move_d[i], vs[i], ve[i], "short_chain[%d]" % i)
    print("  sub-runway chain emits valid profiles (degrades, no hang) OK")


def test_loss_reasons_are_declared():
    # The toolhead stops calling the (expensive) diagnostic once it has logged
    # every reason in pathplan.LOSS_REASONS, so any reason the function can
    # actually emit MUST be listed there or it would be silently unreportable.
    cases = [
        (0.0, 100.0, 0.0, 20.0, make_cons()),
        (0.0, 100.0, 0.0, 20.0, make_cons(notch=55.0, a_const=3000.0)),
        (0.0, 200.0, 0.0, 0.5, make_cons()),
    ]
    for vs, vc, ve, move_d, cons in cases:
        for reason in pathplan.notch_loss_reasons(vs, vc, ve, cons):
            assert reason in pathplan.LOSS_REASONS, reason
    print("  every emitted loss reason is declared in LOSS_REASONS OK")


def test_excessive_slice_request_fails_before_integration():
    cons = pathplan.Constraints(
        a_const=100000.0,
        v_ceil=650.0,
        jerk_dt=0.001,
        notch_freq=55.0,
        max_da=0.01,
    )
    slices, distance = pathplan._ramp_up_jerk(0.0, 300.0, cons)
    assert slices is None
    assert math.isinf(distance)
    move_d = 2.0 * pathplan.notch_dist(0.0, 300.0, 100000.0, 55.0) + 1.0
    try:
        pathplan.validate_profile(0.0, 300.0, 0.0, move_d, cons)
    except pathplan.InfeasibleProfile:
        pass
    else:
        raise AssertionError("excessive slice request reached emission")
    print("  excessive slice request fails before integration OK")


def test_reach_runway_floor_short_circuit():
    # A notch ramp costs 2/f_n of TIME however small dv is, so its distance
    # bottoms out at 2*v0/f_n rather than at zero. Below that the move cannot
    # change speed at all, and notch_reach_v2 must say so exactly (not merely
    # bisect its way to something close).
    fn = 55.0
    for v0 in (20.0, 50.0, 100.0, 200.0, 300.0):
        u0 = v0 * v0
        d_floor = 2.0 * v0 / fn
        pinned = pathplan.notch_reach_v2(
            u0, d_floor * 0.999, A_CONST, 650.0, notch_freq=fn
        )
        assert pinned == u0, (v0, pinned, u0)
        # Just past the floor it must start moving again, and never regress.
        opened = pathplan.notch_reach_v2(
            u0, d_floor * 1.5, A_CONST, 650.0, notch_freq=fn
        )
        assert opened > u0, (v0, opened, u0)
    # From rest there is no floor: any distance buys some speed.
    assert (
        pathplan.notch_reach_v2(0.0, 0.05, A_CONST, 650.0, notch_freq=fn) > 0.0
    )
    print("  reach reports the exact runway floor for short moves OK")


def test_infeasible_notch_fails_closed():
    # An infeasible notch must never be reinterpreted as a fixed-jerk ramp,
    # because that would move or destroy the configured spectral zero.
    fn = 55.0
    cons = make_cons(notch=fn, jerk_dt=25e-6)
    vs = vc = 200.0
    ve = 0.0
    need = pathplan.notch_dist(vs, ve, A_CONST, fn)
    move_d = need * 0.99
    assert pathplan._emit_jerk_core(vs, vc, ve, move_d, cons) is None
    try:
        pathplan.emit_profile(vs, vc, ve, move_d, cons)
    except pathplan.InfeasibleProfile:
        pass
    else:
        raise AssertionError("infeasible notch did not fail closed")
    print("  infeasible notch fails closed without a sharp fallback OK")


def test_validation_matches_rendering():
    cons = make_cons(notch=55.0)
    try:
        pathplan.validate_profile(0.0, 0.0, 0.0, 1.0, cons)
    except pathplan.InfeasibleProfile:
        pass
    else:
        raise AssertionError("preflight accepted positive travel at zero speed")

    for vs, vc, ve, move_d in (
        (0.0, 100.0, 0.0, 10.0),
        (40.0, 120.0, 60.0, 20.0),
        (100.0, 100.0, 100.0, 2.0),
    ):
        pathplan.validate_profile(vs, vc, ve, move_d, cons)
        segs = pathplan._render_validated_profile(vs, vc, ve, move_d, cons)
        assert segs, (vs, vc, ve, move_d)
        check_segs(segs, move_d, vs, ve, "validated render")
    print("  successful preflight and rendering stay equivalent OK")


def test_render_refuses_an_unvalidated_infeasible_profile():
    # _render_validated_profile trusts its caller to have run validate_profile
    # first, and asserts if the profile turns out not to render. That guard was
    # never exercised: the existing test only walks the path where validation
    # PASSED. It matters because the failure it catches is the one that shut a
    # printer down -- the planner and the emitter disagreeing about the same
    # move -- and a guard nothing tests is a guard nobody knows still works.
    cons = make_cons(notch=55.0)
    # 275 -> 400 mm/s needs 0.5*(275+400)*(2/55) = 12.3 mm; give it 5 mm.
    vs, vc, ve, move_d = 275.009495, 400.0, 400.0, 5.000081
    try:
        pathplan.validate_profile(vs, vc, ve, move_d, cons)
    except pathplan.InfeasibleProfile:
        pass
    else:
        raise AssertionError(
            "premise broken: this profile is supposed to be infeasible"
        )
    try:
        pathplan._render_validated_profile(vs, vc, ve, move_d, cons)
    except AssertionError:
        pass
    else:
        raise AssertionError(
            "render silently accepted a profile validation would reject"
        )
    print("  rendering an unvalidated infeasible profile fails loudly OK")


def test_split_segments_never_empties_a_bucket():
    # _process_lookahead asserts that a validated span never splits into an
    # empty per-move bucket, because an empty bucket would emit a move with no
    # motion and desynchronise print_time from the trapq. Assert the property
    # directly, across uneven splits and a span whose ramp is far shorter than
    # the run, which is where a bucket could plausibly come up empty.
    cons = make_cons(notch=55.0)
    for vs, vc, ve, lengths in (
        (0.0, 200.0, 0.0, [5.0, 5.0, 5.0, 5.0]),
        (50.0, 300.0, 50.0, [0.2, 0.2, 30.0, 0.2, 0.2]),
        (100.0, 100.0, 100.0, [1.0, 0.05, 1.0]),
        (11.0, 220.0, 11.0, [0.2] * 40),
    ):
        move_d = math.fsum(lengths)
        pathplan.validate_profile(vs, vc, ve, move_d, cons)
        segs = pathplan._render_validated_profile(vs, vc, ve, move_d, cons)
        buckets = pathplan.split_segments(segs, lengths)
        assert len(buckets) == len(lengths)
        for i, (bucket, want_d) in enumerate(zip(buckets, lengths)):
            assert bucket, ("empty bucket", i, lengths)
            got_d = math.fsum(s[6] for s in bucket)
            assert abs(got_d - want_d) < 1e-9, (
                "bucket distance drifted",
                i,
                got_d,
                want_d,
            )
    print("  split_segments never produces an empty bucket OK")


def main():
    test_zero_is_parked()
    test_apeak_linear_in_dv()
    test_ramp_distance_law()
    test_lookahead_matches_emitter()
    test_lookahead_conservative_across_dt_and_saturation()
    test_reach_is_monotone_and_finite()
    test_emitted_zoh_notch_response()
    test_zoh_notch_error_scales_with_dt()
    test_notch_loss_reasons()
    test_loss_reasons_are_declared()
    test_excessive_slice_request_fails_before_integration()
    test_reach_runway_floor_short_circuit()
    test_infeasible_notch_fails_closed()
    test_validation_matches_rendering()
    test_notch_saturated_trapezoid_reaches_requested_peak()
    test_saturated_emitted_zoh_notch_response()
    test_notch_reach_uses_saturated_plateau()
    test_invariants_hold()
    test_chain_no_sharp_fallback()
    test_short_chain_degrades_cleanly()
    test_render_refuses_an_unvalidated_infeasible_profile()
    test_split_segments_never_empties_a_bucket()
    print("ALL PASS")


if __name__ == "__main__":
    main()

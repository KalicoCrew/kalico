# Tests for solving the emitted ramp's spectral null.
#
# The notch law's zero is exact only in continuous time. Emitting the ramp as
# constant-acceleration slices smears it to ~1.3% of dv at the default
# jerk_dt, and the only lever for that has been jerk_dt itself: halving it
# halves the residual and doubles the motion-queue traffic, bottoming out near
# 0.11% at 379 slices per ramp.
#
# solve_ramp_null instead solves the slice accelerations for the null, since
# the emitted spectrum is linear in them and there are far more of them than
# constraints. Sampling the ideal curve was only ever a convenient choice.
#
# Run: klippy-env/bin/python test/test_spectral_null.py
import cmath
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
import pathplan  # noqa: E402

F = 55.0
ACCEL = 20000.0


def ramp(vs, vc, jerk_dt=0.001, f2=None, a_const=ACCEL):
    cons = pathplan.Constraints(
        a_const=a_const,
        v_ceil=vc + 1.0,
        jerk_dt=jerk_dt,
        notch_freq=F,
        notch_freq2=f2,
    )
    return pathplan._ramp_up_jerk(vs, vc, cons)


def residual(slices, freq=F):
    """Normalized |A(freq)| of the emitted piecewise-constant acceleration."""
    w = 2.0 * math.pi * freq
    t = [0.0]
    for s in slices:
        t.append(t[-1] + s[0])
    acc = sum(
        s[5] * (cmath.exp(-1j * w * t[i]) - cmath.exp(-1j * w * t[i + 1]))
        / (1j * w)
        for i, s in enumerate(slices)
    )
    return abs(acc) / abs(sum(s[5] * s[0] for s in slices))


def test_unsaturated_ramps_null_exactly():
    for vs, vc in ((0.0, 20.0), (0.0, 100.0), (5.0, 10.0), (100.0, 120.0),
                   (200.0, 400.0)):
        slices, _d = ramp(vs, vc)
        solved = pathplan.solve_ramp_null(slices, [F], ACCEL)
        assert solved is not None, (vs, vc)
        assert residual(solved) < 1e-9, (vs, vc, residual(solved))
    print("  unsaturated ramps null exactly OK")


def test_null_holds_at_any_slice_count():
    # jerk_dt stops buying null depth: it only has to leave more unknowns than
    # constraints. That is what lets the planner run COARSER than today --
    # fewer slices is less motion-queue traffic -- with a better null.
    for jerk_dt in (0.0005, 0.001, 0.0018, 0.003):
        slices, _d = ramp(0.0, 100.0, jerk_dt=jerk_dt)
        solved = pathplan.solve_ramp_null(slices, [F], ACCEL)
        assert solved is not None, jerk_dt
        assert len(solved) == len(slices), jerk_dt
        assert residual(solved) < 1e-9, (jerk_dt, residual(solved))
    print("  the null holds at every slice count above the floor OK")


def test_preserves_dv_distance_and_exit_speed():
    # Non-negotiable. The lookahead sizes runway from the ramp's distance and
    # the next move plans from its exit speed; moving either invalidates every
    # reach calculation upstream.
    for vs, vc in ((0.0, 400.0), (0.0, 100.0), (100.0, 120.0), (0.0, 650.0)):
        slices, dist = ramp(vs, vc)
        dv = sum(s[5] * s[0] for s in slices)
        solved = pathplan.solve_ramp_null(slices, [F], ACCEL)
        assert solved is not None, (vs, vc)
        assert abs(sum(s[5] * s[0] for s in solved) - dv) < 1e-9 * abs(dv)
        assert abs(sum(s[6] for s in solved) - dist) < 1e-9 * dist
        assert abs(solved[-1][4] - slices[-1][4]) < 1e-9 * abs(dv)
        for a, b in zip(solved, solved[1:]):
            assert abs(a[4] - b[3]) < 1e-12, (vs, vc)
    print("  dv, distance and exit speed are preserved OK")


def test_saturated_ramps_take_a_partial_null():
    # A saturated ramp holds a plateau exactly on max_accel, so those slices
    # cannot move up. The correction is one vector in the null space of the dv
    # and distance rows, so its components cannot be scaled independently --
    # the plateau is pinned and the rest of the ramp carries what it can. The
    # answer must be a REDUCED residual, never a refusal and never a violated
    # acceleration limit.
    slices, _d = ramp(0.0, 400.0)
    before = residual(slices)
    solved = pathplan.solve_ramp_null(slices, [F], ACCEL)
    assert solved is not None
    after = residual(solved)
    assert after < before, (before, after)
    assert max(abs(s[5]) for s in solved) <= ACCEL * (1.0 + 1e-9)
    print("  saturated ramps take a partial null, never a violation OK")


def test_respects_max_accel_and_never_reverses():
    for vs, vc in ((0.0, 650.0), (0.0, 400.0), (0.0, 100.0), (50.0, 60.0)):
        slices, _d = ramp(vs, vc)
        solved = pathplan.solve_ramp_null(slices, [F], ACCEL)
        assert solved is not None, (vs, vc)
        for dt, ct, dtm, sv, cv, a, dist in solved:
            assert dt > 0.0 and ct == 0.0 and dtm == 0.0, (vs, vc)
            assert abs(a) <= ACCEL * (1.0 + 1e-9), (vs, vc, a)
            # Acceleration must not cross zero: past that the slice list is no
            # longer a ramp and stepcompress rejects the sequence.
            assert a >= -1e-9, (vs, vc, a)
            assert cv >= sv - 1e-9, (vs, vc)
            assert abs(0.5 * (sv + cv) * dt - dist) < 1e-9 * max(dist, 1.0)
    print("  respects max_accel and never reverses acceleration OK")


def test_two_zero_ramp_improves_both_modes():
    slices, _d = ramp(0.0, 400.0, f2=75.0)
    solved = pathplan.solve_ramp_null(slices, [55.0, 75.0], ACCEL)
    assert solved is not None
    for f in (55.0, 75.0):
        assert residual(solved, f) < residual(slices, f), f
    print("  a two-zero ramp improves at both modes OK")


def test_refuses_when_underdetermined():
    slices, _d = ramp(0.0, 100.0)
    assert pathplan.solve_ramp_null(slices[:4], [F]) is None
    assert pathplan.solve_ramp_null(slices[:6], [55.0, 75.0]) is None
    print("  refuses a ramp with fewer slices than constraints OK")


def test_returns_none_rather_than_raising():
    # The contract is that a bad request is answered with None so the caller
    # emits the uncorrected ramp. Raising would surface as an unhandled
    # exception out of the emitter, which is a shutdown.
    slices, _d = ramp(0.0, 100.0)
    # Duplicate rows: the two frequency pairs are identical, so the Gram is
    # singular and the rank guard has to catch it.
    assert pathplan.solve_ramp_null(slices, [55.0, 55.0]) is None
    # A zero or negative frequency has no spectral row -- w divides.
    assert pathplan.solve_ramp_null(slices, [55.0, 0.0]) is None
    assert pathplan.solve_ramp_null(slices, [0.0]) is None
    assert pathplan.solve_ramp_null(slices, [-55.0]) is None
    # A ramp that does not change speed has no dv to preserve.
    flat = [(0.001, 0.0, 0.0, 50.0, 50.0, 0.0, 0.05)] * 30
    assert pathplan.solve_ramp_null(flat, [F]) is None
    print("  answers bad requests with None instead of raising OK")


def test_emitter_is_inert_unless_enabled():
    # The flag must gate it completely: same profile, slice for slice.
    for vs, vc in ((0.0, 400.0), (0.0, 100.0)):
        off = pathplan.Constraints(
            a_const=ACCEL, v_ceil=vc + 1.0, jerk_dt=0.001, notch_freq=F
        )
        on = pathplan.Constraints(
            a_const=ACCEL,
            v_ceil=vc + 1.0,
            jerk_dt=0.001,
            notch_freq=F,
            spectral_null=True,
        )
        move_d = 4.0 * pathplan.notch_dist(vs, vc, ACCEL, F)
        base = pathplan.emit_profile(vs, vc, vs, move_d, off)
        tuned = pathplan.emit_profile(vs, vc, vs, move_d, on)
        assert base == pathplan.emit_profile(vs, vc, vs, move_d, off)
        assert tuned != base, (vs, vc, "flag had no effect")
        assert abs(sum(s[6] for s in tuned) - sum(s[6] for s in base)) < 1e-9
    print("  emitter is inert unless spectral_null is set OK")


def test_decel_ramps_carry_the_null_too():
    # A decel is emitted by time-reversing an accel-shaped ramp, so it is
    # solved in accel form and reversed afterwards -- time reversal preserves
    # the MAGNITUDE spectrum. Measure it in the decel tuple's own convention:
    # the time lives in slot 2, not slot 0, and reading slot 0 silently makes
    # every decel look perfect.
    for vs, vc, ve, move_d in ((0.0, 300.0, 0.0, 60.0), (0.0, 400.0, 100.0,
                                                         80.0)):
        for flag, want in ((False, None), (True, 1e-9)):
            cons = pathplan.Constraints(
                a_const=ACCEL,
                v_ceil=vc + 1.0,
                jerk_dt=0.001,
                notch_freq=F,
                spectral_null=flag,
            )
            segs = pathplan.emit_profile(vs, vc, ve, move_d, cons)
            dec = [s for s in segs if s[2] > 0.0]
            assert dec, (vs, vc, "no decel emitted")
            w = 2.0 * math.pi * F
            t = [0.0]
            for s in dec:
                t.append(t[-1] + s[2])
            acc = sum(
                s[5]
                * (cmath.exp(-1j * w * t[i]) - cmath.exp(-1j * w * t[i + 1]))
                / (1j * w)
                for i, s in enumerate(dec)
            )
            dv = math.fsum(s[5] * s[2] for s in dec)
            got = abs(acc) / abs(dv)
            if want is None:
                assert got > 1e-3, (vs, vc, "unsolved decel should not null")
            else:
                assert got < want, (vs, vc, got)
        # and the whole move still covers exactly the distance asked for
        assert abs(sum(s[6] for s in segs) - move_d) < 1e-9 * move_d
    print("  decel ramps carry the null and the move keeps its distance OK")


def test_bounded_jerk_amplification():
    # The correction is minimum-NORM in acceleration and says nothing about
    # adjacent slices, so unbounded it parks the zero by emitting acceleration
    # steps larger than the profile the planner asked for -- measured up to
    # 32%. Bound it boundary-inclusively: the pulse sits in zero acceleration
    # either side, and those two steps are the ones a correction enlarges most.
    def peak_jerk(sl):
        dt = [s[0] for s in sl]
        a = [s[5] for s in sl]
        pk = 0.0
        prev = 0.0
        for k in range(len(a) + 1):
            cur = a[k] if k < len(a) else 0.0
            span = dt[k] if k < len(a) else dt[-1]
            pk = max(pk, abs(cur - prev) / span)
            prev = cur
        return pk

    for vs, vc, f2 in ((0.0, 100.0, None), (0.0, 100.0, 75.0),
                       (0.0, 400.0, None), (100.0, 120.0, None)):
        slices, _d = ramp(vs, vc, f2=f2)
        freqs = [F] if f2 is None else [F, f2]
        solved = pathplan.solve_ramp_null(slices, freqs, ACCEL)
        if solved is None:
            continue
        ratio = peak_jerk(solved) / peak_jerk(slices)
        assert ratio <= pathplan.SPECTRAL_NULL_JERK_LIMIT + 1e-9, (
            vs, vc, f2, ratio)
    print("  jerk amplification stays inside the bound OK")


def test_saturation_cost_is_linear_not_cubic():
    # Pinned plateau slices are eliminated from the UNKNOWNS, not added as
    # equality rows. A row each would grow the Gram with the plateau, and a low
    # runtime M204 acceleration lengthens that without limit -- which made the
    # solve cubic in plateau length and could stall the planner mid-print.
    for a_const in (100000.0, 5000.0, 1000.0, 200.0):
        cons = pathplan.Constraints(
            a_const=a_const, v_ceil=401.0, jerk_dt=0.001, notch_freq=F
        )
        slices, _d = pathplan._ramp_up_jerk(0.0, 400.0, cons)
        rows, _t, a0, _dv = pathplan._null_rows(slices, [F])
        lim = a_const * (1.0 - 1e-9)
        pinned = sum(1 for a in a0 if abs(a) >= lim)
        solved = pathplan.solve_ramp_null(slices, [F], a_const)
        if solved is None:
            continue
        # The system solved must stay at 2 + 2*len(freqs) rows however much of
        # the ramp is pinned.
        assert len(rows) == 4, (a_const, len(rows))
        if a_const <= 1000.0:
            assert pinned > 100, (a_const, pinned, "expected heavy saturation")
    print("  saturated ramps keep the system at 4 rows OK")


def test_lost_nulls_are_reported():
    # A refused or scaled-back solve loses the null the user asked for. Losing
    # it silently is worse than not offering the option.
    cons = pathplan.Constraints(
        a_const=ACCEL, v_ceil=401.0, jerk_dt=0.001, notch_freq=F,
        spectral_null=True,
    )
    move_d = 4.0 * pathplan.notch_dist(0.0, 400.0, ACCEL, F)
    pathplan.emit_profile(0.0, 400.0, 0.0, move_d, cons)
    if cons.spectral_loss is not None:
        assert cons.spectral_loss in pathplan.LOSS_REASONS, cons.spectral_loss
        assert cons.spectral_loss in pathplan.notch_loss_reasons(
            0.0, 400.0, 0.0, cons)
    # Every reason the module can emit must be declared.
    for name in ("spectral_null_partial", "spectral_null_failed"):
        assert name in pathplan.LOSS_REASONS, name
    # A clean solve reports nothing.
    clean = pathplan.Constraints(
        a_const=ACCEL, v_ceil=101.0, jerk_dt=0.001, notch_freq=F,
        spectral_null=True,
    )
    pathplan.emit_profile(
        0.0, 100.0, 0.0, 4.0 * pathplan.notch_dist(0.0, 100.0, ACCEL, F), clean
    )
    assert clean.spectral_loss is None, clean.spectral_loss
    print("  a lost or partial null is reported, a clean one is not OK")


def main():
    test_unsaturated_ramps_null_exactly()
    test_null_holds_at_any_slice_count()
    test_preserves_dv_distance_and_exit_speed()
    test_saturated_ramps_take_a_partial_null()
    test_respects_max_accel_and_never_reverses()
    test_two_zero_ramp_improves_both_modes()
    test_refuses_when_underdetermined()
    test_returns_none_rather_than_raising()
    test_bounded_jerk_amplification()
    test_saturation_cost_is_linear_not_cubic()
    test_lost_nulls_are_reported()
    test_emitter_is_inert_unless_enabled()
    test_decel_ramps_carry_the_null_too()
    print("ALL PASS")


if __name__ == "__main__":
    main()

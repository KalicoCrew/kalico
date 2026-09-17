# Lightweight performance benchmark for dense jerk/notch planning.
#
# Run: klippy-env/bin/python test/test_pathplan_perf.py
import math
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(ROOT, "klippy", "extras"))
import pathplan  # noqa: E402
from test_pathplan import check_segs  # noqa: E402


def bench_reach(iterations=2000):
    start = time.perf_counter()
    acc = 0.0
    for i in range(iterations):
        v0 = (i % 180) * 0.7
        dist = 0.4 + (i % 37) * 0.18
        nf = 44.0 + (i % 5) * 3.0
        acc += pathplan.notch_reach_v2(
            v0 * v0, dist, 8000.0, 600.0, notch_freq=nf
        )
    return time.perf_counter() - start, acc


def _cons():
    return pathplan.Constraints(
        a_const=8000.0,
        v_ceil=600.0,
        jerk_dt=0.001,
        notch_freq=55.0,
    )


def bench_emit(iterations=400):
    cons = _cons()
    start = time.perf_counter()
    seg_count = 0
    dist_sum = 0.0
    for i in range(iterations):
        vs = float((i * 7) % 40)
        ve = float((i * 11) % 40)
        vc = max(vs, ve) + 20.0 + float(i % 120)
        move_d = 8.0 + float(i % 80) * 0.35
        segs = pathplan.emit_profile(vs, vc, ve, move_d, cons)
        assert segs
        check_segs(segs, move_d, vs, ve, "perf[%d]" % (i,))
        seg_count += len(segs)
        dist_sum += sum(s[6] for s in segs)
    return time.perf_counter() - start, seg_count, dist_sum


def bench_emit_short(iterations=400):
    # Exercise complete shaped accel/decel profiles that fit in less than one
    # millimetre. Infeasible profiles intentionally fail closed, so they are
    # not useful emitter benchmarks.
    cons = _cons()
    start = time.perf_counter()
    seg_count = 0
    for i in range(iterations):
        vs = ve = 0.0
        vc = 5.0 + float(i % 16)
        ramp_d = pathplan.notch_dist(0.0, vc, cons.a_const, cons.notch_freq)
        move_d = 2.0 * ramp_d + 0.01
        segs = pathplan.emit_profile(vs, vc, ve, move_d, cons)
        assert segs, (vs, vc, ve, move_d)
        check_segs(segs, move_d, vs, ve, "perf_short[%d]" % (i,))
        seg_count += len(segs)
    return time.perf_counter() - start, seg_count


def bench_loss_reasons(iterations=400):
    # The diagnostic the toolhead calls alongside emit_profile. It re-runs a
    # feasibility probe, so it must stay CHEAPER than the emit it annotates --
    # it used to cost ~1.7x emit_profile because it built and discarded a full
    # slice list.
    cons = _cons()
    start = time.perf_counter()
    count = 0
    for i in range(iterations):
        vs = float((i * 7) % 40)
        ve = float((i * 11) % 40)
        vc = max(vs, ve) + 20.0 + float(i % 120)
        move_d = 8.0 + float(i % 80) * 0.35
        reasons = pathplan.notch_loss_reasons(vs, vc, ve, cons)
        for r in reasons:
            assert r in pathplan.LOSS_REASONS, r
        count += len(reasons)
    return time.perf_counter() - start, count


def main():
    reach_t, reach_acc = bench_reach()
    emit_t, seg_count, dist_sum = bench_emit()
    short_t, short_segs = bench_emit_short()
    loss_t, loss_count = bench_loss_reasons()
    assert math.isfinite(reach_acc)
    assert seg_count > 0
    assert dist_sum > 0.0
    assert short_segs > 0
    print("reach: %.4fs for 2000 reach solves" % (reach_t,))
    print("emit:  %.4fs for 400 profiles (%d slices)" % (emit_t, seg_count))
    print(
        "short: %.4fs for 400 sub-mm profiles (%d slices)"
        % (short_t, short_segs)
    )
    print(
        "loss:  %.4fs for 400 diagnostics (%d reasons)" % (loss_t, loss_count)
    )
    # Loose guardrail: this is a regression tripwire, not a machine-specific
    # microbenchmark target.
    assert reach_t < 2.0, ("reach benchmark too slow", reach_t)
    assert emit_t < 5.0, ("emit benchmark too slow", emit_t)
    assert short_t < 8.0, ("short-move benchmark too slow", short_t)
    # Relative guardrail: the diagnostic must never again cost more than the
    # emit it describes.
    assert loss_t < emit_t, ("loss diagnostic slower than emit", loss_t, emit_t)
    print("ALL PASS")


if __name__ == "__main__":
    main()

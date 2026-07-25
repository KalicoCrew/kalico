# Lightweight performance benchmark for dense jerk/notch planning.
#
# Run: klippy-env/bin/python klippy/extras/test_pathplan_perf.py
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pathplan  # noqa: E402


def bench_reach(iterations=2000):
    start = time.perf_counter()
    acc = 0.0
    for i in range(iterations):
        v0 = (i % 180) * 0.7
        dist = 0.4 + (i % 37) * 0.18
        nf = 44.0 + (i % 5) * 3.0
        acc += pathplan.jerk_reach_v2(v0 * v0, dist, 8000.0, 250000.0,
                                      600.0, notch_freq=nf)
    return time.perf_counter() - start, acc


def bench_emit(iterations=400):
    cons = pathplan.Constraints(a_const=8000.0, v_ceil=600.0,
                                max_jerk=250000.0, jerk_dt=0.001,
                                notch_freq=55.0)
    start = time.perf_counter()
    seg_count = 0
    dist_sum = 0.0
    for i in range(iterations):
        vs = float((i * 17) % 120)
        ve = float((i * 11) % 100)
        vc = max(vs, ve) + 20.0 + float(i % 160)
        move_d = 0.8 + float(i % 80) * 0.25
        segs = pathplan.emit_profile(vs, vc, ve, move_d, cons)
        assert segs
        seg_count += len(segs)
        dist_sum += sum(s[6] for s in segs)
    return time.perf_counter() - start, seg_count, dist_sum


def main():
    reach_t, reach_acc = bench_reach()
    emit_t, seg_count, dist_sum = bench_emit()
    assert math.isfinite(reach_acc)
    assert seg_count > 0
    assert dist_sum > 0.0
    print("reach: %.4fs for 2000 reach solves" % (reach_t,))
    print("emit:  %.4fs for 400 profiles (%d slices)" % (emit_t, seg_count))
    # Loose guardrail: this is a regression tripwire, not a machine-specific
    # microbenchmark target.
    assert reach_t < 2.0, ("reach benchmark too slow", reach_t)
    assert emit_t < 5.0, ("emit benchmark too slow", emit_t)
    print("ALL PASS")


if __name__ == "__main__":
    main()

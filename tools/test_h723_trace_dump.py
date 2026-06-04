#!/usr/bin/env python3
# Surface C — H723 trace-dump comparison test.
#
# Per Step-5 plan Task 28. Drives kalico_load_curve + kalico_push_segment for
# each fixture in `rust/runtime/tests/fixtures/step5_segments.json`, collects
# kalico_trace responses (emitted by the MCU's runtime_drain task ~1 kHz),
# parses the binary TraceSample stream, and compares each motor channel
# against an analytical NURBS evaluation.
#
# Acceptance: max |motor_traced - motor_analytical| < TOLERANCE_MM (0.05 mm).
#
# Pre-flight: requires flashed H723 hardware (kalico runtime firmware).
# Hardware-deferred. The plot-output mentioned in the plan sketch is a TODO;
# the bring-up gate is the position-error check.
import argparse
import json
import logging
import pathlib
import struct
import sys
import time

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

# Hardware-deferred __main__ trace-dump test against a flashed H723 bench;
# no pytest test functions. Tagged needs_hardware so it is honestly excluded
# from CI. Run directly: `python3 <this file> ...`.
pytestmark = pytest.mark.needs_hardware

TOLERANCE_MM = 0.05
TRACE_SAMPLE_FMT = (
    "<QfffIB7x"  # u64 tick, 3×f32, u32 segment_id, u8 flags, 7 pad
)
TRACE_SAMPLE_SIZE = 32

# Current firmware ABI: scalar per-axis polynomial curve loads with no
# weights, followed by a four-handle segment push. The legacy Step-5 fixture
# file is still usable as source data, but rational weights are intentionally
# ignored to match what the MCU now evaluates.
FORMAT_VERSION_V1 = 1
UNUSED_HANDLE = 0xFFFEFFFE
E_MODE_TRAVEL = 2


# CoreXY+E kinematic transform (matches runtime/src/kinematics.rs).
def corexy_e(x, y, _z, e):
    return (x + y, x - y, e)


def floats_to_blob(values):
    raw = b"".join(struct.pack("<f", float(v)) for v in values)
    return raw.hex()


# --- Pure-Python NURBS evaluation (de Boor's algorithm). -------------------


def find_span(degree, knots, u):
    """Return knot span index k such that knots[k] <= u < knots[k+1]."""
    n = len(knots) - degree - 2
    if u >= knots[n + 1]:
        return n
    if u <= knots[degree]:
        return degree
    lo, hi = degree, n + 1
    mid = (lo + hi) // 2
    while u < knots[mid] or u >= knots[mid + 1]:
        if u < knots[mid]:
            hi = mid
        else:
            lo = mid
        mid = (lo + hi) // 2
    return mid


def basis_funcs(span, degree, knots, u):
    """Return list of (degree+1) basis-function values N[span-degree+i](u)."""
    N = [0.0] * (degree + 1)
    left = [0.0] * (degree + 1)
    right = [0.0] * (degree + 1)
    N[0] = 1.0
    for j in range(1, degree + 1):
        left[j] = u - knots[span + 1 - j]
        right[j] = knots[span + j] - u
        saved = 0.0
        for r in range(j):
            denom = right[r + 1] + left[j - r]
            temp = N[r] / denom if denom != 0.0 else 0.0
            N[r] = saved + right[r + 1] * temp
            saved = left[j - r] * temp
        N[j] = saved
    return N


def eval_scalar_bspline(degree, control_points, knots, u):
    """Evaluate the current scalar polynomial curve format."""
    span = find_span(degree, knots, u)
    N = basis_funcs(span, degree, knots, u)
    value = 0.0
    for i in range(degree + 1):
        idx = span - degree + i
        value += control_points[idx] * N[i]
    return value


# --- Bench harness ---------------------------------------------------------


def _axis_cps(fixture, axis):
    return [
        float(cp[axis]) if len(cp) > axis else 0.0
        for cp in fixture["control_points"]
    ]


def load_scalar_curve(io, fixture, axis, slot, timeout=3.0):
    cps = _axis_cps(fixture, axis)
    knots = list(fixture["knots"])
    degree = int(fixture["degree"])
    cmd = "kalico_load_curve version=%d slot=%d degree=%d cps=%s knots=%s" % (
        FORMAT_VERSION_V1,
        slot,
        degree,
        floats_to_blob(cps),
        floats_to_blob(knots),
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_load_curve_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_load_curve_response result=%s" % resp["result"]
        )
    return int(resp.get("curve_handle_packed", 0))


def load_fixture(io, fixture, base_slot=0, timeout=3.0):
    return {
        "x": load_scalar_curve(io, fixture, 0, base_slot, timeout),
        "y": load_scalar_curve(io, fixture, 1, base_slot + 1, timeout),
        "z": load_scalar_curve(io, fixture, 2, base_slot + 2, timeout),
        "e": UNUSED_HANDLE,
    }


def push_segment(io, seg_id, handles, t_start, t_end, kin=0, timeout=3.0):
    cmd = (
        "kalico_push_segment id=%d x_handle=%d y_handle=%d z_handle=%d "
        "e_handle=%d t_start_hi=%d t_start_lo=%d "
        "t_end_hi=%d t_end_lo=%d kinematics=%d e_mode=%d extrusion_ratio=%d"
        % (
            seg_id,
            handles["x"],
            handles["y"],
            handles["z"],
            handles["e"],
            (t_start >> 32) & 0xFFFFFFFF,
            t_start & 0xFFFFFFFF,
            (t_end >> 32) & 0xFFFFFFFF,
            t_end & 0xFFFFFFFF,
            kin,
            E_MODE_TRAVEL,
            0,
        )
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_push_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_push_response result=%s" % resp["result"]
        )


def drain_traces(io, duration_s, timeout=2.0):
    """Collect kalico_trace responses for `duration_s` plus a small grace."""
    deadline = time.monotonic() + duration_s + 0.500
    samples = []
    while time.monotonic() < deadline:
        try:
            resp = io.wait_for_response(
                "kalico_trace",
                timeout=min(timeout, max(0.05, deadline - time.monotonic())),
            )
        except HostIoError:
            continue
        count = int(resp["count"])
        data = resp["data"]
        if isinstance(data, str):
            data = data.encode("latin-1")
        # data length should be count * TRACE_SAMPLE_SIZE
        for i in range(count):
            chunk = data[i * TRACE_SAMPLE_SIZE : (i + 1) * TRACE_SAMPLE_SIZE]
            if len(chunk) != TRACE_SAMPLE_SIZE:
                break
            tick, ma, mb, me, sid, flags = struct.unpack(
                TRACE_SAMPLE_FMT, chunk
            )
            samples.append(
                {
                    "tick": tick,
                    "motor_a": ma,
                    "motor_b": mb,
                    "motor_e": me,
                    "segment_id": sid,
                    "flags": flags,
                }
            )
    return samples


def analytical_motor_at(fixture, t_start, t_end, tick):
    """Evaluate fixture geometry at a given MCU tick and return (motor_a/b/e)."""
    if tick <= t_start:
        u = 0.0
    elif tick >= t_end:
        u = 1.0
    else:
        u = (tick - t_start) / float(t_end - t_start)
    knots = fixture["knots"]
    # Map u in [0,1] to the active knot range [knots[degree], knots[n_knots-degree-1]].
    degree = int(fixture["degree"])
    u_min = knots[degree]
    u_max = knots[len(knots) - degree - 1]
    u_curve = u_min + u * (u_max - u_min)
    x = eval_scalar_bspline(degree, _axis_cps(fixture, 0), knots, u_curve)
    y = eval_scalar_bspline(degree, _axis_cps(fixture, 1), knots, u_curve)
    z = eval_scalar_bspline(degree, _axis_cps(fixture, 2), knots, u_curve)
    # E channel is zero for these fixtures (no E in the geometric data).
    return corexy_e(x, y, z, 0.0)


def compare(samples, fixture, t_start, t_end):
    max_err = 0.0
    n_compared = 0
    for s in samples:
        if s["segment_id"] != 0 and s["segment_id"] != int(
            fixture.get("segment_id", s["segment_id"])
        ):
            # In practice segment_id is set by push_segment caller; we don't
            # filter strictly here.
            pass
        if s["tick"] < t_start or s["tick"] > t_end:
            continue
        ma_an, mb_an, me_an = analytical_motor_at(
            fixture, t_start, t_end, s["tick"]
        )
        for traced, analytical in (
            (s["motor_a"], ma_an),
            (s["motor_b"], mb_an),
            (s["motor_e"], me_an),
        ):
            err = abs(traced - analytical)
            if err > max_err:
                max_err = err
        n_compared += 1
    return max_err, n_compared


def main():
    p = argparse.ArgumentParser(description="kalico H723 trace-dump test")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--fixtures",
        default=str(
            pathlib.Path(__file__).resolve().parent.parent
            / "rust/runtime/tests/fixtures/step5_segments.json"
        ),
    )
    p.add_argument("--clock-freq", type=int, default=180_000_000)
    p.add_argument("--tolerance-mm", type=float, default=TOLERANCE_MM)
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    fixtures = json.loads(pathlib.Path(args.fixtures).read_text())["fixtures"]
    print("Connecting to %s @ %d ..." % (args.port, args.baud))
    io = KalicoHostIO(args.port, args.baud)
    fail_count = 0
    try:
        # MCU clock ticks accumulate; we reset our t_start to a fresh chunk
        # for each fixture via a tick offset we track here.
        t_cursor = 0
        for idx, fx in enumerate(fixtures):
            slot = idx % 4  # runtime supports a small number of slots
            duration_us = int(fx["duration_us"])
            duration_ticks = int(duration_us * 1e-6 * args.clock_freq)
            print(
                "Fixture %d %r — %d µs (%d ticks)"
                % (idx, fx["name"], duration_us, duration_ticks)
            )
            handles = load_fixture(io, fx, base_slot=slot * 4)
            t_start = t_cursor + int(0.005 * args.clock_freq)  # +5 ms head-room
            t_end = t_start + duration_ticks
            push_segment(
                io,
                seg_id=idx + 1,
                handles=handles,
                t_start=t_start,
                t_end=t_end,
            )
            samples = drain_traces(io, duration_s=duration_us * 1e-6 + 0.050)
            max_err, n_compared = compare(samples, fx, t_start, t_end)
            print(
                "  collected %d trace samples; %d in-range; max error = %.6f mm"
                % (len(samples), n_compared, max_err)
            )
            if n_compared == 0:
                print(
                    "  WARN: no in-range samples — runtime may not be RUNNING"
                )
                fail_count += 1
                continue
            if max_err >= args.tolerance_mm:
                print(
                    "  FAIL: max_err %.6f mm >= tolerance %.6f mm"
                    % (max_err, args.tolerance_mm)
                )
                fail_count += 1
            else:
                print("  PASS")
            t_cursor = t_end + int(0.020 * args.clock_freq)  # 20 ms gap
        if fail_count:
            raise SystemExit("FAIL: %d fixture(s) failed" % fail_count)
        print("PASS (all %d fixtures)" % (len(fixtures),))
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()

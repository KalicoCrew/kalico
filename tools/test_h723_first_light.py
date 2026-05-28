#!/usr/bin/env python3
# Surface C — first-light test: validates IDLE → RUNNING transition on the
# H723 against the kalico runtime DECL_COMMAND surface.
#
# Per Step-5 plan Task 26. PASS/FAIL gate is the kalico_status response.
#
# Pre-flight: requires flashed H723 hardware and CONFIG_KALICO_RUNTIME=y.
# This script is hardware-deferred; it runs only when the user has the bench
# wired up.
import argparse
import json
import logging
import pathlib
import struct
import sys
import time

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import KalicoHostIO  # noqa: E402

# Hardware-deferred __main__ script: talks to a flashed H723 bench over
# serial. No pytest test functions. Tagged needs_hardware so it is honestly
# excluded from CI. Run directly: `python3 <this file> --serial ...`.
pytestmark = pytest.mark.needs_hardware

# Runtime status byte values per `runtime/src/engine.rs::RuntimeStatus`:
#   0 = IDLE, 1 = RUNNING, 2 = DRAINED, 3 = FAULT
# (No LOADED state — load_curve only populates the curve pool slot, doesn't
# transition the runtime state machine.)
STATUS_IDLE = 0
STATUS_RUNNING = 1
STATUS_DRAINED = 2
STATUS_FAULT = 3
STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "DRAINED", 3: "FAULT"}

# Current firmware ABI (Step 7-B): curve pool entries are scalar per-axis
# polynomial curves. `kalico_load_curve` no longer accepts weights, and
# `kalico_push_segment` references four packed handles instead of one curve.
FORMAT_VERSION_V1 = 1
UNUSED_HANDLE = 0xFFFEFFFE
E_MODE_INDEPENDENT = 1


def floats_to_blob(values):
    """Encode a list of f32 as a hex string for `%*s` PT_buffer."""
    raw = b"".join(struct.pack("<f", float(v)) for v in values)
    return raw.hex()


def query_status(io, timeout=2.0):
    io.send("runtime_query_status")
    resp = io.wait_for_response("kalico_status", timeout)
    return int(resp["status"]), int(resp["last_err"])


def expect_status(io, expected, timeout=2.0, label=""):
    status, last_err = query_status(io, timeout)
    name = STATUS_NAMES.get(status, "?%d" % status)
    expected_name = STATUS_NAMES.get(expected, "?%d" % expected)
    if status != expected:
        raise SystemExit(
            "FAIL%s: expected %s got %s (last_err=%d)"
            % (" " + label if label else "", expected_name, name, last_err)
        )
    return status, last_err


def _axis_cps(fixture, axis):
    return [
        float(cp[axis]) if len(cp) > axis else 0.0
        for cp in fixture["control_points"]
    ]


def load_scalar_curve(io, fixture, axis, slot, timeout=3.0):
    """Load one scalar axis curve and return firmware's packed handle."""
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


def load_first_fixture(io, fixture, base_slot=0, timeout=3.0):
    """Load X/Y plus a no-op independent-E curve; return handles and duration."""
    x_handle = load_scalar_curve(
        io, fixture, axis=0, slot=base_slot, timeout=timeout
    )
    y_handle = load_scalar_curve(
        io, fixture, axis=1, slot=base_slot + 1, timeout=timeout
    )
    # First-light intentionally uses Independent E, so E needs a real no-op
    # scalar curve handle rather than UNUSED_HANDLE. The fixture's E axis is
    # absent, so axis=3 materializes as all-zero control points.
    e_handle = load_scalar_curve(
        io, fixture, axis=3, slot=base_slot + 2, timeout=timeout
    )
    return {
        "x": x_handle,
        "y": y_handle,
        "z": UNUSED_HANDLE,
        "e": e_handle,
        "duration_us": int(fixture["duration_us"]),
    }


def read_mcu_clock(io, request_id=1, timeout=2.0):
    """Sample widened MCU clock via the §12.1 clock-sync request."""
    io.send(
        "kalico_clock_sync_request request_id=%d host_send_time_lo=0 "
        "host_send_time_hi=0" % request_id
    )
    resp = io.wait_for_response("kalico_clock_sync_response", timeout)
    lo = int(resp["mcu_clock_lo"]) & 0xFFFFFFFF
    hi = int(resp["mcu_clock_hi"]) & 0xFFFFFFFF
    return (hi << 32) | lo


def query_pool_state(io, slot, timeout=1.0):
    io.send("runtime_query_pool_state slot=%d" % slot)
    resp = io.wait_for_response("kalico_pool_state_response", timeout)
    return (
        int(resp["result"]),
        int(resp["current_gen"]),
        int(resp["last_retired_gen"]),
    )


def stream_open(io, stream_id=0, timeout=2.0):
    io.send("runtime_stream_open stream_id=%d" % stream_id)
    resp = io.wait_for_response("kalico_stream_open_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_stream_open_response result=%s" % resp["result"]
        )
    return int(resp.get("credit_epoch", 0))


def stream_arm(io, t_start, arm_lead_cycles, timeout=2.0):
    cmd = (
        "runtime_stream_arm t_start_t0_lo=%d t_start_t0_hi=%d "
        "arm_lead_cycles=%d"
        % (t_start & 0xFFFFFFFF, (t_start >> 32) & 0xFFFFFFFF, arm_lead_cycles)
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_stream_arm_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_stream_arm_response result=%s" % resp["result"]
        )
    armed_lo = int(resp["armed_t_start_lo"]) & 0xFFFFFFFF
    armed_hi = int(resp["armed_t_start_hi"]) & 0xFFFFFFFF
    return (armed_hi << 32) | armed_lo


def stream_terminal(io, segment_id, timeout=2.0):
    io.send("runtime_stream_terminal segment_id=%d" % segment_id)
    resp = io.wait_for_response("kalico_stream_terminal_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_stream_terminal_response result=%s" % resp["result"]
        )


def stream_flush(io, timeout=2.0):
    io.send("runtime_stream_flush")
    resp = io.wait_for_response("kalico_stream_flush_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_stream_flush_response result=%s" % resp["result"]
        )
    return int(resp.get("credit_epoch", 0))


def push_segment(
    io,
    seg_id,
    handles,
    duration_us,
    kin=0,
    timeout=3.0,
    t_start_ticks=0,
    clock_freq=180_000_000,
):
    """Push a single segment covering `duration_us` of curve-time."""
    # t_start/t_end are MCU ticks (u64). The runtime expects monotonic.
    duration_ticks = int(duration_us * 1e-6 * clock_freq)
    t_end = t_start_ticks + duration_ticks
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
            (t_start_ticks >> 32) & 0xFFFFFFFF,
            t_start_ticks & 0xFFFFFFFF,
            (t_end >> 32) & 0xFFFFFFFF,
            t_end & 0xFFFFFFFF,
            kin,
            E_MODE_INDEPENDENT,
            0,  # f32::to_bits(0.0); ignored for Independent E mode.
        )
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_push_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit(
            "FAIL: kalico_push_response result=%s" % resp["result"]
        )
    return t_end


def main():
    p = argparse.ArgumentParser(description="kalico H723 first-light test")
    p.add_argument(
        "--port", required=True, help="serial device, e.g. /dev/ttyACM0"
    )
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--fixtures",
        default=str(
            pathlib.Path(__file__).resolve().parent.parent
            / "rust/runtime/tests/fixtures/step5_segments.json"
        ),
    )
    p.add_argument(
        "--clock-freq",
        type=int,
        default=520_000_000,
        help="MCU CLOCK_FREQ; H723 Klipper Kconfig default is 520 MHz",
    )
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    fixtures = json.loads(pathlib.Path(args.fixtures).read_text())["fixtures"]
    if not fixtures:
        raise SystemExit("FAIL: no fixtures in %s" % args.fixtures)

    print("Connecting to %s @ %d ..." % (args.port, args.baud))
    io = KalicoHostIO(args.port, args.baud)
    try:
        # Step 1: status must be IDLE before any commands.
        status, last_err = expect_status(io, STATUS_IDLE, label="(initial)")
        print(
            "  initial: status=%s last_err=%d"
            % (STATUS_NAMES[status], last_err)
        )

        # Step 2: probe the slots we're about to use; if any are still
        # occupied from a prior run, fail fast with a clear message rather
        # than the opaque load_curve result=-3. (Pool re-use across runs is
        # out of scope for first-light; power-cycle the MCU between runs.)
        for slot in (0, 1, 2):
            r, cur_gen, last_ret = query_pool_state(io, slot)
            if r != 0 or cur_gen != last_ret:
                raise SystemExit(
                    "FAIL: pool slot %d not free (result=%d current_gen=%d "
                    "last_retired_gen=%d) — power-cycle the H723 between runs"
                    % (slot, r, cur_gen, last_ret)
                )

        # Step 3: load curve into slots 0/1/2. The curve pool is independent
        # of the runtime state machine — load_curve populates a slot but
        # does not transition status, so it stays at IDLE.
        fx = fixtures[0]
        handles = load_first_fixture(io, fx, base_slot=0)
        duration_us = handles["duration_us"]
        print("  loaded curve %r (%d us)" % (fx["name"], duration_us))
        expect_status(io, STATUS_IDLE, label="(post-load)")

        # Step 4: open the stream.
        stream_open(io, stream_id=0)
        print("  stream_open ok (stream_id=0)")

        # Step 5: pick t_start. `widened_now` is only published by the
        # engine ISR (which TIM5 starts on first push), so a pre-push
        # `kalico_clock_sync_request` returns 0 — we can't sample a real
        # MCU clock here. Instead we choose an absolute `t_start` large
        # enough to be in the MCU's future even after seconds of uptime.
        # 10 s × clock_freq ≈ 5.2e9 ticks at 520 MHz: fine for fresh-boot
        # bring-up where the bench is power-cycled before the test.
        # `arm_lead_cycles` is the engine's arm-lead requirement (1 ms).
        t_start_offset_s = 10.0
        t_start = int(t_start_offset_s * args.clock_freq)
        arm_lead_cycles = int(0.001 * args.clock_freq)
        push_segment(
            io,
            seg_id=1,
            handles=handles,
            duration_us=duration_us,
            t_start_ticks=t_start,
            clock_freq=args.clock_freq,
        )
        print(
            "  pushed seg 1 (t_start=%d, ~%.1f s of MCU uptime)"
            % (t_start, t_start_offset_s)
        )

        # Step 6: arm. StreamOpenPriming → Armed; engine ISR will flip to
        # Running once `now >= t_start`.
        stream_arm(io, t_start=t_start, arm_lead_cycles=arm_lead_cycles)
        print("  stream_arm ok (arm_lead_cycles=%d)" % arm_lead_cycles)

        # Step 7: mark seg 1 as the terminal segment. Allowed in Armed;
        # transitions to Draining so the engine returns to Idle cleanly
        # once seg 1 retires (rather than tripping Underrun).
        stream_terminal(io, segment_id=1)
        print("  stream_terminal ok (segment_id=1)")

        # Step 8: poll for RUNNING / DRAINED. Engine fires once widened_now
        # reaches t_start, so the worst-case wait is ~t_start_offset_s.
        deadline = (
            time.monotonic() + t_start_offset_s + (duration_us * 1e-6) + 1.0
        )
        observed_running = False
        observed_drained = False
        last_status = None
        last_last_err = 0
        while time.monotonic() < deadline:
            status, last_err = query_status(io, timeout=1.0)
            last_status, last_last_err = status, last_err
            if status == STATUS_FAULT:
                raise SystemExit(
                    "FAIL: ran into FAULT (last_err=%d)" % last_err
                )
            if status == STATUS_RUNNING:
                observed_running = True
            if status == STATUS_DRAINED:
                observed_drained = True
                break
            time.sleep(0.002)
        if not observed_drained:
            raise SystemExit(
                "FAIL: never reached DRAINED (last status=%s last_err=%d, "
                "observed_running=%s)"
                % (
                    STATUS_NAMES.get(last_status, last_status),
                    last_last_err,
                    observed_running,
                )
            )
        print(
            "  drain: observed_running=%s observed_drained=%s last_err=%d"
            % (observed_running, observed_drained, last_last_err)
        )

        # No final flush: once the engine drains naturally (terminal
        # segment retires), runtime_tick.c disables TIM5 to save CPU.
        # Issuing `runtime_stream_flush` after that point trips
        # KALICO_ERR_LIVENESS_STALLED (-132) because the §8.5 force_idle
        # handshake needs an active ISR to ack within 1 ms. DRAINED is
        # itself the clean end-state for the happy path; flush is for
        # mid-stream aborts.

        print("PASS")
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()

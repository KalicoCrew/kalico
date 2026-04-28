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

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import KalicoHostIO, HostIoError  # noqa: E402

# Runtime status byte values per `runtime/src/engine.rs::RuntimeStatus`:
#   0 = IDLE, 1 = RUNNING, 2 = DRAINED, 3 = FAULT
# (No LOADED state — load_curve only populates the curve pool slot, doesn't
# transition the runtime state machine.)
STATUS_IDLE = 0
STATUS_RUNNING = 1
STATUS_DRAINED = 2
STATUS_FAULT = 3
STATUS_NAMES = {0: "IDLE", 1: "RUNNING", 2: "DRAINED", 3: "FAULT"}


def floats_to_blob(values):
    """Encode a list of f32 as a hex string for `%*s` PT_buffer."""
    raw = b"".join(struct.pack("<f", float(v)) for v in values)
    return raw.hex()


def query_status(io, timeout=2.0):
    io.send("kalico_query_status")
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


def load_first_fixture(io, fixture, slot=0, timeout=3.0):
    """Push `kalico_load_curve` for one fixture; return its duration_us."""
    cps = []
    for cp in fixture["control_points"]:
        cps.extend(cp)
        if len(cp) == 3:
            pass  # already xyz
    knots = list(fixture["knots"])
    weights = list(fixture["weights"])
    degree = int(fixture["degree"])
    # n_cp / n_knots are derived MCU-side from the %*s blob byte-lengths
    # (12 bytes per cp, 4 bytes per knot/weight). The format string carries
    # only `slot`, `degree`, and the three blobs.
    cmd = (
        "kalico_load_curve slot=%d degree=%d "
        "cps=%s knots=%s weights=%s"
        % (slot, degree,
           floats_to_blob(cps), floats_to_blob(knots), floats_to_blob(weights))
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_load_curve_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit("FAIL: kalico_load_curve_response result=%s" % resp["result"])
    return int(fixture["duration_us"])


def push_segment(io, seg_id, slot, duration_us, kin=0, timeout=3.0,
                 t_start_ticks=0, clock_freq=180_000_000):
    """Push a single segment covering `duration_us` of curve-time."""
    # t_start/t_end are MCU ticks (u64). The runtime expects monotonic.
    duration_ticks = int(duration_us * 1e-6 * clock_freq)
    t_end = t_start_ticks + duration_ticks
    cmd = (
        "kalico_push_segment id=%d curve=%d t_start_hi=%d t_start_lo=%d "
        "t_end_hi=%d t_end_lo=%d kinematics=%d"
        % (seg_id, slot,
           (t_start_ticks >> 32) & 0xFFFFFFFF, t_start_ticks & 0xFFFFFFFF,
           (t_end >> 32) & 0xFFFFFFFF, t_end & 0xFFFFFFFF,
           kin)
    )
    io.send(cmd)
    resp = io.wait_for_response("kalico_push_response", timeout)
    if int(resp["result"]) != 0:
        raise SystemExit("FAIL: kalico_push_response result=%s" % resp["result"])
    return t_end


def main():
    p = argparse.ArgumentParser(description="kalico H723 first-light test")
    p.add_argument("--port", required=True, help="serial device, e.g. /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--fixtures",
        default=str(pathlib.Path(__file__).resolve().parent.parent /
                    "rust/runtime/tests/fixtures/step5_segments.json"),
    )
    p.add_argument("--clock-freq", type=int, default=520_000_000,
                   help="MCU CLOCK_FREQ; H723 Klipper Kconfig default is 520 MHz")
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
        print("  initial: status=%s last_err=%d" % (STATUS_NAMES[status], last_err))

        # Step 2: load curve into slot 0. The curve pool is independent of
        # the runtime state machine — load_curve populates a slot but does
        # not transition status, so it stays at IDLE.
        fx = fixtures[0]
        duration_us = load_first_fixture(io, fx, slot=0)
        print("  loaded curve %r (%d us)" % (fx["name"], duration_us))
        status, last_err = query_status(io, timeout=1.0)
        if status != STATUS_IDLE:
            raise SystemExit(
                "FAIL: post-load status=%s last_err=%d (expected IDLE)"
                % (STATUS_NAMES.get(status, status), last_err)
            )
        print("  post-load: status=%s last_err=%d" % (STATUS_NAMES[status], last_err))

        # Step 3: push a segment → RUNNING (mid-segment) or DRAINED (queue
        # exhausted). With t_start=0 sent verbatim and the widened CYCCNT
        # well past the segment duration on the first ISR fire, the engine
        # transitions Idle→Running→Drained inside a single tick, so the
        # host typically sees DRAINED here. Both are valid PASS states.
        push_segment(io, seg_id=1, slot=0, duration_us=duration_us,
                     clock_freq=args.clock_freq)
        # Give the MCU one drain tick (~1 ms) to advance the state machine.
        time.sleep(0.005)
        status, last_err = query_status(io, timeout=1.0)
        if status not in (STATUS_RUNNING, STATUS_DRAINED):
            raise SystemExit(
                "FAIL: post-push status=%s last_err=%d (expected RUNNING or DRAINED)"
                % (STATUS_NAMES.get(status, status), last_err)
            )
        print("  post-push: status=%s last_err=%d" % (STATUS_NAMES[status], last_err))

        # Step 4: wait out the segment + a small grace period; status must NOT
        # be FAULT, and last_err must be zero.
        time.sleep(duration_us * 1e-6 + 0.020)
        status, last_err = query_status(io, timeout=1.0)
        if status == STATUS_FAULT:
            raise SystemExit(
                "FAIL: ran into FAULT (last_err=%d)" % (last_err,))
        if last_err != 0:
            raise SystemExit(
                "FAIL: last_err=%d (status=%s)"
                % (last_err, STATUS_NAMES.get(status, status))
            )
        print("  post-run: status=%s last_err=%d"
              % (STATUS_NAMES.get(status, status), last_err))

        print("PASS")
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()

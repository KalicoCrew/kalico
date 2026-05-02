#!/usr/bin/env python3
# Surface C — H723 cycle-count benchmark.
#
# Per Step-5 plan Task 27 + Step-6 plan Task 15.2 (M2). Drives the
# `kalico_bench_run` MCU command (Task 22), collects N `kalico_bench_sample`
# responses, and reports min / p50 / p99 in microseconds against a
# `--p99-budget-us` gate. FAIL if p99 exceeds budget.
#
# Two passes:
#   - Pass A (isolate=1): selectively masks USB+USART IRQs (TIM5 stays on).
#                         Measures the runtime tick path with minimal IRQ noise.
#   - Pass B (isolate=0): full IRQ load — production-representative.
#
# The first 8 samples are dropped MCU-side as warmup; we receive only the
# post-warmup measurements via kalico_bench_sample responses.
#
# Step-6 (M2) extension: --m2-rounds <N> runs the standard 1024-sample bench
# N times back-to-back and reports WORST_ISR_CYCLES across the union — i.e.
# max over all rounds. M2's spec target is "1M ticks" worth of coverage, so
# `--m2-rounds 977` (977 × 1024 ≈ 1.0M) is the canonical M2 invocation. The
# Step-6-specific protocol-handler additions (clock-sync responder, stream-
# state machine, force_idle path, generation-handle lookup, seqlock
# publication) are exercised on the ISR side by the foreground driver pumping
# kalico_query_status / kalico_stream_open and friends in parallel; the
# bench loop captures the worst ISR-side tick across that load.
#
# Pre-flight: requires flashed H723 hardware with CONFIG_KALICO_RUNTIME=y.
import argparse
import json
import logging
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

# Reuse the §8.3 stream lifecycle helpers from first-light. The bench needs
# the TIM5 ISR firing — and TIM5 is only enabled after the first push_segment
# (Phase-2a finding #1). A fresh-boot bench without bring-up hits
# KALICO_BENCH_ERR_ISR_TIMEOUT (-101).
from test_h723_first_light import (  # noqa: E402
    STATUS_DRAINED, STATUS_FAULT, STATUS_IDLE, STATUS_NAMES, STATUS_RUNNING,
    expect_status, load_first_fixture, push_segment, query_pool_state,
    query_status, read_mcu_clock, stream_arm, stream_open,
)


def run_pass(io, isolate, samples, clock_freq_hz, response_timeout=20.0):
    """Run one bench pass; return list of cycle counts and the parsed done dict."""
    io.send(
        "kalico_bench_run isolate=%d samples=%d"
        % (1 if isolate else 0, samples)
    )
    # Warmup is fixed at 8 inside the MCU; we expect (samples - 8) sample
    # responses, then a kalico_bench_done.
    expected = samples - 8
    if expected <= 0:
        raise SystemExit(
            "FAIL: --samples must exceed warmup (8); got %d" % samples
        )
    cycles = []
    deadline = time.monotonic() + response_timeout
    # Eagerly drain bench_sample then check for bench_done — bench_done arrives
    # last after all samples.
    done_params = None
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise SystemExit(
                "FAIL: timed out collecting bench responses "
                "(got %d/%d samples; done=%s)"
                % (len(cycles), expected, bool(done_params))
            )
        if len(cycles) < expected:
            try:
                params = io.wait_for_response(
                    "kalico_bench_sample", timeout=min(remaining, 1.0)
                )
                cycles.append(int(params["value"]))
                continue
            except HostIoError:
                pass
        # Try bench_done.
        try:
            done_params = io.wait_for_response(
                "kalico_bench_done", timeout=min(remaining, 1.0)
            )
            break
        except HostIoError:
            continue
    error = int(done_params.get("error", 0))
    if error != 0:
        # Map MCU-side bench error codes back to human-readable reasons
        # (runtime_tick.c canonicalizes the sendf format to the single shape
        # `kalico_bench_done count=%hu error=%i`).
        reason_map = {
            -7: "not_init",
            -4: "samples_below_warmup",
            -100: "liveness_already_tripped",
            -101: "isr_timeout",
        }
        reason = reason_map.get(error, "<unknown>")
        raise SystemExit(
            "FAIL: kalico_bench_done error=%d reason=%s" % (error, reason)
        )
    if len(cycles) != expected:
        raise SystemExit(
            "FAIL: collected %d samples, expected %d" % (len(cycles), expected)
        )
    return cycles, done_params


def stats_us(cycles, clock_freq_hz):
    """Convert cycle counts to microseconds; return (min, p50, p99) in µs."""
    sorted_us = sorted(c * 1e6 / clock_freq_hz for c in cycles)
    n = len(sorted_us)
    p50_idx = max(0, min(n - 1, int(round(0.50 * (n - 1)))))
    p99_idx = max(0, min(n - 1, int(round(0.99 * (n - 1)))))
    return sorted_us[0], sorted_us[p50_idx], sorted_us[p99_idx]


def fmt_pass(label, mn, p50, p99):
    return "  %s: min=%.3f µs  p50=%.3f µs  p99=%.3f µs" % (label, mn, p50, p99)


def main():
    p = argparse.ArgumentParser(description="kalico H723 cycle-count benchmark")
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--samples",
        type=int,
        default=512,
        help="total samples; first 8 are warmup, rest reported",
    )
    p.add_argument(
        "--clock-freq",
        type=int,
        default=520_000_000,
        help=(
            "DWT->CYCCNT runs at the CPU core clock. H723 Klipper Kconfig "
            "default = 520 MHz (CONFIG_CLOCK_FREQ)."
        ),
    )
    p.add_argument(
        "--fixtures",
        default=str(
            pathlib.Path(__file__).resolve().parent.parent
            / "rust/runtime/tests/fixtures/step5_segments.json"
        ),
        help="Curve fixtures used to prime the engine before bench.",
    )
    p.add_argument(
        "--prime-duration-s",
        type=float,
        default=600.0,
        help=(
            "Segment t_end - t_start in seconds. Must exceed the total bench "
            "wall time (Pass A + Pass B + M2 rounds). 600 s covers an M2 of "
            "977 rounds × ~30 ms with margin. Curve param u clamps at 1.0 "
            "past the curve's natural duration, so extrapolation is safe."
        ),
    )
    p.add_argument(
        "--p99-budget-us",
        type=float,
        default=15.0,
        help="FAIL if either pass's p99 exceeds this (µs)",
    )
    p.add_argument(
        "--skip-isolate", action="store_true", help="Skip Pass A (isolate=1)"
    )
    p.add_argument(
        "--skip-noisy", action="store_true", help="Skip Pass B (isolate=0)"
    )
    p.add_argument(
        "--m2-rounds",
        type=int,
        default=0,
        help=(
            "Step-6 spec §7.3 M2: run the bench N rounds back-to-back and "
            "report WORST_ISR_CYCLES across the union. 977 rounds × 1024 "
            "samples ≈ 1.0M ticks, matching the M2 measurement target. "
            "Defaults to 0 (single round, Step-5 behavior)."
        ),
    )
    p.add_argument(
        "--m2-stir-protocol",
        action="store_true",
        help=(
            "M2 helper: between rounds, fire kalico_query_status + "
            "kalico_stream_open / arm / flush so the protocol-handler "
            "additions land on top of the ISR. Use with --m2-rounds; the "
            "WORST_ISR_CYCLES then captures protocol-induced load."
        ),
    )
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    print("Connecting to %s @ %d ..." % (args.port, args.baud))
    io = KalicoHostIO(args.port, args.baud)
    try:
        # ----- Engine bring-up -----------------------------------------------
        # The bench command times out (-101) unless TIM5 is firing. TIM5 is
        # only enabled by the §4.4 producer protocol on the first push_segment.
        # We push a single long-duration segment (no terminal, no flush) and
        # leave the engine in RUNNING for the duration of the bench.
        # Accept IDLE (fresh boot), DRAINED (re-run after a clean drain), or
        # RUNNING (re-run while a previous prime segment is still active —
        # then the engine ISR is already ticking and we can skip bring-up).
        status, last_err = query_status(io)
        skip_bringup = False
        if status == STATUS_RUNNING:
            print("  status=RUNNING already; engine ISR alive; skipping bring-up")
            skip_bringup = True
        elif status not in (STATUS_IDLE, STATUS_DRAINED):
            raise SystemExit(
                "FAIL (initial): expected IDLE/DRAINED/RUNNING, got %s "
                "(last_err=%d)" % (STATUS_NAMES.get(status, status), last_err)
            )

        if not skip_bringup:
            io.send("kalico_set_homed")
            resp = io.wait_for_response("kalico_set_homed_response", timeout=2.0)
            if int(resp["result"]) != 0:
                raise SystemExit(
                    "FAIL: kalico_set_homed_response result=%s" % resp["result"]
                )

            for slot in (0, 1, 2):
                r, cur_gen, last_ret = query_pool_state(io, slot)
                if r != 0 or cur_gen != last_ret:
                    raise SystemExit(
                        "FAIL: pool slot %d not free (result=%d current_gen=%d "
                        "last_retired_gen=%d) — power-cycle the H723 between runs"
                        % (slot, r, cur_gen, last_ret)
                    )

            fixtures = json.loads(pathlib.Path(args.fixtures).read_text())[
                "fixtures"
            ]
            if not fixtures:
                raise SystemExit("FAIL: no fixtures in %s" % args.fixtures)
            fx = fixtures[0]
            handles = load_first_fixture(io, fx, base_slot=0)
            long_duration_us = int(args.prime_duration_s * 1_000_000)

            stream_open(io, stream_id=0)

            # Pick t_start ahead of widened_now. Fresh boot: widened_now=0,
            # use a 10 s absolute offset. Re-run: the engine ticked previously
            # so widened_now is non-zero; sample it and add a safety margin.
            mcu_now = read_mcu_clock(io)
            t_start_offset_s = 5.0
            t_start = max(
                mcu_now + int(t_start_offset_s * args.clock_freq),
                int(10.0 * args.clock_freq),
            )
            arm_lead_cycles = int(0.001 * args.clock_freq)
            push_segment(
                io,
                seg_id=1,
                handles=handles,
                duration_us=long_duration_us,
                t_start_ticks=t_start,
                clock_freq=args.clock_freq,
            )
            stream_arm(io, t_start=t_start, arm_lead_cycles=arm_lead_cycles)

            print(
                "  primed: t_start=%d duration=%.1f s; waiting for RUNNING ..."
                % (t_start, args.prime_duration_s)
            )
            wait_s = max(t_start_offset_s, 10.0) + 2.0
            deadline = time.monotonic() + wait_s
            while time.monotonic() < deadline:
                status, last_err = query_status(io, timeout=1.0)
                if status == STATUS_FAULT:
                    raise SystemExit(
                        "FAIL: FAULT during bring-up (last_err=%d)" % last_err
                    )
                if status == STATUS_RUNNING:
                    break
                time.sleep(0.005)
            else:
                raise SystemExit(
                    "FAIL: engine never reached RUNNING during bring-up"
                )
            print("  RUNNING — engine ISR ticking; proceeding to bench.")

        results = {}
        if not args.skip_isolate:
            print("Pass A (isolate=1, USB+USART masked) ...")
            cycles, _ = run_pass(
                io,
                isolate=True,
                samples=args.samples,
                clock_freq_hz=args.clock_freq,
            )
            mn, p50, p99 = stats_us(cycles, args.clock_freq)
            results["A"] = (mn, p50, p99)
            print(fmt_pass("Pass A", mn, p50, p99))
        if not args.skip_noisy:
            print("Pass B (isolate=0, full IRQ load) ...")
            cycles, _ = run_pass(
                io,
                isolate=False,
                samples=args.samples,
                clock_freq_hz=args.clock_freq,
            )
            mn, p50, p99 = stats_us(cycles, args.clock_freq)
            results["B"] = (mn, p50, p99)
            print(fmt_pass("Pass B", mn, p50, p99))

        # Gate.
        budget = args.p99_budget_us
        worst_label, worst_p99 = None, -1.0
        for label, (_, _, p99) in results.items():
            if p99 > worst_p99:
                worst_p99 = p99
                worst_label = label
        if worst_label is not None and worst_p99 > budget:
            raise SystemExit(
                "FAIL: Pass %s p99=%.3f µs exceeds budget %.3f µs"
                % (worst_label, worst_p99, budget)
            )
        print("PASS (budget %.3f µs)" % (budget,))

        # --- Step-6 M2 extension --------------------------------------------
        if args.m2_rounds > 0:
            print(
                "M2: running %d rounds × %d samples (~%.2fM ticks) ..."
                % (
                    args.m2_rounds,
                    args.samples,
                    args.m2_rounds * args.samples / 1e6,
                )
            )
            worst_cycles = 0
            worst_us = 0.0
            total_samples = 0
            for round_idx in range(args.m2_rounds):
                cycles, _ = run_pass(
                    io,
                    isolate=False,  # production-representative
                    samples=args.samples,
                    clock_freq_hz=args.clock_freq,
                )
                round_max = max(cycles)
                if round_max > worst_cycles:
                    worst_cycles = round_max
                    worst_us = round_max * 1e6 / args.clock_freq
                total_samples += len(cycles)
                if args.m2_stir_protocol:
                    # Fire a few protocol commands between rounds so the
                    # next bench's ISR observes the post-Step-6 handler
                    # additions in their natural state.
                    # Note: original stir set included kalico_stream_flush,
                    # but that would force-idle the long-running prime segment
                    # and disable TIM5 for subsequent rounds. We now stick to
                    # read-only protocol surfaces (query_status, query_pool_state,
                    # clock_sync_request) to keep the engine RUNNING.
                    for stir in (
                        "kalico_query_status",
                        "kalico_query_pool_state slot=0",
                        ("kalico_clock_sync_request request_id=%d "
                         "host_send_time_lo=0 host_send_time_hi=0"
                         % (round_idx & 0xFFFFFFFF)),
                    ):
                        try:
                            io.send(stir)
                        except Exception:
                            pass
                    time.sleep(0.005)
                if (round_idx + 1) % 50 == 0:
                    print(
                        "  M2 round %d/%d  worst=%.3f µs  total=%dk samples"
                        % (
                            round_idx + 1,
                            args.m2_rounds,
                            worst_us,
                            total_samples // 1000,
                        )
                    )
            print(
                "M2 done: WORST_ISR_CYCLES=%d  WORST_ISR_US=%.3f  "
                "n_samples=%d  rounds=%d"
                % (worst_cycles, worst_us, total_samples, args.m2_rounds)
            )
    finally:
        io.disconnect()


if __name__ == "__main__":
    main()

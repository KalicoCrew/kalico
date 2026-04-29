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
import logging
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402


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
        default=180_000_000,
        help="DWT->CYCCNT runs at the core clock (default 180 MHz)",
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
                % (args.m2_rounds, args.samples,
                   args.m2_rounds * args.samples / 1e6)
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
                    for stir in (
                        "kalico_query_status",
                        "kalico_stream_open stream_id=999",
                        "kalico_stream_flush",
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
                            round_idx + 1, args.m2_rounds,
                            worst_us, total_samples // 1000,
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

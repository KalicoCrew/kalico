#!/usr/bin/env python3
"""tools/measure_m1_host_stall.py — Spec §7.3 M1 host-stall soak runner.

**Status: SCAFFOLD — user-run on Pi 5 hardware.** Per the Step-6 plan
(Phase 15.1), the subagent ships this script and a placeholder doc;
the actual 8-hour soak runs on the user's Pi 5 against a representative
production load. Results land in
`docs/research/step6-buffer-budget-measurements.md`.

What this measures
==================

M1 (spec §7.3) bounds the worst-case host stall — the maximum duration
between segment-push-completion events as observed by the producer side
of the SPSC. Goal: validate that the buffer budget chosen in
`rust/kalico-host-rt/src/...` (default ~ MIN_SEGMENT_DURATION_MS × N)
is large enough to absorb the actual host-side jitter on a Pi 5 8 GB
running Bookworm desktop + Mainsail (rendering an active trace UI) +
Moonraker WebSocket + journald active logging.

The script measures wall-clock time between consecutive successful
`kalico_push_segment` round-trips (push → push_response). On a healthy
Pi 5 + USB-CDC link, p50 should be sub-millisecond; the long tail is
what we care about.

Recommended workload
====================

Run this in parallel with:

  - `glxgears` or any GPU-rendered foreground app (Mainsail dashboard
    page open in a Chromium tab counts).
  - A `journalctl -f` session writing to disk.
  - Periodic `apt-get` / `apt update` in a cron-equivalent loop.
  - A tcpdump trace — Moonraker WebSocket traffic mimics this load.

The point is to capture the CFS-scheduled, page-fault-y, IRQ-coalescing
worst case the host sees during a real print on a non-RT kernel.

Output
======

Logs every second to stderr; on completion writes a JSON report with
p50/p95/p99/p99.9/p99.99/max latencies in microseconds. The user copies
the relevant lines into
`docs/research/step6-buffer-budget-measurements.md`.

Usage
=====

```bash
# Pre-flight: build and flash production firmware on H723.
# Then on the Pi 5 (NOT the dev host):
python3 tools/measure_m1_host_stall.py \
    --port /dev/ttyACM0 --hours 8 \
    --report /tmp/m1-host-stall-$(date +%Y%m%d).json
```

Fixture
=======

Pushes back-to-back tiny segments referencing fixture slot 1 (loaded
once at startup with `kalico_load_fixture_curve fixture_id=0`). Each
segment is 1 ms long; when the queue fills the producer blocks on
`push_response` until the engine retires a slot — the time between
`io.send` and `wait_for_response` returning IS the host stall.
"""
import argparse
import json
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

CLOCK_FREQ = 520_000_000   # H723 default
TICK_HZ = 40_000
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ  # 13_000

# Long enough that the engine retires segments faster than we can push
# under healthy conditions, so the host-side push latency dominates.
SEG_TICKS = 40  # 1 ms per segment


def percentile(sorted_us, q):
    n = len(sorted_us)
    if n == 0:
        return 0.0
    idx = max(0, min(n - 1, int(round(q * (n - 1)))))
    return sorted_us[idx]


def push_segment(io, seg_id, fixture_handle, t_start_ticks, t_end_ticks,
                 timeout=5.0):
    cmd = (
        f"kalico_push_segment id={seg_id} curve_handle={fixture_handle} "
        f"t_start_hi={(t_start_ticks >> 32) & 0xFFFFFFFF} "
        f"t_start_lo={t_start_ticks & 0xFFFFFFFF} "
        f"t_end_hi={(t_end_ticks >> 32) & 0xFFFFFFFF} "
        f"t_end_lo={t_end_ticks & 0xFFFFFFFF} "
        f"kinematics=0"
    )
    t0 = time.monotonic()
    io.send(cmd)
    r = io.wait_for_response("kalico_push_response", timeout)
    return time.monotonic() - t0, int(r["result"])


def stream_open(io, stream_id):
    io.send(f"kalico_stream_open stream_id={stream_id}")
    r = io.wait_for_response("kalico_stream_open_response", 3.0)
    return int(r["result"])


def stream_arm(io, t_start_t0, arm_lead_cycles):
    io.send(
        f"kalico_stream_arm "
        f"t_start_t0_lo={t_start_t0 & 0xFFFFFFFF} "
        f"t_start_t0_hi={(t_start_t0 >> 32) & 0xFFFFFFFF} "
        f"arm_lead_cycles={arm_lead_cycles}"
    )
    r = io.wait_for_response("kalico_stream_arm_response", 3.0)
    return int(r["result"])


def main():
    p = argparse.ArgumentParser(description="Spec §7.3 M1 host-stall soak")
    p.add_argument("--port", required=True,
                   help="serial URL (e.g. /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument("--hours", type=float, default=8.0,
                   help="soak duration (default 8h)")
    p.add_argument("--report", default="m1-host-stall.json",
                   help="JSON report output path")
    p.add_argument("--fixture-id", type=int, default=0)
    args = p.parse_args()

    end_at = time.monotonic() + args.hours * 3600.0
    samples_us = []
    push_failures = 0
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    print(f"[m1] connecting {args.port} @ {args.baud}", file=sys.stderr)
    io = KalicoHostIO(args.port, args.baud)
    try:
        # Load fixture and open stream once.
        io.send(f"kalico_load_fixture_curve slot=1 fixture_id={args.fixture_id}")
        r = io.wait_for_response("kalico_load_fixture_response", 5.0)
        if int(r["result"]) != 0:
            raise SystemExit(
                f"FAIL: load_fixture rc={r['result']} — abort soak"
            )
        fixture_handle = int(r.get("curve_handle_packed", 0))

        if stream_open(io, stream_id=900) != 0:
            raise SystemExit("FAIL: stream_open")

        seg_cycles = SEG_TICKS * ONE_TICK_CYCLES
        next_seg_id = 1
        t_start_base = 1_000_000_000

        # Initial fill so the engine has something to chew on while we
        # measure steady-state push latency.
        for prefill in range(64):
            t_start = t_start_base + prefill * seg_cycles
            t_end = t_start + seg_cycles
            try:
                _dt, rc = push_segment(io, next_seg_id, fixture_handle,
                                       t_start, t_end, timeout=5.0)
            except HostIoError as exc:
                raise SystemExit(f"FAIL: prefill push: {exc}")
            if rc != 0:
                raise SystemExit(f"FAIL: prefill push rc={rc}")
            next_seg_id += 1

        if stream_arm(io, t_start_base, 2 * ONE_TICK_CYCLES) != 0:
            raise SystemExit("FAIL: stream_arm")

        # Steady-state measurement loop.
        last_log = time.monotonic()
        while time.monotonic() < end_at:
            t_start = t_start_base + (next_seg_id - 1) * seg_cycles
            t_end = t_start + seg_cycles
            try:
                dt, rc = push_segment(io, next_seg_id, fixture_handle,
                                      t_start, t_end, timeout=10.0)
            except HostIoError as exc:
                push_failures += 1
                # Pause and retry — this is exactly the kind of stall
                # we're measuring for, but if it repeats >100 times
                # something is structurally wrong.
                if push_failures > 100:
                    raise SystemExit(
                        f"FAIL: push timeouts > 100 — abort: {exc}"
                    )
                time.sleep(0.1)
                continue
            if rc != 0:
                # Backpressure or fault — fault aborts.
                io.send("kalico_query_status")
                try:
                    s = io.wait_for_response("kalico_status", 1.0)
                    if int(s["status"]) == 3:
                        raise SystemExit(
                            f"FAIL: engine FAULT during soak (last_err="
                            f"{s['last_err']}) after {len(samples_us)} samples"
                        )
                except HostIoError:
                    pass
                # Otherwise rate-limit and retry.
                time.sleep(0.001)
                continue
            samples_us.append(dt * 1e6)
            next_seg_id += 1

            now = time.monotonic()
            if now - last_log > 60.0:
                if samples_us:
                    s = sorted(samples_us)
                    print(
                        f"[m1] n={len(samples_us)} p50={percentile(s, .5):.1f}us "
                        f"p99={percentile(s, .99):.1f}us "
                        f"p999={percentile(s, .999):.2f}us "
                        f"max={s[-1]:.1f}us "
                        f"failures={push_failures}",
                        file=sys.stderr,
                    )
                last_log = now
    finally:
        try:
            io.disconnect()
        except Exception:
            pass

    finished_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
    s = sorted(samples_us)
    report = {
        "test": "M1_host_stall",
        "port": args.port,
        "baud": args.baud,
        "hours": args.hours,
        "started_utc": started_iso,
        "finished_utc": finished_iso,
        "n_samples": len(s),
        "push_failures": push_failures,
        "p50_us": percentile(s, 0.5),
        "p95_us": percentile(s, 0.95),
        "p99_us": percentile(s, 0.99),
        "p999_us": percentile(s, 0.999),
        "p9999_us": percentile(s, 0.9999),
        "max_us": s[-1] if s else 0.0,
        "host_workload_notes": (
            "USER MUST DOCUMENT: Pi model, kernel, Bookworm desktop "
            "active? Mainsail rendering active? Moonraker WS load? "
            "journald level? parallel `apt` / `tcpdump` / build jobs?"
        ),
    }
    out_path = pathlib.Path(args.report)
    out_path.write_text(json.dumps(report, indent=2))
    print(f"[m1] wrote report to {out_path.resolve()}")
    print(f"[m1] p50={report['p50_us']:.1f}us p99={report['p99_us']:.1f}us "
          f"p9999={report['p9999_us']:.2f}us max={report['max_us']:.1f}us "
          f"n={report['n_samples']} failures={report['push_failures']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

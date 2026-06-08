#!/usr/bin/env python3
import argparse
import json
import pathlib
import struct
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

CLOCK_FREQ = 520_000_000  # H723 default
TICK_HZ = 40_000
ONE_TICK_CYCLES = CLOCK_FREQ // TICK_HZ
FIVE_SECONDS_CYCLES = 5 * CLOCK_FREQ
TEN_SECONDS_CYCLES = 10 * CLOCK_FREQ

SEG_TICKS = 4000
FORMAT_VERSION_V1 = 1
UNUSED_HANDLE = 0xFFFEFFFE
E_MODE_TRAVEL = 2


def floats_to_blob(values):
    return b"".join(struct.pack("<f", float(v)) for v in values).hex()


def load_stall_curve(io, slot=1, timeout=5.0):
    cps = [0.0, 0.33333334, 0.6666667, 1.0]
    knots = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]
    io.send(
        f"kalico_load_curve version={FORMAT_VERSION_V1} slot={slot} degree=3 "
        f"cps={floats_to_blob(cps)} knots={floats_to_blob(knots)}"
    )
    r = io.wait_for_response("kalico_load_curve_response", timeout)
    return int(r["result"]), int(r.get("curve_handle_packed", 0))


def percentile(sorted_us, q):
    n = len(sorted_us)
    if n == 0:
        return 0.0
    idx = max(0, min(n - 1, int(round(q * (n - 1)))))
    return sorted_us[idx]


def push_segment(io, seg_id, x_handle, t_start_ticks, t_end_ticks, timeout=5.0):
    cmd = (
        f"kalico_push_segment id={seg_id} x_handle={x_handle} "
        f"y_handle={UNUSED_HANDLE} z_handle={UNUSED_HANDLE} "
        f"e_handle={UNUSED_HANDLE} "
        f"t_start_hi={(t_start_ticks >> 32) & 0xFFFFFFFF} "
        f"t_start_lo={t_start_ticks & 0xFFFFFFFF} "
        f"t_end_hi={(t_end_ticks >> 32) & 0xFFFFFFFF} "
        f"t_end_lo={t_end_ticks & 0xFFFFFFFF} "
        f"kinematics=0 e_mode={E_MODE_TRAVEL} extrusion_ratio=0"
    )
    t0 = time.monotonic()
    io.send(cmd)
    r = io.wait_for_response("kalico_push_response", timeout)
    return time.monotonic() - t0, int(r["result"])


def read_mcu_clock(io):
    io.send(
        "kalico_clock_sync_request request_id=1 host_send_time_lo=0 "
        "host_send_time_hi=0"
    )
    r = io.wait_for_response("kalico_clock_sync_response", 2.0)
    lo = int(r["mcu_clock_lo"]) & 0xFFFFFFFF
    hi = int(r["mcu_clock_hi"]) & 0xFFFFFFFF
    return (hi << 32) | lo


def stream_open(io, stream_id):
    io.send(f"runtime_stream_open stream_id={stream_id}")
    r = io.wait_for_response("kalico_stream_open_response", 3.0)
    return int(r["result"])


def stream_arm(io, t_start_t0, arm_lead_cycles):
    io.send(
        f"runtime_stream_arm "
        f"t_start_t0_lo={t_start_t0 & 0xFFFFFFFF} "
        f"t_start_t0_hi={(t_start_t0 >> 32) & 0xFFFFFFFF} "
        f"arm_lead_cycles={arm_lead_cycles}"
    )
    r = io.wait_for_response("kalico_stream_arm_response", 3.0)
    return int(r["result"])


def main():
    p = argparse.ArgumentParser(description="Spec §7.3 M1 host-stall soak")
    p.add_argument(
        "--port", required=True, help="serial URL (e.g. /dev/ttyACM0)"
    )
    p.add_argument("--baud", type=int, default=250000)
    p.add_argument(
        "--hours", type=float, default=8.0, help="soak duration (default 8h)"
    )
    p.add_argument(
        "--report", default="m1-host-stall.json", help="JSON report output path"
    )
    p.add_argument(
        "--fixture-id",
        type=int,
        default=0,
        help="deprecated; retained for old command lines, ignored",
    )
    args = p.parse_args()

    end_at = time.monotonic() + args.hours * 3600.0
    samples_us = []
    push_failures = 0
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    print(f"[m1] connecting {args.port} @ {args.baud}", file=sys.stderr)
    io = KalicoHostIO(args.port, args.baud)
    try:
        rc, fixture_handle = load_stall_curve(io, slot=1)
        if rc != 0:
            raise SystemExit(f"FAIL: load_curve rc={rc} — abort soak")

        if stream_open(io, stream_id=900) != 0:
            raise SystemExit("FAIL: stream_open")

        seg_cycles = SEG_TICKS * ONE_TICK_CYCLES
        next_seg_id = 1
        # read_mcu_clock returns 0 before the engine ISR has ticked (TIM5 only
        # fires after the first push, §4.4); on a fresh boot use a generous
        # absolute offset so the priming segment is in the MCU's future.
        mcu_now = read_mcu_clock(io)
        t_start_base = max(
            mcu_now + FIVE_SECONDS_CYCLES,
            TEN_SECONDS_CYCLES,
        )

        # Engine queue capacity = 8 (heapless::spsc::Queue<Segment, 8>); we
        # prefill 7 so there's one free slot for the first steady-state push.
        for prefill in range(7):
            t_start = t_start_base + prefill * seg_cycles
            t_end = t_start + seg_cycles
            try:
                _dt, rc = push_segment(
                    io, next_seg_id, fixture_handle, t_start, t_end, timeout=5.0
                )
            except HostIoError as exc:
                raise SystemExit(f"FAIL: prefill push: {exc}")
            if rc != 0:
                raise SystemExit(f"FAIL: prefill push rc={rc}")
            next_seg_id += 1

        if stream_arm(io, t_start_base, 2 * ONE_TICK_CYCLES) != 0:
            raise SystemExit("FAIL: stream_arm")

        last_log = time.monotonic()
        while time.monotonic() < end_at:
            t_start = t_start_base + (next_seg_id - 1) * seg_cycles
            t_end = t_start + seg_cycles
            try:
                dt, rc = push_segment(
                    io,
                    next_seg_id,
                    fixture_handle,
                    t_start,
                    t_end,
                    timeout=10.0,
                )
            except HostIoError as exc:
                push_failures += 1
                if push_failures > 100:
                    raise SystemExit(
                        f"FAIL: push timeouts > 100 — abort: {exc}"
                    )
                time.sleep(0.1)
                continue
            if rc != 0:
                io.send("runtime_query_status")
                try:
                    s = io.wait_for_response("kalico_status", 1.0)
                    if int(s["status"]) == 3:
                        raise SystemExit(
                            f"FAIL: engine FAULT during soak (last_err="
                            f"{s['last_err']}) after {len(samples_us)} samples"
                        )
                except HostIoError:
                    pass
                time.sleep(0.001)
                continue
            samples_us.append(dt * 1e6)
            next_seg_id += 1

            now = time.monotonic()
            if now - last_log > 60.0:
                if samples_us:
                    s = sorted(samples_us)
                    print(
                        f"[m1] n={len(samples_us)} p50={percentile(s, 0.5):.1f}us "
                        f"p99={percentile(s, 0.99):.1f}us "
                        f"p999={percentile(s, 0.999):.2f}us "
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
    print(
        f"[m1] p50={report['p50_us']:.1f}us p99={report['p99_us']:.1f}us "
        f"p9999={report['p9999_us']:.2f}us max={report['max_us']:.1f}us "
        f"n={report['n_samples']} failures={report['push_failures']}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

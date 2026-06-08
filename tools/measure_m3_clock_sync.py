#!/usr/bin/env python3
"""Spec §7.3 M3 clock-sync residual soak.

Usage:
    python3 tools/measure_m3_clock_sync.py --port-h723 /dev/ttyACM0 --hours 24 --report m3.json
    python3 tools/measure_m3_clock_sync.py --port-h723 /dev/ttyACM0 --port-f4x /dev/ttyACM1 --hours 24 --report m3.json
"""

import argparse
import collections
import json
import math
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kalico_host_io import HostIoError, KalicoHostIO  # noqa: E402

# Spec §12.2 — sliding-window depth.
WINDOW = 30
SAMPLE_PERIOD_S = 0.1


class ClockSyncWindow:
    """Sliding-window clock-sync regression (Spec §12.2/§12.4). Not bit-identical to
    `rust/kalico-host-rt/src/clock_sync.rs`; the Rust impl is the source of truth."""

    def __init__(self, baseline_freq):
        self.baseline_freq = baseline_freq
        self.samples = collections.deque(maxlen=WINDOW)
        self.epoch_t = time.monotonic()
        self.freq = baseline_freq
        self.residual_max_us = 0.0
        self.last_sample_age_s = math.inf
        self.last_dedicated_sample_age_s = math.inf

    def add(self, host_send_t, host_recv_t, mcu_at_response):
        rtt_s = max(host_recv_t - host_send_t, 0.0)
        one_way_cycles = (rtt_s / 2.0) * self.freq
        mcu_at_send = mcu_at_response - one_way_cycles
        host_time_secs = host_send_t - self.epoch_t
        self.samples.append((host_time_secs, mcu_at_send, time.monotonic()))
        self._recompute()

    def _recompute(self):
        n = len(self.samples)
        if n < 2:
            return
        xs = [s[0] for s in self.samples]
        ys = [s[1] for s in self.samples]
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom < 1e-12:
            return
        slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
        offset = mean_y - slope * mean_x
        self.freq = slope
        max_resid_us = 0.0
        for x, y in zip(xs, ys):
            predicted = slope * x + offset
            resid_s = (y - predicted) / slope
            r = abs(resid_s) * 1e6
            if r > max_resid_us:
                max_resid_us = r
        self.residual_max_us = max_resid_us
        latest = self.samples[-1][2]
        self.last_sample_age_s = time.monotonic() - latest
        self.last_dedicated_sample_age_s = self.last_sample_age_s

    def drift_ppm(self):
        if abs(self.baseline_freq) < 1e-12:
            return 0.0
        return ((self.freq - self.baseline_freq) / self.baseline_freq) * 1e6


def issue_clock_sync(io):
    host_send_t = time.monotonic()
    io.send(
        "kalico_clock_sync_request request_id=1 "
        "host_send_time_lo=0 host_send_time_hi=0"
    )
    r = io.wait_for_response("kalico_clock_sync_response", timeout=2.0)
    host_recv_t = time.monotonic()
    mcu_at = (int(r.get("mcu_clock_hi", 0)) << 32) | int(
        r.get("mcu_clock_lo", 0)
    )
    return host_send_t, host_recv_t, mcu_at


def main():
    p = argparse.ArgumentParser(description="Spec §7.3 M3 clock-sync soak")
    p.add_argument("--port-h723", required=True)
    p.add_argument("--baud-h723", type=int, default=250000)
    p.add_argument(
        "--baseline-freq-h723",
        type=float,
        default=520_000_000.0,
        help="H723 nominal MCU clock (Hz)",
    )
    p.add_argument(
        "--port-f4x",
        default=None,
        help="optional second MCU port; if absent, single-MCU run",
    )
    p.add_argument("--baud-f4x", type=int, default=250000)
    p.add_argument(
        "--baseline-freq-f4x",
        type=float,
        default=180_000_000.0,
        help="F4x nominal MCU clock (Hz)",
    )
    p.add_argument("--hours", type=float, default=24.0)
    p.add_argument("--report", default="m3-clock-sync.json")
    args = p.parse_args()

    end_at = time.monotonic() + args.hours * 3600.0
    started_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    estimators = {}
    ios = {}

    print(f"[m3] connecting H723 {args.port_h723}", file=sys.stderr)
    ios["h723"] = KalicoHostIO(args.port_h723, args.baud_h723)
    estimators["h723"] = ClockSyncWindow(args.baseline_freq_h723)

    if args.port_f4x:
        print(f"[m3] connecting F4x {args.port_f4x}", file=sys.stderr)
        ios["f4x"] = KalicoHostIO(args.port_f4x, args.baud_f4x)
        estimators["f4x"] = ClockSyncWindow(args.baseline_freq_f4x)

    history = {
        k: {
            "max_residual_us": 0.0,
            "max_abs_drift_ppm": 0.0,
            "max_sample_age_s": 0.0,
            "residuals_us": [],
            "drifts_ppm": [],
            "sample_ages_s": [],
            "round_trip_failures": 0,
        }
        for k in estimators
    }

    last_log_t = time.monotonic()
    try:
        while time.monotonic() < end_at:
            for tag, io in ios.items():
                est = estimators[tag]
                try:
                    h_s, h_r, mcu = issue_clock_sync(io)
                except HostIoError:
                    history[tag]["round_trip_failures"] += 1
                    continue
                est.add(h_s, h_r, mcu)
                h = history[tag]
                if est.residual_max_us > h["max_residual_us"]:
                    h["max_residual_us"] = est.residual_max_us
                drift = abs(est.drift_ppm())
                if drift > h["max_abs_drift_ppm"]:
                    h["max_abs_drift_ppm"] = drift
                if est.last_sample_age_s > h["max_sample_age_s"]:
                    h["max_sample_age_s"] = est.last_sample_age_s
                # Subsample for distribution stats; full-fidelity history
                # is too large for 24h × 10Hz × 2 MCUs = 1.7M points.
                if len(h["residuals_us"]) < 100_000:
                    h["residuals_us"].append(est.residual_max_us)
                    h["drifts_ppm"].append(drift)
                    h["sample_ages_s"].append(est.last_sample_age_s)
            time.sleep(SAMPLE_PERIOD_S)
            if time.monotonic() - last_log_t > 60.0:
                msg = []
                for tag, h in history.items():
                    msg.append(
                        f"{tag}: max_resid={h['max_residual_us']:.2f}us "
                        f"max_drift={h['max_abs_drift_ppm']:.2f}ppm "
                        f"max_age={h['max_sample_age_s']:.3f}s "
                        f"fails={h['round_trip_failures']}"
                    )
                print("[m3] " + " | ".join(msg), file=sys.stderr)
                last_log_t = time.monotonic()
    finally:
        for io in ios.values():
            try:
                io.disconnect()
            except Exception:
                pass

    finished_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())

    def percentile(vals, q):
        if not vals:
            return 0.0
        s = sorted(vals)
        idx = max(0, min(len(s) - 1, int(round(q * (len(s) - 1)))))
        return s[idx]

    report = {
        "test": "M3_clock_sync",
        "hours": args.hours,
        "started_utc": started_iso,
        "finished_utc": finished_iso,
        "mcus": {},
    }
    for tag, h in history.items():
        report["mcus"][tag] = {
            "max_residual_us": h["max_residual_us"],
            "max_abs_drift_ppm": h["max_abs_drift_ppm"],
            "max_sample_age_s": h["max_sample_age_s"],
            "residual_p9999_us": percentile(h["residuals_us"], 0.9999),
            "drift_ppm_p9999": percentile(h["drifts_ppm"], 0.9999),
            "sample_age_p9999_s": percentile(h["sample_ages_s"], 0.9999),
            "n_samples_logged": len(h["residuals_us"]),
            "round_trip_failures": h["round_trip_failures"],
        }
    if "f4x" not in report["mcus"]:
        report["mcus"]["f4x"] = {"status": "not_run"}

    out_path = pathlib.Path(args.report)
    out_path.write_text(json.dumps(report, indent=2))
    print(f"[m3] wrote report to {out_path.resolve()}")
    for tag, m in report["mcus"].items():
        if m.get("status") == "not_run":
            print(f"[m3] {tag}: not run")
            continue
        print(
            f"[m3] {tag}: max_resid={m['max_residual_us']:.2f}us "
            f"p9999_resid={m['residual_p9999_us']:.2f}us "
            f"max_drift={m['max_abs_drift_ppm']:.2f}ppm "
            f"p9999_drift={m['drift_ppm_p9999']:.2f}ppm "
            f"max_age={m['max_sample_age_s']:.3f}s"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Analyze a servo telemetry capture (.scap) produced by SERVO_CAPTURE_START.

Prints following-error, overshoot/settling, and torque-saturation metrics;
--fft prints resonance peaks (notch-filter candidates); --plot opens a
time-series dashboard.
"""

import argparse
import glob
import json
import os
import re
import sys

import numpy as np

CAPTURE_TS_RE = re.compile(r"_(\d{8}_\d{6})\.scap$")

DTYPE_MAP = {
    "u8": "u1",
    "u16": "<u2",
    "i16": "<i2",
    "i32": "<i4",
    "u64": "<u8",
}
FLAG_MOTION_ACTIVE = 1 << 1
SETTLE_HOLD_MS = 50


def resolve_newest_capture(captures_dir, name):
    pattern = os.path.join(
        os.path.expanduser(captures_dir), name + "_*.scap"
    )
    matches = [p for p in glob.glob(pattern) if CAPTURE_TS_RE.search(p)]
    if not matches:
        raise SystemExit(
            "no capture named %r in %s" % (name, captures_dir)
        )
    return max(matches)


def load_capture(path):
    if path.endswith(".failed.scap"):
        raise SystemExit(
            "%s is a FAILED capture (ring overflow or writer error); its "
            "gaps would poison every metric. Re-run the capture." % (path,)
        )
    with open(path, "rb") as f:
        header = json.loads(f.readline())
        if header.get("version") != 1:
            raise SystemExit(
                "unsupported capture version %r" % (header.get("version"),)
            )
        dtype = np.dtype(
            [(c["name"], DTYPE_MAP[c["dtype"]]) for c in header["channels"]]
        )
        for c in header["channels"]:
            actual = dtype.fields[c["name"]][1]
            if actual != c["offset"]:
                raise SystemExit(
                    "channel %r declared offset %d but numpy computed %d"
                    % (c["name"], c["offset"], actual)
                )
        if dtype.itemsize != header["record_size"]:
            raise SystemExit(
                "channel descriptor (%d bytes) disagrees with record_size %d"
                % (dtype.itemsize, header["record_size"])
            )
        body = f.read()
    whole = len(body) // header["record_size"] * header["record_size"]
    data = np.frombuffer(body[:whole], dtype=dtype)
    return header, data


def motion_segments(flags):
    if not len(flags):
        return []
    moving = (flags & FLAG_MOTION_ACTIVE) != 0
    edges = np.flatnonzero(np.diff(moving.astype(np.int8)))
    bounds = np.concatenate(([0], edges + 1, [len(moving)]))
    return [
        (int(bounds[i]), int(bounds[i + 1]))
        for i in range(len(bounds) - 1)
        if moving[bounds[i]]
    ]


def _settle_index(err, band, hold):
    inside = np.abs(err) <= band
    if len(inside) < hold:
        return None
    windows = np.lib.stride_tricks.sliding_window_view(inside, hold)
    ok = np.flatnonzero(windows.all(axis=1))
    return int(ok[0]) if len(ok) else None


def compute_metrics(data, settle_band, torque_limit, fs=1000.0):
    if not len(data):
        raise SystemExit("capture contains no records")
    ms_per_sample = 1000.0 / fs
    hold = int(round(SETTLE_HOLD_MS * fs / 1000.0))
    ferr = data["following_error"].astype(np.float64)
    recomputed = data["target_counts"].astype(np.int64) - data[
        "position_actual"
    ].astype(np.int64)
    segs = motion_segments(data["flags"])
    moves = []
    for idx, (s, e) in enumerate(segs):
        move_err = ferr[s:e]
        post_end = segs[idx + 1][0] if idx + 1 < len(segs) else len(ferr)
        post = ferr[e:post_end]
        settle_sample = _settle_index(post, settle_band, hold)
        overshoot_end = (
            settle_sample if settle_sample is not None else len(post)
        )
        settle_ms = (
            float(settle_sample) * ms_per_sample
            if settle_sample is not None
            else None
        )
        moves.append(
            {
                "move": idx,
                "start_ms": float(s) * ms_per_sample,
                "end_ms": float(e) * ms_per_sample,
                "ferr_peak": float(np.max(np.abs(move_err))),
                "ferr_rms": float(np.sqrt(np.mean(move_err**2))),
                "overshoot": float(np.max(np.abs(post[:overshoot_end])))
                if overshoot_end > 0
                else 0.0,
                "settle_ms": settle_ms,
            }
        )
    torque = np.abs(data["torque_actual"].astype(np.int64))
    return {
        "samples": len(data),
        "moves": moves,
        "torque_saturation_pct": float(
            100.0 * np.count_nonzero(torque >= torque_limit) / max(len(data), 1)
        ),
        "ferr_crosscheck_max": int(
            np.max(np.abs(recomputed - ferr.astype(np.int64)))
        ),
    }


def welch_psd(x, fs, nperseg=1024):
    x = np.asarray(x, dtype=np.float64)
    nperseg = min(nperseg, len(x))
    nperseg = 2 ** int(np.log2(nperseg))
    if nperseg < 64:
        raise SystemExit(
            "segment too short for PSD (%d samples; need >= 64)" % (len(x),)
        )
    step = nperseg // 2
    win = np.hanning(nperseg)
    scale = 1.0 / (fs * np.sum(win * win))
    psds = []
    for start in range(0, len(x) - nperseg + 1, step):
        seg = x[start : start + nperseg]
        seg = (seg - np.mean(seg)) * win
        spec = np.fft.rfft(seg)
        psds.append((spec.real**2 + spec.imag**2) * scale)
    psd = np.mean(psds, axis=0)
    psd[1:-1] *= 2.0
    return np.fft.rfftfreq(nperseg, 1.0 / fs), psd


def moving_psd(data, segs, fs):
    if not segs:
        raise SystemExit("no moving segments in capture — nothing to analyze")
    err = np.concatenate(
        [data["following_error"][s:e].astype(np.float64) for s, e in segs]
    )
    return welch_psd(err, fs)


def top_peaks(freqs, psd, count=5):
    local_max = (
        np.flatnonzero((psd[1:-1] > psd[:-2]) & (psd[1:-1] > psd[2:])) + 1
    )
    ranked = local_max[np.argsort(psd[local_max])[::-1]][:count]
    return [(float(freqs[i]), float(psd[i])) for i in ranked]


def _print_metrics(m, counts_per_mm):
    print("capture: %d samples, %d move(s)" % (m["samples"], len(m["moves"])))
    print(
        "torque saturation: %.1f%% of samples at/above limit"
        % (m["torque_saturation_pct"],)
    )
    print(
        "drive-vs-recomputed following error: max delta %d counts"
        % (m["ferr_crosscheck_max"],)
    )
    for mv in m["moves"]:
        settle = (
            "%.1f ms" % mv["settle_ms"]
            if mv["settle_ms"] is not None
            else "NEVER"
        )
        print(
            "move %d [%.1f..%.1f ms]: ferr peak %.0f counts (%.4f mm), "
            "rms %.1f counts (%.4f mm), overshoot %.0f counts, settle %s"
            % (
                mv["move"],
                mv["start_ms"],
                mv["end_ms"],
                mv["ferr_peak"],
                mv["ferr_peak"] / counts_per_mm,
                mv["ferr_rms"],
                mv["ferr_rms"] / counts_per_mm,
                mv["overshoot"],
                settle,
            )
        )


def export_ident_csv(path, header, data, counts_per_mm):
    # The fitter regresses torque against the kinematics that produced it.
    # With a position loop between command and motor, torque follows actual
    # motion (6064h), not the commanded target — exporting target_counts
    # here yields a wrong (even negative) inertia on a soft loop.
    axis = header["drives"][0]["name"]
    cycle_index = data["cycle_index"].astype(np.int64)
    t = (cycle_index - cycle_index[0]) * (header["cycle_ns"] * 1e-9)
    actual_mm = data["position_actual"].astype(np.float64) / counts_per_mm
    torque = data["torque_actual"].astype(np.float64)
    with open(path, "w") as f:
        f.write("t,target_%s,torque_%s\n" % (axis, axis))
        for row in zip(t, actual_mm, torque):
            f.write("%.6f,%.9g,%.9g\n" % row)
    print(
        "wrote %d samples for axis %r to %s (feed to servo-ident --capture)"
        % (len(t), axis, path)
    )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("capture", nargs="?", help="path to a .scap capture file")
    p.add_argument(
        "--name",
        help="capture base name; analyzes the newest matching capture "
        "in --captures-dir instead of an explicit path",
    )
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument(
        "--settle-band",
        type=int,
        default=50,
        help="settling band in encoder counts (default 50)",
    )
    p.add_argument(
        "--torque-limit",
        type=int,
        default=900,
        help="saturation threshold, per-mille of rated (default 900)",
    )
    p.add_argument(
        "--fft",
        action="store_true",
        help="print resonance peaks from the moving-segment PSD",
    )
    p.add_argument(
        "--plot",
        action="store_true",
        help="show a time-series dashboard (requires matplotlib)",
    )
    p.add_argument(
        "--csv",
        metavar="PATH",
        help="export t/target/torque in servo-ident's CSV contract "
        "(t in s, target in mm, torque in 0.1%% rated) and exit",
    )
    args = p.parse_args(argv)
    if (args.capture is None) == (args.name is None):
        raise SystemExit("pass a .scap path or --name, not both or neither")
    capture_path = args.capture or resolve_newest_capture(
        args.captures_dir, args.name
    )

    header, data = load_capture(capture_path)
    fs = 1e9 / header["cycle_ns"]
    counts_per_mm = header["drives"][0]["counts_per_mm"]

    if args.csv:
        export_ident_csv(args.csv, header, data, counts_per_mm)
        return 0

    print("file: %s" % (capture_path,))
    m = compute_metrics(data, args.settle_band, args.torque_limit, fs=fs)
    _print_metrics(m, counts_per_mm)

    if args.fft:
        freqs, psd = moving_psd(data, motion_segments(data["flags"]), fs)
        print("resonance peaks (notch-filter candidates):")
        for f_hz, power in top_peaks(freqs, psd):
            print("  %7.1f Hz  power %.3e" % (f_hz, power))

    if args.plot:
        _plot(header, data, fs)
    return 0


def _plot(header, data, fs):
    import matplotlib.pyplot as plt

    t = np.arange(len(data)) / fs
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    axes[0].plot(t, data["position_demand"], label="demand (6062h)")
    axes[0].plot(t, data["position_actual"], label="actual (6064h)")
    axes[0].plot(
        t,
        data["target_counts"],
        label="host target (607Ah)",
        linestyle="--",
        alpha=0.6,
    )
    axes[0].set_ylabel("counts")
    axes[0].legend(loc="upper right")
    axes[1].plot(t, data["following_error"], color="tab:red")
    axes[1].set_ylabel("following error (counts)")
    axes[2].plot(t, data["torque_actual"], color="tab:green")
    axes[2].set_ylabel("torque (per-mille)")
    axes[2].set_xlabel("time (s)")
    moving = (data["flags"] & FLAG_MOTION_ACTIVE) != 0
    for ax in axes:
        ax.fill_between(
            t, *ax.get_ylim(), where=moving, alpha=0.08, color="tab:blue"
        )
    fig.suptitle(header["drives"][0]["name"] + " — " + header["started_utc"])
    plt.show()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Render a gain-sweep comparison report from SERVO_CALIBRATE_GAINS captures.

The sweep macro records one capture per gain step, named
<tag>_p<POS>_s<SPEED>_i<INTEGRAL>, which SERVO_CAPTURE timestamps into
<name>_<YYYYmmdd_HHMMSS>.scap. This script resolves each step to its newest
capture, computes tracking metrics, flags resonance, writes a comparison PNG,
and prints a table plus a recommendation.

The sweep macro passes --steps with the exact step names it just recorded,
so the report covers only that run. Without --steps the script falls back to
every step name matching the tag, which mixes in steps left over from older
runs that used different gain lists.

Usage:
  servo_gain_report.py --tag cal --steps cal_p2000_s1250_i1000,cal_p2400_s1500_i833
  servo_gain_report.py --captures-dir ~/printer_data/logs/servo_captures \
      --tag cal --out-dir ~/printer_data/config/servo_calibrate_results
  servo_gain_report.py file1.scap file2.scap ... --out report.png
"""

import argparse
import datetime
import glob
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from servo_capture import load_capture  # noqa: E402

STEP_RE = re.compile(r"_p(\d+)_s(\d+)_i(\d+)_\d{8}_\d{6}\.scap$")
RESONANCE_BAND_HZ = (20.0, 450.0)
LOW_BAND_HZ = (1.0, 4.0)
RESONANCE_RATIO_LIMIT = 8.0


def find_sweep_files(captures_dir, tag):
    newest = {}
    pattern = os.path.join(os.path.expanduser(captures_dir), tag + "_p*.scap")
    for path in glob.glob(pattern):
        m = STEP_RE.search(os.path.basename(path))
        if not m:
            continue
        key = tuple(int(g) for g in m.groups())
        if key not in newest or path > newest[key]:
            newest[key] = path
    return [(k, newest[k]) for k in sorted(newest, key=lambda k: k[1])]


def find_named_steps(captures_dir, step_names):
    files = []
    for name in step_names:
        pattern = os.path.join(
            os.path.expanduser(captures_dir), name + "_*.scap"
        )
        matches = [p for p in glob.glob(pattern) if STEP_RE.search(p)]
        if not matches:
            raise SystemExit(
                "sweep step %r has no capture in %s" % (name, captures_dir)
            )
        path = max(matches)
        files.append((gains_from_name(path), path))
    files.sort(key=lambda kp: kp[0][1])
    return files


def gains_from_name(path):
    m = STEP_RE.search(os.path.basename(path))
    if not m:
        return None
    return tuple(int(g) for g in m.groups())


def cruise_mask(target_mm, t):
    v = np.gradient(np.convolve(target_mm, np.ones(11) / 11, "same"), t)
    speed = np.abs(v)
    moving = speed > 5.0
    if not moving.any():
        raise SystemExit("capture has no motion")
    vnom = np.percentile(speed[moving], 90)
    mask = np.abs(speed - vnom) < max(2.0, 0.02 * vnom)
    m = mask & np.roll(mask, 50) & np.roll(mask, -50)
    return m, vnom, v


def amplitude_spectrum(x):
    seg = np.asarray(x, dtype=np.float64)
    seg = seg - seg.mean()
    n = min(len(seg), 16384)
    seg = seg[:n] * np.hanning(n)
    spectrum = np.abs(np.fft.rfft(seg)) / n * 2.0
    freqs = np.fft.rfftfreq(n, 1e-3)
    return freqs, spectrum


def band_peak(freqs, spectrum, lo, hi):
    band = (freqs >= lo) & (freqs < hi)
    if not band.any():
        return 0.0, 0.0
    i = int(np.argmax(spectrum[band]))
    return float(spectrum[band][i]), float(freqs[band][i])


def resonance_protrusion(freqs, spectrum, lo, hi):
    """How far the worst peak in [lo, hi) sticks out of its local floor.

    A healthy spectrum rolls off smoothly, so every point sits near the
    median of its own +-40% frequency neighbourhood. A loop resonance is a
    narrow spike: amplitude many times that local median.
    """
    smooth = np.convolve(spectrum, np.ones(3) / 3, "same")
    band_idx = np.flatnonzero((freqs >= lo) & (freqs < hi))
    worst_ratio, worst_hz, worst_amp = 0.0, 0.0, 0.0
    for i in band_idx:
        window = (freqs >= freqs[i] * 0.6) & (freqs <= freqs[i] * 1.4)
        floor = np.median(smooth[window])
        if floor <= 0:
            continue
        ratio = smooth[i] / floor
        if ratio > worst_ratio:
            worst_ratio, worst_hz, worst_amp = ratio, freqs[i], smooth[i]
    return worst_ratio, worst_hz, worst_amp


def step_metrics(path):
    header, data = load_capture(path)
    cpm = header["drives"][0]["counts_per_mm"]
    n = len(data)
    t = np.arange(n) / 1000.0
    target = data["target_counts"].astype(np.float64) / cpm
    actual = data["position_actual"].astype(np.float64) / cpm
    ferr = data["following_error"].astype(np.float64) / cpm

    m, vnom, vt = cruise_mask(target, t)
    if m.sum() < 1024:
        raise SystemExit("%s: not enough cruise samples" % (path,))
    freqs, spectrum = amplitude_spectrum(ferr[m])
    low_amp, low_hz = band_peak(freqs, spectrum, *LOW_BAND_HZ)
    res_ratio, res_hz, res_amp = resonance_protrusion(
        freqs, spectrum, *RESONANCE_BAND_HZ
    )
    resonant = res_ratio > RESONANCE_RATIO_LIMIT

    moving = np.abs(vt) > 5.0
    ends = np.where(moving[:-1] & ~moving[1:])[0]
    overshoots = []
    for e in ends:
        if e + 400 >= n or e < 100:
            continue
        endpos = target[e + 50]
        direction = np.sign(target[e] - target[e - 100])
        overshoots.append(np.max((actual[e : e + 400] - endpos) * direction))
    return {
        "path": path,
        "cruise_mm_s": float(vnom),
        "lag_ms": float(np.mean(np.abs(ferr[m])) / vnom * 1000.0),
        "ferr_std_um": float(np.std(np.abs(ferr[m])) * 1000.0),
        "low_band_um": low_amp * 1000.0,
        "low_band_hz": low_hz,
        "res_peak_um": res_amp * 1000.0,
        "res_peak_hz": res_hz,
        "res_ratio": float(res_ratio),
        "resonant": bool(resonant),
        "overshoot_max_um": float(np.max(overshoots) * 1000.0)
        if overshoots
        else 0.0,
        "spectrum": (freqs, spectrum),
        "cruise_ferr": ferr[m],
    }


def render(steps, out_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    colors = plt.cm.viridis(np.linspace(0.0, 0.85, len(steps)))

    spec_ax, time_ax = axes[0]
    for (gains, met), color in zip(steps, colors):
        label = "pos %.0f rad/s / speed %.0f Hz%s" % (
            gains[0] / 10.0,
            gains[1] / 10.0,
            "  RESONANT" if met["resonant"] else "",
        )
        freqs, spectrum = met["spectrum"]
        spec_ax.loglog(
            freqs[1:],
            np.convolve(spectrum[1:] * 1000.0, np.ones(3) / 3, "same"),
            color=color,
            lw=1.0,
            label=label,
        )
        seg = met["cruise_ferr"][:1500]
        time_ax.plot(
            np.arange(len(seg)) / 1000.0, seg * 1000.0, color=color, lw=0.7
        )
    spec_ax.axvspan(*RESONANCE_BAND_HZ, alpha=0.06, color="red")
    spec_ax.set_xlabel("Hz")
    spec_ax.set_ylabel("ferr amplitude (um)")
    spec_ax.set_title(
        "Cruise following-error spectrum (red band: resonance watch)"
    )
    spec_ax.legend(fontsize=8)
    spec_ax.grid(True, which="both", alpha=0.3)
    time_ax.set_xlabel("s into cruise")
    time_ax.set_ylabel("ferr (mm)")
    time_ax.set_title("Cruise following error, time domain")
    time_ax.grid(alpha=0.3)

    curve_ax, table_ax = axes[1]
    hz = [g[1] / 10.0 for g, _ in steps]
    for key, label, scale in (
        ("ferr_std_um", "cruise error std (um)", 1.0),
        ("low_band_um", "low-band disturbance (um)", 1.0),
        ("overshoot_max_um", "overshoot max (um)", 1.0),
        ("lag_ms", "lag (ms) x10", 10.0),
    ):
        curve_ax.plot(
            hz, [m[key] * scale for _, m in steps], marker="o", label=label
        )
    for (gains, met), x in zip(steps, hz):
        if met["resonant"]:
            curve_ax.axvline(x, color="red", ls="--", alpha=0.5)
    curve_ax.set_xlabel("speed loop gain (Hz)")
    curve_ax.set_title("Metrics vs gain (red dashed: resonant step)")
    curve_ax.legend(fontsize=8)
    curve_ax.grid(alpha=0.3)

    table_ax.axis("off")
    rows = [
        [
            "%.0f/%.0f/%.0f" % (g[0] / 10.0, g[1] / 10.0, g[2] / 100.0),
            "%.1f" % m["lag_ms"],
            "%.0f" % m["ferr_std_um"],
            "%.0f" % m["low_band_um"],
            "%.0f @ %.0fHz" % (m["res_peak_um"], m["res_peak_hz"]),
            "%.0f" % m["overshoot_max_um"],
            "YES" if m["resonant"] else "no",
        ]
        for g, m in steps
    ]
    table = table_ax.table(
        cellText=rows,
        colLabels=[
            "pos/spd/Ti",
            "lag ms",
            "err um",
            "low um",
            "res peak",
            "ovsh um",
            "resonant",
        ],
        loc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(8)
    table_ax.set_title("rad/s / Hz / ms", fontsize=9)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)


def recommend(steps):
    clean = [(g, m) for g, m in steps if not m["resonant"]]
    if not clean:
        return None, "every step shows a resonance signature — reduce gains"
    best_err = min(m["ferr_std_um"] for _, m in clean)
    good = [(g, m) for g, m in clean if m["ferr_std_um"] <= best_err * 1.3]
    gains, _ = max(good, key=lambda gm: gm[0][1])
    note = (
        "highest gain whose cruise error stays within 30%% of the best (%.0f um)"
        % best_err
    )
    rejected = [
        (g, m)
        for g, m in steps
        if g[1] > gains[1]
        and (m["resonant"] or m["ferr_std_um"] > best_err * 1.3)
    ]
    if rejected:
        worst = rejected[0]
        why = (
            "resonance at %.0f Hz" % worst[1]["res_peak_hz"]
            if worst[1]["resonant"]
            else "cruise error degraded to %.0f um" % worst[1]["ferr_std_um"]
        )
        note += "; ceiling: %.0f Hz step rejected (%s)" % (
            worst[0][1] / 10.0,
            why,
        )
    return gains, note


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("captures", nargs="*", help="explicit .scap files")
    p.add_argument(
        "--captures-dir", default="~/printer_data/logs/servo_captures"
    )
    p.add_argument("--tag", default="cal")
    p.add_argument(
        "--steps",
        help="comma list of step names recorded by this sweep run "
        "(<tag>_p<P>_s<S>_i<I>); only these steps are reported",
    )
    p.add_argument(
        "--out-dir", default="~/printer_data/config/servo_calibrate_results"
    )
    p.add_argument("--out", help="explicit output PNG path")
    args = p.parse_args(argv)

    if args.captures and args.steps:
        raise SystemExit("pass explicit .scap files or --steps, not both")
    if args.captures:
        files = []
        for path in args.captures:
            gains = gains_from_name(path)
            if gains is None:
                raise SystemExit(
                    "%s: filename lacks _p<P>_s<S>_i<I>_<ts>.scap gain fields"
                    % (path,)
                )
            files.append((gains, path))
        files.sort(key=lambda kp: kp[0][1])
    elif args.steps:
        files = find_named_steps(args.captures_dir, args.steps.split(","))
    else:
        files = find_sweep_files(args.captures_dir, args.tag)
    if not files:
        raise SystemExit("no sweep captures found (tag %r)" % (args.tag,))

    steps = [(gains, step_metrics(path)) for gains, path in files]

    if args.out:
        out_path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    else:
        out_dir = os.path.expanduser(args.out_dir)
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, "gains_%s_%s.png" % (args.tag, stamp))
    render(steps, out_path)

    print(
        "%-16s %7s %7s %7s %12s %8s %s"
        % (
            "pos/spd/Ti",
            "lag ms",
            "err um",
            "low um",
            "res peak",
            "ovsh um",
            "resonant",
        )
    )
    for gains, met in steps:
        print(
            "%-16s %7.1f %7.0f %7.0f %6.0f@%3.0fHz %8.0f %s"
            % (
                "%.0f/%.0f/%.0f"
                % (gains[0] / 10.0, gains[1] / 10.0, gains[2] / 100.0),
                met["lag_ms"],
                met["ferr_std_um"],
                met["low_band_um"],
                met["res_peak_um"],
                met["res_peak_hz"],
                met["overshoot_max_um"],
                "YES" if met["resonant"] else "no",
            )
        )
    gains, note = recommend(steps)
    if gains is not None:
        print(
            "recommended: SERVO_APPLY_GAINS POS_GAIN=%d SPEED_GAIN=%d INTEGRAL=%d  (%s)"
            % (gains[0], gains[1], gains[2], note)
        )
    else:
        print("recommendation: %s" % (note,))
    print("report: %s" % (out_path,))


if __name__ == "__main__":
    main()

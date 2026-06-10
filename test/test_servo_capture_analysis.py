import importlib.util
import json
import os
import struct

import numpy as np
import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_capture.py",
)
_spec = importlib.util.spec_from_file_location("servo_capture_script", _SCRIPT)
sc = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sc)

CHANNELS = [
    {"name": "cycle_index", "dtype": "u64", "offset": 0},
    {"name": "flags", "dtype": "u8", "offset": 8},
    {"name": "target_counts", "dtype": "i32", "offset": 9},
    {"name": "position_demand", "dtype": "i32", "offset": 13},
    {"name": "position_actual", "dtype": "i32", "offset": 17},
    {"name": "following_error", "dtype": "i32", "offset": 21},
    {"name": "torque_actual", "dtype": "i16", "offset": 25},
    {"name": "statusword", "dtype": "u16", "offset": 27},
    {"name": "error_code", "dtype": "u16", "offset": 29},
]


def synth_capture(tmp_path, n=4000, move=(1000, 2000), freq_hz=80.0):
    """1 kHz capture: flat, then a move with an 80 Hz error tone, then a
    post-move exponential decay (settling), then flat."""
    fs = 1000.0
    t = np.arange(n) / fs
    ferr = np.zeros(n)
    ms, me = move
    ferr[ms:me] = 200.0 * np.sin(2 * np.pi * freq_hz * t[ms:me])
    decay = 150.0 * np.exp(-(t[me:] - t[me]) / 0.05)
    ferr[me:] = decay * np.cos(2 * np.pi * 30.0 * (t[me:] - t[me]))
    flags = np.zeros(n, dtype=np.uint8)
    flags[:] = 1  # torque enabled
    flags[ms:me] |= 2  # motion active
    target = np.cumsum(np.where(flags & 2, 100, 0)).astype(np.int64)
    torque = np.zeros(n, dtype=np.int16)
    torque[ms:me] = 950  # saturated during the move

    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "synth.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        for i in range(n):
            fe = int(round(ferr[i]))
            tgt = int(target[i])
            f.write(
                struct.pack(
                    "<QBiiiihHH",
                    i,
                    int(flags[i]),
                    tgt,
                    tgt,
                    tgt - fe,
                    fe,
                    int(torque[i]),
                    0x0627,
                    0,
                )
            )
    return path, ferr


def test_load_capture_reads_header_and_records(tmp_path):
    path, _ = synth_capture(tmp_path)
    header, data = sc.load_capture(path)
    assert header["version"] == 1
    assert len(data) == 4000
    assert data["cycle_index"][0] == 0
    assert data["cycle_index"][-1] == 3999


def test_refuses_failed_capture(tmp_path):
    path, _ = synth_capture(tmp_path)
    failed = path.replace(".scap", ".failed.scap")
    os.rename(path, failed)
    with pytest.raises(SystemExit):
        sc.load_capture(failed)


def test_truncated_file_parses_to_last_whole_record(tmp_path):
    path, _ = synth_capture(tmp_path)
    size = os.path.getsize(path)
    with open(path, "r+b") as f:
        f.truncate(size - 17)  # kill part of the last record
    _, data = sc.load_capture(path)
    assert len(data) == 3999


def test_motion_segments_found(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    segs = sc.motion_segments(data["flags"])
    assert segs == [(1000, 2000)]


def test_following_error_rms_matches_numpy(tmp_path):
    path, ferr = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    expected_rms = float(np.sqrt(np.mean(np.round(ferr[1000:2000]) ** 2)))
    assert m["moves"][0]["ferr_rms"] == pytest.approx(expected_rms, rel=0.01)
    assert m["moves"][0]["ferr_peak"] == pytest.approx(200.0, rel=0.02)


def test_resonance_peak_detected_at_80hz(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    segs = sc.motion_segments(data["flags"])
    freqs, psd = sc.moving_psd(data, segs, fs=1000.0)
    peaks = sc.top_peaks(freqs, psd, count=3)
    assert abs(peaks[0][0] - 80.0) < 2.5, "dominant peak at the injected 80 Hz"


def test_settling_time_in_expected_range(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    # 150*exp(-t/0.05) crosses 10 counts at t = 0.05*ln(15) ~ 135 ms
    assert 80 <= m["moves"][0]["settle_ms"] <= 300


def test_torque_saturation_fraction(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["torque_saturation_pct"] == pytest.approx(25.0, abs=1.0)


def test_drive_vs_recomputed_error_consistent(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["ferr_crosscheck_max"] == 0  # synth file is self-consistent

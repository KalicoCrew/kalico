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

FLAG_TORQUE_ENABLED = 1
FLAG_MOTION_ACTIVE = 2
DECAY_AMP_COUNTS = 150.0
DECAY_TAU_S = 0.05
SATURATED_TORQUE = 950

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
    decay = DECAY_AMP_COUNTS * np.exp(-(t[me:] - t[me]) / DECAY_TAU_S)
    ferr[me:] = decay * np.cos(2 * np.pi * 30.0 * (t[me:] - t[me]))
    flags = np.zeros(n, dtype=np.uint8)
    flags[:] = FLAG_TORQUE_ENABLED
    flags[ms:me] |= FLAG_MOTION_ACTIVE
    target = np.cumsum(np.where(flags & FLAG_MOTION_ACTIVE, 100, 0)).astype(
        np.int64
    )
    torque = np.zeros(n, dtype=np.int16)
    torque[ms:me] = SATURATED_TORQUE

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
    partial_last_record = os.path.getsize(path) - 17
    with open(path, "r+b") as f:
        f.truncate(partial_last_record)
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
    settle_band = 10
    m = sc.compute_metrics(data, settle_band=settle_band, torque_limit=900)
    decay_crosses_band_ms = (
        1000.0 * DECAY_TAU_S * np.log(DECAY_AMP_COUNTS / settle_band)
    )
    assert (
        0.6 * decay_crosses_band_ms
        <= m["moves"][0]["settle_ms"]
        <= 2.2 * decay_crosses_band_ms
    )


def test_torque_saturation_fraction(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["torque_saturation_pct"] == pytest.approx(25.0, abs=1.0)


def test_drive_vs_recomputed_error_consistent(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    assert m["ferr_crosscheck_max"] == 0, "synth file must be self-consistent"


def _write_header_only(tmp_path):
    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "empty.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
    return path


def test_empty_capture_loads_zero_records(tmp_path):
    path = _write_header_only(tmp_path)
    header, data = sc.load_capture(path)
    assert len(data) == 0


def test_empty_capture_compute_metrics_raises(tmp_path):
    path = _write_header_only(tmp_path)
    _, data = sc.load_capture(path)
    with pytest.raises(SystemExit):
        sc.compute_metrics(data, settle_band=10, torque_limit=900)


MOVE0_WITH_DECAYING_ERROR = slice(1000, 1500)
DWELL_BETWEEN_MOVES = slice(1500, 1530)
MOVE1_WITH_PERSISTENT_ERROR = slice(1530, 2000)


def synth_two_move_capture(tmp_path):
    n = 3000
    fs = 1000.0
    t = np.arange(n) / fs
    m0, dwell, m1 = (
        MOVE0_WITH_DECAYING_ERROR,
        DWELL_BETWEEN_MOVES,
        MOVE1_WITH_PERSISTENT_ERROR,
    )

    ferr = np.zeros(n)
    ferr[m0] = 80.0 * np.sin(2 * np.pi * 40.0 * t[m0])
    ferr[dwell] = 5.0 * np.exp(-np.arange(dwell.stop - dwell.start) / 5.0)
    ferr[m1] = 500.0

    flags = np.zeros(n, dtype=np.uint8)
    flags[:] = FLAG_TORQUE_ENABLED
    flags[m0] |= FLAG_MOTION_ACTIVE
    flags[m1] |= FLAG_MOTION_ACTIVE

    target = np.cumsum(np.where(flags & 2, 100, 0)).astype(np.int64)
    torque = np.zeros(n, dtype=np.int16)

    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "two_move.scap")
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
    return path


def test_per_move_post_window_not_contaminated(tmp_path):
    path = synth_two_move_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, settle_band=50, torque_limit=900)
    assert len(m["moves"]) == 2
    move0 = m["moves"][0]
    assert move0["overshoot"] < 50, (
        "move 0 overshoot contaminated by move 1 error: %s" % move0["overshoot"]
    )
    dwell_ms_at_1khz = DWELL_BETWEEN_MOVES.stop - DWELL_BETWEEN_MOVES.start
    if move0["settle_ms"] is not None:
        assert move0["settle_ms"] <= dwell_ms_at_1khz


MOVE_AT_500HZ = slice(200, 400)


def synth_500hz_capture(tmp_path):
    n = 1000
    fs = 500.0
    t = np.arange(n) / fs
    mv = MOVE_AT_500HZ

    ferr = np.zeros(n)
    ferr[mv] = 100.0 * np.sin(2 * np.pi * 20.0 * t[mv])

    flags = np.zeros(n, dtype=np.uint8)
    flags[:] = FLAG_TORQUE_ENABLED
    flags[mv] |= FLAG_MOTION_ACTIVE

    target = np.cumsum(np.where(flags & 2, 100, 0)).astype(np.int64)
    torque = np.zeros(n, dtype=np.int16)

    header = {
        "version": 1,
        "cycle_ns": 2_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": CHANNELS,
    }
    path = os.path.join(str(tmp_path), "500hz.scap")
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
    return path


def test_fs_aware_ms_at_500hz(tmp_path):
    path = synth_500hz_capture(tmp_path)
    _, data = sc.load_capture(path)
    fs = 1e9 / 2_000_000
    m = sc.compute_metrics(data, settle_band=10, torque_limit=900, fs=fs)
    move = m["moves"][0]
    ms_per_sample = 1000.0 / fs
    assert move["start_ms"] == pytest.approx(
        MOVE_AT_500HZ.start * ms_per_sample
    )
    assert move["end_ms"] == pytest.approx(MOVE_AT_500HZ.stop * ms_per_sample)


def test_fs_1khz_values_unchanged(tmp_path):
    """At fs=1000 Hz, sample index == ms — existing numeric expectations unchanged."""
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m_default = sc.compute_metrics(data, settle_band=10, torque_limit=900)
    m_explicit = sc.compute_metrics(
        data, settle_band=10, torque_limit=900, fs=1000.0
    )
    assert (
        m_default["moves"][0]["start_ms"] == m_explicit["moves"][0]["start_ms"]
    )
    assert m_default["moves"][0]["end_ms"] == m_explicit["moves"][0]["end_ms"]
    assert (
        m_default["moves"][0]["settle_ms"]
        == m_explicit["moves"][0]["settle_ms"]
    )
    assert m_default["moves"][0]["start_ms"] == 1000.0
    assert m_default["moves"][0]["end_ms"] == 2000.0


def test_load_capture_offset_mismatch_raises(tmp_path):
    channels_with_wrong_flags_offset = [dict(c) for c in CHANNELS]
    channels_with_wrong_flags_offset[1] = dict(
        channels_with_wrong_flags_offset[1], offset=999
    )
    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 31,
        "started_utc": "2026-06-10T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": channels_with_wrong_flags_offset,
    }
    path = os.path.join(str(tmp_path), "bad_offset.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
    with pytest.raises(SystemExit):
        sc.load_capture(path)


def test_main_requires_exactly_one_capture_source():
    with pytest.raises(SystemExit, match="not both or neither"):
        sc.main([])
    with pytest.raises(SystemExit, match="not both or neither"):
        sc.main(["/tmp/x.scap", "--name", "x"])


def test_resolve_newest_capture_picks_latest(tmp_path):
    for ts in ("20260611_210000", "20260611_230000"):
        with open(tmp_path / ("track_%s.scap" % ts), "w"):
            pass
    newest = sc.resolve_newest_capture(str(tmp_path), "track")
    assert newest.endswith("track_20260611_230000.scap")


def test_resolve_newest_capture_missing_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="track"):
        sc.resolve_newest_capture(str(tmp_path), "track")


def _ext_capture(tmp_path, vel_offsets, tq_offsets, moving_mask):
    channels = CHANNELS + [
        {"name": "velocity_offset", "dtype": "i32", "offset": 31},
        {"name": "torque_offset", "dtype": "i16", "offset": 35},
    ]
    header = {
        "version": 1,
        "cycle_ns": 1_000_000,
        "record_size": 37,
        "started_utc": "2026-06-12T12:00:00Z",
        "started_mono_ns": 0,
        "drives": [{"name": "x", "counts_per_mm": 3276.8}],
        "channels": channels,
    }
    path = os.path.join(str(tmp_path), "ext.scap")
    with open(path, "wb") as f:
        f.write((json.dumps(header) + "\n").encode())
        for i, (vo, tq, moving) in enumerate(
            zip(vel_offsets, tq_offsets, moving_mask)
        ):
            flags = FLAG_TORQUE_ENABLED | (FLAG_MOTION_ACTIVE if moving else 0)
            f.write(
                struct.pack(
                    "<QBiiiihHHih", i, flags, i, i, i, 0, 0, 0x0627, 0, vo, tq
                )
            )
    return path


def test_ff_offset_metrics_cover_only_motion_samples(tmp_path):
    path = _ext_capture(
        tmp_path,
        vel_offsets=[999999, -327680, 100, 0],
        tq_offsets=[500, -120, 3, 0],
        moving_mask=[False, True, True, False],
    )
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, 50, 900)
    assert m["ff_velocity_offset_max"] == 327680
    assert m["ff_torque_offset_max"] == 120


def test_ff_offset_metrics_absent_for_legacy_captures(tmp_path):
    path, _ = synth_capture(tmp_path)
    _, data = sc.load_capture(path)
    m = sc.compute_metrics(data, 50, 900)
    assert "ff_velocity_offset_max" not in m

from __future__ import annotations

import configparser
from unittest.mock import MagicMock, patch

import pytest
from klippy_testing import PrinterShim

BASE_CONFIG = """\
[danger_options]

[screws_tilt_adjust]
screw1: 10,30
screw1_name: front left screw
screw2: 155,30
screw2_name: front right screw
screw3: 155,190
screw3_name: rear right screw
"""


def _make_config(tmp_path, extra=""):
    cfg = tmp_path / "printer.cfg"
    cfg.write_text(BASE_CONFIG + extra)
    return cfg


def _build_sta(tmp_path, extra=""):
    cfg_file = _make_config(tmp_path, extra)
    start_args = {"config_file": str(cfg_file)}
    with PrinterShim(start_args) as printer:
        config = printer.load_config()
        sta_section = config.getsection("screws_tilt_adjust")
        with patch("klippy.extras.probe.ProbePointsHelper"):
            from klippy.extras.screws_tilt_adjust import ScrewsTiltAdjust

            sta = ScrewsTiltAdjust(sta_section)
    return sta


# --- Config parsing: legacy screw_thread ---


@pytest.mark.parametrize(
    "thread,expected_factor,expected_dir",
    [
        ("CW-M3", 0.5, "CW"),
        ("CCW-M3", 0.5, "CCW"),
        ("CW-M4", 0.7, "CW"),
        ("CCW-M4", 0.7, "CCW"),
        ("CW-M5", 0.8, "CW"),
        ("CCW-M5", 0.8, "CCW"),
        ("CW-M6", 1.0, "CW"),
        ("CCW-M6", 1.0, "CCW"),
        ("CW-M8", 1.25, "CW"),
        ("CCW-M8", 1.25, "CCW"),
    ],
)
def test_legacy_screw_thread(tmp_path, thread, expected_factor, expected_dir):
    sta = _build_sta(tmp_path, f"screw_thread: {thread}")
    assert sta.screw_pitch == expected_factor
    assert sta.screw_direction == expected_dir


# --- Config parsing: new universal params ---


def test_error_no_screw_params(tmp_path):
    with pytest.raises(configparser.Error):
        _build_sta(tmp_path)


def test_custom_screw_pitch_and_direction(tmp_path):
    sta = _build_sta(tmp_path, "screw_pitch: 1.5\nscrew_direction: CCW")
    assert sta.screw_pitch == 1.5
    assert sta.screw_direction == "CCW"


def test_error_screw_pitch_only(tmp_path):
    with pytest.raises(configparser.Error):
        _build_sta(tmp_path, "screw_pitch: 2.0")


def test_error_screw_direction_only(tmp_path):
    with pytest.raises(configparser.Error, match="Must specify either"):
        _build_sta(tmp_path, "screw_direction: CCW")


# --- Config parsing: error cases ---


def test_error_screw_thread_with_screw_pitch(tmp_path):
    with pytest.raises(configparser.Error, match="cannot be used together"):
        _build_sta(tmp_path, "screw_thread: CW-M3\nscrew_pitch: 0.5")


def test_error_screw_thread_with_screw_direction(tmp_path):
    with pytest.raises(configparser.Error, match="cannot be used together"):
        _build_sta(tmp_path, "screw_thread: CW-M3\nscrew_direction: CW")


def test_error_invalid_screw_direction(tmp_path):
    with pytest.raises(configparser.Error):
        _build_sta(tmp_path, "screw_pitch: 0.5\nscrew_direction: INVALID")


def test_error_invalid_screw_thread(tmp_path):
    with pytest.raises(configparser.Error, match="Invalid screw_thread"):
        _build_sta(tmp_path, "screw_thread: CW-M99")


def test_error_screw_pitch_zero(tmp_path):
    with pytest.raises(configparser.Error):
        _build_sta(tmp_path, "screw_pitch: 0\nscrew_direction: CW")


def test_error_screw_pitch_negative(tmp_path):
    with pytest.raises(configparser.Error):
        _build_sta(tmp_path, "screw_pitch: -1.0\nscrew_direction: CW")


def test_legacy_screw_thread_case_insensitive(tmp_path):
    sta = _build_sta(tmp_path, "screw_thread: cw-m3")
    assert sta.screw_pitch == 0.5
    assert sta.screw_direction == "CW"


# --- probe_finalize calculation tests ---


def _make_sta_for_calc(screw_pitch, screw_direction, screws, direction=None):
    """Build a minimal ScrewsTiltAdjust-like object for calculation tests."""
    sta = object.__new__(
        __import__(
            "klippy.extras.screws_tilt_adjust", fromlist=["ScrewsTiltAdjust"]
        ).ScrewsTiltAdjust
    )
    sta.screw_pitch = screw_pitch
    sta.screw_direction = screw_direction
    sta.screws = screws
    sta.direction = direction
    sta.max_diff = None
    sta.max_diff_error = False
    sta.results = {}
    sta.gcode = MagicMock()
    sta.gcode.error = Exception
    return sta


def test_probe_finalize_no_adjustment_needed():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CW", screws)
    positions = [[10, 30, 1.0], [155, 30, 1.0], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw1"]["is_base"] is True
    assert sta.results["screw2"]["adjust"] == "00:00"
    assert sta.results["screw3"]["adjust"] == "00:00"


def test_probe_finalize_cw_adjustment():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CW", screws)
    # Base z=1.0, screw2 z=0.5 → diff=0.5, turns=0.5/0.5=1.0 CW
    positions = [[10, 30, 1.0], [155, 30, 0.5], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw1"]["is_base"] is True
    assert sta.results["screw2"]["sign"] == "CW"
    assert sta.results["screw2"]["adjust"] == "01:00"
    assert sta.results["screw3"]["adjust"] == "00:00"


def test_probe_finalize_ccw_adjustment():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CW", screws)
    # Base z=1.0, screw2 z=1.5 → diff=-0.5, turns=-0.5/0.5=-1.0 → CCW
    positions = [[10, 30, 1.0], [155, 30, 1.5], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw2"]["sign"] == "CCW"
    assert sta.results["screw2"]["adjust"] == "01:00"


def test_probe_finalize_partial_turn():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CW", screws)
    # Base z=1.0, screw2 z=0.75 → diff=0.25, turns=0.25/0.5=0.5 → 00:30
    positions = [[10, 30, 1.0], [155, 30, 0.75], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw2"]["sign"] == "CW"
    assert sta.results["screw2"]["adjust"] == "00:30"


def test_probe_finalize_custom_factor():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    # Using screw_pitch=1.5 (e.g. a large lead screw)
    sta = _make_sta_for_calc(1.5, "CW", screws)
    # Base z=1.0, screw2 z=0.25 → diff=0.75, turns=0.75/1.5=0.5 → 00:30
    positions = [[10, 30, 1.0], [155, 30, 0.25], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw2"]["sign"] == "CW"
    assert sta.results["screw2"]["adjust"] == "00:30"


def test_probe_finalize_ccw_screw_direction():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CCW", screws)
    # Base z=1.0, screw2 z=0.5 → diff=0.5, adjust=1.0
    # CCW thread: positive adjust → sign CCW
    positions = [[10, 30, 1.0], [155, 30, 0.5], [155, 190, 1.0]]
    sta.probe_finalize([0, 0, 0], positions)

    assert sta.results["screw2"]["sign"] == "CCW"
    assert sta.results["screw2"]["adjust"] == "01:00"


def test_probe_finalize_max_deviation_error():
    screws = [
        ((10, 30), "front left"),
        ((155, 30), "front right"),
        ((155, 190), "rear right"),
    ]
    sta = _make_sta_for_calc(0.5, "CW", screws)
    sta.max_diff = 0.1
    # diff of 0.5 exceeds max_diff of 0.1
    positions = [[10, 30, 1.0], [155, 30, 0.5], [155, 190, 1.0]]

    with pytest.raises(Exception, match="exceeds configured limits"):
        sta.probe_finalize([0, 0, 0], positions)

    assert sta.max_diff_error is True

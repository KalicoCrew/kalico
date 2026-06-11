import importlib.util
import os

import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_gain_report.py",
)
_spec = importlib.util.spec_from_file_location("servo_gain_report_script", _SCRIPT)
sgr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sgr)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def test_named_steps_pick_newest_capture_per_step(tmp_path):
    d = str(tmp_path)
    _touch(d, "cal_p2000_s1250_i1000_20260611_210000.scap")
    newest = _touch(d, "cal_p2000_s1250_i1000_20260611_220000.scap")
    files = sgr.find_named_steps(d, ["cal_p2000_s1250_i1000"])
    assert files == [((2000, 1250, 1000), newest)]


def test_named_steps_exclude_stale_steps_from_other_runs(tmp_path):
    d = str(tmp_path)
    _touch(d, "cal_p2880_s1800_i694_20260610_180000.scap")
    kept = _touch(d, "cal_p2400_s1500_i833_20260611_220000.scap")
    files = sgr.find_named_steps(d, ["cal_p2400_s1500_i833"])
    assert files == [((2400, 1500, 833), kept)]


def test_named_steps_sort_by_speed_gain(tmp_path):
    d = str(tmp_path)
    slow = _touch(d, "cal_p1600_s1000_i1250_20260611_220000.scap")
    fast = _touch(d, "cal_p2400_s1500_i833_20260611_220010.scap")
    files = sgr.find_named_steps(
        d, ["cal_p2400_s1500_i833", "cal_p1600_s1000_i1250"]
    )
    assert files == [((1600, 1000, 1250), slow), ((2400, 1500, 833), fast)]


def test_named_step_without_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="cal_p2000_s1250_i1000"):
        sgr.find_named_steps(str(tmp_path), ["cal_p2000_s1250_i1000"])


def test_explicit_files_and_steps_are_mutually_exclusive(tmp_path):
    scap = _touch(str(tmp_path), "cal_p2000_s1250_i1000_20260611_220000.scap")
    with pytest.raises(SystemExit, match="not both"):
        sgr.main([scap, "--steps", "cal_p2000_s1250_i1000"])

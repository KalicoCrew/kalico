import argparse
import importlib.util
import os

import pytest

_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "servo_fit_dynamics.py",
)
_spec = importlib.util.spec_from_file_location("servo_fit_dynamics_script", _SCRIPT)
sfd = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sfd)


def _touch(directory, name):
    path = os.path.join(directory, name)
    with open(path, "w"):
        pass
    return path


def _args(**overrides):
    base = {
        "structure": "scalar",
        "rated_torque_nm": None,
        "rotor_inertia_kgm2": None,
        "rotation_distance_mm": None,
    }
    base.update(overrides)
    return argparse.Namespace(**base)


def test_resolves_newest_capture_for_name(tmp_path):
    d = str(tmp_path)
    _touch(d, "ident_20260611_210000.scap")
    newest = _touch(d, "ident_20260611_230000.scap")
    assert sfd.resolve_newest_capture(d, "ident") == newest


def test_missing_capture_fails_loudly(tmp_path):
    with pytest.raises(SystemExit, match="ident"):
        sfd.resolve_newest_capture(str(tmp_path), "ident")


def test_profile_name_carries_capture_timestamp(tmp_path):
    path = sfd.profile_path(
        str(tmp_path), "ident", "/x/ident_20260611_230000.scap"
    )
    assert os.path.basename(path) == "dynamics_ident_20260611_230000.toml"


def test_ident_cmd_without_physical_params():
    cmd = sfd.ident_cmd("/bin/servo-ident", "/tmp/c.csv", "node_x", "/o.toml", _args())
    assert cmd == [
        "/bin/servo-ident",
        "--capture",
        "/tmp/c.csv",
        "--structure",
        "scalar",
        "--axes",
        "node_x",
        "--out",
        "/o.toml",
    ]


def test_ident_cmd_appends_physical_params():
    cmd = sfd.ident_cmd(
        "/bin/servo-ident",
        "/tmp/c.csv",
        "node_x",
        "/o.toml",
        _args(
            rated_torque_nm=1.27,
            rotor_inertia_kgm2=0.000057,
            rotation_distance_mm=40.0,
        ),
    )
    assert cmd[-6:] == [
        "--rated-torque-nm",
        "1.27",
        "--rotor-inertia-kgm2",
        "5.7e-05",
        "--rotation-distance-mm",
        "40.0",
    ]

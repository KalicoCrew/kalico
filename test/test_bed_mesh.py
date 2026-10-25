"""Compatibility checks for shared bed-mesh parameter validation."""

import logging

import pytest

from klippy.extras import bed_mesh


def mesh_params(**overrides):
    return {
        "min_x": 20.0,
        "max_x": 100.0,
        "min_y": 20.0,
        "max_y": 100.0,
        "x_count": 3,
        "y_count": 3,
        "mesh_x_pps": 2,
        "mesh_y_pps": 2,
        "algo": "lagrange",
        "tension": 0.2,
        **overrides,
    }


def test_calibration_algorithm_selection_and_errors(caplog):
    calibration = bed_mesh.BedMeshCalibrate.__new__(bed_mesh.BedMeshCalibrate)
    for algorithm, counts, pps, expected in [
        ("bicubic", (4, 7), (2, 2), "bicubic"),
        ("bicubic", (3, 6), (2, 2), "lagrange"),
        ("lagrange", (7, 7), (0, 0), "direct"),
    ]:
        # Calibration validates before the geometry bounds exist.
        calibration.mesh_config = {
            "algo": algorithm,
            "x_count": counts[0],
            "y_count": counts[1],
            "mesh_x_pps": pps[0],
            "mesh_y_pps": pps[1],
        }
        with caplog.at_level(logging.INFO):
            calibration._verify_algorithm(ValueError)
        assert calibration.mesh_config["algo"] == expected
        if algorithm == "bicubic" and expected == "lagrange":
            assert "Forcing lagrange interpolation" in caplog.text

    for overrides, message in [
        (
            {"algo": "direct", "mesh_x_pps": 0, "mesh_y_pps": 0},
            "bed_mesh: Unknown algorithm <direct>",
        ),
        (
            {"x_count": 7},
            "bed_mesh: cannot exceed a probe_count of 6 when using lagrange interpolation. Configured Probe Count: 7, 3",
        ),
        (
            {"algo": "bicubic", "y_count": 7},
            "bed_mesh: invalid probe_count option when using bicubic interpolation.  Combination of 3 points on one axis with more than 6 on another is not permitted. Configured Probe Count: 3, 7",
        ),
    ]:
        calibration.mesh_config = mesh_params(**overrides)
        with pytest.raises(ValueError) as caught:
            calibration._verify_algorithm(ValueError)
        assert str(caught.value) == message


def test_round_mesh_padding_preserves_samples_and_repeats_edges():
    matrix = [[0.1], [0.2, 0.3, 0.4], [0.5, 0.6, 0.7, 0.8, 0.9]]
    bed_mesh.pad_round_mesh(matrix, 5, ValueError)
    assert matrix == [
        [0.1, 0.1, 0.1, 0.1, 0.1],
        [0.2, 0.2, 0.3, 0.4, 0.4],
        [0.5, 0.6, 0.7, 0.8, 0.9],
    ]

    for row in [[], [0.1, 0.2]]:
        invalid = [row]
        with pytest.raises(ValueError) as caught:
            bed_mesh.pad_round_mesh(invalid, 5, ValueError)
        assert str(caught.value) == (
            "bed_mesh: incorrect number of points sampled on X\nProbed Table:\n"
            + str(invalid)
        )


def test_two_point_direct_mesh_and_validation():
    params = mesh_params(
        algo="direct", x_count=2, y_count=2, mesh_x_pps=0, mesh_y_pps=0
    )
    bed_mesh.verify_mesh_params(params, ValueError)
    mesh = bed_mesh.ZMesh(params, "legacy")
    mesh.build_mesh([[0.0, 1.0], [2.0, 3.0]])
    assert (
        mesh.calc_z(-60, 180) == 2.0
    )  # Existing clamping remains the default.

    for overrides, message in [
        ({"max_x": 20.0}, "Invalid mesh parameters"),
        ({"min_x": float("nan")}, "Mesh parameters must be finite"),
        ({"algo": "direct"}, "Direct meshes require mesh_pps: 0"),
    ]:
        with pytest.raises(ValueError) as caught:
            bed_mesh.verify_mesh_params(mesh_params(**overrides), ValueError)
        assert str(caught.value) == message

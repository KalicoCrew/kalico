from __future__ import annotations

import math

from scripts.fitter_prototype.params import FitterParams
from scripts.g1_to_g5_postprocess import postprocess


def test_smooth_extruding_run_emits_g5_and_preserves_final_e():
    e = 0.0
    lines = ["G90", "M82", "G1 X0 Y0 E0 F1200"]
    for k in range(1, 30):
        e += 0.01
        lines.append(
            f"G1 X{k * 0.2:.4f} Y{2 * math.sin(k * 0.03):.4f} E{e:.5f}"
        )

    output, stats = postprocess(
        "\n".join(lines) + "\n",
        params=FitterParams(),
        min_points=4,
        include_travel=False,
    )

    assert stats.runs_converted == 1
    assert stats.g1_lines_replaced == 29
    assert "G5 " in output
    assert "E0.29" in output


def test_non_extruding_run_is_preserved_by_default():
    text = "\n".join(
        [
            "G90",
            "M82",
            "G1 X0 Y0 F1200",
            "G1 X1 Y0",
            "G1 X2 Y0",
            "G1 X3 Y0",
        ]
    )

    output, stats = postprocess(
        text,
        params=FitterParams(),
        min_points=4,
        include_travel=False,
    )

    assert stats.runs_converted == 0
    assert "G5 " not in output
    assert output == text


def test_relative_extrusion_mode_emits_relative_g5_e_values():
    e_values = []
    lines = ["G90", "M83", "G1 X0 Y0 E0 F1200"]
    for k in range(1, 30):
        e = 0.01
        e_values.append(e)
        lines.append(
            f"G1 X{k * 0.2:.4f} Y{2 * math.sin(k * 0.03):.4f} E{e:.5f}"
        )

    output, stats = postprocess(
        "\n".join(lines) + "\n",
        params=FitterParams(),
        min_points=4,
        include_travel=False,
    )

    emitted_e = [
        float(field[1:])
        for line in output.splitlines()
        if line.startswith("G5 ")
        for field in line.split()
        if field.startswith("E")
    ]
    assert stats.runs_converted == 1
    assert emitted_e
    assert abs(sum(emitted_e) - sum(e_values)) < 1e-4

from __future__ import annotations

from scripts.fitter_prototype.parser import (
    Arc,
    Move,
    parse,
)


def test_parse_simple_g1_sequence():
    text = """
G1 X10 Y20 F1500
G1 X20 Y20
G1 X20 Y10 ; trailing comment
"""
    tokens = parse(text)
    assert all(isinstance(t, Move) for t in tokens)
    assert [t.kind for t in tokens] == ["G1", "G1", "G1"]
    assert tokens[0].x == 10.0
    assert tokens[0].y == 20.0
    assert tokens[2].x == 20.0


def test_parse_arc_with_ij():
    text = "G2 X10 Y0 I5 J0\n"
    tokens = parse(text)
    assert len(tokens) == 1
    assert isinstance(tokens[0], Arc)
    assert tokens[0].kind == "G2"
    assert tokens[0].x == 10.0
    assert tokens[0].i == 5.0


def test_parse_marker_for_nonmotion():
    text = """
G1 X1 Y1
M104 S210
G1 X2 Y2
"""
    tokens = parse(text)
    assert [type(t).__name__ for t in tokens] == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "M104"


def test_parse_g0_is_marker():
    text = "G1 X1 Y1\nG0 X5 Y5\nG1 X6 Y6\n"
    tokens = parse(text)
    assert [type(t).__name__ for t in tokens] == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "G0"


def test_parse_strips_comments_and_blank_lines():
    text = "; pure comment\n\nG1 X1 Y1\n; another\n"
    tokens = parse(text)
    assert len(tokens) == 1
    assert isinstance(tokens[0], Move)


def test_parse_z_change_is_marker():
    text = "G1 X1 Y1\nG1 Z0.4\nG1 X2 Y2\n"
    tokens = parse(text)
    kinds = [type(t).__name__ for t in tokens]
    assert kinds == ["Move", "Marker", "Move"]
    assert tokens[1].reason == "Z_only"

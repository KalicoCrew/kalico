#!/usr/bin/env python3
"""Unit tests for motion_toolhead._derive_mcu_topology (pure, no klippy boot)."""
from __future__ import annotations

from klippy.motion_toolhead import _derive_mcu_topology


def test_corexy_two_mcu_extruder_on_octopus():
    # X,Y,E on handle 7 (octopus); Z on handle 9 (f446).
    axis_to_handle = {0: 7, 1: 7, 3: 7, 2: 9}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(7, [0, 1, 3], 0), (9, [2], 1)]


def test_cartesian_single_mcu_all_axes():
    axis_to_handle = {0: 5, 1: 5, 2: 5, 3: 5}
    topo = _derive_mcu_topology(axis_to_handle, "cartesian")
    assert topo == [(5, [0, 1, 2, 3], 1)]


def test_corexy_single_mcu_gets_corexy_tag():
    axis_to_handle = {0: 5, 1: 5, 2: 5, 3: 5}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(5, [0, 1, 2, 3], 0)]


def test_corexy_z_only_mcu_is_cartesian():
    # An MCU lacking the X/Y pair is cartesian even on a corexy printer.
    axis_to_handle = {2: 9}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(9, [2], 1)]


if __name__ == "__main__":
    test_corexy_two_mcu_extruder_on_octopus()
    test_cartesian_single_mcu_all_axes()
    test_corexy_single_mcu_gets_corexy_tag()
    test_corexy_z_only_mcu_is_cartesian()
    print("all passed")

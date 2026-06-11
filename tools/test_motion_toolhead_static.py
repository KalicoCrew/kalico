#!/usr/bin/env python3
from __future__ import annotations

import importlib

import pytest

from klippy.motion_toolhead import MotionToolhead
from klippy.toolhead import ToolHead

pytestmark = pytest.mark.sim_unit

EXPECTED_LOCAL_METHODS = frozenset(
    {
        "__init__",
        "_load_kinematics",
        "move",
        "_fire_active_callbacks",
        "drip_move",
        "dwell",
        "check_busy",
        "_sync_print_time",
        "wait_moves",
        "flush_step_generation",
        "get_last_move_time",
        "note_mcu_movequeue_activity",
        "set_accel",
        "reset_accel",
        "cmd_SET_VELOCITY_LIMIT",
        "cmd_RESET_VELOCITY_LIMIT",
        "stats",
        "_init_planner",
        "_configure_axes_per_mcu",
        "cmd_KALICO_SIM_STEP_COUNT",
        "cmd_KALICO_SIM_AXIS_STEPS",
        "cmd_KALICO_SIM_AXIS_ACCUM",
        "cmd_KALICO_SIM_ENDSTOP_SET_PIN",
        "cmd_KALICO_SIM_MOTION_STATE",
        "cmd_M400",
        "cmd_KALICO_DIAG_DUMP",
        "_handle_disconnect",
        "wait_moves_and_mcu",
        "_bridge_mcus",
        "_ground_pending_end_time_after_bridge_drain",
        "_bump_pending_end_time",
    }
)


def test_motion_toolhead_override_surface_matches_baseline():
    actual = {
        name
        for name, value in MotionToolhead.__dict__.items()
        if callable(value) and not name.startswith("__")
    }
    actual.add("__init__")
    extra = actual - EXPECTED_LOCAL_METHODS
    missing = EXPECTED_LOCAL_METHODS - actual
    assert not extra and not missing, (
        "Override surface drift!\n"
        "  Added (in MotionToolhead but not in baseline): %s\n"
        "  Removed (in baseline but not in MotionToolhead): %s\n"
        "If intentional, update EXPECTED_LOCAL_METHODS in this file."
        % (sorted(extra), sorted(missing))
    )


EXPECTED_TOOLHEAD_METHODS = frozenset(
    {
        "__init__",
        "_advance_flush_time",
        "_advance_move_time",
        "_calc_junction_deviation",
        "_calc_print_time",
        "_check_pause",
        "_flush_handler",
        "_flush_lookahead",
        "_handle_shutdown",
        "_load_kinematics",
        "_priming_handler",
        "_process_moves",
        "_update_drip_move_time",
        "check_busy",
        "cmd_G4",
        "cmd_M204",
        "cmd_M400",
        "cmd_RESET_VELOCITY_LIMIT",
        "cmd_SET_VELOCITY_LIMIT",
        "drip_move",
        "dwell",
        "flush_step_generation",
        "get_active_rails_for_axis",
        "get_extruder",
        "get_kinematics",
        "get_last_move_time",
        "get_max_velocity",
        "get_position",
        "get_status",
        "get_trapq",
        "limit_next_junction_speed",
        "manual_move",
        "move",
        "note_mcu_movequeue_activity",
        "note_step_generation_scan_time",
        "register_lookahead_callback",
        "register_step_generator",
        "reset_accel",
        "set_accel",
        "set_extruder",
        "set_position",
        "stats",
        "wait_moves",
    }
)


def test_upstream_toolhead_method_baseline():
    actual = {
        name
        for name, value in ToolHead.__dict__.items()
        if callable(value) and not name.startswith("__")
    }
    actual.add("__init__")
    extra = actual - EXPECTED_TOOLHEAD_METHODS
    missing = EXPECTED_TOOLHEAD_METHODS - actual
    assert not extra and not missing, (
        "Upstream ToolHead method baseline drift!\n"
        "  Added in ToolHead: %s — REVIEW whether MotionToolhead needs\n"
        "    to override this for bridge-mode correctness.\n"
        "  Removed from ToolHead: %s — the bridge override may now be\n"
        "    overriding nothing.\n"
        "If reviewed and OK, update EXPECTED_TOOLHEAD_METHODS in this file."
        % (sorted(extra), sorted(missing))
    )


def test_no_motion_toolhead_path_calls_note_mcu_movequeue_activity():
    import inspect

    forbidden = "note_mcu_movequeue_activity"
    overrides = {}
    for name in EXPECTED_LOCAL_METHODS:
        val = MotionToolhead.__dict__.get(name)
        if val is None or not callable(val):
            continue
        try:
            overrides[name] = inspect.getsource(val)
        except (TypeError, OSError):
            continue
    offenders = []
    for name, src in overrides.items():
        if name == forbidden:
            continue
        if (".%s(" % forbidden) in src or ("self.%s(" % forbidden) in src:
            offenders.append(name)
    assert not offenders, (
        "Flush-timer silencing invariant violated!\n"
        "These MotionToolhead methods invoke %s, which rearms the "
        "silenced flush_timer:\n  %s" % (forbidden, offenders)
    )


def test_legacy_toolhead_module_imports_without_bridge():
    th = importlib.import_module("klippy.toolhead")
    assert hasattr(th, "ToolHead")
    assert hasattr(th, "LookAheadQueue")
    assert hasattr(th, "Move")
    assert hasattr(th, "DripModeEndSignal")
    assert hasattr(th, "add_printer_objects")
    assert th.BUFFER_TIME_START == 0.250
    assert th.BUFFER_TIME_HIGH == 2.0
    assert th.SDS_CHECK_TIME == 0.001
    assert hasattr(th.ToolHead, "_load_kinematics")

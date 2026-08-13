"""Behavior tests for TMCVirtualPinHelper and BaseTMCCurrentHelper.

These exercise the sensorless homing save/restore contract and the Kalico
home_current current-switching logic. The goal is to lock down observable
behavior so it survives the upstream "simplify homing start/end" refactor
(Klipper PR #7193) and the TMCCommandHelper cleanup (Klipper PR #7154).
"""

from __future__ import annotations

import collections

import pytest

from klippy.extras import tmc2130, tmc2209, tmc2240
from klippy.extras.tmc import (
    BaseTMCCurrentHelper,
    FieldHelper,
    TMCVirtualPinHelper,
)


class FakeMCUTMC:
    """Captures set_register calls and returns field values on demand."""

    def __init__(self, fields: FieldHelper):
        self.fields = fields
        self.writes: list[tuple[str, int]] = []

    def get_fields(self):
        return self.fields

    def set_register(self, reg_name, val, print_time=None):
        self.writes.append((reg_name, val))
        # mirror the write in the register store so subsequent get_field reflects it
        self.fields.registers[reg_name] = val


def _make_fields(driver_module) -> FieldHelper:
    return FieldHelper(
        driver_module.Fields,
        signed_fields=getattr(driver_module, "SignedFields", []),
    )


def _make_vph(driver_module, diag_pin_field):
    """Build a TMCVirtualPinHelper bypassing __init__ (which needs a printer).

    Initializes both the pre-refactor scalar attributes and the post-refactor
    OrderedDict caches so the tests run on either implementation without
    needing to be rewritten.
    """
    fields = _make_fields(driver_module)
    mcu_tmc = FakeMCUTMC(fields)
    vph = object.__new__(TMCVirtualPinHelper)
    vph.mcu_tmc = mcu_tmc
    vph.fields = fields
    vph.diag_pin = "PA1"
    vph.diag_pin_field = diag_pin_field
    # mcu_endstop is compared with identity to hmove.get_mcu_endstops()
    vph.mcu_endstop = object()
    # pre-refactor attributes (ignored by the new implementation)
    vph.en_pwm = False
    vph.pwmthrs = 0
    vph.coolthrs = 0
    vph.thigh = 0
    # post-refactor attributes (ignored by the old implementation)
    vph._dirty_regs = collections.OrderedDict()
    vph._prev_state = collections.OrderedDict()
    return vph, fields, mcu_tmc


class FakeHMove:
    def __init__(self, endstops):
        self._endstops = endstops

    def get_mcu_endstops(self):
        return self._endstops


# ---------------------------------------------------------------------------
# TMCVirtualPinHelper — round-trip invariant
# ---------------------------------------------------------------------------


def _snapshot(fields: FieldHelper, names):
    return {n: fields.get_field(n) for n in names}


@pytest.mark.parametrize(
    "driver_module,diag_pin_field,extra_fields",
    [
        (tmc2130, "diag1_stall", []),
        (tmc2240, "diag1_stall", ["en_pwm_mode"]),
    ],
    ids=["tmc2130", "tmc2240"],
)
def test_homing_begin_end_round_trip(
    driver_module, diag_pin_field, extra_fields
):
    """After begin+end, every touched field must return to its original value."""
    vph, fields, mcu_tmc = _make_vph(driver_module, diag_pin_field)

    # Configure a realistic starting state with NON-default values so we can
    # actually see save/restore occur (default zeros would hide bugs).
    fields.set_field("tpwmthrs", 0x12345)
    fields.set_field("tcoolthrs", 0x6789A)
    fields.set_field("thigh", 0x0ABCD)
    fields.set_field("en_pwm_mode", 1)
    fields.set_field(diag_pin_field, 0)

    touched = [
        "tpwmthrs",
        "tcoolthrs",
        "thigh",
        "en_pwm_mode",
        diag_pin_field,
    ]
    before = _snapshot(fields, touched)

    hmove = FakeHMove([vph.mcu_endstop])
    vph.handle_homing_move_begin(hmove)
    vph.handle_homing_move_end(hmove)

    after = _snapshot(fields, touched)
    assert after == before, (
        f"Round-trip mismatch: before={before} after={after}"
    )


@pytest.mark.parametrize(
    "driver_module,diag_pin_field",
    [(tmc2130, "diag1_stall"), (tmc2240, "diag1_stall")],
    ids=["tmc2130", "tmc2240"],
)
def test_homing_begin_sets_homing_values(driver_module, diag_pin_field):
    """During homing, the driver must be put into its stall-safe state."""
    vph, fields, mcu_tmc = _make_vph(driver_module, diag_pin_field)

    fields.set_field("tpwmthrs", 0x12345)
    fields.set_field("tcoolthrs", 0)  # unset, should be forced to 0xFFFFF
    fields.set_field("thigh", 0x0ABCD)
    fields.set_field("en_pwm_mode", 1)
    fields.set_field(diag_pin_field, 0)

    hmove = FakeHMove([vph.mcu_endstop])
    vph.handle_homing_move_begin(hmove)

    # stealthchop must be disabled on drivers that have en_pwm_mode
    assert fields.get_field("en_pwm_mode") == 0
    # diag pin must be routed to stall for this axis
    assert fields.get_field(diag_pin_field) == 1
    # thigh must be zero during homing
    assert fields.get_field("thigh") == 0
    # tcoolthrs must be forced to the max-sensitivity value since it was 0
    assert fields.get_field("tcoolthrs") == 0xFFFFF


def test_homing_preserves_nonzero_tcoolthrs_begin_state():
    """If tcoolthrs was already configured, homing should not override it."""
    vph, fields, mcu_tmc = _make_vph(tmc2130, "diag1_stall")

    fields.set_field("tcoolthrs", 0x555)
    fields.set_field("thigh", 0)
    fields.set_field("en_pwm_mode", 0)
    fields.set_field("diag1_stall", 0)

    hmove = FakeHMove([vph.mcu_endstop])
    vph.handle_homing_move_begin(hmove)

    assert fields.get_field("tcoolthrs") == 0x555, (
        "pre-configured tcoolthrs should survive homing begin"
    )


def test_homing_skipped_when_endstop_not_in_move():
    """No register writes when the move doesn't include our endstop."""
    vph, fields, mcu_tmc = _make_vph(tmc2130, "diag1_stall")

    fields.set_field("tpwmthrs", 0x12345)

    hmove = FakeHMove([object()])  # different endstop
    vph.handle_homing_move_begin(hmove)
    vph.handle_homing_move_end(hmove)

    assert mcu_tmc.writes == []
    assert fields.get_field("tpwmthrs") == 0x12345


# ---------------------------------------------------------------------------
# TMC2209 (stallguard4 / no en_pwm_mode) — round trip
# ---------------------------------------------------------------------------


def test_tmc2209_sg4_round_trip():
    """TMC2209 uses en_spreadcycle instead of en_pwm_mode. Must also round-trip."""
    # tmc2209 inherits most fields from tmc2208; ensure we have what we need
    if "en_spreadcycle" not in {
        f for reg in tmc2209.Fields.values() for f in reg
    }:
        pytest.skip("tmc2209 does not expose en_spreadcycle in test data")

    fields = _make_fields(tmc2209)
    mcu_tmc = FakeMCUTMC(fields)
    vph = object.__new__(TMCVirtualPinHelper)
    vph.mcu_tmc = mcu_tmc
    vph.fields = fields
    vph.diag_pin = "PA1"
    vph.diag_pin_field = None  # SG4 drivers have no diag pin field
    vph.mcu_endstop = object()
    vph.en_pwm = False
    vph.pwmthrs = 0
    vph.coolthrs = 0
    vph.thigh = 0
    vph._dirty_regs = collections.OrderedDict()
    vph._prev_state = collections.OrderedDict()

    fields.set_field("tpwmthrs", 0x22222)
    fields.set_field("tcoolthrs", 0x33333)
    fields.set_field("en_spreadcycle", 1)  # was spreadCycle, not stealth

    touched = ["tpwmthrs", "tcoolthrs", "en_spreadcycle"]
    before = _snapshot(fields, touched)

    hmove = FakeHMove([vph.mcu_endstop])
    vph.handle_homing_move_begin(hmove)

    # During homing on SG4 drivers, stealthchop must be ON (spreadcycle OFF)
    assert fields.get_field("en_spreadcycle") == 0
    assert fields.get_field("tpwmthrs") == 0

    vph.handle_homing_move_end(hmove)
    after = _snapshot(fields, touched)
    assert after == before, (
        f"tmc2209 round-trip mismatch: before={before} after={after}"
    )


# ---------------------------------------------------------------------------
# BaseTMCCurrentHelper — Kalico home_current logic
# ---------------------------------------------------------------------------


class _NoopCurrentHelper(BaseTMCCurrentHelper):
    """Bare helper built by hand; bypasses config parsing entirely."""

    def __init__(
        self,
        run_current=0.8,
        home_current=0.4,
        hold_current=0.6,
        max_current=2.0,
    ):
        # bypass BaseTMCCurrentHelper.__init__ (which reads from config)
        self.printer = None
        self.name = "stepper_x"
        self.mcu_tmc = None
        self.fields = None
        self.sense_resistor = 0.11
        self.max_current = max_current
        self.config_run_current = run_current
        self.config_hold_current = hold_current
        self.config_home_current = home_current
        self.current_change_dwell_time = 0.5
        self.req_run_current = run_current
        self.req_hold_current = hold_current
        self.req_home_current = home_current
        self.actual_current = run_current
        self.set_current_calls: list[tuple] = []

    def set_current(self, new_current, hold_current, print_time, force=False):
        # record the call, then update actual_current exactly like the real
        # set_current does via set_actual_current
        self.set_current_calls.append(
            (new_current, hold_current, print_time, force)
        )
        self.actual_current = new_current
        self.req_hold_current = hold_current

    def apply_current(self, print_time):
        pass


def test_needs_home_current_change_true_when_mismatch():
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.4)
    assert ch.needs_home_current_change() is True


def test_needs_home_current_change_false_when_equal():
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.8)
    assert ch.needs_home_current_change() is False


def test_set_current_for_homing_pre_switches_to_home_current():
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.4)
    dwell = ch.set_current_for_homing(print_time=1.0, pre_homing=True)

    assert dwell == ch.current_change_dwell_time
    assert ch.set_current_calls[-1][0] == pytest.approx(0.4)
    assert ch.actual_current == pytest.approx(0.4)


def test_set_current_for_homing_post_restores_run_current():
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.4)
    # simulate pre-homing already happened
    ch.set_current_for_homing(print_time=1.0, pre_homing=True)
    assert ch.actual_current == pytest.approx(0.4)

    dwell = ch.set_current_for_homing(print_time=2.0, pre_homing=False)

    assert dwell == ch.current_change_dwell_time
    assert ch.actual_current == pytest.approx(0.8)


def test_set_current_for_homing_noop_when_already_at_target():
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.8)
    dwell = ch.set_current_for_homing(print_time=1.0, pre_homing=True)
    assert dwell == 0.0
    assert ch.set_current_calls == []


def test_set_home_current_capped_at_max_current():
    ch = _NoopCurrentHelper(home_current=0.4, max_current=1.0)
    ch.set_home_current(99.0)
    assert ch.req_home_current == pytest.approx(1.0)


def test_set_home_current_below_max_passes_through():
    ch = _NoopCurrentHelper(home_current=0.4, max_current=2.0)
    ch.set_home_current(0.7)
    assert ch.req_home_current == pytest.approx(0.7)


def test_post_homing_path_runs_even_if_pre_did_nothing():
    """Regression: after a homing where run_current was already set,
    we still need to consider returning to run_current on post."""
    ch = _NoopCurrentHelper(run_current=0.8, home_current=0.8)
    # pre: no-op
    dwell_pre = ch.set_current_for_homing(print_time=1.0, pre_homing=True)
    # some external force moved actual_current off run (simulating the
    # sensorless bugfix path where SET_TMC_CURRENT swapped things mid-flight)
    ch.actual_current = 0.4
    dwell_post = ch.set_current_for_homing(print_time=2.0, pre_homing=False)

    assert dwell_pre == 0.0
    assert dwell_post == ch.current_change_dwell_time
    assert ch.actual_current == pytest.approx(0.8)

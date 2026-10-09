"""Unit tests for the TMC5262 driver module.

Static checks on the register/field map:
  * Every Fields register has a known address
  * Every ReadRegister is in Registers
  * Field masks are contiguous and non-overlapping
  * Round-trip through FieldHelper at maximum value
  * Critical bit positions Kalico depends on (sensorless homing,
    StealthChop+, ADC, etc.)

Integration coverage (DUMP_TMC, INIT_TMC, SET_TMC_CURRENT) lives in
test/klippy/tmc5262.test.
"""

from __future__ import annotations

import pytest

from klippy.extras import tmc5262
from klippy.extras.tmc import FieldHelper


def _make_fields() -> FieldHelper:
    return FieldHelper(tmc5262.Fields, tmc5262.SignedFields)


def _ffs(mask: int) -> int:
    return (mask & -mask).bit_length() - 1


# ---------------------------------------------------------------------------
# Static layout sanity
# ---------------------------------------------------------------------------


def test_every_field_register_has_an_address():
    missing = [r for r in tmc5262.Fields if r not in tmc5262.Registers]
    assert not missing, f"Fields without a register address: {missing}"


def test_every_read_register_is_known():
    missing = [r for r in tmc5262.ReadRegisters if r not in tmc5262.Registers]
    assert not missing, f"ReadRegisters not in Registers: {missing}"


@pytest.mark.parametrize("reg", sorted(tmc5262.Fields))
def test_field_masks_are_contiguous_and_non_overlapping(reg):
    seen = 0
    for fname, mask in tmc5262.Fields[reg].items():
        assert mask, f"{reg}.{fname} has empty mask"
        shifted = mask >> _ffs(mask)
        assert (shifted & (shifted + 1)) == 0, (
            f"{reg}.{fname} mask 0x{mask:08x} is not contiguous"
        )
        assert (seen & mask) == 0, f"{reg}.{fname} overlaps another field"
        seen |= mask


def test_field_roundtrip_at_max_value():
    fields = _make_fields()
    for reg, fmap in tmc5262.Fields.items():
        for fname, mask in fmap.items():
            width = bin(mask).count("1")
            maxv = (1 << width) - 1
            fields.set_field(fname, maxv)
            got = fields.get_field(fname)
            if fname in fields.signed_fields:
                assert got in (
                    -1,
                    maxv,
                ), f"{reg}.{fname} signed roundtrip got {got}"
            else:
                assert got == maxv, f"{reg}.{fname} roundtrip got {got}"


# ---------------------------------------------------------------------------
# Critical bits the rest of Kalico depends on
# ---------------------------------------------------------------------------


def test_step_dir_bit_is_at_31():
    # step/dir mode is essential for use as an external driver. Bit 31 of
    # GCONF is what flips the chip out of motion-controller mode.
    assert tmc5262.Fields["GCONF"]["step_dir"] == 1 << 31


def test_en_pwm_mode_aliases_en_stealthchop_bit():
    # tmc.TMCStealthchopHelper looks up "en_pwm_mode"; we must keep that
    # name pointing at GCONF bit 1 (en_stealthchop in the datasheet).
    assert tmc5262.Fields["GCONF"]["en_pwm_mode"] == 1 << 1


def test_irun_field_is_eight_bits_at_offset_8():
    # IHOLD_IRUN was widened from 5 to 8 bits on TMC5262.
    assert tmc5262.Fields["IHOLD_IRUN"]["irun"] == 0xFF << 8


def test_drv_status_uses_10bit_sg_result_and_8bit_cs_actual():
    assert tmc5262.Fields["DRV_STATUS"]["sg_result"] == 0x3FF
    assert tmc5262.Fields["DRV_STATUS"]["cs_actual"] == 0xFF << 16


def test_pwmconf_sd_on_meas_bit_positions():
    # Datasheet pg 168: SD_ON_MEAS_LO at PWMCONF[15:12], SD_ON_MEAS_HI at
    # [19:16]. Wrong positions silently disable ICS sampling.
    assert tmc5262.Fields["PWMCONF"]["sd_on_meas_lo"] == 0x0F << 12
    assert tmc5262.Fields["PWMCONF"]["sd_on_meas_hi"] == 0x0F << 16


def test_cur_angle_meas_bit_positions():
    # Datasheet pg 197: AMPL_MEAS at [11:0], ANGLE_MEAS at [25:16].
    assert tmc5262.Fields["CUR_ANGLE_MEAS"]["ampl_meas"] == 0xFFF
    assert tmc5262.Fields["CUR_ANGLE_MEAS"]["angle_meas"] == 0x3FF << 16


def test_adc_i_fields_are_signed():
    # ADC_I_A/B are signed 12-bit per datasheet pg 75.
    assert "adc_i_a" in tmc5262.SignedFields
    assert "adc_i_b" in tmc5262.SignedFields


def test_do_conf_stall_fields_present():
    # Sensorless homing on TMC5262 routes stall through DO_CONF.do0_stall
    # / do1_stall instead of GCONF.diag*_stall on older chips.
    assert "do0_stall" in tmc5262.Fields["DO_CONF"]
    assert "do1_stall" in tmc5262.Fields["DO_CONF"]
    assert tmc5262.Fields["DO_CONF"]["do0_stall"] == 0x01 << 2
    assert tmc5262.Fields["DO_CONF"]["do1_stall"] == 0x01 << 15


def test_do_conf_polarity_fields_present():
    # do0_invpp / do1_invpp control DO0 / DO1 output polarity.
    assert tmc5262.Fields["DO_CONF"]["do0_invpp"] == 0x01 << 29
    assert tmc5262.Fields["DO_CONF"]["do1_invpp"] == 0x01 << 31


def test_temp_from_adc_matches_datasheet_formula():
    # Datasheet pg 134: T(°C) = 1.042 * v - 264.6 (9-bit ADC code).
    # tmc.TMCErrorCheck consumes this via getattr(mcu_tmc, "temp_from_adc").
    assert tmc5262._adc_to_celsius(278) == pytest.approx(25.07, abs=0.01)
    assert tmc5262._adc_to_celsius(360) == pytest.approx(110.52, abs=0.01)


def test_pi_regulator_fields_present():
    # TMC5262 replaces TMC2240's StealthChop2 PWM_GRAD/OFS/REG/LIM with
    # PI regulators (datasheet pp 33-40). These need to be exposed.
    assert tmc5262.Fields["CURRENT_PI_REG"]["cur_p"] == 0xFFF
    assert tmc5262.Fields["CURRENT_PI_REG"]["cur_i"] == 0x3FF << 16
    assert tmc5262.Fields["ANGLE_PI_REG"]["angle_p"] == 0xFFF
    assert tmc5262.Fields["ANGLE_PI_REG"]["angle_i"] == 0x3FF << 16
    assert tmc5262.Fields["CUR_ANGLE_LIMIT"]["cur_pi_limit"] == 0xFFF << 16
    assert tmc5262.Fields["CUR_ANGLE_LIMIT"]["angle_pi_limit"] == 0x3FF
    assert tmc5262.Fields["PI_RESULTS"]["pwm_calc"] == 0x1FFF
    assert tmc5262.Fields["PI_RESULTS"]["angle_corr_calc"] == 0x3FF << 16
    assert "angle_error" in tmc5262.SignedFields
    assert "angle_corr_calc" in tmc5262.SignedFields

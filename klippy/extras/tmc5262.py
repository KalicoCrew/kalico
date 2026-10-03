# TMC5262 configuration
#
# Copyright (C) 2026  Kalico Contributors
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from . import tmc, tmc2130

# Internal 16MHz oscillator divided down to 1MHz fCLK (CLOCK_DIVIDER=15)
TMC_FREQUENCY = 16000000.0


# ADC conversion formulas (datasheet pg 134, 9-bit ADC)
def _adc_to_celsius(adc_value):
    return 1.042 * adc_value - 264.6


def _adc_to_volts(adc_value):
    return adc_value * 0.1409


######################################################################
# Register addresses (TMC5262 datasheet pp 121..136)
######################################################################

Registers = {
    # General configuration
    "GCONF": 0x00,
    "GSTAT": 0x01,
    "DO_CONF": 0x02,
    "DO_SCOPE_CONF": 0x03,
    "IOIN": 0x04,
    "X_COMPARE": 0x05,
    "X_COMPARE_REPEAT": 0x06,
    "DRV_CONF": 0x0A,
    "PLL": 0x0B,
    # Velocity dependent configuration
    "IHOLD_IRUN": 0x10,
    "TPOWERDOWN": 0x11,
    "TSTEP": 0x12,
    "TPWMTHRS": 0x13,
    "TCOOLTHRS": 0x14,
    "THIGH": 0x15,
    "TSGP_LOW_VEL_THRS": 0x16,
    "T_RCOIL_MEAS": 0x17,
    "TUDCSTEP": 0x18,
    "UDC_CONF": 0x19,
    "STEPS_LOST": 0x1A,
    # StealthChop+ PI regulator tuning + readbacks (datasheet pp 33-40)
    "CURRENT_PI_REG": 0x40,
    "ANGLE_PI_REG": 0x41,
    "CUR_ANGLE_LIMIT": 0x42,
    "ANGLE_LOWER_LIMIT": 0x43,
    "CUR_ANGLE_MEAS": 0x44,
    "PI_RESULTS": 0x45,
    # ADC
    "ADC_VSUPPLY_TEMP": 0x58,
    "ADC_I": 0x59,
    "OTW_OV_VTH": 0x5A,
    # Motor driver / wave table
    "MSLUT0": 0x60,
    "MSLUT1": 0x61,
    "MSLUT2": 0x62,
    "MSLUT3": 0x63,
    "MSLUT4": 0x64,
    "MSLUT5": 0x65,
    "MSLUT6": 0x66,
    "MSLUT7": 0x67,
    "MSLUTSEL": 0x68,
    "MSLUTSTART": 0x69,
    "MSCNT": 0x6A,
    "MSCURACT": 0x6B,
    "CHOPCONF": 0x6C,
    "COOLCONF": 0x6D,
    "DRV_STATUS": 0x6F,
    "PWMCONF": 0x70,
}

ReadRegisters = [
    "GCONF",
    "GSTAT",
    "IOIN",
    "DRV_CONF",
    "PLL",
    "IHOLD_IRUN",
    "TPOWERDOWN",
    "TSTEP",
    "TPWMTHRS",
    "TCOOLTHRS",
    "THIGH",
    "ADC_VSUPPLY_TEMP",
    "ADC_I",
    "CURRENT_PI_REG",
    "ANGLE_PI_REG",
    "CUR_ANGLE_LIMIT",
    "ANGLE_LOWER_LIMIT",
    "CUR_ANGLE_MEAS",
    "PI_RESULTS",
    "OTW_OV_VTH",
    "MSCNT",
    "MSCURACT",
    "CHOPCONF",
    "COOLCONF",
    "DRV_STATUS",
    "PWMCONF",
]


######################################################################
# Register field maps
######################################################################

Fields = {}

# GCONF (0x00) - p.137
# Bit 1 is "en_stealthchop" in the datasheet; aliased to "en_pwm_mode"
# for tmc.TMCStealthchopHelper compatibility.
Fields["GCONF"] = {
    "fast_standstill": 0x01 << 0,
    "en_pwm_mode": 0x01 << 1,
    "multistep_filt": 0x01 << 2,
    "shaft": 0x01 << 3,
    "small_hysteresis": 0x01 << 4,
    "stop_enable": 0x01 << 5,
    "direct_mode": 0x01 << 6,
    "length_steppulse": 0x0F << 8,
    "ov_nn": 0x01 << 12,
    "step_dir": 0x01 << 31,
}

# GSTAT (0x01) - p.139, write-1-to-clear
Fields["GSTAT"] = {
    "reset": 0x01 << 0,
    "drv_err": 0x01 << 1,
    "uv_cp": 0x01 << 2,
    "register_reset": 0x01 << 3,
    "vm_uvlo": 0x01 << 4,
    "vccio_uv": 0x01 << 5,
}

# DO_CONF (0x02) - p.140 - DIAG output mux
Fields["DO_CONF"] = {
    "do0_error": 0x01 << 0,
    "do0_otpw": 0x01 << 1,
    "do0_stall": 0x01 << 2,
    "do0_index": 0x01 << 3,
    "do0_step": 0x01 << 4,
    "do0_dir": 0x01 << 5,
    "do0_xcomp": 0x01 << 6,
    "do0_ov": 0x01 << 7,
    "do0_udcstep": 0x01 << 8,
    "do0_ev_stop_ref": 0x01 << 9,
    "do0_ev_stop_sg": 0x01 << 10,
    "do0_ev_pos_reached": 0x01 << 11,
    "do0_ev_n_deviation": 0x01 << 12,
    "do1_error": 0x01 << 13,
    "do1_otpw": 0x01 << 14,
    "do1_stall": 0x01 << 15,
    "do1_index": 0x01 << 16,
    "do1_step": 0x01 << 17,
    "do1_dir": 0x01 << 18,
    "do1_xcomp": 0x01 << 19,
    "do1_ov": 0x01 << 20,
    "do1_udcstep": 0x01 << 21,
    "do1_ev_stop_ref": 0x01 << 22,
    "do1_ev_stop_sg": 0x01 << 23,
    "do1_ev_pos_reached": 0x01 << 24,
    "do1_ev_n_deviation": 0x01 << 25,
    "do0_npp_pp": 0x01 << 28,
    "do0_invpp": 0x01 << 29,
    "do1_npp_pp": 0x01 << 30,
    "do1_invpp": 0x01 << 31,
}

# IOIN (0x04) - p.144 - read-only inputs / silicon revision
Fields["IOIN"] = {
    "refl": 0x01 << 0,
    "refr": 0x01 << 1,
    "encb": 0x01 << 2,
    "enca": 0x01 << 3,
    "drv_enn": 0x01 << 4,
    "encn": 0x01 << 5,
    "ext_res_det": 0x01 << 13,
    "ext_clk": 0x01 << 14,
    "silicon_rv": 0x07 << 16,
}

# DRV_CONF (0x0A) - p.148 - integrated current sense (ICS) range select
Fields["DRV_CONF"] = {
    "current_range": 0x03 << 0,
    "current_range_scale": 0x03 << 2,
    "slope_control": 0x03 << 4,
}

# PLL (0x0B) - p.149 - mandatory clock setup. ext_not_int=0, clk_sys_sel=1,
# clk_fsm_ena=1, CLOCK_DIVIDER=15 -> internal 16MHz osc, 1MHz fCLK.
Fields["PLL"] = {
    "commit": 0x01 << 0,
    "ext_not_int": 0x01 << 1,
    "clk_sys_sel": 0x01 << 2,
    "clk_fsm_ena": 0x01 << 3,
    "clock_divider": 0x1F << 5,
    "clk_1m0_tmo": 0x01 << 11,
    "clk_loss": 0x01 << 13,
    "clk_is_stuck": 0x01 << 14,
}

# IHOLD_IRUN (0x10) - p.151 - 8-bit IRUN/IHOLD (TMC5262 widened from 5)
Fields["IHOLD_IRUN"] = {
    "ihold": 0xFF << 0,
    "irun": 0xFF << 8,
    "iholddelay": 0xFF << 16,
    "irundelay": 0x0F << 24,
}

Fields["TPOWERDOWN"] = {"tpowerdown": 0xFF << 0}
Fields["TSTEP"] = {"tstep": 0xFFFFF << 0}
Fields["TPWMTHRS"] = {"tpwmthrs": 0xFFFFF << 0}
Fields["TCOOLTHRS"] = {"tcoolthrs": 0xFFFFF << 0}
Fields["THIGH"] = {"thigh": 0xFFFFF << 0}
Fields["TSGP_LOW_VEL_THRS"] = {"tsgp_low_vel_thrs": 0xFFFFF << 0}
Fields["T_RCOIL_MEAS"] = {"t_rcoil_meas": 0xFFFFF << 0}

# ADC registers - p.134 (9-bit Vsupply/temperature, 12-bit coil currents)
#   T(°C) = 1.042 * v - 264.6
#   V_supply(V) = v * 0.1409
Fields["ADC_VSUPPLY_TEMP"] = {
    "adc_vsupply": 0x1FF << 0,
    "adc_temp": 0x1FF << 16,
}
Fields["ADC_I"] = {
    "adc_i_a": 0xFFF << 0,
    "adc_i_b": 0xFFF << 16,
}
# StealthChop+ PI regulator config and state (datasheet pp 193-198).
# Defaults match the chip's reset values.
Fields["CURRENT_PI_REG"] = {
    "cur_p": 0xFFF << 0,
    "cur_i": 0x3FF << 16,
}
Fields["ANGLE_PI_REG"] = {
    "angle_p": 0xFFF << 0,
    "angle_i": 0x3FF << 16,
}
Fields["CUR_ANGLE_LIMIT"] = {
    "angle_pi_limit": 0x3FF << 0,
    "angle_pi_int_pos_clip": 0x01 << 12,
    "angle_pi_int_neg_clip": 0x01 << 13,
    "angle_pi_pos_clip": 0x01 << 14,
    "angle_pi_neg_clip": 0x01 << 15,
    "cur_pi_limit": 0xFFF << 16,
    "cur_pi_int_pos_clip": 0x01 << 28,
    "cur_pi_int_neg_clip": 0x01 << 29,
    "cur_pi_pos_clip": 0x01 << 30,
    "cur_pi_neg_clip": 0x01 << 31,
}
Fields["ANGLE_LOWER_LIMIT"] = {
    "angle_lower_i_limit": 0x3FF << 0,
    "angle_error": 0x3FF << 16,
}
# CUR_ANGLE_MEAS (0x44) - p.197 - StealthChop+ readbacks. Both read 0
# in SpreadCycle mode.
Fields["CUR_ANGLE_MEAS"] = {
    "ampl_meas": 0xFFF << 0,
    "angle_meas": 0x3FF << 16,
}
Fields["PI_RESULTS"] = {
    "pwm_calc": 0x1FFF << 0,
    "angle_corr_calc": 0x3FF << 16,
}
Fields["OTW_OV_VTH"] = {
    "overvoltage_vth": 0x1FF << 0,
    "overtempprewarning_vth": 0x1FF << 16,
}

# Wave table - format identical to TMC2240/TMC5160
Fields["MSLUT0"] = {"mslut0": 0xFFFFFFFF}
Fields["MSLUT1"] = {"mslut1": 0xFFFFFFFF}
Fields["MSLUT2"] = {"mslut2": 0xFFFFFFFF}
Fields["MSLUT3"] = {"mslut3": 0xFFFFFFFF}
Fields["MSLUT4"] = {"mslut4": 0xFFFFFFFF}
Fields["MSLUT5"] = {"mslut5": 0xFFFFFFFF}
Fields["MSLUT6"] = {"mslut6": 0xFFFFFFFF}
Fields["MSLUT7"] = {"mslut7": 0xFFFFFFFF}
Fields["MSLUTSEL"] = {
    "x3": 0xFF << 24,
    "x2": 0xFF << 16,
    "x1": 0xFF << 8,
    "w3": 0x03 << 6,
    "w2": 0x03 << 4,
    "w1": 0x03 << 2,
    "w0": 0x03 << 0,
}
Fields["MSLUTSTART"] = {
    "start_sin": 0xFF << 0,
    "start_sin90": 0xFF << 16,
    "offset_sin90": 0xFF << 24,
}

Fields["MSCNT"] = {"mscnt": 0x3FF << 0}

# MSCURACT - coil order is *swapped* relative to TMC2240/TMC5160:
# TMC5262 puts CUR_B at [8:0] and CUR_A at [24:16].
Fields["MSCURACT"] = {"cur_b": 0x1FF << 0, "cur_a": 0x1FF << 16}

# CHOPCONF (0x6C) - p.136 - layout differs from TMC2240:
#   * tbl is contiguous bits [16:15]
#   * hend is contiguous bits [10:7]
#   * vhighfs / vhighchm / diss2g / diss2vs are gone
Fields["CHOPCONF"] = {
    "toff": 0x0F << 0,
    "hstrt": 0x07 << 4,
    "hend": 0x0F << 7,
    "fd3": 0x01 << 11,
    "disfdcc": 0x01 << 12,
    "chm": 0x01 << 14,
    "tbl": 0x03 << 15,
    "tpfd": 0x0F << 20,
    "mres": 0x0F << 24,
    "intpol": 0x01 << 28,
    "dedge": 0x01 << 29,
}

# COOLCONF (0x6D) - p.136 - sedn widened to 3 bits, +thigh_sg_off
Fields["COOLCONF"] = {
    "semin": 0x0F << 0,
    "seup": 0x03 << 5,
    "semax": 0x0F << 8,
    "sedn": 0x07 << 12,
    "seimin": 0x01 << 15,
    "sgt": 0x7F << 16,
    "thigh_sg_off": 0x01 << 23,
    "sfilt": 0x01 << 24,
}

# DRV_STATUS (0x6F) - p.136 - 10-bit sg_result, 8-bit cs_actual,
# +ov / +seq_stopped, no fsactive
Fields["DRV_STATUS"] = {
    "sg_result": 0x3FF << 0,
    "seq_stopped": 0x01 << 10,
    "ov": 0x01 << 11,
    "s2vsa": 0x01 << 12,
    "s2vsb": 0x01 << 13,
    "stealth": 0x01 << 14,
    "cs_actual": 0xFF << 16,
    "stallguard": 0x01 << 24,
    "ot": 0x01 << 25,
    "otpw": 0x01 << 26,
    "s2ga": 0x01 << 27,
    "s2gb": 0x01 << 28,
    "ola": 0x01 << 29,
    "olb": 0x01 << 30,
    "stst": 0x01 << 31,
}

# PWMCONF (0x70) - p.136 - simplified vs TMC2240
Fields["PWMCONF"] = {
    "pwm_freq": 0x0F << 0,
    "freewheel": 0x03 << 4,
    "ol_thrsh": 0x03 << 6,
    "sd_on_meas_lo": 0x0F << 12,
    "sd_on_meas_hi": 0x0F << 16,
}


SignedFields = [
    "cur_a",
    "cur_b",
    "sgt",
    "offset_sin90",
    "adc_i_a",
    "adc_i_b",
    "angle_error",
    "angle_corr_calc",
]


FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update(
    {
        "ov": lambda v: "1(Overvoltage!)" if v else "",
        "s2vsa": lambda v: "1(ShortToSupply_A!)" if v else "",
        "s2vsb": lambda v: "1(ShortToSupply_B!)" if v else "",
        "vm_uvlo": lambda v: "1(VMUndervoltage!)" if v else "",
        "vccio_uv": lambda v: "1(VCCIOUndervoltage!)" if v else "",
        "register_reset": lambda v: "1(RegisterReset!)" if v else "",
        "ext_clk": lambda v: "1(ExternalClock)" if v else "0(Internal16MHz)",
        "ext_res_det": lambda v: "1" if v else "0(MissingRREF!)",
        "drv_enn": lambda v: "1(Disabled)" if v else "0(Enabled)",
        "silicon_rv": lambda v: "%d" % v,
        "clk_loss": lambda v: "1(ClockLoss!)" if v else "",
        "clk_is_stuck": lambda v: "1(ClockStuck!)" if v else "",
        "clk_1m0_tmo": lambda v: "1(ClockTimeout!)" if v else "",
        "adc_temp": (lambda v: "0x%03x(%.1fC)" % (v, _adc_to_celsius(v))),
        "adc_vsupply": (lambda v: "0x%03x(%.3fV)" % (v, _adc_to_volts(v))),
        # ADC_I_*/AMPL_MEAS read 0 in SpreadCycle (datasheet p.75)
        "adc_i_a": (lambda v: "%d(SpreadCycle?)" % v if v == 0 else "%d" % v),
        "adc_i_b": (lambda v: "%d(SpreadCycle?)" % v if v == 0 else "%d" % v),
        "ampl_meas": (lambda v: "%d(SpreadCycle?)" % v if v == 0 else "%d" % v),
        "angle_meas": (lambda v: "%d(%.1fdeg)" % (v, v * 360.0 / 1024.0)),
        "overvoltage_vth": (lambda v: "0x%03x(%.3fV)" % (v, _adc_to_volts(v))),
        "overtempprewarning_vth": (
            lambda v: "0x%03x(%.1fC)" % (v, _adc_to_celsius(v))
        ),
    }
)


######################################################################
# TMC5262 current helper (Integrated Current Sense, no GLOBALSCALER)
######################################################################
# Datasheet Table 21 (p.74):
#   I_FS_peak = 18 * (CR+1) * (CRS+1) / (4 * RREF_kohm)         [A peak]
#   I_FS_RMS  = I_FS_peak / sqrt(2)                              [A RMS]
#   I_RMS(IRUN) = I_FS_RMS * IRUN / 250
# CRS != 3 is only meaningful when CR == 0 per datasheet.

DEFAULT_RREF = 12000.0
KIFS_BASE = 18.0  # peak full-scale * Rref_kohm at CR=0, CRS=0


class TMC5262CurrentHelper(tmc.BaseTMCCurrentHelper):
    def __init__(self, config, mcu_tmc):
        self.Rref = config.getfloat(
            "rref", DEFAULT_RREF, minval=10000.0, maxval=14000.0
        )
        max_current = self._ifs_rms_for(3, 3)
        super().__init__(config, mcu_tmc, max_current, has_sense_resistor=False)

        cr, crs = self._calc_ranges(self.req_run_current)
        self.current_range = config.getint(
            "current_range", cr, minval=0, maxval=3
        )
        self.current_range_scale = config.getint(
            "current_range_scale", crs, minval=0, maxval=3
        )
        if self.current_range != 0 and self.current_range_scale != 3:
            raise config.error(
                "tmc5262 %s: current_range_scale<3 is only valid with "
                "current_range=0 per datasheet" % (self.name,)
            )
        self.fields.set_field("current_range", self.current_range)
        self.fields.set_field("current_range_scale", self.current_range_scale)

        irun, ihold = self._calc_current(
            self.req_run_current, self.req_hold_current
        )
        self.fields.set_field("irun", irun)
        self.fields.set_field("ihold", ihold)

    def _ifs_rms_for(self, cr, crs):
        i_peak = KIFS_BASE * (cr + 1) * (crs + 1) / (4.0 * self.Rref / 1000.0)
        return i_peak / math.sqrt(2.0)

    def _ifs_rms(self):
        cr = self.fields.get_field("current_range")
        crs = self.fields.get_field("current_range_scale")
        return self._ifs_rms_for(cr, crs)

    def _calc_ranges(self, current):
        # Smallest CR that fits at CRS=3; fall back to CRS<3 only at CR=0.
        for cr in range(4):
            if current <= self._ifs_rms_for(cr, 3):
                if cr == 0:
                    for crs in range(4):
                        if current <= self._ifs_rms_for(0, crs):
                            return 0, crs
                return cr, 3
        return 3, 3

    def _calc_current(self, run_current, hold_current):
        ifs = self._ifs_rms()
        # IRUN 251..255 are clipped to 250 internally.
        irun = max(1, min(250, int(round(run_current / ifs * 250.0))))
        if run_current > 0.0:
            ihold = int(round(hold_current / run_current * irun))
        else:
            ihold = 0
        ihold = max(0, min(irun, ihold))
        return irun, ihold

    def _calc_current_from_field(self, field_name):
        return self._ifs_rms() * self.fields.get_field(field_name) / 250.0

    def get_current(self):
        return (
            self._calc_current_from_field("irun"),
            self._calc_current_from_field("ihold"),
            self.req_hold_current,
            self._ifs_rms(),
            self.req_home_current,
        )

    def apply_current(self, print_time):
        irun, ihold = self._calc_current(
            self.actual_current, self.req_hold_current
        )
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


######################################################################
# TMC5262 main object
######################################################################


class TMC5262:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        self.mcu_tmc = tmc2130.MCU_TMC_SPI(
            config, Registers, self.fields, TMC_FREQUENCY
        )
        # tmc.TMCErrorCheck reads this via getattr to use the chip-specific
        # 9-bit ADC formula instead of the default TMC2240 one.
        self.mcu_tmc.temp_from_adc = _adc_to_celsius
        # PLL must come up before any other register write the chip is
        # meant to react to - register before TMCCommandHelper.
        self.printer.register_event_handler(
            "klippy:connect", self._handle_pll_init
        )
        # Sensorless homing: TMC5262 routes stall through DO_CONF.do0_stall
        # / do1_stall (datasheet pg 140), unlike GCONF.diag*_stall on older
        # chips. Prime do0_invpp / do1_invpp = 1 (chip-default active-low)
        # so the homing handler's stall-bit writes don't clear them.
        self.fields.set_field("do0_invpp", 1)
        self.fields.set_field("do1_invpp", 1)
        virtual_pin_helper = tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        if config.get("diag0_pin", None) is not None:
            virtual_pin_helper.diag_pin = config.get("diag0_pin")
            virtual_pin_helper.diag_pin_field = "do0_stall"
        elif config.get("diag1_pin", None) is not None:
            virtual_pin_helper.diag_pin = config.get("diag1_pin")
            virtual_pin_helper.diag_pin_field = "do1_stall"
        # Register commands
        current_helper = TMC5262CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Microstep wave table
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        self.fields.set_config_field(config, "offset_sin90", 0)
        # StealthChop / CoolStep velocity thresholds
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        # PLL steady-state. _handle_pll_init runs the boot sequence; this
        # makes cmd_INIT_TMC re-issue the same value and DUMP_TMC report it.
        self.fields.set_field("clock_divider", 15)
        self.fields.set_field("clk_sys_sel", 1)
        self.fields.set_field("clk_fsm_ena", 1)
        # GCONF: step_dir is required for use as an external step/dir
        # driver. Bits not set here are written as 0 by INIT_TMC, including
        # small_hysteresis (chip default is 1, ends up 0 - matches TMC2240).
        set_config_field(config, "step_dir", True)
        set_config_field(config, "multistep_filt", True)
        # CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "tpfd", 4)
        # COOLCONF
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        # IHOLDIRUN
        set_config_field(config, "iholddelay", 7)
        set_config_field(config, "irundelay", 4)
        # PWMCONF. SD_ON_MEAS_LO/HI control when the ICS samples coil
        # currents within the chopper cycle - datasheet Table 8 (p.75)
        # specifies values per PWM_FREQ. Defaults below match PWM_FREQ=0
        # (19.5 kHz); change them if PWM_FREQ changes or ICS goes silent.
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "sd_on_meas_lo", 14)
        set_config_field(config, "sd_on_meas_hi", 15)
        # TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)
        # DRV_CONF
        set_config_field(config, "slope_control", 3)
        # StealthChop+ PI regulators - chip reset values, exposed for tuning.
        set_config_field(config, "cur_p", 64)
        set_config_field(config, "cur_i", 10)
        set_config_field(config, "angle_p", 50)
        set_config_field(config, "angle_i", 20)
        set_config_field(config, "cur_pi_limit", 0xFFF)
        set_config_field(config, "angle_pi_limit", 256)
        set_config_field(config, "angle_lower_i_limit", 256)

    def _build_pll_value(self, commit, clk_fsm_ena, clear_flags=False):
        # Built from raw bits so __init__'s steady-state cache stays intact.
        val = (
            (1 if commit else 0) << 0
            | (0 << 1)  # ext_not_int = 0 (internal oscillator)
            | (1 << 2)  # clk_sys_sel = 1 (use PLL)
            | ((1 if clk_fsm_ena else 0) << 3)
            | (15 << 5)  # CLOCK_DIVIDER = 15 -> 16MHz/16 = 1MHz fCLK
        )
        if clear_flags:
            # write-1-to-clear clk_1m0_tmo / clk_loss / clk_is_stuck
            val |= (1 << 11) | (1 << 13) | (1 << 14)
        return val

    def _handle_pll_init(self):
        if self.mcu_tmc.mcu.non_critical_disconnected:
            return
        try:
            # Stage 1: program PLL with commit=1, FSM still off. Datasheet
            # p.149: chip enters reset state (except register space).
            self.mcu_tmc.set_register(
                "PLL", self._build_pll_value(commit=1, clk_fsm_ena=0)
            )
            # Wait for commit to self-clear (PLL locked). Cap at 20ms.
            reactor = self.printer.get_reactor()
            deadline = reactor.monotonic() + 0.020
            while reactor.monotonic() < deadline:
                reactor.pause(reactor.monotonic() + 0.001)
                pll = self.mcu_tmc.get_register("PLL")
                if not (pll & 0x01):
                    break
            # Stage 2: release reset, enable FSM, W1C clock-fault flags.
            # Written twice with a settle - the flags re-assert during the
            # PLL lock transient and a single write fails to clear them.
            stage2 = self._build_pll_value(
                commit=0, clk_fsm_ena=1, clear_flags=True
            )
            self.mcu_tmc.set_register("PLL", stage2)
            reactor.pause(reactor.monotonic() + 0.005)
            self.mcu_tmc.set_register("PLL", stage2)
            # Stage 1 puts the chip in reset, which sets GSTAT.reset /
            # register_reset / uv_cp. Clear so TMCErrorCheck doesn't trip.
            self.mcu_tmc.set_register("GSTAT", 0x3F)
        except self.printer.command_error as e:
            logging.info("TMC5262 %s PLL init failed: %s", self.name, str(e))


def load_config_prefix(config):
    return TMC5262(config)

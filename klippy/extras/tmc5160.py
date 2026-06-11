# TMC5160/TMC2160 configuration
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from . import tmc, tmc2130

TMC_FREQUENCY = 12000000.0

Registers = {
    "GCONF": 0x00,
    "GSTAT": 0x01,
    "IFCNT": 0x02,
    "SLAVECONF": 0x03,
    "IOIN": 0x04,
    "X_COMPARE": 0x05,
    "OTP_READ": 0x07,
    "FACTORY_CONF": 0x08,
    "SHORT_CONF": 0x09,
    "DRV_CONF": 0x0A,
    "GLOBALSCALER": 0x0B,
    "OFFSET_READ": 0x0C,
    "IHOLD_IRUN": 0x10,
    "TPOWERDOWN": 0x11,
    "TSTEP": 0x12,
    "TPWMTHRS": 0x13,
    "TCOOLTHRS": 0x14,
    "THIGH": 0x15,
    "RAMPMODE": 0x20,
    "XACTUAL": 0x21,
    "VACTUAL": 0x22,
    "VSTART": 0x23,
    "A1": 0x24,
    "V1": 0x25,
    "AMAX": 0x26,
    "VMAX": 0x27,
    "DMAX": 0x28,
    "D1": 0x2A,
    "VSTOP": 0x2B,
    "TZEROWAIT": 0x2C,
    "XTARGET": 0x2D,
    "VDCMIN": 0x33,
    "SW_MODE": 0x34,
    "RAMP_STAT": 0x35,
    "XLATCH": 0x36,
    "ENCMODE": 0x38,
    "X_ENC": 0x39,
    "ENC_CONST": 0x3A,
    "ENC_STATUS": 0x3B,
    "ENC_LATCH": 0x3C,
    "ENC_DEVIATION": 0x3D,
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
    "DCCTRL": 0x6E,
    "DRV_STATUS": 0x6F,
    "PWMCONF": 0x70,
    "PWM_SCALE": 0x71,
    "PWM_AUTO": 0x72,
    "LOST_STEPS": 0x73,
}

ReadRegisters = [
    "GCONF",
    "CHOPCONF",
    "GSTAT",
    "DRV_STATUS",
    "FACTORY_CONF",
    "IOIN",
    "LOST_STEPS",
    "MSCNT",
    "MSCURACT",
    "OTP_READ",
    "PWM_SCALE",
    "PWM_AUTO",
    "TSTEP",
]

Fields = {}
Fields["COOLCONF"] = {
    "semin": 0x0F << 0,
    "seup": 0x03 << 5,
    "semax": 0x0F << 8,
    "sedn": 0x03 << 13,
    "seimin": 0x01 << 15,
    "sgt": 0x7F << 16,
    "sfilt": 0x01 << 24,
}
Fields["CHOPCONF"] = {
    "toff": 0x0F << 0,
    "hstrt": 0x07 << 4,
    "hend": 0x0F << 7,
    "fd3": 0x01 << 11,
    "disfdcc": 0x01 << 12,
    "chm": 0x01 << 14,
    "tbl": 0x03 << 15,
    "vhighfs": 0x01 << 18,
    "vhighchm": 0x01 << 19,
    "tpfd": 0x0F << 20,  # midrange resonances
    "mres": 0x0F << 24,
    "intpol": 0x01 << 28,
    "dedge": 0x01 << 29,
    "diss2g": 0x01 << 30,
    "diss2vs": 0x01 << 31,
}
Fields["DRV_CONF"] = {
    "bbmtime": 0x1F << 0,
    "bbmclks": 0x0F << 8,
    "otselect": 0x03 << 16,
    "drvstrength": 0x03 << 18,
    "filt_isense": 0x03 << 20,
}
Fields["SHORT_CONF"] = {
    "s2vs_level": 0x0F << 0,
    "s2g_level": 0x0F << 8,
    "short_filter": 0x03 << 16,
    "shortdelay": 0x01 << 18,
}
Fields["DRV_STATUS"] = {
    "sg_result": 0x3FF << 0,
    "s2vsa": 0x01 << 12,
    "s2vsb": 0x01 << 13,
    "stealth": 0x01 << 14,
    "fsactive": 0x01 << 15,
    "cs_actual": 0x1F << 16,
    "stallguard": 0x01 << 24,
    "ot": 0x01 << 25,
    "otpw": 0x01 << 26,
    "s2ga": 0x01 << 27,
    "s2gb": 0x01 << 28,
    "ola": 0x01 << 29,
    "olb": 0x01 << 30,
    "stst": 0x01 << 31,
}
Fields["FACTORY_CONF"] = {"factory_conf": 0x1F << 0}
Fields["FACTORY_CONF"] = {"factory_conf": 0x1F << 0}
Fields["GCONF"] = {
    "recalibrate": 0x01 << 0,
    "faststandstill": 0x01 << 1,
    "en_pwm_mode": 0x01 << 2,
    "multistep_filt": 0x01 << 3,
    "shaft": 0x01 << 4,
    "diag0_error": 0x01 << 5,
    "diag0_otpw": 0x01 << 6,
    "diag0_stall": 0x01 << 7,
    "diag1_stall": 0x01 << 8,
    "diag1_index": 0x01 << 9,
    "diag1_onstate": 0x01 << 10,
    "diag1_steps_skipped": 0x01 << 11,
    "diag0_int_pushpull": 0x01 << 12,
    "diag1_poscomp_pushpull": 0x01 << 13,
    "small_hysteresis": 0x01 << 14,
    "stop_enable": 0x01 << 15,
    "direct_mode": 0x01 << 16,
    "test_mode": 0x01 << 17,
}
Fields["GSTAT"] = {"reset": 0x01 << 0, "drv_err": 0x01 << 1, "uv_cp": 0x01 << 2}
Fields["GLOBALSCALER"] = {"globalscaler": 0xFF << 0}
Fields["IHOLD_IRUN"] = {
    "ihold": 0x1F << 0,
    "irun": 0x1F << 8,
    "iholddelay": 0x0F << 16,
}
Fields["IOIN"] = {
    "refl_step": 0x01 << 0,
    "refr_dir": 0x01 << 1,
    "encb_dcen_cfg4": 0x01 << 2,
    "enca_dcin_cfg5": 0x01 << 3,
    "drv_enn": 0x01 << 4,
    "enc_n_dco_cfg6": 0x01 << 5,
    "sd_mode": 0x01 << 6,
    "swcomp_in": 0x01 << 7,
    "version": 0xFF << 24,
}
Fields["LOST_STEPS"] = {"lost_steps": 0xFFFFF << 0}
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
}
Fields["MSCNT"] = {"mscnt": 0x3FF << 0}
Fields["MSCURACT"] = {"cur_a": 0x1FF << 0, "cur_b": 0x1FF << 16}
Fields["LOST_STEPS"] = {"lost_steps": 0xFFFFF << 0}
Fields["MSCNT"] = {"mscnt": 0x3FF << 0}
Fields["MSCURACT"] = {"cur_a": 0x1FF << 0, "cur_b": 0x1FF << 16}
Fields["OTP_READ"] = {
    "otp_fclktrim": 0x1F << 0,
    "otp_s2_level": 0x01 << 5,
    "otp_bbm": 0x01 << 6,
    "otp_tbl": 0x01 << 7,
}
Fields["PWM_AUTO"] = {"pwm_ofs_auto": 0xFF << 0, "pwm_grad_auto": 0xFF << 16}
Fields["PWMCONF"] = {
    "pwm_ofs": 0xFF << 0,
    "pwm_grad": 0xFF << 8,
    "pwm_freq": 0x03 << 16,
    "pwm_autoscale": 0x01 << 18,
    "pwm_autograd": 0x01 << 19,
    "freewheel": 0x03 << 20,
    "pwm_reg": 0x0F << 24,
    "pwm_lim": 0x0F << 28,
}
Fields["PWM_SCALE"] = {
    "pwm_scale_sum": 0xFF << 0,
    "pwm_scale_auto": 0x1FF << 16,
}
Fields["TPOWERDOWN"] = {"tpowerdown": 0xFF << 0}
Fields["TPWMTHRS"] = {"tpwmthrs": 0xFFFFF << 0}
Fields["TCOOLTHRS"] = {"tcoolthrs": 0xFFFFF << 0}
Fields["TSTEP"] = {"tstep": 0xFFFFF << 0}
Fields["THIGH"] = {"thigh": 0xFFFFF << 0}

SignedFields = ["cur_a", "cur_b", "sgt", "xactual", "vactual", "pwm_scale_auto"]

FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update(
    {
        "s2vsa": (lambda v: "1(ShortToSupply_A!)" if v else ""),
        "s2vsb": (lambda v: "1(ShortToSupply_B!)" if v else ""),
    }
)


######################################################################
# TMC stepper current config helper
######################################################################

VREF = 0.325
MAX_CURRENT = 10.600  # Maximum dependent on board, but 10 is safe sanity check

GLOBALSCALER_ERROR = (
    "[tmc5160 %s]\n"
    "GLOBALSCALER(%d) calculation out of bounds.\n"
    "The target current can't be achieved with the given "
    "CS(%d) value. Please adjust your configuration.\n"
    "Please refer to the tmc5160.xlxs chopper tuning spreadsheet.\n"
    "A value of %d may be a reasonable starting point.\n"
)


class TMC5160CurrentHelper(tmc.BaseTMCCurrentHelper):
    def __init__(self, config, mcu_tmc, direct_mode=False):
        super().__init__(config, mcu_tmc, MAX_CURRENT)
        self._direct_mode = direct_mode
        self.cs = config.getint("driver_CS", None, minval=0, maxval=31)
        gscaler, irun, ihold = self._calc_current(
            self.req_run_current, self.req_hold_current
        )
        self.fields.set_field("globalscaler", gscaler)
        self.fields.set_field("ihold", irun if self._direct_mode else ihold)
        self.fields.set_field("irun", irun)

    def _calc_globalscaler(self, current):
        cs = 31 if self.cs is None else self.cs
        globalscaler = math.floor(
            (current * 32 * 256 * self.sense_resistor * math.sqrt(2.0))
            / ((cs + 1) * VREF)
        )
        if globalscaler == 256:
            return 0
        if self.cs is None and globalscaler < 32:
            return 32
        if 1 <= globalscaler <= 31 or globalscaler > 256:
            Ipeak = current * math.sqrt(2)
            Rsens = self.sense_resistor
            cs_calculated = math.floor(Rsens * 32 * Ipeak / 0.32) - 1
            self.printer.invoke_shutdown(
                GLOBALSCALER_ERROR
                % (
                    self.name,
                    globalscaler,
                    cs,
                    cs_calculated,
                )
            )
        return globalscaler

    def _calc_current_bits(self, current, globalscaler):
        if not globalscaler:
            globalscaler = 256
        cs = int(
            (current * 256.0 * 32.0 * math.sqrt(2.0) * self.sense_resistor)
            / (globalscaler * VREF)
            - 1.0
            + 0.5
        )
        return max(0, min(31, cs))

    def _calc_current(self, run_current, hold_current):
        gscaler = self._calc_globalscaler(run_current)
        irun = (
            self._calc_current_bits(run_current, gscaler)
            if self.cs is None
            else self.cs
        )
        ihold = math.floor(min((hold_current / run_current) * irun, irun))
        return gscaler, irun, ihold

    def _calc_current_from_field(self, field_name):
        globalscaler = self.fields.get_field("globalscaler")
        if not globalscaler:
            globalscaler = 256
        bits = self.fields.get_field(field_name)
        return (
            globalscaler
            * (bits + 1)
            * VREF
            / (256.0 * 32.0 * math.sqrt(2.0) * self.sense_resistor)
        )

    def get_current(self):
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        return (
            run_current,
            hold_current,
            self.req_hold_current,
            MAX_CURRENT,
            self.req_home_current,
        )

    def apply_current(self, print_time):
        gscaler, irun, ihold = self._calc_current(
            self.actual_current, self.req_hold_current
        )
        val = self.fields.set_field("globalscaler", gscaler)
        self.mcu_tmc.set_register("GLOBALSCALER", val, print_time)
        self.fields.set_field("ihold", irun if self._direct_mode else ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


######################################################################
# TMC5160 printer object
######################################################################


def _enable_direct_mode(config, stepper_section, fields):
    # direct_mode is NOT set in the field cache here — it must be
    # written AFTER CHOPCONF (toff>0) is on the chip, or the bootstrap
    # charge pump starves. _xdirect_preload handles the sequencing.
    sct = config.getfloat("stealthchop_threshold", 0.0, minval=0.0)
    if sct > 0.0:
        raise config.error(
            "phase_stepping=True is incompatible with stealthchop_threshold "
            "(StealthChop is bypassed in direct mode). Remove "
            "stealthchop_threshold from [%s] or disable phase_stepping."
            % config.get_name()
        )
    mres = stepper_section.getint("microsteps", 256)
    if mres != 256:
        raise config.error(
            "phase_stepping=True requires microsteps: 256; [%s] has "
            "microsteps: %d." % (stepper_section.get_name(), mres)
        )


class TMC5160:
    def __init__(self, config):
        # Setup mcu communication
        self.printer = config.get_printer()
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        self.mcu_tmc = tmc2130.MCU_TMC_SPI(
            config, Registers, self.fields, TMC_FREQUENCY
        )
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        stepper_name = " ".join(config.get_name().split()[1:])
        if config.has_section(stepper_name):
            stepper_section = config.getsection(stepper_name)
        else:
            stepper_section = None
        self.name = stepper_name
        self._phase_stepping = False
        self._phase_bus_id = None
        self._phase_cs_pin_id = None
        self._phase_stepper_oid = None
        self._phase_axis_idx = None
        self._cached_mscnt = None
        self._phase_mode_active = False
        self._phase_state_query = None
        if stepper_section is not None and stepper_section.getboolean(
            "phase_stepping", False
        ):
            _enable_direct_mode(config, stepper_section, self.fields)
            self._phase_stepping = True
        # Register commands
        current_helper = TMC5160CurrentHelper(
            config,
            self.mcu_tmc,
            direct_mode=self._phase_stepping,
        )
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        self._echeck_helper = cmdhelper.echeck_helper
        if self._phase_stepping:
            cmdhelper.set_post_enable_callback(self.enter_phase_mode)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        #   GCONF
        set_config_field(config, "multistep_filt", True)
        #   CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "vhighfs", 0)
        set_config_field(config, "vhighchm", 0)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "diss2g", 0)
        set_config_field(config, "diss2vs", 0)
        #   COOLCONF
        set_config_field(config, "semin", 0)  # page 52
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        #   DRV_CONF
        set_config_field(config, "drvstrength", 0)
        set_config_field(config, "bbmclks", 4)
        set_config_field(config, "bbmtime", 0)
        set_config_field(config, "filt_isense", 0)
        #   SHORT_CONF, being write only we can't partially update
        # TODO: add a hook to read OTP on connect to get the defaults here
        if config.getint("driver_s2vs_level", None, 4, 15) and config.getint(
            "driver_s2g_level", None, 2, 15
        ):
            set_config_field(config, "s2vs_level", 6)
            set_config_field(config, "s2g_level", 6)
            set_config_field(config, "short_filter", 1)
            set_config_field(config, "shortdelay", 0)
        elif any(
            config.get("driver_%s" % field, None, False)
            for field in Fields["SHORT_CONF"].keys()
        ):
            raise config.error(
                "driver_s2vs_level and driver_s2g_level are required to update short_conf"
            )
        #   IHOLDIRUN
        set_config_field(config, "iholddelay", 6)
        #   PWMCONF
        set_config_field(config, "pwm_ofs", 30)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)
        #   TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)
        if self._phase_stepping:
            # In direct_mode the TMC5160 uses IHOLD for current scaling
            # (no step pulses to trigger IRUN). Extend the standstill
            # timeout to maximum so the driver doesn't power-down the
            # coils while phase stepping is active.
            self.fields.set_field("tpowerdown", 255)

    PHASE_JOG_MAX_PER_SAMPLE = 1
    PHASE_SETTLE_TIMEOUT = 0.5

    def set_phase_stepper_oid(self, oid):
        self._phase_stepper_oid = oid

    def phase_stepping_active(self):
        return self._phase_mode_active

    def _phase_mcu(self):
        return self.mcu_tmc.tmc_spi.spi.get_mcu()

    def _lookup_phase_commands(self):
        mcu_obj = self._phase_mcu()
        if self._phase_stepper_oid is None:
            raise self.printer.command_error(
                "phase_stepping: stepper oid not registered for %s "
                "(motion_toolhead init_planner did not run?)" % (self.name,)
            )
        enable_spi = mcu_obj.lookup_command("kalico_phase_stepping_enable_spi")
        disable_spi = mcu_obj.lookup_command(
            "kalico_phase_stepping_disable_spi"
        )
        set_axis_mode = mcu_obj.lookup_command(
            "kalico_set_axis_mode axis_idx=%c mode=%c"
        )
        jog = mcu_obj.lookup_command(
            "kalico_phase_jog_to oid=%c target_phase=%hu"
            " max_microsteps_per_sample=%hu"
        )
        align = mcu_obj.lookup_command(
            "kalico_phase_align_to oid=%c target_phase=%hu"
        )
        if self._phase_state_query is None:
            self._phase_state_query = mcu_obj.lookup_query_command(
                "kalico_get_phase_state oid=%c",
                "kalico_phase_state oid=%c axis_idx=%c mode=%c phase=%hu"
                " settled=%c",
                oid=self._phase_stepper_oid,
            )
        return enable_spi, disable_spi, set_axis_mode, jog, align

    def _query_phase_state(self):
        return self._phase_state_query.send([self._phase_stepper_oid])

    def enter_phase_mode(self):
        enable_spi, disable_spi, set_axis_mode, _jog, align = (
            self._lookup_phase_commands()
        )
        # Suppress ISR XDIRECT writes during our foreground SPI traffic
        # (the disable command is idempotent; harmless if already disabled).
        disable_spi.send([])
        # Write CHOPCONF (toff>0) first, then set GCONF.direct_mode=1.
        # direct_mode is deliberately NOT in the field cache (removed from
        # _enable_direct_mode) so _init_registers doesn't write it while
        # the chip still has toff=0 from the virtual-enable disable phase.
        # The bootstrap charge pump depends on the chopper switching —
        # direct_mode with toff=0 drains the bootstrap caps and triggers
        # uv_cp after a few moves.
        chopconf_val = self.fields.registers.get("CHOPCONF")
        if chopconf_val is not None:
            self.mcu_tmc.set_register("CHOPCONF", chopconf_val)
        gconf_val = self.fields.registers.get("GCONF", 0)
        gconf_val |= 1 << 16  # direct_mode
        gconf_val &= ~(1 << 2)  # SpreadCycle (clear en_pwm_mode)
        self.mcu_tmc.set_register("GCONF", gconf_val)
        self.fields.registers["GCONF"] = gconf_val
        mscnt = self.mcu_tmc.get_register("MSCNT") & 0x3FF
        self._cached_mscnt = mscnt
        angle = mscnt * 2.0 * math.pi / 1024.0
        coil_a = int(round(248.0 * math.cos(angle)))
        coil_b = int(round(248.0 * math.sin(angle)))
        xdirect_val = ((coil_b & 0xFFFF) << 16) | (coil_a & 0xFFFF)
        self.mcu_tmc.set_register("XTARGET", xdirect_val)
        logging.info(
            "TMC5160 XDIRECT preload: mscnt=%d coil_a=%d coil_b=%d raw=0x%08x",
            mscnt,
            coil_a,
            coil_b,
            xdirect_val,
        )
        state = self._query_phase_state()
        self._phase_axis_idx = state["axis_idx"]
        align.send([self._phase_stepper_oid, mscnt])
        enable_spi.send([])
        set_axis_mode.send([self._phase_axis_idx, 1])
        # Stop the periodic DRV_STATUS/GSTAT checks while the ISR is
        # writing XDIRECT. The ISR's inline SPI manipulates the SPI
        # peripheral registers directly — foreground register reads
        # during ISR activity return corrupted data (e.g., GSTAT reads
        # as 0x010a0023 instead of a valid 3-bit value), triggering
        # false drv_err/uv_cp shutdowns. DMA-based SPI (Phase 2) will
        # fix the arbitration; until then, suppress the checks.
        self._echeck_helper.stop_checks()
        self._phase_mode_active = True
        logging.info("TMC5160 %s: phase mode entered", self.name)

    def exit_phase_mode(self):
        if not self._phase_mode_active:
            raise self.printer.command_error(
                "exit_phase_mode called but %s is not in phase mode"
                % (self.name,)
            )
        _enable_spi, disable_spi, set_axis_mode, jog, _align = (
            self._lookup_phase_commands()
        )
        state = self._query_phase_state()
        if state["mode"] != 1:
            raise self.printer.command_error(
                "phase mode bookkeeping desync on %s: host=phase mcu=%d"
                % (self.name, state["mode"])
            )
        jog.send(
            [
                self._phase_stepper_oid,
                self._cached_mscnt,
                self.PHASE_JOG_MAX_PER_SAMPLE,
            ]
        )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + self.PHASE_SETTLE_TIMEOUT
        while True:
            state = self._query_phase_state()
            if state["settled"] and state["phase"] == self._cached_mscnt:
                break
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "phase handover jog did not settle on %s "
                    "(phase=%d target=%d)"
                    % (self.name, state["phase"], self._cached_mscnt)
                )
            reactor.pause(reactor.monotonic() + 0.005)
        disable_spi.send([])
        gconf_val = self.fields.registers.get("GCONF", 0)
        gconf_val &= ~(1 << 16)  # clear direct_mode
        self.mcu_tmc.set_register("GCONF", gconf_val)
        self.fields.registers["GCONF"] = gconf_val
        set_axis_mode.send([self._phase_axis_idx, 0])
        self._echeck_helper.start_checks()
        self._phase_mode_active = False
        logging.info(
            "TMC5160 %s: phase mode exited (pulse stepping)", self.name
        )

    def get_phase_config(self):
        if not self._phase_stepping:
            raise self.printer.config_error(
                "get_phase_config called on a TMC5160 without "
                "phase_stepping=True on the matching stepper section"
            )
        if self._phase_bus_id is None or self._phase_cs_pin_id is None:
            self._phase_bus_id, self._phase_cs_pin_id = (
                self.mcu_tmc.tmc_spi.get_bus_and_cs_ids()
            )
        return (self._phase_bus_id, self._phase_cs_pin_id)

    def get_spi_oid(self):
        return self.mcu_tmc.tmc_spi.spi.oid


def load_config_prefix(config):
    return TMC5160(config)

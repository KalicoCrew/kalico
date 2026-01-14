# Kinematic input shaper to minimize motion vibrations in XY plane
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections

from klippy import chelper

from . import shaper_defs

MODE_PRIMARY = 0
MODE_COPY = 1
MODE_MIRROR = 2

MODE_NAMES = {MODE_PRIMARY: "PRIMARY", MODE_COPY: "COPY", MODE_MIRROR: "MIRROR"}


class InputShaperParams:
    def __init__(self, axis, config):
        self.axis = axis
        self.shapers = {s.name: s.init_func for s in shaper_defs.INPUT_SHAPERS}

        # parse shaper types
        shaper_type = config.get("shaper_type", "mzv")
        shaper_type_axis = config.get("shaper_type_" + axis, shaper_type)
        self.shaper_types = [
            s.strip() for s in shaper_type_axis.split(",") if s.strip()
        ]
        if not self.shaper_types:
            self.shaper_types = [shaper_type]

        for st in self.shaper_types:
            if st not in self.shapers:
                raise config.error("Unsupported shaper type: %s" % (st,))

        # parse damping ratios
        damping_ratio = config.get(
            "damping_ratio_" + axis, str(shaper_defs.DEFAULT_DAMPING_RATIO)
        )
        self.damping_ratios = [
            float(dr.strip()) for dr in damping_ratio.split(",") if dr.strip()
        ]
        if not self.damping_ratios:
            self.damping_ratios = [shaper_defs.DEFAULT_DAMPING_RATIO]

        for dr in self.damping_ratios:
            if dr < 0.0 or dr > 1.0:
                raise config.error(
                    "Damping ratio must be between 0.0 and 1.0: %s" % (dr,)
                )

        # parse frequencies
        freq = config.get("shaper_freq_" + axis, "0.0")
        self.shaper_freqs = [
            float(f.strip()) for f in freq.split(",") if f.strip()
        ]
        if not self.shaper_freqs:
            self.shaper_freqs = [0.0]

        for f in self.shaper_freqs:
            if f < 0.0:
                raise config.error("Frequency must be non-negative: %s" % (f,))

        # use first value as default
        self.damping_ratio = self.damping_ratios[0]
        self.shaper_type = self.shaper_types[0]
        self.shaper_freq = self.shaper_freqs[0]

    def update(self, gcmd):
        axis = self.axis.upper()
        self.damping_ratio = gcmd.get_float(
            "DAMPING_RATIO_" + axis, self.damping_ratio, minval=0.0, maxval=1.0
        )
        self.shaper_freq = gcmd.get_float(
            "SHAPER_FREQ_" + axis, self.shaper_freq, minval=0.0
        )
        shaper_type = gcmd.get("SHAPER_TYPE", None)
        if shaper_type is None:
            shaper_type = gcmd.get("SHAPER_TYPE_" + axis, self.shaper_type)
        if shaper_type.lower() not in self.shapers:
            raise gcmd.error("Unsupported shaper type: %s" % (shaper_type,))
        self.shaper_type = shaper_type.lower()

    def get_shaper(self):
        if not self.shaper_freq:
            A, T = shaper_defs.get_none_shaper()
        else:
            A, T = self.shapers[self.shaper_type](
                self.shaper_freq, self.damping_ratio
            )
        return len(A), A, T

    def get_status(self):
        return collections.OrderedDict(
            [
                ("shaper_type", self.shaper_type),
                ("shaper_freq", "%.3f" % (self.shaper_freq,)),
                ("damping_ratio", "%.6f" % (self.damping_ratio,)),
            ]
        )


class AxisInputShaper:
    def __init__(self, axis, config):
        self.axis = axis
        self.params = InputShaperParams(axis, config)
        self.n, self.A, self.T = self.params.get_shaper()
        self.saved = None
        self.toolhead_idx = 0
        self.mode_idx = MODE_PRIMARY

    def get_name(self):
        return "shaper_" + self.axis

    def get_shaper(self):
        return self.n, self.A, self.T

    def update(self, gcmd):
        self.params.update(gcmd)
        self.n, self.A, self.T = self.params.get_shaper()

    def set_toolhead_mode(self, toolhead_idx, mode_idx):
        self.toolhead_idx = toolhead_idx
        self.mode_idx = mode_idx
        param_idx = self._get_param_index()
        self._update_params_for_index(param_idx)

    def _get_param_index(self):
        # for single toolhead or single parameter, always use index 0
        if len(self.params.shaper_types) == 1:
            return 0

        # for dual toolhead with 2 parameters, use toolhead index
        elif len(self.params.shaper_types) == 2:
            return self.toolhead_idx

        # for dual toolhead with 4 parameters, calculate based on toolhead and mode
        elif len(self.params.shaper_types) == 4:
            if self.mode_idx == MODE_PRIMARY:
                return self.toolhead_idx  # 0 for toolhead 0, 1 for toolhead 1
            else:  # COPY or MIRROR mode
                return self.mode_idx + 1  # 2 for COPY, 3 for MIRROR

        # Default to first parameter
        return 0

    def _update_params_for_index(self, idx):
        idx = min(idx, len(self.params.shaper_types) - 1)

        self.params.shaper_type = self.params.shaper_types[idx]
        self.params.damping_ratio = self.params.damping_ratios[
            min(idx, len(self.params.damping_ratios) - 1)
        ]
        self.params.shaper_freq = self.params.shaper_freqs[
            min(idx, len(self.params.shaper_freqs) - 1)
        ]
        self.n, self.A, self.T = self.params.get_shaper()

    def set_shaper_kinematics(self, sk):
        ffi_main, ffi_lib = chelper.get_ffi()
        success = (
            ffi_lib.input_shaper_set_shaper_params(
                sk, self.axis.encode(), self.n, self.A, self.T
            )
            == 0
        )
        if not success:
            self.disable_shaping()
            ffi_lib.input_shaper_set_shaper_params(
                sk, self.axis.encode(), self.n, self.A, self.T
            )
        return success

    def disable_shaping(self):
        if self.saved is None and self.n:
            self.saved = (self.n, self.A, self.T)
        A, T = shaper_defs.get_none_shaper()
        self.n, self.A, self.T = len(A), A, T

    def enable_shaping(self):
        if self.saved is None:
            # Input shaper was not disabled
            return
        self.n, self.A, self.T = self.saved
        self.saved = None

    def report(self, gcmd):
        info = " ".join(
            [
                "%s_%s:%s" % (key, self.axis, value)
                for (key, value) in self.params.get_status().items()
            ]
        )
        gcmd.respond_info(info)


class InputShaper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.shapers = [
            AxisInputShaper("x", config),
            AxisInputShaper("y", config),
        ]
        self.input_shaper_stepper_kinematics = []
        self.orig_stepper_kinematics = []
        self.dual_carriage = None
        self.active_toolhead = 0
        self.active_mode = MODE_PRIMARY

        # Register gcode commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "SET_INPUT_SHAPER",
            self.cmd_SET_INPUT_SHAPER,
            desc=self.cmd_SET_INPUT_SHAPER_help,
        )
        gcode.register_command(
            "GET_INPUT_SHAPER",
            self.cmd_GET_INPUT_SHAPER,
            desc=self.cmd_GET_INPUT_SHAPER_help,
        )

        self.printer.register_event_handler(
            "dual_carriage:mode_change", self.handle_dual_carriage_mode_change
        )

    def get_shapers(self):
        return self.shapers

    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")

        try:
            self.dual_carriage = self.printer.lookup_object("dual_carriage")
        except self.printer.config_error:
            self.dual_carriage = None

        # Configure initial values
        self._update_input_shaping(error=self.printer.config_error)

    def _get_input_shaper_stepper_kinematics(self, stepper):
        # Lookup stepper kinematics
        sk = stepper.get_stepper_kinematics()
        if sk in self.orig_stepper_kinematics:
            # Already processed this stepper kinematics unsuccessfully
            return None
        if sk in self.input_shaper_stepper_kinematics:
            return sk
        self.orig_stepper_kinematics.append(sk)
        ffi_main, ffi_lib = chelper.get_ffi()
        is_sk = ffi_main.gc(ffi_lib.input_shaper_alloc(), ffi_lib.free)
        stepper.set_stepper_kinematics(is_sk)
        res = ffi_lib.input_shaper_set_sk(is_sk, sk)
        if res < 0:
            stepper.set_stepper_kinematics(sk)
            return None
        self.input_shaper_stepper_kinematics.append(is_sk)
        return is_sk

    def _update_input_shaping(self, error=None):
        self.toolhead.flush_step_generation()
        ffi_main, ffi_lib = chelper.get_ffi()
        kin = self.toolhead.get_kinematics()
        failed_shapers = []
        for s in kin.get_steppers():
            if s.get_trapq() is None:
                continue
            is_sk = self._get_input_shaper_stepper_kinematics(s)
            if is_sk is None:
                continue
            old_delay = ffi_lib.input_shaper_get_step_generation_window(is_sk)
            for shaper in self.shapers:
                if shaper in failed_shapers:
                    continue
                if not shaper.set_shaper_kinematics(is_sk):
                    failed_shapers.append(shaper)
            new_delay = ffi_lib.input_shaper_get_step_generation_window(is_sk)
            if old_delay != new_delay:
                self.toolhead.note_step_generation_scan_time(
                    new_delay, old_delay
                )
        if failed_shapers:
            error = error or self.printer.command_error
            raise error(
                "Failed to configure shaper(s) %s with given parameters"
                % (", ".join([s.get_name() for s in failed_shapers]))
            )

    def disable_shaping(self):
        for shaper in self.shapers:
            shaper.disable_shaping()
        self._update_input_shaping()

    def enable_shaping(self):
        for shaper in self.shapers:
            shaper.enable_shaping()
        self._update_input_shaping()

    cmd_SET_INPUT_SHAPER_help = "Set cartesian parameters for input shaper"
    cmd_GET_INPUT_SHAPER_help = "Get current input shaper parameters"

    def handle_dual_carriage_mode_change(self, carriage_idx, mode):
        self.active_toolhead = carriage_idx
        if mode == "COPY":
            self.active_mode = MODE_COPY
        elif mode == "MIRROR":
            self.active_mode = MODE_MIRROR
        else:
            self.active_mode = MODE_PRIMARY

        for shaper in self.shapers:
            shaper.set_toolhead_mode(self.active_toolhead, self.active_mode)

        self._update_input_shaping()

    def cmd_SET_INPUT_SHAPER(self, gcmd):
        if gcmd.get_command_parameters():
            toolhead_idx = gcmd.get_int("TOOLHEAD", None)
            mode_idx = gcmd.get_int("MODE", None)

            if toolhead_idx is not None or mode_idx is not None:
                th_idx = (
                    self.active_toolhead
                    if toolhead_idx is None
                    else toolhead_idx
                )
                m_idx = self.active_mode if mode_idx is None else mode_idx

                if th_idx not in [0, 1]:
                    raise gcmd.error("Invalid TOOLHEAD index: %d" % th_idx)
                if m_idx not in [MODE_PRIMARY, MODE_COPY, MODE_MIRROR]:
                    raise gcmd.error("Invalid MODE index: %d" % m_idx)

                for shaper in self.shapers:
                    shaper.set_toolhead_mode(th_idx, m_idx)

            for shaper in self.shapers:
                shaper.update(gcmd)
            self._update_input_shaping()
        for shaper in self.shapers:
            shaper.report(gcmd)

    def cmd_GET_INPUT_SHAPER(self, gcmd):
        mode_name = MODE_NAMES.get(self.active_mode)

        gcmd.respond_info(
            "Active toolhead: %d, Mode: %s" % (self.active_toolhead, mode_name)
        )
        for shaper in self.shapers:
            shaper.report(gcmd)


def load_config(config):
    return InputShaper(config)

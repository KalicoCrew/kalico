# Load Cell Probe
#
# Copyright (C) 2025  Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import numpy as np

from klippy import mcu
from klippy.configfile import ConfigWrapper, PrinterConfig
from klippy.extras.bed_mesh import BedMesh
from klippy.extras.homing import PrinterHoming
from klippy.extras.probe import PrinterProbe, ProbePointsHelper
from klippy.gcode import GCodeCommand, GCodeDispatch
from klippy.printer import Printer
from klippy.toolhead import ToolHead

from . import sos_filter
from .interfaces import LoadCellSensor
from .load_cell import (
    LoadCell,
    LoadCellSampleCollector,
)
from .tap_analysis import TapAnalysis, TapAnalysisHelper, TapClassifierModule
from .tap_quality_classifier import TapQualityClassifier

# constants for fixed point numbers
Q2_INT_BITS = 2
Q2_FRAC_BITS = 32 - (1 + Q2_INT_BITS)
Q16_INT_BITS = 16
Q16_FRAC_BITS = 32 - (1 + Q16_INT_BITS)


# Access a parameter from config or GCode command via a consistent interface
# stores name and constraints to keep things DRY
class ParamHelper:
    def __init__(
        self,
        config: ConfigWrapper,
        name: str,
        type_name: str,
        default=None,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        max_len=None,
    ):
        self._printer: Printer = config.get_printer()
        self._config_section = config.get_name()
        self._config_error = config.error
        self.name = name
        self._type_name = type_name
        self.value = default
        self.minval = minval
        self.maxval = maxval
        self.above = above
        self.below = below
        self.max_len = max_len
        # read from config once
        self.value = self.get(config=config)

    def _get_name(self, gcmd):
        return self.name.upper() if gcmd else self.name

    def _validate_float(self, description, error, value, above, below):
        above = above or self.above
        if above is not None and value <= above:
            raise error("%s must be above %s" % (description, above))
        below = below or self.below
        if below is not None and value >= below:
            raise error("%s must be below %s" % (description, below))

    # support for validating individual options in a list of floats
    def _validate_float_list(self, gcmd, values, above, below):
        if gcmd:
            description = "Error on '%s': %s" % (
                gcmd.get_commandline(),
                self._get_name(gcmd),
            )
            error = gcmd.error
        else:
            description = "Option '%s' in section '%s'" % (
                self._get_name(gcmd),
                self._config_section,
            )
            error = self._config_error
        if self.max_len is not None and len(values) > self.max_len:
            raise error(
                "%s has maximum length %s" % (description, self.max_len)
            )
        for value in values:
            self._validate_float(description, error, value, above, below)

    def _get_int(self, config, gcmd, minval, maxval):
        get = gcmd.get_int if gcmd else config.getint
        return get(
            self._get_name(gcmd),
            self.value,
            minval or self.minval,
            maxval or self.maxval,
        )

    def _get_float(self, config, gcmd, minval, maxval, above, below):
        get = gcmd.get_float if gcmd else config.getfloat
        return get(
            self._get_name(gcmd),
            self.value,
            minval or self.minval,
            maxval or self.maxval,
            above or self.above,
            below or self.below,
        )

    def _get_float_list(self, config, gcmd, above, below):
        # this code defaults to the empty list, never return None
        default = self.value or []
        if gcmd:
            # if the parameter isn't part of the command, return the default
            if self._get_name(gcmd) not in gcmd.get_command_parameters():
                return default
            # parameter exists, always prefer whatever is in the command
            value = gcmd.get(self._get_name(gcmd), default="")
            # Return an empty list for empty value
            if len(value.strip()) == 0:
                return []
            try:
                float_list = [float(p.strip()) for p in value.split(",")]
            except:
                raise gcmd.error(
                    "Error on '%s': unable to parse %s"
                    % (gcmd.get_commandline(), value)
                )
        else:
            float_list = config.getfloatlist(
                self._get_name(gcmd), default=default
            )
        if float_list:
            self._validate_float_list(gcmd, float_list, above, below)
        return float_list

    def get(
        self,
        gcmd=None,
        minval=None,
        maxval=None,
        above=None,
        below=None,
        config=None,
    ):
        if config is None and gcmd is None:
            return self.value
        if self._type_name == "int":
            return self._get_int(config, gcmd, minval, maxval)
        elif self._type_name == "float":
            return self._get_float(config, gcmd, minval, maxval, above, below)
        else:
            return self._get_float_list(config, gcmd, above, below)

    def set(self, value):
        self.value = value

    def save(self):
        configfile: PrinterConfig = self._printer.lookup_object("configfile")
        configfile.set(self._config_section, self.name, self.value)


def intParamHelper(config, name, default=None, minval=None, maxval=None):
    return ParamHelper(
        config, name, "int", default, minval=minval, maxval=maxval
    )


def floatParamHelper(
    config, name, default=None, minval=None, maxval=None, above=None, below=None
):
    return ParamHelper(
        config,
        name,
        "float",
        default,
        minval=minval,
        maxval=maxval,
        above=above,
        below=below,
    )


def floatListParamHelper(
    config, name, default=None, above=None, below=None, max_len=None
):
    return ParamHelper(
        config,
        name,
        "float_list",
        default,
        above=above,
        below=below,
        max_len=max_len,
    )


# container for filter parameters
# allows different filter configurations to be compared
class ContinuousTareFilter:
    def __init__(
        self,
        sps=None,
        drift=None,
        drift_delay=None,
        buzz=None,
        buzz_delay=None,
        notches=None,
        notch_quality=None,
    ):
        self.sps = sps
        self.drift = drift
        self.drift_delay = drift_delay
        self.buzz = buzz
        self.buzz_delay = buzz_delay
        self.notches = notches
        self.notch_quality = notch_quality

    def __eq__(self, other):
        if not isinstance(other, ContinuousTareFilter):
            return False
        return (
            self.sps == other.sps
            and self.drift == other.drift
            and self.drift_delay == other.drift_delay
            and self.buzz == other.buzz
            and self.buzz_delay == other.buzz_delay
            and self.notches == other.notches
            and self.notch_quality == other.notch_quality
        )

    # create a filter design from the parameters
    def design_filter(self, error_func):
        design = sos_filter.DigitalFilter(
            self.sps,
            error_func,
            self.drift,
            self.drift_delay,
            self.buzz,
            self.buzz_delay,
            self.notches,
            self.notch_quality,
        )
        fixed_filter = sos_filter.FixedPointSosFilter(
            design.get_filter_sections(),
            design.get_initial_state(),
            Q2_INT_BITS,
            Q16_INT_BITS,
        )
        return fixed_filter


# Combine ContinuousTareFilter and SosFilter into an easy-to-use class
class ContinuousTareFilterHelper:
    def __init__(self, config, sensor, cmd_queue):
        self._sensor = sensor
        self._sps = self._sensor.get_samples_per_second()
        max_filter_frequency = math.floor(self._sps / 2.0)
        # setup filter parameters
        self._drift_param = floatParamHelper(
            config,
            "drift_filter_cutoff_frequency",
            default=None,
            minval=0.1,
            maxval=20.0,
        )
        self._drift_delay_param = intParamHelper(
            config, "drift_filter_delay", default=2, minval=1, maxval=2
        )
        self._buzz_param = floatParamHelper(
            config,
            "buzz_filter_cutoff_frequency",
            default=None,
            above=min(80.0, max_filter_frequency - 1.0),
            below=max_filter_frequency,
        )
        self._buzz_delay_param = intParamHelper(
            config, "buzz_filter_delay", default=2, minval=1, maxval=2
        )
        self._notches_param = floatListParamHelper(
            config,
            "notch_filter_frequencies",
            default=[],
            above=0.0,
            below=max_filter_frequency,
            max_len=2,
        )
        self._notch_quality_param = floatParamHelper(
            config, "notch_filter_quality", default=2.0, minval=0.5, maxval=6.0
        )
        # filter design specified in the config file, used for defaults
        self._config_design = ContinuousTareFilter()  # empty filter
        self._config_design = self._build_filter()
        # filter design currently inside the MCU
        self._active_design = self._config_design
        self._sos_filter = self._create_filter(
            self._active_design.design_filter(config.error), cmd_queue
        )

    def _build_filter(self, gcmd=None):
        drift = self._drift_param.get(gcmd)
        drift_delay = self._drift_delay_param.get(gcmd)
        buzz = self._buzz_param.get(gcmd)
        buzz_delay = self._buzz_delay_param.get(gcmd)
        # notches must be between drift and buzz:
        notches = self._notches_param.get(gcmd, above=drift, below=buzz)
        notch_quality = self._notch_quality_param.get(gcmd)
        return ContinuousTareFilter(
            self._sps,
            drift,
            drift_delay,
            buzz,
            buzz_delay,
            notches,
            notch_quality,
        )

    def _create_filter(self, fixed_filter, cmd_queue):
        return sos_filter.SosFilter(
            self._sensor.get_mcu(), cmd_queue, fixed_filter, 4
        )

    def update_from_command(self, gcmd):
        gcmd_filter = self._build_filter(gcmd)
        # if filters are identical, no change required
        if self._active_design == gcmd_filter:
            return
        # update MCU filter from GCode command
        self._sos_filter.change_filter(
            self._active_design.design_filter(gcmd.error)
        )

    def get_sos_filter(self) -> sos_filter.SosFilter:
        return self._sos_filter

    def save_drift_filter_cutoff_frequency(self, value):
        self._drift_param.set(value)
        self._drift_param.save()


# check results from the collector for errors and raise an exception is found
def check_sensor_errors(results, printer):
    samples, errors = results
    if errors:
        raise printer.command_error(
            "Load cell sensor reported errors while"
            " probing: %i errors, %i overflows" % (errors[0], errors[1])
        )
    return samples


class LoadCellProbeConfigHelper:
    def __init__(
        self,
        config: ConfigWrapper,
        load_cell_inst: LoadCell,
    ):
        self._printer = config.get_printer()
        self._load_cell: LoadCell = load_cell_inst
        self._sensor = load_cell_inst.get_sensor()
        self._rest_time = 1.0 / float(self._sensor.get_samples_per_second())
        # Collect 5 x 50hz power cycles of data to average across power noise
        self._tare_time_param = floatParamHelper(
            config, "tare_time", default=5.0 / 50.0, minval=0.01, maxval=1.0
        )
        # triggering options
        self._trigger_force_param = intParamHelper(
            config, "trigger_force", default=75, minval=10, maxval=250
        )
        self._force_safety_limit_param = intParamHelper(
            config, "force_safety_limit", minval=0, default=2000
        )
        self._drift_safety_limit = intParamHelper(
            config, "drift_safety_limit", minval=0, default=1000
        )
        # pullback move
        self._disable_pullback_move = config.getboolean(
            "disable_pullback_move", False
        )
        self._pullback_distance_param = floatParamHelper(
            config, "pullback_distance", minval=0.01, maxval=2.0, default=0.2
        )
        sps = self._sensor.get_samples_per_second()
        self._pullback_speed_param = floatParamHelper(
            config,
            "pullback_speed",
            minval=0.1,
            maxval=1.0,
            default=sps * 0.001,
        )

    def get_tare_samples(self, gcmd=None) -> int:
        tare_time = self._tare_time_param.get(gcmd)
        sps = self._sensor.get_samples_per_second()
        return max(2, math.ceil(tare_time * sps))

    def get_trigger_force_grams(self, gcmd=None) -> int:
        return self._trigger_force_param.get(gcmd)

    def get_safety_limit_grams(self, gcmd=None) -> int:
        return self._force_safety_limit_param.get(gcmd)

    def get_drift_safety_limit(self, gcmd=None) -> int:
        return self._drift_safety_limit.get(gcmd)

    def get_pullback_speed(self, gcmd=None) -> float:
        return self._pullback_speed_param.get(gcmd)

    def get_pullback_distance(self, gcmd=None) -> float:
        return self._pullback_distance_param.get(gcmd)

    def is_pullback_move_disabled(self) -> bool:
        return self._disable_pullback_move

    def set_pullback_distance(self, value):
        self._pullback_distance_param.set(value)

    def save_pullback_distance(self, value):
        self._pullback_distance_param.set(value)
        self._pullback_distance_param.save()

    def get_rest_time(self) -> float:
        return self._rest_time

    def get_reference_safety_range(self, gcmd=None) -> Tuple[int, int]:
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # calculate the safety band
        zero = self._load_cell.get_reference_tare_counts()
        safety_counts = int(counts_per_gram * self.get_safety_limit_grams(gcmd))
        safety_min = int(zero - safety_counts)
        safety_max = int(zero + safety_counts)
        # don't allow a safety range outside the sensor's real range
        sensor_min, sensor_max = self._load_cell.get_sensor().get_range()
        if safety_min <= sensor_min or safety_max >= sensor_max:
            cmd_err = self._printer.command_error
            raise cmd_err(
                "Load Cell Probe Error: force_safety_limit exceeds"
                " sensor range!"
            )
        return safety_min, safety_max

    # check if tare_counts is within the force_safety_limit
    def assert_force_safety_limit(self, tare_counts, gcmd=None):
        limit = self.get_safety_limit_grams(gcmd)
        # zero limit disables this check
        if limit == 0:
            return
        safety_min, safety_max = self.get_reference_safety_range(gcmd)
        if tare_counts <= safety_min or tare_counts >= safety_max:
            cmd_err = self._printer.command_error
            force = round(self._load_cell.counts_to_grams(tare_counts), 1)
            raise cmd_err(
                "Load Cell Probe Error: force of {}g exceeds "
                "force_safety_limit ({}g) before probing!".format(force, limit)
            )

    def get_probe_drift_range(self, tare_counts, gcmd=None) -> Tuple[int, int]:
        counts_per_gram = self._load_cell.get_counts_per_gram()
        drift_min: int = -(2**31)
        drift_max: int = 2**31 - 1
        drift_force = self.get_drift_safety_limit(gcmd)
        if drift_force > 0:
            drift_counts = int(counts_per_gram * drift_force)
            drift_min = int(tare_counts - drift_counts)
            drift_max = int(tare_counts + drift_counts)
            sensor_min, sensor_max = self._load_cell.get_sensor().get_range()
            if drift_min <= sensor_min or drift_max >= sensor_max:
                cmd_err = self._printer.command_error
                raise cmd_err(
                    "Load Cell Probe Error: drift_safety_limit exceeds"
                    " sensor range!"
                )
        return drift_min, drift_max

    # calculate 1/counts_per_gram in Q2 fixed point
    def get_grams_per_count(self):
        counts_per_gram = self._load_cell.get_counts_per_gram()
        # The counts_per_gram could be so large that it becomes 0.0 when
        # converted to Q2 format. This would mean the ADC range only measures a
        # few grams which seems very unlikely. Treat this as an error:
        if counts_per_gram >= 2**Q2_FRAC_BITS:
            raise OverflowError("counts_per_gram value is too large to filter")
        return sos_filter.to_fixed_32((1.0 / counts_per_gram), Q2_INT_BITS)


# McuLoadCellProbe is the interface to `load_cell_probe` on the MCU
# This also manages the SosFilter so all commands use one command queue
class McuLoadCellProbe:
    WATCHDOG_MAX = 3
    ERROR_SAFETY_RANGE = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 1
    ERROR_OVERFLOW = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 2
    ERROR_WATCHDOG = mcu.MCU_trsync.REASON_COMMS_TIMEOUT + 3

    def __init__(
        self,
        config: ConfigWrapper,
        load_cell_inst: LoadCell,
        sos_filter_inst: sos_filter.SosFilter,
        config_helper: LoadCellProbeConfigHelper,
        trigger_dispatch: mcu.TriggerDispatch,
    ):
        self._printer = config.get_printer()
        self._load_cell = load_cell_inst
        self._sos_filter = sos_filter_inst
        self._config_helper = config_helper
        self._sensor = load_cell_inst.get_sensor()
        self._mcu: mcu.MCU = self._sensor.get_mcu()
        # configure MCU objects
        self._dispatch = trigger_dispatch
        self._cmd_queue = self._dispatch.get_command_queue()
        self._oid = self._mcu.create_oid()
        self._config_commands()
        self._home_cmd = None
        self._query_cmd = None
        self._set_range_cmd = None
        self._mcu.register_config_callback(self._build_config)
        self._printer.register_event_handler("klippy:connect", self._on_connect)

    def _config_commands(self):
        self._sos_filter.create_filter()
        self._mcu.add_config_cmd(
            "config_load_cell_probe oid=%d sos_filter_oid=%d"
            % (self._oid, self._sos_filter.get_oid())
        )

    def _build_config(self):
        self._query_cmd = self._mcu.lookup_query_command(
            "load_cell_probe_query_state oid=%c",
            "load_cell_probe_state oid=%c is_homing_trigger=%c "
            "trigger_ticks=%u",
            oid=self._oid,
            cq=self._cmd_queue,
        )
        self._set_range_cmd = self._mcu.lookup_command(
            "load_cell_probe_set_range"
            " oid=%c safety_counts_min=%i safety_counts_max=%i tare_counts=%i"
            " trigger_grams=%u grams_per_count=%i",
            cq=self._cmd_queue,
        )
        self._home_cmd = self._mcu.lookup_command(
            "load_cell_probe_home oid=%c trsync_oid=%c trigger_reason=%c"
            " error_reason=%c clock=%u rest_ticks=%u timeout=%u",
            cq=self._cmd_queue,
        )

    # the sensor data stream is connected on the MCU at the ready event
    def _on_connect(self):
        self._sensor.attach_load_cell_probe(self._oid)

    def get_oid(self):
        return self._oid

    def get_mcu(self):
        return self._mcu

    def get_load_cell(self) -> LoadCell:
        return self._load_cell

    def get_dispatch(self):
        return self._dispatch

    def set_endstop_range(self, tare_counts: int, gcmd=None):
        # update the load cell so it reflects the new tare value
        self._load_cell.tare(tare_counts)
        # update internal tare value
        safety_min, safety_max = self._config_helper.get_probe_drift_range(
            tare_counts, gcmd
        )
        args = [
            self._oid,
            safety_min,
            safety_max,
            tare_counts,
            self._config_helper.get_trigger_force_grams(gcmd),
            self._config_helper.get_grams_per_count(),
        ]
        self._set_range_cmd.send(args)
        self._sos_filter.reset_filter()

    def home_start(self, print_time):
        clock = self._mcu.print_time_to_clock(print_time)
        rest_time = self._config_helper.get_rest_time()
        rest_ticks = self._mcu.seconds_to_clock(rest_time)
        self._home_cmd.send(
            [
                self._oid,
                self._dispatch.get_oid(),
                mcu.MCU_trsync.REASON_ENDSTOP_HIT,
                self.ERROR_SAFETY_RANGE,
                clock,
                rest_ticks,
                self.WATCHDOG_MAX,
            ],
            reqclock=clock,
        )

    def clear_home(self):
        params = self._query_cmd.send([self._oid])
        # The time of the first sample that triggered is in "trigger_ticks"
        trigger_ticks = self._mcu.clock32_to_clock64(params["trigger_ticks"])
        # clear trsync from load_cell_endstop
        self._home_cmd.send([self._oid, 0, 0, 0, 0, 0, 0, 0])
        return self._mcu.clock_to_print_time(trigger_ticks)


# Execute probing moves using the McuLoadCellProbe
class LoadCellPrimitives:
    ERROR_MAP = {
        mcu.MCU_trsync.REASON_COMMS_TIMEOUT: "Communication timeout during "
        "homing",
        McuLoadCellProbe.ERROR_SAFETY_RANGE: "Load Cell Probe Error: force "
        "exceeded drift_safety_limit before triggering!",
        McuLoadCellProbe.ERROR_OVERFLOW: "Load Cell Probe Error: fixed point "
        "math overflow",
        McuLoadCellProbe.ERROR_WATCHDOG: "Load Cell Probe Error: timed out "
        "waiting for sensor data",
    }

    def __init__(
        self,
        config: ConfigWrapper,
        mcu_load_cell_probe: McuLoadCellProbe,
        continuous_tare_filter_helper: ContinuousTareFilterHelper,
        config_helper: LoadCellProbeConfigHelper,
    ):
        self._printer = config.get_printer()
        self._mcu_load_cell_probe = mcu_load_cell_probe
        self._continuous_tare_filter_helper = continuous_tare_filter_helper
        self._config_helper = config_helper
        self._load_cell = mcu_load_cell_probe.get_load_cell()
        self._dispatch = mcu_load_cell_probe.get_dispatch()
        # internal state tracking
        self._last_trigger_time = 0

    def get_mcu(self):
        return self._mcu_load_cell_probe.get_mcu()

    def get_dispatch(self):
        return self._dispatch

    def _start_collector(self) -> LoadCellSampleCollector:
        toolhead = self._printer.lookup_object("toolhead")
        # homing uses the toolhead last move time which gets special handling
        # to significantly buffer print_time if the move queue has drained
        print_time = toolhead.get_last_move_time()
        collector = self._load_cell.get_collector()
        collector.start_collecting(min_time=print_time)
        return collector

    # pauses for the last move to complete and then
    # sets the endstop tare value and range
    def tare(self, gcmd=None):
        collector = self._start_collector()
        num_samples = self._config_helper.get_tare_samples(gcmd)
        # use collect_min collected samples are not wasted
        results = collector.collect_min(num_samples)
        tare_samples = check_sensor_errors(results, self._printer)
        tare_counts = int(
            np.average(np.array(tare_samples)[:, 2].astype(float))
        )
        self._config_helper.assert_force_safety_limit(tare_counts, gcmd)
        # update sos_filter with any gcode parameter changes
        self._continuous_tare_filter_helper.update_from_command(gcmd)
        self._mcu_load_cell_probe.set_endstop_range(tare_counts, gcmd)

    def home_start(self, print_time):
        # do not permit homing if the load cell is not calibrated
        if not self._load_cell.is_calibrated():
            raise self._printer.command_error("Load Cell not calibrated")
        # start trsync
        trigger_completion = self._dispatch.start(print_time)
        self._mcu_load_cell_probe.home_start(print_time)
        return trigger_completion

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        # trigger has happened, now to find out why...
        res = self._dispatch.stop()
        # clear the homing state so it stops processing samples
        self._last_trigger_time = self._mcu_load_cell_probe.clear_home()
        if res >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            error = "Load Cell Probe Error: unknown reason code %i" % (res,)
            if res in self.ERROR_MAP:
                error = self.ERROR_MAP[res]
            raise self._printer.command_error(error)
        if res != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.0
        return self._last_trigger_time

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self.get_dispatch().get_steppers()

    def query_endstop(self, print_time):
        return False

    # Probe towards z_min until the load_cell_probe on the MCU triggers
    # returns the running collector instance along with the halt position
    def probing_move(
        self, mcu_probe, pos, speed, gcmd
    ) -> tuple[list[float], LoadCellSampleCollector]:
        # tare the sensor just before probing
        self.tare(gcmd)
        # start collector after tare samples are consumed
        collector = self._start_collector()
        # do homing move
        printer_homing: PrinterHoming = self._printer.lookup_object("homing")
        try:
            return printer_homing.probing_move(mcu_probe, pos, speed), collector
        except self._printer.command_error:
            collector.stop_collecting()
            raise

    # Wait for the MCU to trigger with no movement
    def probing_test(self, gcmd, timeout):
        self.tare(gcmd)
        toolhead = self._printer.lookup_object("toolhead")
        print_time = toolhead.get_last_move_time()
        self.home_start(print_time)
        return self.home_wait(print_time + timeout)

    def get_status(self, eventtime):
        status = self._load_cell.get_status(eventtime)
        status.update(
            {
                "last_trigger_time": self._last_trigger_time,
            }
        )
        return status


# Handle homing the z axis with the load cell probe
class HomingMove:
    def __init__(
        self,
        config: ConfigWrapper,
        load_cell_primitives: LoadCellPrimitives,
    ):
        self.printer = config.get_printer()
        self._load_cell_primitives = load_cell_primitives
        # pwrapper methods
        self.get_mcu = load_cell_primitives.get_mcu
        self.add_stepper = load_cell_primitives.add_stepper
        self.get_steppers = load_cell_primitives.get_steppers
        self.home_wait = load_cell_primitives.home_wait

    # Overrides for the MCU_endstop interface
    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        self._load_cell_primitives.tare()
        toolhead = self.printer.lookup_object("toolhead")
        # taring requires time, so print_time must be updated
        print_time = toolhead.get_last_move_time()
        return self._load_cell_primitives.home_start(print_time)

    def query_endstop(self, print_time):
        return False


# Perform a single complete tap, broadcast results to the socket
class TappingMove:
    def __init__(
        self,
        config: ConfigWrapper,
        load_cell_primitives: LoadCellPrimitives,
        tap_analysis_helper: TapAnalysisHelper,
        config_helper: LoadCellProbeConfigHelper,
    ):
        self._printer = config.get_printer()
        self._load_cell_primitives = load_cell_primitives
        self._tap_analysis_helper = tap_analysis_helper
        self._config_helper = config_helper
        # track results of the last tap
        self._last_analysis = None
        self._last_result = None
        self._is_last_result_valid = False
        # wrappers for MCU_endstop use for probing. tap() overrides the
        # MCU_endstop instance to be this object
        self.get_mcu = load_cell_primitives.get_mcu
        self.add_stepper = load_cell_primitives.add_stepper
        self.get_steppers = load_cell_primitives.get_steppers
        self.home_wait = load_cell_primitives.home_wait
        self.query_endstop = load_cell_primitives.query_endstop

    def home_start(
        self, print_time, sample_time, sample_count, rest_time, triggered=True
    ):
        return self._load_cell_primitives.home_start(print_time)

    # Perform the pullback move and returns the time when the move will end
    def pullback_move(self, gcmd):
        toolhead = self._printer.lookup_object("toolhead")
        pullback_pos = toolhead.get_position()
        pullback_pos[2] += self._config_helper.get_pullback_distance(gcmd)
        pos = [None, None, pullback_pos[2]]
        toolhead.manual_move(pos, self._config_helper.get_pullback_speed(gcmd))
        toolhead.flush_step_generation()
        pullback_end = toolhead.get_last_move_time()
        return pullback_end

    # perform a complete tapping cycle
    def probing_move(self, pos, speed, gcmd) -> tuple[list[float], bool]:
        self._is_last_result_valid = False
        # do the probing/homing move
        epos, collector = self._load_cell_primitives.probing_move(
            self, pos, speed, gcmd
        )
        # when pullback is disabled, skip the pullback move and tap analysis
        if self._config_helper.is_pullback_move_disabled():
            collector.stop_collecting()
            self._is_last_result_valid = True
            return epos, self._is_last_result_valid
        # do the pullback move
        pullback_end_time = self.pullback_move(gcmd)
        # collect samples from the tap
        results = collector.collect_until(pullback_end_time)
        # calculate how long we waited to get the data
        t_end = self._printer.get_reactor().monotonic()
        t_end = self.get_mcu().estimated_print_time(t_end)
        collection_time = t_end - pullback_end_time
        # check for data errors
        samples = check_sensor_errors(results, self._printer)
        trigger_force = self._config_helper.get_trigger_force_grams(gcmd)
        # Analyze the tap data
        tap_analysis = self._tap_analysis_helper.analyze(
            samples, trigger_force, collection_time, gcmd
        )
        self._last_analysis = tap_analysis
        self._is_last_result_valid = tap_analysis.is_valid()
        # if the tap is valid, replace the z position with the calculated one
        if self._is_last_result_valid:
            epos[2] = tap_analysis.get_tap_pos()[2]
        return epos, self._is_last_result_valid

    def get_status(self, eventtime):
        status = self._load_cell_primitives.get_status(eventtime)
        status["is_last_tap_valid"] = self._is_last_result_valid
        return status


class LoadCellProbeCommands:
    def __init__(
        self,
        config: ConfigWrapper,
        load_cell_probing_move: LoadCellPrimitives,
    ):
        self._printer = config.get_printer()
        self._load_cell_probing_move = load_cell_probing_move
        self._register_commands()

    def _register_commands(self):
        gcode = self._printer.lookup_object("gcode")
        gcode.register_command(
            "LOAD_CELL_TEST_TAP",
            self.cmd_LOAD_CELL_TEST_TAP,
            desc=self.cmd_LOAD_CELL_TEST_TAP_help,
        )

    cmd_LOAD_CELL_TEST_TAP_help = "Tap the load cell probe to verify operation"

    def cmd_LOAD_CELL_TEST_TAP(self, gcmd: GCodeCommand):
        taps = gcmd.get_int("TAPS", 3, minval=1, maxval=10)
        timeout = gcmd.get_float("TIMEOUT", 30.0, minval=1.0, maxval=120.0)
        gcmd.respond_info("Tap the load cell %s times:" % (taps,))
        reactor = self._printer.get_reactor()
        for i in range(0, taps):
            result = self._load_cell_probing_move.probing_test(gcmd, timeout)
            if result == 0.0:
                # notify of error, likely due to timeout
                raise gcmd.error("Test timeout out")
            gcmd.respond_info("Tap Detected!")
            # give the user some time for their finger to move away
            reactor.pause(reactor.monotonic() + 0.2)
        gcmd.respond_info("Test complete, %s taps detected" % (taps,))


# Probe `activate_gcode` and `deactivate_gcode` support
class ProbeActivationHelper:
    def __init__(self, config: ConfigWrapper):
        self._printer = config.get_printer()
        gcode_macro = self._printer.load_object(config, "gcode_macro")
        self._activate_gcode = gcode_macro.load_template(
            config, "activate_gcode", ""
        )
        self._deactivate_gcode = gcode_macro.load_template(
            config, "deactivate_gcode", ""
        )
        self._is_multi = False
        self._is_active = False

    def multi_probe_begin(self):
        self._is_multi = True
        self.activate_probe()

    def multi_probe_end(self):
        self._is_multi = False
        self.deactivate_probe()

    def probe_prepare(self, hmove):
        self.activate_probe()

    def probe_finish(self, hmove):
        # only deactivate on probe if not doing a multi-probe
        if not self._is_multi:
            self.deactivate_probe()

    def activate_probe(self):
        if self._is_active:
            return
        self._is_active = True
        toolhead = self._printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self._activate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe activate_gcode script"
            )

    def deactivate_probe(self):
        if not self._is_active:
            return
        self._is_active = False
        toolhead = self._printer.lookup_object("toolhead")
        start_pos = toolhead.get_position()
        self._deactivate_gcode.run_gcode_from_command()
        if toolhead.get_position()[:3] != start_pos[:3]:
            raise self._printer.command_error(
                "Toolhead moved during probe deactivate_gcode script"
            )


class LoadCellEndstopWrapper:
    def __init__(
        self,
        config: ConfigWrapper,
        homing_move: HomingMove,
        tapping_move: TappingMove,
    ):
        self._printer = config.get_printer()
        self._z_offset = config.getfloat("z_offset")
        self._tapping_move = tapping_move
        # Register for MCU identification to add Z steppers
        self._printer.register_event_handler(
            "klippy:mcu_identify", self._handle_mcu_identify
        )
        # Wrappers for MCU_endstop interface.
        # Printer homing uses this object as the MCU_endstop
        self.get_mcu = homing_move.get_mcu
        self.add_stepper = homing_move.add_stepper
        self.get_steppers = homing_move.get_steppers
        self.home_wait = homing_move.home_wait
        self.home_start = homing_move.home_start
        self.query_endstop = homing_move.query_endstop
        # wrappers for probe ProbeEndstopWrapper interface
        self.probing_move = tapping_move.probing_move
        self._probe_activation_helper = ProbeActivationHelper(config)
        self.probe_prepare = self._probe_activation_helper.probe_prepare
        self.probe_finish = self._probe_activation_helper.probe_finish
        self.multi_probe_begin = self._probe_activation_helper.multi_probe_begin
        self.multi_probe_end = self._probe_activation_helper.multi_probe_end

    def _handle_mcu_identify(self):
        kin = self._printer.lookup_object("toolhead").get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis("z"):
                self.add_stepper(stepper)

    # Interface for ProbeEndstopWrapper
    def get_position_endstop(self):
        return self._z_offset

    def get_status(self, eventtime):
        status = self._tapping_move.get_status(eventtime)
        status.update(self._tapping_move.get_status(eventtime))
        return status


class DriftFilterCalibration:
    def __init__(
        self,
        config: ConfigWrapper,
        config_helper: ContinuousTareFilterHelper,
        tare_callback,
        load_cell: LoadCell,
    ):
        self._config_helper = config_helper
        self._tare_callback = tare_callback
        self._load_cell = load_cell
        self._printer: Printer = config.get_printer()
        self._horizontal_move_z: float = 5.0
        if config.has_section("bed_mesh"):
            bed_mesh_config = config.getsection("bed_mesh")
            self._horizontal_move_z: float = bed_mesh_config.getfloat(
                "horizontal_move_z", 5.0, note_valid=False
            )
        self._max_z_position: Optional[float] = None
        if config.has_section("stepper_z"):
            zconfig = config.getsection("stepper_z")
            self._max_z_position = zconfig.getfloat(
                "position_max", None, note_valid=False
            )
        if self._max_z_position is None:
            raise config.error("Printer has no configured maximum z-position")
        pconfig = config.getsection("printer")
        self._max_z_velocity: float = pconfig.getfloat(
            "max_z_velocity", None, note_valid=False
        )
        if self._max_z_velocity is None:
            raise config.error("Printer has no configured maximum z-velocity")

    def calibrate(self, gcmd: GCodeCommand):
        try:
            import scipy.signal as signal

            _ = signal
        except:
            raise gcmd.error("Drift Filter requires the SciPy module")
        toolhead: ToolHead = self._printer.lookup_object("toolhead")
        # delta kinematics have a z limit that is lower than stepper limit:
        kinematics = toolhead.get_kinematics()
        if hasattr(kinematics, "limit_z"):
            self._max_z_position = kinematics.limit_z
        probe = self._printer.lookup_object("probe")
        bed_mesh: BedMesh = self._printer.lookup_object(
            "bed_mesh", default=None
        )
        if bed_mesh is None:
            raise gcmd.error("bed_mesh not configured")

        # Calibration parameters
        max_z_position = gcmd.get_float(
            "MAXIMUM_Z_POSITION", minval=0, default=self._max_z_position
        )
        max_z_velocity = gcmd.get_float(
            "MAX_Z_VELOCITY", minval=0, default=self._max_z_velocity
        )
        approach_speed = gcmd.get_float(
            "APPROACH_SPEED", 10.0, above=0.0, maxval=50.0
        )
        segment_duration = gcmd.get_float("SEGMENT_DURATION", 5.0, above=0.0)
        slope_percentile = gcmd.get_float(
            "SLOPE_PERCENTILE", 99.0, minval=0.0, maxval=100.0
        )
        max_drift_rate = gcmd.get_float("MAX_DRIFT_RATE", 1.0, above=0.0)
        max_cutoff_frequency = gcmd.get_float(
            "MAX_CUTOFF_FREQUENCY", 20.0, above=0.0
        )
        cutoff_increment = gcmd.get_float("CUTOFF_INCREMENT", 0.1, above=0.0)

        gcmd.respond_info("Starting drift filter calibration...")
        points = bed_mesh.generate_points(gcmd, "drift_filter_calibrate")
        # Get actual approach speed (limited by Z max speed)
        approach_speed = min(approach_speed, max_z_velocity)

        gcmd.respond_info(
            f"Collecting drift data at {len(points)} points\n"
            f"Z range: {max_z_position:.1f} -> {self._horizontal_move_z:.1f}mm\n"
            f"Approach speed: {approach_speed:.1f}mm/s"
        )

        # Get probe offsets for XY positioning
        max_filter_cutoff = cutoff_increment
        # Get sampling rate from sensor
        probe_offset_x, probe_offset_y, _ = probe.get_offsets()
        sample_rate: int = self._load_cell.get_sensor().get_samples_per_second()
        failed_count = 0
        for i, point in enumerate(points):
            # Adjust for probe offset (this should be 0 for a nozzle probe)
            point = [
                point[0] - probe_offset_x,
                point[1] - probe_offset_y,
                self._max_z_position,
            ]
            # Move to XY position at max Z
            toolhead.manual_move(point, max_z_velocity)
            toolhead.wait_moves()
            # Tare the sensor (probing_move behavior)
            self._tare_callback(gcmd)
            # Start collecting samples
            collector: LoadCellSampleCollector = self._load_cell.get_collector()
            print_time = toolhead.get_last_move_time()
            collector.start_collecting(min_time=print_time)
            try:
                # Start the downward move
                point[2] = self._horizontal_move_z
                toolhead.manual_move(point, approach_speed)
                toolhead.wait_moves()
                move_end_time = toolhead.get_last_move_time()
            except Exception as ex:
                collector.stop_collecting()
                raise ex
            # Stop collecting
            samples, errors = collector.collect_until(move_end_time)
            if errors:
                raise gcmd.error("Load cell errors detected!")
            force = np.array(samples)[:, 1]
            cutoff_freq = self._run_calibration(
                force,
                sample_rate,
                segment_duration,
                slope_percentile,
                max_cutoff_frequency,
                max_filter_cutoff,
                cutoff_increment,
                max_drift_rate,
                gcmd,
            )
            if math.isnan(cutoff_freq):
                failed_count += 1
            else:
                max_filter_cutoff = cutoff_freq
        self._config_helper.save_drift_filter_cutoff_frequency(
            round(max_filter_cutoff, 4)
        )
        if failed_count > 0:
            raise gcmd.error(
                f"WARNING: {failed_count} calibrations failed with the "
                f"max_cutoff_frequency={max_cutoff_frequency}Hz"
            )
        gcmd.respond_info(
            f"Minimum drift filter cutoff: {max_filter_cutoff:.1f}Hz\n"
            f"drift_filter_cutoff_frequency={max_filter_cutoff:.1f}\n"
            "This has been saved for the current session.\n"
            "The SAVE_CONFIG command will update the printer config file "
            "with the above and restart the printer."
        )

    @staticmethod
    def _calculate_segment_slopes(force_data, sampling_rate, segment_duration):
        """Split the graph into segments and calculate a slope for each."""
        segment_samples = int(segment_duration * sampling_rate)
        num_segments = len(force_data) // segment_samples
        segments = np.array_split(force_data, num_segments)
        slopes = []
        for segment in segments:
            time = np.arange(len(segment)) / sampling_rate
            coefficients = np.polyfit(time, segment, 1)
            slopes.append(coefficients[0])
        return np.array(slopes)

    @staticmethod
    def _apply_drift_filter(force_data, cutoff_freq, sampling_rate):
        import scipy.signal as signal

        sos = signal.butter(
            1, cutoff_freq, fs=sampling_rate, btype="highpass", output="sos"
        )
        # use zi to immediately have the filter start in a steady state at 0
        zi = signal.sosfilt_zi(sos)
        zi = zi * force_data[0]
        filtered_data, zo = signal.sosfilt(sos, force_data, zi=zi)
        return filtered_data

    def _run_calibration(
        self,
        force_data: np.ndarray,
        sampling_rate: float,
        segment_duration: float,
        slope_percentile: float,
        max_cutoff_frequency: float,
        cutoff: float,
        cutoff_increment: float,
        max_drift_rate: float,
        gcmd: GCodeCommand,
    ):
        current_cutoff = cutoff
        while current_cutoff <= max_cutoff_frequency:
            filtered_data = self._apply_drift_filter(
                force_data, current_cutoff, sampling_rate
            )
            filtered_slopes = self._calculate_segment_slopes(
                filtered_data, sampling_rate, segment_duration
            )
            post_filter_pct = float(
                np.percentile(np.abs(filtered_slopes), slope_percentile)
            )
            if post_filter_pct <= max_drift_rate:
                gcmd.respond_info(
                    f"Cutoff frequency: {current_cutoff:.4f} Hz\n"
                )
                return current_cutoff
            current_cutoff += cutoff_increment
        gcmd.respond_info(
            f"Calibration FAILED\n"
            f"Could not meet criteria within the {max_cutoff_frequency}Hz "
            f"limit\n"
        )
        return math.nan


class PullbackDistanceCalibration:
    def __init__(
        self,
        config: ConfigWrapper,
        config_helper: LoadCellProbeConfigHelper,
        register_tap_callback: Callable,
    ):
        self._config = config
        self._config_helper = config_helper
        self._register_tap_callback = register_tap_callback
        self._printer = config.get_printer()
        self._bed_mesh_config = None
        if config.has_section("bed_mesh"):
            self._bed_mesh_config = config.getsection("bed_mesh")
        self._calibrating = False
        self._gcmd: Optional[GCodeCommand] = None
        self._distances: list[float] = []
        self._forces: list[float] = []

    @staticmethod
    def _decompression_distance(tap: TapAnalysis) -> tuple[float, float]:
        if not tap.is_valid():
            return math.nan, math.nan

        tap_points = tap.get_tap_points()
        decompression_start_z = tap.get_toolhead_position(tap_points[3].time)[2]
        decompression_start_force = tap_points[3].force
        break_contact_z = tap.get_toolhead_position(tap_points[4].time)[2]
        break_contact_force = tap_points[4].force
        return (
            abs(decompression_start_z - break_contact_z),
            abs(decompression_start_force - break_contact_force),
        )

    def _tap_callback(self, tap: TapAnalysis):
        if not self._calibrating or isinstance(tap, dict):
            return self._calibrating
        decomp_dist, decomp_force = self._decompression_distance(tap)
        if math.isnan(decomp_dist):
            # TODO: consider if we want to allow this...
            return self._calibrating
        self._distances.append(decomp_dist)
        self._forces.append(decomp_force)
        self._gcmd.respond_info(
            f"Decompression distance: {self._distances[-1]:.3}mm, force: "
            f"{self._forces[-1]:.0}g"
        )
        return self._calibrating

    def _finalize_callback(self, probe_offsets, results):
        pass

    def calibrate(self, gcmd: GCodeCommand):
        self._gcmd = gcmd
        gcmd.respond_info("Starting pullback_distance calibration...")
        bed_mesh: BedMesh = self._printer.lookup_object(
            "bed_mesh", default=None
        )
        if bed_mesh is None:
            raise gcmd.error("bed_mesh not configured")

        original_pullback_distance = self._config_helper.get_pullback_distance()
        pullback_distance = gcmd.get_float(
            "PULLBACK_DISTANCE", default=1.0, minval=0.5, maxval=2.0
        )
        self._config_helper.set_pullback_distance(pullback_distance)
        self._calibrating: bool = True
        self._register_tap_callback(self._tap_callback)
        self._distances = []
        self._forces = []

        try:
            points = bed_mesh.generate_points(
                gcmd, "pullback_distance_calibrate"
            )
            points_helper = ProbePointsHelper(
                self._bed_mesh_config,
                self._finalize_callback,
                points,
                use_offsets=True,
                enable_horizontal_z_clearance=True,
            )
            points_helper.start_probe(gcmd)
        finally:
            self._calibrating = False
            self._register_tap_callback(None)
            # restore state in case of an error
            self._config_helper.set_pullback_distance(
                original_pullback_distance
            )

        min_distance = min(self._distances)
        max_distance = max(self._distances)
        mean_distance = float(np.mean(self._distances))
        mean_force = float(np.mean(self._forces))
        distance_std = float(np.std(self._distances))
        pullback_distance = (mean_distance + (3.0 * distance_std)) * 2.0
        deflection_per_kg = (mean_distance / mean_force) * 1000.0

        self._config_helper.save_pullback_distance(pullback_distance)
        gcmd.respond_info(
            f"Decompression Distance: mean={mean_distance:.4f}mm, "
            f"min={min_distance:.4f}mm, max={max_distance:.4f}mm, "
            f"std={distance_std:.4f}mm\n"
            f"pullback_distance={pullback_distance:.4f}\n"
            f"deflection per kilogram={deflection_per_kg:.1f}mm\n"
            "This has been saved for the current session.\n"
            "The SAVE_CONFIG command will update the printer config file "
            "with the above and restart the printer."
        )


class LoadCellPrinterProbe:
    def __init__(
        self,
        config: ConfigWrapper,
        sensor: LoadCellSensor,
        tap_classifier: TapClassifierModule,
    ):
        self._config = config
        self._printer = config.get_printer()
        self._load_cell = LoadCell(config, sensor)
        self._tap_classifier = tap_classifier
        # Read all user configuration and build modules
        name = config.get_name()
        self._tap_analysis_helper = TapAnalysisHelper(
            self._printer, name, tap_classifier
        )
        self._config_helper = LoadCellProbeConfigHelper(config, self._load_cell)
        self._mcu = self._load_cell.get_sensor().get_mcu()
        trigger_dispatch = mcu.TriggerDispatch(self._mcu)
        continuous_tare_filter_helper = ContinuousTareFilterHelper(
            config, sensor, trigger_dispatch.get_command_queue()
        )
        # Probe Interface
        self._mcu_load_cell_probe = McuLoadCellProbe(
            config,
            self._load_cell,
            continuous_tare_filter_helper.get_sos_filter(),
            self._config_helper,
            trigger_dispatch,
        )
        self._load_cell_primitives = LoadCellPrimitives(
            config,
            self._mcu_load_cell_probe,
            continuous_tare_filter_helper,
            self._config_helper,
        )
        homing_move = HomingMove(config, self._load_cell_primitives)
        self._tapping_move = TappingMove(
            config,
            self._load_cell_primitives,
            self._tap_analysis_helper,
            self._config_helper,
        )
        # printer integration
        LoadCellProbeCommands(config, self._load_cell_primitives)
        wrapper = LoadCellEndstopWrapper(
            config, homing_move, self._tapping_move
        )
        printer_probe = PrinterProbe(config, wrapper)
        self._printer.add_object("probe", printer_probe)
        # calibration macro setup:
        self._drift_filter_calibration = DriftFilterCalibration(
            config,
            continuous_tare_filter_helper,
            self._load_cell_primitives.tare,
            self._load_cell,
        )
        self._pullback_distance_calibration = PullbackDistanceCalibration(
            config,
            self._config_helper,
            self._tap_analysis_helper.set_calibration_callback,
        )
        self._register_macros()

    def _register_macros(self):
        gcode_dispatch: GCodeDispatch = self._printer.lookup_object("gcode")
        gcode_dispatch.register_command(
            "LOAD_CELL_PROBE_CALIBRATE",
            self.cmd_LOAD_CELL_PROBE_CALIBRATE,
            desc=self.cmd_LOAD_CELL_PROBE_CALIBRATE_help,
        )

    cmd_LOAD_CELL_PROBE_CALIBRATE_help: str = "Run a calibration routine"

    def cmd_LOAD_CELL_PROBE_CALIBRATE(self, gcmd: GCodeCommand):
        calibration: str = gcmd.get("CALIBRATION")
        if calibration == "DRIFT_FILTER":
            self._drift_filter_calibration.calibrate(gcmd)
        elif calibration == "PULLBACK_DISTANCE":
            self._pullback_distance_calibration.calibrate(gcmd)
        elif calibration == "DECOMPRESSION_ANGLE" and isinstance(
            self._tap_classifier, TapQualityClassifier
        ):
            self._tap_classifier.calibrate(gcmd)
        elif not calibration:
            gcmd.error(
                "CALIBRATION must be one of DRIFT_FILTER, "
                "PULLBACK_DISTANCE or DECOMPRESSION_ANGLE"
            )
        else:
            gcmd.error(f"Unknown CALIBRATION value '{calibration}'")

    def add_client(self, callback):
        self._tap_analysis_helper.add_client(callback)

    def get_status(self, eventtime):
        return self._tapping_move.get_status(eventtime)

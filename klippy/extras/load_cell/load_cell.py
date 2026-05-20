# Load Cell Implementation
#
# Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from __future__ import annotations

import collections
import logging
from dataclasses import dataclass
from enum import Enum

from klippy import Printer
from klippy.configfile import ConfigWrapper
from klippy.extras.bulk_sensor import BatchWebhooksClient
from klippy.extras.load_cell.interfaces import BulkAdcSensor
from klippy.gcode import GCodeCommand, GCodeDispatch
from klippy.toolhead import ToolHead


# alternative to numpy's column selection:
def select_column(data, column_idx):
    return list(zip(*data))[column_idx]


def avg(data):
    return sum(data) / len(data)


def has_flag(gcmd: GCodeCommand, name):
    return gcmd.get(name, default="False").strip().lower() in {"1", "true"}


class SampleStructure(Enum):
    # COLUMN = column index, colum header label
    TIME = 0, "time"
    FORCE_G = 1, "force (g)"
    COUNTS = 2, "counts"
    TARE_COUNTS = 3, "tare_counts"

    def __new__(cls, index, label):
        obj = object.__new__(cls)
        obj._value_ = index
        obj.label = label
        return obj

    @staticmethod
    def header_labels(channel_count: int) -> list[str]:
        labels = [col.label for col in SampleStructure]
        for idx in range(channel_count):
            labels.append(f"channel-{idx} {SampleStructure.FORCE_G.label}")
            labels.append(f"channel-{idx} {SampleStructure.COUNTS.label}")
        return labels

    @staticmethod
    def channel_force_col(idx):
        return len(SampleStructure) + idx * 2

    @staticmethod
    def channel_counts_col(idx):
        return len(SampleStructure) + idx * 2 + 1


# Helper for event driven webhooks and subscription based API clients
class ApiClientHelper(object):
    def __init__(self, printer):
        self.printer = printer
        self.client_cbs = []
        self.webhooks_start_resp = {}

    # send data to clients
    def send(self, msg):
        for client_cb in list(self.client_cbs):
            res = client_cb(msg)
            if not res:
                # This client no longer needs updates - unregister it
                self.client_cbs.remove(client_cb)

    # Add a client that gets data callbacks
    def add_client(self, client_cb):
        self.client_cbs.append(client_cb)

    # Add Webhooks client and send header
    def _add_webhooks_client(self, web_request):
        whbatch = BatchWebhooksClient(web_request)
        self.add_client(whbatch.handle_batch)
        web_request.send(self.webhooks_start_resp)

    # Set up a webhooks endpoint with a static header
    def add_mux_endpoint(self, path, key, value, webhooks_start_resp):
        self.webhooks_start_resp = webhooks_start_resp
        wh = self.printer.lookup_object("webhooks")
        wh.register_mux_endpoint(path, key, value, self._add_webhooks_client)


# Class for handling gcode commands related to load cells
class LoadCellCommandHelper:
    def __init__(self, config: ConfigWrapper, load_cell: LoadCell):
        self.printer: Printer = config.get_printer()
        self.load_cell: LoadCell = load_cell
        name_parts = config.get_name().split()
        self.name = name_parts[-1]
        self.register_commands(self.name)
        if len(name_parts) == 1:
            self.register_commands(None)

    def register_commands(self, name):
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "LOAD_CELL_TARE",
            "LOAD_CELL",
            name,
            self.cmd_LOAD_CELL_TARE,
            desc=self.cmd_LOAD_CELL_TARE_help,
        )
        gcode.register_mux_command(
            "LOAD_CELL_CALIBRATE",
            "LOAD_CELL",
            name,
            self.cmd_LOAD_CELL_CALIBRATE,
            desc=self.cmd_CALIBRATE_LOAD_CELL_help,
        )
        gcode.register_mux_command(
            "LOAD_CELL_READ",
            "LOAD_CELL",
            name,
            self.cmd_LOAD_CELL_READ,
            desc=self.cmd_LOAD_CELL_READ_help,
        )
        gcode.register_mux_command(
            "LOAD_CELL_DIAGNOSTIC",
            "LOAD_CELL",
            name,
            self.cmd_LOAD_CELL_DIAGNOSTIC,
            desc=self.cmd_LOAD_CELL_DIAGNOSTIC_help,
        )

    cmd_LOAD_CELL_TARE_help = "Set the Zero point of the load cell"

    def cmd_LOAD_CELL_TARE(self, gcmd: GCodeCommand):
        reporter = ForceReporter(gcmd, self.load_cell)
        self.load_cell.tare(reporter.channel_counts)
        reporter.report(
            ZeroReference.REFERENCE_TARE, "Load cell tare value: 0 = "
        )
        LoadCellGuidedCalibrationHelper.calibration_warning(
            gcmd, self.load_cell, reporter.counts
        )

    cmd_CALIBRATE_LOAD_CELL_help = "Start interactive calibration tool"

    def cmd_LOAD_CELL_CALIBRATE(self, gcmd: GCodeCommand):
        # just tare or run the whole guided calibration experience?
        if has_flag(gcmd, "TARE"):
            self._calibration_tare(gcmd)
        else:
            LoadCellGuidedCalibrationHelper(self.printer, self.load_cell)

    # set reference tare counts on a calibrated load cell
    def _calibration_tare(self, gcmd: GCodeCommand):
        # it is invalid to set the reference tare counts on an uncalibrated
        # load cell. Prompt user to run full calibration
        if not self.load_cell.is_calibrated():
            raise gcmd.error(
                "Load cell has not been calibrated. Run LOAD_CELL_CALIBRATE first."
            )
        reporter: ForceReporter = ForceReporter(gcmd, self.load_cell)
        reporter.report(ZeroReference.ZERO, "New reference tare value: 0 = ")
        self._report_calibration_change(gcmd, reporter)
        LoadCellGuidedCalibrationHelper.calibration_warning(
            gcmd, self.load_cell, reporter.counts
        )
        save = gcmd.get("SAVE", default="False").strip().lower() in {
            "1",
            "true",
        }
        self.load_cell.set_reference_tare_counts(
            reporter.channel_counts, save=save
        )

    def _report_calibration_change(
        self, gcmd: GCodeCommand, reporter: ForceReporter
    ):
        change = reporter.counts - self.load_cell.reference_tare_counts
        formatter = ForceFormatter(
            counts=change,
            percent=self.load_cell.counts_to_percent(change),
            force=self.load_cell.counts_to_grams(change, ZeroReference.ZERO),
        )
        gcmd.respond_info(
            "Calibration changed by: " + formatter.format(show_counts=False)
        )

    cmd_LOAD_CELL_READ_help = "Take a reading from the load cell"

    def cmd_LOAD_CELL_READ(self, gcmd: GCodeCommand):
        reporter = ForceReporter(gcmd, self.load_cell)
        reporter.report(ZeroReference.TARE)

    cmd_LOAD_CELL_DIAGNOSTIC_help = "Check the health of the load cell"

    def cmd_LOAD_CELL_DIAGNOSTIC(self, gcmd: GCodeCommand):
        gcmd.respond_info("Collecting load cell data for 10 seconds...")
        collector = self.load_cell.get_collector()
        reactor = self.printer.get_reactor()
        collector.start_collecting()
        reactor.pause(reactor.monotonic() + 10.0)
        samples, errors = collector.stop_collecting()
        if errors:
            gcmd.respond_info(
                "Sensor reported errors: %i errors,"
                " %i overflows" % (errors[0], errors[1])
            )
        else:
            gcmd.respond_info("Sensor reported no errors")
        if not samples:
            raise gcmd.error("No samples returned from sensor!")
        counts = select_column(samples, SampleStructure.COUNTS.value)
        range_min, range_max = self.load_cell.saturation_range()
        good_count = 0
        saturation_count = 0
        for sample in counts:
            if sample >= range_max or sample <= range_min:
                saturation_count += 1
            else:
                good_count += 1
        gcmd.respond_info("Samples Collected: %i" % (len(samples)))
        if len(samples) > 2:
            sensor_sps = self.load_cell.sensor.get_samples_per_second()
            sps = float(len(samples)) / (samples[-1][0] - samples[0][0])
            gcmd.respond_info(
                "Measured samples per second: %.1f, "
                "configured: %.1f" % (sps, sensor_sps)
            )
        gcmd.respond_info(
            "Good samples: %i, Saturated samples: %i, Unique"
            " values: %i" % (good_count, saturation_count, len(set(counts)))
        )
        max_pct = self.load_cell.counts_to_percent(max(counts))
        min_pct = self.load_cell.counts_to_percent(min(counts))
        gcmd.respond_info(
            "Sample range: [%.2f%% to %.2f%%]" % (min_pct, max_pct)
        )
        gcmd.respond_info(
            "Sample range / sensor capacity: %.5f%%"
            % ((max_pct - min_pct) / 2.0)
        )


# Class to guide the user through calibrating a load cell
class LoadCellGuidedCalibrationHelper:
    def __init__(self, printer: Printer, load_cell: LoadCell):
        self.printer: Printer = printer
        self.gcode: GCodeDispatch = printer.lookup_object("gcode")
        self.load_cell: LoadCell = load_cell
        self._tare_counts: tuple[int, ...] | None = None
        self._counts_per_gram: float | None = None
        self.tare_percent: float = 0.0
        self.register_commands()
        self.gcode.respond_info(
            "Starting load cell calibration. \n"
            "1.) Remove all load and run TARE. \n"
            "2.) Apply a known load, run CALIBRATE GRAMS=nnn. \n"
            "Complete calibration with the ACCEPT command.\n"
            "Use the ABORT command to quit."
        )

    # warn if the tare point is far enough from zero to reduce sensor range
    @staticmethod
    def calibration_warning(
        gcmd: GCodeCommand, load_cell: LoadCell, counts: int
    ):
        if load_cell.counts_to_percent(counts) <= 2.0:
            return
        gcmd.respond_info(
            "WARNING: tare value is more than 2% away from 0!\n"
            "The load cell's range will be impacted.\n"
            "Check for external force on the load cell."
        )

    def verify_no_active_calibration(
        self,
    ):
        try:
            self.gcode.register_command("TARE", "dummy")
        except self.printer.config_error as e:
            raise self.gcode.error(
                "Already Calibrating a Load Cell. Use ABORT to quit."
            )
        self.gcode.register_command("TARE", None)

    def register_commands(self):
        self.verify_no_active_calibration()
        register_command = self.gcode.register_command
        register_command("ABORT", self.cmd_ABORT, desc=self.cmd_ABORT_help)
        register_command("ACCEPT", self.cmd_ACCEPT, desc=self.cmd_ACCEPT_help)
        register_command("TARE", self.cmd_TARE, desc=self.cmd_TARE_help)
        register_command(
            "CALIBRATE", self.cmd_CALIBRATE, desc=self.cmd_CALIBRATE_help
        )

    # convert the delta of counts to a counts/gram metric
    def counts_per_gram(self, grams: float, cal_counts: tuple[int, ...]):
        tare_sum = sum(self._tare_counts)
        cal_sum = sum(cal_counts)
        return float(abs(tare_sum - cal_sum)) / grams

    # calculate max force that the load cell can register
    # given tare bias, at saturation in kilograms
    def capacity_kg(self, counts_per_gram: float):
        range_min, range_max = self.load_cell.saturation_range()
        tare_sum = sum(self._tare_counts)
        return int((range_max - abs(tare_sum)) / counts_per_gram) / 1000.0

    def finalize(self, save_results: bool = False):
        for name in ["ABORT", "ACCEPT", "TARE", "CALIBRATE"]:
            self.gcode.register_command(name, None)
        if not save_results:
            self.gcode.respond_info("Load cell calibration aborted")
            return
        if self._counts_per_gram is None or self._tare_counts is None:
            self.gcode.respond_info(
                "Calibration process is incomplete, aborting"
            )
        self.load_cell.set_calibration(self._counts_per_gram, self._tare_counts)
        ref_tare_str = ", ".join("%i" % c for c in self._tare_counts)
        self.gcode.respond_info(
            "Load cell calibration settings:\n\n"
            "counts_per_gram: %.6f\n"
            "reference_tare_counts: %s\n\n"
            "The SAVE_CONFIG command will update the printer config file"
            " with the above and restart the printer."
            % (self._counts_per_gram, ref_tare_str)
        )
        self.load_cell.tare(self._tare_counts)

    cmd_ABORT_help = "Abort load cell calibration tool"

    def cmd_ABORT(self, gcmd):
        self.finalize(False)

    cmd_ACCEPT_help = "Accept calibration results and apply to load cell"

    def cmd_ACCEPT(self, gcmd):
        self.finalize(True)

    cmd_TARE_help = "Tare the load cell"

    def cmd_TARE(self, gcmd: GCodeCommand):
        self._counts_per_gram = None  # require re-calibration on tare
        reporter = ForceReporter(gcmd, self.load_cell)
        self._tare_counts = reporter.channel_counts
        reporter.report(
            ZeroReference.ZERO,
            "Load cell tare value: 0 = ",
            show_force=False,
        )
        self.tare_percent = self.load_cell.counts_to_percent(reporter.counts)
        self.calibration_warning(gcmd, self.load_cell, reporter.counts)
        gcmd.respond_info(
            "Now apply a known force to the load cell and enter \
                         the force value with:\n CALIBRATE GRAMS=nnn"
        )

    cmd_CALIBRATE_help = "Enter the load cell value in grams"

    def cmd_CALIBRATE(self, gcmd: GCodeCommand):
        if self._tare_counts is None:
            gcmd.respond_info("You must use TARE first.")
            return
        grams = gcmd.get_float("GRAMS", minval=50.0, maxval=25000.0)
        cal_counts = self.load_cell.avg_counts()
        cal_sum = sum(cal_counts)
        cal_percent = self.load_cell.counts_to_percent(cal_sum)
        c_per_g = self.counts_per_gram(grams, cal_counts)
        cap_kg = self.capacity_kg(c_per_g)
        gcmd.respond_info(
            "Calibration value: %.2f%% (%i), Counts/gram: %.5f, \
            Total capacity: +/- %0.2fKg"
            % (cal_percent, cal_sum, c_per_g, cap_kg)
        )
        range_min, range_max = self.load_cell.saturation_range()
        if cal_sum >= range_max or cal_sum <= range_min:
            raise self.printer.command_error(
                "ERROR: Sensor is saturated with too much load!\n"
                "Use less force to calibrate the load cell."
            )
        if cal_counts == self._tare_counts:
            raise self.printer.command_error(
                "ERROR: Tare and Calibration readings are the same!\n"
                "Check wiring and validate sensor with READ_LOAD_CELL command."
            )
        if (abs(cal_percent - self.tare_percent)) < 1.0:
            raise self.printer.command_error(
                "ERROR: Tare and Calibration readings are less than 1% "
                "different!\n"
                "Use more force when calibrating or a higher sensor gain."
            )
        # only set _counts_per_gram after all errors are raised
        self._counts_per_gram = c_per_g
        if cap_kg < 1.0:
            gcmd.respond_info(
                "WARNING: Load cell capacity is less than 1kg!\n"
                "Check wiring and consider using a lower sensor gain."
            )
        if cap_kg > 25.0:
            gcmd.respond_info(
                "WARNING: Load cell capacity is more than 25Kg!\n"
                "Check wiring and consider using a higher sensor gain."
            )
        gcmd.respond_info("Accept calibration with the ACCEPT command.")


# Utility to collect some samples from the LoadCell for later analysis
# Optionally blocks execution while collecting with reactor.pause()
# can collect a minimum n samples or collect until a specific print_time
# samples returned in [[time],[force],[counts]] arrays for easy processing
class LoadCellSampleCollector:
    def __init__(self, printer, load_cell):
        self._printer = printer
        self._load_cell = load_cell
        self._reactor = printer.get_reactor()
        self._mcu = load_cell.sensor.get_mcu()
        self.min_time = 0.0
        self.max_time = float("inf")
        self.min_count = float("inf")  # In Python 3.5 math.inf is better
        self.is_started = False
        self._completion = None
        self._samples = []
        self._errors = 0
        self._overflows = 0

    # move from the started to stopped state and trigger the completion
    def _complete(self):
        self.is_started = False
        if self._completion is not None:
            self._completion.complete(True)
            self._completion = None

    def _on_samples(self, msg):
        if not self.is_started:
            return False  # already stopped, ignore
        self._errors += msg["errors"]
        self._overflows += msg["overflows"]
        samples = msg["data"]
        for sample in samples:
            time = sample[0]
            if self.min_time <= time <= self.max_time:
                self._samples.append(sample)
            if time > self.max_time:
                self._complete()
        if len(self._samples) >= self.min_count:
            self._complete()
        return self.is_started

    def _finish_collecting(self):
        self.is_started = False
        self.min_time = 0.0
        self.max_time = float("inf")
        self.min_count = float("inf")  # In Python 3.5 math.inf is better
        samples = self._samples
        self._samples = []
        errors = self._errors
        self._errors = 0
        overflows = self._overflows
        self._overflows = 0
        return samples, (errors, overflows) if errors or overflows else 0

    def _collect_until(self, timeout):
        self.start_collecting()
        # calculate print time delay and convert to reactor time
        now = self._reactor.monotonic()
        print_time = self._mcu.estimated_print_time(now)
        wake_time = now + (timeout - print_time)
        if self.is_started:
            self._completion = self._reactor.completion()
            result = self._completion.wait(waketime=wake_time)
            if result is None:
                self._finish_collecting()
                raise self._printer.command_error(
                    f"LoadCellSampleCollector timed out! Errors: {self._errors},"
                    f" Overflows: {self._overflows}"
                )
        return self._finish_collecting()

    # start collecting with no automatic end to collection
    def start_collecting(self, min_time=None):
        if self.is_started:
            return
        self.min_time = min_time if min_time is not None else self.min_time
        self.is_started = True
        self._load_cell.add_client(self._on_samples)

    # stop collecting immediately and return results
    def stop_collecting(self):
        return self._finish_collecting()

    # block execution until at least min_count samples are collected
    # will return all samples collected, not just up to min_count
    def collect_min(self, min_count=1):
        self.min_count = min_count
        if len(self._samples) >= min_count:
            return self._finish_collecting()
        print_time = self._mcu.estimated_print_time(self._reactor.monotonic())
        start_time = max(print_time, self.min_time)
        sps = self._load_cell.sensor.get_samples_per_second()
        return self._collect_until(start_time + 1.0 + (min_count / sps))

    # returns when a sample is collected with a timestamp after print_time
    def collect_until(self, print_time=None):
        self.max_time = print_time
        if len(self._samples) and self._samples[-1][0] >= print_time:
            return self._finish_collecting()
        return self._collect_until(self.max_time + 1.0)


# Printer class that controls the load cell
MIN_COUNTS_PER_GRAM = 1.0


@dataclass
class ForceFormatter:
    counts: int
    percent: float
    force: float | None

    def format(self, show_force=True, show_counts=True) -> str:
        parts: list[str] = []
        if show_force:
            if self.force is None:
                parts.append("---.-g")
            else:
                parts.append(f"{self.force:.1f}g")
            parts.append("/")
        parts.append(f"{self.percent:.2f}%")
        if show_counts:
            parts.append(f"({self.counts})")
        return " ".join(parts)


# enum of zero references for force calculations
class ZeroReference(Enum):
    ZERO = 1  # the raw value 0 from the sensor
    REFERENCE_TARE = 2  # the reference_tare_counts value
    TARE = 3  # the tare_counts value


class ForceReporter:
    channel_counts: tuple[int, ...]
    counts: int
    load_cell: LoadCell
    gcmd: GCodeCommand

    def __init__(self, gcmd: GCodeCommand, load_cell: LoadCell):
        self.channel_counts = load_cell.avg_counts()
        self.counts = sum(self.channel_counts)
        self.gcmd = gcmd
        self.load_cell = load_cell

    # build the summed measurement against the zero reference
    def _summed(self, reference: ZeroReference) -> ForceFormatter:
        lc = self.load_cell
        return ForceFormatter(
            counts=self.counts,
            percent=lc.counts_to_percent(self.counts),
            force=lc.counts_to_grams(self.counts, reference),
        )

    # build the per-channel breakdown against the zero reference
    def _channels(self, reference: ZeroReference) -> list[ForceFormatter]:
        lc = self.load_cell
        return [
            ForceFormatter(
                counts=ch_counts,
                percent=lc.channel_counts_to_percent(ch_counts),
                force=lc.counts_to_grams(ch_counts, reference, channel=idx),
            )
            for idx, ch_counts in enumerate(self.channel_counts)
        ]

    # report the summed measurement against the zero reference,
    # optionally with a per-channel breakdown
    def report(
        self,
        reference: ZeroReference,
        prefix: str = "",
        show_force: bool = True,
        show_channels: bool = True,
    ):
        summed = self._summed(reference)
        self.gcmd.respond_info(prefix + summed.format(show_force=show_force))
        # single channel sensors don't show a per-channel breakdown
        if not show_channels or len(self.channel_counts) == 1:
            return
        for idx, channel in enumerate(self._channels(reference)):
            line = channel.format(show_force=show_force)
            self.gcmd.respond_info(f"    Channel {idx}: {line}")


class LoadCell:
    def __init__(self, config: ConfigWrapper, sensor: BulkAdcSensor):
        self.printer = printer = config.get_printer()
        self.config_name = config.get_name()
        self.name = config.get_name().split()[-1]
        self.sensor = sensor
        self.channel_count = self.sensor.get_channel_count()
        buffer_size = sensor.get_samples_per_second() // 2
        self._force_buffer = collections.deque(maxlen=buffer_size)
        self._channel_force_buffers: list[collections.deque] = [
            collections.deque(maxlen=buffer_size)
            for _ in range(self.channel_count)
        ]
        ref_tare = config.getintlist("reference_tare_counts", default=None)
        if ref_tare is not None:
            if len(ref_tare) != self.channel_count:
                raise config.error(
                    "reference_tare_counts has %d values, expected %d"
                    % (len(ref_tare), self.channel_count)
                )
        self.reference_tare_counts_per_channel: tuple[int, ...] | None = (
            ref_tare
        )
        self.reference_tare_counts: int | None = (
            sum(ref_tare) if ref_tare is not None else None
        )
        self.tare_counts_per_channel = self.reference_tare_counts_per_channel
        self.tare_counts = self.reference_tare_counts
        self.tare_force: float | None = (
            0.0 if self.reference_tare_counts is not None else None
        )
        self.counts_per_gram = config.getfloat(
            "counts_per_gram", minval=MIN_COUNTS_PER_GRAM, default=None
        )
        self.invert = config.getchoice(
            "sensor_orientation",
            {"normal": 1.0, "inverted": -1.0},
            default="normal",
        )
        LoadCellCommandHelper(config, self)
        # Client support:
        self.clients = ApiClientHelper(printer)
        header = {"header": SampleStructure.header_labels(self.channel_count)}
        self.clients.add_mux_endpoint(
            "load_cell/dump_force", "load_cell", self.name, header
        )
        # startup, when klippy is ready, start capturing data
        printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        self.sensor.add_client(self._sensor_data_event)
        self.add_client(self._track_force)
        # announce calibration status on ready
        if self.is_calibrated():
            self.printer.send_event("load_cell:calibrate", self)
        if self.is_tared():
            self.printer.send_event("load_cell:tare", self)

    # convert raw counts to grams and broadcast to clients
    def _sensor_data_event(self, msg):
        data = msg.get("data")
        errors = msg.get("errors")
        overflows = msg.get("overflows")
        if data is None:
            return None
        samples = []
        for row in data:
            channel_counts = row[1:][::2]
            if len(channel_counts) != self.channel_count:
                logging.info(
                    f"Missing Sensor Channels! Expected: "
                    f"{self.channel_count}, found "
                    f"{len(channel_counts)}"
                )
            sum_counts = sum(channel_counts)
            sample = [
                row[0],
                self.counts_to_grams(sum_counts),
                sum_counts,
                self.tare_counts,
            ]
            for idx, counts in enumerate(channel_counts):
                sample.append(
                    self.counts_to_grams(
                        counts, ZeroReference.TARE, channel=idx
                    )
                )
                sample.append(counts)
            samples.append(sample)
        msg = {"data": samples, "errors": errors, "overflows": overflows}
        self.clients.send(msg)
        return True

    # get internal events of force data
    def add_client(self, callback):
        self.clients.add_client(callback)

    def tare(self, tare_counts_per_channel: tuple[int, ...]):
        self.tare_counts_per_channel = tare_counts_per_channel
        self.tare_counts = sum(tare_counts_per_channel)
        if self.is_calibrated():
            tare_delta = float(self.tare_counts - self.reference_tare_counts)
            self.tare_force = round(
                self.invert * (tare_delta / self.counts_per_gram), 1
            )
        self.printer.send_event("load_cell:tare", self)

    def set_reference_tare_counts(
        self,
        tare_counts_per_channel: tuple[int, ...] | None,
        save: bool = False,
    ):
        if tare_counts_per_channel is None:
            raise self.printer.command_error("Missing tare counts")
        self.reference_tare_counts_per_channel = tare_counts_per_channel
        self.reference_tare_counts = sum(tare_counts_per_channel)
        self.tare(self.reference_tare_counts_per_channel)
        if save:
            configfile = self.printer.lookup_object("configfile")
            configfile.set(
                self.config_name,
                "reference_tare_counts",
                ", ".join(
                    "%i" % c for c in self.reference_tare_counts_per_channel
                ),
            )

    def set_calibration(
        self,
        counts_per_gram: float,
        reference_tare_counts_per_channel: tuple[int, ...] | None,
        save=True,
    ):
        if (
            counts_per_gram is None
            or abs(counts_per_gram) < MIN_COUNTS_PER_GRAM
        ):
            raise self.printer.command_error("Invalid counts per gram value")
        self.counts_per_gram = counts_per_gram
        self.set_reference_tare_counts(
            reference_tare_counts_per_channel, save=save
        )
        if save:
            configfile = self.printer.lookup_object("configfile")
            configfile.set(
                self.config_name,
                "counts_per_gram",
                "%.5f" % (self.counts_per_gram,),
            )
        self.printer.send_event("load_cell:calibrate", self)

    def counts_to_grams(
        self,
        counts: int,
        reference: ZeroReference = ZeroReference.TARE,
        channel: int | None = None,
    ) -> float | None:
        if not self.is_calibrated():
            return None
        ref_tare: tuple[int, ...] = (0,) * self.channel_count
        if reference == ZeroReference.REFERENCE_TARE:
            ref_tare = self.reference_tare_counts_per_channel
        elif reference == ZeroReference.TARE:
            ref_tare = self.tare_counts_per_channel
        ref_counts: int = (
            sum(ref_tare) if channel is None else ref_tare[channel]
        )
        sample_delta: float = float(counts - ref_counts)
        return self.invert * (sample_delta / self.counts_per_gram)

    # The maximum range of a single ADC channel, based on its bit width
    def channel_saturation_range(self):
        return self.sensor.get_range()

    # The maximum range for the sum of all channels
    def saturation_range(self):
        range_min, range_max = self.channel_saturation_range()
        return range_min * self.channel_count, range_max * self.channel_count

    # convert raw counts to a +/- percentage of the sensors total range
    def counts_to_percent(self, counts):
        _, range_max = self.saturation_range()
        return (float(counts) / float(range_max)) * 100.0

    def channel_counts_to_percent(self, counts):
        range_min, range_max = self.channel_saturation_range()
        return (float(counts) / float(range_max)) * 100.0

    # read 1 second of load cell data and average it
    # performs safety checks for errors and saturation
    def avg_counts(self, num_samples: int | None = None) -> tuple[int, ...]:
        if num_samples is None:
            num_samples = self.sensor.get_samples_per_second()
        toolhead: ToolHead = self.printer.lookup_object("toolhead")
        # callers expect a load from the context of the call, all moves must
        # finish before samples are valid
        collector: LoadCellSampleCollector = self.get_collector()
        collector.start_collecting(min_time=toolhead.get_last_move_time())
        samples, errors = collector.collect_min(num_samples)
        self.validate_samples(samples, errors)
        return self._avg_counts_from_samples(samples)

    def validate_samples(self, samples, errors):
        if errors:
            raise self.printer.command_error(
                "Sensor reported %i errors while sampling"
                % (errors[0] + errors[1])
            )
        # check individual channels for saturated readings
        range_min, range_max = self.channel_saturation_range()
        first_channel_col = SampleStructure.channel_counts_col(0)
        for sample in samples:
            channel_counts = sample[first_channel_col::2]
            for counts in channel_counts:
                if counts >= range_max or counts <= range_min:
                    raise self.printer.command_error(
                        "Some samples are saturated (+/-100%)"
                    )

    # Provide ongoing force tracking/averaging for status updates
    def _track_force(self, msg):
        if not (self.is_calibrated() and self.is_tared()):
            return True
        samples = msg["data"]
        for sample in samples:
            self._force_buffer.append(sample[SampleStructure.FORCE_G.value])
            for idx in range(self.channel_count):
                col = SampleStructure.channel_force_col(idx)
                self._channel_force_buffers[idx].append(sample[col])
        return True

    def _force_g(self):
        if (
            self.is_calibrated()
            and self.is_tared()
            and len(self._force_buffer) > 0
        ):
            channel_force: list[float] = []
            for buf in self._channel_force_buffers:
                if len(buf) == 0:
                    return None
                channel_force.append(round(avg(buf), 1))
            return {
                "force_g": round(avg(self._force_buffer), 1),
                "force_g_per_channel": channel_force,
                "min_force_g": round(min(self._force_buffer), 1),
                "max_force_g": round(max(self._force_buffer), 1),
            }
        return {}

    def is_tared(self):
        return self.tare_counts is not None

    def is_calibrated(self):
        return (
            self.counts_per_gram is not None
            and self.reference_tare_counts is not None
        )

    def get_sensor(self):
        return self.sensor

    def get_reference_tare_counts(self):
        return self.reference_tare_counts

    def get_tare_counts(self):
        return self.tare_counts

    def get_counts_per_gram(self):
        return self.counts_per_gram

    # convert samples into average per channel
    def _avg_counts_from_samples(self, samples: list) -> tuple[int, ...]:
        per_channel: list[int] = []
        for idx in range(self.channel_count):
            col = SampleStructure.channel_counts_col(idx)
            per_channel.append(int(avg(select_column(samples, col))))
        return tuple(per_channel)

    def get_collector(self) -> LoadCellSampleCollector:
        return LoadCellSampleCollector(self.printer, self)

    def get_status(self, eventtime):
        status = self._force_g()
        status.update(
            {
                "is_calibrated": self.is_calibrated(),
                "counts_per_gram": self.counts_per_gram,
                "reference_tare_counts": self.reference_tare_counts,
                "reference_tare_counts_per_channel": (
                    self.reference_tare_counts_per_channel
                ),
                "tare_counts": self.tare_counts,
                "tare_counts_per_channel": self.tare_counts_per_channel,
                "tare_force": self.tare_force,
            }
        )
        return status

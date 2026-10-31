# Z Thermal Adjust
#
# Copyright (C) 2022  Robert Pazdzior <robertp@norbital.com>
# Copyright (C) 2026  Gareth Farrington <garth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# Adjusts Z position in real-time using a thermal probe to e.g. compensate
# for thermal expansion of the printer frame.
from __future__ import annotations

import os
import socket
import threading
import time

import numpy as np

from klippy import Printer
from klippy.configfile import ConfigWrapper, PrinterConfig
from klippy.extras.heaters import Heater, PrinterHeaters
from klippy.extras.probe import ProbeList
from klippy.gcode import (
    GCodeCommand,
    GCodeDispatch,
)
from klippy.toolhead import ToolHead

KELVIN_TO_CELSIUS = -273.15


class ZThermalAdjuster:
    def __init__(self, config: ConfigWrapper):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.lock = threading.Lock()
        self.config: ConfigWrapper = config

        # Get config parameters, convert to SI units where necessary
        self.temp_coeff = config.getfloat(
            "temp_coeff", minval=-1, maxval=1, default=0
        )
        self.off_above_z = config.getfloat("z_adjust_off_above", 99999999.0)
        self.max_z_adjust_mm = config.getfloat("max_z_adjustment", 99999999.0)

        # Register printer events
        self.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )
        self.printer.register_event_handler(
            "homing:home_rails_end", self.handle_homing_move_end
        )

        # Setup temperature sensor
        self.smooth_time = config.getfloat("smooth_time", 2.0, above=0.0)
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        pheaters = self.printer.load_object(config, "heaters")
        self.sensor = pheaters.setup_sensor(config)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.init_temperature_callback)
        pheaters.register_sensor(config, self)

        self.last_temp = 0.0
        self.measured_min = self.measured_max = 0.0
        self.smoothed_temp = 0.0
        self.last_temp_time = 0.0
        self.ref_temperature = 0.0
        self.ref_temp_override = False

        # Z transformation
        self.z_adjust_mm = 0.0
        self.adjust_enable = True
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.next_transform = None

        # Register gcode commands
        full_name = config.get_name()
        self.component = None
        if not full_name == "z_thermal_adjust":
            self.component = full_name.split(maxsplit=1)[-1]
        self.register_commands(self.component)

    def register_commands(self, component):
        self.gcode.register_mux_command(
            "SET_Z_THERMAL_ADJUST",
            "COMPONENT",
            component,
            self.cmd_SET_Z_THERMAL_ADJUST,
            desc=self.cmd_SET_Z_THERMAL_ADJUST_help,
        )
        self.gcode.register_mux_command(
            "Z_THERMAL_ADJUST_CALIBRATE",
            "COMPONENT",
            component,
            self.cmd_Z_THERMAL_ADJUST_CALIBRATE,
            desc=self.cmd_Z_THERMAL_ADJUST_CALIBRATE_help,
        )

    def handle_connect(self):
        "Called after all printer objects are instantiated"
        self.toolhead = self.printer.lookup_object("toolhead")
        gcode_move = self.printer.lookup_object("gcode_move")

        # Register move transformation
        self.next_transform = gcode_move.set_move_transform(self, force=True)

        # Pull Z step distance for minimum adjustment increment
        kin = self.printer.lookup_object("toolhead").get_kinematics()
        steppers = [s.get_name() for s in kin.get_steppers()]
        z_stepper = kin.get_steppers()[steppers.index("stepper_z")]
        self.z_step_dist = z_stepper.get_step_dist()

    def get_status(self, eventtime):
        return {
            "temperature": self.smoothed_temp,
            "measured_min_temp": round(self.measured_min, 2),
            "measured_max_temp": round(self.measured_max, 2),
            "current_z_adjust": self.z_adjust_mm,
            "z_adjust_ref_temperature": self.ref_temperature,
            "enabled": self.adjust_enable,
        }

    def handle_homing_move_end(self, homing_state, rails):
        "Set reference temperature after Z homing."
        if 2 in homing_state.get_axes():
            self.ref_temperature = self.smoothed_temp
            self.ref_temp_override = False
            self.z_adjust_mm = 0.0

    def calc_adjust(self, pos):
        "Z adjustment calculation"
        if pos[2] < self.off_above_z:
            delta_t = self.smoothed_temp - self.ref_temperature

            # Calculate Z adjustment
            adjust = -1 * self.temp_coeff * delta_t

            # compute sign (+1 or -1) for maximum offset setting
            sign = 1 - (adjust <= 0) * 2

            # Don't apply adjustments smaller than step distance
            if abs(adjust - self.z_adjust_mm) > self.z_step_dist:
                self.z_adjust_mm = min(
                    [self.max_z_adjust_mm * sign, adjust], key=abs
                )

        # Apply Z adjustment
        new_z = pos[2] + self.z_adjust_mm
        return [pos[0], pos[1], new_z] + pos[3:]

    def calc_unadjust(self, pos):
        "Remove Z adjustment"
        unadjusted_z = pos[2] - self.z_adjust_mm
        return [pos[0], pos[1], unadjusted_z] + pos[3:]

    def get_position(self):
        position = self.calc_unadjust(self.next_transform.get_position())
        self.last_position = self.calc_adjust(position)
        return position

    def move(self, newpos, speed):
        # don't apply to extrude only moves or when disabled
        if (newpos[0:3] == self.last_position[0:3]) or not self.adjust_enable:
            z = newpos[2] + self.z_adjust_mm
            adjusted_pos = [newpos[0], newpos[1], z, newpos[3]]
            self.next_transform.move(adjusted_pos, speed)
        else:
            adjusted_pos = self.calc_adjust(newpos)
            self.next_transform.move(adjusted_pos, speed)
        self.last_position[:] = newpos

    def init_temperature_callback(self, read_time, temp):
        "Initialize Z adjust thermistor ref temp"
        with self.lock:
            self.ref_temperature = temp
            self.sensor.setup_callback(self.temperature_callback)

    def temperature_callback(self, read_time, temp):
        "Called everytime the Z adjust thermistor is read"
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time
            self.measured_min = min(self.measured_min, self.smoothed_temp)
            self.measured_max = max(self.measured_max, self.smoothed_temp)

    def get_temp(self, eventtime):
        return self.smoothed_temp, 0.0

    def stats(self, eventtime):
        return False, "%s: temp=%.1f" % ("z_thermal_adjust", self.smoothed_temp)

    def save_temp_coeff(self, temp_coeff):
        if temp_coeff < -1.0 or temp_coeff > 1.0:
            raise self.config.error(
                f"temp_coeff value {temp_coeff} is out of range -1.0 to 1.0"
            )
        self.temp_coeff = temp_coeff
        configfile: PrinterConfig = self.printer.lookup_object("configfile")
        configfile.set(self.config.get_name(), "temp_coeff", self.temp_coeff)

    def cmd_SET_Z_THERMAL_ADJUST(self, gcmd):
        enable = gcmd.get_int("ENABLE", None, minval=0, maxval=1)
        coeff = gcmd.get_float("TEMP_COEFF", None, minval=-1, maxval=1)
        ref_temp = gcmd.get_float("REF_TEMP", None, minval=KELVIN_TO_CELSIUS)

        if ref_temp is not None:
            self.ref_temperature = ref_temp
            self.ref_temp_override = True
        if coeff is not None:
            self.temp_coeff = coeff
        if enable is not None:
            if enable != self.adjust_enable:
                self.adjust_enable = True if enable else False
                gcode_move = self.printer.lookup_object("gcode_move")
                gcode_move.reset_last_position()

        state = "1 (enabled)" if self.adjust_enable else "0 (disabled)"
        override = " (manual)" if self.ref_temp_override else ""
        component = ""
        if self.component is not None:
            component = "component: %s\n" % (self.component,)
        msg = (
            "%s"
            "enable: %s\n"
            "temp_coeff: %f mm/degC\n"
            "ref_temp: %.2f degC%s\n"
            "-------------------\n"
            "Current Z temp: %.2f degC\n"
            "Applied Z adjustment: %.4f mm"
            % (
                component,
                state,
                self.temp_coeff,
                self.ref_temperature,
                override,
                self.smoothed_temp,
                self.z_adjust_mm,
            )
        )
        gcmd.respond_info(msg)

    cmd_SET_Z_THERMAL_ADJUST_help = "Set/query Z Thermal Adjust parameters."

    cmd_Z_THERMAL_ADJUST_CALIBRATE_help = (
        "Measure thermal expansion coefficient (requires HEATER parameter)"
    )

    def cmd_Z_THERMAL_ADJUST_CALIBRATE(self, gcmd: GCodeCommand):
        calibration = gcmd.get("CALIBRATION")
        if calibration == "NOZZLE":
            NozzleCalibration(self, self.printer, gcmd).calibrate()
        elif calibration == "FRAME":
            NozzleCalibration(self, self.printer, gcmd).calibrate()
        else:
            raise gcmd.error("CALIBRATION must be NOZZLE or FRAME")


class FrameCalibration:
    def __init__(self, gcmd: GCodeCommand):
        self.gcmd: GCodeCommand = gcmd

    def calibrate(self):
        # TK a frame calibration routine that uses a frame temp sensor
        raise self.gcmd.error("Not Implemented")


class NozzleCalibration:
    def __init__(
        self,
        z_thermal_adjust: ZThermalAdjuster,
        printer: Printer,
        gcmd: GCodeCommand,
    ):
        self.gcmd: GCodeCommand = gcmd
        self.printer: Printer = printer
        self.z_thermal_adjust = z_thermal_adjust
        self.component = z_thermal_adjust.component
        self.gcode: GCodeDispatch = printer.lookup_object("gcode")
        self.toolhead: ToolHead = self.printer.lookup_object("toolhead")
        self.pheaters: PrinterHeaters = self.printer.lookup_object("heaters")

        self.probe = ProbeList.get_list(printer).get_command_probe(gcmd)
        heater_name: str = gcmd.get("HEATER")
        try:
            self.heater: Heater = self.pheaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))

        self.min_temp: float = gcmd.get_float("MIN_TEMP", 140.0)
        self.max_temp: float = gcmd.get_float("MAX_TEMP", 250.0)
        self.soak_time: float = gcmd.get_float("SOAK_TIME", 20.0)
        self.measurements: int = gcmd.get_int("MEASUREMENTS", 10, minval=2)
        self.z_lift: float = gcmd.get_float("Z_LIFT", 10.0)
        self.z_lift_speed: float = gcmd.get_float("Z_LIFT_SPEED", 5.0)
        self.temperatures = []
        self.z_values = []

    def _calibration_temperatures(self) -> list[float]:
        if self.max_temp <= self.min_temp:
            raise self.gcmd.error("MAX_TEMP must be greater than MIN_TEMP")
        step_size = (self.max_temp - self.min_temp) / (self.measurements - 1)
        temps = [
            round(self.min_temp + step_size * i, 1)
            for i in range(self.measurements)
        ]
        temps[0] = self.min_temp
        temps[-1] = self.max_temp
        return temps

    def calibrate(self):
        self.gcmd.respond_info("Starting thermal coefficient measurement...")
        temps_range = self._calibration_temperatures()

        try:
            # Measure temperature going up
            for temp in temps_range:
                self.gcmd.respond_info(f"Heating to {temp}C...")
                self._change_temp(temp)
                self.temperatures.append(temp)
                self._measure_z()

            # Measure temperature going down
            for temp in reversed(temps_range):
                self.gcmd.respond_info(f"Cooling to {temp}C...")
                self._change_temp(temp)
                self.temperatures.append(temp)
                self._measure_z()
        finally:
            # Turn off heater
            self.pheaters.set_temperature(self.heater, 0)

        # Perform linear regression
        temp_coeff = self._calculate_and_report_coeff()
        self._save_config(temp_coeff)

    def _change_temp(self, temp: float):
        self.pheaters.set_temperature(self.heater, temp, wait=True)
        self.toolhead.dwell(self.soak_time)

    def _measure_z(self):
        probe_result: list[float] = self.probe.run_probe(self.gcmd)
        self.z_values.append(probe_result[2])
        probe_result[2] += self.z_lift
        self.toolhead.manual_move(probe_result, speed=self.z_lift_speed)

    def _plot_results(self, slope: float, intercept: float):
        try:
            import matplotlib.pyplot as plt
        except:
            self.gcmd.respond_info(
                "matplotlib not installed, cannot plot results!"
            )
            return

        # Save into the logs directory, which Moonraker serves
        start_args = self.printer.get_start_args()
        log_file = start_args.get("log_file")
        if not log_file:
            self.gcmd.respond_info("Logging not enabled, cannot plot results!")
            return

        fig, ax = plt.subplots()
        z_min = min(self.z_values)
        z_values = [z_value - z_min for z_value in self.z_values]
        ax.scatter(self.temperatures, z_values, label="measurements")
        line_x = np.array([min(self.temperatures), max(self.temperatures)])
        line_y = slope * line_x + (intercept - z_min)
        ax.plot(line_x, line_y, color="tab:red", label=f"temp_coeff=-{slope}")
        ax.set_title("Nozzle Elongation Coefficient")
        ax.set_xlabel("Temperature °C")
        ax.set_ylabel("Nozzle Elongation mm")
        ax.legend()
        root, root_dir = "logs", os.path.dirname(log_file)
        timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
        filename = f"nozzle_elongation_{timestamp}.png"
        fig.savefig(os.path.join(root_dir, filename), dpi=150)
        plt.close(fig)
        self.gcmd.respond_info(
            "View the Nozzle elongation results: "
            f"http://{socket.gethostname()}/server/files/{root}/{filename}"
        )

    def _calculate_and_report_coeff(self) -> float:
        # I don't know that there is a lot of value in this:
        # A picture would be worth 1000 words here
        # self.gcmd.respond_info(f"Temperatures: {self.temperatures}")
        # self.gcmd.respond_info(f"Z Values: {self.z_values}")

        # say something a human can understand
        expansion = max(self.z_values) - min(self.z_values)
        temp_range = max(self.temperatures) - min(self.temperatures)
        self.gcmd.respond_info(
            f"Nozzle expands {expansion}mm over {temp_range}C"
        )

        # Linear regression using numpy
        temps = np.array(self.temperatures)
        z_vals = np.array(self.z_values)
        a = np.vstack([temps, np.ones(len(temps))]).T
        slope, intercept = np.linalg.lstsq(a, z_vals, rcond=None)[0]
        slope = round(slope, 8)
        temp_coeff = -1.0 * slope

        self._plot_results(slope, intercept)
        self.gcmd.respond_info(f"Temperature Coefficient: {temp_coeff}")
        return temp_coeff

    def _save_config(self, temp_coeff: float):
        self.z_thermal_adjust.save_temp_coeff(temp_coeff)
        self.gcmd.respond_info(
            "Temperature coefficient saved. "
            "Run SAVE_CONFIG to update printer.cfg and restart."
        )


def load_config_prefix(config):
    return ZThermalAdjuster(config)


def load_config(config):
    return ZThermalAdjuster(config)

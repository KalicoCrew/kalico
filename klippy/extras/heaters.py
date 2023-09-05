# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os
import threading


######################################################################
# Heater
######################################################################

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.0
PID_PARAM_BASE = 255.0


class Heater:
    def __init__(self, config, sensor, secondary_sensor=None):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat("min_temp", minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat("max_temp", above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        self.secondary_sensor = secondary_sensor
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            "min_extrude_temp",
            170.0,
            minval=self.min_temp,
            maxval=self.max_temp,
        )
        is_fileoutput = (
            self.printer.get_start_args().get("debugoutput") is not None
        )
        self.can_extrude = self.min_extrude_temp <= 0.0 or is_fileoutput
        self.max_power = config.getfloat(
            "max_power", 1.0, above=0.0, maxval=1.0
        )
        self.smooth_time = config.getfloat("smooth_time", 1.0, above=0.0)
        self.inv_smooth_time = 1.0 / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.0
        self.last_temp_time = 0.0
        # pwm caching
        self.next_pwm_time = 0.0
        self.last_pwm_value = 0.0
        # Setup control algorithm sub-class
        algos = {
            "watermark": ControlBangBang,
            "pid": ControlPID,
            "pid_v": ControlVelocityPID,
            "dual_loop_pid": ControlDualLoopPID,
        }
        algo = config.getchoice("control", algos)
        self.control = algo(self, config)
        if (
            isinstance(self.control, ControlDualLoopPID)
            and self.secondary_sensor is None
        ):
            raise config.error("dual_loop_pid requires a secondary sensor")
        # Setup output heater pin
        heater_pin = config.get("heater_pin")
        ppins = self.printer.lookup_object("pins")
        self.mcu_pwm = ppins.setup_pin("pwm", heater_pin)
        pwm_cycle_time = config.getfloat(
            "pwm_cycle_time", 0.100, above=0.0, maxval=self.pwm_delay
        )
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (self.name,))
        self.printer.load_object(config, "pid_calibrate")
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_HEATER_TEMPERATURE",
            "HEATER",
            self.name,
            self.cmd_SET_HEATER_TEMPERATURE,
            desc=self.cmd_SET_HEATER_TEMPERATURE_help,
        )
        gcode.register_mux_command(
            "SET_HEATER_PID",
            "HEATER",
            self.name,
            self.cmd_SET_HEATER_PID,
            desc=self.cmd_SET_HEATER_PID_help,
        )

    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.0:
            value = 0.0
        if (read_time < self.next_pwm_time or not self.last_pwm_value) and abs(
            value - self.last_pwm_value
        ) < 0.05:
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)
        # logging.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #              self.name, value, pwm_time,
        #              self.last_temp, self.last_temp_time, self.target_temp)

    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            if self.secondary_sensor is not None:
                status = self.secondary_sensor.get_status(read_time)
                secondary_temp = status["temperature"]
            else:
                secondary_temp = None
            self.control.temperature_update(
                read_time, temp, self.target_temp, secondary_temp=secondary_temp
            )
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.0)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = self.smoothed_temp >= self.min_extrude_temp
        # logging.debug("temp: %.3f %f = %f", read_time, temp)

    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay

    def get_max_power(self):
        return self.max_power

    def get_smooth_time(self):
        return self.smooth_time

    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp)
            )
        with self.lock:
            self.target_temp = degrees

    def get_temp(self, eventtime):
        print_time = (
            self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.0
        )
        with self.lock:
            if self.last_temp_time < print_time:
                return 0.0, self.target_temp
            return self.smoothed_temp, self.target_temp

    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp
            )

    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.0
        return old_control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp

    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.0
        return is_active, "%s: target=%.0f temp=%.1f pwm=%.3f" % (
            self.name,
            target_temp,
            last_temp,
            last_pwm_value,
        )

    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
        return {
            "temperature": round(smoothed_temp, 2),
            "target": target_temp,
            "power": last_pwm_value,
        }

    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        temp = gcmd.get_float("TARGET", 0.0)
        pheaters = self.printer.lookup_object("heaters")
        pheaters.set_temperature(self, temp)

    cmd_SET_HEATER_PID_help = "Sets a heater PID parameter"

    def cmd_SET_HEATER_PID(self, gcmd):
        if not isinstance(self.control, ControlPID):
            raise gcmd.error("Not a PID controlled heater")
        kp = gcmd.get_float("KP", None)
        if kp is not None:
            self.control.Kp = kp / PID_PARAM_BASE
        ki = gcmd.get_float("KI", None)
        if ki is not None:
            self.control.Ki = ki / PID_PARAM_BASE
        kd = gcmd.get_float("KD", None)
        if kd is not None:
            self.control.Kd = kd / PID_PARAM_BASE


######################################################################
# Bang-bang control algo
######################################################################


class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat("max_delta", 2.0, above=0.0)
        self.heating = False

    def temperature_update(self, read_time, temp, target_temp, *args, **kwargs):
        if self.heating and temp >= target_temp + self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp - self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.0)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp - self.max_delta

    def get_type(self):
        return "watermark"


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.0
PID_SETTLE_SLOPE = 0.1


class ControlPID:
    def __init__(
        self,
        heater,
        config,
        pid_kp_arg_name="pid_kp",
        pid_ki_arg_name="pid_ki",
        pid_kd_arg_name="pid_kd",
    ):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self._Kp = config.getfloat(pid_kp_arg_name) / PID_PARAM_BASE
        self._Ki = config.getfloat(pid_ki_arg_name) / PID_PARAM_BASE
        self._Kd = config.getfloat(pid_kd_arg_name) / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        self.temp_integ_max = 0.0
        if self._Ki:
            self.temp_integ_max = self.heater_max_power / self._Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.0
        self.prev_temp_deriv = 0.0
        self.prev_temp_integ = 0.0

    def calculate_output(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (
                self.prev_temp_deriv * (self.min_deriv_time - time_diff)
                + temp_diff
            ) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0.0, min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self._Kp * temp_err + self._Ki * temp_integ - self._Kd * temp_deriv
        # logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

        return co, bounded_co

    def temperature_update(self, read_time, temp, target_temp, *args, **kwargs):
        _, bounded_co = self.calculate_output(read_time, temp, target_temp)
        self.heater.set_pwm(read_time, bounded_co)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA
            or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE
        )

    @property
    def Kp(self):
        return self._Kp * PID_PARAM_BASE

    @property
    def Ki(self):
        return self._Ki * PID_PARAM_BASE

    @property
    def Kd(self):
        return self._Kd * PID_PARAM_BASE

    @Kp.setter
    def Kp(self, value):
        self._Kp = value / PID_PARAM_BASE

    @Ki.setter
    def Ki(self, value):
        self._Ki = value / PID_PARAM_BASE

    @Kd.setter
    def Kd(self, value):
        self._Kd = value / PID_PARAM_BASE

    def get_type(self):
        return "pid"


######################################################################
# Velocity (PID) control algo
######################################################################


class ControlVelocityPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat("pid_Kp") / PID_PARAM_BASE
        self.Ki = config.getfloat("pid_Ki") / PID_PARAM_BASE
        self.Kd = config.getfloat("pid_Kd") / PID_PARAM_BASE
        self.smooth_time = heater.get_smooth_time()  # smoothing window
        self.temps = [AMBIENT_TEMP] * 3  # temperature readings
        self.times = [0.0] * 3  # temperature reading times
        self.d1 = 0.0  # previous smoothed 1st derivative
        self.d2 = 0.0  # previous smoothed 2nd derivative
        self.pwm = 0.0  # the previous pwm setting

    def temperature_update(self, read_time, temp, target_temp):
        # update the temp and time lists
        self.temps.pop(0)
        self.temps.append(temp)
        self.times.pop(0)
        self.times.append(read_time)

        # calculate the 1st derivative: p part in velocity form
        # note the derivative is of the temp and not the error
        # this is to prevent derivative kick
        d1 = self.temps[-1] - self.temps[-2]

        # calculate the error : i part in velocity form
        error = self.times[-1] - self.times[-2]
        error = error * (target_temp - self.temps[-1])

        # calculate the 2nd derivative: d part in velocity form
        # note the derivative is of the temp and not the error
        # this is to prevent derivative kick
        d2 = self.temps[-1] - 2.0 * self.temps[-2] + self.temps[-3]
        d2 = d2 / (self.times[-1] - self.times[-2])

        # smooth both the derivatives using a modified moving average
        # that handles unevenly spaced data points
        n = max(1.0, self.smooth_time / (self.times[-1] - self.times[-2]))
        self.d1 = ((n - 1.0) * self.d1 + d1) / n
        self.d2 = ((n - 1.0) * self.d2 + d2) / n

        # calculate the output
        p = self.Kp * -self.d1  # invert sign to prevent derivative kick
        i = self.Ki * error
        d = self.Kd * -self.d2  # invert sign to prevent derivative kick

        self.pwm = max(0.0, min(self.heater_max_power, self.pwm + p + i + d))
        if target_temp == 0.0:
            self.pwm = 0.0

        # update the heater
        self.heater.set_pwm(read_time, self.pwm)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (
            abs(temp_diff) > PID_SETTLE_DELTA or abs(self.d1) > PID_SETTLE_SLOPE
        )

    def get_type(self):
        return "pid_v"


class ControlDualLoopPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()

        self.primary_pid = ControlPID(
            heater, config, "primary_pid_kp", "primary_pid_ki", "primary_pid_kd"
        )
        self.secondary_pid = ControlPID(
            heater,
            config,
            "secondary_pid_kp",
            "secondary_pid_ki",
            "secondary_pid_kd",
        )
        arg_name = "secondary_max_temp_target"
        self.sec_max_temp_target = config.getfloat(arg_name)

    def temperature_update(
        self,
        read_time,
        primary_temp,
        target_temp,
        secondary_temp,
        *args,
        **kwargs
    ):

        pco, _ = self.primary_pid.calculate_output(
            read_time, primary_temp, target_temp
        )
        sco, _ = self.secondary_pid.calculate_output(
            read_time, secondary_temp, self.sec_max_temp_target
        )

        co = min(pco, sco)
        bounded_co = max(0.0, min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return self.primary_pid.check_busy(
            eventtime, smoothed_temp, target_temp
        )


######################################################################
# Sensor and heater lookup
######################################################################


class PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.gcode_id_to_sensor = {}
        self.available_heaters = []
        self.available_sensors = []
        self.available_monitors = []
        self.has_started = self.have_load_sensors = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler(
            "gcode:request_restart", self.turn_off_all_heaters
        )
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "TURN_OFF_HEATERS",
            self.cmd_TURN_OFF_HEATERS,
            desc=self.cmd_TURN_OFF_HEATERS_help,
        )
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
        gcode.register_command(
            "TEMPERATURE_WAIT",
            self.cmd_TEMPERATURE_WAIT,
            desc=self.cmd_TEMPERATURE_WAIT_help,
        )

    def load_config(self, config):
        self.have_load_sensors = True
        # Load default temperature sensors
        pconfig = self.printer.lookup_object("configfile")
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, "temperature_sensors.cfg")
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            raise config.config_error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(""):
            self.printer.load_object(dconfig, c.get_name())

    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory

    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Setup 1st sensor
        sensor = self.setup_sensor(config)
        # Setup 2nd sensor
        secondary_sensor_name = config.get("secondary_sensor_name", None)
        if secondary_sensor_name is not None:
            full_name = "temperature_sensor " + secondary_sensor_name
            secondary_sensor = self.printer.lookup_object(full_name)
        else:
            secondary_sensor = None
        # Create heater
        heater = Heater(config, sensor, secondary_sensor)
        self.heaters[heater_name] = heater
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater

    def get_all_heaters(self):
        return self.available_heaters

    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,)
            )
        return self.heaters[heater_name]

    def setup_sensor(self, config):
        if not self.have_load_sensors:
            self.load_config(config)
        sensor_type = config.get("sensor_type")
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,)
            )
        if sensor_type == "NTC 100K beta 3950":
            config.deprecate("sensor_type", "NTC 100K beta 3950")
        return self.sensor_factories[sensor_type](config)

    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get("gcode_id", None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,)
            )
        self.gcode_id_to_sensor[gcode_id] = psensor

    def register_monitor(self, config):
        self.available_monitors.append(config.get_name())

    def get_status(self, eventtime):
        return {
            "available_heaters": self.available_heaters,
            "available_sensors": self.available_sensors,
            "available_monitors": self.available_monitors,
        }

    def turn_off_all_heaters(self, print_time=0.0):
        for heater in self.heaters.values():
            heater.set_temp(0.0)

    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"

    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()

    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True

    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)

    def cmd_M105(self, gcmd):
        # Get Extruder Temperature
        reactor = self.printer.get_reactor()
        msg = self._get_temp(reactor.monotonic())
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)

    def _wait_for_temperature(self, heater):
        # Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get("debugoutput") is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.0)

    def set_temperature(self, heater, temp, wait=False):
        toolhead = self.printer.lookup_object("toolhead")
        toolhead.register_lookahead_callback((lambda pt: None))
        heater.set_temp(temp)
        if wait and temp:
            self._wait_for_temperature(heater)

    cmd_TEMPERATURE_WAIT_help = "Wait for a temperature on a sensor"

    def cmd_TEMPERATURE_WAIT(self, gcmd):
        sensor_name = gcmd.get("SENSOR")
        if sensor_name not in self.available_sensors:
            raise gcmd.error("Unknown sensor '%s'" % (sensor_name,))
        min_temp = gcmd.get_float("MINIMUM", float("-inf"))
        max_temp = gcmd.get_float("MAXIMUM", float("inf"), above=min_temp)
        if min_temp == float("-inf") and max_temp == float("inf"):
            raise gcmd.error(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM."
            )
        if self.printer.get_start_args().get("debugoutput") is not None:
            return
        if sensor_name in self.heaters:
            sensor = self.heaters[sensor_name]
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            temp, target = sensor.get_temp(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.0)


def load_config(config):
    return PrinterHeaters(config)

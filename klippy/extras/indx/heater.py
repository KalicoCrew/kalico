import logging
import math
import struct
from collections import namedtuple
from time import strftime

from ..thermistor import CustomThermistor
from . import calibration, compat
from .compat import ConfigWrapper
from .heatsink_fan import IndxHeatsinkFan

KELVIN_OFFSET = 273.15

# Mirrors the MCU coil-tuner enums (coil_driver.c++ tune_status / tune_error).
COIL_TUNE_STATUS = {0: "idle", 1: "running", 2: "success", 3: "failed"}
COIL_TUNE_STATUS_SUCCESS = 2
COIL_TUNE_STATUS_FAILED = 3
COIL_TUNE_ERROR = {
    0: "none",
    1: "overvoltage triggered during the tune (target removed?)",
    2: "ON-time tuning hit maximum time without reaching the target peak",
    3: "the resonance never returned to zero within the sweep window",
    4: "steady ON hit the ceiling without reaching the target",
    5: "timed out",
}
COIL_TUNE_STATUS_RUNNING = 1
COIL_TUNE_TIMEOUT = 15.0  # seconds. The MCU self-aborts a stuck tune at 10 s
COIL_TUNE_POLL_INTERVAL = 0.2  # seconds between latched-status polls
COIL_TUNE_WAVE_PATH = "/tmp/indx_coil_waveform.csv"
FILAMENT_LOAD_THRESHOLD = 10.0
FILAMENT_LOAD_PRIME_TIME = 5.0
FILAMENT_LOAD_MAX_LENGTH = 120.0
FILAMENT_LOAD_SPEED = 10.0
FILAMENT_LOAD_SEGMENT_TIME = 1.0


NozzleTemperature = namedtuple(
    "NozzleTemperature",
    [
        "time",
        "nozzle_temperature",
        "sensor_temperature",
        "delta_time",
        "delta_energy",
        "pwm_avg",
        "delta_pwm_energy",
    ],
)


def float_to_u32(val):
    return int(struct.unpack("!i", struct.pack("!f", val))[0])


class IndxThermistorWrapper:
    def __init__(
        self, toolboard, sensor, config, def_max_temp, thermistor_config
    ):
        self.toolboard = toolboard
        self.temperature_callbacks = []

        mcu_name = self.toolboard.mcu.get_name()
        config_thermistor = ConfigWrapper(
            self.toolboard.printer,
            f"indx_{sensor}_thermistor",
            thermistor_config,
        )

        config_sensor = ConfigWrapper(
            self.toolboard.printer,
            f"temperature_sensor {self.toolboard.name}_{sensor}",
            {
                "sensor_pin": f"{mcu_name}:temp_{sensor}",
                "pullup_resistor": 3900,
            },
        )
        self.sensor = CustomThermistor(config_thermistor).create(config_sensor)
        self.max_temp = config.getfloat(
            f"max_temp_{sensor}", minval=0.0, default=def_max_temp
        )
        self.sensor.setup_minmax(-KELVIN_OFFSET, self.max_temp)
        self.sensor.setup_callback(self.callback)

        self.last_temp = None

    def get_last(self):
        return self.last_temp

    def add_callback(self, callback):
        self.temperature_callbacks.append(callback)

    def callback(self, read_time, value):
        self.last_temp = value
        for cb in self.temperature_callbacks:
            cb(read_time, value)


class IndxBracketTempSensor:
    def __init__(self, toolboard, heater, sensor):
        self.toolboard = toolboard
        self.heater = heater

        sensor.add_callback(self.callback)

        self.last_temperature = None
        self.toolboard.mcu.register_config_callback(self.build_config)
        self.force_temp = None

        gcode = self.toolboard.printer.lookup_object("gcode")
        gcode.register_command(
            "INDX_FORCE_BRACKET_TEMP", self.cmd_FORCE_BRACKET_TEMP
        )

    def build_config(self):
        self.cmd_set_ambient_temp = self.toolboard.mcu.lookup_command(
            "indx_set_bracket_temp temp=%u", cq=self.heater.cmd_queue
        )

    def callback(self, read_time, value):
        if value < 0.0:
            value = None
        self.last_temperature = value
        if self.force_temp is not None:
            value = self.force_temp
        if value is None:
            return
        self.cmd_set_ambient_temp.send([float_to_u32(value + KELVIN_OFFSET)])

    def cmd_FORCE_BRACKET_TEMP(self, gcmd):
        self.force_temp = gcmd.get_float("TEMP", None)


class IndxToolboardHeater:
    def __init__(self, toolboard, config):
        self.toolboard = toolboard
        self.toolhead = None
        self.heater = None
        self.fan = IndxHeatsinkFan(toolboard)
        self.aux_temp_sensors = {
            "bracket": IndxThermistorWrapper(
                toolboard,
                "bracket",
                config,
                130.0,
                {
                    # Source:
                    # https://amphenol-sensors.com/hubfs/Documents/AAS-913-318C-Temperature-resistance-curves-071816-web.pdf
                    # Material Type 1, products NK
                    "temperature1": 25.0,
                    "resistance1": 10000.0,
                    "temperature2": 65.0,
                    "resistance2": 2080.0,
                    "temperature3": 100.0,
                    "resistance3": 678.0,
                },
            ),
            "board": IndxThermistorWrapper(
                toolboard,
                "board",
                config,
                100.0,
                {
                    # 10K Murata NCU15XH103J6SRC
                    "beta": 3425.0,
                    "temperature1": 25.0,
                    "resistance1": 10000.0,
                },
            ),
        }
        self.bracket_temp_update = IndxBracketTempSensor(
            self.toolboard, self, self.aux_temp_sensors["bracket"]
        )

        self.vin_adc = toolboard.mcu.setup_pin("adc", {"pin": "vin_mon"})
        compat.register_adc_callback(self.vin_adc, self.handle_vin_mon)
        query_adc = toolboard.printer.load_object(config, "query_adc")
        query_adc.register_adc(
            f"{toolboard.mcu.get_name()}_vin_mon", self.vin_adc
        )
        self.supply_voltage = 24.0

        # The autotuned thermal-model parameters have no defaults.
        # The user must run INDX_CALIBRATE to generate them.
        model_params = ThermalModelParams(
            max_power=config.getfloat("max_power", default=None, above=0.0),
            max_power_temp_coeff=config.getfloat(
                "model_max_power_temp_coeff", default=None
            ),
            thermal_capacity=config.getfloat(
                "model_thermal_capacity", default=None, above=0.0
            ),
            to_ambient_r=config.getfloat(
                "model_to_ambient_r", default=None, above=0.0
            ),
            filament_radius=config.getfloat(
                "model_filament_diameter",
                default=1.75,
            )
            / 2.0,
            filament_density=config.getfloat(
                "model_filament_density", default=1.20
            ),
            filament_heat_capacity=config.getfloat(
                "model_filament_heat_capacity", default=1.8
            ),
            part_cooling_fan_a=config.getfloat(
                "model_part_cooling_fan_a", default=0.0
            ),
            part_cooling_fan_k=config.getfloat(
                "model_part_cooling_fan_k", default=0.0
            ),
            ambient_blend_board=config.getfloat(
                "model_ambient_blend_board", default=0.0
            ),
            ambient_blend_bracket=config.getfloat(
                "model_ambient_blend_bracket", default=1.0
            ),
            ambient_blend_sensor=config.getfloat(
                "model_ambient_blend_sensor", default=1.0
            ),
            error_application=config.getfloat(
                "model_error_application", default=1.0
            ),
        )
        self.thermal_model = ThermalModel(model_params)
        self.max_model_error = config.getfloat("max_model_error", default=50.0)

        self.pid_kp = config.getfloat("pid_kp", default=4.0)
        self.pid_ti = config.getfloat("pid_ti", minval=0.0, default=0.0)
        self.pid_td = config.getfloat("pid_td", minval=0.0, default=0.0)
        self.pid_b = config.getfloat(
            "pid_b", minval=0.0, maxval=1.0, default=1.0
        )
        self.max_temp_nozzle = config.getfloat("max_temp_nozzle", default=305.0)
        self.max_temp_sensor = config.getfloat("max_temp_sensor", default=130.0)

        self.ir_sensor_tuning = None
        ir_sensor_exponent = config.getfloat("ir_sensor_exponent", default=None)
        ir_sensor_obj_gain = config.getfloat("ir_sensor_obj_gain", default=None)
        ir_sensor_bracket_gain = config.getfloat(
            "ir_sensor_bracket_gain", default=None
        )
        tuning = (
            ir_sensor_exponent,
            ir_sensor_obj_gain,
            ir_sensor_bracket_gain,
        )
        if any(v is not None for v in tuning):
            if not all(v is not None for v in tuning):
                raise config.error(
                    "All of ir_sensor_{exponent,obj_gain,bracket_gain} must be given if one is"
                )
            self.ir_sensor_tuning = tuning

        # Coil-drive timings (microseconds) to apply to the MCU at connect. When not set,
        # heating can't be done, but we allow not setting them so the user can run the
        # tuning routine. These are specified in microseconds by the user.
        coil_timings = (
            config.getfloat(
                "coil_time_on", default=None, above=0.0, below=10.0
            ),
            config.getfloat(
                "coil_time_off", default=None, above=0.0, below=10.0
            ),
            config.getfloat(
                "coil_time_on_first", default=None, above=0.0, below=10.0
            ),
        )
        self.coil_timings = None
        if all(coil_timings):
            t_on, t_off, t_on_first = coil_timings
            if t_on_first > t_on:
                raise config.error(
                    "coil_time_on_first must not exceed coil_time_on"
                )
            # microseconds -> seconds for the MCU
            self.coil_timings = (t_on * 1e-6, t_off * 1e-6, t_on_first * 1e-6)
        elif any(coil_timings):
            raise config.error(
                "coil_time_on, coil_time_off, coil_time_on_first must be set"
                " together"
            )

        fan_name = config.get("part_cooling_fan", default="fan")
        self.part_cooling_fan = None
        if fan_name:
            fan_obj = toolboard.printer.load_object(config, fan_name, None)
            if fan_obj is None:
                fan_obj = toolboard.printer.lookup_object(fan_name)
            if fan_obj is None:
                raise config.error(
                    f"Unknown cooling_fan '{fan_name}' specified"
                )
            if not hasattr(fan_obj, "fan") or not hasattr(
                fan_obj.fan, "set_speed"
            ):
                raise config.error(
                    f"cooling_fan '{fan_name}' is not a valid fan object"
                )
            self.part_cooling_fan = fan_obj.fan

        self.toolboard.mcu.register_config_callback(self.build_config)
        self.temperature_callbacks = []
        self.cmd_queue = self.toolboard.mcu.alloc_command_queue()

        self.toolboard.printer.register_event_handler(
            "klippy:connect", self.handle_connect
        )

        self.last_report = None
        self.power_scale = 0
        self.heaters = []

        gcode = self.toolboard.printer.lookup_object("gcode")
        gcode.register_command("INDX_SET_PID", self.cmd_SET_PID)
        gcode.register_command("INDX_SET_CYCLE_LIMIT", self.cmd_SET_CYCLE_LIMIT)
        gcode.register_command(
            "INDX_SET_IR_SENSOR_PARAMS", self.cmd_SET_IR_SENSOR_PARAMS
        )
        gcode.register_command("INDX_CALIBRATE", self.cmd_CALIBRATE)
        gcode.register_command("INDX_FAN_CALIBRATE", self.cmd_FAN_CALIBRATE)
        gcode.register_command("INDX_MEASURE_POWER", self.cmd_MEASURE_POWER)
        gcode.register_command("INDX_LOAD_FILAMENT", self.cmd_LOAD_FILAMENT)
        gcode.register_command("INDX_CLEAR_FILAMENT", self.cmd_CLEAR_FILAMENT)
        gcode.register_command("INDX_EXTRUDER_MOVE", self.cmd_EXTRUDER_MOVE)
        gcode.register_command(
            "INDX_SET_MODEL_PARAMS", self.cmd_SET_MODEL_PARAMS
        )
        gcode.register_command(
            "INDX_DUMP_MODEL_UPDATE", self.cmd_DUMP_MODEL_UPDATE
        )

        gcode.register_command("INDX_LOG", self.cmd_LOG)
        self.log_file = None

        gcode.register_command(
            "INDX_DEBUG_IR_SENSOR_EEPROM",
            self.cmd_DEBUG_IR_SENSOR_EEPROM,
        )
        gcode.register_command(
            "INDX_DEBUG_STREAM_RAW_IR_SENSOR",
            self.cmd_DEBUG_STREAM_RAW_IR_SENSOR,
        )
        self.raw_ir_log_file = None
        self.raw_ir_log_sensors = []

    def build_config(self):
        mcu = self.toolboard.mcu

        compat.register_response(
            mcu,
            self.handle_report_nozzle_temp,
            "indx_nozzle_temp clock=%u nozzle=%u sensor=%u power=%u charge=%u overvoltage=%u",
        )

        compat.register_response(
            mcu,
            self.handle_debug_raw_ir,
            "indx_debug_raw_ir object=%u ambient=%u",
        )
        self.cmd_debug_stream_raw_ir = mcu.lookup_command(
            "indx_debug_stream_raw_ir_sensor enable=%c", cq=self.cmd_queue
        )

        self.tune_coil_cmd = mcu.lookup_command(
            "indx_tune_coil debug=%c", cq=self.cmd_queue
        )

        self.query_coil_tune_cmd = mcu.lookup_query_command(
            "indx_query_coil_tune",
            "indx_coil_tune_result status=%c error=%c on_first=%u off=%u on=%u",
            cq=self.cmd_queue,
        )
        amp_per_count = mcu.get_constant_float(
            "INDX_CURRENT_SENSE_AMP_PER_COUNT"
        )
        current_sense_rate = mcu.get_constant_float("INDX_CURRENT_SENSE_RATE")
        self.power_scale = amp_per_count / current_sense_rate

        mcu.add_config_cmd(
            f"indx_set_control_params kp={float_to_u32(self.pid_kp)} ti={float_to_u32(self.pid_ti)} td={float_to_u32(self.pid_td)} b={float_to_u32(self.pid_b)}"
        )
        self.cmd_set_control_params = mcu.lookup_command(
            "indx_set_control_params kp=%u ti=%u td=%u b=%u",
            cq=self.cmd_queue,
        )
        self.cmd_set_target = mcu.lookup_command(
            "indx_set_target target=%u", cq=self.cmd_queue
        )

        if self.ir_sensor_tuning is not None:
            (exponent, obj_gain, bracket_gain) = self.ir_sensor_tuning
            mcu.add_config_cmd(
                f"indx_set_ir_sensor_params exponent={float_to_u32(exponent)} obj_gain={float_to_u32(obj_gain)} bracket_gain={float_to_u32(bracket_gain)}"
            )

        if self.coil_timings is not None:
            (t_on, t_off, t_on_first) = self.coil_timings
            mcu.add_config_cmd(
                f"indx_set_coil_driver_params time_on={float_to_u32(t_on)}"
                f" time_off={float_to_u32(t_off)}"
                f" time_on_first={float_to_u32(t_on_first)}"
            )

    def handle_connect(self):
        # Look for heaters that have us as their PWM output
        pheaters = self.toolboard.printer.lookup_object("heaters")
        for heater_name in pheaters.get_all_heaters():
            heater = pheaters.lookup_heater(heater_name)
            if heater.mcu_pwm != self:
                continue
            self.heater = heater
            control = HeaterControlIndx(heater, self)
            heater.set_control(control)

            # Refuse to heat until the coil driver and thermal model are tuned
            # (uncalibrated coil timings also make the MCU shut down). Monkey-
            # patch set_temp so we can give a clear command error to the user.
            base_set_temp = heater.set_temp

            def guarded_set_temp(degrees):
                if degrees and not self._heater_config_valid():
                    raise self.toolboard.printer.command_error(
                        "INDX heater has not been calibrated yet. "
                        "Run INDX_CALIBRATE to perform calibration."
                    )
                base_set_temp(degrees)

            heater.set_temp = guarded_set_temp
            self.heater_raw_set_temp = base_set_temp

            self.fan.add_heater(heater)
            self.heaters.append(heater)

            # Support for Kalico profile system
            if hasattr(heater, "pmgr"):
                control.profile = heater.pmgr.init_default_profile()

            break

    def apply_pid_params(self):
        self.cmd_set_control_params.send(
            [
                float_to_u32(self.pid_kp),
                float_to_u32(self.pid_ti),
                float_to_u32(self.pid_td),
                float_to_u32(self.pid_b),
            ]
        )

    def get_max_power(self, temp=None):
        params = self.thermal_model.params
        base = params.max_power
        if not base:
            # If the model isn't loaded, just return 0 for now - the user won't
            # be allowed to heat regardless.
            return 0.0
        if temp is None:
            return base
        # Correct for decrease in delivered power as the nozzle heats.
        coeff = params.max_power_temp_coeff or 0.0
        adjusted = base + coeff * (temp - calibration.MODEL_MAX_POWER_REF_TEMP)
        return max(0.0, adjusted)

    def _heater_config_valid(self):
        # Heating is allowed only once with fully populated coil and thermal models.
        return self.coil_timings is not None and self.thermal_model.is_valid()

    def cur_target_temp(self):
        time = self.toolboard.printer.get_reactor().monotonic()
        return next(
            (
                h.get_temp(time)[1]
                for h in self.heaters
                if h.get_temp(time)[1] != 0.0
            ),
            None,
        )

    def extruder_position(self, time):
        if self.toolhead is None:
            self.toolhead = self.toolboard.printer.lookup_object("toolhead")
        if self.toolhead is None:
            return 0.0
        extruder = self.toolhead.get_extruder()
        if extruder.get_heater() != self.heater:
            return 0.0
        if not hasattr(extruder, "find_past_position"):
            return 0.0
        return extruder.find_past_position(time)

    def fan_speed(self, time):
        if self.part_cooling_fan is None:
            return 0.0
        return max(
            0.0, min(1.0, self.part_cooling_fan.get_status(time)["speed"])
        )

    def _blend_ambient_temp(self, sensor_temp):
        # Weighted blend of the board/bracket thermistors and the IR sensor's
        # ambient reading, using the model's ambient_blend_* weights. This is
        # the same ambient the model is fed at runtime, reused by calibration.
        params = self.thermal_model.params
        components = [
            (
                self.aux_temp_sensors["board"].get_last() or 25.0,
                params.ambient_blend_board,
            ),
            (
                self.aux_temp_sensors["bracket"].get_last() or 25.0,
                params.ambient_blend_bracket,
            ),
            (sensor_temp, params.ambient_blend_sensor),
        ]
        w_total = sum(w for (_, w) in components)
        if w_total == 0.0:
            return 25.0
        return sum(t * w for (t, w) in components) / w_total

    def handle_report_nozzle_temp(self, params):
        clock = self.toolboard.mcu.clock32_to_clock64(params["clock"])
        time = self.toolboard.mcu.clock_to_print_time(clock)
        nozzle_temp = params["nozzle"] / 100.0 - KELVIN_OFFSET
        sensor_temp = params["sensor"] / 100.0 - KELVIN_OFFSET
        extruder_pos = self.extruder_position(time)

        current_target_temp = self.cur_target_temp()
        is_active = current_target_temp is not None

        power = params["power"] / 1000000.0
        if not is_active:
            self.thermal_model.force_temp(nozzle_temp)

        charge = params["charge"]
        if self.last_report is not None:
            delta_time = time - self.last_report[0]
            delta_charge = charge - self.last_report[1]
            if delta_charge < 0:
                delta_charge += 2**32
            delta_charge *= self.power_scale
            delta_energy = delta_charge * self.supply_voltage
            input_power = delta_energy / delta_time

            if self.log_file is not None:
                try:
                    self.log_file.write(
                        f"{time},{nozzle_temp},{self.thermal_model.temperature},{delta_energy},{input_power}\n"
                    )
                except:
                    pass

            if is_active:
                is_cooling = nozzle_temp > (current_target_temp + 5)
                error = nozzle_temp - self.thermal_model.temperature
                force_adjust = max(
                    0.0, error * self.thermal_model.params.error_application
                )
                if is_cooling or not self.thermal_model.is_valid():
                    force_adjust = error

                if force_adjust:
                    self.thermal_model.force_temp(
                        self.thermal_model.temperature + force_adjust
                    )
                # When thermal model isn't valid, we won't run it. This is okay,
                # because the user isn't allowed to start heating when no thermal
                # model is set. We need this check to allow the tuner to work.
                if self.thermal_model.is_valid():
                    filament_distance = max(
                        0.0, extruder_pos - self.last_report[3]
                    )
                    fan_speed = self.fan_speed(time)

                    ambient_temp = self._blend_ambient_temp(sensor_temp)
                    filament_temp = ambient_temp  # TODO

                    input_power = power * self.get_max_power(nozzle_temp)
                    self.thermal_model.step(
                        delta_time,
                        input_power,
                        ambient_temp,
                        filament_temp,
                        filament_distance,
                        fan_speed,
                    )
                    error = self.thermal_model.temperature - nozzle_temp
                    if error > self.max_model_error:
                        self.toolboard.printer.invoke_shutdown(
                            f"Modelled nozzle temperature {self.thermal_model.temperature:.1f} "
                            f"exceeds measured temperature of {nozzle_temp:.1f} by more than "
                            f"{self.max_model_error:.1f} deg C",
                        )

        else:
            delta_time = None
            delta_energy = None

        self.last_report = (time, charge, power, extruder_pos)
        if nozzle_temp >= self.max_temp_nozzle:
            self.toolboard.printer.invoke_shutdown(
                f"Maximum nozzle temperature of {self.max_temp_nozzle} exceeded, saw {nozzle_temp}"
            )
        elif sensor_temp >= self.max_temp_sensor:
            self.toolboard.printer.invoke_shutdown(
                f"Maximum IR sensor temperature of {self.max_temp_sensor} exceeded, saw {sensor_temp}"
            )
        for cb in self.temperature_callbacks:
            cb(
                NozzleTemperature(
                    time=time,
                    nozzle_temperature=nozzle_temp,
                    sensor_temperature=sensor_temp,
                    delta_time=delta_time,
                    delta_energy=delta_energy,
                    pwm_avg=power,
                    delta_pwm_energy=power
                    * self.get_max_power(nozzle_temp)
                    * (delta_time or 0.0),
                )
            )

    def handle_vin_mon(self, _read_time, read_value):
        vin = read_value * 3.3 * (4700 + 60400) / 4700
        self.supply_voltage = vin

    def set_target_temp(self, target_temp):
        if target_temp == 0.0:
            target_temp = float("nan")
        self.cmd_set_target.send([float_to_u32(target_temp + KELVIN_OFFSET)])

    def cmd_SET_PID(self, gcmd):
        self.pid_kp = gcmd.get_float("KP", self.pid_kp)
        self.pid_ti = gcmd.get_float("TI", self.pid_ti, minval=0.0)
        self.pid_td = gcmd.get_float("TD", self.pid_td, minval=0.0)
        self.pid_b = gcmd.get_float("B", self.pid_b, minval=0.0)
        self.apply_pid_params()

    def cmd_SET_CYCLE_LIMIT(self, gcmd):
        cmd = self.toolboard.mcu.lookup_command(
            "indx_cycle_limit limit=%u", cq=self.cmd_queue
        )
        cmd.send([gcmd.get_int("LIMIT")])

    def cmd_SET_IR_SENSOR_PARAMS(self, gcmd):
        cmd = self.toolboard.mcu.lookup_command(
            "indx_set_ir_sensor_params exponent=%u obj_gain=%u bracket_gain=%u",
            cq=self.cmd_queue,
        )
        exponent = gcmd.get_float("EXPONENT")
        obj_gain = gcmd.get_float("OBJ_GAIN")
        bracket_gain = gcmd.get_float("BRACKET_GAIN")
        cmd.send(
            [
                float_to_u32(exponent),
                float_to_u32(obj_gain),
                float_to_u32(bracket_gain),
            ]
        )

    def cmd_CALIBRATE(self, gcmd):
        if self.cur_target_temp() is not None:
            raise gcmd.error(
                "Cannot calibrate while the heater is active. "
                "Turn the heater off first."
            )
        self.thermal_model.params = self.thermal_model.params._replace(
            max_power=None,
            max_power_temp_coeff=None,
            thermal_capacity=None,
            to_ambient_r=None,
        )
        self._calibrate_coil_driver(gcmd)
        self._calibrate_thermal_model(gcmd)
        gcmd.respond_info(
            "INDX calibration complete. Run SAVE_CONFIG to save the new "
            "parameters and restart."
        )

    def _calibrate_coil_driver(self, gcmd):
        mcu = self.toolboard.mcu
        capture = gcmd.get("SAVE_WAVEFORM", None) is not None

        # Tune at a consistent (cold) temperature: wait until the nozzle is
        # below the same MIN_TEMP used for thermal-model calibration, so the
        # coil timings are always measured at (almost) the same temperature.
        min_temp = gcmd.get_float(
            "MIN_TEMP",
            calibration.MODEL_CAL_MIN_TEMP_DEFAULT,
            above=0.0,
            below=calibration.MODEL_CAL_MIN_TEMP_CEILING,
        )
        self._wait_cooldown(gcmd, min_temp)

        gcmd.respond_info("INDX: tuning coil driver, please wait...")

        # Consume both tune streams so they don't log as unhandled; we poll the
        # latched status, so the notification is a no-op.
        samples = []
        reactor = self.toolboard.printer.get_reactor()
        finished = compat.register_response(
            mcu, lambda p: None, "indx_coil_tune_finished status=%c error=%c"
        )
        wave = compat.register_response(
            mcu,
            lambda p: samples.append((p["index"], p["ns"], p["mv"])),
            "indx_coil_tune_wave index=%u ns=%u mv=%u",
        )
        try:
            self.tune_coil_cmd.send([int(capture)])
            deadline = reactor.monotonic() + COIL_TUNE_TIMEOUT
            while True:
                result = self.query_coil_tune_cmd.send([])
                if result["status"] != COIL_TUNE_STATUS_RUNNING:
                    break
                if reactor.monotonic() > deadline:
                    self.coil_timings = None
                    raise gcmd.error("INDX coil driver tuning timed out")
                reactor.pause(reactor.monotonic() + COIL_TUNE_POLL_INTERVAL)
        finally:
            wave.unregister()
            finished.unregister()
            if capture:
                with open(COIL_TUNE_WAVE_PATH, "w") as f:
                    f.write("index,ns,mv\n")
                    for index, ns, mv in samples:
                        f.write("%d,%d,%d\n" % (index, ns, mv))
                gcmd.respond_info(
                    "INDX coil tune waveform (%d samples) saved to %s"
                    % (len(samples), COIL_TUNE_WAVE_PATH)
                )

        status = result["status"]
        if status != COIL_TUNE_STATUS_SUCCESS:
            # MCU no longer holds a valid model; re-gate host heating.
            self.coil_timings = None
            if status == COIL_TUNE_STATUS_FAILED:
                reason = COIL_TUNE_ERROR.get(result["error"], result["error"])
            else:
                reason = COIL_TUNE_STATUS.get(status, status)
            raise gcmd.error("INDX coil driver tuning failed: %s" % reason)

        # Cache in seconds (opens the heating gate for this session); ns -> s.
        self.coil_timings = (
            result["on"] * 1e-9,
            result["off"] * 1e-9,
            result["on_first"] * 1e-9,
        )
        self._save_coil_config()
        gcmd.respond_info(
            "INDX coil driver tuned: TIME_ON=%.3f us TIME_OFF=%.3f us "
            "TIME_ON_FIRST=%.3f us"
            % (
                result["on"] / 1000.0,
                result["off"] / 1000.0,
                result["on_first"] / 1000.0,
            )
        )

    def _save_coil_config(self):
        configfile = self.toolboard.printer.lookup_object("configfile")
        section = self.toolboard.name
        (t_on, t_off, t_on_first) = self.coil_timings
        configfile.set(section, "coil_time_on", "%.3f" % (t_on * 1e6))
        configfile.set(section, "coil_time_off", "%.3f" % (t_off * 1e6))
        configfile.set(
            section, "coil_time_on_first", "%.3f" % (t_on_first * 1e6)
        )

    def _toggle_steppers(self, toolhead, ets, enable):
        # Enable/disable a set of EnableTracking lines with proper toolhead
        # sync (mirrors stepper_enable's motor_debug_enable; portable both forks).
        if not ets:
            return
        toolhead.dwell(calibration.MODEL_CAL_STEPPER_DWELL)
        print_time = toolhead.get_last_move_time()
        for et in ets:
            if enable:
                et.motor_enable(print_time)
            else:
                et.motor_disable(print_time)
        toolhead.dwell(calibration.MODEL_CAL_STEPPER_DWELL)

    def _model_cal_wait_temp(
        self, reactor, gcmd, predicate, last_temp, timeout, timeout_msg
    ):
        deadline = reactor.monotonic() + timeout
        while True:
            t = last_temp[0]
            if t is not None and predicate(t):
                return
            if reactor.monotonic() > deadline:
                raise gcmd.error(timeout_msg)
            reactor.pause(
                reactor.monotonic() + calibration.MODEL_CAL_POLL_INTERVAL
            )

    def _wait_cooldown(self, gcmd, min_temp):
        # Block until the nozzle has cooled below min_temp (the heater must
        # already be off) so the following operation runs at a consistent,
        # repeatable temperature.
        reactor = self.toolboard.printer.get_reactor()
        last_temp = [None]

        def temp_cb(reading):
            last_temp[0] = reading.nozzle_temperature

        if self.heater is not None:
            self.heater.set_temp(0.0)
        self.temperature_callbacks.append(temp_cb)
        try:
            self._model_cal_wait_temp(
                reactor,
                gcmd,
                lambda t: True,
                last_temp,
                calibration.MODEL_CAL_FIRST_READ_TIMEOUT,
                "INDX: no temperature reports received",
            )
            if last_temp[0] > min_temp:
                gcmd.respond_info(
                    "INDX: cooling below %.0f C before tuning" % min_temp
                )
                self._model_cal_wait_temp(
                    reactor,
                    gcmd,
                    lambda t: t < min_temp,
                    last_temp,
                    calibration.MODEL_CAL_PRECOOL_TIMEOUT,
                    "INDX: timed out cooling to MIN_TEMP",
                )
        finally:
            self.temperature_callbacks.remove(temp_cb)

    def _calibrate_thermal_model(self, gcmd):
        if self.heater is None:
            raise gcmd.error("INDX heater is not bound to an extruder/heater")
        if self.coil_timings is None:
            raise gcmd.error(
                "INDX heater has not been calibrated yet. "
                "Run INDX_CALIBRATE to perform calibration."
            )
        if self.cur_target_temp() is not None:
            raise gcmd.error(
                "Cannot calibrate while the heater is active. "
                "Turn the heater off first."
            )

        min_temp = gcmd.get_float(
            "MIN_TEMP",
            calibration.MODEL_CAL_MIN_TEMP_DEFAULT,
            above=0.0,
            below=calibration.MODEL_CAL_MIN_TEMP_CEILING,
        )
        heater_max = getattr(self.heater, "max_temp", self.max_temp_nozzle)
        max_temp = gcmd.get_float(
            "MAX_TEMP",
            calibration.MODEL_CAL_MAX_TEMP_DEFAULT,
            minval=min_temp + 1.0,
            maxval=heater_max,
        )
        hold_time = gcmd.get_float(
            "HOLD_TIME", calibration.MODEL_CAL_HOLD_TIME_DEFAULT, minval=0.0
        )
        cooldown_time = gcmd.get_float(
            "COOLDOWN_TIME",
            calibration.MODEL_CAL_COOLDOWN_TIME_DEFAULT,
            minval=0.0,
            maxval=calibration.MODEL_CAL_COOLDOWN_TIME_MAX,
        )

        printer = self.toolboard.printer
        reactor = printer.get_reactor()
        toolhead = printer.lookup_object("toolhead")

        samples = []
        phase = ["precool"]
        last_temp = [None]

        def sampler(reading):
            last_temp[0] = reading.nozzle_temperature
            if reading.delta_time is None or reading.delta_time <= 0.0:
                return
            samples.append(
                (
                    reading.nozzle_temperature,
                    reading.delta_energy / reading.delta_time,
                    reading.pwm_avg,
                    reading.delta_time,
                    self._blend_ambient_temp(reading.sensor_temperature),
                    phase[0],
                )
            )

        old_max_model_error = self.max_model_error
        reenable = []
        installed = False
        try:
            # Don't let the un-calibrated production model trip the
            # model-vs-measured divergence guard during the heatup. The hard
            # over-temperature interlocks stay active.
            self.max_model_error = float("inf")

            # Heater off and disable the toolboard steppers (extruder motor) so
            # they can't perturb the power/baseline measurement. Reuse the
            # enable lines the heatsink fan already collected for this MCU.
            self.heater.set_temp(0.0)
            reenable = [et for et in self.fan.steppers if et.is_motor_enabled()]
            self._toggle_steppers(toolhead, reenable, False)

            self.temperature_callbacks.append(sampler)
            installed = True

            self._model_cal_wait_temp(
                reactor,
                gcmd,
                lambda t: True,
                last_temp,
                calibration.MODEL_CAL_FIRST_READ_TIMEOUT,
                "INDX: no temperature reports received",
            )
            if last_temp[0] > min_temp:
                gcmd.respond_info(
                    "INDX: cooling below %.0f C before calibration" % min_temp
                )
                self._model_cal_wait_temp(
                    reactor,
                    gcmd,
                    lambda t: t < min_temp,
                    last_temp,
                    calibration.MODEL_CAL_PRECOOL_TIMEOUT,
                    "INDX: timed out cooling to MIN_TEMP",
                )

            # Baseline (heater off) draw to subtract from measured power.
            phase[0] = "baseline"
            gcmd.respond_info("INDX: measuring baseline power")
            reactor.pause(
                reactor.monotonic() + calibration.MODEL_CAL_BASELINE_TIME
            )
            baseline_power = calibration.mean_phase_power(samples, "baseline")
            if baseline_power is None:
                raise gcmd.error("INDX: no baseline power samples received")

            # Heatup to MAX_TEMP.
            phase[0] = "heat"
            gcmd.respond_info(
                "INDX: heating to %.0f C (baseline %.2f W)"
                % (max_temp, baseline_power)
            )
            self.heater_raw_set_temp(max_temp)
            self._model_cal_wait_temp(
                reactor,
                gcmd,
                lambda t: t >= max_temp,
                last_temp,
                calibration.MODEL_CAL_HEATUP_TIMEOUT,
                "INDX: timed out heating to MAX_TEMP",
            )

            # Hold at temperature (strong constraint on to_ambient_r).
            if hold_time > 0.0:
                phase[0] = "hold"
                gcmd.respond_info("INDX: holding for %.0f s" % hold_time)
                reactor.pause(reactor.monotonic() + hold_time)

            # Cooldown (constrains R*C).
            self.heater.set_temp(0.0)
            if cooldown_time > 0.0:
                phase[0] = "cooldown"
                gcmd.respond_info(
                    "INDX: measuring cooldown for %.0f s" % cooldown_time
                )
                reactor.pause(reactor.monotonic() + cooldown_time)

            self.temperature_callbacks.remove(sampler)
            installed = False
            self.heater.set_temp(0.0)

            gcmd.respond_info(
                "INDX: collected %d samples, fitting model..." % len(samples)
            )
            result = calibration.run_fit_in_background(
                printer,
                calibration.thermal_model_fit,
                samples,
                baseline_power,
            )
        finally:
            if installed:
                try:
                    self.temperature_callbacks.remove(sampler)
                except ValueError:
                    pass
            self.heater.set_temp(0.0)
            self.max_model_error = old_max_model_error
            self._toggle_steppers(toolhead, reenable, True)

        if result.get("error"):
            raise gcmd.error(
                "INDX thermal model fit failed: %s" % result["error"]
            )
        self._apply_model_calibration(gcmd, result)

    def _apply_model_calibration(self, gcmd, result):
        max_power = result["max_power"]
        coeff = result["max_power_temp_coeff"]
        ref = result["max_power_ref_temp"]
        capacity = result["thermal_capacity"]
        to_ambient_r = result["to_ambient_r"]

        self.thermal_model.params = self.thermal_model.params._replace(
            max_power=max_power,
            max_power_temp_coeff=coeff,
            thermal_capacity=capacity,
            to_ambient_r=to_ambient_r,
        )
        configfile = self.toolboard.printer.lookup_object("configfile")
        section = self.toolboard.name
        configfile.set(section, "max_power", "%.3f" % max_power)
        configfile.set(section, "model_max_power_temp_coeff", "%.5f" % coeff)
        configfile.set(section, "model_thermal_capacity", "%.4f" % capacity)
        configfile.set(section, "model_to_ambient_r", "%.4f" % to_ambient_r)

        def fmt(val, ci, unit, spec="%.4f"):
            if ci is None:
                return (spec % val) + " " + unit
            return (spec % val) + " +/- " + (spec % ci) + " " + unit + " (95%)"

        lines = []
        mp_se = result.get("max_power_se")
        lines.append(
            "max_power: "
            + fmt(
                max_power,
                1.96 * mp_se if mp_se else None,
                "W (at %.0f C)" % ref,
                "%.2f",
            )
        )
        c_se = result.get("max_power_temp_coeff_se")
        lines.append(
            "max_power_temp_coeff: "
            + fmt(coeff, 1.96 * c_se if c_se else None, "W/K", "%.4f")
        )
        # Illustrate the power change across the measured heatup range.
        t_min = result.get("fit_t_min")
        t_max = result.get("fit_t_max")
        if t_min is not None and t_max is not None:
            p_lo = max_power + coeff * (t_min - ref)
            p_hi = max_power + coeff * (t_max - ref)
            drop_pct = (100.0 * (p_lo - p_hi) / p_lo) if p_lo else 0.0
            lines.append(
                "  power %.1f W @ %.0f C -> %.1f W @ %.0f C (%+.1f%%)"
                % (p_lo, t_min, p_hi, t_max, -drop_pct)
            )
        lines.append(
            "thermal_capacity: "
            + fmt(capacity, result["thermal_capacity_ci"], "J/K")
        )
        lines.append(
            "to_ambient_r: "
            + fmt(to_ambient_r, result["to_ambient_r_ci"], "K/W")
        )
        lines.append(
            "fit RMS error: %.2f C over %d samples"
            % (result["rms"], result["n_fit"])
        )
        gcmd.respond_info("INDX thermal model calibrated:\n" + "\n".join(lines))

    def cmd_FAN_CALIBRATE(self, gcmd):
        if self.heater is None or not self._heater_config_valid():
            raise gcmd.error("INDX thermal model is not calibrated")
        if self.part_cooling_fan is None:
            raise gcmd.error("INDX part cooling fan is not configured")

        breaks = gcmd.get_int(
            "BREAKS", calibration.FAN_CAL_BREAKS_DEFAULT, minval=3
        )
        hold_time = gcmd.get_float(
            "HOLD_TIME", calibration.FAN_CAL_HOLD_TIME_DEFAULT, above=0.0
        )
        min_speed = gcmd.get_float(
            "MIN_SPEED",
            calibration.FAN_CAL_MIN_SPEED_DEFAULT,
            above=0.0,
            below=1.0,
        )
        heater_max = getattr(self.heater, "max_temp", self.max_temp_nozzle)
        target = gcmd.get_float(
            "TEMP",
            calibration.FAN_CAL_TEMP_DEFAULT,
            above=0.0,
            maxval=heater_max,
        )
        settle_time = gcmd.get_float(
            "SETTLE_TIME",
            calibration.FAN_CAL_SETTLE_TIME_DEFAULT,
            minval=0.0,
        )
        speeds = calibration.fan_calibration_breakpoints(breaks, min_speed)
        printer = self.toolboard.printer
        reactor = printer.get_reactor()
        last_temp = [None]
        active_samples = [None]

        def sampler(reading):
            last_temp[0] = reading.nozzle_temperature
            if (
                active_samples[0] is None
                or reading.delta_time is None
                or reading.delta_time <= 0.0
            ):
                return
            active_samples[0].append(
                (
                    reading.nozzle_temperature,
                    self._blend_ambient_temp(reading.sensor_temperature),
                    reading.delta_time,
                    reading.delta_pwm_energy,
                )
            )

        old_max_model_error = self.max_model_error
        samples = []
        installed = False
        try:
            self.max_model_error = float("inf")
            self.part_cooling_fan.set_speed(0.0)
            self.temperature_callbacks.append(sampler)
            installed = True
            self.heater_raw_set_temp(target)
            gcmd.respond_info(
                "INDX: waiting for %.0f C for fan calibration" % target
            )
            self._model_cal_wait_temp(
                reactor,
                gcmd,
                lambda t: abs(t - target) <= 1.0,
                last_temp,
                calibration.FAN_CAL_HEATUP_TIMEOUT,
                "INDX: timed out reaching fan calibration temperature",
            )
            if settle_time:
                gcmd.respond_info("INDX: settling for %.0f s" % settle_time)
                reactor.pause(reactor.monotonic() + settle_time)

            for speed in speeds:
                phase_samples = []
                active_samples[0] = phase_samples
                self.part_cooling_fan.set_speed(speed)
                gcmd.respond_info(
                    "INDX: fan %.0f%% for %.1f s" % (speed * 100.0, hold_time)
                )
                reactor.pause(reactor.monotonic() + hold_time)
                active_samples[0] = None
                samples.append((speed, phase_samples))
        finally:
            active_samples[0] = None
            if installed:
                self.temperature_callbacks.remove(sampler)
            self.part_cooling_fan.set_speed(0.0)
            self.heater_raw_set_temp(0.0)
            self.max_model_error = old_max_model_error

        result = calibration.fit_fan_model(
            samples,
            self.thermal_model.params.thermal_capacity,
            self.thermal_model.params.to_ambient_r,
        )
        if result.get("error"):
            raise gcmd.error("INDX fan model fit failed: %s" % result["error"])

        a = result["part_cooling_fan_a"]
        k = result["part_cooling_fan_k"]
        self.thermal_model.params = self.thermal_model.params._replace(
            part_cooling_fan_a=a, part_cooling_fan_k=k
        )
        configfile = printer.lookup_object("configfile")
        section = self.toolboard.name
        configfile.set(section, "model_part_cooling_fan_a", "%.5f" % a)
        configfile.set(section, "model_part_cooling_fan_k", "%.5f" % k)

        def fit_value(value, ci):
            return "%.5f%s" % (
                value,
                " +/- %.5f (95%%)" % ci if ci is not None else "",
            )

        point_lines = [
            "  %.0f%%: %.2f W over %.1f s" % (speed * 100.0, power, duration)
            for speed, power, _, _, duration in result["points"]
        ]
        r_squared = result["r_squared"]
        gcmd.respond_info(
            "INDX fan model calibrated:\n"
            "part_cooling_fan_a: %s K/W\n"
            "part_cooling_fan_k: %s\n"
            "fit RMS power error: %.2f W%s\n%s\n"
            "Run SAVE_CONFIG to save the new fan parameters."
            % (
                fit_value(a, result["part_cooling_fan_a_ci"]),
                fit_value(k, result["part_cooling_fan_k_ci"]),
                result["rms_power"],
                "; R^2: %.4f" % r_squared if r_squared is not None else "",
                "\n".join(point_lines),
            )
        )

    def cmd_MEASURE_POWER(self, gcmd):
        duration = gcmd.get_float("DURATION", above=0.0, default=5.0)
        gcmd.respond_info(f"Measuring power for {duration:.1f} seconds")

        reactor = self.toolboard.printer.get_reactor()

        totals = [0, 0, 0]

        def cb(reading):
            totals[0] += reading.delta_time
            totals[1] += reading.delta_energy
            totals[2] += reading.delta_pwm_energy

        try:
            self.temperature_callbacks.append(cb)
            begin_time = reactor.monotonic()
            reactor.pause(begin_time + duration)
        finally:
            self.temperature_callbacks.remove(cb)

        if totals[0] == 0:
            raise gcmd.error(
                "No power meansurements received over entire duration"
            )

        power = totals[1] / totals[0]
        gcmd.respond_info(
            f"Measured power for {totals[0]:.1f} seconds.\n"
            f"Total energy {totals[1]:.1f} J.\n"
            f"Average power {power:.2f} W.\n"
            f"PWM modelled: {totals[2]:.1f} J / {totals[2] / totals[0]:.2f} W"
        )

    def cmd_LOAD_FILAMENT(self, gcmd):
        if not self.thermal_model.is_valid():
            raise gcmd.error("INDX thermal model is not calibrated")

        toolhead = self.toolboard.printer.lookup_object("toolhead")
        extruder = toolhead.get_extruder()
        if self.heater is None or extruder.get_heater() != self.heater:
            raise gcmd.error("Active extruder is not using the INDX heater")
        if not self.heater.can_extrude:
            raise gcmd.error(
                "INDX_LOAD_FILAMENT requires the active extruder to be above "
                "min_extrude_temp"
            )
        target_temp = self.cur_target_temp()
        if target_temp is None or target_temp < self.heater.min_extrude_temp:
            raise gcmd.error(
                "INDX_LOAD_FILAMENT requires the active extruder target "
                "temperature to be set to at least min_extrude_temp"
            )

        speed = gcmd.get_float("SPEED", FILAMENT_LOAD_SPEED, above=0.0)
        max_length = gcmd.get_float(
            "MAX_LENGTH", FILAMENT_LOAD_MAX_LENGTH, above=0.0
        )
        prime_time = gcmd.get_float(
            "PRIME_TIME", FILAMENT_LOAD_PRIME_TIME, above=0.0
        )
        prime_length = gcmd.get_float("PRIME_LENGTH", None, above=0.0)
        if prime_length is None:
            prime_length = speed * prime_time
        threshold = gcmd.get_float(
            "THRESHOLD", FILAMENT_LOAD_THRESHOLD, above=0.0
        )
        segment_time = gcmd.get_float(
            "SEGMENT_TIME", FILAMENT_LOAD_SEGMENT_TIME, above=0.0
        )
        apply_result = gcmd.get_int("APPLY", 1, minval=0, maxval=1)

        params = self.thermal_model.params
        filament_area = (
            params.filament_radius * params.filament_radius * math.pi
        )
        last_temp = last_pos = start_pos = loaded_at_pos = None
        load_energy = loaded_energy = loaded_denominator = 0.0
        loaded = False
        stop_requested = False
        reactor = self.toolboard.printer.get_reactor()

        def cb(reading):
            nonlocal last_temp, last_pos, start_pos, loaded_at_pos
            nonlocal load_energy, loaded_energy, loaded_denominator, loaded
            nonlocal stop_requested

            if reading.delta_time is None or reading.delta_time <= 0.0:
                return
            pos = self.extruder_position(reading.time)
            if last_temp is None:
                last_temp = reading.nozzle_temperature
                last_pos = pos
                start_pos = pos
                return
            dt = reading.delta_time
            ambient = self._blend_ambient_temp(reading.sensor_temperature)
            loss_ambient = (
                reading.nozzle_temperature - ambient
            ) / params.to_ambient_r
            loss_part_cooling = 0.0
            fan_speed = self.fan_speed(reading.time)
            if fan_speed > 0.0:
                pcf_ambient_r = params.part_cooling_fan_a * fan_speed ** (
                    -params.part_cooling_fan_k
                )
                if pcf_ambient_r > 0.0:
                    loss_part_cooling = max(
                        0.0,
                        (reading.nozzle_temperature - ambient) / pcf_ambient_r,
                    )
            stored = params.thermal_capacity * (
                reading.nozzle_temperature - last_temp
            )
            loss = (loss_ambient + loss_part_cooling) * dt
            residual = reading.delta_pwm_energy - loss - stored
            load_energy = max(0.0, load_energy + residual)
            pos_delta = max(0.0, pos - last_pos)
            temp_delta = max(0.0, reading.nozzle_temperature - ambient)
            if loaded:
                loaded_energy += max(0.0, residual)
                loaded_denominator += (
                    pos_delta * filament_area / 1000.0 * temp_delta
                )
            elif load_energy >= threshold:
                loaded = True
                loaded_at_pos = pos
                last_pos = pos
            last_temp = reading.nozzle_temperature
            last_pos = pos
            if not stop_requested:
                if not loaded and pos - start_pos >= max_length:
                    stop_requested = True
                elif loaded and pos - loaded_at_pos >= prime_length:
                    stop_requested = True

        gcmd.respond_info("INDX: loading filament")
        accel_dist = speed * speed / extruder.max_e_accel
        move_len = max(speed * segment_time, accel_dist * 1.1)
        move_len = min(move_len, extruder.max_e_dist)
        if move_len <= 0.0:
            raise gcmd.error("INDX: invalid extruder segment length")
        queued = 0.0
        self.temperature_callbacks.append(cb)
        try:
            while not stop_requested and queued < max_length + prime_length:
                pos = toolhead.get_position()
                segment = min(move_len, max_length + prime_length - queued)
                pos[3] += segment
                toolhead.manual_move(pos, speed)
                queued += segment
                reactor.pause(reactor.monotonic() + 0.01)
            toolhead.wait_moves()
        finally:
            self.temperature_callbacks.remove(cb)

        moved = 0.0 if start_pos is None else last_pos - start_pos
        if not loaded:
            raise gcmd.error(
                "INDX: filament load was not detected after %.1f mm "
                "(%.1f J load energy)" % (moved, load_energy)
            )
        if loaded_denominator <= 0.0:
            raise gcmd.error("INDX: insufficient prime data for heat capacity")

        heat_capacity = loaded_energy / loaded_denominator
        if apply_result:
            self.thermal_model.params = params._replace(
                filament_density=1.0,
                filament_heat_capacity=heat_capacity,
            )
            configfile = self.toolboard.printer.lookup_object("configfile")
            section = self.toolboard.name
            configfile.set(section, "model_filament_density", "1.0000")
            configfile.set(
                section, "model_filament_heat_capacity", "%.4f" % heat_capacity
            )
        gcmd.respond_info(
            "INDX filament loaded after %.1f mm; primed %.1f mm.\n"
            "filament_density: 1.0000 g/cm^3\n"
            "filament_heat_capacity: %.4f J/g/K\n"
            "%s"
            % (
                loaded_at_pos - start_pos,
                last_pos - loaded_at_pos,
                heat_capacity,
                "Run SAVE_CONFIG to save the new filament parameters."
                if apply_result
                else "Use APPLY=1 to apply the measured filament parameters.",
            )
        )

    def cmd_CLEAR_FILAMENT(self, gcmd):
        self.thermal_model.params = self.thermal_model.params._replace(
            filament_density=0.0,
            filament_heat_capacity=0.0,
        )
        configfile = self.toolboard.printer.lookup_object("configfile")
        section = self.toolboard.name
        configfile.set(section, "model_filament_density", "0.0000")
        configfile.set(section, "model_filament_heat_capacity", "0.0000")
        gcmd.respond_info(
            "INDX filament model cleared. Run SAVE_CONFIG to save."
        )

    def cmd_EXTRUDER_MOVE(self, gcmd):
        distance = gcmd.get_float("DISTANCE")
        speed = gcmd.get_float("SPEED", above=0.0)
        current = gcmd.get_float("CURRENT", above=0.0)

        toolhead = self.toolboard.printer.lookup_object("toolhead")
        extruder = toolhead.get_extruder()
        if self.heater is None or extruder.get_heater() != self.heater:
            raise gcmd.error("Active extruder is not using the INDX heater")
        if extruder.extruder_stepper is None:
            raise gcmd.error("Active extruder does not have a stepper")
        stepper = extruder.extruder_stepper.stepper
        current_helper = compat.get_tmc_current_helper(stepper)
        if current_helper is None:
            raise gcmd.error(
                "Active extruder does not have a TMC current helper"
            )

        run_current, hold_current, req_hold_current, *_ = (
            current_helper.get_current()
        )
        current = min(current, run_current)
        restore_hold_current = (
            req_hold_current if req_hold_current is not None else hold_current
        )

        toolhead.wait_moves()
        print_time = toolhead.get_last_move_time()
        current_helper.set_current(current, restore_hold_current, print_time)
        try:
            pos = toolhead.get_position()
            pos[3] += distance
            can_extrude = self.heater.can_extrude
            self.heater.can_extrude = True
            try:
                toolhead.manual_move(pos, speed)
            finally:
                self.heater.can_extrude = can_extrude
            toolhead.wait_moves()
        finally:
            print_time = toolhead.get_last_move_time()
            current_helper.set_current(
                run_current, restore_hold_current, print_time
            )

    def cmd_SET_MODEL_PARAMS(self, gcmd):
        cur = self.thermal_model.params

        max_power = gcmd.get_float(
            "MAX_POWER", default=cur.max_power, minval=0.0
        )
        max_power_temp_coeff = gcmd.get_float(
            "MAX_POWER_TEMP_COEFF", default=cur.max_power_temp_coeff
        )
        thermal_capacity = gcmd.get_float(
            "THERMAL_CAPACITY", default=cur.thermal_capacity, above=0.0
        )
        to_ambient_r = gcmd.get_float(
            "TO_AMBIENT_R", default=cur.to_ambient_r, above=0.0
        )
        filament_radius = (
            gcmd.get_float(
                "FILAMENT_DIAMETER",
                default=cur.filament_radius * 2.0,
                minval=0.0,
            )
            / 2.0
        )
        filament_density = gcmd.get_float(
            "FILAMENT_DENSITY", default=cur.filament_density, minval=0.0
        )
        filament_heat_capacity = gcmd.get_float(
            "FILAMENT_HEAT_CAPACITY",
            default=cur.filament_heat_capacity,
            minval=0.0,
        )
        part_cooling_fan_a = gcmd.get_float(
            "PART_COOLING_FAN_A", default=cur.part_cooling_fan_a
        )
        part_cooling_fan_k = gcmd.get_float(
            "PART_COOLING_FAN_K", default=cur.part_cooling_fan_k
        )
        ambient_blend_board = gcmd.get_float(
            "AMBIENT_BLEND_BOARD", default=cur.ambient_blend_board
        )
        ambient_blend_bracket = gcmd.get_float(
            "AMBIENT_BLEND_BRACKET", default=cur.ambient_blend_bracket
        )
        ambient_blend_sensor = gcmd.get_float(
            "AMBIENT_BLEND_SENSOR", default=cur.ambient_blend_sensor
        )
        error_application = gcmd.get_float(
            "ERROR_APPLICATION", default=cur.error_application
        )

        new_model = ThermalModelParams(
            max_power=max_power,
            max_power_temp_coeff=max_power_temp_coeff,
            thermal_capacity=thermal_capacity,
            to_ambient_r=to_ambient_r,
            filament_radius=filament_radius,
            filament_density=filament_density,
            filament_heat_capacity=filament_heat_capacity,
            part_cooling_fan_a=part_cooling_fan_a,
            part_cooling_fan_k=part_cooling_fan_k,
            ambient_blend_board=ambient_blend_board,
            ambient_blend_bracket=ambient_blend_bracket,
            ambient_blend_sensor=ambient_blend_sensor,
            error_application=error_application,
        )
        self.thermal_model.params = new_model

        params = "\n".join(f"{k}: {v}" for k, v in new_model._asdict().items())
        gcmd.respond_info(f"New model parameters:\n{params}")

    def cmd_DUMP_MODEL_UPDATE(self, gcmd):
        if self.cur_target_temp() is None:
            raise gcmd.error("Heating is not currently active")
        state = self.thermal_model.last_step
        if state is not None:
            params = "\n".join(f"{k}: {v}" for k, v in state._asdict().items())
            gcmd.respond_info(f"Last model update:\n{params}")
        else:
            gcmd.respond_info("Thermal model has not yet run")

    def cmd_LOG(self, gcmd):
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
            return
        self.log_file = open("/tmp/indx_log.csv", "w")
        self.log_file.write("time,temp,model,delta_energy,power\n")

    def cmd_DEBUG_IR_SENSOR_EEPROM(self, gcmd):
        cmd = self.toolboard.mcu.lookup_query_command(
            "indx_debug_ir_sensor_eeprom",
            "indx_debug_ir_sensor_eeprom_data data=%*s",
            cq=self.cmd_queue,
        )
        res = cmd.send([])
        print(res)
        gcmd.respond_info(
            "".join(hex(c)[2:].ljust(2, "0") for c in res["data"])
        )

    def cmd_DEBUG_STREAM_RAW_IR_SENSOR(self, gcmd):
        if self.raw_ir_log_file is not None:
            try:
                self.cmd_debug_stream_raw_ir.send([0])
            finally:
                self.raw_ir_log_file.close()
                self.raw_ir_log_file = None
                self.raw_ir_log_sensors = []
            gcmd.respond_info("INDX raw IR streaming stopped")
            return

        reactor = self.toolboard.printer.get_reactor()
        eventtime = reactor.monotonic()
        pheaters = self.toolboard.printer.lookup_object("heaters")
        self.raw_ir_log_sensors = list(
            pheaters.get_status(eventtime)["available_sensors"]
        )
        print(self.raw_ir_log_sensors)
        path = "/tmp/indx_raw_ir_%s.csv" % strftime("%Y%m%d_%H%M%S")
        self.raw_ir_log_file = open(path, "w")
        names = [
            n[len("temperature_sensor ") :]
            if n.startswith("temperature_sensor ")
            else n
            for n in self.raw_ir_log_sensors
        ]
        header = ["time", "raw_object", "raw_ambient"] + names
        self.raw_ir_log_file.write(",".join(header) + "\n")
        self.cmd_debug_stream_raw_ir.send([1])
        gcmd.respond_info("INDX raw IR streaming to %s" % path)

    def handle_debug_raw_ir(self, params):
        if self.raw_ir_log_file is None:
            return
        printer = self.toolboard.printer
        eventtime = printer.get_reactor().monotonic()
        cols = [
            "%.6f" % eventtime,
            str(params["object"]),
            str(params["ambient"]),
        ]
        for name in self.raw_ir_log_sensors:
            try:
                cols.append(
                    "%.2f"
                    % printer.lookup_object(name).get_status(eventtime)[
                        "temperature"
                    ]
                )
            except Exception as e:
                cols.append("")
        try:
            self.raw_ir_log_file.write(",".join(cols) + "\n")
        except Exception as e:
            logging.error(f"Write failed: {e}")

    # PWM pin interface

    def get_mcu(self):
        return self.toolboard.mcu

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        # No-op for INDX, controlled by hardware
        pass

    def setup_max_duration(self, max_duration):
        # No-op for INDX, controlled by hardware
        pass

    def set_pwm(self, print_time, value):
        pass


class HeaterControlIndx:
    def __init__(self, heater, indx_heater):
        self.heater = heater
        self.indx_heater = indx_heater
        self.profile = None

    def temperature_update(self, read_time, temp, target_temp):
        self.indx_heater.set_target_temp(target_temp)
        self.heater.set_pwm(read_time, self.indx_heater.last_report[2])

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        # TODO: Improve this
        return smoothed_temp < target_temp - 1

    def get_profile(self):
        return self.profile


ThermalModelParams = namedtuple(
    "ThermalModelParams",
    [
        "max_power",
        "max_power_temp_coeff",
        "thermal_capacity",
        "to_ambient_r",
        "filament_radius",
        "filament_density",
        "filament_heat_capacity",
        "part_cooling_fan_a",
        "part_cooling_fan_k",
        "ambient_blend_bracket",
        "ambient_blend_board",
        "ambient_blend_sensor",
        "error_application",
    ],
)

ThermalModelStep = namedtuple(
    "ThermalModelStep",
    [
        "current_temp",
        "avg_input_power",
        "ambient_temp",
        "filament_temp",
        "loss_ambient",
        "loss_filament",
        "loss_part_cooling",
        "delta_temp_rate",
        "dt",
        "new_temp",
    ],
)


class ThermalModel:
    def __init__(self, params):
        self.temperature = 0
        self.params = params
        self.last_step = None

    def is_valid(self):
        return (
            self.params.max_power is not None
            and self.params.max_power_temp_coeff is not None
            and self.params.thermal_capacity is not None
            and self.params.to_ambient_r is not None
        )

    def force_temp(self, temperature):
        self.temperature = temperature

    def calc_step(
        self,
        dt,
        avg_input_power,
        ambient_temp,
        filament_temp,
        filament_distance,
        fan_speed,
    ):
        filament_heat_capacity_mm = (
            (self.params.filament_radius * self.params.filament_radius)
            * math.pi
            / 1000.0
            * self.params.filament_density
            * self.params.filament_heat_capacity
        )

        loss_part_cooling = 0.0
        if fan_speed > 0.0:
            pcf_ambient_r = self.params.part_cooling_fan_a * fan_speed ** (
                -self.params.part_cooling_fan_k
            )
            if pcf_ambient_r > 0.0:
                loss_part_cooling = max(
                    0.0, (self.temperature - ambient_temp) / pcf_ambient_r
                )

        loss_ambient = (
            self.temperature - ambient_temp
        ) / self.params.to_ambient_r
        loss_filament = (
            filament_distance
            * filament_heat_capacity_mm
            * (self.temperature - filament_temp)
            / dt
        )
        total_power = (
            avg_input_power - loss_ambient - loss_filament - loss_part_cooling
        )
        delta_temp_rate = total_power / self.params.thermal_capacity
        new_temp = self.temperature + delta_temp_rate * dt

        return ThermalModelStep(
            current_temp=self.temperature,
            avg_input_power=avg_input_power,
            ambient_temp=ambient_temp,
            filament_temp=filament_temp,
            loss_ambient=loss_ambient,
            loss_filament=loss_filament,
            loss_part_cooling=loss_part_cooling,
            dt=dt,
            delta_temp_rate=delta_temp_rate,
            new_temp=new_temp,
        )

    def step(
        self,
        dt,
        avg_input_power,
        ambient_temp,
        filament_temp,
        filament_distance,
        fan_speed,
    ):
        info = self.calc_step(
            dt,
            avg_input_power,
            ambient_temp,
            filament_temp,
            filament_distance,
            fan_speed,
        )
        self.last_step = info
        self.temperature = info.new_temp

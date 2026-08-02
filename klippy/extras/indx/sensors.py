from ..thermistor import CustomThermistor
from .compat import ConfigWrapper

TEMP_REPORT_TIME = 0.3


def indx_temp_sensor_factory(config):
    kind = config.getchoice(
        "indx_sensor",
        [
            "heater",
            "board",
            "ldc_coil",
            "bracket",
            "sensor",
            "check_model",
            "check_model_delta",
        ],
        "heater",
    )
    toolboard_name = config.get("toolboard", None)
    toolboard = config.get_printer().lookup_object(
        "indx" if toolboard_name is None else f"indx {toolboard_name}"
    )

    if kind == "heater":
        return IndxHeaterTemperatureSensor(toolboard, "heater")
    elif kind == "board":
        return IndxTempSensorWrapper(toolboard, "board")
    elif kind == "ldc_coil":
        return ldc_coil_sensor(toolboard, config)
    elif kind == "bracket":
        return IndxTempSensorWrapper(toolboard, "bracket")
    elif kind == "sensor":
        return IndxHeaterTemperatureSensor(toolboard, "sensor")
    elif kind == "check_model":
        return IndxHeaterTemperatureSensor(toolboard, "model")
    elif kind == "check_model_delta":
        return IndxHeaterTemperatureSensor(toolboard, "model_delta")


class IndxHeaterTemperatureSensor:
    def __init__(self, toolboard, source):
        self.toolboard = toolboard
        if source == "heater":
            cb = self.handle_nozzle_temperature
        elif source == "sensor":
            cb = self.handle_sensor_temperature
        elif source == "model":
            cb = self.handle_model_temperature
        elif source == "model_delta":
            cb = self.handle_model_delta
        else:
            raise Exception("Unknown temperature source")
        self.toolboard.heater.temperature_callbacks.append(cb)
        self.temperature_callback = lambda time, value: 0

    def handle_nozzle_temperature(self, reading):
        self.temperature_callback(reading.time, reading.nozzle_temperature)

    def handle_sensor_temperature(self, reading):
        self.temperature_callback(reading.time, reading.sensor_temperature)

    def handle_model_temperature(self, reading):
        self.temperature_callback(
            reading.time, self.toolboard.heater.thermal_model.temperature
        )

    def handle_model_delta(self, reading):
        self.temperature_callback(
            reading.time,
            self.toolboard.heater.thermal_model.temperature
            - reading.nozzle_temperature,
        )

    # Sensor callbacks

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def get_report_time_delta(self):
        return TEMP_REPORT_TIME

    def setup_minmax(self, min_temp, max_temp):
        pass


def ldc_coil_sensor(toolboard, config):
    config_thermistor = ConfigWrapper(
        toolboard.printer,
        config.get_name(),
        {
            # 10K Murata NCU15XH103J6SRC
            "beta": 3425.0,
            "temperature1": 25.0,
            "resistance1": 10000.0,
        },
    )
    config_sensor = ConfigWrapper(
        toolboard.printer,
        config.get_name(),
        {
            "sensor_pin": f"{toolboard.mcu.get_name()}:temp_ldc",
            "pullup_resistor": 3900,
        },
        config,
    )
    return CustomThermistor(config_thermistor).create(config_sensor)


class IndxTempSensorWrapper:
    def __init__(self, toolboard, sensor):
        self.toolboard = toolboard
        self.sensor = sensor

    def setup_callback(self, temperature_callback):
        self.toolboard.heater.aux_temp_sensors[self.sensor].add_callback(
            temperature_callback
        )

    def setup_minmax(self, min_temp, max_temp):
        pass

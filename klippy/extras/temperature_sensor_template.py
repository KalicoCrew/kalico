import ast
import logging
from .danger_options import get_danger_options

DEFAULT_REPORT_TIME = 0.300


class PrinterTemperatureTemplate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        self.last_temp = self.min_temp = self.max_temp = 0.0

        self.gcode_macro = self.printer.lookup_object("gcode_macro")
        self.template = self.gcode_macro.load_template(config, "template")

        self.heater = None
        self.report_time = DEFAULT_REPORT_TIME
        self.temperature_update_timer = None

        self.heater_name = config.get("heater_name", None)
        if self.heater_name:
            if config.get("report_time", None, False):
                raise config.error(
                    'temperature_sensor_combined {self.name} you cannot have both "heater_name" and "report_time"'
                )
        else:
            self.report_time = config.getfloat(
                "report_time", self.report_time, minval=DEFAULT_REPORT_TIME
            )

        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect
        )
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_connect(self):
        if self.heater_name:
            pheaters = self.printer.lookup_object("heaters")
            heater = pheaters.lookup_heater(self.heater_name)
            heater.add_sensor_callback(self._sensor_callback)

        else:
            self.temperature_update_timer = self.reactor.register_timer(
                self._temperature_update_event
            )

    def _handle_ready(self):
        if self.temperature_update_timer:
            self.reactor.update_timer(
                self.temperature_update_timer,
                self.reactor.monotonic() + self.report_time,
            )

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def get_report_time_delta(self):
        return self.report_time

    def get_status(self, eventtime):
        return {"temperature": round(self.last_temp, 2)}

    def get_temp(self, eventtime):
        return self.last_temp, 0.0

    def update_temp(self, eventtime):
        temp = self._eval_template(eventtime)

        if temp:
            self.last_temp = temp

        if not get_danger_options().temp_ignore_limits:
            if self.last_temp < self.min_temp:
                self.printer.invoke_shutdown(
                    f"template sensor temperature {self.last_temp:0.1f} below minimum temperature of {self.min_temp:0.1f}"
                )
            if self.last_temp > self.max_temp:
                self.printer.invoke_shutdown(
                    f"template sensor temperature {self.last_temp:0.1f} above maximum temperature of {self.max_temp:0.1f}"
                )

        self.temperature_callback(eventtime, self.last_temp)

    def _sensor_callback(self, eventtime, heater):
        self.update_temp(eventtime)

    def _temperature_update_event(self, eventtime):
        self.update_temp(eventtime)

        # Since this is pretty low priority we're just updating monotonically
        return self.reactor.monotonic() + self.report_time

    def _eval_template(self, eventtime):
        context = self.gcode_macro.create_template_context(eventtime)
        try:
            statement = self.template.render(context)
        except:
            logging.exception(
                f"{self.name} failed to render template {self.template}"
            )
            raise
        try:
            value = ast.literal_eval(statement) if statement else None
        except:
            logging.exception(f"{self.name} failed to evaluate {statement}")
            raise
        try:
            return float(statement)
        except:
            logging.exception(
                f"{self.name} failed to coerce {value!r} as float"
            )
            raise


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory("template", PrinterTemperatureTemplate)

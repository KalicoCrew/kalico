from . import heaters
from .danger_options import get_danger_options


class MPC_AMBIENT_TEMP_WRAPPER:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]

        self.heater = None

        self.heater_name = config.get("heater_name")

        self.temperature_callback = None

        self.temp = self.min_temp = self.max_temp = 0.0

        self.ignore = (
            config.getboolean("ignore_limits", False)
            or get_danger_options().temp_ignore_limits
        )

        self.echo_limits_to_console = config.getboolean(
            "echo_limits_to_console", False
        )

        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        pheaters = self.printer.lookup_object("heaters")
        self.heater = pheaters.lookup_heater(self.heater_name)
        self.heater.add_mpc_sensor(self)

    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback

    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp

    def get_report_time_delta(self):
        return self.heater.sensor.get_report_time_delta()

    def process_temp_update(self, control, read_time):
        if control.get_type() == "mpc" or control.get_type() == "tuning":
            self.temp = self.heater.get_control().state_ambient_temp
        else:
            self.temp = heaters.AMBIENT_TEMP

        if self.temp is not None:
            if not self.heater.mcu_pwm.get_mcu().non_critical_disconnected and (
                self.temp < self.min_temp or self.temp > self.max_temp
            ):
                if not self.ignore:
                    self.printer.invoke_shutdown(
                        "Ambient MPC %s\nTemperature %0.1f outside range of %0.1f-%.01f"
                        % (self.name, self.temp, self.min_temp, self.max_temp)
                    )
                elif self.echo_limits_to_console:
                    gcode = self.printer.lookup_object("gcode")
                    gcode.respond_error(
                        "Ambient MPC %s\nTemperature %0.1f outside range of %0.1f-%.01f"
                        % (self.name, self.temp, self.min_temp, self.max_temp)
                    )
        else:
            self.temp = 0.0

        self.temperature_callback(
            read_time,
            self.temp,
        )

    def set_report_time(self, report_time):
        pass


def load_config(config):
    pheaters = config.get_printer().load_object(config, "heaters")
    pheaters.add_sensor_factory(
        "mpc_ambient_temperature", MPC_AMBIENT_TEMP_WRAPPER
    )

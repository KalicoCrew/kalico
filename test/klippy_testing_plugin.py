import ast
import logging
from klippy.extras import gcode_macro


class KlippyTestingPlugin:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode_macro = self.printer.load_object(config, "gcode_macro")

        self.printer.register_event_handler(
            "gcode:command_error", self._command_error
        )
        self.printer.register_event_handler(
            "gcode:unknown_command", self._unknown_command
        )

        self.gcode.register_command("ASSERT", self.cmd_ASSERT)

    def _command_error(self):
        self.printer.request_exit("error_exit")
        self.printer.invoke_shutdown("Exception during testing")

    def _unknown_command(self, cmd):
        logging.error(f"Unknown command during test execution: {cmd}")
        self.printer.request_exit("error_exit")
        self.printer.invoke_shutdown(
            f"Unknown command during test execution: {cmd}"
        )

    def cmd_ASSERT(self, gcmd):
        "Evaluate an expression, raising an error if the return value is False"
        expression = gcmd.get("TEST")

        try:
            template = gcode_macro.Template(
                self.printer,
                self.gcode_macro.env,
                "ASSERT:runtime_expression",
                expression,
            )
        except:
            raise gcmd.error(f"ASSERT: Failed to parse '{expression}'")

        context = self.gcode_macro.create_template_context()
        statement = template.render(context)
        value = ast.literal_eval(statement) if statement else None

        if not value:
            raise gcmd.error(f"ASSERT: {expression} == {value}")


def load_config(config):
    return KlippyTestingPlugin(config)

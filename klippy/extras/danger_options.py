class DangerOptions:
    def __init__(self, config):
        self.minimal_logging = config.getboolean("minimal_logging", False)
        self.log_statistics = config.getboolean("log_statistics", True)
        self.log_config_file_at_startup = config.getboolean(
            "log_config_file_at_startup", True
        )
        self.log_bed_mesh_at_startup = config.getboolean(
            "log_bed_mesh_at_startup", True
        )
        self.log_shutdown_info = config.getboolean("log_shutdown_info", True)
        self.log_serial_reader_warnings = config.getboolean(
            "log_serial_reader_warnings", True
        )
        self.log_startup_info = config.getboolean("log_startup_info", True)
        self.log_webhook_method_register_messages = config.getboolean(
            "log_webhook_method_register_messages", False
        )
        self.error_on_unused_config_options = config.getboolean(
            "error_on_unused_config_options", True
        )

        self.allow_plugin_override = config.getboolean(
            "allow_plugin_override", False
        )

        self.multi_mcu_trsync_timeout = config.getfloat(
            "multi_mcu_trsync_timeout", 0.025, minval=0.0
        )
        self.homing_elapsed_distance_tolerance = config.getfloat(
            "homing_elapsed_distance_tolerance", 0.5, minval=0.0
        )
        self.adc_ignore_limits = config.getboolean("adc_ignore_limits", False)
        self.autosave_includes = config.getboolean("autosave_includes", False)

        self.bgflush_extra_time = config.getfloat(
            "bgflush_extra_time", 0.250, minval=0.0
        )

        if self.minimal_logging:
            self.log_statistics = False
            self.log_config_file_at_startup = False
            self.log_bed_mesh_at_startup = False
            self.log_shutdown_info = False
            self.log_serial_reader_warnings = False
            self.log_startup_info = False
            self.log_webhook_method_register_messages = False


DANGER_OPTIONS: DangerOptions = None


def get_danger_options():
    global DANGER_OPTIONS
    if DANGER_OPTIONS is None:
        raise Exception("DangerOptions has not been loaded yet!")
    return DANGER_OPTIONS


def load_config(config):
    global DANGER_OPTIONS
    DANGER_OPTIONS = DangerOptions(config)
    return DANGER_OPTIONS

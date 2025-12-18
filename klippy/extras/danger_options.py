class DangerOptions:
    def __init__(self, config):
        self.minimal_logging = config.getboolean("minimal_logging", False)
        verbose = not self.minimal_logging
        self.log_statistics = config.getboolean("log_statistics", verbose)
        self.log_config_file_at_startup = config.getboolean(
            "log_config_file_at_startup", verbose
        )
        self.log_bed_mesh_at_startup = config.getboolean(
            "log_bed_mesh_at_startup", verbose
        )
        self.log_velocity_limit_changes = config.getboolean(
            "log_velocity_limit_changes", verbose
        )
        self.log_pressure_advance_changes = config.getboolean(
            "log_pressure_advance_changes", verbose
        )
        self.log_shutdown_info = config.getboolean("log_shutdown_info", verbose)
        self.log_serial_reader_warnings = config.getboolean(
            "log_serial_reader_warnings", verbose
        )
        self.log_startup_info = config.getboolean("log_startup_info", verbose)
        self.log_webhook_method_register_messages = config.getboolean(
            "log_webhook_method_register_messages", verbose
        )
        self.error_on_unused_config_options = config.getboolean(
            "error_on_unused_config_options", True
        )
        self.allow_plugin_override = config.getboolean(
            "allow_plugin_override", False
        )
        self.single_mcu_trsync_timeout = config.getfloat(
            "single_mcu_trsync_timeout", 0.25, minval=0.0
        )
        self.multi_mcu_trsync_timeout = config.getfloat(
            "multi_mcu_trsync_timeout", 0.025, minval=0.0
        )
        self.homing_elapsed_distance_tolerance = config.getfloat(
            "homing_elapsed_distance_tolerance", 0.5, minval=0.0
        )

        temp_ignore_limits = False
        if config.getboolean("temp_ignore_limits", None) is None:
            adc_ignore_limits = config.getboolean("adc_ignore_limits", None)
            if adc_ignore_limits is not None:
                config.deprecate("adc_ignore_limits")
                temp_ignore_limits = adc_ignore_limits

        self.temp_ignore_limits = config.getboolean(
            "temp_ignore_limits", temp_ignore_limits
        )

        self.autosave_includes = config.getboolean("autosave_includes", False)
        self.bgflush_extra_time = config.getfloat(
            "bgflush_extra_time", 0.250, minval=0.0
        )
        self.homing_start_delay = config.getfloat(
            "homing_start_delay", 0.001, minval=0.0
        )
        self.endstop_sample_time = config.getfloat(
            "endstop_sample_time", 0.000015, minval=0
        )
        self.endstop_sample_count = config.getint(
            "endstop_sample_count", 4, minval=1
        )


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

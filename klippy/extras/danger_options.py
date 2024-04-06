class DangerOptions:
    def __init__(self, config):
        self.log_statistics = config.getboolean("log_statistics", True)
        self.log_config_file_at_startup = config.getboolean(
            "log_config_file_at_startup", True
        )
        self.log_bed_mesh_at_startup = config.getboolean(
            "log_bed_mesh_at_startup", True
        )
        self.log_shutdown_info = config.getboolean("log_shutdown_info", True)
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
        self.disable_serial_reader_warnings = config.getboolean(
            "disable_serial_reader_warnings", False
        )
        self.disable_webhook_logging = config.getboolean(
            "disable_webhook_logging", False
        )
        self.minimal_logging = config.getboolean("minimal_logging", False)
        if self.minimal_logging:
            self.log_statistics = False
            self.log_config_file_at_startup = False
            self.log_bed_mesh_at_startup = False
            self.log_shutdown_info = False
            self.disable_serial_reader_warnings = True
            self.disable_webhook_logging = True


def load_config(config):
    return DangerOptions(config)

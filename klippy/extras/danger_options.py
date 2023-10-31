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


def load_config(config):
    return DangerOptions(config)

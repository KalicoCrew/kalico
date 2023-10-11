class DebugOptions:
    def __init__(self, config):
        self.log_statistics = config.getboolean("log_statistics", True)
        self.log_config_file_at_startup = config.getboolean(
            "log_config_file_at_startup", True
        )
        self.log_bed_mesh_at_startup = config.getboolean(
            "log_bed_mesh_at_startup", True
        )
        self.log_shutdown_info = config.getboolean("log_shutdown_info", True)


def load_config_prefix(config):
    return DebugOptions(config)

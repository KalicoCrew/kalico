from .toolhead import CocoaToolheadControl


def load_config(config):
    return CocoaToolheadControl(config)


def load_config_prefix(config):
    return CocoaToolheadControl(config)

import os
import sys
import types


def _ensure_klippy_package():
    if "klippy" in sys.modules:
        return
    # Upstream Klipper adds the klippy/ directory itself to sys.path and loads
    # extras as a top-level package. Kalico exposes klippy as a real package.
    # Create the missing package alias here so Kalico imports keep working.
    extras_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    klippy_dir = os.path.dirname(extras_dir)
    module = types.ModuleType("klippy")
    module.__path__ = [klippy_dir]
    sys.modules["klippy"] = module


_ensure_klippy_package()

from .sensors import indx_temp_sensor_factory  # noqa: E402
from .toolboard import IndxToolboard  # noqa: E402


class IndxFactories:
    def __init__(self, config, printer):
        pheaters = printer.load_object(config, "heaters")
        pheaters.add_sensor_factory("indx", indx_temp_sensor_factory)


def register_factories(config):
    printer = config.get_printer()
    was_registered = printer.lookup_object("indx_factories", None)
    if was_registered is not None:
        return
    printer.add_object("indx_factories", IndxFactories(config, printer))


def load_config(config):
    register_factories(config)
    return IndxToolboard(config)

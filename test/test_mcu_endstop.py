from types import SimpleNamespace

import pytest

from klippy import mcu as mcu_mod
from klippy.extras import danger_options
from klippy.extras import non_critical_endstop_disconnect_values as ncedv


class _CommandError(Exception):
    pass


class _ConfigError(Exception):
    pass


class _FakePrinter:
    command_error = _CommandError
    config_error = _ConfigError

    def __init__(self, objects=None):
        self.objects = objects or {}
        self.event_handlers = {}

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)


class _FakeMCU:
    def __init__(self, printer=None, disconnected=True):
        self.non_critical_disconnected = disconnected
        self._printer = printer or _FakePrinter()

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "toolboard"

    def print_time_to_clock(self, print_time):
        return 0

    def is_fileoutput(self):
        return False


class _FakeQueryCmd:
    def __init__(self, pin_value):
        self.pin_value = pin_value

    def send(self, data, minclock=0):
        return {"pin_value": self.pin_value}


class _FakeConfig:
    error = _ConfigError

    def __init__(self, printer, options):
        self._printer = printer
        self._options = options

    def get_printer(self):
        return self._printer

    def get_name(self):
        return "non_critical_endstop_disconnect_values"

    def get_prefix_options(self, prefix):
        return [o for o in self._options if o.startswith(prefix)]

    def get(self, option):
        return self._options[option]


@pytest.fixture(autouse=True)
def _danger_options(monkeypatch):
    opts = SimpleNamespace(ignore_non_critical_endstop_disconnect=False)
    monkeypatch.setattr(danger_options, "DANGER_OPTIONS", opts)
    return opts


def _make_endstop(printer=None, disconnected=True, names=(), pin_value=0):
    endstop = mcu_mod.MCU_endstop.__new__(mcu_mod.MCU_endstop)
    endstop._mcu = _FakeMCU(printer, disconnected)
    endstop._home_cmd = None
    endstop._query_cmd = _FakeQueryCmd(pin_value)
    endstop._invert = 0
    endstop._oid = 0
    for name in names:
        endstop.add_endstop_name(name)
    return endstop


def _make_printer_with_values(options):
    printer = _FakePrinter()
    values = ncedv.NonCriticalEndstopDisconnectValues(
        _FakeConfig(printer, options)
    )
    printer.objects["non_critical_endstop_disconnect_values"] = values
    return printer


def test_query_endstop_on_disconnected_mcu_raises_command_error():
    with pytest.raises(_CommandError, match="disconnected MCU 'toolboard'"):
        _make_endstop().query_endstop(0.0)


def test_home_start_on_disconnected_mcu_raises_command_error():
    with pytest.raises(_CommandError, match="disconnected MCU 'toolboard'"):
        _make_endstop().home_start(0.0, 0.0, 1, 0.0)


def test_connected_mcu_uses_real_state(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"mmu_gear_0": "triggered"})
    endstop = _make_endstop(
        printer, disconnected=False, names=["mmu_gear_0"], pin_value=0
    )
    assert endstop.query_endstop(0.0) == 0


def test_fallback_ignored_when_danger_option_disabled():
    printer = _make_printer_with_values({"mmu_gear_0": "triggered"})
    endstop = _make_endstop(printer, names=["mmu_gear_0"])
    with pytest.raises(_CommandError, match="disconnected MCU"):
        endstop.query_endstop(0.0)


@pytest.mark.parametrize(
    "value,expected",
    [
        ("open", 0),
        ("false", 0),
        ("0", 0),
        ("triggered", 1),
        ("TRIGGERED", 1),
        ("closed", 1),
        ("true", 1),
        ("1", 1),
    ],
)
def test_fallback_value_returned(_danger_options, value, expected):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"mmu_gear_0": value})
    endstop = _make_endstop(printer, names=["mmu_gear_0"])
    assert endstop.query_endstop(0.0) == expected


def test_unconfigured_endstop_still_raises(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"mmu_gear_0": "triggered"})
    endstop = _make_endstop(printer, names=["mmu_pre_gate_0"])
    with pytest.raises(_CommandError, match="disconnected MCU"):
        endstop.query_endstop(0.0)


def test_home_start_never_uses_fallback(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"mmu_gear_0": "triggered"})
    endstop = _make_endstop(printer, names=["mmu_gear_0"])
    with pytest.raises(_CommandError, match="disconnected MCU"):
        endstop.home_start(0.0, 0.0, 1, 0.0)


def test_reconnect_uses_real_state_again(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"mmu_gear_0": "triggered"})
    endstop = _make_endstop(printer, names=["mmu_gear_0"], pin_value=0)
    assert endstop.query_endstop(0.0) == 1
    endstop._mcu.non_critical_disconnected = False
    assert endstop.query_endstop(0.0) == 0


def test_invalid_value_is_config_error():
    with pytest.raises(_ConfigError, match="Invalid value 'invalid'"):
        _make_printer_with_values({"mmu_gear_0": "invalid"})


def test_unknown_endstop_name_is_config_error():
    printer = _make_printer_with_values({"typo_endstop": "open"})
    printer.objects["query_endstops"] = SimpleNamespace(
        endstops=[(None, "mmu_gear_0")]
    )
    with pytest.raises(_ConfigError, match="typo_endstop"):
        for cb in printer.event_handlers["klippy:connect"]:
            cb()


class _ProbeEndstopWrapper(mcu_mod.MCU_endstop):
    # Mimics probe endstop wrappers (e.g. Cartographer's
    # KlipperProbeEndstop) that never call MCU_endstop.__init__
    def __init__(self, mcu, pin_value=0):
        self._mcu = mcu
        self._oid = 0
        self._invert = 0
        self._query_cmd = _FakeQueryCmd(pin_value)


def test_wrapper_without_init_names():
    wrapper = _ProbeEndstopWrapper(_FakeMCU())
    assert "_endstop_names" not in wrapper.__dict__
    assert wrapper.get_endstop_names() == []
    wrapper.add_endstop_name("probe")
    wrapper.add_endstop_name("probe")
    wrapper.add_endstop_name("z_virtual")
    assert wrapper.get_endstop_names() == ["probe", "z_virtual"]
    # Names are per instance, the class default stays untouched
    assert mcu_mod.MCU_endstop._endstop_names == ()
    assert _ProbeEndstopWrapper(_FakeMCU()).get_endstop_names() == []


def test_wrapper_without_init_uses_fallback(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"probe": "triggered"})
    wrapper = _ProbeEndstopWrapper(_FakeMCU(printer), pin_value=0)
    wrapper.add_endstop_name("probe")
    assert wrapper.query_endstop(0.0) == 1


def test_wrapper_without_init_unnamed_still_raises(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"probe": "triggered"})
    wrapper = _ProbeEndstopWrapper(_FakeMCU(printer))
    with pytest.raises(_CommandError, match="disconnected MCU"):
        wrapper.query_endstop(0.0)


def test_wrapper_without_init_connected_uses_real_state(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values({"probe": "triggered"})
    wrapper = _ProbeEndstopWrapper(
        _FakeMCU(printer, disconnected=False), pin_value=0
    )
    wrapper.add_endstop_name("probe")
    assert wrapper.query_endstop(0.0) == 0


def test_wrapper_without_init_option_disabled_raises():
    printer = _make_printer_with_values({"probe": "triggered"})
    wrapper = _ProbeEndstopWrapper(_FakeMCU(printer))
    wrapper.add_endstop_name("probe")
    with pytest.raises(_CommandError, match="disconnected MCU"):
        wrapper.query_endstop(0.0)

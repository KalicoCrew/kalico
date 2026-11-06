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

    def get(self, option, default=None):
        return self._options.get(option, default)


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


def _run_connect(printer):
    for cb in printer.event_handlers["klippy:connect"]:
        cb()


def _make_printer_with_values(options):
    # Section options as the config parser returns them, for example
    # {"mmu_gear_0": "triggered"} or {"open": "mmu:a, mmu:b"}
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


@pytest.mark.parametrize(
    "value,hint",
    [
        ("filament_tension: triggered", "'triggered: mmu:filament_tension'"),
        ("filament_tension: TRUE", "'triggered: mmu:filament_tension'"),
        ("mmu_shared_exit: open", "'open: mmu:mmu_shared_exit'"),
    ],
)
def test_name_with_colon_as_option_explains_list_options(value, hint):
    # "mmu:filament_tension: triggered" reaches us as option "mmu" with
    # value "filament_tension: triggered"
    with pytest.raises(_ConfigError) as excinfo:
        _make_printer_with_values({"mmu": value})
    msg = str(excinfo.value)
    assert "names containing ':' must be listed" in msg
    assert hint in msg


def test_names_with_colon_and_upper_case(_danger_options):
    # Names as reported by QUERY_ENDSTOPS for Happy Hare sensors; neither
    # ':' nor upper case letters would survive as a config option name
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values(
        {
            "open": "mmu:mmu_shared_exit, mmu:filament_compression",
            "triggered": "mmu:filament_tension, Toolhead_Sensor",
        }
    )
    values = printer.objects["non_critical_endstop_disconnect_values"]
    assert values.values == {
        "mmu:mmu_shared_exit": 0,
        "mmu:filament_compression": 0,
        "mmu:filament_tension": 1,
        "Toolhead_Sensor": 1,
    }
    endstop = _make_endstop(printer, names=["mmu:filament_tension"])
    assert endstop.query_endstop(0.0) == 1
    endstop = _make_endstop(printer, names=["mmu:mmu_shared_exit"])
    assert endstop.query_endstop(0.0) == 0


def test_multiline_lists():
    printer = _make_printer_with_values(
        {
            "open": "\nmmu:gate_0,\n  mmu:gate_1\nmmu:gate_2,,",
            "triggered": "",
        }
    )
    values = printer.objects["non_critical_endstop_disconnect_values"]
    assert values.values == {"mmu:gate_0": 0, "mmu:gate_1": 0, "mmu:gate_2": 0}


def test_empty_section():
    printer = _make_printer_with_values({})
    values = printer.objects["non_critical_endstop_disconnect_values"]
    assert values.values == {}
    _run_connect(printer)


def test_name_in_both_lists_is_config_error():
    with pytest.raises(_ConfigError, match="both 'open' and 'triggered'"):
        _make_printer_with_values(
            {"open": "mmu:gate_0", "triggered": "x, mmu:gate_0"}
        )


def test_single_and_list_options_combined(_danger_options):
    _danger_options.ignore_non_critical_endstop_disconnect = True
    printer = _make_printer_with_values(
        {
            "x": "open",
            "mmu_gear_0": "triggered",
            "open": "mmu:mmu_shared_exit",
            "triggered": "mmu:filament_tension, mmu_gear_0",
        }
    )
    values = printer.objects["non_critical_endstop_disconnect_values"]
    assert values.values == {
        "x": 0,
        "mmu_gear_0": 1,
        "mmu:mmu_shared_exit": 0,
        "mmu:filament_tension": 1,
    }


def test_single_and_list_option_conflict_is_config_error():
    with pytest.raises(_ConfigError, match="both 'open' and 'triggered'"):
        _make_printer_with_values({"x": "open", "triggered": "x"})


def test_unknown_endstop_name_is_config_error():
    # Stepper endstops are registered with their short name ("x"), so
    # "stepper_x" is reported as unknown together with the valid names
    printer = _make_printer_with_values(
        {"stepper_x": "open", "mmu_gear_0": "triggered"}
    )
    printer.objects["query_endstops"] = SimpleNamespace(
        endstops=[(None, "x"), (None, "mmu_gear_0")]
    )
    with pytest.raises(_ConfigError) as excinfo:
        _run_connect(printer)
    msg = str(excinfo.value)
    assert (
        "Unknown endstop(s) in [non_critical_endstop_disconnect_values]" in msg
    )
    assert "stepper_x" in msg
    assert "known endstops, as reported by QUERY_ENDSTOPS: mmu_gear_0, x" in msg


def test_known_endstop_names_pass_connect_check():
    printer = _make_printer_with_values(
        {"x": "open", "mmu_gear_0": "triggered"}
    )
    printer.objects["query_endstops"] = SimpleNamespace(
        endstops=[(None, "x"), (None, "y"), (None, "mmu_gear_0")]
    )
    _run_connect(printer)


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

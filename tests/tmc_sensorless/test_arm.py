import pytest

from klippy import pins
from klippy.bridge_endstop import PROVIDER_ID_FIRST
from klippy.extras import tmc, tmc2130, tmc2208, tmc2209


class _FakeMcu:
    def __init__(self):
        self.oids = 0
        self.config_cmds = []
        self.config_callbacks = []

    def create_oid(self):
        oid = self.oids
        self.oids += 1
        return oid

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def add_config_cmd(self, cmd):
        self.config_cmds.append(cmd)

    def lookup_command(self, template):
        return None

    def lookup_query_command(self, template, response, oid=None):
        return None


class _FakePins:
    def __init__(self, mcu):
        self._mcu = mcu
        self.chips = {}

    def register_chip(self, name, obj):
        self.chips[name] = obj

    def parse_pin(self, pin_desc, can_invert=False, can_pullup=False):
        pin = pin_desc
        pullup = invert = 0
        if can_pullup and pin.startswith("^"):
            pullup = 1
            pin = pin[1:]
        if can_invert and pin.startswith("!"):
            invert = 1
            pin = pin[1:]
        return {
            "chip": self._mcu,
            "chip_name": "mcu",
            "pin": pin,
            "pullup": pullup,
            "invert": invert,
        }


class _FakePrinter:
    def __init__(self, ppins):
        self._objects = {"pins": ppins}

    def lookup_object(self, name, default=None):
        return self._objects.get(name, default)

    def add_object(self, name, obj):
        self._objects[name] = obj


class _FakeConfig:
    def __init__(self, name, options):
        self._name = name
        self._options = options
        self.printer = _FakePrinter(_FakePins(_FakeMcu()))

    def get_printer(self):
        return self.printer

    def get(self, key, default=None):
        return self._options.get(key, default)

    def get_name(self):
        return self._name


class _FakeMcuTmc:
    def __init__(self, fields):
        self._fields = fields
        self.writes = []

    def get_fields(self):
        return self._fields

    def set_register(self, reg_name, val, print_time=None):
        self.writes.append((reg_name, val))


def _virtual_endstop_params(pin="virtual_endstop", invert=0, pullup=0):
    return {
        "chip_name": "tmc",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def _helper_2209(sgthrs=75, tpwmthrs=120, en_spreadcycle=1, tcoolthrs=0):
    fields = tmc.FieldHelper(
        tmc2209.Fields, tmc2208.SignedFields, tmc2209.FieldFormatters
    )
    fields.set_field("sgthrs", sgthrs)
    fields.set_field("tpwmthrs", tpwmthrs)
    fields.set_field("en_spreadcycle", en_spreadcycle)
    fields.set_field("tcoolthrs", tcoolthrs)
    mcu_tmc = _FakeMcuTmc(fields)
    config = _FakeConfig("tmc2209 stepper_x", {"diag_pin": "^PA1"})
    return tmc.TMCVirtualPinHelper(config, mcu_tmc), fields, mcu_tmc


def _helper_2130(options):
    fields = tmc.FieldHelper(
        tmc2130.Fields, tmc2130.SignedFields, tmc2130.FieldFormatters
    )
    mcu_tmc = _FakeMcuTmc(fields)
    config = _FakeConfig("tmc2130 stepper_x", options)
    return tmc.TMCVirtualPinHelper(config, mcu_tmc), fields, mcu_tmc


def test_arm_writes_threshold_and_forces_stealthchop():
    helper, fields, mcu_tmc = _helper_2209(sgthrs=75)
    helper.arm()

    written = {reg for reg, _ in mcu_tmc.writes}
    assert {"SGTHRS", "GCONF", "TPWMTHRS", "TCOOLTHRS"} <= written
    assert fields.get_field("sgthrs") == 75
    assert fields.get_field("en_spreadcycle") == 0
    assert fields.get_field("tpwmthrs") == 0
    assert fields.get_field("tcoolthrs") == 0xFFFFF


def test_disarm_restores_prior_state():
    helper, fields, mcu_tmc = _helper_2209(
        tpwmthrs=120, en_spreadcycle=1, tcoolthrs=0
    )
    helper.arm()
    helper.disarm()

    assert fields.get_field("en_spreadcycle") == 1
    assert fields.get_field("tpwmthrs") == 120
    assert fields.get_field("tcoolthrs") == 0


def test_arm_leaves_nonzero_tcoolthrs_untouched():
    helper, fields, mcu_tmc = _helper_2209(tcoolthrs=0x1234)
    helper.arm()
    assert fields.get_field("tcoolthrs") == 0x1234


def test_arm_on_driver_without_sgthrs_register_skips_threshold_write():
    helper, fields, mcu_tmc = _helper_2130({"diag1_pin": "PA1"})

    helper.arm()

    assert "SGTHRS" not in {reg for reg, _ in mcu_tmc.writes}
    assert fields.get_field("en_pwm_mode") == 0
    assert fields.get_field("diag1_stall") == 1


def test_setup_bridge_endstop_builds_endstop_from_diag_pin():
    helper, fields, mcu_tmc = _helper_2130({"diag1_pin": "^!PG9"})

    endstop = helper.setup_bridge_endstop(_virtual_endstop_params(), 0)

    assert endstop.endstop_id == PROVIDER_ID_FIRST
    assert endstop.pin == "PG9"
    assert endstop.pullup == 1
    assert endstop.invert == 1
    assert helper.setup_bridge_endstop(_virtual_endstop_params(), 0) is endstop


def test_setup_bridge_endstop_rejects_bad_requests():
    helper, fields, mcu_tmc = _helper_2130({"diag1_pin": "PG9"})
    with pytest.raises(pins.error):
        helper.setup_bridge_endstop(_virtual_endstop_params(pin="PA0"), 0)
    with pytest.raises(pins.error):
        helper.setup_bridge_endstop(_virtual_endstop_params(invert=1), 0)

    no_diag, fields, mcu_tmc = _helper_2130({})
    with pytest.raises(pins.error):
        no_diag.setup_bridge_endstop(_virtual_endstop_params(), 0)


def test_trip_move_hooks_arm_and_disarm():
    helper, fields, mcu_tmc = _helper_2209()

    helper.trip_move_begin(entry=None)
    assert fields.get_field("tcoolthrs") == 0xFFFFF
    helper.trip_move_end(entry=None)
    assert fields.get_field("tcoolthrs") == 0

from klippy.extras import tmc, tmc2130, tmc2208, tmc2209


class _FakePins:
    def register_chip(self, name, obj):
        pass


class _FakePrinter:
    def lookup_object(self, name):
        return _FakePins()

    def register_event_handler(self, event, cb):
        pass


class _FakeConfig:
    def __init__(self, name, pins):
        self._name = name
        self._pins = pins

    def get_printer(self):
        return _FakePrinter()

    def get(self, key, default=None):
        return self._pins.get(key, default)

    def getboolean(self, key, default=None):
        return default

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


def _helper_2209(sgthrs=75, tpwmthrs=120, en_spreadcycle=1, tcoolthrs=0):
    fields = tmc.FieldHelper(
        tmc2209.Fields, tmc2208.SignedFields, tmc2209.FieldFormatters
    )
    fields.set_field("sgthrs", sgthrs)
    fields.set_field("tpwmthrs", tpwmthrs)
    fields.set_field("en_spreadcycle", en_spreadcycle)
    fields.set_field("tcoolthrs", tcoolthrs)
    mcu_tmc = _FakeMcuTmc(fields)
    config = _FakeConfig("tmc2209 stepper_x", {"diag_pin": "PA1"})
    return tmc.TMCVirtualPinHelper(config, mcu_tmc), fields, mcu_tmc


def test_arm_writes_threshold_and_forces_stealthchop():
    helper, fields, mcu_tmc = _helper_2209(sgthrs=75)
    helper.arm()

    written = {reg for reg, _ in mcu_tmc.writes}
    assert {"SGTHRS", "GCONF", "TPWMTHRS", "TCOOLTHRS"} <= written
    assert fields.get_field("sgthrs") == 75
    # stallguard4 driver: stealthchop forced on for homing
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
    fields = tmc.FieldHelper(
        tmc2130.Fields, tmc2130.SignedFields, tmc2130.FieldFormatters
    )
    mcu_tmc = _FakeMcuTmc(fields)
    config = _FakeConfig("tmc2130 stepper_x", {"diag1_pin": "PA1"})
    helper = tmc.TMCVirtualPinHelper(config, mcu_tmc)

    helper.arm()

    assert "SGTHRS" not in {reg for reg, _ in mcu_tmc.writes}
    # earlier driver: stealthchop disabled and diag stall routed to the pin
    assert fields.get_field("en_pwm_mode") == 0
    assert fields.get_field("diag1_stall") == 1

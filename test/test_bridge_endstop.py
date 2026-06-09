from klippy.bridge_endstop import (
    PROVIDER_ID_FIRST,
    BridgeEndstop,
    allocate_provider_id,
)


class FakeCommand:
    def __init__(self, response=None):
        self.sent = []
        self.response = response

    def send(self, args):
        self.sent.append(list(args))
        return self.response


class FakeMcu:
    def __init__(self):
        self.oids = 0
        self.config_cmds = []
        self.config_callbacks = []
        self.query_cmd = FakeCommand()
        self.state_cmd = FakeCommand({"oid": 0, "armed": 0, "pin_value": 0})

    def create_oid(self):
        oid = self.oids
        self.oids += 1
        return oid

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def add_config_cmd(self, cmd):
        self.config_cmds.append(cmd)

    def lookup_command(self, template):
        return self.query_cmd

    def lookup_query_command(self, template, response, oid=None):
        return self.state_cmd

    def seconds_to_clock(self, seconds):
        return int(seconds * 1_000_000)


class FakePrinter:
    def __init__(self):
        self.objects = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def _pin_params(mcu, pin="PA8", invert=0, pullup=1):
    return {
        "chip": mcu,
        "chip_name": "mcu",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def _connected(mcu, endstop):
    for cb in mcu.config_callbacks:
        cb()
    return endstop


def test_config_cmd_emitted():
    mcu = FakeMcu()
    _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    assert mcu.config_cmds == [
        "config_endstop oid=0 endstop_id=3 pin=PA8 pull_up=1 invert=0"
    ]


def test_is_triggered_applies_invert():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu, invert=1), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 0}
    assert endstop.is_triggered() is True
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.is_triggered() is False


def test_arm_sends_rest_ticks():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert mcu.query_cmd.sent == [[0, 1000]]


def test_query_endstop_matches_is_triggered():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.query_endstop(0.0) is True


def test_provider_ids_allocate_sequentially():
    printer = FakePrinter()
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 1
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 2

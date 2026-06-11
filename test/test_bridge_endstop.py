import pytest

from klippy.bridge_endstop import (
    PROVIDER_ID_FIRST,
    BridgeEndstop,
    RemoteBridgeEndstop,
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
        self.state_cmd = FakeCommand(
            {"oid": 0, "armed": 0, "pin_value": 0, "tripped": 0, "trip_clock": 0}
        )

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
    assert endstop.query_endstop(0.0) is endstop.is_triggered() is True


def test_arm_zero_period_rejected():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    with pytest.raises(ValueError, match="rest_ticks"):
        endstop.arm(0.0)
    assert mcu.query_cmd.sent == []


def test_query_trip_state_not_tripped():
    mcu = FakeMcu()
    es = BridgeEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    assert es.query_trip_state() == {"tripped": False, "trip_clock": 0}


def test_query_trip_state_tripped_returns_latched_clock():
    mcu = FakeMcu()
    es = BridgeEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    mcu.state_cmd.response = {
        "oid": 0,
        "armed": 0,
        "pin_value": 1,
        "tripped": 1,
        "trip_clock": 0xDEADBEEF,
    }
    assert es.query_trip_state() == {
        "tripped": True,
        "trip_clock": 0xDEADBEEF,
    }


def test_provider_ids_allocate_sequentially():
    printer = FakePrinter()
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 1
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 2


class FakeBridge:
    def __init__(self):
        self.calls = []

    def arm_remote_trigger(self, mcu_handle, trsync_oid, endstop_id):
        self.calls.append(("arm", mcu_handle, trsync_oid, endstop_id))

    def disarm_remote_trigger(self, endstop_id):
        self.calls.append(("disarm", endstop_id))


class FakeRemoteMcu:
    _bridge_handle = 42


def _remote_setup():
    printer = FakePrinter()
    bridge = FakeBridge()
    printer.add_object("motion_bridge", bridge)
    es = RemoteBridgeEndstop(printer, FakeRemoteMcu(), trsync_oid=9)
    return bridge, es


def test_remote_endstop_allocates_provider_id():
    _, es = _remote_setup()
    assert es.endstop_id >= PROVIDER_ID_FIRST


def test_remote_endstop_arm_and_disarm_delegate_to_bridge():
    bridge, es = _remote_setup()
    es.arm(0.001)
    es.disarm()
    assert bridge.calls == [
        ("arm", 42, 9, es.endstop_id),
        ("disarm", es.endstop_id),
    ]


def test_remote_endstop_default_query_state():
    _, es = _remote_setup()
    assert es.is_triggered() is False
    assert es.query_endstop(0.0) is False
    assert es.bridge_mcu_handle() == 42

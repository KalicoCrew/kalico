import pytest

from klippy.extras import servo_param


def test_parse_address():
    assert servo_param.parse_address("0x2002.0") == (0x2002, 0)
    assert servo_param.parse_address("0x6041.0x1F") == (0x6041, 0x1F)


@pytest.mark.parametrize(
    "bad", ["2002", "2002.0", "0x2002.0.1", "0x12345.0", "0x2002.300", "x.y"]
)
def test_parse_address_rejects(bad):
    with pytest.raises(ValueError):
        servo_param.parse_address(bad)


def test_parse_param_entry_probed():
    entry = servo_param.parse_param_entry("0x2002.0: 100")
    assert entry == (0x2002, 0, 0, 100)


def test_parse_param_entry_typed():
    assert servo_param.parse_param_entry("0x2003.0: u16 250") == (
        0x2003,
        0,
        2,
        250,
    )
    assert servo_param.parse_param_entry("0x2010.1: i32 -4096") == (
        0x2010,
        1,
        4,
        -4096,
    )


def test_parse_param_entry_hex_value():
    assert servo_param.parse_param_entry("0x2002.0: u16 0x64") == (
        0x2002,
        0,
        2,
        0x64,
    )


@pytest.mark.parametrize(
    "bad",
    [
        "0x2002.0 100",
        "0x2002.0: u16 -5",
        "0x2002.0: i8 200",
        "0x2002.0: q16 1",
        "0x2002.0: u16 1 2",
        "0x2002.0: 0x1_0000_0000",
    ],
)
def test_parse_param_entry_rejects(bad):
    with pytest.raises(ValueError):
        servo_param.parse_param_entry(bad)


def test_parse_params_block_skips_blanks():
    text = "\n0x2002.0: 100\n\n0x2003.0: u16 250\n"
    assert servo_param.parse_params_block(text) == [
        (0x2002, 0, 0, 100),
        (0x2003, 0, 2, 250),
    ]


def test_format_value_untyped_shows_both_interpretations():
    out = servo_param.format_value(0x2002, 0, 2, 0xFFFE, None)
    assert out == "0x2002.0 = 0xfffe (u16: 65534, i16: -2)"


def test_format_value_typed_shows_one():
    assert (
        servo_param.format_value(0x2002, 0, 2, 0xFFFE, "i16")
        == "0x2002.0 = 0xfffe (i16: -2)"
    )
    assert (
        servo_param.format_value(0x2010, 1, 4, 100, "u32")
        == "0x2010.1 = 0x00000064 (u32: 100)"
    )


from klippy.extras import servo_axis


class FakeGcmd:
    error = RuntimeError

    def __init__(self, params):
        self.params = params
        self.responses = []

    def get(self, name, default=KeyError):
        if name in self.params:
            return self.params[name]
        if default is KeyError:
            raise RuntimeError("missing param %s" % (name,))
        return default

    def respond_info(self, msg):
        self.responses.append(msg)


class FakeBridge:
    def __init__(self):
        self.reads = []
        self.writes = []
        self.read_result = (2, 100)
        self.write_result = (2, 100)

    def sdo_read(self, handle, index, subindex):
        self.reads.append((handle, index, subindex))
        return self.read_result

    def sdo_write(self, handle, index, subindex, size, value):
        self.writes.append((handle, index, subindex, size, value))
        return self.write_result


class FakeNode:
    name = "node_x"

    def __init__(self, handle):
        self._h = handle

    def get_bridge_handle(self):
        return self._h


class FakeKin:
    def __init__(self, rails):
        self.rails = rails


class FakeToolhead:
    def __init__(self, kin):
        self.kin = kin

    def get_kinematics(self):
        return self.kin


class FakePrinter:
    command_error = RuntimeError

    def __init__(self, objs):
        self._objs = objs

    def lookup_object(self, name):
        return self._objs[name]


def make_servo_param(bridge, node):
    rail = servo_axis.ServoRail.__new__(servo_axis.ServoRail)
    rail.name = "servo_x"
    rail.axis = "x"
    rail.node_name = "node_x"
    sp = servo_param.ServoParam.__new__(servo_param.ServoParam)
    sp.printer = FakePrinter(
        {
            "toolhead": FakeToolhead(FakeKin([rail])),
            "ethercat_node node_x": node,
            "motion_bridge": bridge,
        }
    )
    return sp


def test_cmd_get_reads_and_formats():
    bridge = FakeBridge()
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd({"SERVO": "servo_x", "GET": "0x2002.0"})
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.reads == [(7, 0x2002, 0)]
    assert gcmd.responses == ["0x2002.0 = 0x0064 (u16: 100, i16: 100)"]


def test_cmd_set_typed_passes_size():
    bridge = FakeBridge()
    bridge.write_result = (2, 250)
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd(
        {"SERVO": "servo_x", "SET": "0x2002.0", "VALUE": "250", "TYPE": "u16"}
    )
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.writes == [(7, 0x2002, 0, 2, 250)]
    assert gcmd.responses == ["set 0x2002.0 = 0x00fa (u16: 250)"]


def test_cmd_set_untyped_passes_size_zero():
    bridge = FakeBridge()
    sp = make_servo_param(bridge, FakeNode(7))
    gcmd = FakeGcmd({"SERVO": "servo_x", "SET": "0x2002.0", "VALUE": "100"})
    sp.cmd_SERVO_PARAM(gcmd)
    assert bridge.writes == [(7, 0x2002, 0, 0, 100)]


def test_cmd_requires_exactly_one_of_get_set():
    sp = make_servo_param(FakeBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="exactly one"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_x"}))
    with pytest.raises(RuntimeError, match="exactly one"):
        sp.cmd_SERVO_PARAM(
            FakeGcmd(
                {
                    "SERVO": "servo_x",
                    "GET": "0x2002.0",
                    "SET": "0x2002.0",
                    "VALUE": "1",
                }
            )
        )


def test_cmd_fails_without_bridge_handle():
    sp = make_servo_param(FakeBridge(), FakeNode(None))
    with pytest.raises(RuntimeError, match="no bridge handle"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_x", "GET": "0x2002.0"}))


def test_cmd_unknown_servo_fails():
    sp = make_servo_param(FakeBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="no servo rail"):
        sp.cmd_SERVO_PARAM(FakeGcmd({"SERVO": "servo_q", "GET": "0x2002.0"}))


def test_cmd_propagates_bridge_failure():
    class FailingBridge(FakeBridge):
        def sdo_write(self, *args):
            raise RuntimeError("CoE abort 0x06010002")

    sp = make_servo_param(FailingBridge(), FakeNode(7))
    with pytest.raises(RuntimeError, match="CoE abort"):
        sp.cmd_SERVO_PARAM(
            FakeGcmd({"SERVO": "servo_x", "SET": "0x6041.0", "VALUE": "1"})
        )

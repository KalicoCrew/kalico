from klippy.extras import neopixel


class _FakeMutex:
    def __enter__(self):
        return self

    def __exit__(self, *_):
        pass


class _FakeReactor:
    def mutex(self):
        return _FakeMutex()


class _FakeMcu:
    def __init__(self, non_critical_disconnected=False):
        self.non_critical_disconnected = non_critical_disconnected

    def create_oid(self):
        return 0

    def register_config_callback(self, cb):
        pass

    def get_non_critical_reconnect_event_name(self):
        return "danger:non_critical_mcu_pixel:reconnected"


class _FakePins:
    def __init__(self, mcu):
        self._mcu = mcu

    def lookup_pin(self, pin):
        return {"chip": self._mcu, "pin": pin}


class _FakePrinter:
    def __init__(self, mcu):
        self._objects = {"pins": _FakePins(mcu)}
        self.event_handlers = {}

    def get_reactor(self):
        return _FakeReactor()

    def lookup_object(self, name):
        return self._objects[name]

    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)

    def get_start_args(self):
        return {}


class _FakeConfig:
    error = Exception

    def __init__(self, printer):
        self._printer = printer

    def get_printer(self):
        return self._printer

    def get(self, name):
        assert name == "pin"
        return "pixel:PA1"

    def getint(self, name, default, **kw):
        return default

    def getlist(self, name, default):
        return default


class _FakeLEDHelper:
    def __init__(self, config, update_func, count):
        pass

    def get_status(self, eventtime=None):
        return {"color_data": [(0.0, 0.0, 0.0, 0.0)]}


def _make_neopixel(monkeypatch, mcu):
    monkeypatch.setattr(neopixel.led, "LEDHelper", _FakeLEDHelper)
    printer = _FakePrinter(mcu)
    return neopixel.PrinterNeoPixel(_FakeConfig(printer)), printer


def test_send_data_skips_disconnected_non_critical_mcu(monkeypatch):
    mcu = _FakeMcu(non_critical_disconnected=True)
    np, _ = _make_neopixel(monkeypatch, mcu)
    # build_config never ran (mcu disconnected), so the commands are None;
    # klippy:connect must not crash with a NoneType .send AttributeError
    assert np.neopixel_update_cmd is None
    assert np.color_data != np.old_color_data
    np.send_data()


def test_reconnect_event_resends_led_data(monkeypatch):
    mcu = _FakeMcu()
    np, printer = _make_neopixel(monkeypatch, mcu)
    reconnect_event = mcu.get_non_critical_reconnect_event_name()
    assert printer.event_handlers.get(reconnect_event) == [np.send_data]

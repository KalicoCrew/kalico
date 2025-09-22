from klippy import mcu as mcu_mod

RECON_EVENT = "danger:non_critical_mcu_test:reconnected"


class _FakeSerial:
    def __init__(self):
        self.disconnected = False

    def disconnect(self):
        self.disconnected = True


class _FakeClockSync:
    def __init__(self):
        self.disconnected = False

    def disconnect(self):
        self.disconnected = True


class _FakePrinter:
    def __init__(self):
        self.events = []

    def send_event(self, name, *args):
        self.events.append(name)


class _FakeReactor:
    NOW = 0.0
    NEVER = 9e99

    def monotonic(self):
        return 0.0

    def pause(self, waketime):
        pass


class _FakeConfigHelper:
    def reset_to_initial_state(self):
        pass

    def connect_config(self):
        pass


class _FakeCommand:
    def __init__(self, response=None):
        self.sent = 0
        self._response = response

    def send(self, *args, **kwargs):
        self.sent += 1
        return self._response


def _make_mcu(is_shutdown_response=0):
    mcu = mcu_mod.MCU.__new__(mcu_mod.MCU)
    conn = mcu_mod.MCUConnectHelper.__new__(mcu_mod.MCUConnectHelper)
    mcu._name = conn._name = "test"
    mcu._printer = conn._printer = _FakePrinter()
    mcu._reactor = conn._reactor = _FakeReactor()
    mcu._serial = conn._serial = _FakeSerial()
    mcu._clocksync = conn._clocksync = _FakeClockSync()
    mcu._conn_helper = conn
    conn._mcu = mcu
    conn._is_shutdown = False
    conn._shutdown_msg = ""
    restart = mcu_mod.MCURestartHelper.__new__(mcu_mod.MCURestartHelper)
    restart._mcu = mcu
    restart._name = "test"
    restart._reactor = conn._reactor
    restart._reset_cmd = None
    restart._config_reset_cmd = None
    conn._restart_helper = restart
    mcu.non_critical_disconnected = True
    mcu._stats_helper = mcu_mod.MCUStatsHelper.__new__(mcu_mod.MCUStatsHelper)
    mcu._stats_helper._get_status_info = {}
    mcu._non_critical_reconnect_event_name = RECON_EVENT
    # Fake out the recon path boundaries; recon_mcu itself is the unit
    mcu._mcu_identify = lambda: True
    mcu._config_helper = _FakeConfigHelper()
    get_config = _FakeCommand(
        {
            "is_config": 1,
            "crc": 0,
            "is_shutdown": is_shutdown_response,
            "move_count": 500,
        }
    )
    mcu.lookup_query_command = lambda msg, resp: get_config
    return mcu


def test_failed_connect_does_not_propagate():
    mcu = _make_mcu()

    def _boom():
        raise mcu_mod.error("Failed automated reset of MCU 'test'")

    mcu._config_helper.connect_config = _boom
    assert mcu.recon_mcu() is False
    assert mcu.non_critical_disconnected is True
    assert mcu._serial.disconnected is True
    assert RECON_EVENT not in mcu._printer.events


def test_shutdown_mcu_gets_reset_and_retries():
    mcu = _make_mcu(is_shutdown_response=1)
    mcu._conn_helper._restart_helper._reset_cmd = _FakeCommand()
    assert mcu.recon_mcu() is False
    assert mcu._conn_helper._restart_helper._reset_cmd.sent == 1
    assert mcu._serial.disconnected is True
    assert mcu.non_critical_disconnected is True
    assert RECON_EVENT not in mcu._printer.events


def test_shutdown_mcu_clear_shutdown_fallback():
    mcu = _make_mcu(is_shutdown_response=1)
    clear_cmd = _FakeCommand()
    mcu.try_lookup_command = lambda name: clear_cmd
    assert mcu.recon_mcu() is False
    assert clear_cmd.sent == 1
    assert mcu._serial.disconnected is True


def test_shutdown_skips_disconnected_non_critical_mcu():
    mcu = _make_mcu()
    conn = mcu._conn_helper
    conn._emergency_stop_cmd = _FakeCommand()
    mcu.non_critical_disconnected = True
    conn._shutdown()
    assert conn._emergency_stop_cmd.sent == 0


def test_successful_recon_clears_stale_shutdown_flag():
    mcu = _make_mcu()
    mcu._conn_helper._is_shutdown = True
    assert mcu.recon_mcu() is True
    assert mcu._conn_helper._is_shutdown is False
    assert mcu.non_critical_disconnected is False
    assert mcu._printer.events == [RECON_EVENT]

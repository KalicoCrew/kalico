from klippy.extras import tmc


class _CommandError(Exception):
    pass


class _FakeMcu:
    non_critical_disconnected = False


class _FakeMcuTmc:
    def __init__(self, mcu):
        self.mcu = mcu

    def get_mcu(self):
        return self.mcu


class _FakePrinter:
    command_error = _CommandError

    def __init__(self):
        self.shutdowns = []

    def invoke_shutdown(self, msg):
        self.shutdowns.append(msg)

    def get_reactor(self):
        return self

    NEVER = 9e99


def _make_check(query_error=None):
    check = tmc.TMCErrorCheck.__new__(tmc.TMCErrorCheck)
    check.printer = _FakePrinter()
    check.mcu = _FakeMcu()
    check.mcu_tmc = _FakeMcuTmc(check.mcu)
    check.mcu_offline = False
    check.drv_status_reg_info = check.gstat_reg_info = None
    check.adc_temp_reg = None
    check.queries = 0

    def _query_register(reg_info):
        check.queries += 1
        if query_error is not None:
            raise _CommandError(query_error)

    check._query_register = _query_register
    return check


def test_skips_until_the_reconnect_event():
    # non_critical_disconnected is cleared before the config is resent
    check = _make_check()
    check._handle_mcu_offline()
    assert check._do_periodic_check(10.0) == 11.0
    assert check.queries == 0
    check._handle_mcu_online()
    assert check._do_periodic_check(20.0) == 21.0
    assert check.queries == 1


def test_mcu_drop_mid_query_does_not_shut_down():
    check = _make_check()

    def _query_register(reg_info):
        check.mcu.non_critical_disconnected = True
        raise _CommandError("Serial connection closed")

    check._query_register = _query_register
    assert check._do_periodic_check(10.0) == 11.0
    assert check.printer.shutdowns == []


def test_driver_fault_on_connected_mcu_still_shuts_down():
    check = _make_check(query_error="TMC reports error")
    check._do_periodic_check(10.0)
    assert check.printer.shutdowns == ["TMC reports error"]

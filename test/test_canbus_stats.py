from klippy import gcode
from klippy.extras import canbus_stats


class _FakeReactor:
    def monotonic(self):
        return 100.0


class _FakeMcu:
    non_critical_disconnected = False


class _FakePrinter:
    command_error = gcode.CommandError


class _RaisingCmd:
    def send(self, *args, **kwargs):
        raise gcode.CommandError("mcu 'sht36': Serial connection closed")


def test_query_event_survives_send_error():
    stats = canbus_stats.PrinterCANBusStats.__new__(
        canbus_stats.PrinterCANBusStats
    )
    stats.printer = _FakePrinter()
    stats.reactor = _FakeReactor()
    stats.name = "sht36"
    stats.mcu = _FakeMcu()
    stats.get_canbus_status_cmd = _RaisingCmd()
    stats.status = {
        "rx_error": 1,
        "tx_error": 2,
        "tx_retries": 3,
        "bus_state": "active",
    }
    # A disconnect while the query is in flight must not propagate out of
    # the reactor timer callback
    waketime = stats.query_event(100.0)
    assert waketime == 101.0
    assert stats.status["rx_error"] == 1

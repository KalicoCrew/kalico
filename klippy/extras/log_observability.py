import logging

from .. import structured_log

HEARTBEAT_INTERVAL = 30.0
LAG_CHECK_INTERVAL = 60.0
LAG_THRESHOLD_BYTES = 8 * 1024 * 1024


def emit_heartbeat():
    structured_log.event(
        "observability",
        "heartbeat",
        level=logging.INFO,
        msg="pipeline heartbeat",
    )


def check_lag(bytes_behind, threshold=LAG_THRESHOLD_BYTES):
    return bytes_behind > threshold


class LogObservability:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.events_dir = self.printer.get_start_args().get("log_events_dir")
        self._last_stale = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)

    def _handle_ready(self):
        now = self.reactor.monotonic()
        self.reactor.register_timer(
            self._heartbeat_timer, now + HEARTBEAT_INTERVAL
        )
        self.reactor.register_timer(self._lag_timer, now + LAG_CHECK_INTERVAL)

    def _heartbeat_timer(self, eventtime):
        emit_heartbeat()
        return eventtime + HEARTBEAT_INTERVAL

    def _vector_bytes_behind(self):
        return None

    def _lag_timer(self, eventtime):
        behind = self._vector_bytes_behind()
        if behind is not None:
            stale = check_lag(behind)
            if stale and not self._last_stale:
                logging.warning(
                    "observability: Vector shipper lagging %d bytes behind "
                    "events files — logs may not be reaching VictoriaLogs",
                    behind,
                )
                structured_log.event(
                    "observability",
                    "shipper_lag",
                    level=logging.WARNING,
                    msg="vector shipper lagging",
                    bytes_behind=behind,
                )
            self._last_stale = stale
        return eventtime + LAG_CHECK_INTERVAL


def load_config(config):
    return LogObservability(config)

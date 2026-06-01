# Host self-observability for the structured-logging pipeline (spec §12).
#
# Because "the store is down" is exactly the case that cannot self-report, the
# host emits a periodic heartbeat record (queryable to confirm end-to-end
# health) and checks that the shipper (Vector) is keeping up (checkpoint lag
# bounded). A silent pipeline stall is itself a reportable fault — surfaced
# loudly via a text-log warning plus a queryable observability event.
#
# Loaded on-by-default (printer.py extras list); no user config required.
import logging
import os

from .. import structured_log

HEARTBEAT_INTERVAL = 30.0  # seconds between heartbeats (tunable)
LAG_CHECK_INTERVAL = 60.0  # seconds between lag checks
LAG_THRESHOLD_BYTES = 8 * 1024 * 1024  # bytes-behind-EOF before "stale"


def emit_heartbeat():
    # One structured heartbeat record. Querying for it confirms the whole
    # path (emit -> jsonl -> Vector -> VL) is live.
    structured_log.event(
        "observability",
        "heartbeat",
        level=logging.INFO,
        msg="pipeline heartbeat",
    )


def check_lag(bytes_behind, threshold=LAG_THRESHOLD_BYTES):
    # Pure predicate: True == stale (lag strictly exceeds threshold). Kept
    # side-effect free so it is unit-testable; the component wraps it with
    # logging.
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
        # Best-effort: how far the shipper is behind the events files. Returns
        # the byte gap, or None if checkpoint state is unavailable (Vector not
        # installed / not running) — None is treated as "unknown", not stale.
        #
        # The concrete Vector-checkpoint diff is wired against the deployed
        # Vector data_dir layout on the printer (a Trident-only follow-up in
        # the Stage 3 plan). Until then this returns None so the heartbeat is
        # the active liveness signal and no false "stale" is raised.
        if not self.events_dir or not os.path.isdir(self.events_dir):
            return None
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

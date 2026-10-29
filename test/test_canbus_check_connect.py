import logging
import sys
import types

import pytest

from klippy import serialhdl

UUID = "0123456789ab"
OTHER_UUID = "ba9876543210"
CANBUS_ID_ADMIN_RESP = 0x3F0 + 1
RESP_NEED_NODEID = 0x20
RESP_HAVE_NODEID = 0x21


class _CanError(Exception):
    pass


class _Message:
    def __init__(self, arbitration_id=0, data=(), is_extended_id=False):
        self.arbitration_id = arbitration_id
        self.data = list(data)
        self.dlc = len(self.data)
        self.is_extended_id = is_extended_id


class _FakeReactor:
    # Virtual clock; pause() is where other reactor work would run
    def __init__(self):
        self.now = 100.0
        self.pauses = []

    def monotonic(self):
        return self.now

    def pause(self, waketime):
        self.pauses.append(waketime - self.now)
        self.now = max(self.now, waketime)
        return self.now


class _FakeBus:
    def __init__(
        self, reactor, replies=(), send_error=None, recv_error_at=None
    ):
        self.reactor = reactor
        # List of (relative time, message) the node "answers" with
        self.start = reactor.now
        self.replies = list(replies)
        self.send_error = send_error
        self.recv_error_at = recv_error_at
        self.sent = []
        self.recv_timeouts = []
        self.blocked_time = 0.0
        self.is_shutdown = False

    def send(self, msg):
        if self.send_error is not None:
            raise self.send_error
        self.sent.append(msg)

    def recv(self, timeout=None):
        self.recv_timeouts.append(timeout)
        elapsed = self.reactor.now - self.start
        if self.recv_error_at is not None and elapsed >= self.recv_error_at:
            raise OSError(100, "Network is down")
        if self.replies and self.replies[0][0] <= elapsed:
            return self.replies.pop(0)[1]
        if timeout:
            # A blocking recv() holds the reactor thread until a reply
            # arrives or the timeout expires
            if self.replies and self.replies[0][0] - elapsed <= timeout:
                reply_time, msg = self.replies.pop(0)
                self.blocked_time += reply_time - elapsed
                self.reactor.now = self.start + reply_time
                return msg
            self.blocked_time += timeout
            self.reactor.now += timeout
        return None

    def shutdown(self):
        self.is_shutdown = True


@pytest.fixture
def can_env(monkeypatch):
    reactor = _FakeReactor()
    env = types.SimpleNamespace(reactor=reactor, buses=[], bus_factory=None)

    def make_bus(channel=None, can_filters=None, bustype=None):
        bus = env.bus_factory()
        env.buses.append(bus)
        return bus

    fake_can = types.ModuleType("can")
    fake_can.CanError = _CanError
    fake_can.Message = _Message
    fake_can.interface = types.SimpleNamespace(Bus=make_bus)
    monkeypatch.setitem(sys.modules, "can", fake_can)
    env.reader = serialhdl.SerialReader(reactor, warn_prefix="mcu 'mmu': ")
    return env


def _reply(uuid, cmd=RESP_NEED_NODEID):
    raw = int(uuid, 16)
    data = [cmd] + [(raw >> (40 - i * 8)) & 0xFF for i in range(6)] + [0]
    return _Message(CANBUS_ID_ADMIN_RESP, data)


def _check(env):
    return env.reader.check_canbus_connect(UUID, 4, "can0")


def test_no_reply_does_not_block_reactor(can_env):
    can_env.bus_factory = lambda: _FakeBus(can_env.reactor)
    start = can_env.reactor.now
    assert _check(can_env) is False
    bus = can_env.buses[0]
    assert bus.blocked_time == 0.0
    assert set(bus.recv_timeouts) == {0.0}
    # Same 1 s listening window as before, but spent in reactor pauses of
    # at most CANBUS_CHECK_POLL_TIME each
    assert can_env.reactor.now - start == pytest.approx(
        serialhdl.CANBUS_CHECK_TIME
    )
    assert max(can_env.reactor.pauses) <= serialhdl.CANBUS_CHECK_POLL_TIME
    assert len(can_env.reactor.pauses) >= 100
    assert bus.is_shutdown


def test_matching_uuid_reply_connects(can_env):
    for cmd in (RESP_NEED_NODEID, RESP_HAVE_NODEID):
        can_env.bus_factory = lambda: _FakeBus(
            can_env.reactor, replies=[(0.3, _reply(UUID, cmd))]
        )
        start = can_env.reactor.now
        assert _check(can_env) is True
        bus = can_env.buses[-1]
        assert bus.blocked_time == 0.0
        # Returns as soon as the reply is seen, not after the full window
        assert can_env.reactor.now - start == pytest.approx(0.3, abs=0.006)
        assert bus.is_shutdown
        assert bus.sent[0].data == [0x00, 0x01]


def test_other_nodes_and_invalid_frames_are_ignored(can_env):
    short = _Message(CANBUS_ID_ADMIN_RESP, [RESP_NEED_NODEID, 1, 2])
    wrong_id = _reply(UUID)
    wrong_id.arbitration_id = 0x123
    wrong_cmd = _reply(UUID, cmd=0x55)
    can_env.bus_factory = lambda: _FakeBus(
        can_env.reactor,
        replies=[
            (0.1, _reply(OTHER_UUID)),
            (0.2, short),
            (0.3, wrong_id),
            (0.4, wrong_cmd),
        ],
    )
    assert _check(can_env) is False
    assert can_env.buses[0].replies == []
    assert can_env.buses[0].is_shutdown


def test_reply_after_other_node_reply_connects(can_env):
    can_env.bus_factory = lambda: _FakeBus(
        can_env.reactor,
        replies=[(0.1, _reply(OTHER_UUID)), (0.5, _reply(UUID))],
    )
    assert _check(can_env) is True


def test_reply_after_window_is_not_seen(can_env):
    can_env.bus_factory = lambda: _FakeBus(
        can_env.reactor, replies=[(1.5, _reply(UUID))]
    )
    assert _check(can_env) is False


def test_missing_bus_interface(can_env, caplog):
    def no_bus():
        raise OSError(19, "No such device")

    can_env.bus_factory = no_bus
    with caplog.at_level(logging.WARNING):
        assert _check(can_env) is False
    assert can_env.reactor.pauses == []
    assert "can issue" in caplog.text


def test_send_error_closes_bus(can_env):
    can_env.bus_factory = lambda: _FakeBus(
        can_env.reactor, send_error=_CanError("Transmit buffer full")
    )
    assert _check(can_env) is False
    assert can_env.buses[0].is_shutdown


def test_bus_going_down_during_check(can_env):
    can_env.bus_factory = lambda: _FakeBus(can_env.reactor, recv_error_at=0.2)
    assert _check(can_env) is False
    assert can_env.buses[0].is_shutdown
    assert can_env.buses[0].blocked_time == 0.0


def test_repeated_attempts_until_node_appears(can_env):
    # Reconnect timer: the node is absent for three attempts, then answers
    attempts = iter([[], [], [], [(0.05, _reply(UUID))]])
    can_env.bus_factory = lambda: _FakeBus(
        can_env.reactor, replies=next(attempts)
    )
    results = []
    for _ in range(4):
        results.append(_check(can_env))
        can_env.reactor.now += 1.12  # rest of the reconnect interval
    assert results == [False, False, False, True]
    assert len(can_env.buses) == 4
    assert all(bus.is_shutdown for bus in can_env.buses)
    assert sum(bus.blocked_time for bus in can_env.buses) == 0.0


def test_invalid_uuid_still_raises(can_env):
    with pytest.raises(serialhdl.error, match="Invalid CAN uuid"):
        can_env.reader.check_canbus_connect("xyz", 4, "can0")

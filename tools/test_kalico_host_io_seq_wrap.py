#!/usr/bin/env python3
import json
import pathlib
import sys
import threading
import zlib

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import kalico_host_io  # noqa: E402
import msgproto  # noqa: E402


def _frame(seq, payload=b""):
    msglen = msgproto.MESSAGE_MIN + len(payload)
    seq_byte = (seq & msgproto.MESSAGE_SEQ_MASK) | msgproto.MESSAGE_DEST
    out = [msglen, seq_byte] + list(payload)
    out.extend(msgproto.crc16_ccitt(out))
    out.append(msgproto.MESSAGE_SYNC)
    return bytes(out)


class _ReadSerial:
    def __init__(self, chunks):
        self._chunks = list(chunks)
        self.timeout = 0.0

    def read(self, _count):
        if self._chunks:
            return self._chunks.pop(0)
        return b""


def test_stale_empty_ack_does_not_rewind_at_seq_wrap():
    io = kalico_host_io.KalicoHostIO.__new__(kalico_host_io.KalicoHostIO)
    io._parser = msgproto.MessageParser()
    io._rxbuf = kalico_host_io._RxBuffer(io._parser)
    io._seq = 0
    io._lock = threading.Lock()
    io._ser = _ReadSerial([_frame(15)])

    params = io._wait_packet_sync(
        "identify_response",
        kalico_host_io.time.monotonic() + 0.01,
        sync_seq=True,
        sent_seq=15,
    )

    assert params is None
    assert io._seq == 0


def test_empty_identify_response_is_parsed_not_classified_as_nak():
    parser = msgproto.MessageParser()
    payload = parser.messages_by_name["identify_response"].encode_by_name(
        offset=2080, data=b""
    )
    io = kalico_host_io.KalicoHostIO.__new__(kalico_host_io.KalicoHostIO)
    io._parser = parser
    io._rxbuf = kalico_host_io._RxBuffer(parser)
    io._seq = 0
    io._lock = threading.Lock()
    io._ser = _ReadSerial([_frame(1, payload)])

    params = io._wait_packet_sync(
        "identify_response",
        kalico_host_io.time.monotonic() + 0.01,
        sync_seq=True,
        sent_seq=0,
    )

    assert params["offset"] == 2080
    assert params["data"] == b""
    assert io._seq == 1


class _FakeMcuSerial:
    def __init__(
        self,
        expected_seq=0,
        pad_len=1600,
        past_end="empty",
        drop_every_response=None,
    ):
        pad = "".join(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789"[
                (i * 37 + pad_len) % 62
            ]
            for i in range(pad_len)
        )
        dictionary = {
            "commands": {"identify offset=%u count=%c": 1},
            "responses": {"identify_response offset=%u data=%.*s": 0},
            "output": {},
            "config": {"PAD": pad},
            "enumerations": {},
            "app": "fake",
            "version": "test",
            "build_versions": "",
        }
        self._identify = zlib.compress(json.dumps(dictionary).encode())
        self._parser = msgproto.MessageParser()
        self._expected_seq = expected_seq & msgproto.MESSAGE_SEQ_MASK
        self._past_end = past_end
        self._drop_every_response = drop_every_response
        self._response_count = 0
        self.requests = []
        self.sent_sequences = []
        self._rx = bytearray()
        self.timeout = 0.0
        self.closed = False

    def write(self, data):
        pkt = bytes(data)
        seq = pkt[msgproto.MESSAGE_POS_SEQ] & msgproto.MESSAGE_SEQ_MASK
        self.sent_sequences.append(seq)
        if seq != self._expected_seq:
            self._rx.extend(_frame(self._expected_seq))
            return len(data)

        params = self._parser.parse(pkt)
        if params["#name"] == "identify":
            offset = params["offset"]
            count = params["count"]
            self.requests.append((offset, count))
            self._response_count += 1
            if (
                self._drop_every_response
                and self._response_count % self._drop_every_response == 0
            ):
                return len(data)
            self._expected_seq = (seq + 1) & msgproto.MESSAGE_SEQ_MASK
            if offset >= len(self._identify) and self._past_end == "silent":
                self._rx.extend(_frame(self._expected_seq))
                return len(data)
            chunk = self._identify[offset : offset + count]
            payload = self._parser.messages_by_name[
                "identify_response"
            ].encode_by_name(offset=offset, data=chunk)
            self._rx.extend(_frame(self._expected_seq, payload))
            self._rx.extend(_frame(self._expected_seq))
        return len(data)

    def read(self, count):
        if not self._rx:
            return b""
        out = bytes(self._rx[:count])
        del self._rx[:count]
        return out

    def flush(self):
        pass

    def close(self):
        self.closed = True


class _FakeSerialModule:
    SerialException = Exception

    def __init__(
        self,
        expected_seq,
        pad_len=1600,
        past_end="empty",
        drop_every_response=None,
    ):
        self.expected_seq = expected_seq
        self.pad_len = pad_len
        self.past_end = past_end
        self.drop_every_response = drop_every_response
        self.last_serial = None

    def Serial(self, _port, _baud, timeout=0.1):
        ser = _FakeMcuSerial(
            self.expected_seq,
            self.pad_len,
            self.past_end,
            self.drop_every_response,
        )
        ser.timeout = timeout
        self.last_serial = ser
        return ser


class _NoEofDecompressor:
    eof = False

    def decompress(self, _data):
        return b""


def test_identify_sweeps_seq_after_host_restart(monkeypatch):
    monkeypatch.setattr(kalico_host_io, "serial", _FakeSerialModule(4))

    io = kalico_host_io.KalicoHostIO("fake-port", identify_timeout=2.0)
    try:
        assert io.get_msgparser().get_app_info() == "fake"
    finally:
        io.disconnect()


def test_identify_terminates_on_partial_final_chunk(monkeypatch):
    fake_serial = _FakeSerialModule(0, pad_len=55, past_end="silent")
    monkeypatch.setattr(kalico_host_io, "serial", fake_serial)
    monkeypatch.setattr(
        kalico_host_io.zlib,
        "decompressobj",
        lambda: _NoEofDecompressor(),
    )

    io = kalico_host_io.KalicoHostIO("fake-port", identify_timeout=2.0)
    try:
        assert io.get_msgparser().get_app_info() == "fake"
        assert fake_serial.last_serial.requests[-1][0] == 200
        assert 213 not in [offset for offset, _count in fake_serial.last_serial.requests]
    finally:
        io.disconnect()


def test_identify_terminates_on_empty_response(monkeypatch):
    fake_serial = _FakeSerialModule(0, pad_len=9, past_end="empty")
    monkeypatch.setattr(kalico_host_io, "serial", fake_serial)
    monkeypatch.setattr(
        kalico_host_io.zlib,
        "decompressobj",
        lambda: _NoEofDecompressor(),
    )

    io = kalico_host_io.KalicoHostIO("fake-port", identify_timeout=2.0)
    try:
        assert io.get_msgparser().get_app_info() == "fake"
        assert fake_serial.last_serial.requests[-1][0] == 160
    finally:
        io.disconnect()


def test_identify_uses_zlib_eof_before_silent_past_end(monkeypatch):
    fake_serial = _FakeSerialModule(0, pad_len=9, past_end="silent")
    monkeypatch.setattr(kalico_host_io, "serial", fake_serial)

    io = kalico_host_io.KalicoHostIO("fake-port", identify_timeout=2.0)
    try:
        assert io.get_msgparser().get_app_info() == "fake"
        assert fake_serial.last_serial.requests[-1][0] == 120
        assert 160 not in [offset for offset, _count in fake_serial.last_serial.requests]
    finally:
        io.disconnect()


def test_identify_retransmits_same_seq_once_after_dropped_response(monkeypatch):
    fake_serial = _FakeSerialModule(0, pad_len=2600, drop_every_response=2)
    monkeypatch.setattr(kalico_host_io, "serial", fake_serial)

    io = kalico_host_io.KalicoHostIO("fake-port", identify_timeout=2.0)
    try:
        assert io.get_msgparser().get_app_info() == "fake"
        requests = fake_serial.last_serial.requests
        duplicate_offsets = [
            offset
            for (offset, _count), (next_offset, _next_count) in zip(
                requests, requests[1:]
            )
            if offset == next_offset
        ]
        assert duplicate_offsets
        for idx, ((offset, _count), (next_offset, _next_count)) in enumerate(
            zip(requests, requests[1:])
        ):
            if offset == next_offset:
                assert (
                    fake_serial.last_serial.sent_sequences[idx]
                    == fake_serial.last_serial.sent_sequences[idx + 1]
                )
    finally:
        io.disconnect()

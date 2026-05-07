"""Contract test for chip_socket_server: clients connect via Unix socket,
send arbitrary bytes, the registered handler returns its reply bytes
back. Wire layout is fully driven by the chip emulator — the server is
just a synchronous request/response framer over the socket."""
import os
import socket
import time
import pytest

from tools.sim_klippy.orchestrator.chip_socket_server import (
    ChipSocketServer,
)


def test_echo_handler_round_trips():
    sock_path = "/tmp/test_chip_socket_echo"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def echo_handler(req: bytes) -> bytes:
        return req[::-1]  # reverse, so we can tell handler ran

    server = ChipSocketServer(sock_path, echo_handler, chunk=4)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(b"\x01\x02\x03\x04")
        reply = client.recv(4)
        assert reply == b"\x04\x03\x02\x01"
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)


def test_handler_with_empty_reply_does_not_send():
    """For TMC2209 writes (8-byte request, no reply), handler returns b''.
    Server must not send anything in that case."""
    sock_path = "/tmp/test_chip_socket_no_reply"
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    def silent_handler(req: bytes) -> bytes:
        return b""

    server = ChipSocketServer(sock_path, silent_handler, chunk=8)
    server.start()
    try:
        for _ in range(50):
            if os.path.exists(sock_path):
                break
            time.sleep(0.01)
        client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        client.connect(sock_path)
        client.sendall(b"\x05\x00\x80\x00\x00\x00\x05\x42")
        client.settimeout(0.2)
        try:
            data = client.recv(1)
            assert data == b"", f"unexpected reply: {data!r}"
        except socket.timeout:
            pass  # expected — no reply
        client.close()
    finally:
        server.stop()
        if os.path.exists(sock_path):
            os.unlink(sock_path)

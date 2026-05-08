"""Async-ish Unix-socket server for chip emulators.

Accepts one client per socket; each request is a fixed-length byte
sequence the handler interprets per-chip. The handler returns a reply
of equal length (TMC SPI is symmetric; TMC2209 UART writes are 8-byte
no-reply but reads are 8-byte reply — the chip emulators handle the
asymmetry by reading the request first and constructing the reply
accordingly).

Threaded model: one accept thread, one worker thread per connection.
Sufficient for our handful of chip stubs."""
import os
import socket
import threading
from typing import Callable


class ChipSocketServer:
    def __init__(self, path: str, handler: Callable[[bytes], bytes],
                 chunk: int = 16):
        self._path = path
        self._handler = handler
        self._chunk = chunk
        self._sock = None
        self._accept_thread = None
        self._stop = threading.Event()

    def start(self) -> None:
        if os.path.exists(self._path):
            os.unlink(self._path)
        self._sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self._sock.bind(self._path)
        self._sock.listen(4)
        self._sock.settimeout(0.1)
        self._accept_thread = threading.Thread(
            target=self._accept_loop, daemon=True
        )
        self._accept_thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
        if self._accept_thread:
            self._accept_thread.join(timeout=1.0)

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                client, _ = self._sock.accept()
            except (socket.timeout, OSError):
                continue
            t = threading.Thread(
                target=self._serve, args=(client,), daemon=True,
            )
            t.start()

    def _serve(self, client: socket.socket):
        client.settimeout(1.0)
        try:
            while not self._stop.is_set():
                data = client.recv(self._chunk)
                if not data:
                    break
                reply = self._handler(data)
                if reply:
                    client.sendall(reply)
        except (socket.timeout, ConnectionResetError, OSError):
            pass
        finally:
            try:
                client.close()
            except OSError:
                pass

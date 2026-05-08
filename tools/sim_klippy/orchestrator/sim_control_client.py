"""Client for the LD_PRELOAD shim's control socket.

Wire format: line-oriented text. See
docs/superpowers/specs/2026-05-08-syscall-shim-design.md §"Control socket
protocol" for the grammar.
"""
import socket
import threading


class SimControlError(Exception):
    pass


class SimControlClient:
    """Synchronous client. Single-threaded usage; instantiate in tests."""

    def __init__(self, socket_path: str, timeout: float = 5.0):
        self.socket_path = socket_path
        self.timeout = timeout
        self._lock = threading.Lock()
        self._sock = None

    def connect(self) -> None:
        if self._sock is not None:
            return
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self.socket_path)
        self._sock = s

    def close(self) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.close()

    def _send_recv(self, line: str) -> str:
        with self._lock:
            self.connect()
            self._sock.sendall((line + "\n").encode("ascii"))
            buf = b""
            while b"\n" not in buf:
                chunk = self._sock.recv(256)
                if not chunk:
                    raise SimControlError("control socket closed unexpectedly")
                buf += chunk
            reply = buf.split(b"\n", 1)[0].decode("ascii")
            if reply.startswith("error:"):
                raise SimControlError(reply)
            return reply

    def ping(self) -> None:
        r = self._send_recv("ping")
        if r != "ok":
            raise SimControlError(f"unexpected ping reply: {r}")

    def set_gpio_input(self, chip: int, line: int, value: int) -> None:
        r = self._send_recv(f"set_gpio_input chip={chip} line={line} value={value}")
        if r != "ok":
            raise SimControlError(f"unexpected reply: {r}")

    def set_adc(self, channel: int, value: int) -> None:
        r = self._send_recv(f"set_adc channel={channel} value={value}")
        if r != "ok":
            raise SimControlError(f"unexpected reply: {r}")

    def get_gpio_output(self, chip: int, line: int) -> int:
        r = self._send_recv(f"get_gpio_output chip={chip} line={line}")
        if not r.startswith("value="):
            raise SimControlError(f"unexpected reply: {r}")
        return int(r[len("value="):])

    def get_pwm(self, chip: int, pwm: int, file: str = "duty_cycle") -> int:
        r = self._send_recv(f"get_pwm chip={chip} pwm={pwm} file={file}")
        if not r.startswith("value="):
            raise SimControlError(f"unexpected reply: {r}")
        return int(r[len("value="):])

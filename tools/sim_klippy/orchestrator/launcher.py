"""Spawn the two Linux MACH_LINUX klipper.elf instances that back the
faithful sim. H7-flavored has KALICO_RUNTIME=y; F4-flavored doesn't.

Each instance opens a PTY at the supplied socket path. We wait for
both PTYs to exist before returning, so callers can immediately do
attach_serial against them."""
import dataclasses
import os
import signal
import subprocess
import time


@dataclasses.dataclass
class McuHandle:
    name: str
    process: subprocess.Popen
    socket_path: str
    log_path: str


@dataclasses.dataclass
class McuHandles:
    h7: McuHandle
    f4: McuHandle

    def shutdown(self) -> None:
        for h in (self.h7, self.f4):
            if h.process.poll() is None:
                h.process.send_signal(signal.SIGTERM)
        for h in (self.h7, self.f4):
            try:
                h.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                h.process.kill()
                try:
                    h.process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    pass
        for h in (self.h7, self.f4):
            try:
                os.unlink(h.socket_path)
            except FileNotFoundError:
                pass


def _spawn_one(elf: str, socket_path: str, log_path: str,
               name: str) -> McuHandle:
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    log_fd = open(log_path, "wb")
    proc = subprocess.Popen(
        [elf, "-I", socket_path],
        stdout=log_fd,
        stderr=subprocess.STDOUT,
    )
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        if os.path.exists(socket_path):
            return McuHandle(
                name=name, process=proc,
                socket_path=socket_path, log_path=log_path,
            )
        if proc.poll() is not None:
            log_fd.close()
            log_content = open(log_path).read()
            raise RuntimeError(
                f"{name}: klipper.elf exited early (rc={proc.returncode})\n"
                f"---log---\n{log_content}"
            )
        time.sleep(0.05)
    proc.kill()
    log_fd.close()
    raise RuntimeError(f"{name}: PTY {socket_path} did not appear in 5s")


def spawn_mcus(
    h7_elf: str = "out/klipper-h7-sim.elf",
    f4_elf: str = "out/klipper-f4-sim.elf",
    h7_socket: str = "/tmp/klipper_sim_h7",
    f4_socket: str = "/tmp/klipper_sim_f4",
    log_dir: str = "/tmp/klipper_sim_logs",
) -> McuHandles:
    os.makedirs(log_dir, exist_ok=True)
    h7 = _spawn_one(h7_elf, h7_socket,
                    os.path.join(log_dir, "h7.log"), "h7")
    f4 = _spawn_one(f4_elf, f4_socket,
                    os.path.join(log_dir, "f4.log"), "f4")
    return McuHandles(h7=h7, f4=f4)

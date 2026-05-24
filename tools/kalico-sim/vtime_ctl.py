"""Control the shared virtual-time clock used by libvtime.so."""
import ctypes
import mmap
import os
import struct

SHM_NAME = "/kalico_vtime"
SHM_SIZE = 32  # sizeof(struct vtime_shm): 4 x uint64/uint32 aligned

# Match struct vtime_shm layout: nanos(u64), num_sleepers(u32),
# num_participants(u32), initialized(u32), padding(u32)
_STRUCT_FMT = "<QIIII"


def _shm_path() -> str:
    return f"/dev/shm{SHM_NAME}"


def create(start_ns: int = 1_000_000_000) -> None:
    """Create and initialize the shared memory clock.

    start_ns defaults to 1 second to avoid zero-time edge cases in
    Klipper's timer initialization (timer.c sets start_sec = curtime.tv_sec + 1).
    """
    path = _shm_path()
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, SHM_SIZE)
    buf = mmap.mmap(fd, SHM_SIZE)
    # Write initial state: nanos=start_ns, sleepers=0, participants=0, init=1
    struct.pack_into(_STRUCT_FMT, buf, 0, start_ns, 0, 0, 1, 0)
    buf.flush()
    buf.close()
    os.close(fd)


def read_ns() -> int:
    """Read the current virtual time in nanoseconds."""
    path = _shm_path()
    fd = os.open(path, os.O_RDONLY)
    buf = mmap.mmap(fd, SHM_SIZE, access=mmap.ACCESS_READ)
    nanos, _, participants, _, _ = struct.unpack_from(_STRUCT_FMT, buf, 0)
    buf.close()
    os.close(fd)
    return nanos


def read_seconds() -> float:
    """Read the current virtual time in seconds."""
    return read_ns() / 1e9


def destroy() -> None:
    """Remove the shared memory segment."""
    path = _shm_path()
    try:
        os.unlink(path)
    except FileNotFoundError:
        pass


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} [create|read|destroy]")
        sys.exit(1)
    cmd = sys.argv[1]
    if cmd == "create":
        start = int(sys.argv[2]) if len(sys.argv) > 2 else 1_000_000_000
        create(start)
        print(f"Created vtime shm at {start} ns ({start/1e9:.3f} s)")
    elif cmd == "read":
        ns = read_ns()
        print(f"{ns} ns ({ns/1e9:.3f} s)")
    elif cmd == "destroy":
        destroy()
        print("Destroyed vtime shm")
    else:
        print(f"Unknown command: {cmd}")
        sys.exit(1)

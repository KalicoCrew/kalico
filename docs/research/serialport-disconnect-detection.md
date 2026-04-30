---
topic: serialport crate USB-CDC disconnect detection on Unix
created: 2026-04-30
last_updated: 2026-04-30
verified_claims:
  - 2026-04-30 VERIFIED — On Unix, USB-CDC unplug surfaces from `serialport::SerialPort::read()` as `Err` (typically `io::ErrorKind::BrokenPipe` via the POLLHUP path or `Other` via the EIO read path), not as `Ok(0)`. Designs that treat `Ok(0)` as the primary disconnect signal will miss the dominant case; the load-bearing path is the non-`TimedOut` `Err` branch.
sources:
  - https://www.kernel.org/doc/html/latest/driver-api/tty/tty_struct.html
  - https://linux-usb.vger.kernel.narkive.com/uJP6gJG3/usb-serial-crash-again-at-unplug
  - https://github.com/torvalds/linux/blob/master/drivers/usb/class/cdc-acm.c
  - https://github.com/serialport/node-serialport/issues/886
  - serialport crate v4.9.0 source (local registry)
  - nix crate v0.26.4 source (local registry)
  - klippy/chelper/serialqueue.c (Klipper reference behavior)
---

# `serialport` crate USB-CDC disconnect detection on Unix

## Summary

When a USB-CDC serial device is unplugged on Linux (and very likely macOS) while
the host is reading from it via the Rust `serialport` crate, the failure surfaces
as `Err`, not as `Ok(0)`. The kernel's `acm_disconnect` path calls
`tty_port_tty_vhangup`, which sets `TTY_IO_ERROR` on the tty struct; subsequent
`read()` syscalls return `-EIO`. Depending on whether the disconnect is observed
in serialport's poll wrapper or in the raw read syscall, the consumer sees
`io::ErrorKind::BrokenPipe` (POLLHUP path) or `io::ErrorKind::Other` (EIO read
path — note: the crate has no explicit `EIO` arm in `posix/error.rs`, so it
falls into `Unknown → Other`). It is **not** `NoDevice`/`NotFound` at read time;
`NoDevice` is reserved for open-time `EBUSY`.

A reactor-level disconnect rule should therefore be `non-TimedOut Err → Closed`,
**with explicit retry** on `Interrupted` (EINTR) and `WouldBlock` (EAGAIN) since
both are contractually retry-safe per `std::io::Read`. A long debounce on
`Ok(0)` is largely unnecessary for live USB-CDC; it only matters for PTY-based
test harnesses.

## Verified claim — 2026-04-30

**Original claim (Codex Finding 3):** `Ok(0)` is not the only or primary disconnect
signal in the `serialport` crate on Unix. The non-`TimedOut` error path is the
load-bearing path because USB-CDC removal commonly arrives as an error/hangup,
and `serialport`'s Unix `TTYPort::read` wraps `nix::unistd::read` errors. Treating
only `Ok(0)` specially with a 1-second debounce would be insufficient.

### Verification

**Klipper reference (`klippy/chelper/serialqueue.c:320-330`):** Klipper itself
treats any `read() <= 0` as immediate exit — no debounce. Distinguishes EOF (0)
from errno (<0) only for the log message; both terminate the reactor.

**`serialport` v4.9.0 read path (`src/posix/tty.rs:466-473`):**

```
fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
    if let Err(e) = super::poll::wait_read_fd(self.fd, self.timeout) {
        return Err(io::Error::from(Error::from(e)));
    }
    nix::unistd::read(self.fd, buf).map_err(|e| io::Error::from(Error::from(e)))
}
```

Two paths can surface the disconnect:

1. **Poll path (`src/posix/poll.rs:42-52`).** When `poll`/`ppoll` returns and
   `revents` contains `POLLHUP` or `POLLNVAL`, the wrapper returns
   `io::Error::new(io::ErrorKind::BrokenPipe, EPIPE.desc())`. (Subtlety: the
   "exact match" arm requires `revents == events`; if the kernel sets
   `POLLIN | POLLHUP` together — common during unplug — the match falls to
   the BrokenPipe arm.)

2. **Read path (`src/posix/error.rs:18-44`).** `nix::Error` → `serialport::Error`
   has explicit arms for `EBUSY → NoDevice`, `ETIMEDOUT → TimedOut`,
   `EINTR → Interrupted`, `EAGAIN → WouldBlock`, etc. There is **no arm for
   EIO**, so EIO falls to the `_ => K::Unknown` default. `K::Unknown` then maps
   to `io::ErrorKind::Other` in `From<Error> for io::Error` (`src/lib.rs:127-137`).
   `NoDevice → NotFound` only fires for `EBUSY`, which is an open-time exclusive-
   lock condition — not a runtime read error.

**Linux kernel ground truth (`drivers/usb/class/cdc-acm.c:1605-1634`):** On USB
disconnect, `acm_disconnect` sets `acm->disconnected = true` and calls
`tty_port_tty_vhangup`. Per kernel docs (`tty_struct.html`), `tty_vhangup` sets
the `TTY_IO_ERROR` flag, after which "all subsequent userspace read/write calls
on the tty fail, returning `-EIO`." So the dominant errno on Linux USB-CDC
unplug is `EIO`, surfacing through `serialport` as either `BrokenPipe` (caught
in poll) or `Other` (caught in read).

**`Ok(0)` is preserved by `nix::unistd::read` (`unistd.rs:1071-1077`):**
`Errno::result(res).map(|r| r as usize)` — only negative returns become `Err`.
Zero returns become `Ok(0)`. But the tty driver does not return 0 after
`tty_vhangup`; it returns `-EIO`. So `Ok(0)` is essentially a phantom path on
live USB-CDC tty devices. (It's still observable on PTY-based test fixtures
where the master/slave relationship can deliver true EOF.)

**macOS:** Indirect evidence (`node-serialport` issue #886 documents ENXIO at
open time after unplug-replug; Apple Forums discussions describe `read()`
returning errors not 0). I did not trace the IOKit/`IOSerialBSDClient` source
directly. macOS verification status: presumed-similar but unconfirmed; flagged
under unchecked assumptions.

### Adversarial findings

Attacks attempted, with outcomes:

- **Could `Ok(0)` actually be the dominant signal?** No. `acm_disconnect` →
  `tty_vhangup` → `TTY_IO_ERROR` → `read()` returns `-EIO`. No `Ok(0)` path.
- **Could the kernel deliver `POLLIN` (no `POLLHUP`) and then `read() == 0`?**
  No. `tty_vhangup` is synchronous in `acm_disconnect`; the flag is set before
  any subsequent reader runs. Reads return `-EIO` even with buffered data.
- **Is "non-`TimedOut` Err → Closed" too aggressive?** Yes — partially.
  `EINTR` maps to `io::ErrorKind::Interrupted`, which `std::io::Read`
  contractually requires consumers to retry. The `serialport` crate's
  `read()` does not loop on EINTR (only `flush` does). Closing on
  `Interrupted` is incorrect; the design must explicitly retry.
  `WouldBlock` (EAGAIN) is similarly retry-safe but rare given the
  poll-then-read sequence.
- **Is `NoDevice`/`NotFound` actually observed at read time?** No. Despite
  `ErrorKind::NoDevice` doc-comment (`lib.rs:60-70`) mentioning disconnect,
  the only Unix code path emitting `NoDevice` is the `EBUSY → NoDevice` arm
  for open-time exclusive locking. At read time, unplug surfaces as
  `BrokenPipe` or `Other`. This refines but does not break Codex's main
  thrust.
- **Is the 1-second debounce justified?** Not for live USB-CDC. Real
  disconnect arrives as `Err` within milliseconds of the unplug event. A
  1-second `Ok(0)` debounce only matters in test harnesses using PTYs.
  Could be shortened to ~100ms or removed.

### Reactor recommendations (Step 7-C-io)

The disconnect rule should be:

```text
Closed if:
  Err(BrokenPipe | ConnectionAborted | ConnectionReset
      | NotConnected | UnexpectedEof | NotFound
      | Other matching EIO description)

Retry if:
  Err(Interrupted | WouldBlock | TimedOut)

Closed-after-debounce if:
  Ok(0) sustained for N ms  (N ≪ 1000; only matters for PTY tests)
```

The whitelist-retry / blacklist-close split should be inverted from "any
non-TimedOut Err is Closed" to: explicitly retry the small known-retry-safe
set, treat everything else as Closed. This keeps the safety property Codex
identified (no silent miss) while not over-closing on transient EINTR.

### Sources

- Linux kernel `tty_struct.html` — TTY_IO_ERROR semantics
  https://www.kernel.org/doc/html/latest/driver-api/tty/tty_struct.html
  retrieved 2026-04-30
- `drivers/usb/class/cdc-acm.c` — `acm_disconnect → tty_port_tty_vhangup`
  https://github.com/torvalds/linux/blob/master/drivers/usb/class/cdc-acm.c
  retrieved 2026-04-30 (line 1605–1634 in the master tip)
- linux-usb mailing list — usb_serial unplug semantics
  https://linux-usb.vger.kernel.narkive.com/uJP6gJG3/usb-serial-crash-again-at-unplug
  retrieved 2026-04-30
- node-serialport issue #886 — ENXIO on macOS USB-CDC unplug
  https://github.com/serialport/node-serialport/issues/886
  retrieved 2026-04-30
- `serialport` crate v4.9.0 — `src/posix/{tty,poll,error}.rs`, `src/lib.rs`
  (local registry; equivalent at https://github.com/serialport/serialport-rs)
- `nix` crate v0.26.4 — `src/unistd.rs:1071`
- `klippy/chelper/serialqueue.c:320-330` — Klipper reference exits on
  `ret <= 0` with no debounce

### Caveats / unchecked assumptions

- macOS path is verified by analogy to Linux + indirect issue reports, not by
  direct IOKit source trace. Step 7-D bring-up should empirically confirm the
  observed `io::ErrorKind` on the actual Mac dev environment.
- Empirical observation on the BTT Octopus Pro (H723) over USB-CDC has not
  been performed; this is a Step 7-D bring-up task.
- Future versions of `nix` may auto-retry EINTR in `poll`. The analysis here
  assumes nix 0.26 behavior (no auto-retry).
- The `serialport` `posix/error.rs` lacks an `EIO → BrokenPipe` (or similar)
  explicit arm, so EIO surfaces as `Other`. If the upstream crate adds such an
  arm in a future release, the reactor's mapping table needs updating.
- Windows behavior is out of scope (the design and Codex finding are Unix-only).

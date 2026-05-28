"""Behavioral emulator for the TMC5160 stepper driver chip.

Models register state, side-effects on read/write of specific registers,
and StallGuard load injection so tests can drive the DIAG-trigger path.
Does NOT model coil-current dynamics, microstep tables, or COOLSTEP
beyond what static register reads expose.

PyTrinamic note: the library names the register GLOBAL_SCALER (with
underscore), not GLOBALSCALER. The local constant here is named
GLOBALSCALER for readability, mapping to address 0x0B.
"""

from __future__ import annotations

from typing import Callable, Optional

# Register addresses — TMC5160 datasheet §5
GCONF = 0x00
GSTAT = 0x01
IFCNT = 0x02
SLAVECONF = 0x03
IOIN = 0x04
GLOBALSCALER = 0x0B
IHOLD_IRUN = 0x10
TPOWERDOWN = 0x11
TSTEP = 0x12
TPWMTHRS = 0x13
TCOOLTHRS = 0x14
THIGH = 0x15
CHOPCONF = 0x6C
COOLCONF = 0x6D
DCCTRL = 0x6E
DRV_STATUS = 0x6F
PWMCONF = 0x70

# Power-on-reset defaults (datasheet Table 6)
POR_DEFAULTS = {
    GCONF: 0x00000000,
    GSTAT: 0x00000007,  # reset bit set on POR
    IFCNT: 0x00000000,
    SLAVECONF: 0x00000000,
    IOIN: 0x30000000,
    GLOBALSCALER: 256,  # datasheet POR = 0 (full-scale); treated as 256 internally
    IHOLD_IRUN: 0x00000000,
    TPOWERDOWN: 0x0000000A,
    TSTEP: 0x000FFFFF,
    CHOPCONF: 0x00410150,
    DRV_STATUS: 0x00000000,
}

# Registers that are cleared to zero after a read
CLEAR_ON_READ = {GSTAT}


class TMC5160Emulator:
    """Behavioral emulator for a single TMC5160 stepper driver.

    Usage::

        chip = TMC5160Emulator()
        # 5-byte write: byte0 = 0x80 | addr, bytes 1-4 = big-endian data
        chip.transfer(bytes([0x80 | 0x0B, 0, 0, 0, 128]))
        # Latched read: two transfers to get current value
        chip.transfer(bytes([0x0B, 0, 0, 0, 0]))  # 1st: latches value
        reply = chip.transfer(bytes([0x0B, 0, 0, 0, 0]))  # 2nd: returns it
    """

    def __init__(self) -> None:
        self._registers: dict[int, int] = dict(POR_DEFAULTS)
        # Latched-read state: stores the value captured during the last read
        # transfer; subsequent read returns *this* value, not the current one.
        self._last_read_data: int = 0
        # StallGuard result injected via set_load()
        self._sg_result: int = 0
        self._diag_callback: Optional[Callable[[bool], None]] = None
        self._diag_high: bool = False

    # ------------------------------------------------------------------
    # Public test-hook interface
    # ------------------------------------------------------------------

    def set_load(self, sg_result: int) -> None:
        """Inject a synthetic StallGuard result (0–1023) for the next DRV_STATUS read."""
        self._sg_result = sg_result & 0x03FF

    def set_diag_callback(self, cb: Callable[[bool], None]) -> None:
        """Register a callback that fires when the DIAG output changes state.

        The callback receives True when DIAG is asserted (SG_RESULT < threshold),
        and False when DIAG is de-asserted (SG_RESULT >= threshold).
        """
        self._diag_callback = cb

    def maybe_trigger_diag(self, sg_threshold: int) -> None:
        """Evaluate the current SG_RESULT against sg_threshold and fire the
        DIAG callback on any edge (assert or de-assert)."""
        should_be_high = self._sg_result < sg_threshold
        if should_be_high != self._diag_high:
            self._diag_high = should_be_high
            if self._diag_callback is not None:
                self._diag_callback(should_be_high)

    # ------------------------------------------------------------------
    # SPI datagram interface
    # ------------------------------------------------------------------

    def transfer(self, req: bytes) -> bytes:
        """Process a 5-byte SPI datagram and return the 5-byte reply.

        Datasheet §5.1 framing:
          byte 0  = R/W (bit 7) | register address (bits 6-0)
          bytes 1-4 = data, MSB first (big-endian)

        Reads use latched-read semantics: the reply carries the value
        captured during the *previous* read transfer, not the current one.
        Writes return five zero bytes (status byte is 0x00 for simplicity).
        """
        if len(req) != 5:
            raise ValueError(f"TMC5160 expects 5-byte datagram, got {len(req)}")

        is_write = bool(req[0] & 0x80)
        addr = req[0] & 0x7F
        data = (req[1] << 24) | (req[2] << 16) | (req[3] << 8) | req[4]

        if is_write:
            self._do_write(addr, data)
            # Real TMC5160: every reply contains the data field of the
            # PRIOR datagram. After a write, the next transfer's reply
            # echoes the written value — klippy's tmc2130.set_register
            # relies on this for write verification (write_cmd then a
            # dummy_read; the dummy_read's reply must carry the written
            # value). Mirror that here by latching the post-clamp stored
            # value into _last_read_data.
            self._last_read_data = self._registers.get(addr, 0)
            return bytes(5)

        # Read path: return previously latched data, then capture current value
        latched = self._last_read_data
        self._last_read_data = self._do_read(addr)
        return bytes(
            [
                0x00,  # status byte (simplified: always 0)
                (latched >> 24) & 0xFF,
                (latched >> 16) & 0xFF,
                (latched >> 8) & 0xFF,
                latched & 0xFF,
            ]
        )

    # ------------------------------------------------------------------
    # Internal register access with side-effects
    # ------------------------------------------------------------------

    def _do_write(self, addr: int, value: int) -> None:
        """Apply write with any address-specific clamping or masking."""
        if addr == GLOBALSCALER:
            # Datasheet: 0 = full-scale (256), valid range 32-255 for all other
            # values. Clamp to [32, 255] for any explicit non-zero write.
            value = max(32, min(255, value))
        elif addr == IHOLD_IRUN:
            # Extract full field values (no pre-mask) then saturate-clamp to
            # the datasheet maximum of 31 for both ihold (bits 4-0 field,
            # 5 bits) and irun (bits 12-8 field, 5 bits).  Using a wider mask
            # before the clamp ensures that e.g. 0x40 (= 64) clamps to 31
            # rather than being silently zeroed by a 5-bit mask first.
            ihold = min(31, value & 0xFF)  # bits 7-0 raw, clamp
            irun = min(31, (value >> 8) & 0xFF)  # bits 15-8 raw, clamp
            iholddelay = (value >> 16) & 0x0F  # bits 19-16
            value = ihold | (irun << 8) | (iholddelay << 16)
        elif addr == CHOPCONF:
            value &= ~(0x7 << 17)  # bits 17-19 are reserved; force to 0
        self._registers[addr] = value

    def _do_read(self, addr: int) -> int:
        """Return register value, applying any read-time side-effects."""
        if addr == DRV_STATUS:
            # Inject current SG_RESULT into low 10 bits; preserve upper bits
            base = self._registers.get(addr, 0) & ~0x03FF
            return base | self._sg_result

        value = self._registers.get(addr, 0)
        if addr in CLEAR_ON_READ:
            self._registers[addr] = 0
        return value

"""Behavioral emulator for the TMC2209 stepper driver chip.

Single-wire UART, CRC8 polynomial 0x07. Smaller register surface than
the 5160. Same per-chip register dict + clear-on-read for GSTAT."""

CRC8_POLY = 0x07


def crc8(data: bytes) -> int:
    """TMC datasheet CRC8: x^8 + x^2 + x + 1 (polynomial 0x07), init 0,
    process LSB first within each byte."""
    crc = 0
    for byte in data:
        v = byte
        for _ in range(8):
            if (crc >> 7) ^ (v & 1):
                crc = ((crc << 1) ^ CRC8_POLY) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            v >>= 1
    return crc


# Register addresses
GCONF       = 0x00
GSTAT       = 0x01
IFCNT       = 0x02
IHOLD_IRUN  = 0x10
CHOPCONF    = 0x6C
DRV_STATUS  = 0x6F
PWMCONF     = 0x70

POR_DEFAULTS = {
    GCONF:       0x00000000,
    GSTAT:       0x00000005,
    IFCNT:       0x00000000,
    IHOLD_IRUN:  0x00000000,
    CHOPCONF:    0x10000053,
    DRV_STATUS:  0x00000000,
    PWMCONF:     0xC10D0024,
}

CLEAR_ON_READ = {GSTAT}


class TMC2209Emulator:
    def __init__(self, slave_addr: int):
        self._slave = slave_addr
        self._registers = dict(POR_DEFAULTS)

    def handle(self, msg: bytes) -> bytes:
        """Process one inbound UART datagram. Returns reply bytes (8
        for reads, empty for writes)."""
        if len(msg) == 4:
            # read request
            if msg[0] != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:3]) != msg[3]:
                raise ValueError("TMC2209 read CRC mismatch")
            reg = msg[2] & 0x7F
            value = self._registers.get(reg, 0)
            if reg in CLEAR_ON_READ:
                self._registers[reg] = 0
            reply_body = bytes([
                0x05, 0xFF, reg,
                (value >> 24) & 0xFF,
                (value >> 16) & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            ])
            return reply_body + bytes([crc8(reply_body)])

        if len(msg) == 8:
            # write request
            if msg[0] != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:7]) != msg[7]:
                raise ValueError("TMC2209 write CRC mismatch")
            reg = msg[2] & 0x7F
            value = (msg[3] << 24) | (msg[4] << 16) | (msg[5] << 8) | msg[6]
            self._registers[reg] = value
            return b""

        raise ValueError(f"TMC2209 frame must be 4 or 8 bytes, got {len(msg)}")

CRC8_POLY = 0x07


def _decode_uart_bits(data: bytes) -> bytes:
    """Strip UART start/stop framing bits from a klippy-tmcuart wire
    payload. klippy's tmc_uart._add_serial_bits encodes each logical
    byte as 10 wire bits: start (0), 8 data bits LSB-first, stop (1).
    For an N-byte logical frame the wire form is ceil(N*10/8) bytes;
    so 4 → 5 bytes and 8 → 10 bytes. We invert that to get back the
    original bytes."""
    bitcount = len(data) * 8
    val = 0
    for i, b in enumerate(data):
        val |= b << (i * 8)
    out = bytearray()
    pos = 0
    while pos + 10 <= bitcount:
        slot = (val >> pos) & 0x3FF
        # start bit must be 0, stop bit must be 1
        if (slot & 1) != 0 or (slot & 0x200) == 0:
            # framing error — return raw, let caller decide
            return bytes(data)
        out.append((slot >> 1) & 0xFF)
        pos += 10
    return bytes(out)


def _encode_uart_bits(data: bytes) -> bytes:
    bitcount = len(data) * 10
    val = 0
    for i, b in enumerate(data):
        slot = (b << 1) | 0x200
        val |= slot << (i * 10)
    out = bytearray()
    for i in range((bitcount + 7) // 8):
        out.append((val >> (i * 8)) & 0xFF)
    return bytes(out)


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


GCONF = 0x00
GSTAT = 0x01
IFCNT = 0x02
IHOLD_IRUN = 0x10
CHOPCONF = 0x6C
DRV_STATUS = 0x6F
PWMCONF = 0x70

POR_DEFAULTS = {
    GCONF: 0x00000000,
    GSTAT: 0x00000005,
    IFCNT: 0x00000000,
    IHOLD_IRUN: 0x00000000,
    CHOPCONF: 0x10000053,
    DRV_STATUS: 0x00000000,
    PWMCONF: 0xC10D0024,
}

CLEAR_ON_READ = {GSTAT}


class TMC2209Emulator:
    def __init__(self, slave_addr: int):
        self._slave = slave_addr
        self._registers = dict(POR_DEFAULTS)

    def handle(self, msg: bytes) -> bytes:
        if len(msg) == 5:
            decoded = _decode_uart_bits(msg)
            if len(decoded) != 4:
                raise ValueError(
                    f"TMC2209 5-byte frame failed UART decode (got "
                    f"{len(decoded)}-byte logical)"
                )
            msg = decoded
        elif len(msg) == 10:
            decoded = _decode_uart_bits(msg)
            if len(decoded) != 8:
                raise ValueError(
                    f"TMC2209 10-byte frame failed UART decode (got "
                    f"{len(decoded)}-byte logical)"
                )
            msg = decoded

        if len(msg) == 4:
            # TMC2209 datasheet sync byte is 0x05; klippy actually sends
            # 0xF5 (upper nibble reserved). Accept both — only the low
            # nibble identifies the protocol.
            if (msg[0] & 0x0F) != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:3]) != msg[3]:
                raise ValueError("TMC2209 read CRC mismatch")
            reg = msg[2] & 0x7F
            value = self._registers.get(reg, 0)
            if reg in CLEAR_ON_READ:
                self._registers[reg] = 0
            reply_body = bytes(
                [
                    0x05,
                    0xFF,
                    reg,
                    (value >> 24) & 0xFF,
                    (value >> 16) & 0xFF,
                    (value >> 8) & 0xFF,
                    value & 0xFF,
                ]
            )
            reply = reply_body + bytes([crc8(reply_body)])
            return _encode_uart_bits(reply)

        if len(msg) == 8:
            if (msg[0] & 0x0F) != 0x05 or msg[1] != self._slave:
                return b""
            if crc8(msg[:7]) != msg[7]:
                raise ValueError("TMC2209 write CRC mismatch")
            reg = msg[2] & 0x7F
            value = (msg[3] << 24) | (msg[4] << 16) | (msg[5] << 8) | msg[6]
            self._registers[reg] = value
            # Real TMC2209: IFCNT increments by 1 on every successful
            # register write. klippy's tmc_uart.set_register verifies
            # writes by reading IFCNT before and after, expecting +1.
            self._registers[IFCNT] = (self._registers.get(IFCNT, 0) + 1) & 0xFF
            return b""

        raise ValueError(f"TMC2209 frame must be 4 or 8 bytes, got {len(msg)}")

"""Behavioral emulator for the MAX31865 RTD-to-digital converter.

The faithful sim wires this in alongside the TMC5160 emulators on the H7
SPI1 bus so the extruder thermistor's periodic SPI reads dispatch
cleanly instead of crashing the TMC handler with a length mismatch.

Wire protocol (datasheet §"Serial Interface"):
- byte 0: address. bit 7 set ⇒ write, clear ⇒ read; bits 6-0 select the
  register. Address auto-increments across the same transfer.
- bytes 1..N: payload. SPI is symmetric — for reads the controller
  shifts in dummy bytes and reads the chip's reply on MISO; for writes
  the bytes overwrite the addressed register and onward.

Registers modeled (datasheet Table 1):
  0x00  CONFIG
  0x01  RTD MSB    (bits 15-8 of the 15-bit ADC + fault bit in LSB.0)
  0x02  RTD LSB
  0x03  HFAULT MSB / 0x04 HFAULT LSB
  0x05  LFAULT MSB / 0x06 LFAULT LSB
  0x07  FAULT STATUS

The RTD value reads back a constant corresponding to ~25 °C with
``rtd_nominal_r=1000`` / ``rtd_reference_r=4300`` (the user's printer
config). Klippy's ``calc_temp`` will turn this into a believable ambient
reading; we're not modeling thermal dynamics here, just keeping the
heater_check happy enough for boot.
"""

from typing import Dict

CONFIG_REG = 0x00
RTD_MSB_REG = 0x01
RTD_LSB_REG = 0x02
HFAULT_MSB_REG = 0x03
HFAULT_LSB_REG = 0x04
LFAULT_MSB_REG = 0x05
LFAULT_LSB_REG = 0x06
FAULT_STATUS_REG = 0x07

# Default ADC reading: encodes ~25 °C for rtd_nominal_r=1000,
# rtd_reference_r=4300. Computation:
#   R(25C) = 1000 * (1 + 3.9083e-3*25 + (-5.775e-7)*625) ≈ 1097.35 Ω
#   adc = R / (rtd_reference_r / 32768) ≈ 1097.35 * 32768 / 4300 ≈ 8362
#   wire form is adc << 1 (low bit reserved for "fault present" flag).
DEFAULT_ADC = 8362
DEFAULT_RTD_REGISTER = (DEFAULT_ADC << 1) & 0xFFFF


class MAX31865Emulator:
    def __init__(self) -> None:
        self._regs: Dict[int, int] = {
            CONFIG_REG: 0x00,
            RTD_MSB_REG: (DEFAULT_RTD_REGISTER >> 8) & 0xFF,
            RTD_LSB_REG: DEFAULT_RTD_REGISTER & 0xFF,
            HFAULT_MSB_REG: 0xFF,
            HFAULT_LSB_REG: 0xFF,
            LFAULT_MSB_REG: 0x00,
            LFAULT_LSB_REG: 0x00,
            FAULT_STATUS_REG: 0x00,
        }

    def transfer(self, req: bytes) -> bytes:
        if len(req) < 1:
            raise ValueError("MAX31865 transfer must be >= 1 byte")
        addr_byte = req[0]
        is_write = bool(addr_byte & 0x80)
        addr = addr_byte & 0x7F
        # Reply: status byte mirrors the address byte echo (datasheet
        # leaves the address-byte slot undefined on MISO; use 0x00).
        reply = bytearray(len(req))
        reply[0] = 0x00
        for i in range(1, len(req)):
            cur = (addr + i - 1) & 0x7F
            if is_write:
                self._regs[cur] = req[i]
                reply[i] = 0x00
            else:
                reply[i] = self._regs.get(cur, 0x00)
        return bytes(reply)

    def set_rtd_register(self, raw_15bit_with_fault_bit: int) -> None:
        v = raw_15bit_with_fault_bit & 0xFFFF
        self._regs[RTD_MSB_REG] = (v >> 8) & 0xFF
        self._regs[RTD_LSB_REG] = v & 0xFF

    def get_config(self) -> int:
        return self._regs[CONFIG_REG]

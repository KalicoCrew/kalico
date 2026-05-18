//
// TMC5160 SPI-slave stub for Renode (kalico phase-stepping sim).
//
// Decodes Trinamic 40-bit (5-byte) datagrams, tracks GCONF + XDIRECT +
// IHOLD_IRUN register state, records an XDIRECT write history with
// virtual-time timestamps for host-side verification, and silently rejects
// XDIRECT writes when GCONF.direct_mode (bit 16) is clear — matching real
// silicon behaviour.
//
// Frame boundaries
// ----------------
// Renode's `SPIMultiplexer` is the canonical way to share an SPI bus among
// multiple slaves driven by distinct CS lines. The multiplexer receives CS
// transitions via `OnGPIO(line, value)`, routes `Transmit` bytes to the
// selected child, and forwards `FinishTransmission()` to that child on
// CS-deassert. We rely on that forwarded call as the authoritative frame
// boundary — this catches every real bus transaction whether driven by the
// CS GPIO or by the SPI controller's hardware NSS line, without us having
// to duplicate edge-detection logic here.
//
// We also implement `IGPIOReceiver` so the same peripheral can be wired
// directly to a CS pin (`gpioPortA <n> -> tmc_x@0`) for unit-style tests
// that skip the multiplexer. Both paths converge on the same `FinishFrame`
// routine.
//
// Datagram layout (firmware side: src/stm32/phase_stepping_spi.c)
// --------------------------------------------------------------
//   byte 0 : (write_bit << 7) | reg_addr     // 0xAD = write | XDIRECT(0x2D)
//   byte 1 : (value >> 24) & 0xFF            // coil_B sign bit (bit 24)
//   byte 2 : (value >> 16) & 0xFF            // coil_B low 8 bits  (bits 23:16)
//   byte 3 : (value >>  8) & 0xFF            // coil_A sign bit (bit 8)
//   byte 4 : (value      ) & 0xFF            // coil_A low 8 bits  (bits 7:0)
//
// After reconstructing `value` as a big-endian uint32:
//   coil_A = SignExtend9(value & 0x1FF)
//   coil_B = SignExtend9((value >> 16) & 0x1FF)
//

using System;
using System.Collections.Generic;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class TMC5160 : ISPIPeripheral, IGPIOReceiver
    {
        public TMC5160(IMachine machine)
        {
            this.machine = machine;
            this.registers = new Dictionary<byte, uint>();
            this.frameBuffer = new List<byte>();
            this.xdirectHistory = new List<XDirectRecord>();
            Reset();
        }

        public void Reset()
        {
            frameBuffer.Clear();
            registers.Clear();
            xdirectHistory.Clear();
            csAsserted = false;
            xdirectWriteCount = 0;
            xdirectRejectedCount = 0;
            frameErrorCount = 0;
        }

        // --- IGPIOReceiver (optional direct CS wiring) -----------------------
        // Renode convention: GPIO carries the *electrical* level. The firmware
        // drives CS low to assert; we therefore treat `value == false` as
        // "CS asserted". Multiple receiver pins are tolerated (any falling
        // edge starts a frame, any rising edge finalizes), but in practice
        // platforms wire a single CS line here.
        public void OnGPIO(int number, bool value)
        {
            bool newAsserted = !value;
            if (newAsserted && !csAsserted)
            {
                // Falling edge: start of frame.
                frameBuffer.Clear();
            }
            else if (!newAsserted && csAsserted)
            {
                // Rising edge: finalize frame.
                FinishFrame();
            }
            csAsserted = newAsserted;
        }

        // --- ISPIPeripheral --------------------------------------------------
        public byte Transmit(byte data)
        {
            // The SPIMultiplexer only forwards Transmit() when a single child
            // is selected via its CS lines, so reaching this code path means
            // the controller considers CS asserted regardless of whether
            // OnGPIO has been invoked on us. Accept the byte.
            frameBuffer.Add(data);
            // TMC5160's real status byte (spi_status) is ignored by this stub;
            // returning 0 matches the canonical no-fault response.
            return 0;
        }

        public void FinishTransmission()
        {
            // SPIMultiplexer forwards this on CS-deassert. If we got here via
            // direct OnGPIO wiring the rising edge has already called
            // FinishFrame() and frameBuffer is empty — the guard makes this
            // a no-op in that case.
            if (frameBuffer.Count == 0)
            {
                return;
            }
            FinishFrame();
        }

        // --- Frame decode ----------------------------------------------------
        private void FinishFrame()
        {
            if (frameBuffer.Count != FrameBytes)
            {
                this.Log(LogLevel.Warning,
                    "TMC5160 frame error: expected {0} bytes, got {1}",
                    FrameBytes, frameBuffer.Count);
                frameErrorCount++;
                frameBuffer.Clear();
                return;
            }

            byte addrByte = frameBuffer[0];
            bool isWrite = (addrByte & 0x80) != 0;
            byte regAddr = (byte)(addrByte & 0x7F);
            uint value =
                ((uint)frameBuffer[1] << 24) |
                ((uint)frameBuffer[2] << 16) |
                ((uint)frameBuffer[3] << 8) |
                ((uint)frameBuffer[4]);

            if (isWrite)
            {
                switch (regAddr)
                {
                    case RegGconf:
                        registers[RegGconf] = value;
                        break;
                    case RegIholdIrun:
                        registers[RegIholdIrun] = value;
                        break;
                    case RegXdirect:
                        HandleXDirect(value);
                        break;
                    default:
                        // Stub keeps a generic register shadow so unsupported
                        // writes are still observable via the monitor.
                        registers[regAddr] = value;
                        break;
                }
            }
            // Read-side response is not modelled (we always return 0 in
            // Transmit). Real firmware does not currently issue reads.

            frameBuffer.Clear();
        }

        private void HandleXDirect(uint value)
        {
            uint gconf = registers.TryGetValue(RegGconf, out var g) ? g : 0u;
            if ((gconf & GconfDirectModeMask) == 0)
            {
                // direct_mode bit is clear: silently reject. Real silicon
                // discards the write. The host test detects missing GCONF
                // setup by observing XDirectRejectedCount > 0.
                xdirectRejectedCount++;
                return;
            }
            registers[RegXdirect] = value;
            int coilA = SignExtend9(value & 0x1FFu);
            int coilB = SignExtend9((value >> 16) & 0x1FFu);
            ulong tUs = (ulong)machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;
            // Cap history to avoid unbounded growth during long runs; the
            // monitor command surfaces the most-recent N writes regardless.
            if (xdirectHistory.Count >= HistoryCapacity)
            {
                xdirectHistory.RemoveAt(0);
            }
            xdirectHistory.Add(new XDirectRecord
            {
                TimeUs = tUs,
                Raw = value,
                CoilA = coilA,
                CoilB = coilB,
            });
            xdirectWriteCount++;
        }

        private static int SignExtend9(uint v)
        {
            v &= 0x1FFu;
            return ((v & 0x100u) != 0) ? (int)(v | 0xFFFFFE00u) : (int)v;
        }

        // --- Monitor commands (auto-discovered by reflection) ----------------
        // Renode exposes any public method/property as a monitor command. We
        // keep these as zero-arg methods so callers spell them without parens.
        public uint ReadGconf()
        {
            return registers.TryGetValue(RegGconf, out var v) ? v : 0u;
        }

        public uint ReadXDirect()
        {
            return registers.TryGetValue(RegXdirect, out var v) ? v : 0u;
        }

        public uint ReadIholdIrun()
        {
            return registers.TryGetValue(RegIholdIrun, out var v) ? v : 0u;
        }

        public uint WriteCountXDirect()
        {
            return xdirectWriteCount;
        }

        public uint XDirectRejectedCount()
        {
            return xdirectRejectedCount;
        }

        public uint FrameErrorCount()
        {
            return frameErrorCount;
        }

        public int LastCoilA()
        {
            return xdirectHistory.Count == 0
                ? 0
                : xdirectHistory[xdirectHistory.Count - 1].CoilA;
        }

        public int LastCoilB()
        {
            return xdirectHistory.Count == 0
                ? 0
                : xdirectHistory[xdirectHistory.Count - 1].CoilB;
        }

        // Returns up to `max` most-recent XDIRECT writes as a multi-line
        // string: "<time_us>,<coil_a>,<coil_b>,<raw_hex>\n". One row per write.
        public string XDirectHistory(int max)
        {
            if (max <= 0)
            {
                return string.Empty;
            }
            int n = Math.Min(max, xdirectHistory.Count);
            int start = xdirectHistory.Count - n;
            var sb = new StringBuilder();
            for (int i = start; i < xdirectHistory.Count; i++)
            {
                var r = xdirectHistory[i];
                sb.AppendFormat("{0},{1},{2},0x{3:X8}\n",
                    r.TimeUs, r.CoilA, r.CoilB, r.Raw);
            }
            return sb.ToString();
        }

        // --- internals ------------------------------------------------------
        private struct XDirectRecord
        {
            public ulong TimeUs;
            public uint Raw;
            public int CoilA;
            public int CoilB;
        }

        private const int FrameBytes = 5;
        private const int HistoryCapacity = 1024;
        private const byte RegGconf = 0x00;
        private const byte RegIholdIrun = 0x10;
        private const byte RegXdirect = 0x2D;
        // GCONF bit 16 = direct_mode. (TMC5160 datasheet rev 1.18, table 5.1.)
        private const uint GconfDirectModeMask = 1u << 16;

        private readonly IMachine machine;
        private readonly Dictionary<byte, uint> registers;
        private readonly List<byte> frameBuffer;
        private readonly List<XDirectRecord> xdirectHistory;
        private bool csAsserted;
        private uint xdirectWriteCount;
        private uint xdirectRejectedCount;
        private uint frameErrorCount;
    }
}

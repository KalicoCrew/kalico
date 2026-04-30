---
topic: "STM32H723 double-precision FPU capability and f64 cycle costs"
created: 2026-04-30
last_updated: 2026-04-30
verified_claims:
  - 2026-04-30 INCORRECT — "Cortex-M7 has single-precision hardware FP only; software f64 will blow the ~200 cycle step-generation budget on STM32H723"
sources:
  - https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32h7-series/stm32h723-733/stm32h723zg.html
  - https://en.wikipedia.org/wiki/ARM_Cortex-M
  - https://reviews.llvm.org/D91355
---

# STM32H723 Double-Precision FPU Capability and f64 Cycle Costs

## Summary

The STM32H723 Cortex-M7 includes a hardware double-precision FPU (DP-FPU, VFPv5). The claim that it has "single-precision hardware FP only" is factually incorrect. Hardware f64 arithmetic on the M7 costs 4-7 cycles per operation (not 50-70 cycles for software emulation), making the proposed f64 step accumulator well within the ~200 cycle per-motor budget at ~23 cycles per motor per tick.

## Verified claim — 2026-04-30

"Cortex-M7 has single-precision hardware FP only. Software f64 operations per motor at 40 kHz will blow the ~200 cycle step-generation budget."

### Verification

**Premise check: Does the STM32H723 lack hardware f64?**

No. The STM32H723/733 product pages on st.com explicitly state: "32-bit Arm Cortex-M7 CPU with DP-FPU" (double-precision floating-point unit). The Cortex-M7 architecture offers the FPU in three configurations: no FPU, SP-only FPU, or SP+DP FPU. The STM32H7 series (including H723, H743, H745, H747, H750, H753, H755, H757) all implement the DP variant. Some other M7 implementations (e.g., certain NXP i.MX RT parts, Microchip SAM E70) also include DP; it is not universal across all M7 silicon, but the H723 specifically has it.

**Hardware f64 instruction costs (from LLVM Cortex-M7 scheduling model, D91355):**

| Instruction | Latency (cycles) |
|---|---|
| VADD.F64 | 4 |
| VSUB.F64 | 4 |
| VMUL.F64 | 7 |
| VCVT.F32.F64 / VCVT.F64.F32 | 4 |
| VCVT (float-to-int truncation) | ~4 |

Note: f64 operations cannot dual-issue (they occupy both VFP ports), so these latencies are also the throughput bottleneck. f32 operations (1-3 cycles) can dual-issue.

**Cost analysis for the proposed step accumulator (per motor per tick):**

| Operation | Instruction | Cycles |
|---|---|---|
| f32 to f64 promotion | VCVT.F64.F32 | 4 |
| f64 multiply (position_mm * steps_per_mm) | VMUL.F64 | 7 |
| f64 subtract (accumulated - old) | VSUB.F64 | 4 |
| f64 truncate to integer | VCVT (to int) | ~4 |
| f64 add (update accumulator) | VADD.F64 | 4 |
| **Total per motor** | | **~23** |

For 3 motors: ~69 cycles. This is well within the 200-cycle step-generation budget and trivially within the 13,000-cycle tick budget (520 MHz / 40 kHz).

**Comparison with i64 fixed-point:**

UMULL (32x32->64 unsigned multiply) is 1 cycle on M7. A fixed-point accumulator doing equivalent work would cost ~5-8 cycles per motor (~15-24 for 3 motors). Cheaper by ~3x, but hardware f64 is already so cheap that the savings (~45 cycles out of 13,000) are negligible. The i64 approach would add code complexity (scaling, overflow management) for minimal gain.

### Sources

- ST STM32H723ZG product page — confirms DP-FPU, retrieved 2026-04-30
- Wikipedia ARM Cortex-M — confirms optional SP/DP FPU variants, retrieved 2026-04-30
- LLVM D91355 — Cortex-M7 scheduling model with f64 instruction latencies, retrieved 2026-04-30
- SEGGER floating-point benchmark blog — software f64 costs on M4 (54-71 cycles per op) for comparison, retrieved 2026-04-30

### Caveats / unchecked assumptions

- Cycle counts from the LLVM scheduling model are design-intent values from ARM's published timing; actual silicon may vary by +/-1 cycle due to pipeline effects, memory stalls, or ISR preemption. The step5-h723-cycle-budget.md framework exists to measure actuals on hardware.
- The 520 MHz clock frequency is assumed. Note: step5-h723-cycle-budget.md contains "180 MHz" on line 18, which appears to be an error (the H723 runs at up to 550 MHz; BTT Octopus Pro documentation and CLAUDE.md both state 520 MHz).
- The analysis assumes the f64 data is in registers or L1 cache (no DTCM/AXI stalls). For a 5-register accumulator state per motor, this is reasonable.

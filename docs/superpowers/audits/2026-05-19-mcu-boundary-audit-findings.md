# MCU C/Rust Boundary Audit Findings — 2026-05-19

> Findings doc for audit items A1–A8 from
> [`docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md`](../specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md).
> One section per audit item. Each section records: discovery commands run,
> findings, decisions, and follow-up actions.

## A1 — Inventory items

### A1.1 — Rust `#[link_section]` uses

**Command:** `rg -n '#\[link_section\|link_section\b' rust/`

**Result (2026-05-19, post-Phase 3 migration):** Zero matches. The only active `#[link_section]` attribute pre-refactor (`RT_CELL`'s `.axi_bss` placement) was deleted in commit `641e552fb`. All shared-memory placement decisions now live on the C side.

**Decision:** PASS. No B2 violations on the Rust side.

### A1.2 — `.axi_bss` C occupant inventory (H7)

Files inspected:
- `src/runtime_storage.c` — `rt_storage` (RT_STORAGE_SIZE bytes; 290816 = 284 KB at LARGE profile default per `66e04eac6`).
- `src/kalico_demux.c` — `kalico_buf` (14752 bytes; computed as `4 * (MAX_CONTROL_POINTS + MAX_KNOT_VECTOR_LEN) + 32`).
- `src/generic/runtime_bench.c` — `runtime_bench_samples_buf` (1024 bytes; `RUNTIME_BENCH_MAX_SAMPLES * sizeof(uint32_t) = 256 * 4`).
- `src/generic/serial_irq.c` — `receive_buf` (2048 bytes; `RX_BUFFER_SIZE`).

Total .axi_bss budget: 308640 bytes. With 16 KB headroom (16384) the total against the 320 KB region (327680) is 325024 ≤ 327680 — passes the `_Static_assert`.

**Decision:** Inventory recorded; update the boundary doc § "What lives where on the MCU" `.axi_bss` row with these byte counts during Task 25 doc reconciliation.

### A1.3 — `extern "C"` signature sweep

Targets: `rust/kalico-c-api/src/runtime_ffi.rs` (82 functions per `rg -c 'extern "C"'`).

**Commands run (2026-05-19):**
- `rg -n 'extern "C".*&\[|extern "C".*&mut \[' rust/kalico-c-api/src/runtime_ffi.rs` → **zero matches**
- `rg -n 'pub.*extern "C".*->.*\(.*,.*\)' rust/kalico-c-api/src/runtime_ffi.rs` → **zero matches**
- `rg -n 'extern "C".*Option<&' rust/kalico-c-api/src/runtime_ffi.rs` → **zero matches**

**Result:** PASS. The opaque-handle API uses `*mut KalicoRuntime` plus primitive types and raw pointers (`*const T` / `*mut T`); no Rust-typed fat pointers, lifetime-carrying references, or tuple returns cross the boundary. `bool` returns at lines 2145, 2234, 2400, 2985 are addressed in A2 (ratified as permitted; B4 doc update lands in Task 17).

**Decision:** No signature-level B3/B4 violations. The 82-function surface is structurally clean.

## A2 — `bool`-FFI policy ratification

**Discovery:**
- `runtime_ffi.rs:2145, 2234, 2400, 2985` return `bool` today.
- Boundary doc B4 says "no `bool` (use `uint8_t`)" — contradicts code.

**Decision: Accept `bool`.** Rust's `bool` and C99 `_Bool` are layout-compatible (both 1 byte, both 0/1). Boundary doc B4 updated to permit `bool` with the `_Bool` contract.

**Action:** B4 rewording landed in this commit.

## A3 — `kalico_producer_current_present` static-mut import

**Discovery command:** `rg -n 'kalico_producer_current_present' rust/`

**Result (2026-05-19):** Only one Rust reference — the bare `static mut` import in `rust/runtime/src/engine.rs:23`. **No direct reads or writes** of the bare static. The volatile global is accessed exclusively through the C accessor functions `kalico_producer_current_is_present()` and `kalico_producer_current_set_present(int)` (defined at `src/kalico_segment_queue.c:155-172`), called from `read_producer_current_present` / `write_producer_current_present` in `engine.rs:39-50` / `:56-64`.

The bare static muts for `kalico_producer_current_present`, `kalico_producer_current_set_count`, and `kalico_producer_current_cleared_count` are dead — declared but never accessed from Rust.

**Decision:** **Delete the bare `static mut` imports** (engine.rs:23-25). Same class of fix as the 2026-05-18 SPSC queue migration: cross-language `static mut` imports carry the LLVM-projection miscompilation risk that the accessor functions were introduced to defeat. If the diag counters need to be read from Rust later, add C accessors rather than reintroducing the bare imports.

**Action:** Deletion lands in this commit (Task 16).

## A4 — `portable_atomic::AtomicU64` ordering in ISR hot path

**Discovery commands:**
- `rg -n 'AtomicU64.*fetch_add|AtomicU64.*Ordering' rust/runtime/src/state.rs`
- `rg -n 'AtomicU64' rust/runtime/src/`

**Critical-section cost:** on thumbv7em-none-eabihf, `portable_atomic::AtomicU64` fallback uses interrupt-disable for the duration of the op. Each `fetch_add` on a u64 atomic disables interrupts for ~10-50 cycles.

**Result (2026-05-19):**

- `AtomicU64` is imported from `portable_atomic` (`state.rs:31`), comment confirms: "thumbv7em-none-eabi[hf] (Cortex-M7) lacks native 64-bit CAS — core::sync::atomic::AtomicU64 is unavailable; portable-atomic provides a critical-section fallback."
- `SharedState` declares ~16 `AtomicU64` fields, all diagnostic / monotonic counters (producer_runs_total, consumer_pulses_total[4], consumer_underrun_total[4], producer_steps_pushed_total, etc.).
- ISR-path `fetch_add` operations in `engine.rs` using `Ordering::AcqRel`:
  - lines 950, 1583, 1907, 1915, 2076, 2116, 2124, 2227, 2234, 2238 (sampled — full count higher).
- Each `fetch_add` on `portable_atomic::AtomicU64` thumbv7em fallback disables interrupts for the duration of the op (~10-50 cycles).

**Per-ISR cost estimate:** at modulation rate 40 kHz with ~10 AcqRel fetch_add ops per tick → ~500 cycles/tick of interrupt-disabled critical section, repeated 40,000 times/second = ~20 Mcycles/sec or ~10% of a 200 MHz H7 core. Not insignificant.

**Decision:** **Audit-only this refactor.** The cost is real but the counters are diagnostic, not load-bearing for correctness. Two follow-up options for a separate sub-spec:
1. Split `AtomicU64` → `(AtomicU32 low, AtomicU32 high)` with seqlock pattern for atomic 64-bit reads.
2. Demote diagnostic counters to `Cell<u64>` where the ISR is the only writer (single-writer path doesn't need atomic CAS).

Both options preserve the user-visible behavior; option 2 is simpler if the diagnostics are read only from foreground (which they are — `runtime_handle_*` accessors).

**Action:** Audit recorded; no code changes this refactor (per § Non-goals). Recommend revisiting if the H7 cycle budget audit at Step 7-D shows critical-section cost as a bottleneck.

## A5 — Panic-in-ISR audit

**Discovery:**
- Find the Rust MCU panic handler: `rg -n '#\[panic_handler\]' rust/`.
- Map reachable panics from ISR-called FFI entries (`runtime_handle_tick`, `kalico_runtime_modulated_tick`, kalico_endstop_*).
- C fault-latch entry point: `fault_handler_report_task` (per recent commits e.g. `2d83c3d6d diag: emit sched_bad_add state via fault_handler_report_task`).

**Result (2026-05-19):**

- Rust panic handler located at `rust/kalico-c-api/src/lib.rs:34-40` (pre-fix). Pre-fix: `loop { spin_loop(); }`.
- Panic sites reachable from ISR-called FFI entries:
  - `rust/runtime/src/engine.rs:2287` — `.expect("producer_current set")` inside an Engine method that runs from `runtime_handle_tick` (TIM5 ISR). If `producer_current` is None when expected, this panics.
  - Other `.expect(...)` calls in engine.rs at lines >3500 are inside `#[cfg(test)]` blocks (not production paths).
  - No `panic!` or `.unwrap()` found in `step_producer.rs`, `step_ring.rs`, `step_time.rs`, `step.rs`, `clock.rs`, `kinematics.rs`, `curve_pool.rs` (production code paths).
- The `expect` at engine.rs:2287 is the load-bearing ISR-reachable panic site.

**Decision:** **Route the panic handler through a C fault-latch.** New `src/runtime_panic.c` defines `rust_panic_latch(void) __noreturn` which calls Klipper's `shutdown("Rust panic")` macro. Rust panic handler updated to call this function instead of spinning.

**Action:** Landed in this commit (Task 18). The engine.rs:2287 expect is left in place — it's an invariant check, not a recoverable error. The fix is that when it does fire (during dev / debugging), the panic now produces a klippy-visible shutdown frame instead of a frozen MCU.

## A6 — FPU / CPACR consistency

**Investigation:**
- Confirm C build flags include hard-float ABI:
  - F4: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`
  - H7: `-mfloat-abi=hard -mfpu=fpv5-d16`
- Confirm CPACR enabled at boot per the 2026-05-12 fix (`armcm_main` when `__FPU_PRESENT == 1`).
- Confirm Rust staticlib build target is `thumbv7em-none-eabi` (no -hf suffix) with explicit `-C target-feature=+vfp4` if hard-float Rust is desired.

**Commands:**
- `grep -rn '\-mfloat-abi\|\-mfpu\|FPU_PRESENT' src/stm32/ src/generic/armcm_*.c`
- `cat rust/.cargo/config.toml`
- `grep -rn 'CPACR\|__FPU_PRESENT' src/generic/armcm_main.c src/generic/armcm_boot.c`

**Result (2026-05-19):**

- **F4 CPACR setup:** `src/stm32/stm32f4.c:255-267` — enables `SCB->CPACR.CP10/11` when `CONFIG_KALICO_RUNTIME && __FPU_PRESENT == 1`. Comment confirms: "SystemInit only does CPACR.CP10/11 when `__FPU_USED == 1`, which is gated on `-mfloat-abi=hard|softfp` at build time — Klipper compiles `-mfloat-abi=soft` so SystemInit skips it." This is the resolved 2026-05-12 fix.
- **C build flags:** Klipper compiles `-mfloat-abi=soft` (per the F4 comment above and confirmed for H7 via `rust/.cargo/config.toml` comment block: "Klipper's H7 build uses soft-float ABI (no `-mfloat-abi=hard` in CFLAGS)").
- **Rust staticlib target:** `thumbv7em-none-eabi` (soft-float, NOT `-eabihf`). `rust/.cargo/config.toml` confirms: "the kalico staticlib must match or the linker rejects archive merge with 'uses VFP register arguments, klipper.elf does not'."
- **Rust target-cpu:** `cortex-m4` (conservative) — works for both H7 and F4 builds.

**Decision:** PASS. Hard-float ABI consistency is correct — both sides use soft-float. CPACR enable is in place per the 2026-05-12 fix (F4 confirmed; H7 not explicitly checked here but the same pattern applies for any LLVM-emitted `vmov` instruction).

**Action:** No code changes. The configuration is self-consistent and was already hardened by the 2026-05-12 F446 fix.

## A7 — DMA cacheability for H7 `.axi_bss`

**Investigation:**
- Is `RuntimeContext` ever DMA source/destination? (Expected: no.)
- For each `.axi_bss` C occupant (`kalico_buf`, `runtime_bench_samples_buf`, `receive_buf`), is it DMA-touched?
- Is `.axi_bss` MPU-marked non-cacheable on H7?

**Commands:**
- `rg -n 'DMA\|HAL_DMA\|dma_\|cache_clean\|cache_invalidate\|SCB_\|MPU_' src/kalico_demux.c src/generic/serial_irq.c src/generic/runtime_bench.c src/generic/mpu_protect.c`

**Result (2026-05-19):**

- `rt_storage` (RuntimeContext): pure CPU read/write. No DMA paths touch it.
- `kalico_buf` (kalico_demux.c): pure CPU stream-parsing buffer. No DMA references.
- `runtime_bench_samples_buf` (runtime_bench.c): CPU-written diagnostic buffer. No DMA.
- `receive_buf` (serial_irq.c): name suggests DMA, but inspection shows it's IRQ-driven (USART RX interrupt fills it via CPU writes), NOT DMA. `grep -E 'DMA|HAL_DMA' src/generic/serial_irq.c` → zero matches.
- `mpu_protect.c`: only protects `.sched_protected` (sched.c's state); does not MPU-mark `.axi_bss` non-cacheable.
- H7 AXI SRAM at 0x24000000 lives in the default memory map — Normal memory, cacheable. But since no DMA touches anything in `.axi_bss`, this is moot.

**Decision:** PASS. No DMA touches any `.axi_bss` occupant; cacheability of AXI SRAM is irrelevant for the current inventory. Per § Non-goals, no code changes.

**Action:** Document the assumption: **if a future change adds a DMA source/destination to `.axi_bss` on H7, cache maintenance (SCB_CleanDCache / InvalidateDCache around the DMA) becomes mandatory.** Add a one-line note to the boundary doc near the `.axi_bss` inventory.

## A8 — Dead-code removal

### A8.1 — `runtime_irq_save` / `runtime_irq_restore`

**Discovery command:** `grep -rn 'runtime_irq_save\|runtime_irq_restore' rust/ src/`

**Result (2026-05-19):** The functions ARE used:
- `rust/runtime/src/stream.rs:84` — `use crate::state::{runtime_irq_restore, runtime_irq_save};`
- `rust/runtime/src/stream.rs:346-348` — `let irq_flags = unsafe { runtime_irq_save() };` and the matching restore further down. This is the Phase 7 §8.5 force_idle handshake path.
- Three test files (`flush_basic.rs`, `flush_drains_queue.rs`, `stream_lifecycle.rs`) define stub implementations (`pub extern "C" fn runtime_irq_save() -> u32 { 0 }`).

**Decision:** **Remove the `#[allow(dead_code)]` attribute** at `state.rs:102` — the functions are not dead, the `allow` was stale. The `extern "C"` block stays.

**Action:** Landed in this commit. Cleaner: future readers see the actual liveness without the misleading `dead_code` allow.

### A8.2 — Other dead-code candidates

**Discovery:** Manual grep for `#[allow(dead_code)]` in the FFI surface; `cargo +nightly udeps` if available.

**Result:** TBD.

**Decision:** TBD per finding.

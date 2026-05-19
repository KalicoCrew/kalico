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

**Result:** TBD — enumerate ISR-path AtomicU64 ops and quantify per-tick cost.

**Decision:** **Audit-only this refactor.** If the audit decides a split-into-`AtomicU32`-pair is needed, that work goes into a follow-up sub-spec. Locked per spec § Non-goals.

Lands in Task 19.

## A5 — Panic-in-ISR audit

**Discovery:**
- Find the Rust MCU panic handler: `rg -n '#\[panic_handler\]' rust/`.
- Map reachable panics from ISR-called FFI entries (`runtime_handle_tick`, `kalico_runtime_modulated_tick`, kalico_endstop_*).
- C fault-latch entry point: `fault_handler_report_task` (per recent commits e.g. `2d83c3d6d diag: emit sched_bad_add state via fault_handler_report_task`).

**Result:** TBD — fill in during audit run.

**Decision:** If panics reachable in ISR context, route the panic handler through the C fault-latch path rather than the current spin-forever handler.

Lands in Task 18.

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

**Result:** TBD.

**Decision:** Investigation-only. Document drift if found; no code changes per § Non-goals (this is a separate-PR concern if any inconsistency surfaces).

Lands in Task 19.

## A7 — DMA cacheability for H7 `.axi_bss`

**Investigation:**
- Is `RuntimeContext` ever DMA source/destination? (Expected: no.)
- For each `.axi_bss` C occupant (`kalico_buf`, `runtime_bench_samples_buf`, `receive_buf`), is it DMA-touched?
- Is `.axi_bss` MPU-marked non-cacheable on H7?

**Commands:**
- `rg -n 'DMA\|HAL_DMA\|dma_\|cache_clean\|cache_invalidate\|SCB_\|MPU_' src/kalico_demux.c src/generic/serial_irq.c src/generic/runtime_bench.c src/generic/mpu_protect.c`

**Result:** TBD.

**Decision:** Per spec § Non-goals: investigation-only; no code changes.

Lands in Task 19.

## A8 — Dead-code removal

### A8.1 — `runtime_irq_save` / `runtime_irq_restore`

**Discovery command:** `rg -n 'runtime_irq_save\|runtime_irq_restore' rust/ src/`

**Result:** TBD — fill in during audit run.

**Decision:** If unused, delete the declarations from `rust/runtime/src/state.rs:103-106`. If still used (Phase 7 §8.5 flush path), remove the `dead_code` allow attribute.

Lands in Task 20.

### A8.2 — Other dead-code candidates

**Discovery:** Manual grep for `#[allow(dead_code)]` in the FFI surface; `cargo +nightly udeps` if available.

**Result:** TBD.

**Decision:** TBD per finding.

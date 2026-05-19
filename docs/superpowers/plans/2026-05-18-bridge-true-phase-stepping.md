# Bridge wiring for true TMC5160 phase stepping — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire klippy's `[stepper_*] phase_stepping: True` through to actual TMC5160 XDIRECT-driven motion on the H723 bench, so `DUMP_TMC STEPPER=stepper_x` reports `direct_mode=1` and `runtime_modulated_tick` dispatches to `PhaseDirectModulator` rather than the step-pulse fallback.

**Architecture:** Three integration gaps to close in sequence:
1. `rust/motion-bridge/src/bridge.rs::configure_axes` emits the 33-byte ConfigureAxes body with per-motor `(bus_id, cs_pin_id)`.
2. Klippy invokes `runtime_register_phase_bus` before `configure_axes` so the firmware's `phase_stepping_spi.c` can drive SPI3.
3. Klippy's existing TMC field infrastructure queues `GCONF.direct_mode=1` and `IHOLDIRUN.IHOLD=run_current` for phase-stepped TMC5160 chips at connect time.
   Plus an ISR-priority busy-flag in `phase_stepping_spi.c` to mediate SPI3 contention between TIM5-rate XDIRECT writes and Klipper's 1 Hz `DRV_STATUS` polling.

**Tech Stack:** Rust (motion-bridge crate, PyO3), Python (klippy/extras, klippy/motion_*), C (src/stm32/), Renode sim, TMC5160 SPI.

**Spec reference:** [`docs/superpowers/specs/2026-05-18-bridge-true-phase-stepping-design.md`](../specs/2026-05-18-bridge-true-phase-stepping-design.md).

---

## File map

| File | Action | Why |
|---|---|---|
| `rust/motion-bridge/src/bridge.rs` | Modify | Extract `build_configure_axes_body` helper; extend `configure_axes` for `phase_configs`; add `register_phase_bus` PyO3 method. |
| `rust/motion-bridge/src/bridge.rs` (tests mod) | Modify | Unit tests against the new body-builder helper. |
| `klippy/extras/tmc2130.py` | Modify | Expose `get_bus_and_cs_ids()` on `MCU_TMC_SPI_chain`. |
| `klippy/extras/tmc5160.py` | Modify | Direct-mode init guarded by sister `[stepper_*]` `phase_stepping=True`; `get_phase_config()` accessor; CurrentHelper direct-mode awareness. |
| `klippy/motion_toolhead.py` | Modify | Build `phase_configs[4]` array; call `register_phase_bus` per unique bus; thread `phase_configs` into `configure_axes`. |
| `klippy/motion_bridge.py` | Modify | Add `register_phase_bus` wrapper; thread `phase_configs` kwarg through `configure_axes`. |
| `src/stm32/phase_stepping_spi.c` | Modify | Add `phase_spi_busy` flag + `phase_spi_skip_count`; gate `phase_stepping_write_xdirect` on it. |
| `src/stm32/phase_stepping_spi.h` | Modify | Declare new acquire/release primitives + skip-count accessor. |
| `src/stm32/stm32h7_spi.c` | Modify | Bidirectional acquire/release hook around `spi_transfer` for the registered phase bus. |
| `src/runtime_commands.c` (status surface) | Modify | Expose `phase_spi_skip_count` via the existing kalico status / trace surface. |
| `rust/runtime/src/ffi.rs` (or wherever C-side skip_count is read) | Modify | Surface skip_count on whatever struct gets reported in `runtime_query_status`. |
| `tools/test_sim_phase_stepping.py` | Modify | New regression case asserting 33-byte body shape, Tmc5160 stub captures GCONF+XDIRECT, skip_count==0 in sim. |

---

## Task 1: Refactor `configure_axes` body construction into a testable helper

**Why first:** decouples the wire-format logic from the PyO3 transport layer so subsequent tasks can write fast unit tests against the byte builder without standing up a full mock transport.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:1013-1029` (extract body-construction block)
- Modify: `rust/motion-bridge/src/bridge.rs` tests module (add at the bottom, near line 2484)

- [ ] **Step 1: Add the helper signature + a stub returning Vec::new()**

In `rust/motion-bridge/src/bridge.rs`, just above `impl PyMotionBridge { ... fn configure_axes(...)` (find the `impl PyMotionBridge` block starting around line 414), add the free function:

```rust
/// Build the kalico-native `ConfigureAxes` wire body.
///
/// Body layouts:
///   - 20 bytes when `step_modes` and `phase_configs` are both None
///     (legacy path; kinematics + 3 masks + 4 × f32 steps_per_mm).
///   - 25 bytes when `step_modes` is Some, `phase_configs` is None
///     (Step 7-B: adds phase_capable flag + 4-byte step_mode array).
///   - 33 bytes when both `step_modes` and `phase_configs` are Some
///     (Step 7-D / true phase stepping: bytes 25..32 carry 4 ×
///     (bus_id u8, cs_pin_id u8) pairs interleaved).
///
/// `phase_capable` is the identify-time PHASE_STEPPING bit (bit 0 of
/// `identify_caps`). It is purely an MCU-side sanity check; the wire
/// position is fixed at byte 20 for the 25-byte and 33-byte layouts.
pub(crate) fn build_configure_axes_body(
    kinematics: u8,
    present_mask: u8,
    awd_mask: u8,
    invert_mask: u8,
    steps_per_mm: &[f32; 4],
    step_modes: Option<&[u8; 4]>,
    phase_configs: Option<&[(u8, u8); 4]>,
    phase_capable: u8,
) -> Vec<u8> {
    let mut body = Vec::with_capacity(33);
    body.push(kinematics);
    body.push(present_mask);
    body.push(awd_mask);
    body.push(invert_mask);
    for v in steps_per_mm {
        body.extend_from_slice(&v.to_le_bytes());
    }
    if let Some(sm) = step_modes {
        body.push(phase_capable);
        for &m in sm.iter() {
            body.push(m);
        }
    }
    if let Some(pc) = phase_configs {
        debug_assert!(
            step_modes.is_some(),
            "phase_configs requires step_modes (33-byte format extends 25-byte)"
        );
        for &(bus_id, cs_pin_id) in pc.iter() {
            body.push(bus_id);
            body.push(cs_pin_id);
        }
    }
    body
}
```

- [ ] **Step 2: Replace the inline body construction in `configure_axes` with a call to the helper**

In `configure_axes` around line 1013-1029, replace:

```rust
let mut body = Vec::with_capacity(25);
body.push(kinematics);
body.push(present_mask);
body.push(awd_mask);
body.push(invert_mask);
for v in &steps_per_mm {
    body.extend_from_slice(&v.to_le_bytes());
}
if let Some(ref sm) = step_modes {
    // byte 20: phase_capable flag (bit 0 from identify capabilities)
    let phase_capable: u8 = if identify_caps & 0x1 != 0 { 1 } else { 0 };
    body.push(phase_capable);
    // bytes 21-24: step_mode[0..4]
    for &m in sm.iter().take(4) {
        body.push(m);
    }
}
```

with:

```rust
let phase_capable: u8 = if identify_caps & 0x1 != 0 { 1 } else { 0 };
let steps_arr: [f32; 4] = [steps_per_mm[0], steps_per_mm[1], steps_per_mm[2], steps_per_mm[3]];
let step_modes_arr: Option<[u8; 4]> = step_modes.as_ref().map(|sm| [sm[0], sm[1], sm[2], sm[3]]);
let body = build_configure_axes_body(
    kinematics,
    present_mask,
    awd_mask,
    invert_mask,
    &steps_arr,
    step_modes_arr.as_ref(),
    None,             // Task 4 threads phase_configs here
    phase_capable,
);
```

- [ ] **Step 3: Add the failing unit test for the legacy 20-byte path**

In the existing `#[cfg(test)] mod tests` block (near line 2469), add:

```rust
#[test]
fn build_configure_axes_body_legacy_20() {
    let body = build_configure_axes_body(
        /* kinematics */ 0,
        /* present_mask */ 0x0F,
        /* awd_mask */ 0x03,
        /* invert_mask */ 0,
        &[160.0, 160.0, 800.0, 800.0],
        /* step_modes */ None,
        /* phase_configs */ None,
        /* phase_capable */ 0,
    );
    assert_eq!(body.len(), 20, "legacy body is 20 bytes");
    assert_eq!(body[0], 0);
    assert_eq!(body[1], 0x0F);
    assert_eq!(body[2], 0x03);
    assert_eq!(body[3], 0);
    assert_eq!(&body[4..8], &160.0f32.to_le_bytes());
    assert_eq!(&body[16..20], &800.0f32.to_le_bytes());
}
```

- [ ] **Step 4: Add the failing unit test for the 25-byte step-modes path**

```rust
#[test]
fn build_configure_axes_body_step_modes_25() {
    let body = build_configure_axes_body(
        0, 0x0F, 0x03, 0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 0, 1, 1]),
        None,
        /* phase_capable */ 1,
    );
    assert_eq!(body.len(), 25, "step-modes body is 25 bytes");
    assert_eq!(body[20], 1, "byte 20 carries phase_capable");
    assert_eq!(&body[21..25], &[0u8, 0, 1, 1], "step_modes array");
}
```

- [ ] **Step 5: Add the failing unit test for the 33-byte phase_configs path**

```rust
#[test]
fn build_configure_axes_body_phase_configs_33() {
    let body = build_configure_axes_body(
        0, 0x0F, 0x03, 0,
        &[160.0, 160.0, 800.0, 800.0],
        Some(&[0, 0, 1, 1]),
        Some(&[(3, 5), (3, 6), (0xFF, 0xFF), (0xFF, 0xFF)]),
        /* phase_capable */ 1,
    );
    assert_eq!(body.len(), 33, "phase-configs body is 33 bytes");
    // Legacy + step_modes portion unchanged
    assert_eq!(body[20], 1);
    assert_eq!(&body[21..25], &[0u8, 0, 1, 1]);
    // Phase-configs portion: 4 × (bus_id, cs_pin_id), interleaved
    assert_eq!(
        &body[25..33],
        &[3u8, 5, 3, 6, 0xFF, 0xFF, 0xFF, 0xFF],
        "bytes 25..33 are (bus_id, cs_pin_id) pairs",
    );
}
```

- [ ] **Step 6: Run the tests**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
cargo test -p motion-bridge --lib build_configure_axes_body
```

Expected: 3 tests pass.

- [ ] **Step 7: Run the full motion-bridge test suite to check for regressions**

```bash
cargo test -p motion-bridge --lib
```

Expected: all tests pass (the refactor preserved the existing wire format because Task 1 doesn't yet thread phase_configs through `configure_axes` — the call passes `None`).

- [ ] **Step 8: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "refactor(motion-bridge): extract build_configure_axes_body helper

Pure-function body builder unit-tested for all three layout variants
(20/25/33 bytes). configure_axes PyO3 wrapper now delegates. No wire-
format change; phase_configs threading lands in a follow-up task."
```

---

## Task 2: Add `register_phase_bus` PyO3 method on `PyMotionBridge`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add new method inside the second `impl PyMotionBridge` block near line 2399)

- [ ] **Step 1: Add the method skeleton**

Find the `impl PyMotionBridge` block around line 2399 (the one that contains the `#[pymethods]` wrapper if applicable — note the file has the `#[pymethods]` attribute on `impl PyMotionBridge` further up; mirror whichever block already contains methods like `attach_serial`). Add:

```rust
/// Register a phase-stepping SPI bus with the MCU. Must be called once
/// per unique `bus_id` BEFORE `configure_axes` for that MCU. Wraps the
/// `runtime_register_phase_bus bus_id=%c cs_pin_id=%c rate=%u` wire
/// command defined at `src/runtime_commands.c:556`. `cs_pin_id` is the
/// firmware's GPIO encoding (port * 16 + pin); the firmware-side
/// command handler calls `spi_setup(bus_id, mode=3, rate)` and
/// `gpio_out_setup(cs_pin_id, 1 /* idle high */)`.
///
/// `cs_pin_id` argument here is "an anchor pin to identify the bus";
/// real per-motor CS routing comes from `phase_configs` in the
/// `configure_axes` 33-byte body. The firmware caches one CS per bus
/// in `phase_buses[bus_id].cs` (see `phase_stepping_spi.c:43`); for
/// the v1 single-bus-per-axis case the anchor pin == the per-motor pin.
#[pyo3(signature = (mcu_handle, bus_id, cs_pin_id, rate, timeout_s = 5.0))]
fn register_phase_bus(
    &self,
    py: Python<'_>,
    mcu_handle: u32,
    bus_id: u8,
    cs_pin_id: u8,
    rate: u32,
    timeout_s: f64,
) -> PyResult<()> {
    let io = {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "register_phase_bus: unknown mcu_handle {mcu_handle}"
            ))
        })?;
        if !conn.kalico_native_supported {
            // Stock-Klipper MCU: silently no-op so multi-MCU setups
            // where Z lives on an F446 with no phase-stepping still
            // complete the per-MCU iteration.
            return Ok(());
        }
        conn.host_io.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_bus: attach_serial has not been called",
            )
        })?.clone()
    };
    let timeout = std::time::Duration::from_secs_f64(timeout_s);
    let msg = format!(
        "runtime_register_phase_bus bus_id={bus_id} cs_pin_id={cs_pin_id} rate={rate}"
    );
    let resp = py.allow_threads(|| {
        io.bridge_call(&msg, "kalico_register_phase_bus_response", timeout)
    });
    match resp {
        Ok(params) => {
            let result = params.get("result")
                .and_then(|v| v.as_i64())
                .ok_or_else(|| PyRuntimeError::new_err(
                    "register_phase_bus: response missing result field"
                ))?;
            if result != 0 {
                return Err(PyRuntimeError::new_err(format!(
                    "register_phase_bus: MCU returned error {result} \
                     (bus_id={bus_id} cs_pin_id={cs_pin_id})"
                )));
            }
            Ok(())
        }
        Err(e) => Err(PyRuntimeError::new_err(format!(
            "register_phase_bus: transport error: {e:?}"
        ))),
    }
}
```

**Note on `io.bridge_call`:** the existing `bridge_call` PyO3 method (lines 996ff in motion_bridge.py confirmed) takes `(mcu_handle, msg, response, timeout_s)` and returns a dict-style response. If the internal Rust signature on `HostIo::bridge_call` differs — e.g. it takes `kalico_protocol::MessageKind` rather than a wire string — find the analogous call site for `attach_serial` or another command using msgproto and follow that pattern. Inspect: `grep -n "fn bridge_call\|fn kalico_call" rust/motion-bridge/src/host_io.rs` (or wherever `HostIo` lives) to confirm.

- [ ] **Step 2: Build to verify the method compiles**

```bash
cargo build -p motion-bridge
```

Expected: clean build. If `io.bridge_call` signature differs from what we wrote, fix to match — the contract is "send msgproto wire string, await named response, get the result field as i64".

- [ ] **Step 3: Add an integration test exercising the round-trip (sim-style)**

This test requires fixture-style transport mocking. The existing tests in `rust/motion-bridge/tests/sim_motion_jogs.rs` use the Renode sim path; we add the assertion there in Task 11 once the sim's Tmc5160 stub is also asserting receipt. For Task 2, the unit-level acceptance is "compiles + smoke-builds when invoked from Python in Task 7".

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): add register_phase_bus PyO3 method

Wraps the existing runtime_register_phase_bus wire command so klippy
can register an SPI bus with the MCU's phase-stepping subsystem
before configure_axes is sent. No klippy caller yet; that lands in
the motion_toolhead task."
```

---

## Task 3: Expose bus/CS ids on `MCU_TMC_SPI_chain`

**Files:**
- Modify: `klippy/extras/tmc2130.py:275-358` (the `MCU_TMC_SPI_chain` class)

- [ ] **Step 1: Understand the existing resolution path**

`MCU_TMC_SPI_chain.__init__` (line 277) constructs `self.spi = bus.MCU_SPI_from_config(config, 3, ...)`. `MCU_SPI_from_config` (in `klippy/extras/bus.py:155`) does the string resolution: `cs_pin = config.get("cs_pin")` → `cs_pin_params = ppins.lookup_pin(cs_pin, ...)` → `pin = cs_pin_params["pin"]` (string like `"PA5"`), and `bus = config.get("spi_bus", None)` (string like `"spi3"`). These strings are stored on the resulting `MCU_SPI` instance as `self.bus` (string bus name) and embedded in the `config_spi` command via the per-MCU pin-resolver.

The runtime needs **integer** IDs (firmware's `spi_setup(uint8_t bus_id, …)` / `gpio_out_setup(uint8_t pin_id)`). These integers come from the MCU's identify-time enumeration tables (`pin` and `spi_bus` enums), which klippy already parses and stashes on the message parser as `mcu.get_msgparser().enumerations`.

- [ ] **Step 2: Add an `MCU_TMC_SPI_chain.__init__` line to resolve and stash integer IDs**

Inside `MCU_TMC_SPI_chain.__init__` (after the `self.spi = bus.MCU_SPI_from_config(...)` line at ~line 286), add:

```python
# 2026-05-18 phase-stepping integration: resolve the SPI bus name
# ("spi3") and CS pin name ("PA5") to the integer IDs the firmware's
# spi_setup / gpio_out_setup expect. The msgparser's enumeration tables
# were populated at identify time from the MCU's data dictionary.
self._phase_bus_id = None
self._phase_cs_pin_id = None
try:
    ppins = config.get_printer().lookup_object("pins")
    cs_pin_str = config.get("cs_pin")
    cs_pin_params = ppins.lookup_pin(cs_pin_str, share_type="tmc_spi_cs")
    pin_str = cs_pin_params["pin"]    # e.g. "PA5"
    mcu = cs_pin_params["chip"]
    bus_str = config.get("spi_bus", None)  # e.g. "spi3"
    if bus_str is not None:
        enums = mcu.get_msgparser().enumerations
        self._phase_bus_id = enums.get("spi_bus", {}).get(bus_str)
        self._phase_cs_pin_id = enums.get("pin", {}).get(pin_str)
except Exception:
    # Resolution failures are non-fatal here; get_bus_and_cs_ids()
    # below raises if the caller actually needs the values and they
    # weren't resolvable (e.g. software SPI, missing enums, etc.).
    pass
```

If `enums["spi_bus"]` or `enums["pin"]` shape differs from `dict[str, int]` in the actual codebase, adjust accordingly — inspect with:

```bash
grep -n "enumerations" /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/klippy/msgproto.py | head -5
```

The same `enumerations` dict drives the wire-protocol substitution for `%s` parameters with enum names, so it definitely exists and is indexed by enum name → mapping. The mapping is typically `{name: id}` or `{name: (id, count)}`; if it's the latter, take `[0]`.

- [ ] **Step 3: Add the accessor**

After `MCU_TMC_SPI_chain.__init__` exits but inside the class (just before `_build_cmd`), add:

```python
def get_bus_and_cs_ids(self):
    """Return (bus_id, cs_pin_id) as integers matching the firmware's
    spi_setup / gpio_out_setup. Raises if either was not resolvable
    (e.g. software SPI, missing enumeration). Used by the phase-
    stepping bridge integration (motion_toolhead._configure_axes_per_mcu).
    """
    if self._phase_bus_id is None or self._phase_cs_pin_id is None:
        raise self.printer.config_error(
            "TMC SPI bus/pin could not be resolved to integer IDs "
            "(software SPI or missing MCU enumeration?); phase stepping "
            "requires hardware SPI with enumerated bus and pin."
        )
    return (self._phase_bus_id, self._phase_cs_pin_id)
```

- [ ] **Step 4: Quick manual verification**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
python3 -c "
import ast, sys
tree = ast.parse(open('klippy/extras/tmc2130.py').read())
for node in ast.walk(tree):
    if isinstance(node, ast.ClassDef) and node.name == 'MCU_TMC_SPI_chain':
        methods = [m.name for m in node.body if isinstance(m, ast.FunctionDef)]
        assert 'get_bus_and_cs_ids' in methods, methods
        print('OK:', methods)
        sys.exit(0)
print('MCU_TMC_SPI_chain not found'); sys.exit(1)
"
```

Expected: `OK: ['__init__', 'get_bus_and_cs_ids', ...]`

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/tmc2130.py
git commit -m "feat(klippy/tmc2130): expose bus_id/cs_pin_id accessors

MCU_TMC_SPI_chain.get_bus_and_cs_ids() returns the numeric (bus_id,
cs_pin_id) used by the config_spi MCU command. Needed by the upcoming
phase-stepping bridge integration to populate the 33-byte
configure_axes blob and to call runtime_register_phase_bus."
```

---

## Task 4: TMC5160 `direct_mode` init + `get_phase_config()` + validation

**Files:**
- Modify: `klippy/extras/tmc5160.py` (TMC5160 class, around line 367)

- [ ] **Step 1: Add the helper at module level**

Before the `class TMC5160:` line, add:

```python
def _enable_direct_mode(config, stepper_section, fields):
    """Configure a TMC5160 for phase stepping (direct_mode=1).

    Sets GCONF.direct_mode via the field collector so the value lands in
    the connect-time SPI burst. Validates that incompatible options
    (stealthchop_threshold > 0, microsteps != 256) are absent — raises
    config.error on violation.
    """
    fields.set_field("direct_mode", 1)
    sct = stepper_section.getfloat("stealthchop_threshold", 0., minval=0.)
    if sct > 0.:
        raise config.error(
            "phase_stepping=True is incompatible with stealthchop_threshold "
            "(StealthChop is bypassed in direct mode). Remove "
            "stealthchop_threshold from [%s] or disable phase_stepping."
            % stepper_section.get_name()
        )
    mres = config.getint("microsteps", 256)
    if mres != 256:
        raise config.error(
            "phase_stepping=True requires microsteps: 256; [%s] has "
            "microsteps: %d." % (config.get_name(), mres)
        )
```

- [ ] **Step 2: Wire it into `TMC5160.__init__`**

At the end of `TMC5160.__init__` (right after the last `set_config_field(config, "tpowerdown", 10)` call), add:

```python
# 2026-05-18 phase-stepping integration: when the matching
# [stepper_*] section sets phase_stepping=True, queue GCONF.direct_mode=1
# and validate incompatible options. The TMC SPI burst at klippy
# connect time will write the bit before any motion starts.
stepper_name = " ".join(config.get_name().split()[1:])  # "stepper_x"
printer = config.get_printer()
try:
    stepper_section = config.getsection(stepper_name)
except Exception:
    stepper_section = None
self._phase_stepping = False
self._phase_bus_id = None
self._phase_cs_pin_id = None
if stepper_section is not None and stepper_section.getboolean(
    "phase_stepping", False
):
    _enable_direct_mode(config, stepper_section, self.fields)
    self._phase_stepping = True
    self._phase_bus_id, self._phase_cs_pin_id = (
        self.mcu_tmc.tmc_spi.get_bus_and_cs_ids()
    )
```

- [ ] **Step 3: Add the public accessor**

Inside the `TMC5160` class, after `__init__`, add:

```python
def get_phase_config(self):
    """Return (bus_id, cs_pin_id) for phase-stepping integration.

    Raises if this TMC5160 is not configured for phase stepping. Called
    by motion_toolhead._configure_axes_per_mcu when building the
    33-byte configure_axes blob.
    """
    if not self._phase_stepping:
        raise self.printer.config_error(
            "get_phase_config called on a TMC5160 without "
            "phase_stepping=True on the matching stepper section"
        )
    return (self._phase_bus_id, self._phase_cs_pin_id)
```

Note: `self.printer` is not currently stored on the TMC5160 instance. Stash it in step 2 by adding `self.printer = config.get_printer()` near the top of `__init__` if it isn't there already. (Quick grep to confirm: `grep -n "self.printer\s*=" klippy/extras/tmc5160.py`.)

- [ ] **Step 4: Manual smoke test the import path**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
python3 -c "
import sys
sys.path.insert(0, 'klippy')
sys.path.insert(0, 'klippy/extras')
import tmc5160
print('module imports OK')
print('TMC5160 has get_phase_config:', hasattr(tmc5160.TMC5160, 'get_phase_config'))
print('_enable_direct_mode defined:', callable(getattr(tmc5160, '_enable_direct_mode', None)))
"
```

Expected: `module imports OK`, both `True`.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/tmc5160.py
git commit -m "feat(klippy/tmc5160): direct_mode init for phase-stepped axes

TMC5160.__init__ checks the matching [stepper_*] section for
phase_stepping=True. When set: queues GCONF.direct_mode=1 via the
field collector (so it lands in the connect-time TMC SPI burst),
validates microsteps==256 and stealthchop_threshold absent, stashes
(bus_id, cs_pin_id) on the instance for motion_toolhead retrieval.

New get_phase_config() accessor for the upcoming
_configure_axes_per_mcu integration."
```

---

## Task 5: TMC5160CurrentHelper direct-mode IHOLD=IRUN mapping

**Files:**
- Modify: `klippy/extras/tmc5160.py:270-360` (TMC5160CurrentHelper class)

- [ ] **Step 1: Read the existing CurrentHelper structure**

```bash
sed -n '270,362p' /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/klippy/extras/tmc5160.py
```

Locate (a) the `__init__` signature and (b) where the IRUN/IHOLD register values are computed/written (look for `set_field("ihold", ...)` and `set_field("irun", ...)` in the surrounding code, possibly in `tmc.BaseTMCCurrentHelper`).

- [ ] **Step 2: Add a `direct_mode` flag to the helper**

The CurrentHelper is constructed in `TMC5160.__init__` at line 377:

```python
current_helper = TMC5160CurrentHelper(config, self.mcu_tmc)
```

Change the call site to:

```python
current_helper = TMC5160CurrentHelper(
    config, self.mcu_tmc, direct_mode=self._phase_stepping,
)
```

(Ordering note: `self._phase_stepping` is set in Task 4 above the CurrentHelper construction line. Move the phase-stepping detection block above the `current_helper = ...` line so the flag is available.)

In the `TMC5160CurrentHelper.__init__` signature, add the kwarg:

```python
def __init__(self, config, mcu_tmc, direct_mode=False):
    self._direct_mode = direct_mode
    # ... existing body ...
```

- [ ] **Step 3: Override the IRUN/IHOLD mapping when direct_mode is on**

Inside `TMC5160CurrentHelper`, locate the method(s) that set `irun` and `ihold` fields (likely `_update_current` or `set_current` inherited from `BaseTMCCurrentHelper`). Look for:

```python
self.fields.set_field("irun", irun_val)
self.fields.set_field("ihold", ihold_val)
```

When `self._direct_mode` is True, set both equal to the IRUN value (the active-motion current):

```python
if self._direct_mode:
    # Phase stepping: chip's current scaling is selected by step pulses
    # (IRUN on a step event, decays to IHOLD via IHOLDDELAY). With no
    # step pulses, IHOLD is the effective ceiling. Set both equal so
    # the effective current is identical regardless of which the chip
    # internally selects — protects against unexpected step events.
    self.fields.set_field("irun", irun_val)
    self.fields.set_field("ihold", irun_val)
else:
    self.fields.set_field("irun", irun_val)
    self.fields.set_field("ihold", ihold_val)
```

If the existing CurrentHelper does not have a single point where these fields are set (e.g. inherits from `BaseTMCCurrentHelper` and the base writes them), override the relevant method in `TMC5160CurrentHelper` to do the direct-mode swap.

- [ ] **Step 4: Smoke test**

```bash
python3 -c "
import sys
sys.path.insert(0, 'klippy')
sys.path.insert(0, 'klippy/extras')
import tmc5160
import inspect
sig = inspect.signature(tmc5160.TMC5160CurrentHelper.__init__)
print('TMC5160CurrentHelper signature:', sig)
assert 'direct_mode' in sig.parameters, sig.parameters
print('OK')
"
```

Expected: `direct_mode` shown in signature.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/tmc5160.py
git commit -m "feat(klippy/tmc5160): IHOLD=IRUN mapping in direct mode

When phase_stepping=True on the matching stepper section, TMC5160-
CurrentHelper sets IHOLDIRUN.IHOLD = IHOLDIRUN.IRUN = computed
run-current value. In direct mode the chip's current scaling is
selected by step pulses (IRUN on step, decay to IHOLD via IHOLDDELAY).
With no step pulses ever asserted in phase stepping, IHOLD is the
effective ceiling — setting both equal makes the effective current
identical regardless of any unexpected step events."
```

---

## Task 6: `motion_toolhead._configure_axes_per_mcu` — build `phase_configs[]` + call `register_phase_bus`

**Files:**
- Modify: `klippy/motion_toolhead.py:631-770` (the `_configure_axes_per_mcu` method)

- [ ] **Step 1: Read the existing function to locate the insertion point**

```bash
sed -n '680,770p' /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/klippy/motion_toolhead.py
```

Look for the existing `step_modes[i] = 0  # Modulated` line and the `self.bridge.configure_axes(...)` call. The insertion goes between them.

- [ ] **Step 2: Add `phase_configs` collection after `step_modes` is fully populated**

After the loop that sets `step_modes[i] = 0` for each phase-stepped slot (right before the existing `awd_mask = awd_default & present_mask` line), insert:

```python
# 2026-05-18 phase-stepping bridge integration: build the per-motor
# (bus_id, cs_pin_id) array for the 33-byte configure_axes blob, and
# register each unique SPI bus with the MCU's phase-stepping
# subsystem. PHASE_CFG_NONE encodes "no phase config" in firmware-
# compatible form (0xFF, 0xFF). Slot stays absent if step_modes != 0.
PHASE_CFG_NONE = (0xFF, 0xFF)
phase_configs = [PHASE_CFG_NONE] * 4
any_phase_stepping = False
for i, slot in enumerate(slot_steppers):
    if step_modes[i] != 0 or not slot:
        continue
    primary_name = slot[0][0]
    tmc_name = "tmc5160 " + primary_name
    try:
        tmc = self.printer.lookup_object(tmc_name)
    except Exception:
        raise self.printer.config_error(
            "phase_stepping=True on stepper '%s' requires a [tmc5160 %s] "
            "section (current driver type or absence of TMC5160 section "
            "is incompatible with phase stepping)" % (primary_name, primary_name)
        )
    if not hasattr(tmc, "get_phase_config"):
        raise self.printer.config_error(
            "phase_stepping=True on stepper '%s' requires a TMC5160 driver; "
            "found driver type with no phase-stepping support"
            % primary_name
        )
    phase_configs[i] = tmc.get_phase_config()
    any_phase_stepping = True
```

- [ ] **Step 3: Add the `register_phase_bus` invocations before `configure_axes`**

Just before the existing `self.bridge.configure_axes(...)` call (around line 750), insert:

```python
if any_phase_stepping:
    seen_buses = set()
    for (bus_id, cs_pin_id) in phase_configs:
        if bus_id == 0xFF:
            continue
        if bus_id in seen_buses:
            continue
        seen_buses.add(bus_id)
        self.bridge.register_phase_bus(
            mcu_handle, bus_id, cs_pin_id, rate=2_000_000,
        )
```

- [ ] **Step 4: Thread `phase_configs` through `configure_axes`**

Replace the existing call (the exact text varies by spacing; this is the structure):

```python
self.bridge.configure_axes(
    mcu_handle, kin_tag, present_mask, awd_mask,
    invert_mask, steps_per_mm, step_modes,
)
```

with:

```python
self.bridge.configure_axes(
    mcu_handle, kin_tag, present_mask, awd_mask,
    invert_mask, steps_per_mm, step_modes,
    phase_configs=phase_configs if any_phase_stepping else None,
)
```

- [ ] **Step 5: Update the logging line below to include phase_configs**

A few lines below (around line 787), find the existing `logging.info(...)` that logs `step_modes=%s`. Append `phase_configs=%s any_phase_stepping=%s` so debug logs surface what was sent.

Locate:
```python
logging.info(
    "MotionToolhead: MCU=%s configured for kinematics=%d present=0x%x ..."
    "step_modes=%s mcu_caps=0x%x runtime_bindings=%s",
    ..., step_modes, mcu_caps, ...,
)
```

Add `phase_configs` and `any_phase_stepping` to the args + format string.

- [ ] **Step 6: Smoke-import the module**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
python3 -c "
import sys
sys.path.insert(0, 'klippy')
import motion_toolhead
src = open('klippy/motion_toolhead.py').read()
assert 'phase_configs' in src
assert 'register_phase_bus' in src
assert 'PHASE_CFG_NONE' in src
print('OK')
"
```

Expected: `OK`.

- [ ] **Step 7: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(klippy/motion_toolhead): wire phase_configs + register_phase_bus

_configure_axes_per_mcu now: (1) builds a phase_configs[4] array by
looking up each phase-stepped slot's TMC5160 object via lookup_object
+ get_phase_config; (2) calls bridge.register_phase_bus once per unique
SPI bus before configure_axes; (3) threads phase_configs through
configure_axes for the 33-byte body emission. Hard config_error when
phase_stepping=True but no matching [tmc5160 stepper_*] section."
```

---

## Task 7: `motion_bridge.py` — Python wrappers for the new bridge methods

**Files:**
- Modify: `klippy/motion_bridge.py` (around the existing `configure_axes` wrapper at line 133)

- [ ] **Step 1: Extend the `configure_axes` wrapper**

Find the existing wrapper (line 133):

```python
def configure_axes(
    self,
    mcu_handle,
    kinematics,
    present_mask,
    awd_mask,
    invert_mask,
    steps_per_mm,
    step_modes=None,
    timeout_s=2.0,
):
```

Add the new kwarg:

```python
def configure_axes(
    self,
    mcu_handle,
    kinematics,
    present_mask,
    awd_mask,
    invert_mask,
    steps_per_mm,
    step_modes=None,
    phase_configs=None,
    timeout_s=2.0,
):
    """Send the kalico-native ConfigureAxes message to an attached MCU.

    step_modes: optional list of 4 ints (0=Modulated/phase-stepping,
    1=StepTime/classic). When supplied the bridge emits the 25-byte
    extended format.

    phase_configs: optional list of 4 (bus_id, cs_pin_id) tuples. When
    supplied (alongside step_modes), the bridge emits the 33-byte
    extended format with bytes 25..32 = per-motor SPI bus + CS pin.
    Sentinel value (0xFF, 0xFF) means 'no phase config for this slot'.
    """
    return self._bridge.configure_axes(
        mcu_handle,
        kinematics,
        present_mask,
        awd_mask,
        invert_mask,
        list(steps_per_mm),
        list(step_modes) if step_modes is not None else None,
        list(phase_configs) if phase_configs is not None else None,
        timeout_s,
    )
```

(Reorder positional args to match the Rust PyO3 signature from Task 1 / Task 2; the Rust side has `phase_configs` between `step_modes` and `timeout_s`.)

- [ ] **Step 2: Add the `register_phase_bus` wrapper**

Below the `configure_axes` wrapper, add:

```python
def register_phase_bus(self, mcu_handle, bus_id, cs_pin_id, rate, timeout_s=5.0):
    """Register an SPI bus with the MCU's phase-stepping subsystem.

    Wraps the runtime_register_phase_bus wire command. Must be called
    once per unique (bus_id) on each phase-stepping-capable MCU BEFORE
    configure_axes for that MCU. Stock-Klipper MCUs (no kalico runtime)
    are no-op via bridge.rs's early-return.
    """
    return self._bridge.register_phase_bus(
        mcu_handle, bus_id, cs_pin_id, rate, timeout_s,
    )
```

- [ ] **Step 3: Smoke-test the import path**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
python3 -c "
import sys
sys.path.insert(0, 'klippy')
src = open('klippy/motion_bridge.py').read()
assert 'phase_configs' in src
assert 'register_phase_bus' in src
print('OK')
"
```

Expected: `OK`.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_bridge.py
git commit -m "feat(klippy/motion_bridge): Python wrappers for phase-stepping methods

configure_axes gains phase_configs kwarg threaded through to the Rust
33-byte body builder. New register_phase_bus wrapper for the
runtime_register_phase_bus wire command. Both delegate to PyMotionBridge."
```

---

## Task 8: Thread `phase_configs` through `bridge.rs::configure_axes`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (`configure_axes` PyO3 method around line 952)

- [ ] **Step 1: Extend the PyO3 signature and pass `phase_configs` to the body builder**

Find the existing signature attribute:

```rust
#[pyo3(signature = (mcu_handle, kinematics, present_mask, awd_mask, invert_mask, steps_per_mm, step_modes = None, timeout_s = 2.0))]
fn configure_axes(
    &self,
    py: Python<'_>,
    mcu_handle: u32,
    kinematics: u8,
    present_mask: u8,
    awd_mask: u8,
    invert_mask: u8,
    steps_per_mm: Vec<f32>,
    step_modes: Option<Vec<u8>>,
    timeout_s: f64,
) -> PyResult<()> {
```

Change to:

```rust
#[pyo3(signature = (mcu_handle, kinematics, present_mask, awd_mask, invert_mask, steps_per_mm, step_modes = None, phase_configs = None, timeout_s = 2.0))]
fn configure_axes(
    &self,
    py: Python<'_>,
    mcu_handle: u32,
    kinematics: u8,
    present_mask: u8,
    awd_mask: u8,
    invert_mask: u8,
    steps_per_mm: Vec<f32>,
    step_modes: Option<Vec<u8>>,
    phase_configs: Option<Vec<(u8, u8)>>,
    timeout_s: f64,
) -> PyResult<()> {
```

- [ ] **Step 2: Validate `phase_configs` length and convert to a fixed array**

Just after the existing `if let Some(ref sm) = step_modes { if sm.len() != 4 { ... } }` block, add:

```rust
if let Some(ref pc) = phase_configs {
    if pc.len() != 4 {
        return Err(PyRuntimeError::new_err(
            "configure_axes: phase_configs must be a list of 4 (bus_id, cs_pin_id) tuples",
        ));
    }
    if step_modes.is_none() {
        return Err(PyRuntimeError::new_err(
            "configure_axes: phase_configs requires step_modes (33-byte format extends 25-byte)",
        ));
    }
}
```

- [ ] **Step 3: Pass `phase_configs` into the body builder**

Update the call from Task 1, Step 2 to thread phase_configs:

```rust
let phase_capable: u8 = if identify_caps & 0x1 != 0 { 1 } else { 0 };
let steps_arr: [f32; 4] = [steps_per_mm[0], steps_per_mm[1], steps_per_mm[2], steps_per_mm[3]];
let step_modes_arr: Option<[u8; 4]> = step_modes.as_ref().map(|sm| [sm[0], sm[1], sm[2], sm[3]]);
let phase_configs_arr: Option<[(u8, u8); 4]> = phase_configs.as_ref().map(|pc| [pc[0], pc[1], pc[2], pc[3]]);
let body = build_configure_axes_body(
    kinematics,
    present_mask,
    awd_mask,
    invert_mask,
    &steps_arr,
    step_modes_arr.as_ref(),
    phase_configs_arr.as_ref(),
    phase_capable,
);
```

- [ ] **Step 4: Build and run the existing unit tests**

```bash
cargo test -p motion-bridge --lib build_configure_axes_body
cargo build -p motion-bridge
```

Expected: all build_configure_axes_body tests still pass; build clean.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): thread phase_configs through configure_axes

PyO3 signature gains phase_configs kwarg; validates length=4 and
requires step_modes when present. Body construction goes via the
existing build_configure_axes_body helper (Task 1) which produces
the 33-byte layout when both step_modes and phase_configs are Some."
```

---

## Task 9: `phase_stepping_spi.c` — busy-flag + skip counter

**Files:**
- Modify: `src/stm32/phase_stepping_spi.h`
- Modify: `src/stm32/phase_stepping_spi.c`

- [ ] **Step 1: Declare the new API in the header**

In `src/stm32/phase_stepping_spi.h`, after the existing `phase_stepping_write_xdirect` declaration, add:

```c
// ---------- 2026-05-18 SPI3 contention arbitration ----------------------
// Cooperative busy-flag mediating SPI3 access between two writers:
//   - TIM5-rate (40 kHz) phase_stepping_write_xdirect from the ISR
//   - Lower-priority TMC SPI register access from Klipper task code
//     (e.g. _do_periodic_check's 1 Hz DRV_STATUS polling)
//
// Both paths MUST acquire before initiating an SPI transfer and release
// after. The flag uses irq_save / irq_restore for mutual exclusion;
// CMSIS atomic primitives are not required because all writers are
// single-instruction reads/writes against a uint8_t.
//
// Return value of phase_spi_try_acquire(): 1 if acquired, 0 if busy.
// The TIM5 ISR's phase_stepping_write_xdirect skips its cycle on 0 and
// increments phase_spi_skip_count for telemetry. Klipper's spi_transfer
// for the registered phase bus instead spins (or yields, per
// stm32h7_spi.c convention) until acquire succeeds — that path tolerates
// the wait latency.
uint8_t phase_spi_try_acquire(void);
void    phase_spi_release(void);
uint32_t phase_spi_get_skip_count(void);
```

- [ ] **Step 2: Define the new primitives in the .c file**

At the top of `src/stm32/phase_stepping_spi.c`, after the existing `#include` block, add:

```c
#include "sched.h"   // irq_save, irq_restore, irqstatus_t

static volatile uint8_t  phase_spi_busy = 0;
static volatile uint32_t phase_spi_skip_count = 0;

__attribute__((used, externally_visible))
uint8_t
phase_spi_try_acquire(void)
{
    irqstatus_t flag = irq_save();
    uint8_t was_busy = phase_spi_busy;
    if (!was_busy)
        phase_spi_busy = 1;
    irq_restore(flag);
    return !was_busy;
}

__attribute__((used, externally_visible))
void
phase_spi_release(void)
{
    // Single-byte volatile write is atomic on M4/M7. No critical
    // section needed because we are the sole writer of the cleared
    // state; preemption between read and write of the held state
    // cannot violate invariants.
    phase_spi_busy = 0;
}

__attribute__((used, externally_visible))
uint32_t
phase_spi_get_skip_count(void)
{
    return phase_spi_skip_count;
}
```

- [ ] **Step 3: Gate `phase_stepping_write_xdirect` on the flag**

Modify the existing `phase_stepping_write_xdirect` to early-exit on skip:

```c
__attribute__((used, externally_visible))
void
phase_stepping_write_xdirect(uint8_t bus_id, uint8_t cs_pin,
                             int16_t coil_a, int16_t coil_b)
{
    (void)cs_pin;

    if (bus_id >= MAX_PHASE_BUSES || !phase_buses[bus_id].configured)
        return;

    // ISR-priority: if Klipper's spi_transfer holds the bus, skip this
    // modulation cycle. One skip = 25 us at 40 kHz, inaudible. The
    // skip-count telemetry is the canary for SPI3 contention going wild.
    if (!phase_spi_try_acquire()) {
        phase_spi_skip_count++;
        return;
    }

    uint16_t ua = (uint16_t)coil_a;
    uint16_t ub = (uint16_t)coil_b;

    uint8_t datagram[5] = {
        0xAD,
        (uint8_t)((ub >> 8) & 0x01),
        (uint8_t)(ub & 0xFF),
        (uint8_t)((ua >> 8) & 0x01),
        (uint8_t)(ua & 0xFF),
    };

    spi_prepare(phase_buses[bus_id].cfg);
    gpio_out_write(phase_buses[bus_id].cs, 0);
    spi_transfer(phase_buses[bus_id].cfg, 0, sizeof(datagram), datagram);
    gpio_out_write(phase_buses[bus_id].cs, 1);

    phase_spi_release();
}
```

- [ ] **Step 4: Verify the C file compiles in the existing build**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
# H7 sim build is the fastest local check:
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -20
```

Expected: clean build, `out/klipper.elf` produced.

- [ ] **Step 5: Commit**

```bash
git add src/stm32/phase_stepping_spi.c src/stm32/phase_stepping_spi.h
git commit -m "feat(phase_stepping_spi): cooperative SPI3 busy-flag + skip counter

phase_spi_try_acquire / phase_spi_release primitives mediate SPI3
between the TIM5-rate XDIRECT ISR and Klipper's task-context TMC SPI
writes (notably _do_periodic_check's 1 Hz DRV_STATUS poll).

phase_stepping_write_xdirect skips + bumps phase_spi_skip_count when
Klipper holds the bus. Klipper's spi_transfer hook in stm32h7_spi.c
(next task) acquires before its transfer.

The skip telemetry is the canary: bench tests assert sustained
skip_count growth stays below 100/s during modulation-active idle."
```

---

## Task 10: `stm32h7_spi.c` — acquire/release hook around `spi_transfer`

**Files:**
- Modify: `src/stm32/stm32h7_spi.c:128` (the `spi_transfer` implementation)

- [ ] **Step 1: Inspect the current spi_transfer signature and scope**

```bash
sed -n '128,200p' /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/src/stm32/stm32h7_spi.c
```

Confirm it's a leaf function (no nested calls to another `spi_transfer` on a different bus) and identify the existing entry/exit pattern.

- [ ] **Step 2: Add the acquire/release wrap**

At the very top of `spi_transfer`, before any other logic, add:

```c
#include "stm32/phase_stepping_spi.h"  // phase_spi_try_acquire / release
// (Or wherever the path resolves; the existing #include block at top
//  of file should already pick up most stm32/ headers.)

// 2026-05-18 phase-stepping SPI3 contention: Klipper's task-context
// SPI access must coordinate with the TIM5-rate XDIRECT ISR. The
// busy-flag is per-MCU global (one SPI3 instance per H723), so we
// gate every spi_transfer call. Non-SPI3 transfers see the flag
// uncontested and acquire/release with negligible overhead (~10
// cycles per pair). The wait path is bounded: the TIM5 ISR releases
// within ~25 us of acquiring.
while (!phase_spi_try_acquire()) {
    // Spin; the ISR-side write completes within one TIM5 period
    // (25 us at 40 kHz). On real hardware the CPU is not idle here
    // — the next TIM5 ISR fire will release. In Renode sim, the
    // virtual time advances under the spin loop.
}
```

And at every return path / function exit (typically the bottom of the function and any early-return after error handling), add:

```c
phase_spi_release();
```

If the function has multiple return points, the cleanest pattern is a single trailing `phase_spi_release()` and converting early returns to `goto cleanup` jumps; or wrap with a small helper. For minimum-diff, count the return points and add a release before each.

Note: if the H7 spi_transfer is only used for SPI3 phase-stepping AND for other peripherals (e.g. an accelerometer on SPI1), the acquire/release is "unnecessary but harmless" overhead on the non-phase buses. The busy flag is single-MCU global, not per-bus, so this is correct for v1. A bus-specific busy-flag is a future optimization if SPI1/SPI2 paths become hot.

- [ ] **Step 3: Build and verify**

```bash
bash tools/sim/build_sim_firmware.sh 2>&1 | tail -20
```

Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/stm32/stm32h7_spi.c
git commit -m "feat(stm32h7_spi): acquire phase_spi_busy around spi_transfer

Klipper's task-context SPI3 transfers (notably TMC5160 SPI from
_do_periodic_check's 1 Hz DRV_STATUS poll) must coordinate with the
TIM5-rate XDIRECT ISR for the same peripheral. spi_transfer now
spin-acquires phase_spi_busy before initiating its transfer; ISR's
phase_stepping_write_xdirect skips with skip_count++ on contention.

Acquire-wait is bounded: ISR releases within ~25 us (one TIM5 period
at 40 kHz). Overhead on non-SPI3 transfers is ~10 cycles per pair
(uncontested CAS-equivalent through irq_save/restore)."
```

---

## Task 11: Surface `phase_spi_skip_count` via the kalico_status response

**Files:**
- Modify: `src/runtime_commands.c:30-40` (`command_runtime_query_status` handler emits `kalico_status`)
- Possibly modify: `rust/motion-bridge/src/host_io.rs` or wherever `kalico_status` is parsed host-side

- [ ] **Step 1: Extend the C-side `kalico_status` response**

Current `command_runtime_query_status` (line 30) emits:
```c
sendf("kalico_status status=%c last_err=%i", status, last_err);
```

Change to:
```c
#include "stm32/phase_stepping_spi.h"  // phase_spi_get_skip_count (CONFIG_MACH_STM32 only)

void
command_runtime_query_status(uint32_t *args)
{
    if (!runtime_handle) {
        sendf("kalico_status status=%c last_err=%i phase_spi_skip_count=%u",
              (uint8_t)255, -7, 0u);
        return;
    }
    uint8_t status = runtime_handle_status(runtime_handle);
    int32_t last_err = runtime_handle_last_error(runtime_handle);
    uint32_t phase_skip = 0;
#if CONFIG_MACH_STM32
    phase_skip = phase_spi_get_skip_count();
#endif
    sendf("kalico_status status=%c last_err=%i phase_spi_skip_count=%u",
          status, last_err, phase_skip);
}
```

The msgproto format-string change is backward-incompatible for hosts that strictly validate the message format. The mainline klippy / kalico host code parses the response field-by-field (via `mcu.lookup_command()` + named parameter access), so adding a field is non-breaking. Verify with:
```bash
grep -rn "kalico_status" rust/motion-bridge/src/ klippy/ 2>/dev/null | head -10
```

- [ ] **Step 2: Add host-side parsing**

Find the existing `kalico_status` consumer site (from the grep above). If the host parses via `params["status"]` / `params["last_err"]` dict access, just add `params["phase_spi_skip_count"]` to the call sites that need it. If there's a typed struct that fully validates the schema, add a `phase_spi_skip_count: u32` field.

Common access pattern in motion-bridge will look like:
```rust
let phase_spi_skip_count: u32 = params.get("phase_spi_skip_count")
    .and_then(|v| v.as_u64()).unwrap_or(0) as u32;
```

The `.unwrap_or(0)` keeps the parser tolerant of older MCU firmware that does not yet emit the field — useful for staged rollouts.

- [ ] **Step 4: Build, sim-run, smoke-check the field**

```bash
bash tools/sim/build_sim_firmware.sh
bash tools/sim/run_sim.sh &
sleep 8
python3 -c "
from tools.test_sim_phase_stepping import register_phase_bus
# Or whichever test-helper exists; the goal is just to issue
# runtime_query_status and confirm the new field appears.
"
pkill -f renode || true
```

Expected: status frame includes `phase_spi_skip_count=0` (no contention in fresh sim boot).

- [ ] **Step 5: Commit**

```bash
git add src/runtime_commands.c rust/motion-bridge/src/*.rs rust/runtime/src/*.rs
git commit -m "feat(runtime/status): expose phase_spi_skip_count

Adds the SPI3 contention skip counter to runtime_query_status_response.
Host can monitor mid-print to assert SPI3 contention is staying in the
expected envelope (<100/s sustained). Surface for the bench acceptance
gate in the phase-stepping bridge spec."
```

---

## Task 12: Sim integration test — `tools/test_sim_phase_stepping.py` extension

**Files:**
- Modify: `tools/test_sim_phase_stepping.py`
- Possibly modify: `tools/sim/renode_peripherals/Tmc5160.cs` (verify it surfaces written-register history; extend if needed)

- [ ] **Step 1: Inspect the existing test entry points**

```bash
grep -n "^def \|^class " /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/tools/test_sim_phase_stepping.py | head -20
sed -n '600,700p' /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping/tools/test_sim_phase_stepping.py
```

Identify the existing main test flow.

- [ ] **Step 2: Add a regression case `test_phase_stepping_wire_format`**

Add a new test function:

```python
def test_phase_stepping_wire_format(io, capture):
    """Assert configure_axes emits the 33-byte body when phase_configs
    is supplied, and that runtime_register_phase_bus is sent before
    the configure_axes blob.
    """
    # Pre-condition: register the phase bus.
    rc = register_phase_bus(io, bus_id=3, cs_pin=5, rate=2_000_000)
    assert rc == 0, f"register_phase_bus failed: {rc}"

    # Capture must include the runtime_register_phase_bus frame.
    rpb_frames = [
        f for f in capture.frames()
        if "runtime_register_phase_bus" in f.text
    ]
    assert len(rpb_frames) >= 1, "no runtime_register_phase_bus emitted"

    # Issue configure_axes with phase_configs.
    body_len = send_configure_axes_blob(
        io,
        kinematics=0,
        present_mask=0x03,   # X+Y only for this test
        awd_mask=0x00,
        invert_mask=0x00,
        steps_per_mm=[160.0, 160.0, 0.0, 0.0],
        step_modes=[0, 0, 1, 1],
        phase_configs=[(3, 5), (3, 6), (0xFF, 0xFF), (0xFF, 0xFF)],
    )
    assert body_len == 33, f"expected 33-byte body, got {body_len}"

    # Assert the order: register_phase_bus comes before configure_axes.
    cax_frames = [
        f for f in capture.frames()
        if "kalico_configure_axes_blob" in f.text
    ]
    assert cax_frames[0].timestamp > rpb_frames[0].timestamp, (
        "register_phase_bus must precede configure_axes"
    )
```

If `send_configure_axes_blob` does not exist in the test harness today (check via grep), add a tiny helper that emits the 33-byte body via the existing `kalico_send` wire path and returns the body length captured on the wire.

- [ ] **Step 3: Add a Tmc5160 sim-stub capture check**

Renode's Tmc5160 stub at `tools/sim/renode_peripherals/Tmc5160.cs` should expose a register-write history. Inspect it and add a Robot-Framework / Renode monitor call to query the history. Example assertion structure:

```python
def test_phase_stepping_gconf_xdirect(io, renode):
    """After phase_stepping_register_bus + a segment push, the sim's
    TMC5160 stub should have recorded:
      1. At least one GCONF write with bit 16 (direct_mode) set
      2. At least one XDIRECT (register 0x2D) write
    """
    # ... bring-up: configure_axes(33-byte), push a small segment ...
    history = renode.monitor("tmc_x GetWriteHistory")
    gconf_writes = [w for w in history if w.register == 0x00]
    direct_mode_writes = [w for w in gconf_writes if w.value & (1 << 16)]
    assert direct_mode_writes, "no GCONF.direct_mode=1 write captured"

    xdirect_writes = [w for w in history if w.register == 0x2D]
    assert xdirect_writes, "no XDIRECT writes captured"
```

If the Tmc5160 stub does not currently expose `GetWriteHistory`, extend it — the necessary instrumentation is a simple Dictionary<uint, List<uint>> mapping reg → value-history. See `tools/sim/renode_peripherals/Tmc5160.cs` for the existing pattern.

- [ ] **Step 4: Add a `phase_spi_skip_count` assertion**

```python
def test_phase_spi_skip_count_clean_in_sim(io):
    """Sim has no concurrent klippy TMC SPI traffic; skip_count must
    remain at 0 throughout the test run.
    """
    status_before = query_runtime_status(io)
    push_test_segment(io)
    time.sleep(0.5)
    status_after = query_runtime_status(io)
    assert status_after.phase_spi_skip_count == 0, (
        f"unexpected skip_count={status_after.phase_spi_skip_count} "
        f"in sim (sim has no contending TMC SPI traffic)"
    )
```

- [ ] **Step 5: Run the extended sim test**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
bash tools/sim/build_sim_firmware.sh
bash tools/sim/run_sim.sh &
sleep 8
python3 tools/test_sim_phase_stepping.py --test wire_format
python3 tools/test_sim_phase_stepping.py --test gconf_xdirect
python3 tools/test_sim_phase_stepping.py --test skip_count
pkill -f renode || true
```

Expected: all three PASS.

- [ ] **Step 6: Commit**

```bash
git add tools/test_sim_phase_stepping.py tools/sim/renode_peripherals/Tmc5160.cs
git commit -m "test(sim/phase-stepping): assert 33-byte wire + GCONF + XDIRECT

Three new regression cases:
  - test_phase_stepping_wire_format: configure_axes emits 33 bytes,
    runtime_register_phase_bus precedes it.
  - test_phase_stepping_gconf_xdirect: TMC5160 sim stub records both
    a GCONF.direct_mode=1 write and a steady stream of XDIRECT writes
    during segment push.
  - test_phase_spi_skip_count_clean_in_sim: contention canary stays
    at 0 in sim (no concurrent klippy TMC SPI traffic in this harness).

Tmc5160.cs gains a GetWriteHistory accessor surfacing the register-
write log for the test driver to inspect."
```

---

## Task 13: Bench verification on Trident (user-executed)

**Files:** None (verification only).

This task is executed by the user with the H723 + F446 connected. The agent prepares the build per `feedback_bench_firmware_flow.md` (commit → push → pull on Pi → compile → flash) but **does not run motion commands without explicit user permission** per `feedback_no_gcode_without_permission.md`.

- [ ] **Step 1: Push the branch and prepare the bench-side build**

```bash
cd /Users/daniladergachev/Developer/kalico/.claude/worktrees/phase-stepping
git push origin worktree-phase-stepping
```

Then on `trident.local`:

```bash
ssh dderg@trident.local
cd /home/dderg/kalico
git fetch && git checkout worktree-phase-stepping && git pull
# Reset to H7 config:
cp .config.h7.last .config
make clean
cd rust && cargo clean && cd ..
make -j4
# Flash via the saved DFU path (see feedback_flash_h7.md).
```

(Run as a single `ssh` command sequence; expect ~3 min compile.)

- [ ] **Step 2: After flash, request user permission to run motion checks**

Send the user a message like:

> Bench firmware flashed (commit `<sha>`). I'd like to run:
>
> 1. `DUMP_TMC STEPPER=stepper_x` and `DUMP_TMC STEPPER=stepper_y` (read-only, no motion)
> 2. `DUMP_TMC STEPPER=stepper_z` as negative control
> 3. After homing (user-triggered `G28`), a slow G1 jog on X to engage modulation
> 4. Query `phase_spi_skip_count` via the runtime status frame after 60 s of slow modulation
>
> Steps 1–2 and 4 are non-motion (no risk). Step 3 requires you to authorize the homing + jog explicitly. Should I proceed with the read-only checks first?

- [ ] **Step 3: Run the read-only DUMP_TMC checks**

(After explicit user OK for the read-only calls.) Via moonraker or the printer console:

```
DUMP_TMC STEPPER=stepper_x
DUMP_TMC STEPPER=stepper_y
DUMP_TMC STEPPER=stepper_z
```

Verify:
- `stepper_x` GCONF shows `direct_mode=1`.
- `stepper_y` GCONF shows `direct_mode=1`.
- `stepper_z` GCONF shows `direct_mode=0` (negative control).
- `IHOLDIRUN` on X+Y: IHOLD field non-zero and equal to IRUN.

- [ ] **Step 4: Wait for explicit motion authorization, then run the modulation-active idle test**

(After user OKs motion.) Issue G28 to home, then a 60-second slow jog:

```
G28
G91
G1 X0.001 F1   ; very slow, sustained
```

Read status frame for `phase_spi_skip_count` at start and end of the 60 s. Assert delta ≤ 120 (≤2 skips per 1 Hz Klipper poll × 60 s).

- [ ] **Step 5: Negative test — config_error refusal**

In a separate test config branch (or temporary edit to printer.cfg), set `phase_stepping: True` on `stepper_z` (which uses TMC2209, not TMC5160). Restart klippy. Assert that klippy refuses to start with the expected `config_error` message:

> "phase_stepping=True on stepper 'stepper_z' requires a [tmc5160 stepper_z] section..."

Restore printer.cfg afterwards.

- [ ] **Step 6: Capture and report**

Summarize for the user:
- `direct_mode=1` confirmed: YES / NO (and any anomalies)
- Skip-count delta over 60 s: <value>
- Negative test refused as expected: YES / NO
- Audible/subjective motion-quality comparison vs. control branch (`sota-motion` with `phase_stepping: False`): note any differences.

No commit for this task — it's verification only. The plan ships when the bench results match the §9 acceptance gate in the spec.

---

## Self-review checklist (run after Task 12, before bench)

- [ ] Spec §1 problem (1)/(2)/(3) all addressed: 33-byte body (Task 1, 8); `runtime_register_phase_bus` invocation (Tasks 2, 6, 7); `GCONF.direct_mode=1` (Task 4).
- [ ] Spec §4.2 contention design implemented: ISR-priority busy-flag (Task 9); bidirectional Klipper-side acquire (Task 10); skip-count telemetry (Task 11); bench validation (Task 13).
- [ ] Spec §6 error cases — five config_errors — implemented:
  - No `[tmc5160 <stepper>]` block: Task 6 Step 2.
  - Non-5160 driver: Task 6 Step 2 (hasattr check).
  - `stealthchop_threshold` > 0: Task 4 Step 1 (_enable_direct_mode).
  - `microsteps` != 256: Task 4 Step 1 (_enable_direct_mode).
  - MCU lacks `PHASE_STEPPING_CAPABLE`: pre-existing, no change.
- [ ] Spec §7 testing — all three layers covered: Rust unit (Task 1), Renode sim (Task 12), bench (Task 13).
- [ ] Spec §9 acceptance gate items 1-5 — all enumerated in Task 13 Step 6.

If any line above is unchecked when reading back the plan, add the missing task or step inline before handing off to execution.

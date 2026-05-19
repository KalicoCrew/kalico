// Rust → C panic latch.
//
// The Rust kalico-c-api crate's #[panic_handler] used to spin forever
// (rust/kalico-c-api/src/lib.rs:34-40 before the 2026-05-19 A5 audit fix).
// On the MCU that would lock inside whatever context the panic occurred —
// including the TIM5 ISR and stepper-timer callbacks — preventing both
// the IWDG bite and the shutdown-report frame.
//
// Routing the panic through Klipper's shutdown() macro instead:
//   - emits a "Rust panic" shutdown frame the host receives;
//   - lets the C side service IWDG and emit fault telemetry one last time
//     before the watchdog fires;
//   - makes the failure visible in the user's klippy log instead of as
//     a frozen MCU.
//
// rust_panic_latch is __noreturn so the Rust panic handler's "-> !"
// return type contract is upheld without needing a loop after the call.
//
// Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md A5.

#include "autoconf.h"
#include "command.h"   // shutdown() macro → sched_shutdown(_DECL_STATIC_STR(...))
#include "sched.h"     // sched_shutdown prototype (referenced by the macro)
#include "compiler.h"  // __noreturn

#if CONFIG_KALICO_RUNTIME

__attribute__((used, externally_visible))
void __noreturn
rust_panic_latch(void)
{
    shutdown("Rust panic");
}

#endif // CONFIG_KALICO_RUNTIME

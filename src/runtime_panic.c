// Rust → C panic latch. Routes a Rust panic through Klipper's shutdown() so it
// emits a shutdown frame and lets C service IWDG, instead of spinning inside
// the panic context (e.g. the TIM5 ISR). __noreturn satisfies the Rust panic
// handler's "-> !" contract without a trailing loop.

#include "autoconf.h"
#include "command.h"   // shutdown() macro → sched_shutdown(_DECL_STATIC_STR(...))
#include "sched.h"     // sched_shutdown prototype (referenced by the macro)
#include "compiler.h"  // __noreturn


__attribute__((used, externally_visible))
void __noreturn
rust_panic_latch(void)
{
    shutdown("Rust panic");
}


#include "autoconf.h"
#include "command.h"
#include "sched.h"
#include "compiler.h"

__attribute__((used, externally_visible))
void __noreturn
rust_panic_latch(void)
{
    shutdown("Rust panic");
}


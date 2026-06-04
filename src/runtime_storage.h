// Backing storage for the Kalico runtime engine (RuntimeContext).
// C owns linker-section placement on the MCU (mcu-c-rust-boundary.md rule B2),
// so this replaces the Rust-side #[link_section] static.
//
// RT_STORAGE_SIZE flows from Kconfig (LARGE on H7, SMALL on F4); rust/runtime/
// build.rs reads the same env var, so both sides agree at compile time. The
// Rust-side const_assert(size_of::<RuntimeContext>() <= RT_STORAGE_SIZE)
// backstops the lower bound; runtime_storage.c backstops AXI overflow.

#ifndef KALICO_RUNTIME_STORAGE_H
#define KALICO_RUNTIME_STORAGE_H

#include "autoconf.h"
#include <stdint.h>

#if CONFIG_RUNTIME_TARGET_LARGE
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_LARGE
#elif CONFIG_RUNTIME_TARGET_SMALL
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_SMALL
#else
#  error "No CONFIG_RUNTIME_TARGET_* profile selected — pick LARGE or SMALL"
#endif

extern uint8_t rt_storage[RT_STORAGE_SIZE];

#endif // KALICO_RUNTIME_STORAGE_H

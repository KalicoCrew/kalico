// Backing storage for the Kalico runtime engine (RuntimeContext).
// Replaces the Rust-side RT_CELL static with #[link_section] —
// per docs/kalico-rewrite/mcu-c-rust-boundary.md rule B2, C owns
// linker-section placement on the MCU.
//
// Storage size flows from Kconfig (RUNTIME_STORAGE_SIZE_LARGE on H7,
// _SMALL on F4) so both this header and Rust's RT_STORAGE_SIZE (emitted
// by rust/runtime/build.rs from the same env var) agree at compile time.
//
// Rust-side const_assert backstops the lower bound:
//   const _: () = assert!(size_of::<RuntimeContext>() <= RT_STORAGE_SIZE);
// C-side _Static_assert in runtime_storage.c backstops AXI overflow.
//
// Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md.

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

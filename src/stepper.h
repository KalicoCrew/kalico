#ifndef __STEPPER_H
#define __STEPPER_H

// Legacy klipper-protocol stepper scheduling has been removed; the Rust
// runtime emits step pulses directly via `runtime_emit_step_pulses` from
// the TIM5 ISR. This header is retained for #include compatibility from
// `src/sched.c`; the previous `stepper_event` declaration is gone.

#endif // stepper.h

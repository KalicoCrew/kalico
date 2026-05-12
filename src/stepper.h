#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t
#include "board/gpio.h" // struct gpio_out

uint_fast8_t stepper_event(struct timer *t);

// Return the step pin for the primary stepper bound to runtime motor index
// `motor_idx`. Sets `*out_resolved` to 1 on success, 0 if unavailable.
// Only available when CONFIG_KALICO_RUNTIME is enabled.
struct gpio_out stepper_get_runtime_step_pin(uint8_t motor_idx,
                                             uint8_t *out_resolved);

#endif // stepper.h

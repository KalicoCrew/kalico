#ifndef __KALICO_RUNTIME_TICK_TIMER_H
#define __KALICO_RUNTIME_TICK_TIMER_H
// Motion-engine ISR timer alias.
//
// The motion engine evaluates the trajectory and emits steps from a hardware
// timer ISR. WHICH physical timer that is depends on the MCU: it must be a
// timer not already claimed by Klipper's scheduler or the board's PWM/peripherals.
// Rather than hardcode it in each per-family runtime_tick_*.c, this header maps
// a per-architecture choice to a small set of macros the ISR body uses:
//
//   MOTION_TIM             - the TIM_TypeDef* instance
//   MOTION_TIM_IRQn        - its NVIC IRQn_Type
//   MOTION_TIM_IRQHandler  - the vector handler symbol name (note: several G0
//                            timers SHARE a vector, so the handler name is not
//                            simply TIMx_IRQHandler)
//   MOTION_TIM_RCC_ENABLE()- enable the timer's peripheral clock (the RCC
//                            register differs per family)
//
#include "autoconf.h"
#include "internal.h" // CMSIS device header: TIMx, RCC, *_IRQn

#if CONFIG_MACH_STM32G0
// STM32G0B1 has no TIM5. TIM2 is Klipper's 32-bit scheduler clock
// (stm32f0_timer.c). TIM6/TIM7 are basic timers (no output channels), so
// hard_pwm can never claim them — the safe choices. Selected via the
// KALICO_MOTION_TIMER_* Kconfig choice (default TIM7). Both share their NVIC
// vector with low-power timers a toolhead board does not use.
//
// All G0 timers run at CONFIG_CLOCK_FREQ (64 MHz; STM32G0 applies no APB
// timer-clock doubling), which is exactly the value runtime_clock_freq holds —
// so the standard ARR = clock_freq / sample_rate - 1 formula is correct, and
// TIM6/TIM7 being 16-bit is fine for sample rates >= ~977 Hz (PSC stays 0).
  #if CONFIG_KALICO_MOTION_TIMER_TIM6
    #define MOTION_TIM               TIM6
    #define MOTION_TIM_IRQn          TIM6_DAC_LPTIM1_IRQn
    #define MOTION_TIM_IRQHandler    TIM6_DAC_LPTIM1_IRQHandler
    #define MOTION_TIM_RCC_ENABLE()  do { RCC->APBENR1 |= RCC_APBENR1_TIM6EN; } while (0)
  #else
    #define MOTION_TIM               TIM7
    #define MOTION_TIM_IRQn          TIM7_LPTIM2_IRQn
    #define MOTION_TIM_IRQHandler    TIM7_LPTIM2_IRQHandler
    #define MOTION_TIM_RCC_ENABLE()  do { RCC->APBENR1 |= RCC_APBENR1_TIM7EN; } while (0)
  #endif

#elif CONFIG_MACH_STM32H7 || CONFIG_MACH_STM32F4
  #define MOTION_TIM                 TIM5
  #define MOTION_TIM_IRQn            TIM5_IRQn
  #define MOTION_TIM_IRQHandler      TIM5_IRQHandler
#endif

#endif

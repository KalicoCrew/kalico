#ifndef __GD32_INTERNAL_H
#define __GD32_INTERNAL_H
// Local definitions for GD32 code (GD32F30x and GD32E23x families)

#include "autoconf.h"

#if CONFIG_MACH_GD32F30X
  // GD32F30x: use STM32F103 CMSIS (identical peripheral register layout)
  #if CONFIG_MACH_GD32F303XB || CONFIG_MACH_GD32F303XC
    #define STM32F103xB
  #else
    #define STM32F103xE
  #endif
  #include "stm32f1xx.h"
  #ifndef UID_BASE
    #define UID_BASE 0x1FFFF7E8UL
  #endif
#elif CONFIG_MACH_GD32E23X
  // GD32E23x: use STM32F051 CMSIS (GPIO/SPI/USART/RCC/IWDG compatible layouts)
  // Note: I2C_TypeDef in stm32f0xx.h is new-style (TIMINGR), NOT compatible with
  //       GD32E23x old-style I2C. Use raw struct in i2c.c.
  // Note: GD32E23x USART0 is at 0x40013800, stm32f0xx.h USART1 is at 0x40013400.
  //       Use explicit addresses in serial.c.
  #include "stm32f0xx.h"
#endif

// gpio.c
extern GPIO_TypeDef * const digital_regs[];
#define GPIO(PORT, NUM) (((PORT)-'A') * 16 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 16)
#define GPIO2BIT(PIN) (1<<((PIN) % 16))

// gpio mode macros
#define GPIO_INPUT 0
#define GPIO_OUTPUT 1
#define GPIO_OPEN_DRAIN 0x100
#define GPIO_HIGH_SPEED 0x200
#define GPIO_FUNCTION(fn) (2 | ((fn) << 4))
#define GPIO_ANALOG 3
void gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup);

// clockline.c
void enable_pclock(uint32_t periph_base);
int is_enabled_pclock(uint32_t periph_base);

// MCU-specific (gd32f30x.c or gd32e23x.c)
struct cline { volatile uint32_t *en, *rst; uint32_t bit; };
struct cline lookup_clock_line(uint32_t periph_base);
uint32_t get_pclock_frequency(uint32_t periph_base);
void gpio_clock_enable(GPIO_TypeDef *regs);

#endif // internal.h

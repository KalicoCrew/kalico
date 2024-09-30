// Code to setup clocks on stm32h5
//
// Copyright (C) 2024 Philipp Molitor <phil.x64 at gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_CLOCK_REF_FREQ
#include "board/armcm_boot.h" // VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/irq.h" // irq_disable
#include "board/gpio.h" // irq_disable
#include "board/misc.h" // bootloader_request
#include "command.h" // DECL_CONSTANT_STR
#include "internal.h" // get_pclock_frequency
#include "sched.h" // sched_main


/****************************************************************
 * Clock setup
 ****************************************************************/

#define FREQ_PERIPH_DIV 1
#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / FREQ_PERIPH_DIV)

// Map a peripheral address to its enable bits
struct cline
lookup_clock_line(uint32_t periph_base)
{
    if (periph_base < APB2PERIPH_BASE_NS) {
        uint32_t pos = (periph_base - APB1PERIPH_BASE_NS) / 0x400;
        if (pos < 32) {
            return (struct cline){.en = &RCC->APB1LENR,
                                  .rst = &RCC->APB1LRSTR,
                                  .bit = 1 << pos};
        } else {
            return (struct cline){.en = &RCC->APB1HENR,
                                  .rst = &RCC->APB1HRSTR,
                                  .bit = 1 << (pos - 32)};
        }
    } else if (periph_base < AHB1PERIPH_BASE_NS) {
        uint32_t pos = (periph_base - APB2PERIPH_BASE_NS) / 0x400;
        return (struct cline){.en = &RCC->APB2ENR,
                              .rst = &RCC->APB2RSTR,
                              .bit = 1 << pos};

    } else if (periph_base < AHB2PERIPH_BASE_NS) {
        uint32_t pos = (periph_base - AHB1PERIPH_BASE_NS) / 0x400;
        return (struct cline){.en = &RCC->AHB1ENR,
                              .rst = &RCC->AHB1RSTR,
                              .bit = 1 << pos};

    } else if (periph_base < APB3PERIPH_BASE_NS) {
        uint32_t pos = (periph_base - AHB2PERIPH_BASE_NS) / 0x400;
        return (struct cline){.en = &RCC->AHB2ENR,
                              .rst = &RCC->AHB2RSTR,
                              .bit = 1 << pos};

    } else {
        uint32_t pos = (periph_base - APB3PERIPH_BASE_NS) / 0x400;
        return (struct cline){.en = &RCC->APB3ENR,
                              .rst = &RCC->APB3RSTR,
                              .bit = 1 << pos};
    }
    // TODO: h562 has an AHB4
}

// Return the frequency of the given peripheral clock
uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

// Enable a GPIO peripheral clock
void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - GPIOA_BASE) / 0x400;
    RCC->AHB2ENR |= 1 << rcc_pos;
    RCC->AHB2ENR;
}

#if !CONFIG_STM32_CLOCK_REF_INTERNAL
DECL_CONSTANT_STR("RESERVE_PINS_crystal", "PH0,PH1");
#endif

// Main clock setup called at chip startup
static void
clock_setup(void)
{
    // Set flash latency -- assume vos0
    uint32_t latency = ((CONFIG_CLOCK_FREQ>210000000) ? FLASH_ACR_LATENCY_5WS :
                       ((CONFIG_CLOCK_FREQ>168000000) ? FLASH_ACR_LATENCY_4WS :
                       ((CONFIG_CLOCK_FREQ>126000000) ? FLASH_ACR_LATENCY_3WS :
                       ((CONFIG_CLOCK_FREQ>84000000) ? FLASH_ACR_LATENCY_2WS :
                       ((CONFIG_CLOCK_FREQ>42000000) ? FLASH_ACR_LATENCY_1WS :
                                                    FLASH_ACR_LATENCY_0WS)))));
    uint32_t wrhf = ((CONFIG_CLOCK_FREQ>168000000) ? FLASH_ACR_WRHIGHFREQ_1 :
                       ((CONFIG_CLOCK_FREQ>84000000) ? FLASH_ACR_WRHIGHFREQ_0 :
                                                    0));
    FLASH->ACR = (latency | FLASH_ACR_PRFTEN | wrhf);

    PWR->VOSCR = PWR_VOSCR_VOS_Msk; // set vreg to vosc0 for max freq
    while (!(PWR->VOSSR & PWR_VOSSR_VOSRDY))
        ;

    RCC->CFGR2 = 0;
    RCC->CR &= ~RCC_CR_PLL1ON;

    // pll1 to sysclk, pll2 to usb
    uint32_t pll_base = 4000000, pll_freq = CONFIG_CLOCK_FREQ * 2, pllcfgr;
    if (!CONFIG_STM32_CLOCK_REF_INTERNAL) {
        // Configure 250Mhz PLL from external crystal (HSE)
        uint32_t div = CONFIG_CLOCK_REF_FREQ / pll_base;
        RCC->CR |= RCC_CR_HSEON;
        while (!(RCC->CR & RCC_CR_HSERDY))
            ;
        pllcfgr = RCC_PLL1CFGR_PLL1SRC | RCC_PLL1CFGR_PLL1PEN | RCC_PLL1CFGR_PLL1QEN // HSE
	            | RCC_PLL1CFGR_PLL1RGE_1 | (div << RCC_PLL1CFGR_PLL1M_Pos); // 4-8mhz 
    } else {
        // Configure 150Mhz PLL from internal 16Mhz oscillator (HSI)
        uint32_t div = 32000000 / pll_base;
        pllcfgr = RCC_PLL1CFGR_PLL1SRC_0 | RCC_PLL1CFGR_PLL1PEN // HSI
	            | RCC_PLL1CFGR_PLL1RGE_1 | (div << RCC_PLL1CFGR_PLL1M_Pos); // 4-8mhz 
        RCC->CR |= RCC_CR_HSION;
        while (!(RCC->CR & RCC_CR_HSIRDY))
            ;
    }
    RCC->PLL1CFGR = pllcfgr;
    RCC->PLL1DIVR = ((pll_freq/pll_base - 1) << RCC_PLL1DIVR_PLL1N_Pos)
	            | RCC_PLL1DIVR_PLL1R_0 | (9 << RCC_PLL1DIVR_PLL1Q_Pos) | RCC_PLL1DIVR_PLL1P_0;

    // Enable 48Mhz USB clock using clock recovery -- switch to pll later..
    if (CONFIG_USBSERIAL) {
        RCC->CR |= RCC_CR_HSI48ON;
        while (!(RCC->CR & RCC_CR_HSI48RDY))
            ;
        enable_pclock(CRS_BASE);
        CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
	RCC->CCIPR4 |= RCC_CCIPR4_USBSEL_0; // hsi48
    }

    RCC->CR |= RCC_CR_PLL1ON;
    // Wait for PLL lock
    while (!(RCC->CR & RCC_CR_PLL1RDY))
        ;

    // Switch system clock to PLL
    RCC->CFGR1 |= RCC_CFGR1_SW;
    while ((RCC->CFGR1 & RCC_CFGR1_SWS_Msk) != RCC_CFGR1_SWS)
        ;
}

// Handle reboot requests
void
bootloader_request(void)
{
    try_request_canboot();
    dfu_reboot();
}

void configure_mpu(void) {
  __DMB(); /* Force any outstanding transfers to complete before disabling MPU */

  /* Disable fault exceptions */
  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;

  /* Disable the MPU */
  MPU->CTRL  &= ~MPU_CTRL_ENABLE_Msk;

  /* Follow ARM recommendation with */
  /* Data Synchronization and Instruction Synchronization Barriers to ensure MPU configuration */
  __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
  __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */

    __DMB();
  /* Set the Region number */
  MPU->RNR = 0;

  /* Disable the Region */
  MPU->RLAR &= ~MPU_RLAR_EN_Msk;

  // TODO: convert these to defined constants
  MPU->RBAR = (((uint32_t)UID_BASE & 0xFFFFFFE0UL)  |
                (0           << MPU_RBAR_SH_Pos)  |
                (3      << MPU_RBAR_AP_Pos)  |
                (1           << MPU_RBAR_XN_Pos));

  MPU->RLAR = (((uint32_t)0x08ffffff                    & 0xFFFFFFE0UL) |
                (0       << MPU_RLAR_AttrIndx_Pos) |
                (1                << MPU_RLAR_EN_Pos));

    __DMB(); /* Data Memory Barrier operation to force any outstanding writes to memory before enabling the MPU */


    MPU->MAIR0 = ARM_MPU_ATTR_DEVICE_nGnRnE | 0x4 | 0x40
                            | 0x0 | 0x0;

    __DMB();


  /* Enable the MPU */
  MPU->CTRL |= MPU_CTRL_ENABLE_Msk | 0x4;

  /* Enable fault exceptions */
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;

  /* Follow ARM recommendation with */
  /* Data Synchronization and Instruction Synchronization Barriers to ensure MPU configuration */
  __DSB(); /* Ensure that the subsequent instruction is executed only after the write to memory */
  __ISB(); /* Flush and refill pipeline with updated MPU configuration settings */
}


/****************************************************************
 * Startup
 ****************************************************************/

// Main entry point - called from armcm_boot.c:ResetHandler()
void
armcm_main(void)
{
    // Run SystemInit() and then restore VTOR
    SystemInit();

    configure_mpu();
    enable_pclock(ICACHE_BASE);
    ICACHE->CR |= ICACHE_CR_EN;

    SCB->VTOR = (uint32_t)VectorTable;

    dfu_reboot_check();

    clock_setup();

    sched_main();
}

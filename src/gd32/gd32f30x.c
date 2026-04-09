// Code to setup clocks and gpio on GD32F30x (Cortex-M4, 120MHz)
//
// GD32F30x is register-compatible with STM32F103 for GPIO/USART/SPI/I2C/ADC.
// The key differences are the RCU (instead of RCC) and 120MHz PLL setup.
//
// We use STM32F103 CMSIS headers for all peripheral structs.
// GD32F30x-specific registers (RCU, PMU, FMC) are accessed via raw macros below.

#include "autoconf.h"
#include "board/armcm_boot.h" // VectorTable
#include "board/armcm_reset.h" // try_request_canboot
#include "board/irq.h" // irq_disable
#include "board/misc.h" // bootloader_request
#include "internal.h" // enable_pclock (includes stm32f1xx.h)
#include "sched.h" // sched_main

/****************************************************************
 * GD32F30x raw register definitions (RCU, PMU, FMC)
 * These are the GD32-specific registers not present in stm32f1xx.h
 ****************************************************************/

#define GD32_REG32(addr) (*(volatile uint32_t *)(uint32_t)(addr))
#define GD32_BIT(x)      ((uint32_t)(1UL << (x)))
#define GD32_BITS(s,e)   ((uint32_t)(0xFFFFFFFFUL << (s)) & (0xFFFFFFFFUL >> (31UL - (e))))

// RCU base (same as STM32F103 RCC: 0x40021000)
#define GD32_RCU_BASE     (0x40021000UL)
#define GD32_RCU_CTL      GD32_REG32(GD32_RCU_BASE + 0x00U)
#define GD32_RCU_CFG0     GD32_REG32(GD32_RCU_BASE + 0x04U)
#define GD32_RCU_APB1EN   GD32_REG32(GD32_RCU_BASE + 0x1CU)
#define GD32_RCU_CFG1     GD32_REG32(GD32_RCU_BASE + 0x2CU)

// PMU base: 0x40007000
#define GD32_PMU_BASE     (0x40007000UL)
#define GD32_PMU_CTL      GD32_REG32(GD32_PMU_BASE + 0x00U)
#define GD32_PMU_CS       GD32_REG32(GD32_PMU_BASE + 0x04U)

// FMC (Flash) base: 0x40022000
#define GD32_FMC_BASE     (0x40022000UL)
#define GD32_FMC_WS       GD32_REG32(GD32_FMC_BASE + 0x00U)

// HXTAL startup timeout
#define GD32_HXTAL_TIMEOUT  ((uint16_t)0xFFFFU)

// RCU_CTL bits
#define GD32_RCU_CTL_HXTALEN   GD32_BIT(16)
#define GD32_RCU_CTL_HXTALSTB  GD32_BIT(17)
#define GD32_RCU_CTL_PLLEN     GD32_BIT(24)
#define GD32_RCU_CTL_PLLSTB    GD32_BIT(25)
#define GD32_RCU_CTL_PLL1EN    GD32_BIT(26)
#define GD32_RCU_CTL_PLL1STB   GD32_BIT(27)

// RCU_CFG0 bits
#define GD32_RCU_CFG0_SCS_PLL  ((uint32_t)0x00000002U) // PLL as sysclk
#define GD32_RCU_SCSS_PLL      ((uint32_t)0x00000008U) // sysclk status = PLL
#define GD32_RCU_AHB_DIV1      ((uint32_t)0x00000000U)
#define GD32_RCU_APB2_DIV2     ((uint32_t)0x00002000U) // APB2 = AHB/2
#define GD32_RCU_APB1_DIV2     ((uint32_t)0x00000400U) // APB1 = AHB/2
#define GD32_RCU_CFG0_PLLSEL   GD32_BIT(16)
#define GD32_RCU_CFG0_PREDV0   GD32_BIT(17) // HXTAL/2 as PLL src (HD/XD)
#define GD32_RCU_PLLSRC_HXTAL  GD32_BIT(16)
// PLL MUL30: PLLMF[5:0] = 0b011101 (bits 30,27,21:18)
// PLLMF_4 = bit27, PLLMF = bits[21:18], MUL30: PLLMF_4|CFG0_PLLMF(13)
#define GD32_CFG0_PLLMF_MASK   (GD32_BITS(18,21) | GD32_BIT(27) | GD32_BIT(30))
#define GD32_RCU_PLL_MUL30     (GD32_BIT(27) | ((uint32_t)(13U) << 18U))

// CL series CFG1 for PLL1 chain
#define GD32_RCU_CFG1_PLLPRESEL  GD32_BIT(30)
#define GD32_RCU_CFG1_PREDV0SEL  GD32_BIT(16)
#define GD32_RCU_CFG1_PLL1MF_MASK GD32_BITS(8,11)
#define GD32_RCU_CFG1_PREDV1_MASK GD32_BITS(4,7)
#define GD32_RCU_CFG1_PREDV0_MASK GD32_BITS(0,3)
#define GD32_RCU_PLLPRESRC_HXTAL  GD32_BIT(30)  // 0 = HXTAL
#define GD32_RCU_PREDV0SRC_PLL1   GD32_BIT(16)
#define GD32_RCU_PLL1_MUL8        ((uint32_t)(6U) << 8U)  // MUL=6+2=8
#define GD32_RCU_PREDV1_DIV5      ((uint32_t)(4U) << 4U)  // DIV=4+1=5
#define GD32_RCU_PREDV0_DIV10     ((uint32_t)(9U) << 0U)  // DIV=9+1=10

// APB1EN
#define GD32_RCU_APB1EN_PMUEN  GD32_BIT(28)

// PMU_CTL bits
#define GD32_PMU_CTL_LDOVS  GD32_BITS(14,15)
#define GD32_PMU_CTL_HDEN   GD32_BIT(16)
#define GD32_PMU_CTL_HDS    GD32_BIT(17)

// PMU_CS bits
#define GD32_PMU_CS_HDRF    GD32_BIT(16)
#define GD32_PMU_CS_HDSRF   GD32_BIT(17)

// FMC wait states: bits[2:0]
#define GD32_FMC_WSCNT_MASK ((uint32_t)0x7U)
#define GD32_FMC_WSCNT_2    ((uint32_t)0x2U)


/****************************************************************
 * Clock setup
 ****************************************************************/

// APB1 and APB2 are at SYSCLK/2 = 60MHz
#define FREQ_PERIPH (CONFIG_CLOCK_FREQ / 2)

// Map a peripheral address to its enable/reset bits.
// GD32F30x RCU has same layout as STM32F103 RCC for AHBENR/APB1ENR/APB2ENR.
// We use the STM32F1 RCC struct for this (identical register layout).
struct cline
lookup_clock_line(uint32_t periph_base)
{
    if (periph_base >= AHBPERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - AHBPERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->AHBENR, .bit=bit};
    } else if (periph_base >= APB2PERIPH_BASE) {
        uint32_t bit = 1 << ((periph_base - APB2PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB2ENR, .rst=&RCC->APB2RSTR, .bit=bit};
    } else {
        uint32_t bit = 1 << ((periph_base - APB1PERIPH_BASE) / 0x400);
        return (struct cline){.en=&RCC->APB1ENR, .rst=&RCC->APB1RSTR, .bit=bit};
    }
}

uint32_t
get_pclock_frequency(uint32_t periph_base)
{
    return FREQ_PERIPH;
}

void
gpio_clock_enable(GPIO_TypeDef *regs)
{
    uint32_t rcc_pos = ((uint32_t)regs - APB2PERIPH_BASE) / 0x400;
    RCC->APB2ENR |= 1 << rcc_pos;
    RCC->APB2ENR; // read-back to ensure clock enabled
}


/****************************************************************
 * GD32F30x 120MHz PLL clock setup
 ****************************************************************/

static void
gd32f30x_clock_setup(void)
{
    // Enable HXTAL
    GD32_RCU_CTL |= GD32_RCU_CTL_HXTALEN;
    uint32_t timeout = GD32_HXTAL_TIMEOUT;
    while (!(GD32_RCU_CTL & GD32_RCU_CTL_HXTALSTB) && --timeout)
        ;
    if (!(GD32_RCU_CTL & GD32_RCU_CTL_HXTALSTB))
        while (1); // HXTAL failed

    // Enable PMU clock and configure LDO voltage for 120MHz
    GD32_RCU_APB1EN |= GD32_RCU_APB1EN_PMUEN;
    GD32_PMU_CTL |= GD32_PMU_CTL_LDOVS;

    // AHB = SYSCLK, APB2 = AHB/2, APB1 = AHB/2
    GD32_RCU_CFG0 |= GD32_RCU_AHB_DIV1;
    GD32_RCU_CFG0 |= GD32_RCU_APB2_DIV2;
    GD32_RCU_CFG0 |= GD32_RCU_APB1_DIV2;

#if defined(GD32F30X_HD) || defined(GD32F30X_XD)
    // HD/XD: PLL source = HXTAL/2, multiply by 30 = 120MHz (8MHz HXTAL)
    GD32_RCU_CFG0 &= ~(GD32_RCU_CFG0_PLLSEL | GD32_RCU_CFG0_PREDV0);
    GD32_RCU_CFG0 |= (GD32_RCU_PLLSRC_HXTAL | GD32_RCU_CFG0_PREDV0);
    GD32_RCU_CFG0 &= ~GD32_CFG0_PLLMF_MASK;
    GD32_RCU_CFG0 |= GD32_RCU_PLL_MUL30;

#elif defined(GD32F30X_CL)
    // CL: HXTAL(25MHz) -> PREDV1/5 -> PLL1*8 -> PREDV0/10 -> 4MHz -> PLL*30 = 120MHz
    GD32_RCU_CFG0 &= ~GD32_CFG0_PLLMF_MASK;
    GD32_RCU_CFG0 |= (GD32_RCU_PLLSRC_HXTAL | GD32_RCU_PLL_MUL30);
    GD32_RCU_CFG1 &= ~(GD32_RCU_CFG1_PLLPRESEL | GD32_RCU_CFG1_PREDV0SEL
                       | GD32_RCU_CFG1_PLL1MF_MASK | GD32_RCU_CFG1_PREDV1_MASK
                       | GD32_RCU_CFG1_PREDV0_MASK);
    GD32_RCU_CFG1 |= (GD32_RCU_PREDV0SRC_PLL1 | GD32_RCU_PLL1_MUL8
                      | GD32_RCU_PREDV1_DIV5 | GD32_RCU_PREDV0_DIV10);
    GD32_RCU_CTL |= GD32_RCU_CTL_PLL1EN;
    while (!(GD32_RCU_CTL & GD32_RCU_CTL_PLL1STB))
        ;
#endif

    // Enable main PLL and wait
    GD32_RCU_CTL |= GD32_RCU_CTL_PLLEN;
    while (!(GD32_RCU_CTL & GD32_RCU_CTL_PLLSTB))
        ;

    // Enable high-drive mode for 120MHz
    GD32_PMU_CTL |= GD32_PMU_CTL_HDEN;
    while (!(GD32_PMU_CS & GD32_PMU_CS_HDRF))
        ;
    GD32_PMU_CTL |= GD32_PMU_CTL_HDS;
    while (!(GD32_PMU_CS & GD32_PMU_CS_HDSRF))
        ;

    // Set flash wait states to 2 for 120MHz
    GD32_FMC_WS = (GD32_FMC_WS & ~GD32_FMC_WSCNT_MASK) | GD32_FMC_WSCNT_2;

    // Switch sysclk to PLL
    GD32_RCU_CFG0 &= ~(uint32_t)0x3U; // clear SCS
    GD32_RCU_CFG0 |= GD32_RCU_CFG0_SCS_PLL;
    while ((GD32_RCU_CFG0 & (uint32_t)0xCU) != (uint32_t)0x8U) // wait SCSS=PLL
        ;
}


/****************************************************************
 * GPIO setup (STM32F1-style - same register layout on GD32F30x)
 ****************************************************************/

static void
gd32f30x_alternative_remap(uint32_t mapr_mask, uint32_t mapr_value)
{
    static uint32_t mapr = 0;
    mapr &= ~mapr_mask;
    mapr |= mapr_value;
    AFIO->MAPR = mapr;
}

#define GD32_OSPEED 0x1 // ~10MHz output speed

void
gpio_peripheral(uint32_t gpio, uint32_t mode, int pullup)
{
    GPIO_TypeDef *regs = digital_regs[GPIO2PORT(gpio)];
    gpio_clock_enable(regs);

    uint32_t pos = gpio % 16, shift = (pos % 8) * 4, msk = 0xf << shift, cfg;
    if (mode == GPIO_INPUT) {
        cfg = pullup ? 0x8 : 0x4;
    } else if (mode == GPIO_OUTPUT) {
        cfg = GD32_OSPEED;
    } else if (mode == (GPIO_OUTPUT | GPIO_OPEN_DRAIN)) {
        cfg = 0x4 | GD32_OSPEED;
    } else if (mode == GPIO_ANALOG) {
        cfg = 0x0;
    } else {
        if (mode & GPIO_OPEN_DRAIN)
            cfg = 0xc | GD32_OSPEED;
        else if (pullup > 0)
            cfg = 0x8;
        else
            cfg = 0x8 | GD32_OSPEED;
    }
    if (pos & 0x8)
        regs->CRH = (regs->CRH & ~msk) | (cfg << shift);
    else
        regs->CRL = (regs->CRL & ~msk) | (cfg << shift);

    if (pullup > 0)
        regs->BSRR = 1 << pos;
    else if (pullup < 0)
        regs->BSRR = 1 << (pos + 16);

    if (gpio == GPIO('A', 13) || gpio == GPIO('A', 14))
        gd32f30x_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
                                   AFIO_MAPR_SWJ_CFG_DISABLE);

    uint32_t func = (mode >> 4) & 0xf;
    if (func == 1) {
        if (gpio == GPIO('A', 15) || gpio == GPIO('B', 3))
            gd32f30x_alternative_remap(AFIO_MAPR_TIM2_REMAP_Msk,
                                       AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1);
        else if (gpio == GPIO('B', 10) || gpio == GPIO('B', 11))
            gd32f30x_alternative_remap(AFIO_MAPR_TIM2_REMAP_Msk,
                                       AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2);
    } else if (func == 2) {
        if (gpio == GPIO('B', 4) || gpio == GPIO('B', 5))
            gd32f30x_alternative_remap(AFIO_MAPR_TIM3_REMAP_Msk,
                                       AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
        else if (gpio == GPIO('C', 6) || gpio == GPIO('C', 7)
                 || gpio == GPIO('C', 8) || gpio == GPIO('C', 9))
            gd32f30x_alternative_remap(AFIO_MAPR_TIM3_REMAP_Msk,
                                       AFIO_MAPR_TIM3_REMAP_FULLREMAP);
        else if (gpio == GPIO('D', 12) || gpio == GPIO('D', 13)
                 || gpio == GPIO('D', 14) || gpio == GPIO('D', 15))
            gd32f30x_alternative_remap(AFIO_MAPR_TIM4_REMAP_Msk,
                                       AFIO_MAPR_TIM4_REMAP);
    } else if (func == 4) {
        if (gpio == GPIO('B', 8) || gpio == GPIO('B', 9))
            gd32f30x_alternative_remap(AFIO_MAPR_I2C1_REMAP_Msk,
                                       AFIO_MAPR_I2C1_REMAP);
    } else if (func == 5) {
        if (gpio == GPIO('B', 3) || gpio == GPIO('B', 4)
            || gpio == GPIO('B', 5))
            gd32f30x_alternative_remap(AFIO_MAPR_SPI1_REMAP_Msk,
                                       AFIO_MAPR_SPI1_REMAP);
    } else if (func == 7) {
        if (gpio == GPIO('B', 6) || gpio == GPIO('B', 7))
            gd32f30x_alternative_remap(AFIO_MAPR_USART1_REMAP_Msk,
                                       AFIO_MAPR_USART1_REMAP);
        else if (gpio == GPIO('D', 5) || gpio == GPIO('D', 6))
            gd32f30x_alternative_remap(AFIO_MAPR_USART2_REMAP_Msk,
                                       AFIO_MAPR_USART2_REMAP);
        else if (gpio == GPIO('D', 8) || gpio == GPIO('D', 9))
            gd32f30x_alternative_remap(AFIO_MAPR_USART3_REMAP_Msk,
                                       AFIO_MAPR_USART3_REMAP_FULLREMAP);
    }
}


/****************************************************************
 * Bootloader
 ****************************************************************/

static void
usb_hid_bootloader(void)
{
    irq_disable();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR4 = 0x424C;
    PWR->CR &=~ PWR_CR_DBP;
    NVIC_SystemReset();
}

static void
usb_stm32duino_bootloader(void)
{
    irq_disable();
    RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
    PWR->CR |= PWR_CR_DBP;
    BKP->DR10 = 0x01;
    PWR->CR &=~ PWR_CR_DBP;
    NVIC_SystemReset();
}

void
bootloader_request(void)
{
    try_request_canboot();
    if (CONFIG_GD32_FLASH_START_800)
        usb_hid_bootloader();
    else if (CONFIG_GD32_FLASH_START_2000)
        usb_stm32duino_bootloader();
}


/****************************************************************
 * Startup
 ****************************************************************/

void
armcm_main(void)
{
    SCB->VTOR = (uint32_t)VectorTable;

    // Reset peripheral clocks
    RCC->AHBENR  = 0x14;
    RCC->APB1ENR = 0;
    RCC->APB2ENR = 0;

    // Setup 120MHz PLL
    gd32f30x_clock_setup();

    // Disable JTAG, keep SWD
    enable_pclock(AFIO_BASE);
    gd32f30x_alternative_remap(AFIO_MAPR_SWJ_CFG_Msk,
                               AFIO_MAPR_SWJ_CFG_JTAGDISABLE);

    sched_main();
}

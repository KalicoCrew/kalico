// STM32 serial
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/serial_irq.h" // serial_rx_byte
#include "command.h" // DECL_CONSTANT_STR
#include "generic/kalico_nvic_prio.h" // KALICO_MOTION_NVIC_PRIO
#include "internal.h" // enable_pclock
#include "sched.h" // DECL_INIT

// Select the configured serial port
#if CONFIG_STM32_SERIAL_USART1
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA10,PA9");
  #define GPIO_Rx GPIO('A', 10)
  #define GPIO_Tx GPIO('A', 9)
  #define GPIO_AF_MODE 7
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn
  #define USARTx_RX_DMA_USART1 1
#elif CONFIG_STM32_SERIAL_USART1_ALT_PB7_PB6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB7,PB6");
  #define GPIO_Rx GPIO('B', 7)
  #define GPIO_Tx GPIO('B', 6)
  #define GPIO_AF_MODE 7
  #define USARTx USART1
  #define USARTx_IRQn USART1_IRQn
  #define USARTx_RX_DMA_USART1 1
#elif CONFIG_STM32_SERIAL_USART2
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA3,PA2");
  #define GPIO_Rx GPIO('A', 3)
  #define GPIO_Tx GPIO('A', 2)
  #define GPIO_AF_MODE 7
  #define USARTx USART2
  #define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART2_ALT_PD6_PD5
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD6,PD5");
  #define GPIO_Rx GPIO('D', 6)
  #define GPIO_Tx GPIO('D', 5)
  #define GPIO_AF_MODE 7
  #define USARTx USART2
  #define USARTx_IRQn USART2_IRQn
#elif CONFIG_STM32_SERIAL_USART3
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PB11,PB10");
  #define GPIO_Rx GPIO('B', 11)
  #define GPIO_Tx GPIO('B', 10)
  #define GPIO_AF_MODE 7
  #define USARTx USART3
  #define USARTx_IRQn USART3_IRQn
#elif CONFIG_STM32_SERIAL_USART3_ALT_PD9_PD8
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PD9,PD8");
  #define GPIO_Rx GPIO('D', 9)
  #define GPIO_Tx GPIO('D', 8)
  #define GPIO_AF_MODE 7
  #define USARTx USART3
  #define USARTx_IRQn USART3_IRQn
#elif CONFIG_STM32_SERIAL_USART6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PA12,PA11");
  #define GPIO_Rx GPIO('A', 12)
  #define GPIO_Tx GPIO('A', 11)
  #define GPIO_AF_MODE 8
  #define USARTx USART6
  #define USARTx_IRQn USART6_IRQn
#elif CONFIG_STM32_SERIAL_USART6_ALT_PC7_PC6
  DECL_CONSTANT_STR("RESERVE_PINS_serial", "PC7,PC6");
  #define GPIO_Rx GPIO('C', 7)
  #define GPIO_Tx GPIO('C', 6)
  #define GPIO_AF_MODE 8
  #define USARTx USART6
  #define USARTx_IRQn USART6_IRQn
#endif

// DMA-based RX. Without an RX FIFO, the per-byte RX interrupt drops bytes
// whenever an equal-priority motion ISR is running (no time to read DR before
// the next byte overruns it) -> corrupt frames -> retransmit stalls. DMA
// captures every byte into a RAM ring with no per-byte interrupt, so the motion
// ISRs and serial RX no longer compete. Scoped to F401/USART1 (the only
// validated bench); every other config keeps the byte-interrupt path below.
// USART1_RX is DMA2 Stream 2 Channel 4 on the STM32F4 request map.
#if USARTx_RX_DMA_USART1 && CONFIG_MACH_STM32F401
  #define SERIAL_RX_DMA          1
  #define SERIAL_RX_DMA_CTRL     DMA2
  #define SERIAL_RX_DMA_STREAM   DMA2_Stream2
  #define SERIAL_RX_DMA_IRQn     DMA2_Stream2_IRQn
  #define SERIAL_RX_DMA_CHANNEL  4u
  #define SERIAL_RX_DMA_RCC_EN   RCC_AHB1ENR_DMA2EN
  #define SERIAL_RX_DMA_CLEAR    (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2     \
                                  | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2  \
                                  | DMA_LIFCR_CFEIF2)
#endif

#if SERIAL_RX_DMA
  #define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                     | USART_CR1_IDLEIE)
#else
  #define CR1_FLAGS (USART_CR1_UE | USART_CR1_RE | USART_CR1_TE   \
                     | USART_CR1_RXNEIE)
#endif

#if SERIAL_RX_DMA

// Power-of-two so the read cursor wraps with a mask. 256 B at 250000 baud is
// ~10 ms of runway; the drain runs every 128 B (DMA half/full IRQ) plus on each
// idle gap, so the ring never laps even when a motion ISR delays the drain.
#define SERIAL_RX_DMA_BUF_SIZE 256
static uint8_t dma_rx_buf[SERIAL_RX_DMA_BUF_SIZE];
static uint16_t dma_rx_tail;

// Feed the parser everything the DMA engine has captured since last drain. The
// DMA write cursor is BUF_SIZE - NDTR; bytes [tail, head) are new. Non-critical
// timing: DMA has already saved the bytes, so a delayed drain can't lose data.
static void
serial_rx_dma_drain(void)
{
    uint16_t ndtr = SERIAL_RX_DMA_STREAM->NDTR;
    uint16_t head = (SERIAL_RX_DMA_BUF_SIZE - ndtr) & (SERIAL_RX_DMA_BUF_SIZE - 1);
    while (dma_rx_tail != head) {
        serial_rx_byte(dma_rx_buf[dma_rx_tail]);
        dma_rx_tail = (dma_rx_tail + 1) & (SERIAL_RX_DMA_BUF_SIZE - 1);
    }
}

// Non-static: the armcm vector table (a separately-generated TU) references
// this handler by name, so it needs external linkage like USARTx_IRQHandler.
void
serial_rx_dma_irq(void)
{
    SERIAL_RX_DMA_CTRL->LIFCR = SERIAL_RX_DMA_CLEAR;
    serial_rx_dma_drain();
}

static void
serial_rx_dma_init(void)
{
    RCC->AHB1ENR |= SERIAL_RX_DMA_RCC_EN;
    (void)RCC->AHB1ENR;
    SERIAL_RX_DMA_STREAM->CR = 0;
    while (SERIAL_RX_DMA_STREAM->CR & DMA_SxCR_EN)
        ;
    SERIAL_RX_DMA_CTRL->LIFCR = SERIAL_RX_DMA_CLEAR;
    SERIAL_RX_DMA_STREAM->PAR = (uint32_t)&USARTx->DR;
    SERIAL_RX_DMA_STREAM->M0AR = (uint32_t)dma_rx_buf;
    SERIAL_RX_DMA_STREAM->NDTR = SERIAL_RX_DMA_BUF_SIZE;
    dma_rx_tail = 0;
    // peripheral-to-memory (DIR=00), byte size (00): hardware defaults
    SERIAL_RX_DMA_STREAM->CR =
        (SERIAL_RX_DMA_CHANNEL << DMA_SxCR_CHSEL_Pos)
        | DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_HTIE | DMA_SxCR_TCIE;
    SERIAL_RX_DMA_STREAM->CR |= DMA_SxCR_EN;
    USARTx->CR3 |= USART_CR3_DMAR;
    armcm_enable_irq(serial_rx_dma_irq, SERIAL_RX_DMA_IRQn, 2);
}

#endif

void
USARTx_IRQHandler(void)
{
    uint32_t sr = USARTx->SR;
#if SERIAL_RX_DMA
    if (sr & USART_SR_IDLE) {
        // Reading SR (above) then DR clears IDLE; the byte itself was already
        // taken by DMA, so this read just acks the flag.
        (void)USARTx->DR;
        serial_rx_dma_drain();
    }
#else
    if (sr & (USART_SR_RXNE | USART_SR_ORE)) {
        // The ORE flag is automatically cleared by reading SR, followed
        // by reading DR.
        serial_rx_byte(USARTx->DR);
    }
#endif
    if (sr & USART_SR_TXE && USARTx->CR1 & USART_CR1_TXEIE) {
        uint8_t data;
        int ret = serial_get_tx_byte(&data);
        if (ret)
            USARTx->CR1 = CR1_FLAGS;
        else
            USARTx->DR = data;
    }
}

void
serial_enable_tx_irq(void)
{
    USARTx->CR1 = CR1_FLAGS | USART_CR1_TXEIE;
}

void
serial_init(void)
{
    enable_pclock((uint32_t)USARTx);

    uint32_t pclk = get_pclock_frequency((uint32_t)USARTx);
    uint32_t div = DIV_ROUND_CLOSEST(pclk, CONFIG_SERIAL_BAUD);
    USARTx->BRR = (((div / 16) << USART_BRR_DIV_Mantissa_Pos)
                   | ((div % 16) << USART_BRR_DIV_Fraction_Pos));
    USARTx->CR1 = CR1_FLAGS;
    armcm_enable_irq(USARTx_IRQHandler, USARTx_IRQn, 2);

#if SERIAL_RX_DMA
    serial_rx_dma_init();
#endif

    gpio_peripheral(GPIO_Rx, GPIO_FUNCTION(GPIO_AF_MODE), 1);
    gpio_peripheral(GPIO_Tx, GPIO_FUNCTION(GPIO_AF_MODE), 0);
}
DECL_INIT(serial_init);

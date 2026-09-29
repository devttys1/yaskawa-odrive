/**
 * @file    yaskawa_odrive.c
 * @brief   AS5047P-compatible SPI slave; see yaskawa_odrive.h.
 */

#include "yaskawa_odrive.h"
#include "yaskawa_hw.h"

#ifndef YASK_HOST_TEST
#include "main.h"      /* Error_Handler */
#endif

/* ── Frame construction (pure, host-testable) ────────────────────────────── */

uint16_t yask_odrive_word(uint16_t angle14, uint8_t ef)
{
    /*
     * AS5047P read-response layout:
     *
     *   bit 15     even parity over bits[14:0]
     *   bit 14     EF, error flag
     *   bits[13:0] 14-bit angle
     *
     * The angle occupies the LOW 14 bits.  An earlier version placed it in
     * bits[14:1] with EF in bit 0, which is the layout the reference header
     * described; the controller then read every angle as exactly double the
     * intended value (17 sent, 34 reported) because it takes bits[13:0]
     * verbatim.
     */
    uint16_t word = (uint16_t)((angle14 & 0x3FFFU)
                             | ((uint16_t)(ef & 1U) << 14));

    /* Even parity over bits[14:0], placed in bit 15. */
    uint16_t t = word;
    t ^= (uint16_t)(t >> 8);
    t ^= (uint16_t)(t >> 4);
    t ^= (uint16_t)(t >> 2);
    t ^= (uint16_t)(t >> 1);

#if YASK_ODRIVE_ODD_PARITY
    /* Odd parity: set bit 15 when the low bits already have an even count. */
    return (uint16_t)(word | (uint16_t)(((t & 1U) ^ 1U) << 15));
#else
    return (uint16_t)(word | (uint16_t)((t & 1U) << 15));
#endif
}

#ifndef YASK_HOST_TEST

/*
 * ── Transfer path ────────────────────────────────────────────────────────
 *
 * The controller polls continuously and asynchronously -- an ODrive clocks
 * a 16-bit frame every 125 us (8 kHz), measured on a logic analyser -- while
 * this firmware's main loop turns at roughly 600 Hz.  Servicing SPI in
 * software cannot bridge that: an earlier RXNE-interrupt version kept up
 * with only 11% of transfers (53 447 serviced against 490 464 chip-select
 * edges), the receive FIFO stayed full, OVR latched, and byte framing
 * degraded until the slave was emitting half of one word and half of the
 * next.
 *
 * So the CPU is taken out of the path entirely.  Two circular DMA streams
 * run forever:
 *
 *   TX  s_tx_word  ->  SPI4->DR    reloads the same word on every transfer
 *   RX  SPI4->DR   ->  s_rx_sink   discards what the master sends
 *
 * Circular mode means the stream never completes, so the hardware serves
 * every transfer with no interrupt and no chance of the FIFO overrunning.
 * Publishing a new position is then a single aligned 16-bit store into
 * s_tx_word, which the DMA picks up on the next poll.
 */

/* DMA-visible: must live in SRAM1, since DMA2 cannot reach DTCM. */
__attribute__((section(".dma_buffer"), aligned(4)))
static volatile uint16_t s_tx_word;

__attribute__((section(".dma_buffer"), aligned(4)))
static volatile uint16_t s_rx_sink;

static DMA_HandleTypeDef s_hdma_tx;
static DMA_HandleTypeDef s_hdma_rx;

volatile uint32_t g_nss_count;
volatile uint32_t g_tx_count;
volatile uint32_t g_rx_count;

/**
 * @brief Configure one circular stream between memory and SPI4->DR.
 */
static void dma_setup(DMA_HandleTypeDef *h, DMA_Stream_TypeDef *stream,
                      uint32_t channel, uint32_t dir)
{
    h->Instance                 = stream;
    h->Init.Channel             = channel;
    h->Init.Direction           = dir;
    h->Init.PeriphInc           = DMA_PINC_DISABLE;
    /*
     * The memory side is a single word that is re-sent (or re-overwritten)
     * every transfer, so the pointer must not advance.
     */
    h->Init.MemInc              = DMA_MINC_DISABLE;
    h->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    h->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    h->Init.Mode                = DMA_CIRCULAR;
    h->Init.Priority            = DMA_PRIORITY_HIGH;
    h->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(h) != HAL_OK)
    {
        Error_Handler();
    }
}

void yask_odrive_init(SPI_HandleTypeDef *hspi)
{
    s_tx_word = yask_odrive_word(0U, 1U);   /* EF=1 until a real decode */
    s_rx_sink = 0U;

    __HAL_RCC_DMA2_CLK_ENABLE();

    /* SPI4_TX is DMA2 Stream4 channel 5; Stream1 is taken by TIM8_UP. */
    dma_setup(&s_hdma_tx, DMA2_Stream4, DMA_CHANNEL_5, DMA_MEMORY_TO_PERIPH);
    /* SPI4_RX is DMA2 Stream0 channel 4. */
    dma_setup(&s_hdma_rx, DMA2_Stream0, DMA_CHANNEL_4, DMA_PERIPH_TO_MEMORY);

    /*
     * A circular transfer of length 1 reloads the same address indefinitely,
     * which is exactly the behaviour wanted: every poll re-sends whatever
     * s_tx_word currently holds.
     */
    if (HAL_DMA_Start(&s_hdma_rx, (uint32_t)&hspi->Instance->DR,
                      (uint32_t)&s_rx_sink, 1U) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_DMA_Start(&s_hdma_tx, (uint32_t)&s_tx_word,
                      (uint32_t)&hspi->Instance->DR, 1U) != HAL_OK)
    {
        Error_Handler();
    }

    /* Hand the data register over to DMA, then enable the peripheral. */
    SET_BIT(hspi->Instance->CR2, SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
    __HAL_SPI_ENABLE(hspi);
}

/**
 * @brief Publish a word for the next and all subsequent polls.
 *
 * A naturally aligned 16-bit store is atomic on Cortex-M, so the DMA can
 * never read a half-updated value.
 */
static void publish(uint16_t word)
{
    s_tx_word = word;
    g_tx_count++;
}

void yask_odrive_update(SPI_HandleTypeDef *hspi, const yask_frame_t *f)
{
    (void)hspi;
    uint16_t angle14 = (uint16_t)((f->single_turn >> YASK_ODRIVE_SHIFT)
                                 & (YASK_ODRIVE_CPR - 1U));
#if YASK_ODRIVE_INVERT
    angle14 = (uint16_t)((YASK_ODRIVE_CPR - 1U) - angle14);
#endif
    publish(yask_odrive_word(angle14, 0U));
}

void yask_odrive_set_error(SPI_HandleTypeDef *hspi)
{
    (void)hspi;
    publish(yask_odrive_word(0U, 1U));
}

void yask_odrive_nss_isr(SPI_HandleTypeDef *hspi)
{
    /*
     * Nothing to do: DMA already holds the current word ready.  The count is
     * kept because it is the one unambiguous sign that the controller is
     * actually polling this pin.
     *
     * Note for anyone tempted to detect mis-clocked frames here: it does not
     * work.  Inverter noise adds clock edges (~10% of CS windows take 17-27
     * clocks instead of 16, measured on the analyser), but checking
     * SPI4->SR for OVR/RXNE/FRLVL on the CS edge caught 0 of 199780 -- the
     * RX DMA drains the FIFO fast enough that the register looks clean again
     * by the next edge.  The corruption is inside the shift register during
     * the transfer, and the peripheral offers no flag for it.
     */
    (void)hspi;
    g_nss_count++;
}

void yask_odrive_spi_isr(SPI_HandleTypeDef *hspi)
{
    /* Retained for the header's sake; DMA services transfers now. */
    (void)hspi;
}

#endif /* !YASK_HOST_TEST */

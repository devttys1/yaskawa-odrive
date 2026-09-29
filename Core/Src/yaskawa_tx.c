/**
 * @file    yaskawa_tx.c
 * @brief   Request-frame transmitter: TIM1 update -> DMA -> GPIOE->BSRR.
 *
 * A pre-built table holds one 32-bit BSRR word per Manchester symbol.  Each
 * timer update event moves one word to the GPIO, so the line toggles with
 * cycle-exact 125 ns timing and zero CPU involvement.
 *
 * Request frame, 60 raw bits before Manchester encoding:
 *   sync preamble  25 bits  0101010101010101010101010
 *   HDLC flag       8 bits  0x7E
 *   payload        19 bits  0xFFFF with HDLC bit stuffing
 *   HDLC flag       8 bits  0x7E
 */

#include "yaskawa_enc.h"
#include "yaskawa_hw.h"

/* ── Frame definition ────────────────────────────────────────────────────── */

#define YASK_TX_RAW_BITS   60U
#define YASK_TX_SYMBOLS    (YASK_TX_RAW_BITS * 2U)          /* 120 */
#define YASK_TX_BUF_LEN    (YASK_TX_SYMBOLS + 2U)           /* 122 */

static const uint8_t tx_raw_bits[YASK_TX_RAW_BITS] =
{
    /* Sync preamble -- 25 alternating bits */
    0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,1,0,1,0,1, 0,1,0,1,0,

    /* HDLC start flag 0x7E */
    0,1,1,1,1,1,1,0,

    /* Payload 0xFFFF after bit stuffing: a 0 inserted after each run of 5 */
    1,1,1,1,1, 0, 1,1,1,1,1, 0, 1,1,1,1,1, 0, 1,

    /* HDLC end flag 0x7E */
    0,1,1,1,1,1,1,0,
};

/*
 * BSRR words, one per Manchester symbol.  DMA reads this buffer; on the F7 it
 * must live in a DMA-reachable region.  DTCM (the default .bss on this part)
 * is NOT reachable by DMA2, so the buffer is placed in SRAM1 via a linker
 * section -- see STM32F767ZITX_FLASH.ld.
 */
__attribute__((section(".dma_buffer")))
static uint32_t tx_bsrr[YASK_TX_BUF_LEN];

/* ── Build the toggle table ──────────────────────────────────────────────── */

void yask_tx_build_table(void)
{
    const uint32_t set = (uint32_t)YASK_TX_Pin;         /* low half  -> HIGH */
    const uint32_t clr = (uint32_t)YASK_TX_Pin << 16;   /* high half -> LOW  */

    uint32_t i = 0;

    for (uint32_t b = 0; b < YASK_TX_RAW_BITS; b++)
    {
        if (tx_raw_bits[b] == 0U)
        {
            tx_bsrr[i++] = clr;   /* Manchester 0: LOW then HIGH */
            tx_bsrr[i++] = set;
        }
        else
        {
            tx_bsrr[i++] = set;   /* Manchester 1: HIGH then LOW */
            tx_bsrr[i++] = clr;
        }
    }

    /* Leave the line idle low after the last symbol. */
    tx_bsrr[i++] = clr;
    tx_bsrr[i++] = clr;
}

/* ── Transmit ────────────────────────────────────────────────────────────── */

/**
 * @brief Send one request frame (blocking, ~15 us on the wire).
 *
 * DE is left asserted on return.  The caller drops it only after the receive
 * path is armed, so that no response edges are lost during RX setup.
 */
void yask_tx_send(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{
    HAL_GPIO_WritePin(YASK_DE_GPIO_Port, YASK_DE_Pin, GPIO_PIN_SET);

    /* Park the line low before the first symbol so the preamble edge is clean. */
    YASK_TX_GPIO_Port->BSRR = (uint32_t)YASK_TX_Pin << 16;

    /*
     * Abort first so the handle is READY: nothing here takes the transfer
     * interrupt, so a handle left BUSY makes HAL_DMA_Start() return HAL_BUSY
     * and transmit nothing.
     */
    (void)HAL_DMA_Abort(hdma);

    if (HAL_DMA_Start(hdma,
                      (uint32_t)tx_bsrr,
                      (uint32_t)&YASK_TX_GPIO_Port->BSRR,
                      YASK_TX_BUF_LEN) != HAL_OK)
    {
        return;
    }

    /*
     * Restore the symbol-clock period.  With YASK_RX_METHOD_CAPTURE the
     * receiver borrows this same timer and opens ARR up to 0xFFFF so its
     * capture timestamps do not wrap mid-frame, so the transmitter cannot
     * assume MX_TIM1_Init()'s value still stands.
     */
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
    htim->Instance->CCER &= ~TIM_CCER_CC2E;
    htim->Instance->ARR   = YASK_TX_TIM_ARR;

    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_ENABLE(htim);

    /*
     * Spin on NDTR rather than calling HAL_DMA_PollForTransfer().
     *
     * The HAL routine costs about 62 us beyond the last symbol -- far longer
     * than the 15 us frame itself.  The encoder starts replying roughly 17 us
     * after the request ends, so that overhead made us arm the receiver about
     * 14 us into the response: the HDLC start flag was already gone and the
     * decode failed with NO_FLAG even though the capture was otherwise clean.
     *
     * Reading the stream's remaining-count register directly costs a few
     * hundred nanoseconds and leaves the whole inter-frame gap available.
     * The loop is bounded so a stalled stream cannot hang the caller.
     */
    uint32_t guard = 0;
    while (__HAL_DMA_GET_COUNTER(hdma) != 0U && ++guard < 100000U)
    {
        __asm volatile ("nop");
    }

    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
}

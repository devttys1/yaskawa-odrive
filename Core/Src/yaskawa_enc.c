/**
 * @file    yaskawa_enc.c
 * @brief   Top-level request/response sequencing.
 */

#include "yaskawa_enc.h"
#include "yaskawa_hw.h"

/* Provided by the transport layer. */
void yask_tx_build_table(void);
void yask_tx_send(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void yask_rx_arm_sampling(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma);
void yask_rx_arm_capture(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma,
                         uint32_t channel);
yask_status_t yask_rx_decode(yask_frame_t *out);
#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)
yask_status_t yask_rx_decode_edges(uint32_t nedges, yask_frame_t *out);
#endif

/* CubeMX-generated handles. */
extern TIM_HandleTypeDef htim1;   /* TX bit clock          */
extern TIM_HandleTypeDef htim8;   /* RX sample clock       */
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim8_up;
#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)
extern DMA_HandleTypeDef hdma_tim1_ch2;
#endif

void yask_init(void)
{
    yask_tx_build_table();

    /* Idle state: receiver enabled, transmit line low. */
    HAL_GPIO_WritePin(YASK_DE_GPIO_Port, YASK_DE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(YASK_TX_GPIO_Port, YASK_TX_Pin, GPIO_PIN_RESET);
}

/**
 * @brief One full request/response exchange.
 *
 * Ordering matters: the encoder starts replying only a few microseconds
 * after the request ends, so the receive path is armed *before* DE is
 * released.  Arming first costs nothing -- the line is still driven by us,
 * so the leading samples are just our own idle level.
 */
yask_status_t yask_read(yask_frame_t *out)
{
    /*
     * 1. Open the capture window BEFORE transmitting.
     *
     * Arming afterwards always loses the first bits of the response: even
     * with the fast NDTR spin in yask_tx_send(), setting up the stream and
     * starting the sample clock costs a few microseconds, and the encoder
     * begins replying ~17 us after the request ends.  Measured that way the
     * capture started 6 bits into the frame, so the HDLC start flag was
     * already gone and every decode failed with NO_FLAG.
     *
     * Starting the sampler first costs nothing: the window simply records
     * our own transmission and the idle gap ahead of the response, and the
     * decoder skips them when it hunts for the flag.  The window is sized
     * (YASK_RX_SAMPLES / 36 MHz) to span the request, the gap and the reply.
     */
#if (YASK_RX_METHOD == YASK_RX_METHOD_SAMPLE)
    yask_rx_arm_sampling(&htim8, &hdma_tim8_up);
#endif

    /* 2. Transmit the request (DE stays asserted on return). */
    yask_tx_send(&htim1, &hdma_tim1_up);

    /*
     * 3. Release the bus so the encoder can drive it.
     *
     * The original order armed the capture first, on the reasoning that no
     * response edge should be missed during setup.  That is wrong here: the
     * capture window is finite (YASK_RX_SAMPLES / 36 MHz), and arming first
     * spends it watching our own transceiver still in transmit mode -- the
     * window closed before the encoder began replying, so every sample held
     * the idle level and the decode reported TIMEOUT.
     *
     * The encoder does not answer for ~30 us, which is far longer than the
     * few microseconds between these two statements, so there is no risk of
     * losing the start of the frame this way.
     */
    HAL_GPIO_WritePin(YASK_DE_GPIO_Port, YASK_DE_Pin, GPIO_PIN_RESET);

    /* 4. The input-capture back-end cannot be armed before the transmission,
       because it shares TIM1 with the transmitter. */
#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)
    yask_rx_arm_capture(&htim1, &hdma_tim1_ch2, TIM_CHANNEL_2);
#endif

    /*
     * 5. Wait for the capture window to fill.
     *
     * The window is YASK_RX_SAMPLES / 36 MHz -- about 171 us.  Spin on the
     * DMA's remaining-count register rather than HAL_GetTick(): the tick is
     * 1 ms, so waiting on it cost ~2 ms per read and capped polling near
     * 500 Hz.  At 0.5 revolutions per sample interval the single-turn
     * position aliases (it cannot represent more than half a turn between
     * reads), and a drill-driven shaft at ~320 RPM exceeded that at 10 Hz.
     * Spinning on NDTR brings a read down to roughly the window length, so
     * the poll rate is limited by the link rather than the RTOS tick.
     */
#if (YASK_RX_METHOD == YASK_RX_METHOD_SAMPLE)
    uint32_t guard = 0;
    while (__HAL_DMA_GET_COUNTER(&hdma_tim8_up) != 0U && ++guard < 200000U)
    {
        __asm volatile ("nop");
    }

    /* Quiesce the sample clock completely: stopping the counter alone
       leaves TIM_DMA_UPDATE set, which would re-trigger on the next arm. */
    __HAL_TIM_DISABLE(&htim8);
    __HAL_TIM_DISABLE_DMA(&htim8, TIM_DMA_UPDATE);

    /* 6. Recover Manchester -> HDLC -> CRC -> struct. */
    return yask_rx_decode(out);
#else
    /*
     * Input capture produces one entry per edge, not a fixed-length window,
     * so there is no "buffer full" to wait for -- a short frame simply
     * leaves the stream part-drained.  Wait until the edges stop arriving:
     * poll NDTR and finish once it has held still for longer than the
     * longest legal gap between edges (two symbols, 250 ns).
     *
     * The guard is a hard ceiling in case the encoder never answers.
     */
    uint32_t guard    = 0;
    uint32_t prev     = __HAL_DMA_GET_COUNTER(&hdma_tim1_ch2);
    uint32_t stable   = 0;
    const uint32_t kStableSpins = 2000U;   /* comfortably > 2 symbol periods */

    while (++guard < 200000U)
    {
        uint32_t now = __HAL_DMA_GET_COUNTER(&hdma_tim1_ch2);
        if (now != prev)
        {
            prev   = now;
            stable = 0;
            if (now == 0U) break;          /* buffer full: nothing more fits */
        }
        else if (++stable > kStableSpins)
        {
            break;                         /* line went quiet */
        }
        __asm volatile ("nop");
    }

    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC2);

    /*
     * Entries actually written = capacity - remaining.  Passing the capacity
     * instead walks the zeroed tail and decodes garbage after the frame.
     */
    const uint32_t cap    = YASK_RX_MAX_BITS * 2U;
    const uint32_t remain = __HAL_DMA_GET_COUNTER(&hdma_tim1_ch2);
    const uint32_t nedges = (remain <= cap) ? (cap - remain) : 0U;

    /* 6. Recover Manchester -> HDLC -> CRC -> struct. */
    return yask_rx_decode_edges(nedges, out);
#endif
}

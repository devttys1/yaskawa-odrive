/**
 * @file    yaskawa_rx.c
 * @brief   Response capture and Manchester recovery.
 *
 * Two capture back-ends are provided, selected by YASK_RX_METHOD:
 *
 *   YASK_RX_METHOD_SAMPLE (default)
 *       TIM8 update at 36 MHz triggers DMA reads of GPIOE->IDR into a sample
 *       buffer.  Deterministic, immune to capture overrun, and easy to dump
 *       for offline analysis.  Costs 4 KB of SRAM per frame.
 *
 *   YASK_RX_METHOD_CAPTURE
 *       TIM1_CH2 in both-edge input-capture mode, DMA copies CCR2 on every
 *       transition.  Compact (edge timestamps only) and self-timing, but the
 *       CCR must be serviced within one symbol or an overcapture is lost.
 *
 * Both produce the same thing: an array of Manchester-decoded data bits,
 * which is then handed to the shared HDLC/CRC decoder in yaskawa_decode.c.
 */

#include "yaskawa_enc.h"
#include "yaskawa_hw.h"
#include <string.h>

yask_status_t yask_decode_bitstream(const uint8_t *raw, uint32_t nraw,
                                    yask_frame_t *out);

/* ── Capture buffers (SRAM1: DMA2 cannot reach DTCM) ─────────────────────── */

/* Only the selected back-end's buffer is allocated. */
#if (YASK_RX_METHOD == YASK_RX_METHOD_SAMPLE)
__attribute__((section(".dma_buffer")))
static uint16_t rx_samples[YASK_RX_SAMPLES];
#else
__attribute__((section(".dma_buffer")))
static uint16_t rx_edges[YASK_RX_MAX_BITS * 2U];
#endif

/* Manchester-decoded data bits, one byte per bit. */
static uint8_t  rx_bits[YASK_RX_MAX_BITS];

/* ── Back-end 1: oversampled GPIO ────────────────────────────────────────── */
#if (YASK_RX_METHOD == YASK_RX_METHOD_SAMPLE)

/**
 * @brief Arm the IDR sampling DMA.  Returns immediately; the timer free-runs.
 */
void yask_rx_arm_sampling(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma)
{
    /*
     * Order matters here.  The timer must be fully quiesced before the DMA
     * stream is enabled, otherwise a request left over from the previous
     * cycle fires the moment HAL_DMA_Start() enables the stream and the
     * whole buffer drains in 57 us -- while the line is still idle and DE
     * is still asserted, so every sample captures our own idle level.
     *
     * Stop the counter, drop the DMA request line, then clear the pending
     * update flag that stopping the counter may itself have raised.
     */
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    /*
     * Return the handle to READY.  Nothing in the receive path polls or
     * takes the transfer-complete interrupt, so after the first capture the
     * handle is left in HAL_DMA_STATE_BUSY -- and HAL_DMA_Start() then
     * refuses to arm, returning HAL_BUSY, which is easy to miss because the
     * buffer still looks plausible (it is the memset below, not captured
     * data).  Aborting first makes each cycle independent of the last.
     */
    (void)HAL_DMA_Abort(hdma);

    /*
     * No memset here: DMA overwrites every slot it fills, and clearing the
     * buffer costs tens of microseconds -- a significant part of the window
     * we are trying to capture.  Slots beyond NDTR keep the previous
     * cycle's data, which the decoder ignores because it stops at the
     * frame's end flag.
     */
    if (HAL_DMA_Start(hdma,
                      (uint32_t)&YASK_RX_GPIO_Port->IDR,
                      (uint32_t)rx_samples,
                      YASK_RX_SAMPLES) != HAL_OK)
    {
        return;   /* leaves the buffer flat; yask_rx_decode reports TIMEOUT */
    }

    /*
     * Setting the counter close to the reload makes the first sample land
     * within one tick of the timer being enabled, rather than up to a full
     * period later.
     */
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
    __HAL_TIM_ENABLE(htim);
}

/**
 * @brief Recover data bits from the oversampled line state.
 *
 * Manchester carries a guaranteed transition in the centre of every bit
 * period, so we lock onto edges rather than counting samples:
 *
 *   1. Skip the idle run before the first edge.
 *   2. From each mid-bit edge, step forward one full bit period (2 symbols)
 *      to land on the next mid-bit edge, correcting the phase whenever the
 *      observed edge sits slightly off the predicted position.
 *   3. The direction of the mid-bit transition is the data bit:
 *      rising = 0, falling = 1  (matching the transmit convention).
 *
 * @return number of bits recovered.
 */
static uint32_t manchester_decode_samples(uint32_t *first_edge_out)
{
    const uint32_t mask = 1U << YASK_RX_BITPOS;

    /* Bit period in samples, Q8 fixed point: 2 symbols * 4.5 samples. */
    const int32_t  bit_q8  = (int32_t)YASK_RX_SPS_Q8 * 2;
    const int32_t  half_q8 = bit_q8 / 2;

    uint32_t i = 1;
    uint8_t  prev = (rx_samples[0] & mask) ? 1U : 0U;

    /*
     * 1. Find the first transition -- the start of the preamble.
     *
     * The encoder answers ~30 us after the request, which at 36 MS/s is
     * ~1080 idle samples ahead of any data: about a third of the capture,
     * and on the critical path because the ODrive re-reads the last
     * published angle until this decode finishes.
     *
     * The idle run is a constant level, so scan it a word at a time.  Build
     * a mask of the idle level replicated into all 16 bit positions of a
     * uint32 pair and step 2 samples per compare; any word that differs
     * contains the edge, which the byte loop below then pins down exactly.
     * Same result as the sample-by-sample scan, ~2x fewer compares over the
     * idle run.
     */
    {
        const uint16_t idle_word = prev ? (uint16_t)mask : 0U;
        const uint32_t idle_pair = ((uint32_t)idle_word << 16) | idle_word;
        const uint32_t pair_mask = ((uint32_t)mask << 16) | mask;

        /* Only safe while both samples of the pair are in range. */
        while (i + 1U < YASK_RX_SAMPLES)
        {
            uint32_t pair;
            memcpy(&pair, &rx_samples[i], sizeof(pair));
            if ((pair & pair_mask) != idle_pair) break;
            i += 2U;
        }
    }

    while (i < YASK_RX_SAMPLES)
    {
        uint8_t cur = (rx_samples[i] & mask) ? 1U : 0U;
        if (cur != prev) break;
        prev = cur;
        i++;
    }
    if (i >= YASK_RX_SAMPLES)
    {
        return 0;                       /* line never moved */
    }
    if (first_edge_out) *first_edge_out = i;

    /*
     * 2. Walk bit by bit.  pos_q8 tracks the position of the last mid-bit
     *    edge; each iteration steps one full bit period forward, searches a
     *    window around the prediction for the real transition, and re-centres
     *    on what it finds.  That absorbs the fractional 4.5 samples/symbol
     *    rate and any encoder clock drift.
     *
     *    The first transition found above is the *leading* edge of the first
     *    symbol -- a bit boundary, not a mid-bit edge.  The first mid-bit
     *    edge is therefore only half a bit period away, so seed pos_q8 half a
     *    period behind it to make the first full step land correctly.
     */
    int32_t  pos_q8 = (int32_t)i * 256 - (int32_t)YASK_RX_SPS_Q8;
    uint32_t n = 0;

    while (n < YASK_RX_MAX_BITS)
    {
        int32_t  target = pos_q8 + bit_q8;
        uint32_t centre = (uint32_t)((target + 128) / 256);

        if (centre + 2U >= YASK_RX_SAMPLES)
        {
            break;                      /* ran off the end of the capture */
        }

        /* Search +/- half a bit for the mid-bit transition. */
        int32_t  best   = 0;
        bool     found  = false;
        uint32_t window = (uint32_t)(half_q8 / 256) + 1U;

        for (uint32_t d = 0; d <= window; d++)
        {
            for (int32_t sgn = -1; sgn <= 1; sgn += 2)
            {
                int32_t idx = (int32_t)centre + sgn * (int32_t)d;
                if (idx < 1 || (uint32_t)(idx + 1) >= YASK_RX_SAMPLES) continue;

                uint8_t a = (rx_samples[idx - 1] & mask) ? 1U : 0U;
                uint8_t b = (rx_samples[idx]     & mask) ? 1U : 0U;
                if (a != b)
                {
                    best  = idx;
                    found = true;
                    break;
                }
                if (d == 0) break;      /* only one candidate at distance 0 */
            }
            if (found) break;
        }

        if (!found)
        {
            break;                      /* transition vanished -- end of frame */
        }

        /* 3. Falling edge encodes a 1, rising edge a 0. */
        uint8_t before = (rx_samples[best - 1] & mask) ? 1U : 0U;
        rx_bits[n++] = before ? 1U : 0U;

        pos_q8 = best * 256;            /* re-centre on the observed edge */
    }

    return n;
}

#endif /* YASK_RX_METHOD_SAMPLE */

/* ── Back-end 2: timer input capture ─────────────────────────────────────── */
#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)

/**
 * @brief Arm both-edge input capture with DMA into rx_edges[].
 */
void yask_rx_arm_capture(TIM_HandleTypeDef *htim, DMA_HandleTypeDef *hdma,
                         uint32_t channel)
{
    (void)channel;

    /*
     * TIM1 does double duty: it is the 8 MHz TX symbol clock (ARR=26, so it
     * wraps every 125 ns) and the only timer whose channel reaches PE11
     * (TIM1_CH2, AF1).  Capture timestamps need a counter that does not wrap
     * within a frame -- ~64 us is 13824 ticks at 216 MHz -- so the period
     * has to be opened up here and restored by the transmitter next cycle.
     *
     * Safe to do at this point: yask_tx_send() has already clocked the last
     * request bit out, and the encoder does not answer for ~30 us.
     */
    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);

    (void)HAL_DMA_Abort(hdma);       /* see the note in arm_sampling() */

    /*
     * No memset: the DMA overwrites each slot it fills and the caller passes
     * the exact edge count to the decoder, so stale tail entries are never
     * read.  Clearing 1 KB here would cost more than the capture itself.
     */

    /* Free-running for the duration of the frame. */
    htim->Instance->ARR = 0xFFFFU;
    htim->Instance->CNT = 0U;

    /*
     * Both-edge capture on CH2: Manchester carries data in the direction of
     * every transition, so both edges matter.
     */
    htim->Instance->CCER &= ~(TIM_CCER_CC2E | TIM_CCER_CC2P | TIM_CCER_CC2NP);
    htim->Instance->CCMR1 =
        (htim->Instance->CCMR1 & ~(TIM_CCMR1_CC2S | TIM_CCMR1_IC2F
                                   | TIM_CCMR1_IC2PSC))
        | (1U << TIM_CCMR1_CC2S_Pos);        /* CC2 = input, mapped to TI2 */
    htim->Instance->CCER |= TIM_CCER_CC2P | TIM_CCER_CC2NP;  /* both edges */

    if (HAL_DMA_Start(hdma,
                      (uint32_t)&htim->Instance->CCR2,
                      (uint32_t)rx_edges,
                      YASK_RX_MAX_BITS * 2U) != HAL_OK)
    {
        return;
    }

    /*
     * Enable the channel first, then clear: arming CC2E can itself latch the
     * line's current level as a capture, and the CC2IF that comes with it
     * makes the DMA fire immediately.  Reading CCR2 before CC2E left that
     * stale value in place, which showed up as a duplicated first timestamp
     * (a zero-length gap the decoder cannot classify).
     */
    htim->Instance->CCER |= TIM_CCER_CC2E;
    (void)htim->Instance->CCR2;      /* discard whatever arming latched */
    htim->Instance->SR = 0U;
    __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
    __HAL_TIM_ENABLE(htim);
}

/**
 * @brief Recover data bits from edge timestamps.
 *
 * In Manchester the gap between consecutive edges is either one symbol
 * (125 ns) or two (250 ns).  A long gap always lands on a mid-bit edge; a
 * pair of short gaps spans one bit boundary plus one mid-bit edge.  Tracking
 * that alternation recovers the data without an explicit clock.
 *
 * @param nedges number of valid entries in rx_edges[].
 * @return number of bits recovered.
 */
static uint32_t manchester_decode_edges(uint32_t nedges)
{
    if (nedges < 4U) return 0;

    /* One symbol in timer ticks, and the 1.5-symbol discriminator. */
    const uint32_t sym      = YASK_TIMER_CLK_HZ / YASK_BAUD_HZ;   /* 27 */
    const uint32_t sym_1p5  = sym + sym / 2U;

    /*
     * Drop the priming artefacts at the head of the buffer.
     *
     * Enabling CC2E latches the line's present level as a capture and raises
     * CC2IF with it, so the DMA writes one or two entries before the first
     * real transition -- seen as a leading 0 and a repeated timestamp.  A
     * zero-length gap is not a legal Manchester symbol, so skip any entry
     * that does not advance the counter rather than feeding the phase
     * tracker a delta it cannot classify.
     */
    const uint16_t *e = rx_edges;

    /* Repeated timestamp: the same capture written twice. */
    while (nedges >= 2U && e[1] == e[0])
    {
        e++;
        nedges--;
    }

    /*
     * A capture latched by arming rather than by a transition sits at (or
     * very near) the counter start, an implausibly short gap ahead of the
     * first real edge.  It is not a repeat, so the test above misses it, and
     * leaving it in shifts the whole mid-bit/boundary alternation by one
     * edge -- which decodes to a full frame of plausible-looking garbage.
     * Require the opening gap to be at least a symbol.
     */
    while (nedges >= 2U && (uint16_t)(e[1] - e[0]) < (sym - sym / 4U))
    {
        e++;
        nedges--;
    }

    if (nedges < 4U) return 0;


    uint32_t n     = 0;
    uint8_t  level;

    /*
     * e[0] is the very first transition out of the idle line state.
     * That edge is the *leading* edge of the first symbol -- a bit boundary,
     * not a mid-bit transition -- so the first mid-bit edge is the next one
     * and the phase starts at "expecting mid-bit".
     */
    bool mid = false;

    /* The line idles low, so the first captured edge is rising. */
    level = 1U;

    /* Classify the first edge itself before walking the deltas. */
    for (uint32_t i = 1; i < nedges && n < YASK_RX_MAX_BITS; i++)
    {
        uint16_t delta = (uint16_t)(e[i] - e[i - 1]);

        level ^= 1U;          /* every capture is a transition */

        if (delta > sym_1p5)
        {
            /* Long gap: this edge is always a mid-bit transition. */
            rx_bits[n++] = level ? 0U : 1U;
            mid = true;
        }
        else
        {
            /* Short gap: mid-bit and boundary edges alternate. */
            if (mid)
            {
                mid = false;              /* this one is a bit boundary */
            }
            else
            {
                rx_bits[n++] = level ? 0U : 1U;
                mid = true;
            }
        }
    }

    return n;
}

#endif /* YASK_RX_METHOD_CAPTURE */

/* ── Public entry point ──────────────────────────────────────────────────── */

/**
 * @brief Decode whatever the armed back-end captured into a frame struct.
 */
yask_status_t yask_rx_decode(yask_frame_t *out)
{
    uint32_t nbits;

#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)
    /* Kept for callers that do not know the edge count; decodes the whole
       buffer, which is only correct if it happens to be full. */
    nbits = manchester_decode_edges(YASK_RX_MAX_BITS * 2U);
#else
    uint32_t first_edge = 0;
    nbits = manchester_decode_samples(&first_edge);
    (void)first_edge;
#endif

    if (nbits == 0U)
    {
        return YASK_ERR_TIMEOUT;
    }

    return yask_decode_bitstream(rx_bits, nbits, out);
}

#if (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE)
/**
 * @brief Decode exactly @p nedges captured timestamps.
 *
 * The DMA stops wherever the frame ended, so the caller must say how many
 * entries are valid.  Decoding the full buffer instead walks the zeroed
 * tail, where every delta is 0 and the phase tracker produces noise.
 */
yask_status_t yask_rx_decode_edges(uint32_t nedges, yask_frame_t *out)
{
    uint32_t nbits = manchester_decode_edges(nedges);

    if (nbits == 0U)
    {
        return YASK_ERR_TIMEOUT;
    }

    return yask_decode_bitstream(rx_bits, nbits, out);
}
#endif

/** Expose the raw sample buffer so main() can dump it over the VCP. */
const uint16_t *yask_rx_raw_samples(uint32_t *count)
{
#if (YASK_RX_METHOD == YASK_RX_METHOD_SAMPLE)
    if (count) *count = YASK_RX_SAMPLES;
    return rx_samples;
#else
    /* The input-capture back-end stores edge timestamps, not line samples. */
    if (count) *count = 0U;
    return NULL;
#endif
}

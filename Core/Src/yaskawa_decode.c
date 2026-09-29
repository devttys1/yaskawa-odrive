/**
 * @file    yaskawa_decode.c
 * @brief   Manchester / HDLC / CRC decoding for the Yaskawa encoder.
 *
 * This file is deliberately free of HAL calls so it can be unit-tested on a
 * host PC against the logic-analyser captures (see tools/).
 */

#include "yaskawa_enc.h"
#include <string.h>

/* ── Little helper: gather @p len bits LSB-first into an integer ─────────── */

static uint32_t bits_le(const uint8_t *bits, uint32_t off, uint32_t len)
{
    uint32_t v = 0;
    for (uint32_t i = 0; i < len; i++)
    {
        v |= (uint32_t)(bits[off + i] & 1U) << i;
    }
    return v;
}

/* ── CRC-16/X.25 ─────────────────────────────────────────────────────────── */

/*
 * Polynomial 0x1021, init 0xFFFF, reflected input and output, final XOR
 * 0xFFFF.  Because the data is reflected we run the reversed polynomial
 * 0x8408 and shift right, which is the usual bitwise formulation.
 *
 * The CRC covers payload bits[0:96] -- the 12 bytes preceding the CRC field.
 * Each byte is assembled LSB-first, matching the on-wire bit order.
 */
uint16_t yask_crc16_x25(const uint8_t *bits)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t byte_idx = 0; byte_idx < YASK_CRC_DATA_BYTES; byte_idx++)
    {
        uint16_t byte = (uint16_t)bits_le(bits, byte_idx * 8U, 8U);

        crc ^= byte;
        for (uint32_t b = 0; b < 8U; b++)
        {
            if (crc & 1U)
            {
                crc = (uint16_t)((crc >> 1) ^ 0x8408U);
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return (uint16_t)(crc ^ 0xFFFFU);
}

/* ── HDLC framing ────────────────────────────────────────────────────────── */

/**
 * @brief Locate the HDLC start flag, then bit-unstuff until the end flag.
 *
 * The start flag is 0x7E = 01111110.  We scan for six consecutive 1s and
 * consume the trailing 0.  After that, every 0 immediately following five
 * 1s is a stuffed bit and is discarded; six 1s marks the end flag.
 *
 * @param raw    Manchester-decoded bit stream (0/1 per element).
 * @param nraw   Number of valid entries in @p raw.
 * @param out    Destination for unstuffed payload bits.
 * @param maxout Capacity of @p out.
 * @return Number of payload bits written.
 */
static uint32_t hdlc_unstuff(const uint8_t *raw, uint32_t nraw,
                             uint8_t *out, uint32_t maxout)
{
    uint32_t n     = 0;
    uint32_t ones  = 0;
    bool     started = false;

    for (uint32_t i = 0; i < nraw; i++)
    {
        uint8_t b = raw[i] & 1U;

        if (!started)
        {
            /* Hunt for the opening flag. */
            if (b)
            {
                if (++ones == 6U)
                {
                    /* Consume the flag's trailing 0 if it is present. */
                    if ((i + 1U) < nraw && raw[i + 1U] == 0U)
                    {
                        i++;
                    }
                    started = true;
                    ones    = 0;
                }
            }
            else
            {
                ones = 0;
            }
            continue;
        }

        if (b == 0U)
        {
            if (ones == 5U)
            {
                ones = 0;      /* stuffed zero -- drop it */
                continue;
            }
            ones = 0;
        }
        else
        {
            if (++ones == 6U)
            {
                break;         /* closing flag -- frame complete */
            }
        }

        if (n < maxout)
        {
            out[n++] = b;
        }
        else
        {
            break;             /* caller's buffer is full */
        }
    }

    return n;
}

/* ── Payload -> struct ───────────────────────────────────────────────────── */

yask_status_t yask_decode_payload(const uint8_t *bits, uint32_t nbits,
                                  yask_frame_t *out)
{
    if (nbits < YASK_PAYLOAD_BITS)
    {
        return YASK_ERR_SHORT;
    }

    memset(out, 0, sizeof(*out));

    out->single_turn = bits_le(bits, YASK_OFF_SINGLE_TURN, YASK_ST_BITS);
    out->multi_turn  = (uint8_t) bits_le(bits, YASK_OFF_MULTI_TURN,  4U);
    out->rot3        = (uint8_t) bits_le(bits, YASK_OFF_ROT3,        3U);
    out->hall        = (uint8_t) bits_le(bits, YASK_OFF_HALL,        3U);
    out->ts_slow     = (uint8_t) bits_le(bits, YASK_OFF_TS_SLOW,     8U);
    out->ts_fast     = (uint16_t)bits_le(bits, YASK_OFF_TS_FAST,    16U);
    out->status_hi   = (uint8_t) bits_le(bits, YASK_OFF_STATUS_HI,   3U);
    out->id_ff       = (uint8_t) bits_le(bits, YASK_OFF_ID_FF,       8U);
    out->id_latched  = (uint16_t)bits_le(bits, YASK_OFF_ID_LATCHED,  9U);

    out->raw40 = (uint16_t)bits_le(bits, YASK_OFF_RAW40, 16U);

    out->unindexed    = bits[YASK_OFF_UNINDEXED]  != 0U;
    out->battery_fail = bits[YASK_OFF_BATT_FAIL]  != 0U;
    out->comp_valid   = bits[YASK_OFF_COMP_VALID] == 0U;   /* 0 == valid */

    out->crc_rx   = (uint16_t)bits_le(bits, YASK_OFF_CRC, 16U);
    out->crc_calc = yask_crc16_x25(bits);
    out->crc_ok   = (out->crc_rx == out->crc_calc);

    return out->crc_ok ? YASK_OK : YASK_ERR_CRC;
}

/**
 * @brief Full decode of a raw Manchester-decoded bit stream.
 *
 * Kept separate from yask_decode_payload() so the host-side test harness can
 * feed either representation.
 */
yask_status_t yask_decode_bitstream(const uint8_t *raw, uint32_t nraw,
                                    yask_frame_t *out)
{
    uint8_t  payload[YASK_PAYLOAD_BITS + 8U];
    uint32_t n = hdlc_unstuff(raw, nraw, payload, sizeof(payload));

    if (n == 0U)
    {
        return YASK_ERR_NO_FLAG;
    }
    if (n < YASK_PAYLOAD_BITS)
    {
        return YASK_ERR_SHORT;
    }

    return yask_decode_payload(payload, n, out);
}

/* ── Diagnostics ─────────────────────────────────────────────────────────── */

const char *yask_status_str(yask_status_t s)
{
    switch (s)
    {
        case YASK_OK:          return "OK";
        case YASK_ERR_NO_FLAG: return "NO_FLAG";
        case YASK_ERR_SHORT:   return "SHORT";
        case YASK_ERR_CRC:     return "CRC";
        case YASK_ERR_TIMEOUT: return "TIMEOUT";
        case YASK_ERR_OVERRUN: return "OVERRUN";
        default:               return "?";
    }
}

/* ── Absolute position tracking ──────────────────────────────────────────── */

void yask_track_reset(yask_tracker_t *t)
{
    t->position = 0;
    t->last_st  = 0;
    t->last_mt  = 0;
    t->primed   = false;
}

int64_t yask_track_update(yask_tracker_t *t, const yask_frame_t *f)
{
    /*
     * Track on single_turn, and use multi_turn only to catch whole
     * revolutions that single_turn could not have shown.
     *
     * An earlier version combined (multi_turn, single_turn) into one value
     * modulo 16 revolutions and took the shortest path.  That is wrong near
     * the zero crossing: multi_turn steps at about 290 degrees, not at the
     * single_turn wrap, so for roughly 70 degrees the two disagree about
     * which revolution the shaft is in.  Measured over a 23 027-frame drill
     * run, that produced 122 phantom steps of a full revolution -- one
     * reported -0.999 rev where the shaft had actually moved +0.0016 rev.
     * They happened to cancel over that run, so the final total was right,
     * but any velocity taken from consecutive samples was corrupted.
     *
     * single_turn alone is unambiguous while the shaft moves less than half
     * a revolution between reads; over the same run its largest step was
     * 3450 counts (0.026 rev), with no spurious jumps at all.  multi_turn
     * then only has to account for motion beyond that, which is why the
     * correction below subtracts the wrap single_turn has already shown.
     */
    if (!t->primed)
    {
        t->last_st = f->single_turn;
        t->last_mt = f->multi_turn;
        t->primed  = true;
        return t->position;
    }

    /* Fine motion: shortest path around the single-turn circle. */
    int32_t d = (int32_t)f->single_turn - (int32_t)t->last_st;
    if (d >  (int32_t)(YASK_ENCODER_RES / 2U)) d -= (int32_t)YASK_ENCODER_RES;
    if (d < -(int32_t)(YASK_ENCODER_RES / 2U)) d += (int32_t)YASK_ENCODER_RES;

    /*
     * multi_turn correction, applied only when single_turn cannot have told
     * the whole story.
     *
     * Because multi_turn steps near 290 degrees rather than at the wrap,
     * the two fields disagree about the revolution number for roughly 70
     * degrees either side of zero.  In that band single_turn can wrap with
     * no multi_turn step, or multi_turn can step with no wrap; treating
     * either as a missed revolution invents a full turn of motion that did
     * not happen -- the failure that produced 122 phantom steps across a
     * 23 027-frame run.
     *
     * single_turn is already unambiguous for anything under half a
     * revolution, so the correction is only consulted when the observed
     * step is close to that limit and a revolution really could have been
     * missed.  Below the threshold multi_turn is ignored entirely.
     */
    const int32_t near_limit = (int32_t)(YASK_ENCODER_RES / 2U)
                             - (int32_t)(YASK_ENCODER_RES / 8U);   /* 0.375 rev */
    int32_t missed = 0;

    if (d > near_limit || d < -near_limit)
    {
        int32_t dm = (int32_t)f->multi_turn - (int32_t)t->last_mt;
        if (dm >  (int32_t)(YASK_MT_MODULO / 2U)) dm -= (int32_t)YASK_MT_MODULO;
        if (dm < -(int32_t)(YASK_MT_MODULO / 2U)) dm += (int32_t)YASK_MT_MODULO;

        int32_t st_wrap = 0;
        if (d > 0 && f->single_turn < t->last_st) st_wrap =  1;
        if (d < 0 && f->single_turn > t->last_st) st_wrap = -1;

        missed = dm - st_wrap;
    }

    t->position += (int64_t)d + (int64_t)missed * (int64_t)YASK_ENCODER_RES;
    t->last_st   = f->single_turn;
    t->last_mt   = f->multi_turn;

    return t->position;
}

/* ── Glitch rejection ────────────────────────────────────────────────────── */

void yask_filter_reset(yask_filter_t *f)
{
    f->last_st  = 0;
    f->last_mt  = 0;
    f->run      = 0;
    f->primed   = false;
    f->rejected = 0;
    f->accepted = 0;
}

bool yask_filter_accept(yask_filter_t *f, const yask_frame_t *frame)
{
    if (!f->primed)
    {
        f->last_st = frame->single_turn;
        f->last_mt = frame->multi_turn;
        f->primed  = true;
        f->accepted++;
        return true;
    }

#if (YASK_MAX_STEP_COUNTS > 0U)
    /*
     * TODO: make this rate-aware.  The limit is counts-per-CALL, so its
     * effective rpm ceiling moves with however fast the caller happens to
     * loop -- and when it is set too low the filter rejects genuine motion,
     * holds a stale value, and then YASK_GLITCH_RUN_LIMIT releases the
     * accumulated jump anyway.  That is worse than not filtering, because
     * the jump reaches the controller with valid parity.
     *
     * The frame already carries ts_fast (a ~2 us down-counter), and
     * yask_interval_us() turns two of them into a real elapsed interval.
     * The threshold should be derived from that interval and a configured
     * maximum shaft speed, so it is correct whatever the loop rate.
     *
     * Shortest path around the single-turn circle, so a pair of readings
     * either side of the 131071/0 wrap reads as a small step rather than a
     * full revolution.
     */
    int32_t d = (int32_t)frame->single_turn - (int32_t)f->last_st;
    if (d >  (int32_t)(YASK_ENCODER_RES / 2U)) d -= (int32_t)YASK_ENCODER_RES;
    if (d < -(int32_t)(YASK_ENCODER_RES / 2U)) d += (int32_t)YASK_ENCODER_RES;

    if (d < 0) d = -d;

    if ((uint32_t)d > YASK_MAX_STEP_COUNTS)
    {
        /*
         * Implausible.  Hold the previous position -- but not indefinitely:
         * if the shaft genuinely accelerated, several frames in a row will
         * disagree with the stale value and the filter must give way.
         */
        if (++f->run < YASK_GLITCH_RUN_LIMIT)
        {
            f->rejected++;
            return false;
        }
    }
#endif

    f->run     = 0;
    f->last_st = frame->single_turn;
    f->last_mt = frame->multi_turn;
    f->accepted++;
    return true;
}

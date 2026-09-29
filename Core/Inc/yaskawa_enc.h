/**
 * @file    yaskawa_enc.h
 * @brief   Yaskawa 17-bit incremental encoder (UTSIH-B17CK) driver.
 *
 * Protocol summary
 * ----------------
 *   Physical  : RS-485 half duplex, Manchester encoded.
 *   Symbol rate: 8 MBaud  -> 4 Mbit/s data rate (2 symbols per data bit).
 *   Framing   : HDLC (0x7E flags, 5-ones bit stuffing).
 *   Payload   : 112 bits = 14 bytes, transmitted LSB-first within each field.
 *   Integrity : CRC-16/X.25 over payload bits[0:96].
 *
 * The master sends a 60-bit request frame; the encoder answers with a
 * response frame roughly 30 us later.  See yaskawa_tx.c / yaskawa_rx.c.
 */

#ifndef YASKAWA_ENC_H
#define YASKAWA_ENC_H

#include <stdint.h>
#include <stdbool.h>

/*
 * The decoding layer (yaskawa_decode.c) is pure C so it can be compiled and
 * regression-tested on a host PC against logic-analyser captures.  Only the
 * transport layer needs the HAL, so the include is skipped under the host
 * test build.
 */
#ifndef YASK_HOST_TEST
#include "stm32f7xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* ── Encoder characteristics ─────────────────────────────────────────────── */

#define YASK_ST_BITS          17U          /* single-turn resolution, bits    */
#define YASK_ENCODER_RES      131072U      /* 2^17 counts per revolution      */
#define YASK_MT_MODULO        16U          /* multi-turn counter wraps 0..15  */

/* ts_fast tick period in nanoseconds (measured; see the note on ts_fast). */
#define YASK_TS_FAST_NS       2000U

/* ── Frame geometry ──────────────────────────────────────────────────────── */

#define YASK_PAYLOAD_BITS     112U         /* data bits after HDLC unstuffing */
#define YASK_PAYLOAD_BYTES    14U
#define YASK_CRC_DATA_BYTES   12U          /* CRC covers bits[0:96]           */

/*
 * Worst-case on-wire response length, used to size the RX capture buffer.
 * 112 payload bits + 16 flag bits + up to 112/5 stuffed bits ~= 151 bits;
 * round up generously to tolerate preamble and trailing idle.
 */
#define YASK_RX_MAX_BITS      256U

/* ── Decoded frame ───────────────────────────────────────────────────────── */

/**
 * @brief One fully decoded encoder response.
 *
 * Field offsets were reverse-engineered from logic-analyser captures and
 * verified against CRC on >22000 captured frames.
 */
typedef struct
{
    /* Position */
    uint32_t single_turn;     /**< bits[60:77]  0..131071, LSB-first          */

    /*
     * Two revolution counters, both stepping once per turn but at different
     * mechanical angles (confirmed over a 2.66-revolution CCW run):
     *   rot3       steps exactly at the single_turn 131071->0 wrap (0 deg)
     *   multi_turn steps at a fixed point near 290 deg
     * Earlier notes described rot3 as a 180-deg crossing counter; it is not,
     * it is a 3-bit revolution counter aligned to the zero crossing.
     */
    uint8_t  multi_turn;      /**< bits[89:93]  revolution counter 0..15      */
    uint8_t  rot3;            /**< bits[77:80]  revolution counter 0..7,
                                   stepping at the 0-degree crossing         */

    /*
     * bits[40:56].  Named "accel" in the original reverse-engineering notes,
     * but measurement does not support that: over a run from standstill to
     * ~0.9 counts/tick the field showed no correlation with speed (median 18
     * when stopped, 17 when moving fastest) and never went negative in
     * either direction of travel.  Treat the meaning as unknown.
     */
    uint16_t raw40;

    /* Commutation */
    uint8_t  hall;            /**< bits[57:60]  U=bit0, V=bit1, W=bit2        */

    /* Timestamps (free-running counters inside the encoder) */
    uint8_t  ts_slow;         /**< bits[16:24]                                */
    uint16_t ts_fast;         /**< bits[24:40]  16-bit DOWN counter, ~2 us
                                   per tick.  It decrements, so an interval
                                   is (previous - current) mod 65536.       */

    /* Status */
    bool     unindexed;       /**< bit[0]  set until the index is found       */
    bool     battery_fail;    /**< bit[2]                                     */
    bool     comp_valid;      /**< bit[7] == 0 means compensation is valid    */
    uint8_t  status_hi;       /**< bits[93:96] unknown status bits            */

    /* Identification / diagnostics */
    uint8_t  id_ff;           /**< bits[8:16]   observed constant 0xFF        */
    uint16_t id_latched;      /**< bits[80:89]  9 bits, constant within a
                                   power cycle but varies between them
                                   (observed 0x0DB and 0x10A)                 */

    /* Integrity */
    uint16_t crc_rx;          /**< bits[96:112] as received                   */
    uint16_t crc_calc;        /**< recomputed over bits[0:96]                 */
    bool     crc_ok;          /**< crc_rx == crc_calc                         */
} yask_frame_t;

/* ── Bit-field map (payload bit offsets) ─────────────────────────────────── */

#define YASK_OFF_UNINDEXED     0U
#define YASK_OFF_BATT_FAIL     2U
#define YASK_OFF_COMP_VALID    7U
#define YASK_OFF_ID_FF         8U
#define YASK_OFF_TS_SLOW      16U
#define YASK_OFF_TS_FAST      24U
#define YASK_OFF_RAW40        40U
#define YASK_OFF_HALL         57U
#define YASK_OFF_SINGLE_TURN  60U
#define YASK_OFF_ROT3         77U
#define YASK_OFF_ID_LATCHED   80U
#define YASK_OFF_MULTI_TURN   89U
#define YASK_OFF_STATUS_HI    93U
#define YASK_OFF_CRC          96U

/* ── Return codes ────────────────────────────────────────────────────────── */

typedef enum
{
    YASK_OK = 0,
    YASK_ERR_NO_FLAG,        /**< no HDLC start flag found in the capture   */
    YASK_ERR_SHORT,          /**< frame ended before 112 payload bits       */
    YASK_ERR_CRC,            /**< payload decoded but CRC mismatched        */
    YASK_ERR_TIMEOUT,        /**< no response edges seen at all             */
    YASK_ERR_OVERRUN         /**< capture buffer filled before end of frame */
} yask_status_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/** One-time setup: builds the TX bit table and arms the RX capture path. */
void          yask_init(void);

/** Blocking request/response cycle.  Fills @p out on YASK_OK. */
yask_status_t yask_read(yask_frame_t *out);

/** CRC-16/X.25 over the first 96 payload bits (12 bytes, LSB-first). */
uint16_t      yask_crc16_x25(const uint8_t *bits);

/**
 * @brief Decode an unstuffed payload bit array into a frame struct.
 * @param bits  Array of @p nbits values, each 0 or 1, LSB-first per field.
 */
yask_status_t yask_decode_payload(const uint8_t *bits, uint32_t nbits,
                                  yask_frame_t *out);

/** Human-readable name for a status code (for logging). */
const char   *yask_status_str(yask_status_t s);

/* ── Glitch rejection ────────────────────────────────────────────────────── */

/**
 * @brief Largest believable change in single_turn between two reads.
 *
 * This trades noise rejection against the fastest motion the filter will
 * pass.  The threshold is only meaningful relative to the poll rate: a step
 * of N counts corresponds to N * 60 * f_poll / 131072 rpm.
 *
 * CAUTION: the rate that matters is the ENCODER LOOP rate -- how often
 * yask_filter_accept() is called -- not the 8 kHz at which the ODrive polls
 * the SPI slave.  The loop rate varies with what else main() is doing, so a
 * fixed counts-per-iteration limit is only ever approximate:
 *
 *   loop rate    counts needed to pass 280 rpm (this bus's ceiling)
 *      500 Hz        1223
 *     1000 Hz         612
 *     2000 Hz         306
 *
 * Sized too low the filter rejects real motion, holds a stale value, and
 * then YASK_GLITCH_RUN_LIMIT lets the accumulated jump through anyway --
 * which is worse than no filter, because the jump arrives with valid parity
 * and the ODrive cannot tell it is wrong.  A 164-count trial did exactly
 * that: inflation of 27x at speeds the shaft was comfortably turning.
 *
 * 2048 counts passes 280 rpm down to a 300 Hz loop while still rejecting the
 * ~1700-count disturbances measured with the motor energised.  It is a
 * compromise, not a correct design: the filter should derive its limit from
 * the frame's own ts_fast timestamp so the threshold tracks the real
 * interval.  See the TODO in yaskawa_decode.c.
 *
 * Set to 0 to disable rejection entirely.
 */
#ifndef YASK_MAX_STEP_COUNTS
#define YASK_MAX_STEP_COUNTS  2048U       /* see the rate caveat above */
#endif

/** How many consecutive rejections before the new value is accepted anyway. */
#ifndef YASK_GLITCH_RUN_LIMIT
#define YASK_GLITCH_RUN_LIMIT 4U
#endif

/**
 * @brief Running filter state for one encoder.
 */
typedef struct
{
    uint32_t last_st;         /**< last accepted single_turn               */
    uint8_t  last_mt;         /**< last accepted multi_turn                */
    uint8_t  run;             /**< consecutive rejections                  */
    bool     primed;          /**< false until the first frame is accepted */
    uint32_t rejected;        /**< total rejections, for diagnostics       */
    uint32_t accepted;        /**< total accepted frames                   */
} yask_filter_t;

/** Reset the filter; the next frame is accepted unconditionally. */
void yask_filter_reset(yask_filter_t *f);

/**
 * @brief Judge whether a decoded frame is physically plausible.
 *
 * Rejects a frame whose single_turn moved further than YASK_MAX_STEP_COUNTS
 * from the last accepted one, taking the shortest path around the wrap so a
 * reading either side of zero is not mistaken for a jump.
 *
 * A sustained run of rejections is accepted anyway after
 * YASK_GLITCH_RUN_LIMIT frames: if the shaft really has moved that far the
 * filter must not latch onto a stale position forever.
 *
 * @return true if the frame should be used, false if it looks like a glitch.
 */
bool yask_filter_accept(yask_filter_t *f, const yask_frame_t *frame);

/* ── Absolute position tracking ──────────────────────────────────────────── */

/**
 * @brief Running absolute position, in counts, across revolutions.
 *
 * single_turn alone cannot represent more than half a revolution of motion
 * between two reads: past that it aliases and the shaft appears to move the
 * short way round, in the wrong direction.  The encoder's own revolution
 * counters resolve this, but both are short (3 and 4 bits) and step at
 * different angles, so this tracker combines them with the single-turn
 * position and keeps a wide running total.
 */
typedef struct
{
    int64_t  position;        /**< accumulated counts since yask_track_reset */
    uint32_t last_st;         /**< previous single_turn                      */
    uint8_t  last_mt;         /**< previous multi_turn                       */
    bool     primed;          /**< false until the first frame is seen       */
} yask_tracker_t;

/** Start (or restart) tracking; the next frame establishes the origin. */
void    yask_track_reset(yask_tracker_t *t);

/**
 * @brief Interval between two frames, in microseconds, from ts_fast.
 *
 * ts_fast decrements, so the elapsed time is (previous - current) modulo
 * 65536 ticks of ~2 us.
 *
 * Only trustworthy for intervals between roughly 1 ms and the counter's
 * 131 ms period.  Below that the observed deltas stop reducing to a
 * consistent tick -- a 1.65 ms poll produced 59 199 ticks, which is neither
 * the expected 824 nor its complement -- so do not derive velocity from it
 * when polling fast.  Use position differences and a known poll rate.
 */
static inline uint32_t yask_interval_us(uint16_t prev_ts, uint16_t cur_ts)
{
    return (uint32_t)((uint16_t)(prev_ts - cur_ts)) * YASK_TS_FAST_NS / 1000U;
}

/**
 * @brief Fold one decoded frame into the running position.
 *
 * Uses multi_turn to count whole revolutions, so it stays correct as long as
 * the shaft turns less than 8 revolutions between reads, rather than the
 * half revolution that single_turn alone allows.
 *
 * @return the updated absolute position in counts.
 */
int64_t yask_track_update(yask_tracker_t *t, const yask_frame_t *f);

#ifdef __cplusplus
}
#endif

#endif /* YASKAWA_ENC_H */

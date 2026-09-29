/**
 * @file    yaskawa_hw.h
 * @brief   Board wiring and timing constants for the Nucleo-F767ZI.
 *
 * Pin map (chosen to avoid the on-board Ethernet PHY, ST-Link VCP, USB OTG
 * and the user LEDs, all of which are hard-wired on this Nucleo):
 *
 *   Signal            Port/Pin   Zio header   Function
 *   ----------------  ---------  -----------  ------------------------------
 *   YASK_TX  (DI)     PE12       CN10-24      GPIO out, driven by TIM1_UP DMA
 *   YASK_DE  (DE+/RE) PE10       CN10-28      GPIO out, RS-485 direction
 *   YASK_RX  (RO)     PE11       CN10-26      GPIO in  + TIM1_CH2 alt func
 *
 *   ODrive SPI slave (AS5047P emulation), SPI4:
 *   ODRV_SCK          PE2        CN9-14       SPI4_SCK,  input from ODrive
 *   ODRV_NSS  (CS)    PE4        CN9-16       SPI4_NSS + EXTI4, input
 *   ODRV_MISO         PE5        CN9-18       SPI4_MISO, output to ODrive
 *   ODRV_MOSI         PE6        CN9-20       SPI4_MOSI, unused by the master
 *
 * Wire DE and !RE together on the transceiver: high = transmit, low = receive.
 *
 * Timer allocation
 *   TIM1  - transmit bit clock, 8 MHz update -> DMA2 Stream5 -> GPIOE->BSRR
 *   TIM8  - receive sample clock, 32 MHz update -> DMA2 Stream1 -> GPIOE->IDR
 *   TIM4  - microsecond delay helper / response timeout
 *
 * Both timers live on APB2 (216 MHz timer clock on this part).
 */

#ifndef YASKAWA_HW_H
#define YASKAWA_HW_H

/* The pin and timing constants are useful to host tests too; only the HAL
   types need the vendor header. */
#ifndef YASK_HOST_TEST
#include "stm32f7xx_hal.h"
#endif

/* ── Capture back-end selection ──────────────────────────────────────────── */

/*
 * Two receive strategies are implemented; pick one here (or define
 * YASK_RX_METHOD on the compiler command line to override).
 *
 *   SAMPLE  - timer-triggered DMA reads of GPIOE->IDR at 36 MHz.
 *             Deterministic, cannot overrun, 4 KB per frame.  Start here.
 *   CAPTURE - TIM1_CH2 both-edge input capture, DMA copies CCR2.
 *             Compact (1 KB instead of 4 KB, no oversampling) and self-timing.
 *             DOES NOT WORK AT 8 MBaud ON THIS PART -- see below.  Selectable
 *             with `make RX_METHOD=1` for experiments.
 *
 * Why CAPTURE fails here: an edge arrives every 125 ns worst case, and DMA2
 * must read CCR2 and write SRAM inside that window.  Stream arbitration plus
 * the peripheral read and memory write is ~90-185 ns at the 108 MHz AHB
 * clock, with the SPI-slave and TX streams contending for the same
 * controller.  Measured result: TIM1 SR bit 10 (CC2OF, overcapture) sets
 * mid-frame, edges are dropped, and the decode stops at ~123 of the ~150
 * bits with NO_FLAG.  The capture path itself is correct -- edge timestamps
 * come out as clean 27/54-tick gaps -- it simply cannot keep up.
 */
#define YASK_RX_METHOD_SAMPLE    0
#define YASK_RX_METHOD_CAPTURE   1

#ifndef YASK_RX_METHOD
#define YASK_RX_METHOD           YASK_RX_METHOD_SAMPLE
#endif

/* ── Pin assignment ──────────────────────────────────────────────────────── */

#define YASK_TX_GPIO_Port     GPIOE
#define YASK_TX_Pin           GPIO_PIN_12

#define YASK_DE_GPIO_Port     GPIOE
#define YASK_DE_Pin           GPIO_PIN_10

#define YASK_RX_GPIO_Port     GPIOE
#define YASK_RX_Pin           GPIO_PIN_11
#define YASK_RX_BITPOS        11U          /* bit index inside GPIOE->IDR    */

/* ── Line timing ─────────────────────────────────────────────────────────── */

#define YASK_TIMER_CLK_HZ     216000000U   /* APB2 timer clock               */

#define YASK_BAUD_HZ            8000000U   /* Manchester symbol rate         */
#define YASK_BITRATE_HZ         4000000U   /* data bit rate (2 symbols/bit)  */

/*
 * TIM1 update period for one Manchester symbol (125 ns).
 * 216 MHz / 8 MHz = 27 ticks exactly -> ARR = 26.
 */
#define YASK_TX_TIM_ARR       (YASK_TIMER_CLK_HZ / YASK_BAUD_HZ - 1U)   /* 26 */

/*
 * RX oversampling.
 *
 * 216 MHz / 6 = 36 MHz is an exact integer divider and gives 4.5 samples per
 * 125 ns Manchester symbol -- the fastest rate DMA can comfortably sustain
 * and well above the 3 samples/symbol needed to place an edge unambiguously.
 *
 * The fractional 0.5 does not matter: the decoder re-synchronises its phase
 * on every Manchester transition rather than free-running off a fixed
 * sample count, so accumulated drift never exceeds one sample period.
 *
 *   36 MHz sample clock -> 27.8 ns per sample
 *   125 ns symbol       -> 4 or 5 samples, alternating
 */
#define YASK_RX_SAMPLE_HZ     36000000U
#define YASK_RX_TIM_ARR       (YASK_TIMER_CLK_HZ / YASK_RX_SAMPLE_HZ - 1U)  /* 5 */

/* Nominal samples per Manchester symbol, in 1/256 fixed point (4.5 * 256). */
#define YASK_RX_SPS_Q8        1152U

/*
 * Number of IDR samples per response.
 *
 * At 36 MHz each sample is 27.8 ns, so the window is SAMPLES/36 us.  The
 * request ends, the encoder answers about 17 us later, and the reply runs
 * about 32 us -- measured on hardware, the last edge lands at sample 1884
 * (52.3 us).  3072 samples gives an 85 us window: 60% headroom over the
 * observed frame, while keeping the decode short.
 *
 * The decoder scans this buffer linearly, so the size directly sets the
 * per-read cost: at 6144 samples a read took 0.430 ms (59% of it decoding),
 * against 0.171 ms of actual capture.
 */
#define YASK_RX_SAMPLES       3072U

/* ── ODrive SPI slave (AS5047P emulation) ────────────────────────────────── */

#define ODRV_SCK_GPIO_Port    GPIOE
#define ODRV_SCK_Pin          GPIO_PIN_2

#define ODRV_NSS_GPIO_Port    GPIOE
#define ODRV_NSS_Pin          GPIO_PIN_4

#define ODRV_MISO_GPIO_Port   GPIOE
#define ODRV_MISO_Pin         GPIO_PIN_5

#define ODRV_MOSI_GPIO_Port   GPIOE
#define ODRV_MOSI_Pin         GPIO_PIN_6

/* Response timeout: the encoder replies well within this window. */
#define YASK_RESPONSE_TIMEOUT_US   200U

#endif /* YASKAWA_HW_H */

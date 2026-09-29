/**
 * @file    yaskawa_odrive.h
 * @brief   AS5047P-compatible SPI slave: presents the decoded Yaskawa
 *          position to an ODrive (or FFBeast) motion controller.
 *
 * How it works
 * ------------
 * The STM32 is the SPI *slave*.  The controller is the master and polls
 * position exactly as it would poll an AMS AS5047P magnetic encoder.
 *
 * When the controller asserts CS (NSS falling edge) an EXTI interrupt fires
 * and stages the latest angle word in the SPI data register; the hardware
 * then clocks it out on SCK with no further CPU involvement.
 *
 * AS5047P frame, 16 bits, MSB first
 * ---------------------------------
 *   bit 15     even parity over bits[14:0]
 *   bit 14     EF, error flag (1 = no valid position)
 *   bits[13:0] 14-bit angle, 0..16383
 *
 * The encoder's 17-bit single-turn value is mapped to 14 bits by dropping
 * the three least-significant bits, giving 16384 counts per revolution and
 * 0.022 degrees of resolution.
 *
 * ODrive configuration
 * --------------------
 *   odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
 *   odrv0.axis0.encoder.config.cpr  = 16384
 *   odrv0.save_configuration()
 *   odrv0.reboot()
 *
 * FFBeast: select "AMS AS5047 / SPI absolute", resolution 16384.
 *
 * Wiring (SPI4; see yaskawa_hw.h for the pin map)
 * -----------------------------------------------
 *   STM32 PE5  MISO  ->  controller encoder MISO
 *   STM32 PE2  SCK   <-  controller encoder SCK
 *   STM32 PE4  NSS   <-  controller encoder CS   (also the EXTI source)
 *   PE6 MOSI is unused by the master; leave it or tie it low.
 *
 * Ground the two boards together.  The Nucleo's pins are 3.3 V; check the
 * controller's encoder port is 3.3 V logic before connecting.
 */

#ifndef YASKAWA_ODRIVE_H
#define YASKAWA_ODRIVE_H

#include <stdint.h>
#include "yaskawa_enc.h"

#ifndef YASK_HOST_TEST
#include "stm32f7xx_hal.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Parity convention.  The AS5047P uses even parity over bits[14:0]; set this
 * to 1 if a controller expects odd parity instead.
 */
#ifndef YASK_ODRIVE_ODD_PARITY
#define YASK_ODRIVE_ODD_PARITY 0
#endif

/** Counts per revolution as seen by the controller (17-bit >> 3). */
#define YASK_ODRIVE_CPR       16384U

/** Bits discarded from single_turn to reach the 14-bit AS5047P angle. */
#define YASK_ODRIVE_SHIFT     3U

/**
 * @brief Reverse the reported direction of rotation.
 *
 * Off by default: the direction has NOT been shown to be wrong.
 *
 * An apparent inversion (open-loop field forward reading -196 rpm, reverse
 * reading +815 rpm) turned out not to be a polarity problem at all -- the
 * magnitudes are impossible on a 54 V bus, whose ceiling is ~280 rpm, and
 * enabling this made no difference to them.  Whatever produces those numbers
 * is not a sign error.  Set to 1 only if a measurement at a speed the bus can
 * actually sustain shows the encoder counting against the motor.
 */
#ifndef YASK_ODRIVE_INVERT
#define YASK_ODRIVE_INVERT    0
#endif

/**
 * @brief Build an AS5047P frame word.
 *
 * Exposed for testing; @p angle14 must already be masked to 14 bits.
 *
 * @param angle14 angle, 0..16383
 * @param ef      1 to flag the position as invalid, 0 for good data
 */
uint16_t yask_odrive_word(uint16_t angle14, uint8_t ef);

#ifndef YASK_HOST_TEST

/**
 * @brief Enable the SPI slave and stage an error word.
 *
 * Call once after MX_SPI4_Init().  Until the first successful decode the
 * controller reads EF=1 rather than a stale or zero position.
 */
void yask_odrive_init(SPI_HandleTypeDef *hspi);

/** Publish a freshly decoded position (call only when crc_ok). */
void yask_odrive_update(SPI_HandleTypeDef *hspi, const yask_frame_t *f);

/** Flag the position invalid after a failed read. */
void yask_odrive_set_error(SPI_HandleTypeDef *hspi);

/** Chip-select edges seen; 0 means the controller is not polling us. */
extern volatile uint32_t g_nss_count;

/** Words staged into the transmit register. */
extern volatile uint32_t g_tx_count;

/** Words received and discarded (one per completed transfer). */
extern volatile uint32_t g_rx_count;

/**
 * @brief Stage the pending word on the NSS falling edge.
 *
 * Call from HAL_GPIO_EXTI_Callback() for ODRV_NSS_Pin.  Loading the data
 * register here, before the first SCK edge, means the controller always
 * reads the freshest word regardless of when the last update ran.
 */
void yask_odrive_nss_isr(SPI_HandleTypeDef *hspi);

/**
 * @brief Service one completed SPI transfer.
 *
 * Call from SPI4_IRQHandler().  Drains the receive FIFO and reloads the
 * transmit register, which a master polling faster than the main loop
 * requires -- otherwise the FIFO overruns between stages.
 */
void yask_odrive_spi_isr(SPI_HandleTypeDef *hspi);

#endif /* !YASK_HOST_TEST */

#ifdef __cplusplus
}
#endif

#endif /* YASKAWA_ODRIVE_H */

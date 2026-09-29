/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration
  *          of the SPI instances.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "spi.h"

/* USER CODE BEGIN 0 */
#include "yaskawa_hw.h"
/* USER CODE END 0 */

SPI_HandleTypeDef hspi4;

/* SPI4 init function */
void MX_SPI4_Init(void)
{
  /* USER CODE BEGIN SPI4_Init 0 */
  /*
   * Slave, 16-bit, MSB first, hardware NSS.
   *
   * Clock phase: the AS5047P datasheet specifies mode 1 (CPHA=1), but with
   * that setting the controller latched bits mid-transition -- a logic
   * analyser showed a constant transmitted word (angle 18, 0x0012) arriving
   * as 0, 2, 8, 10, 16, 18, 19, 24, 25, which is 18 with individual bits
   * gained and lost rather than random corruption. That is the signature of
   * sampling on the wrong clock edge, so mode 0 is used instead.
   */
  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_SLAVE;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;   /* mode 1; see the note below */
  hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI4)
  {
    /* USER CODE BEGIN SPI4_MspInit 0 */

    /* USER CODE END SPI4_MspInit 0 */
    /* SPI4 clock enable */
    __HAL_RCC_SPI4_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SPI4 GPIO Configuration
    PE2     ------> SPI4_SCK
    PE4     ------> SPI4_NSS
    PE5     ------> SPI4_MISO
    PE6     ------> SPI4_MOSI
    */
    /*
     * NSS is pulled up and SCK pulled down so the slave has a defined idle
     * state when the master is absent or powered off.  Left floating, noise
     * on these inputs is taken as chip-select and clock edges: the slave
     * "receives" continuously, the RX FIFO fills and OVR latches, and no
     * amount of draining clears it because with a phantom clock the shift
     * register keeps refilling.  Observed as SR=0x06c3, BSY=1, FRLVL=3 with
     * nothing connected.
     *
     * The pull directions match SPI mode 1: CS idles high, clock idles low.
     */
    GPIO_InitStruct.Pin = ODRV_NSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*
     * SCK and MOSI are inputs on a slave, and the pin speed setting still
     * matters: VERY_HIGH selects the fastest input path, which faithfully
     * reproduces the ~20 ns glitches the motor's switching couples onto the
     * clock.  The slave counts those as clock edges, shifts its register, and
     * hands the master a rotated word that often still passes parity.
     *
     * A slower setting filters nanosecond pulses while passing the 1.3 MHz
     * clock with orders of magnitude to spare (a half-period is ~380 ns).
     * This does not remove the coupling -- twisting SCK with ground took it
     * from 64% of transitions to 10% -- but it stops what remains from
     * reaching the shift register.
     */
    GPIO_InitStruct.Pin = ODRV_SCK_Pin | ODRV_MOSI_Pin;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* restore default */

    /*
     * MISO is driven at LOW speed on purpose.  Its rising edge capacitively
     * couples into the CS wire running alongside it, and CS is held only by
     * the internal pull-up (30-50k on this part), so the coupled charge drags
     * it briefly over threshold: a ~20ns phantom deassert, mid-frame.
     *
     * The analyser shows the signature unambiguously -- every CS runt is a
     * HIGH pulse, 100% of them on a MISO 0->1 edge, none on 1->0.  Mode 0 put
     * that MISO transition on the falling clock edge, which is why this only
     * appeared when we moved to mode 1 (fee4e93).
     *
     * An external ~2.2k pull-up on CS is the textbook fix, but slowing this
     * edge removes the aggressor instead of bracing against it, and costs
     * nothing: at 1.3MHz the bit period is ~770ns, so even LOW speed
     * (~10-20ns edge into the analyser's load) settles with orders of
     * magnitude to spare.
     */
    GPIO_InitStruct.Pin = ODRV_MISO_Pin;
    GPIO_InitStruct.Pull = GPIO_NOPULL;      /* we drive this one */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;  /* restore default */

    /* USER CODE BEGIN SPI4_MspInit 1 */
    /*
     * Route NSS to EXTI as well, so the falling edge of CS can stage the
     * next word before the master's first SCK edge.
     *
     * This is done with registers rather than HAL_GPIO_Init(): passing
     * GPIO_MODE_IT_FALLING for the same pin would rewrite MODER and undo
     * the alternate-function setting above.  EXTI takes its input from the
     * pin's input path, which stays live in AF mode, so the pin can be both
     * SPI4_NSS and the EXTI4 source.
     */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /* EXTI line 4 <- port E (EXTICR[1] holds lines 4..7, nibble 0). */
    SYSCFG->EXTICR[1] = (SYSCFG->EXTICR[1] & ~SYSCFG_EXTICR2_EXTI4)
                      | SYSCFG_EXTICR2_EXTI4_PE;

    EXTI->IMR  |= ODRV_NSS_Pin;    /* unmask the interrupt          */
    EXTI->FTSR |= ODRV_NSS_Pin;    /* trigger on the falling edge   */
    EXTI->RTSR &= ~ODRV_NSS_Pin;   /* not on the rising edge        */

    /*
     * Highest priority: this ISR only stages a word, and any delay eats
     * into the master's CS-to-SCK setup time.
     */
    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    /* Transfers are served by DMA; no per-transfer interrupt is needed. */
    /* USER CODE END SPI4_MspInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

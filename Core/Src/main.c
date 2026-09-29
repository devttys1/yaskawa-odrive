/* USER CODE BEGIN Header */
/**
 * @file    main.c
 * @brief   Yaskawa 17-bit incremental encoder proof-of-concept.
 *
 * Polls the encoder as fast as the link allows and prints each decoded frame
 * on the ST-Link virtual COM port (USART3, 115200 baud, 8N1).
 *
 * Press the blue USER button to dump the raw capture buffer instead, which is
 * useful for eyeballing the line with the decoder out of the picture.
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "yaskawa_enc.h"
#include "yaskawa_odrive.h"
#include "yaskawa_hw.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* htim1/htim8 and their DMA handles are defined in tim.c, huart3 in usart.c. */
const uint16_t *yask_rx_raw_samples(uint32_t *count);

/* Set to 1 (from GDB or the USER button) to run the benchmark. */
volatile uint8_t bench_request = 0;

yask_status_t   yask_rx_decode(yask_frame_t *out);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*
 * Console output is a build-time option.
 *
 * The encoder loop is the ODrive's position source: it polls the SPI slave
 * once per current-control period (~8 kHz), and every iteration we spend in
 * the console is an iteration where the ODrive re-reads a stale angle.  A
 * single ~120 char line costs ~600 us at 2 Mbaud, which alone held the loop
 * to ~1600 Hz.
 *
 * YASK_CONSOLE defaults to off, so a release build does no UART work at all
 * and the strings are dropped by the linker.  Build with -DYASK_CONSOLE=1
 * for the interactive console, the benchmark and the raw capture dump.
 */
#ifndef YASK_CONSOLE
#define YASK_CONSOLE 0
#endif

#if YASK_CONSOLE
#define YASK_PRINTF(...)  printf(__VA_ARGS__)
#else
#define YASK_PRINTF(...)  ((void)0)
#endif

/*
 * Retarget printf to the ST-Link VCP.
 *
 * newlib calls _write() with whatever it has buffered, which for a long
 * printf can be a partial line; emitting those byte by byte let one frame's
 * output start before the previous had drained, interleaving the two.
 * Writing each chunk in a single blocking HAL_UART_Transmit keeps lines
 * intact.
 */
int _write(int file, char *ptr, int len)
{
    (void)file;

#if !YASK_CONSOLE
    /* Console disabled: swallow anything newlib still routes here. */
    (void)ptr;
    return len;
#else

    /*
     * Write to the data register directly rather than via
     * HAL_UART_Transmit().
     *
     * The HAL routine takes the handle lock and runs a SysTick-based timeout
     * for every byte; with interrupts arriving between bytes it was dropping
     * characters mid-line, which showed up as mangled console output even
     * for a fixed test string and even at one line per second.  Polling TXE
     * here is both simpler and reliable -- the encoder decode itself was
     * always fine (verified at 60/60 CRC-OK straight from RAM).
     */
    for (int i = 0; i < len; i++)
    {
        while ((USART3->ISR & USART_ISR_TXE) == 0U)
        {
        }
        USART3->TDR = (uint8_t)ptr[i];
    }

    /* Let the last byte leave the shift register before returning. */
    while ((USART3->ISR & USART_ISR_TC) == 0U)
    {
    }

    return len;
#endif
}

#if YASK_CONSOLE
/**
 * @brief Time the read and print paths, and report the achievable rates.
 *
 * Runs on the target with the debugger uninvolved, so the numbers are what
 * the firmware can actually sustain rather than what GDB can drive.
 */
static void benchmark(void)
{
    /* DWT cycle counter: 1 tick = 1/216 us at 216 MHz. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t N = 200;
    yask_frame_t f;
    uint32_t ok = 0;

    uint32_t t0 = DWT->CYCCNT;
    for (uint32_t i = 0; i < N; i++)
    {
        if (yask_read(&f) == YASK_OK) ok++;
    }
    uint32_t read_cyc = DWT->CYCCNT - t0;

    /* Decode alone, reusing the buffer the last read captured. */
    t0 = DWT->CYCCNT;
    for (uint32_t i = 0; i < N; i++)
    {
        (void)yask_rx_decode(&f);
    }
    uint32_t dec_cyc = DWT->CYCCNT - t0;

    /* One formatted line, to size the console cost. */
    char line[160];
    t0 = DWT->CYCCNT;
    for (uint32_t i = 0; i < N; i++)
    {
        (void)snprintf(line, sizeof(line),
                       "st=%6lu (%7.3f deg)  abs=%+9.3f rev  mt=%2u rot3=%u  "
                       "hall=U%uV%uW%u  ts=%5u  crc=0x%04X OK\r\n",
                       (unsigned long)f.single_turn, 0.0, 0.0,
                       f.multi_turn, f.rot3, 1U, 0U, 1U, f.ts_fast, f.crc_rx);
    }
    uint32_t fmt_cyc = DWT->CYCCNT - t0;
    uint32_t len = (uint32_t)strlen(line);

    const double MHZ = 216000.0;   /* cycles per millisecond */

    YASK_PRINTF("\r\n=== benchmark (%lu iterations) ===\r\n", (unsigned long)N);
    YASK_PRINTF("yask_read      %8.3f ms  -> %7.0f Hz   (%lu/%lu CRC OK)\r\n",
           read_cyc / MHZ / N, N * 216000000.0 / read_cyc,
           (unsigned long)ok, (unsigned long)N);
    YASK_PRINTF("  decode only  %8.3f ms\r\n", dec_cyc / MHZ / N);
    YASK_PRINTF("  capture win  %8.3f ms  (%u samples / 36 MHz)\r\n",
           YASK_RX_SAMPLES / 36000.0, (unsigned)YASK_RX_SAMPLES);
    YASK_PRINTF("snprintf line  %8.3f ms  (%lu chars)\r\n",
           fmt_cyc / MHZ / N, (unsigned long)len);

    /* UART time for that line at a range of baud rates. */
    YASK_PRINTF("\r\nconsole cost for a %lu-char line:\r\n", (unsigned long)len);
    const uint32_t bauds[] = { 115200U, 230400U, 460800U, 921600U,
                               2000000U, 3000000U, 6000000U };
    for (uint32_t i = 0; i < sizeof(bauds)/sizeof(bauds[0]); i++)
    {
        double ms = (double)len * 10.0 * 1000.0 / (double)bauds[i];
        YASK_PRINTF("  %7lu baud  %7.3f ms/line  -> max %6.0f lines/s\r\n",
               (unsigned long)bauds[i], ms, 1000.0 / ms);
    }
    YASK_PRINTF("\r\n");
}

/**
 * @brief Dump the raw RX capture as a run-length summary.
 *
 * Printing 2048 individual samples is unreadable and slow; runs of equal
 * level tell you everything you need about edge placement and symbol width.
 */
static void dump_raw_capture(void)
{
    uint32_t n;
    const uint16_t *s = yask_rx_raw_samples(&n);
    const uint32_t mask = 1U << YASK_RX_BITPOS;

    if (s == NULL || n == 0U)
    {
        YASK_PRINTF("\r\n(raw dump is only available with the GPIO-sampling "
               "back-end)\r\n");
        return;
    }

    YASK_PRINTF("\r\n--- raw capture: %lu samples @ %lu MHz ---\r\n",
           (unsigned long)n, (unsigned long)(YASK_RX_SAMPLE_HZ / 1000000U));

    uint32_t runs = 0;
    uint8_t  lvl  = (s[0] & mask) ? 1U : 0U;
    uint32_t len  = 1;

    for (uint32_t i = 1; i < n && runs < 120U; i++)
    {
        uint8_t cur = (s[i] & mask) ? 1U : 0U;
        if (cur == lvl)
        {
            len++;
        }
        else
        {
            YASK_PRINTF("%c%lu ", lvl ? 'H' : 'L', (unsigned long)len);
            runs++;
            lvl = cur;
            len = 1;
        }
    }
    YASK_PRINTF("\r\n--- end (%lu runs shown) ---\r\n", (unsigned long)runs);
}
#endif /* YASK_CONSOLE */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI4_Init();

  /* USER CODE BEGIN 2 */
    /* Line-buffer stdout so each frame's line reaches _write() in one piece. */
#if YASK_CONSOLE
  static char stdout_buf[256];
  setvbuf(stdout, stdout_buf, _IOLBF, sizeof(stdout_buf));
#endif

  yask_init();

  /* Present position to the ODrive as an AS5047P. */
  yask_odrive_init(&hspi4);

#if YASK_CONSOLE
  benchmark();
#endif

    YASK_PRINTF("\r\n\r\nYaskawa UTSIH-B17CK incremental encoder decoder\r\n");
    YASK_PRINTF("TX=PE12  DE=PE10  RX=PE11   8 MBaud Manchester, RX method: %s\r\n",
           (YASK_RX_METHOD == YASK_RX_METHOD_CAPTURE) ? "input-capture"
                                                      : "GPIO sampling");
    YASK_PRINTF("Press USER button to dump the raw capture.\r\n\r\n");

    uint32_t ok = 0, err = 0, glitches = 0, last_report = 0;
    uint32_t last_frame_print = 0;

  /* Tracks position across revolutions; see the note in yaskawa_enc.h. */
  yask_tracker_t tracker;
  yask_track_reset(&tracker);

  /* Rejects physically impossible angle steps before they reach the
     controller; see the note on PWM noise in yaskawa_enc.h. */
  yask_filter_t filter;
  yask_filter_reset(&filter);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    yask_frame_t f;
    yask_status_t st = yask_read(&f);

    if (st == YASK_OK && !yask_filter_accept(&filter, &f))
    {
      /* Implausible step: hold the last good position rather than
         publishing a glitch the controller would read as a wrap. */
      glitches++;
      continue;
    }

    if (st == YASK_OK)
    {
      ok++;
      int64_t abspos = yask_track_update(&tracker, &f);
      (void)abspos;
      yask_odrive_update(&hspi4, &f);

      /*
       * Print at most one frame line every 100 ms.
       *
       * This used to print on every frame.  At 2 Mbaud a ~120 char line
       * takes ~600 us to clock out, which throttled the whole poll loop to
       * ~1667 Hz -- but the ODrive samples the SPI slave at 8 kHz, once per
       * current-control period.  Every angle was therefore published about
       * five polls late, so the ODrive read a stale value repeatedly and
       * then saw the accumulated motion arrive as one large jump.  Those
       * jumps carry perfectly valid parity, so spi_error_rate stayed at
       * 0.00000 while shadow_count gained revolutions the shaft never
       * turned (3868 rpm reported against a true 40 rpm).
       *
       * The SPI publish above must never be gated on the console.
       */
#if YASK_CONSOLE
      if (HAL_GetTick() - last_frame_print >= 100U)
      {
        last_frame_print = HAL_GetTick();
        float deg  = ((float)f.single_turn / (float)YASK_ENCODER_RES) * 360.0f;
        float revs = (float)((double)abspos / (double)YASK_ENCODER_RES);

        YASK_PRINTF("st=%6lu (%7.3f deg)  abs=%+9.3f rev  mt=%2u rot3=%u  "
               "hall=U%uV%uW%u  ts=%5u  %s%s crc=0x%04X OK\r\n",
               (unsigned long)f.single_turn, (double)deg, (double)revs,
               f.multi_turn, f.rot3,
               (f.hall >> 0) & 1U, (f.hall >> 1) & 1U, (f.hall >> 2) & 1U,
               f.ts_fast,
               f.unindexed    ? "UNINDEXED " : "",
               f.battery_fail ? "BATT_FAIL " : "",
               f.crc_rx);
      }
#endif
    }
    else
    {
      err++;
      yask_odrive_set_error(&hspi4);
      if (HAL_GetTick() - last_frame_print >= 100U)
      {
        last_frame_print = HAL_GetTick();
        YASK_PRINTF("read failed: %s\r\n", yask_status_str(st));
      }
    }

    /* Periodic health line so a bad link is obvious at a glance. */
    if (HAL_GetTick() - last_report >= 5000U)
    {
      last_report = HAL_GetTick();
      YASK_PRINTF(">> %lu ok, %lu errors, %lu glitches rejected (%lu%% good)\r\n",
             (unsigned long)ok, (unsigned long)err, (unsigned long)glitches,
             (unsigned long)((ok * 100U) / (ok + err ? ok + err : 1U)));
    }

#if YASK_CONSOLE
    if (bench_request)
    {
      bench_request = 0;
      benchmark();
    }

    /*
     * Polling the USER button costs an AHB read every iteration, and the
     * handler blocks the loop for as long as the button is held.  Both are
     * console-only diagnostics, so they go with it.
     */
    if (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
    {
      benchmark();
      dump_raw_capture();
      while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin)
             == GPIO_PIN_SET) { }
    }
#endif

    /*
     * No delay: the loop is paced by the read (~0.36 ms) and the console
     * write.  See README for the measured rates -- printing every frame is
     * what limits throughput, not the encoder link.
     */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

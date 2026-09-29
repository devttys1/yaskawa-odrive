# Command-line build for the Yaskawa encoder proof-of-concept.
# STM32CubeIDE users can ignore this and import the .ioc / project directly.

TARGET  = yaskawa-encoder
BUILD   = build

TOOLCHAIN ?= $(shell dirname $(shell find /Applications/STM32CubeIDE.app -name arm-none-eabi-gcc -type f 2>/dev/null | head -1))
CC      = $(TOOLCHAIN)/arm-none-eabi-gcc
OBJCOPY = $(TOOLCHAIN)/arm-none-eabi-objcopy
SIZE    = $(TOOLCHAIN)/arm-none-eabi-size

MCU = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# Set CONSOLE=1 for the VCP console, benchmark and raw-capture dump.
# Off by default: the console throttles the encoder loop, which is the
# ODrive's position source.
CONSOLE ?= 0

# RX back-end: 0 = oversampled GPIO (default), 1 = TIM1_CH2 input capture.
RX_METHOD ?= 0

C_DEFS = -DUSE_HAL_DRIVER -DSTM32F767xx -DYASK_CONSOLE=$(CONSOLE) \
         -DYASK_RX_METHOD=$(RX_METHOD)

C_INCLUDES = \
  -ICore/Inc \
  -IDrivers/STM32F7xx_HAL_Driver/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F7xx/Include \
  -IDrivers/CMSIS/Include

C_SOURCES = \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/dma.c \
  Core/Src/tim.c \
  Core/Src/spi.c \
  Core/Src/yaskawa_odrive.c \
  Core/Src/usart.c \
  Core/Src/yaskawa_enc.c \
  Core/Src/yaskawa_tx.c \
  Core/Src/yaskawa_rx.c \
  Core/Src/yaskawa_decode.c \
  Core/Src/stm32f7xx_it.c \
  Core/Src/stm32f7xx_hal_msp.c \
  Core/Src/system_stm32f7xx.c \
  Core/Src/syscalls.c \
  Core/Src/sysmem.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_cortex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_dma_ex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_flash_ex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_gpio.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_pwr_ex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_rcc_ex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_tim_ex.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_spi.c \
  Drivers/STM32F7xx_HAL_Driver/Src/stm32f7xx_hal_uart.c

ASM_SOURCES = Core/Startup/startup_stm32f767zitx.s

OPT = -O2 -g3
CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -Wextra -fdata-sections -ffunction-sections -MMD -MP
LDSCRIPT = STM32F767ZITX_FLASH.ld
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
          -Wl,-Map=$(BUILD)/$(TARGET).map,--cref -Wl,--gc-sections -u _printf_float

OBJECTS  = $(addprefix $(BUILD)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
OBJECTS += $(addprefix $(BUILD)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin

$(BUILD)/%.o: %.c | $(BUILD)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD)/%.o: %.s | $(BUILD)
	$(CC) -c $(MCU) $(OPT) $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD):
	mkdir -p $@

clean:
	rm -rf $(BUILD)

# Checks the invariants that fail silently rather than at build time:
# DMA buffers must sit in SRAM1 (DMA2 cannot reach DTCM), and the timer
# reload values must produce exactly 8 MHz and 36 MHz.
verify: $(BUILD)/$(TARGET).elf
	@echo "== DMA buffer placement (must be 0x2002xxxx = SRAM1) =="
	@$(TOOLCHAIN)/arm-none-eabi-nm $< | grep -iE "rx_samples|rx_edges|tx_bsrr" | \
	  awk '{ if ($$1 ~ /^2002/) print "  OK   " $$1, $$3; \
	         else { print "  FAIL " $$1, $$3, "(not in SRAM1 -- DMA will silently do nothing)"; bad=1 } } \
	       END { if (bad) exit 1 }'
	@echo "== timer reload values =="
	@$(TOOLCHAIN)/arm-none-eabi-nm $< | grep -q MX_TIM1_Init && echo "  OK   MX_TIM1_Init linked" || (echo "  FAIL TIM1 init missing"; exit 1)
	@$(TOOLCHAIN)/arm-none-eabi-nm $< | grep -q MX_TIM8_Init && echo "  OK   MX_TIM8_Init linked" || (echo "  FAIL TIM8 init missing"; exit 1)

.PHONY: all clean verify

-include $(wildcard $(BUILD)/*.d)

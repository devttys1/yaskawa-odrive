# Debugging

Scripts for flashing and debugging over the Nucleo's on-board ST-Link. They use
the toolchain bundled with STM32CubeIDE — nothing extra to install.

Every script is **non-interactive and terminates on its own**, so they can be
driven from a script or by an assistant reading the output.

## Quick start

```sh
./tools/debug/probe.sh            # is the ST-Link visible?
./tools/debug/flash.sh            # build + flash + reset
./tools/debug/serial.py 50        # read 50 lines of decoded frames
./tools/debug/serial.py -o run.txt   # capture to a file until Ctrl-C
```

## Scripts

| Script | Purpose |
|---|---|
| `probe.sh` | Reports whether an ST-Link and target are detected. Run this first. |
| `flash.sh` | Builds and flashes. `--no-build` to flash the existing ELF. |
| `serial.py [N] [-o FILE]` | Reads the VCP at 115200 baud. `N` stops after N lines; `-o` captures to a file. Refuses to start if another process holds the port. |
| `gdbserver.sh {start\|stop\|status}` | Manages the ST-Link GDB server. |
| `gdb.sh "cmd"...` | Runs GDB commands in batch mode and prints the output. |
| `reset-run.sh` | Resets the target and lets it run free. |
| `inspect.gdb` | Halts in the decode path and dumps frame, buffers, timers, DMA. |
| `rate.gdb` | Measures CRC success rate over 60 reads, straight from RAM (no UART). |
| `spi.gdb` | Is the motion controller polling us? Chip-select edge count and SPI state. |

`gdb.sh` starts the gdbserver automatically.

## Examples

```sh
# Where is it stuck?
./tools/debug/gdb.sh "monitor halt" "bt" "info registers"

# Watch the decode result
./tools/debug/gdb.sh "break yask_rx_decode" "continue" "finish" "print f"

# Full state dump — frame, raw samples, timers, DMA, GPIO
./tools/debug/gdb.sh -f tools/debug/inspect.gdb

# Is the RX line moving at all?
./tools/debug/gdb.sh "monitor halt" \
  "print/x 'yaskawa_rx.c'::rx_samples[0]@64"
```

## Garbled console output

If lines arrive interleaved or with chunks missing mid-field, something else
has the serial port open and is consuming bytes from the same stream --
STM32CubeIDE's built-in serial terminal is the usual cause. Check with:

```sh
lsof /dev/cu.usbmodem*
```

`serial.py` checks this before opening the port and refuses to start rather
than hanging or capturing corrupted lines. Pass `--force` to override. It is not a firmware fault: the
decode counters read straight out of RAM over GDB are unaffected.

```sh
# Decode success rate without involving the UART at all
./tools/debug/gdb.sh -f tools/debug/rate.gdb
```

## ODrive notes

`odrv0.clear_errors()` **does not exist** on ODrive v3.6 firmware. Calling it
raises `AttributeError`, which is easy to swallow in a `try` block and then
believe the errors were cleared when they were not. Clear the fields directly:

```python
odrv0.axis0.encoder.error = 0
odrv0.axis0.motor.error = 0
odrv0.axis0.error = 0
```

`axis.error = 0x100` (ENCODER_FAILED) is only ever a mirror of
`encoder.error` — `Encoder::set_error()` sets both — so clearing the axis
alone leaves the encoder error in place and the axis re-flags immediately.

`ENCODER_ERROR_NO_RESPONSE` (0x4) is **not** an SPI fault. It is raised only
inside `run_offset_calibration()`, when the open-loop scan moves the shaft by
8 counts or fewer. A healthy SPI link with a shaft that will not turn produces
it, which is misleading; check `spi_error_rate` and `pos_abs` to judge the
link itself.

## What to check when frames do not decode

1. **Is the request going out?** Scope PE12 — 120 symbols, 125 ns each, ~15 µs.
   Or check that the TX DMA ran to completion:
   ```sh
   ./tools/debug/gdb.sh "monitor halt" \
     "printf \"S5 NDTR=%d\\n\", ((DMA_Stream_TypeDef*)0x40026488)->NDTR"
   ```
   `NDTR` should read 0 after a transfer; 122 means the timer never ticked.

2. **Is anything arriving?** Dump `rx_samples` (above). All `0x0000` or all
   `0x0800` means the line never moved — check DE timing and the transceiver.

3. **Are the timers running?** `inspect.gdb` prints `CR1`/`ARR`/`DIER`.
   `ARR` must be 26 for TIM1 and 5 for TIM8; `DIER` bit 8 (`UDE`) must be set
   while a transfer is armed.

4. **Are the DMA buffers in the right memory?**
   ```sh
   arm-none-eabi-nm build/yaskawa-encoder.elf | grep -E "rx_samples|tx_bsrr"
   ```
   Addresses must start `0x2002` (SRAM1). DMA2 cannot reach DTCM (`0x2000`),
   and a misplaced buffer fails silently.

## Environment overrides

`CUBE_IDE`, `GDB`, `GDBSERVER`, `PROGRAMMER`, `ELF`, `GDB_PORT`, `PORT`.

```sh
./tools/debug/serial.py 20 --port /dev/cu.usbmodem14203
```

# Yaskawa Incremental Encoder Decoder — STM32 Proof of Concept

Decodes the serial protocol of a **Yaskawa UTSIH-B17CK** 17-bit incremental
encoder (AC servo motor) using an **STM32 Nucleo-F767ZI** and an RS-485
transceiver.

The link is 8 MBaud Manchester over half-duplex RS-485. The MCU sends a request
frame, the encoder replies with position, velocity, hall state and status. Both
directions are handled entirely by DMA, so the 125 ns symbol timing does not
depend on interrupt latency or CPU load.

## Status

Working on hardware: the STM32 requests and decodes encoder frames at a
100% CRC pass rate, returning position, revolution count, hall state and
status. Verified under rotation over 2.66 revolutions with zero CRC errors
(3448 consecutive good frames in one run). The decoder was also validated
offline against 22 000+ frames from logic-analyser captures.

| Piece | State |
|---|---|
| Request frame TX (timer + DMA → BSRR) | Implemented |
| Response capture, GPIO sampling back-end | Implemented, validated offline |
| Response capture, input-capture back-end | Implemented, validated offline |
| Manchester → HDLC → CRC-16/X.25 decode | Implemented, validated offline |
| ST-Link flash / serial / GDB tooling | Implemented |
| **Verification on real hardware** | **Working — 60/60 frames CRC-OK** |

## Hardware

Wire an RS-485 transceiver (MAX485, SN65HVD72, or similar) between the Nucleo
and the encoder. Tie `DE` and `!RE` together so one pin controls direction.

| Signal | Nucleo pin | Header | Transceiver |
|---|---|---|---|
| `YASK_TX` | **PE12** | CN10-24 | `DI` (driver input) |
| `YASK_DE` | **PE10** | CN10-28 | `DE` + `!RE` (tied) |
| `YASK_RX` | **PE11** | CN10-26 | `RO` (receiver output) |

Console output appears on the ST-Link virtual COM port at **2 000 000 baud,
8N1**. (3 Mbaud was tested and produced nothing — the ST-Link VCP tops out
between the two, even though USART3 itself reaches 3.375 Mbaud.)

These pins are on GPIO port E, which is free of the Nucleo's on-board Ethernet
PHY, ST-Link VCP, USB OTG and user LEDs. Keeping TX and RX on the same port is
deliberate: the receive DMA reads the whole `GPIOE->IDR` register in one
transfer.

> **Note on 5 V and termination.** Most RS-485 transceivers are 5 V parts while
> the F767 pins are 3.3 V. The Nucleo's PE11 input is 5 V-tolerant, but check
> your transceiver's `V_IH` accepts a 3.3 V drive on `DI`/`DE` — use a 3.3 V
> part such as the SN65HVD72 if in doubt. Fit the usual 120 Ω termination at
> the far end of the differential pair.

## Building

```sh
make            # builds build/yaskawa-encoder.elf and .bin
make verify     # checks the invariants that fail silently (see below)
make clean
```

The Makefile finds the toolchain bundled with STM32CubeIDE automatically; set
`TOOLCHAIN=` to point elsewhere.

`make verify` checks the things that produce no compiler error but stop the
encoder working: DMA buffers landing outside SRAM1, and the timer init
functions being dropped from the link.

### Why there is no .ioc

This was originally a CubeMX project. CubeMX repeatedly discarded the TIM1 and
TIM8 entries from the .ioc on load — leaving `Dma.Request0=TIM1_UP` pointing at
a peripheral it had just deleted — and each regeneration then broke the build
in four separate places: no `tim.c`, `HAL_TIM_MODULE_ENABLED` switched off, an
emptied `Drivers/` tree, and a brace mismatch where its `while (1)` collided
with the existing `USER CODE 3` block.

The timers are the whole point of this project (they clock both DMA paths at
exact 8 MHz and 36 MHz), so they are now owned by hand in
[`Core/Src/tim.c`](Core/Src/tim.c) and the project is a plain HAL project built
by the Makefile. The original .ioc is kept at
`reference/yaskawa-incremental-encoder.ioc.orig` for its pin and clock
settings.

### STM32CubeIDE

`.project` and `.cproject` are present and CubeIDE imports and indexes the
project, but its managed build reports "Unknown target" — the device lookup
needs state that only the GUI writes. To get a working IDE build, import once
through the GUI (**File > Import > Existing Projects into Workspace**, or
**New > STM32 Project** targeting STM32F767ZITx) and let it write its own
configuration. Building and debugging from the command line needs none of
this.

## ODrive / FFBeast output

The STM32 also acts as an **SPI slave emulating an AMS AS5047P**, so a motion
controller can read the Yaskawa position as if it were an ordinary magnetic
encoder. See [`yaskawa_odrive.h`](Core/Inc/yaskawa_odrive.h).

| Signal | Nucleo pin | Header | Direction |
|---|---|---|---|
| `ODRV_SCK` | **PE2** | CN9-14 | in, from controller |
| `ODRV_NSS` (CS) | **PE4** | CN9-16 | in, also the EXTI4 source |
| `ODRV_MISO` | **PE5** | CN9-18 | **out**, to controller |
| `ODRV_MOSI` | **PE6** | CN9-20 | unused by the master |

SPI **mode 1** (CPOL=0, CPHA=1), 16-bit, MSB first, hardware NSS. Ground the two
boards together, and check the controller's encoder port is 3.3 V logic.

Mode 1 matches both the AS5047P datasheet and ODrive's own `encoder.cpp`,
which configures `SPI_PHASE_2EDGE`. An earlier experiment here appeared to
favour mode 0, but that test was confounded: the frame layout was also wrong at
the time, and correcting the layout was what actually helped.

Both modes read correctly on a quiet bus. Under PWM switching noise mode 1 is
markedly better, because matching the master's phase puts the sampling instant
at the centre of each bit rather than half a bit-time away. Measured with the
motor armed at zero torque, the spread in the angle the controller reads fell
from **512 counts to 53**.

NSS is configured with a pull-up and SCK/MOSI with pull-downs, so the slave
has a defined idle state when the controller is absent or powered off. Left
floating, noise on those inputs is read as chip-select and clock edges: the
slave receives continuously, the RX FIFO fills and `OVR` latches, and draining
does not help because the phantom clock keeps refilling the shift register
(seen as `SR=0x06c3`, `BSY=1`, `FRLVL=3` with nothing connected). With the
pulls the idle state is `SR=0x1800` — no overrun, empty FIFO.

**ODrive:**

```python
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS
odrv0.axis0.encoder.config.cpr  = 16384
# ODrive drives SCK/MISO/MOSI on its encoder header; only CS is a GPIO
# choice, and it must be named or the controller never asserts CS.
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4      # the GPIO wired to PE4
odrv0.config.gpio4_mode = GPIO_MODE_DIGITAL
odrv0.save_configuration()
odrv0.reboot()
```

On **ODrive v3.6 firmware there is no `gpioN_mode`** — the GPIO config only
exposes pwm/analog mappings for GPIO 1–4. Naming the pin in
`abs_spi_cs_gpio_pin` is sufficient; the firmware configures it as chip
select. The `gpioN_mode` API belongs to the later v3.7/S1 firmware line.

The CS number must match the **silkscreen label** on the SPI header, not the
STM32 pin. Then check `odrv0.axis0.encoder.shadow_count` moves with the shaft
and `odrv0.axis0.encoder.error` stays 0.

**`odrv0.clear_errors()` does not exist on v3.6 firmware** — it raises
`AttributeError`, which is easy to swallow in a `try` block and then believe
the errors were cleared. Clear the fields directly:

```python
odrv0.axis0.encoder.error = 0
odrv0.axis0.motor.error = 0
odrv0.axis0.error = 0
```

`axis.error = 0x100` is only a mirror of `encoder.error`, so clearing the axis
alone achieves nothing.

**Motor parameters must suit the motor, not ODrive's defaults.** A Yaskawa
SGMGH-13DCA6F measures about 1.5 Ω per phase (3 Ω line-to-line) and 9.1 mH,
against ODrive defaults aimed at hobby motors. Two limits get in the way:

- `resistance_calib_max_voltage` defaults to 2 V, which can only drive
  1.33 A into 1.5 Ω while `calibration_current` asks for 5 A. The measurement
  never converges, `get_resistance()` returns NaN and calibration fails with
  `PHASE_RESISTANCE_OUT_OF_RANGE` — with **zero measured current**, which
  looks exactly like disconnected motor leads. Raise it to 6 V and drop
  `calibration_current` to 2 A.
- The inductance sanity check is `2e-6 … 4000e-6` H, marked
  `// TODO arbitrary values set for now` in the firmware. A real industrial
  servo at 9.1 mH is rejected, so set `phase_resistance` and
  `phase_inductance` directly after measuring them.

`calib_range` (default 0.02) may also need widening: the offset scan here
measured 31 989 counts against an expected 32 768, a 2.4% error, just outside
tolerance. A wrong pole count would be out by 25% or more, so a couple of
percent simply reflects truncation to 14 bits and open-loop rotor lag.

**Closed-loop velocity control works.** Commanded 1.0 and 2.0 turns/s both
track to 0.0% error under encoder feedback. Two things had to be right:

- **`encoder.config.bandwidth` must come down.** At the default 3000 (and at
  1000 and 500) entering closed loop fails immediately with
  `MOTOR_ERROR_CONTROL_DEADLINE_MISSED` — `Board/v3/board.cpp` raises that
  when the control loop overruns its PWM period. **200 works.** A high PLL
  bandwidth on a 16384 cpr encoder amplifies quantisation noise, and the
  firmware's own guard is `current_meas_period * (2 * bandwidth) < 1`.
- **`vel_gain` is in Nm per (turn/s)**, already in torque units, so it does
  *not* need scaling down for a large torque constant. 0.2 with
  `vel_integrator_gain` 0.4 is a workable starting point here; the default
  0.02 was tuned for a hobby motor and commands almost no torque against this
  one.

Allow several seconds to settle before measuring: with `vel_ramp_rate` at
2.0 turns/s² a short window catches the ramp and reads high.

**Open-loop spin works.** `AXIS_STATE_LOCKIN_SPIN` drives the motor with no
encoder feedback, which separates the motor drive from the control loop:

```python
odrv0.axis0.config.general_lockin.current = 3.0   # default 10 A exceeds a 5.4 A motor
odrv0.axis0.config.general_lockin.vel     = 20.0  # rad/s electrical
odrv0.axis0.requested_state = AXIS_STATE_LOCKIN_SPIN
```

The shaft turned continuously with no errors and the encoder tracked it from
0.349 to 3.514 revolutions — 0.703 turns/s measured against 0.796 commanded,
the 12% shortfall being the usual open-loop rotor lag. That exercises the whole
chain under real rotation and independently confirms `pole_pairs = 4`.

**ODrive validates our frame exactly as implemented.** From
`Encoder::abs_spi_cb()` in the firmware:

```c
case MODE_SPI_ABS_AMS: {
    uint16_t rawVal = abs_spi_dma_rx_[0];
    // check if parity is correct (even) and error flag clear
    if (ams_parity(rawVal) || ((rawVal >> 14) & 1)) goto done;
    pos = rawVal & 0x3fff;
}
```

Even parity over the whole word, error flag in bit 14, angle in bits[13:0] —
which is what `yask_odrive_word()` produces.

**`ENCODER_ERROR_NO_RESPONSE` (0x4) is not an SPI fault.** In
`Firmware/MotorControl/encoder.cpp` it is set in exactly one place — inside
`run_offset_calibration()`, when the open-loop scan moves the encoder by 8
counts or fewer. It means "I spun the motor and the count barely changed". To
judge the link itself use `spi_error_rate` (should be ~0) and `pos_abs`
(should equal the transmitted angle).

If nothing happens, check from the STM32 side first:

```sh
./tools/debug/gdb.sh -f tools/debug/spi.gdb
```

`NSS (CS) edges seen` is decisive. Zero means no chip-select edge has ever
reached PE4, so the controller is not talking to this pin — a wiring or
CS-number problem, and there is no point reading counts or error codes until
it is non-zero.

**FFBeast:** select *AMS AS5047 / SPI absolute*, resolution 16384.

The 17-bit single-turn position is mapped to the AS5047P's 14-bit angle by
dropping three LSBs — 16384 cpr, 0.022° resolution. Frame layout is bit 15
even parity, bits[14:1] angle, bit 0 error flag; the error flag is raised
before the first successful decode and after any failed read, so the
controller never sees a stale position presented as valid.

On the CS falling edge an EXTI interrupt stages the pending word in the SPI
data register before the master's first clock edge. PE4 is both `SPI4_NSS`
and the EXTI source: EXTI takes its input from the pin's input path, which
stays live in alternate-function mode, so the two coexist — but the EXTI side
is configured with registers rather than `HAL_GPIO_Init()`, which would
rewrite `MODER` and undo the alternate function.

## Protocol

### Physical layer

| Property | Value |
|---|---|
| Symbol rate | 8 MBaud (125 ns per Manchester symbol) |
| Data rate | 4 Mbit/s (2 symbols per bit) |
| Encoding | Manchester — `1` = HIGH→LOW, `0` = LOW→HIGH |
| Framing | HDLC: `0x7E` flags, five-ones bit stuffing |
| Integrity | CRC-16/X.25 over payload bits `[0:96]` |

### Request frame (master → encoder)

60 raw bits → 120 Manchester symbols:

```
sync preamble   25 bits   0101010101010101010101010
HDLC start flag  8 bits   0x7E
payload         19 bits   0xFFFF, bit-stuffed: 1111101111101111101
HDLC end flag    8 bits   0x7E
```

### Response frame (encoder → master)

112 payload bits after unstuffing. **All multi-bit fields are LSB-first.**

| Bits | Field | Notes |
|---|---|---|
| `0` | `unindexed` | set until the index mark has been seen |
| `2` | `battery_fail` | |
| `7` | `comp_valid` | **0 means valid** (inverted) |
| `8:16` | — | observed constant `0xFF` |
| `16:24` | `ts_slow` | slow timestamp counter |
| `24:40` | `ts_fast` | free-running **down** counter, **~2 µs per tick** |
| `40:56` | `raw40` | **meaning unknown** — see note below |
| `56` | — | unknown |
| `57:60` | `hall` | U, V, W commutation sensors |
| `60:77` | `single_turn` | **position, 0–131071** (2¹⁷) |
| `77:80` | `rot3` | 3-bit revolution counter, steps at the **0° crossing** |
| `80:89` | `id_latched` | see note below |
| `89:93` | `multi_turn` | 4-bit revolution counter, steps near **290°** |
| `93:96` | `status_hi` | unknown status bits |
| `96:112` | `crc` | CRC-16/X.25 over bits `[0:96]` |

**On `ts_fast` (bits 24:40).** It counts **down**, not up: an interval is
`(previous - current) mod 65536`. It is only usable between roughly 1 ms and
its 131 ms wrap period — at a 1.65 ms poll the deltas stop reducing to a
consistent tick (59 199 observed, against an expected 824 or its complement),
so derive velocity from position differences and a known poll rate instead. An earlier note here recorded 6.00 µs per
tick, which came from reading it as an up-counter in a single run. Across two
runs at different loop periods only the down-counter reading is
self-consistent — it gives ~2 µs per tick in both, and reconstructs the
`HAL_Delay(100)` and `HAL_Delay(10)` loop periods as 97.7 ms and 10.13 ms.
Read as an up-counter the same data implies 6.00 µs and 0.165 µs, which
cannot both be true.

**On the two revolution counters.** Both `rot3` and `multi_turn` advance once
per revolution, but at different mechanical angles — confirmed over a
2.66-revolution CCW run. `rot3` steps exactly when `single_turn` wraps
131071→0; `multi_turn` steps at a fixed point near 290°. Earlier notes
described `rot3` as a 180° crossing counter, which the data does not support:
it stepped once per full turn, always at the zero crossing.

**On `raw40` (bits 40:56), previously called `accel`.** Measurement does not
support that name. Over a run from standstill to ~0.9 counts/tick the field
showed no correlation with speed (median 18 when stopped, 17 when moving
fastest) and never went negative in either direction of travel. It is small
and jittery frame to frame. Treat it as unidentified until someone correlates
it against a known stimulus.

**On `id_latched` (bits 80:89).** Earlier notes recorded this as a fixed
constant `0x0DB`. Across the captures it is constant *within* a power cycle but
differs *between* them — `0x0DB` in the drill captures and `0x10A` in the
others. It appears to be latched at power-on rather than being a fixed device
ID, so do not rely on a specific value.

Because `multi_turn` is only 4 bits it wraps every 16 revolutions; it is not an
absolute multi-turn position. This is an incremental encoder — `unindexed`
stays set until the index mark is crossed.

## Receive back-ends

Two capture strategies are implemented, selected at compile time. Both feed the
same Manchester → HDLC → CRC decoder.

Set in [`Core/Inc/yaskawa_hw.h`](Core/Inc/yaskawa_hw.h), or override with
`-DYASK_RX_METHOD=…`:

```c
#define YASK_RX_METHOD  YASK_RX_METHOD_SAMPLE    /* default */
```

### `YASK_RX_METHOD_SAMPLE` — oversampled GPIO (default)

TIM8 triggers DMA reads of `GPIOE->IDR` at **36 MHz** (216 MHz ÷ 6), giving
**4.5 samples per symbol**. The decoder re-locks its phase on every Manchester
transition, so the fractional sample rate and any encoder clock drift are
absorbed.

Deterministic and impossible to overrun; the raw buffer can be dumped for
offline analysis. Costs 4 KB of SRAM. **Start here.**

### `YASK_RX_METHOD_CAPTURE` — timer input capture

TIM1_CH2 captures both edges, DMA copies `CCR2` timestamps. Compact and
self-timing, but an edge is lost if the CCR is not read within one symbol
period. Useful for cross-checking the sampling back-end.

## Memory placement — important

DMA2 **cannot reach DTCM RAM** (`0x20000000`), where `.bss` normally lives on
the F7. A DMA buffer left in `.bss` produces no error — the transfer silently
moves nothing.

All DMA buffers are therefore placed in SRAM1 via a `.dma_buffer` section
defined in [`STM32F767ZITX_FLASH.ld`](STM32F767ZITX_FLASH.ld). Verify after
building:

```sh
arm-none-eabi-nm build/yaskawa-encoder.elf | grep -E "rx_samples|tx_bsrr"
# addresses must be 0x2002xxxx (SRAM1), not 0x2000xxxx (DTCM)
```

## Layout

```
Core/
  Inc/
    yaskawa_enc.h          protocol constants, frame struct, API
    yaskawa_hw.h           pin map, timer arithmetic, back-end selection
  Src/
    main.c                 demo loop, prints frames to the VCP
    yaskawa_enc.c          request/response sequencing
    yaskawa_tx.c           TX: bit table + TIM1/DMA transmit
    yaskawa_rx.c           RX: both capture back-ends + Manchester recovery
    yaskawa_decode.c       HDLC unstuffing, CRC-16/X.25, field extraction
    tim.c                  TIM1 (8 MHz TX) and TIM8 (36 MHz RX) + DMA links
    gpio.c dma.c usart.c   peripheral init
tools/
  debug/                   ST-Link flash, serial and GDB scripts
reference/
  yaskawa-incremental-encoder.ioc.orig  original CubeMX config (pins/clocks)
```

## Debugging

[`tools/debug/`](tools/debug/) drives the Nucleo's on-board ST-Link entirely
from the command line, using the tools bundled with STM32CubeIDE. Every script
is non-interactive and terminates on its own, so a debug session can be
scripted and its output read back.

```sh
./tools/debug/probe.sh          # is the ST-Link visible?
./tools/debug/flash.sh          # build + flash + reset
./tools/debug/serial.py 50      # read 50 lines of decoded frames
./tools/debug/gdb.sh "monitor halt" "bt"
./tools/debug/gdb.sh -f tools/debug/inspect.gdb   # full state dump
```

`inspect.gdb` halts in the decode path and prints the decoded frame, the raw
capture buffer, both timers' `CR1`/`ARR`/`DIER`, the DMA stream registers and
the GPIOE state — enough to tell wiring problems from capture problems from
decode problems in one shot.

See [tools/debug/README.md](tools/debug/README.md) for the full list and a
checklist for when frames do not decode.

## Glitch rejection

With the motor energised, PWM switching couples noise into the encoder and SPI
wiring. Measured on a shaft that was not moving:

| Motor state | `single_turn` spread | CRC |
|---|---|---|
| idle | 1–3 counts | 100/100 |
| armed, zero torque | 37–73 counts | 100/100 |

CRC passes throughout — the frames are intact, the reported angle itself moves.
Worse, the controller saw `pos_abs` swings of several hundred counts and read
them as wraps: `shadow_count` reached 15 623 114, inventing **953 revolutions**
on a stationary shaft.

[`yask_filter_accept()`](Core/Inc/yaskawa_enc.h) rejects a frame whose
`single_turn` moved further than `YASK_MAX_STEP_COUNTS` from the last accepted
one, measured the short way around the wrap so a reading either side of zero is
not mistaken for a jump. A run of rejections is accepted anyway after
`YASK_GLITCH_RUN_LIMIT` frames, so a genuine fast move is throttled rather than
blocked and the filter cannot latch onto a stale position.

The threshold trades noise rejection against the fastest motion passed. At a
600 Hz poll, 4096 counts (the default) passes about 1125 rpm; 2048 passes 560;
16384 passes 4500. The disturbances measured were around 1700 counts.

With the filter active the invented motion drops from 953 revolutions to
**15 counts** across the same test, with no errors raised.

**But the filter cannot fix the underlying problem, because the corruption
happens after our data leaves the STM32.** Measured with the motor armed at
zero torque:

| | angle14 |
|---|---|
| what we transmit | 16093–16098 (spread **5**) |
| what ODrive receives | 15811, 15813, 15915, 16097, 16098, 16323 (spread **512**) |

Comparing bit patterns against our transmitted 16097, the received words differ
in scattered bits — 15811 in four bits, 15915 in four, 16323 in three. That is
electrical noise on the SPI wiring, not a protocol or timing fault.

It also explains why `spi_error_rate` stays near zero: the AS5047P frame
carries a single even-parity bit, which only detects an **odd** number of
flipped bits. A frame corrupted in two or four bits passes parity and is
accepted as valid, so roughly half the corrupted frames get through
undetected.

**Shielding, routing and grounding are the only real fix.** A filter on the
STM32 side cannot help, because the data is already correct when it leaves.

### Ground topology

Both boards being connected to the same laptop by USB already ties their
grounds together. Adding a separate ground wire from the Nucleo to the supply
therefore **creates a loop** rather than removing one: Nucleo → USB → laptop →
USB → ODrive → supply ground → Nucleo. Each loop encloses area, picks up the
PWM magnetic field, and drives current through the SPI ground return — which is
the reference the receiver measures its signals against.

Measured effect of adding that wire, motor armed at zero torque:

| | before | after |
|---|---|---|
| STM32 `single_turn` spread | 40 | **20** |
| ODrive `pos_abs` spread | 512 | 512 |

Idle became perfect on both sides, so it helped the quiet case, but the armed
case is unchanged.

### Noise reduction, measured

`pos_abs` spread with the motor armed at zero torque, as each change was made:

| Change | Spread |
|---|---|
| starting point (mode 0, daisy-chained ground) | 512 counts |
| ground wire direct to the supply | 512 |
| **SPI mode 1** | 53 |
| **ground pin moved adjacent to the bus, wires bundled** | **12** |

12 counts of 16384 is 0.26°, against a 1-count quantisation floor of 0.022°.
The readings also progress smoothly now rather than jumping, which is what
genuine slow drift looks like.

Bundling the wires and taking the ground return from a pin next to the signals
both attack loop area, which is what couples the PWM field in — and mattered
more than the total length, which was already only 10 cm.

With that, position hold works: 0.0005 revolutions of drift over 3.5 seconds,
no errors.

**The bus is not clean, though.** Watching CS and SCK on the analyser rather
than only the decoded data shows CS transitioning **mid-frame** and SCK
transitioning **while CS is high**. A CS edge during a transfer resets the
slave's shift register, so the rest of that frame is garbage and the next one
starts misaligned. Spurious clocks while deselected mean the bus is picking up
energy even when idle.

That is worse than the 12-count data spread suggested: a corrupted clock or
chip select can produce a perfectly plausible-looking angle, so measuring only
the decoded value understates the problem. It also explains the 1-, 3- and
0-byte transactions in the armed capture.

**The internal pull resistors are far too weak for this.** The STM32's pulls
are about 40 kΩ, which holds a line against roughly 40 µA. A phase lead
slewing 54 V in 50 ns injects milliamps through even 1 pF of stray coupling —
a factor of 100 too much.

Firmware cannot help here: CS and SCK are inputs, and the SPI peripheral acts
on their edges directly. The remedies are all external:

| Fix | Parts | Effect |
|---|---|---|
| 2.2 kΩ pull-up on CS, pull-down on SCK, at the Nucleo | 2 resistors | ~20× stronger hold than the internal pulls |
| 100 Ω in series with CS and SCK, at the ODrive end | 2 resistors | damps reflections, slows edges |
| 100 pF from CS and SCK to ground, at the Nucleo | 2 capacitors | with the series resistor, a 16 MHz corner — passes the 1.3 MHz clock, attenuates nanosecond spikes |

The last two together form a standard RC filter and are the usual remedy.

Velocity control also still oscillates and trips OVERSPEED. That may be loop
tuning, but it cannot be judged properly until the bus is clean.

Logic-analyser captures settle it. With the motor **idle**, about 180
transactions all carry the identical word `0x3EE6`, matching `s_tx_word`
exactly, at a steady 8 kHz poll. With the motor **armed at zero torque**, the
same bus carries 20 distinct words in 40 sampled transactions, none of them
the transmitted value, and some transactions frame as one, three or zero bytes
instead of two.

The corrupted words are telling. The real word `0x3EE6` is
`0011111011100110`, with runs of five consecutive ones. What appears on the
wire is `0x8092`, `0x8004`, `0x0093` — mostly zeros with isolated ones. MISO
is not carrying attenuated data, it is being pulled toward one level. The
malformed transaction lengths say SCK is disturbed too, since spurious or
missing clock edges change how many bytes the analyser frames.

Meanwhile the STM32 side stays healthy throughout: `s_tx_word` correct, SPI
with no overrun and the FIFO loaded, both DMA streams running.

This is severe coupling rather than marginal noise, and it appeared on moving
from a 30 V bus to 54 V. No amount of firmware filtering can recover a signal
this badly disturbed — the SPI link needs shielding, separation from the phase
leads, and series damping before closed-loop control is workable at this
voltage.

## Sampling rate and aliasing

`single_turn` cannot represent more than **half a revolution** of motion
between two reads. Past that the shaft appears to take the short way round,
and the naive unwrap reports the wrong magnitude *and the wrong direction*.

Measured on a drill-driven shaft at ~321 RPM polled at 10 Hz: the unwrapped
position claimed **+26.7 revolutions** while the encoder's own counter said
**−31**. Every frame in that run still passed CRC — this is a sampling limit,
not a decode error.

| Poll rate | Unambiguous up to |
|---|---|
| 10 Hz (`HAL_Delay(100)`) | 300 RPM |
| 665 Hz (current: polling and printing every frame) | 19 950 RPM |
| 2 780 Hz (polling only, no console) | 83 400 RPM |

Two changes address this:

- **`yask_read()` no longer waits on `HAL_GetTick()`.** The 1 ms tick cost
  ~2 ms per read and capped polling near 500 Hz; spinning on the DMA counter
  instead brings a read to ~0.44 ms, so **2 290 Hz** is achievable.
- **[`yask_track_update()`](Core/Inc/yaskawa_enc.h)** combines `multi_turn`
  and `single_turn` into one absolute value modulo 16 revolutions and takes
  the shortest path. That tolerates **8 revolutions** between reads instead of
  half a revolution, and needs no special case for `multi_turn` stepping at
  290° rather than at the zero crossing.

Validated against both captured runs: +2.66 rev (slow, forward) and −30.29 rev
(fast, reverse) — matching the encoder's own revolution counters.

**The tracker is driven by `single_turn`, with `multi_turn` only as a
backstop.** An earlier version combined the two into one value modulo 16
revolutions and took the shortest path. That is wrong near the zero crossing:
`multi_turn` steps at ~290°, so for roughly 70° the two fields disagree about
which revolution the shaft is in. Over a 23 027-frame run that produced **122
phantom full-revolution steps** — one reported −0.999 rev where the shaft had
moved +0.0016 rev. They cancelled out over that run, so the total was right,
but any velocity from consecutive samples was corrupted. `single_turn` alone
is unambiguous below half a revolution per sample and showed a largest step of
3 450 counts (0.026 rev) with no jumps at all, so `multi_turn` is now
consulted only when a step approaches the half-revolution limit.

**The tracker still has a hard ceiling of half a revolution per sample.**
Combining `multi_turn` with `single_turn` widens the *unwrap* window to 8
revolutions, but only if each individual step is unambiguous. A drill run
sampled at 100 Hz put 119 of 971 steps near 0.9–1.1 revolutions, where the
shortest-path rule cannot tell forward from backward — the recorded position
wandered to −32.8 rev on a shaft that ended 1.1 rev from where it started.
The firmware tracker was verified correct against that log (0 mismatches in
972 frames recomputed offline); the limit is the sampling rate, not the code.
For a shaft that can reach a few thousand RPM, poll faster (the link sustains
~2 290 Hz) or drop the console printing, which dominates the loop.

## Measured rates

All figures from the on-target `benchmark()`, which the USER button runs.

| | Time | Rate |
|---|---|---|
| `yask_read()` | **0.36 ms** | **2 780 Hz** |
|  — capture window | 0.085 ms | 3 072 samples @ 36 MHz |
|  — decode | ~0.25 ms | scans the sample buffer |
| 96-char line @ 2 Mbaud | 0.48 ms | 2 083 lines/s |
| **read + print, measured** | **1.5 ms** | **665 Hz** |

Two things follow:

- **Decode dominates a read**, not the capture. It scans the whole sample
  buffer, so `YASK_RX_SAMPLES` sets the cost directly. The response is
  complete by sample 1884 (52.3 µs, measured), so the window was cut from
  6 144 to 3 072 samples — 60% headroom over the observed frame, and a read
  went from 0.43 ms to 0.36 ms.
- **Printing every frame is what limits throughput**, not the encoder link.
  At 2 Mbaud the console costs 0.48 ms against the read's 0.36 ms. Drop to
  printing every Nth frame, or none, to reach the full 2 780 Hz.

### Console baud rate

115200 was originally chosen while chasing corrupted output that turned out
to be STM32CubeIDE holding the port (see below), not a baud-rate problem.
Retested since: **2 Mbaud is clean** — 399 of 400 lines well-formed at
665 lines/s, the one exception being the reader attaching mid-line. 3 Mbaud
returns nothing at all, so the VCP's ceiling lies between them.

`serial.py` sets non-standard rates through the `IOSSIOSPEED` ioctl, since
macOS `termios` has no constant above `B230400` and `tcsetattr` rejects a raw
integer speed.

## Bring-up notes

Problems found and fixed while getting this working on hardware, in case they
recur:

- **`DI`/`RO` swapped on the transceiver.** The encoder stayed silent because
  it never heard the request.
- **`HAL_DMA_Start()` silently refusing to arm.** Nothing in the receive path
  polls the transfer, so the handle stayed `HAL_DMA_STATE_BUSY` and every
  later call returned `HAL_BUSY`. The buffer still looked plausible because
  the `memset` had just cleared it. Both paths now `HAL_DMA_Abort()` first and
  check the return.
- **`HAL_DMA_PollForTransfer()` costing ~62 µs** beyond the last symbol —
  four times the 15 µs frame. The encoder replies ~17 µs after the request, so
  the receiver was armed 14 µs into the response and the HDLC start flag was
  already gone (`NO_FLAG`). Replaced with a bounded `NDTR` spin.
- **Arming the sampler after transmitting.** Even a few microseconds of setup
  clipped the start of the frame. The capture window is now opened *before*
  the request goes out; it simply records our own transmission first, which
  the decoder skips while hunting for the flag.
- **Garbled console output is not a firmware fault.** If another process holds
  the VCP (STM32CubeIDE's serial terminal is the usual one) the two readers
  split the byte stream. `lsof /dev/cu.usbmodem*` shows the culprit, and
  `tools/debug/rate.gdb` measures the decode rate straight from RAM with the
  UART out of the picture.

## Next steps

1. Compare the two receive back-ends under motion for robustness.
2. Identify `raw40` (bits 40:56) and the unknown bits (`56`, `93:96`) against
   a known stimulus.
3. `unindexed` has not yet been seen to clear — establish what the encoder
   needs in order to consider itself indexed.

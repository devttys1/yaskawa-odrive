/*
 * Linker script for STM32F767ZI on the Nucleo-F767ZI.
 *
 * Memory layout note
 * ------------------
 * DTCM (0x20000000, 128K) is tightly coupled to the core and is NOT reachable
 * by the DMA2 controller.  Any buffer touched by DMA must therefore live in
 * SRAM1/SRAM2 (0x20020000).  The .dma_buffer section below exists for exactly
 * that purpose -- yaskawa_tx.c and yaskawa_rx.c place their buffers in it.
 */

ENTRY(Reset_Handler)

_estack    = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  ITCMRAM (xrw) : ORIGIN = 0x00000000, LENGTH = 16K
  DTCMRAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  RAM     (xrw) : ORIGIN = 0x20020000, LENGTH = 384K
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >DTCMRAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >DTCMRAM

  /*
   * DMA-accessible buffers.  Placed in SRAM1 because DMA2 has no path to
   * DTCM; a buffer left in .bss would silently transfer nothing.
   */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(32);
    _sdma_buffer = .;
    *(.dma_buffer)
    *(.dma_buffer*)
    . = ALIGN(32);
    _edma_buffer = .;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >DTCMRAM

  /DISCARD/ : { libc.a ( * ) libm.a ( * ) libgcc.a ( * ) }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}

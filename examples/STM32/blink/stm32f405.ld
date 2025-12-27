/*
 * Linker script for STM32F405RGT6
 * 1MB Flash, 192KB SRAM (128KB + 64KB CCM)
 */

/* Entry point */
ENTRY(Reset_Handler)

/* Memory layout */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (rwx)   : ORIGIN = 0x20000000, LENGTH = 128K
  CCM (rwx)   : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Stack size - grows downward from end of RAM */
_Min_Stack_Size = 0x400; /* 1KB */

/* End of RAM = top of stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Sections */
SECTIONS
{
  /* Vector table must be at beginning of flash */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))  /* Vector table */
    . = ALIGN(4);
  } >FLASH

  /* Program code and constants */
  .text :
  {
    . = ALIGN(4);
    *(.text)              /* Program code */
    *(.text*)
    *(.glue_7)            /* ARM/Thumb interworking */
    *(.glue_7t)
    *(.eh_frame)

    KEEP(*(.init))        /* Constructors */
    KEEP(*(.fini))

    . = ALIGN(4);
    _etext = .;           /* End of code section */
  } >FLASH

  /* Read-only data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* ARM exception unwinding info (can be removed to save space) */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH

  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  /* Initialized data - stored in flash, copied to RAM at startup */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;           /* Start of .data in RAM */
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;           /* End of .data in RAM */
  } >RAM AT>FLASH

  _sidata = LOADADDR(.data);  /* Start of .data in flash */

  /* Uninitialized data - zeroed at startup */
  .bss :
  {
    . = ALIGN(4);
    _sbss = .;            /* Start of .bss */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;            /* End of .bss */
    __bss_end__ = _ebss;
  } >RAM

  /* User heap (optional) */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE (end = .);
    PROVIDE (_end = .);
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* Remove debug info to save space (optional) */
  /DISCARD/ :
  {
    libc.a (*)
    libm.a (*)
    libgcc.a (*)
  }

  /* Debug sections */
  .ARM.attributes 0 : { *(.ARM.attributes) }
}


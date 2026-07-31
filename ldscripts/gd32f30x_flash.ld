/* memory map */
MEMORY
{
  /* Public target: M820 / GD32F303RCT6 / BL820. */
  FLASH    (rx)  : ORIGIN = 0x08005000, LENGTH = 230K
  CONFIG_A (rw)  : ORIGIN = 0x0803E800, LENGTH = 2K
  CONFIG_B (rw)  : ORIGIN = 0x0803F000, LENGTH = 2K
  SOC       (rw) : ORIGIN = 0x0803F800, LENGTH = 2K
  RAM      (xrw) : ORIGIN = 0x20000000, LENGTH = 48K
}

ENTRY(Reset_Handler)

PHDRS
{
  flash_text   PT_LOAD FLAGS(5); /* read + execute */
  flash_rodata PT_LOAD FLAGS(4); /* read only */
  ram_init     PT_LOAD FLAGS(6); /* initialized RAM, loaded from Flash */
  ram_zero     PT_LOAD FLAGS(6); /* zeroed/reserved RAM, no Flash payload */
}

SECTIONS
{
  __app_flash_start = ORIGIN(FLASH);
  __app_flash_limit = ORIGIN(FLASH) + LENGTH(FLASH);
  __config_a_start = ORIGIN(CONFIG_A);
  __config_b_start = ORIGIN(CONFIG_B);
  __soc_flash_start = ORIGIN(SOC);
  __physical_flash_end = ORIGIN(SOC) + LENGTH(SOC);

  __stack_size = DEFINED(__stack_size) ? __stack_size : 2K;
  __heap_size = DEFINED(__heap_size) ? __heap_size : 1K;

  /* ISR vectors */
  .vectors :
  {
    . = ALIGN(4);
    KEEP(*(.vectors))
    . = ALIGN(4);
    __Vectors_End = .;
    __Vectors_Size = __Vectors_End - __gVectors;
  } >FLASH :flash_text

  /* text section, such as program code */
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
    /* the symbol '_etext' will be defined at the end of code section */
    _etext = .;
  } >FLASH :flash_text

  /* ro-data section, such as constant data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH :flash_rodata

  /* exception process table */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } >FLASH :flash_rodata

  /* exception process table index */
  .ARM :
  {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH :flash_rodata

  /* ARM attributes */
  .ARM.attributes :
  {
   *(.ARM.attributes)
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH :flash_rodata

  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH :flash_rodata

  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH :flash_rodata

  /* provide some necessary symbols for initialized data */
  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    /* the symbol '_sdata' will be defined at the data section end start */
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    /* the symbol '_edata' will be defined at the data section end */
    _edata = .;
  } > RAM AT> FLASH :ram_init

  __flash_image_end = LOADADDR(.data) + SIZEOF(.data);

  /* provide some necessary symbols for uninitialized data */
  . = ALIGN(4);
  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    /* the symbol '_sbss' will be defined at the bss section start */
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    /* the symbol '_ebss' will be defined at the bss section end */
    _ebss = .;
    __bss_end__ = _ebss;
  } > RAM AT> RAM :ram_zero

  /* heap and stack space */
  .heap_stack (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE ( end = _ebss );
    PROVIDE ( _end = _ebss );
    . = . + __heap_size;
    PROVIDE( _heap_end = . );
    . = . + __stack_size;
    PROVIDE( _sp = . );
    . = ALIGN(8);
  } > RAM AT> RAM :ram_zero

  ASSERT(__flash_image_end <= __app_flash_limit,
    "Application image overlaps persistent Config A page")
  ASSERT(ORIGIN(CONFIG_A) + LENGTH(CONFIG_A) <= ORIGIN(CONFIG_B),
    "Config A overlaps Config B")
  ASSERT(ORIGIN(CONFIG_B) + LENGTH(CONFIG_B) <= ORIGIN(SOC),
    "Config B overlaps SOC page")
  ASSERT(ORIGIN(SOC) + LENGTH(SOC) <= 0x08040000,
    "Persistent storage exceeds physical Flash")
  ASSERT(_sp <= ORIGIN(RAM) + LENGTH(RAM),
    "Static RAM, heap and stack exceed physical RAM")
}

/* input sections */
GROUP(libgcc.a libc.a libm.a libnosys.a)

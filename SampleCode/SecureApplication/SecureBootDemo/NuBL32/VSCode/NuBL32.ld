__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x00000400;

/* ARMv8-M stack sealing:
   to use ARMv8-M stack sealing set __STACKSEAL_SIZE to 8 otherwise keep 0
 */
__STACKSEAL_SIZE = 8;

/* 0x400 is secure image header size. (It should be the same as -H value specified in FwSign script) */
__SIGNED_HDR_SIZE = DEFINED(SIGNED_HDR_SIZE) ? SIGNED_HDR_SIZE : 0x400;
__NUBL2_SIZE      = 0x40000;
/* Check FMC_SECURE_REGION_SIZE in partition_M55M1.h */
__FMC_SECURE_REGION_SIZE = 0x80000;

/* Linker script to configure memory regions. */
MEMORY
{
  FLASH (rx) : ORIGIN = (0x00000000 + __NUBL2_SIZE + __SIGNED_HDR_SIZE), LENGTH = (__FMC_SECURE_REGION_SIZE - __NUBL2_SIZE - __SIGNED_HDR_SIZE)
  RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 0x00008000    /* 32 KB */
}

/* Library configurations */
GROUP(libgcc.a libc.a libm.a libnosys.a)

/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 *
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __data_start__
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackLimit
 *   __StackTop
 *   __stack
 *   __Vectors_End
 *   __Vectors_Size
 */
ENTRY(Reset_Handler)

SECTIONS
{
	.text :
	{
    	KEEP(*(.vectors))
    	*startup_*.o(.text*)

		*(.text*)

		KEEP(*(.init))
		KEEP(*(.fini))

		/* .ctors */
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)

		/* .dtors */
 		*crtbegin.o(.dtors)
 		*crtbegin?.o(.dtors)
 		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
 		*(SORT(.dtors.*))
 		*(.dtors)

		*(.rodata*)

		KEEP(*(.eh_frame*))
	} > FLASH

    __start_NSC = ALIGN (32);
    .gnu.sgstubs :
    {
        . = ALIGN(4);
        *(.gnu.sgstubs.*)
    } > FLASH
    __end_NSC = .;

	.ARM.extab :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > FLASH

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > FLASH

	.copy.table :
	{
		. = ALIGN(4);
		__copy_table_start__ = .;
		LONG (__etext)
		LONG (__data_start__)
		LONG ((__data_end__ - __data_start__) / 4)
		__copy_table_end__ = .;
	} > FLASH

	.zero.table :
	{
		. = ALIGN(4);
		__zero_table_start__ = .;
		LONG (__bss_start__)
		LONG ((__bss_end__ - __bss_start__) / 4)
		__zero_table_end__ = .;
	} > FLASH

  /**
   * Location counter can end up 2 byte aligned with narrow Thumb code but
   * __etext is assumed by startup code to be the LMA of a section
   * which must be 4byte aligned
   */
	__etext = ALIGN (4);

	.data : AT (__etext)
	{
		__data_start__ = .;
		*(vtable)
		*(.data*)

		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);

		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);


		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);

		KEEP(*(.jcr*))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;

	} > RAM

	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
	} > RAM

  	.heap (NOLOAD) :
  	{
  	  . = ALIGN(8);
  	  __end__ = .;
  	  PROVIDE(end = .);
  	  . = . + __HEAP_SIZE;
  	  . = ALIGN(8);
  	  __HeapLimit = .;
  	} > RAM

  	.stack (ORIGIN(RAM) + LENGTH(RAM) - __STACK_SIZE - __STACKSEAL_SIZE) (NOLOAD) :
  	{
  	  . = ALIGN(8);
  	  __StackLimit = .;
  	  . = . + __STACK_SIZE;
  	  . = ALIGN(8);
  	  __StackTop = .;
  	} > RAM
  	PROVIDE(__stack = __StackTop);

    /* ARMv8-M stack sealing:
     to use ARMv8-M stack sealing uncomment '.stackseal' section
   */
    .stackseal (ORIGIN(RAM) + LENGTH(RAM) - __STACKSEAL_SIZE) (COPY) :
    {
        . = ALIGN(8);
        __StackSeal = .;
        . = . + __STACKSEAL_SIZE;
        . = ALIGN(8);
    } > RAM

	/* Check if data + heap + stack exceeds RAM limit */
	ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}

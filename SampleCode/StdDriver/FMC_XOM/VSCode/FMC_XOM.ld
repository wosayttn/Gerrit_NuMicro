/******************************************************************************
 * @file     FMC_XOM.ld
 * @brief    GNU Linker Script for the M2U51 FMC XOM sample
 * @version  V1.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 ******************************************************************************/

/*
 * Flash configuration
 */
__ROM_BASE = 0x00000000;
__ROM_SIZE = 0x00040000;

/*
 * The normal application image is limited to the area before XOM0.
 *
 * Normal Flash:
 *   0x00000000 - 0x0000FFFF
 *
 * XOM0:
 *   0x00010000 - 0x00010FFF
 */
__FLASH_BASE = __ROM_BASE;
__FLASH_SIZE = 0x00010000;

__XOM0_BASE = 0x00010000;
__XOM0_SIZE = 0x00001000;

/*
 * Embedded RAM configuration
 */
__RAM_BASE = 0x20000000;
__RAM_SIZE = 0x00008000;

/*
 * Stack and heap configuration
 */
__STACK_SIZE = 0x00000200;
__HEAP_SIZE  = 0x00000200;

/*
 * ARMv8-M stack sealing.
 * Set this value to 8 when stack sealing is required.
 */
__STACKSEAL_SIZE = 0;


/*
 * Memory regions
 */
MEMORY
{
  FLASH (rx)  : ORIGIN = __FLASH_BASE, LENGTH = __FLASH_SIZE
  XOM0  (rx)  : ORIGIN = __XOM0_BASE,   LENGTH = __XOM0_SIZE
  RAM   (rwx) : ORIGIN = __RAM_BASE,    LENGTH = __RAM_SIZE
}


/*
 * Application entry point
 */
ENTRY(Reset_Handler)


SECTIONS
{
  /*
   * XOM code
   *
   * This output section must be placed before the generic .text output
   * section. Otherwise, the generic *(.text*) selector will consume
   * .text.Lib_XOM_ADD before it can be placed in XOM0.
   */
  .xom0 ORIGIN(XOM0) :
  {
    . = ALIGN(4);

    __xom0_start__ = .;
    __XOM0_Base = .;

    /*
     * Keep all executable sections generated from xom_add.c.
     *
     * The leading wildcard permits matching a CMake object path such as:
     * CMakeFiles/.../xom_add.c.obj
     */
    KEEP(*xom_add.c.obj(.text))
    KEEP(*xom_add.c.obj(.text.*))
    KEEP(*xom_add.c.obj(.rodata))
    KEEP(*xom_add.c.obj(.rodata.*))

    . = ALIGN(4);

    __xom0_end__ = .;
    __XOM0_Limit = .;
  } > XOM0


  /*
   * Normal executable code and readonly data
   */
  .text :
  {
    KEEP(*(.vectors))

    /*
     * The xom_add.c.obj sections have already been consumed by .xom0.
     */
    *(.text)
    *(.text.*)
    *(.gnu.linkonce.t.*)

    KEEP(*(.init))
    KEEP(*(.fini))

    /*
     * C constructors
     */
    *crtbegin.o(.ctors)
    *crtbegin?.o(.ctors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
    *(SORT(.ctors.*))
    *(.ctors)

    /*
     * C destructors
     */
    *crtbegin.o(.dtors)
    *crtbegin?.o(.dtors)
    *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
    *(SORT(.dtors.*))
    *(.dtors)

    /*
     * Readonly data
     */
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)

    KEEP(*(.eh_frame*))

    . = ALIGN(4);
  } > FLASH


  /*
   * Secure Gateway veneers
   *
   * Enable this section if CMSE Secure Gateway veneers are required.
   */
  /*
  .gnu.sgstubs :
  {
    . = ALIGN(32);
  } > FLASH
  */


  /*
   * ARM exception unwinding tables
   */
  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH


  __exidx_start = .;

  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH

  __exidx_end = .;


  /*
   * Data initialization copy table
   */
  .copy.table (READONLY) :
  {
    . = ALIGN(4);

    __copy_table_start__ = .;

    LONG(LOADADDR(.data))
    LONG(ADDR(.data))
    LONG(SIZEOF(.data) / 4)

    /*
     * Add additional initialized data sections here when required.
     *
     * LONG(LOADADDR(.data2))
     * LONG(ADDR(.data2))
     * LONG(SIZEOF(.data2) / 4)
     */

    __copy_table_end__ = .;
  } > FLASH


  /*
   * Zero initialization table
   */
  .zero.table (READONLY) :
  {
    . = ALIGN(4);

    __zero_table_start__ = .;

    /*
     * The default startup code already initializes .bss to zero.
     *
     * LONG(ADDR(.bss))
     * LONG(SIZEOF(.bss) / 4)
     */

    /*
     * Add additional zero-initialized sections here when required.
     *
     * LONG(ADDR(.bss2))
     * LONG(SIZEOF(.bss2) / 4)
     */

    __zero_table_end__ = .;
  } > FLASH


  /*
   * Retained for compatibility with startup implementations that use
   * __etext as the load address of initialized data.
   */
  PROVIDE(__etext = LOADADDR(.data));


  /*
   * Stack at the beginning of RAM
   */
  .stack ORIGIN(RAM) (NOLOAD) :
  {
    . = ALIGN(8);

    __StackLimit = .;

    . = . + __STACK_SIZE;
    . = ALIGN(8);

    __StackTop = .;
  } > RAM

  PROVIDE(__stack = __StackTop);


  /*
   * ARMv8-M stack seal immediately after the stack
   */
  .stackseal (NOLOAD) :
  {
    . = ALIGN(8);

    __StackSeal = .;

    . = . + __STACKSEAL_SIZE;
    . = ALIGN(8);
  } > RAM


  /*
   * Initialized data
   */
  .data : ALIGN(4)
  {
    __data_start__ = .;

    *(vtable)

    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)

    . = ALIGN(4);

    /*
     * C/C++ pre-initialization array
     */
    PROVIDE_HIDDEN(__preinit_array_start = .);

    KEEP(*(.preinit_array))

    PROVIDE_HIDDEN(__preinit_array_end = .);

    . = ALIGN(4);

    /*
     * C/C++ initialization array
     */
    PROVIDE_HIDDEN(__init_array_start = .);

    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))

    PROVIDE_HIDDEN(__init_array_end = .);

    . = ALIGN(4);

    /*
     * C/C++ finalization array
     */
    PROVIDE_HIDDEN(__fini_array_start = .);

    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array))

    PROVIDE_HIDDEN(__fini_array_end = .);

    KEEP(*(.jcr*))

    . = ALIGN(4);

    __data_end__ = .;
  } > RAM AT > FLASH


  /*
   * Optional secondary initialized data section
   */
  /*
  .data2 : ALIGN(4)
  {
    . = ALIGN(4);

    __data2_start__ = .;

    *(.data2)
    *(.data2.*)

    . = ALIGN(4);

    __data2_end__ = .;
  } > RAM2 AT > FLASH
  */


  /*
   * Zero-initialized data
   */
  .bss :
  {
    . = ALIGN(4);

    __bss_start__ = .;

    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)

    . = ALIGN(4);

    __bss_end__ = .;
  } > RAM AT > RAM


  /*
   * Optional secondary zero-initialized data section
   */
  /*
  .bss2 :
  {
    . = ALIGN(4);

    __bss2_start__ = .;

    *(.bss2)
    *(.bss2.*)

    . = ALIGN(4);

    __bss2_end__ = .;
  } > RAM2 AT > RAM2
  */


  /*
   * Heap
   */
  .heap (NOLOAD) :
  {
    . = ALIGN(8);

    __end__ = .;
    PROVIDE(end = .);

    . = . + __HEAP_SIZE;
    . = ALIGN(8);

    __HeapLimit = .;
  } > RAM


  /*
   * Discard metadata that is not required in the final embedded image.
   */
  /DISCARD/ :
  {
    *(.note.GNU-stack)
  }


  /*
   * Link-time validation
   */
  ASSERT(
    __HeapLimit <= (ORIGIN(RAM) + LENGTH(RAM)),
    "RAM overflow: data, bss and heap exceed the RAM region"
  )

  ASSERT(
    __StackTop <= (ORIGIN(RAM) + LENGTH(RAM)),
    "RAM overflow: stack exceeds the RAM region"
  )

  ASSERT(
    SIZEOF(.xom0) > 0,
    "XOM0 section is empty"
  )

  ASSERT(
    SIZEOF(.xom0) <= LENGTH(XOM0),
    "XOM0 region overflowed"
  )

  ASSERT(
    ADDR(.xom0) == ORIGIN(XOM0),
    "XOM0 section start address is incorrect"
  )

  ASSERT(
    __xom0_end__ <= (ORIGIN(XOM0) + LENGTH(XOM0)),
    "XOM0 content exceeds the configured XOM0 region"
  )
}
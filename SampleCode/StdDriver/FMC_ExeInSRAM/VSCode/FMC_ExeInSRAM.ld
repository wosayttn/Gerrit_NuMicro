/*
 * FMC_ExeInSRAM_gcc.ld
 * GNU ld linker script converted from FMC_ExeInSRAM.sct
 * Target: Cortex-M23
 *
 * Memory layout:
 *   FLASH : 0x00000000 - 0x0003FFFF (256 KB)
 *   RAM0  : 0x20000000 - 0x20003FFF ( 16 KB) : fmc.o RO executes here
 *   RAM1  : 0x20004000 - 0x20007FFF ( 16 KB) : RW/ZI, heap, stack
 *
 * fmc.o has VMA in RAM0 and LMA in FLASH. This script emits a copy
 * table for both .fmc_ram_code and .data. Pair this file with
 * startup_M2U51_FMC_ExeInSRAM.S so both sections are copied at reset.
 */

ENTRY(Reset_Handler)

FLASH_SIZE = DEFINED(FLASH_SIZE) ? FLASH_SIZE : 0x00040000;
SRAM_SIZE  = DEFINED(SRAM_SIZE)  ? SRAM_SIZE  : 0x00008000;
STACK_SIZE = DEFINED(STACK_SIZE) ? STACK_SIZE : 0x00000200;
HEAP_SIZE  = DEFINED(HEAP_SIZE)  ? HEAP_SIZE  : 0x00000200;

FLASH_BASE = 0x00000000;
SRAM_BASE  = 0x20000000;
SRAM_SIZE0 = SRAM_SIZE / 2;
SRAM_SIZE1 = SRAM_SIZE / 2;

MEMORY
{
    FLASH (rx)  : ORIGIN = FLASH_BASE,             LENGTH = FLASH_SIZE
    RAM0  (rwx) : ORIGIN = SRAM_BASE,              LENGTH = SRAM_SIZE0
    RAM1  (rwx) : ORIGIN = SRAM_BASE + SRAM_SIZE0, LENGTH = SRAM_SIZE1
}

SECTIONS
{
    /* Vector table / reset section must be first in FLASH. */
    .vectors :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        KEEP(*(.vectors))
        KEEP(*(.intvec))
        KEEP(*(RESET))
        . = ALIGN(4);
    } > FLASH

    /*
     * FMC driver executes from RAM0, but its initialization image is stored
     * in FLASH. Put this output section before the generic .text/.rodata
     * selectors so fmc.o is consumed here first.
     */
    .fmc_ram_code :
    {
        . = ALIGN(4);
        __fmc_start__ = .;
        *fmc.o(.text .text.*)
        *fmc.o(.rodata .rodata.*)
        *fmc.o(.constdata .constdata.*)
        . = ALIGN(4);
        __fmc_end__ = .;
    } > RAM0 AT > FLASH

    __fmc_load_start__ = LOADADDR(.fmc_ram_code);
    __fmc_load_end__   = LOADADDR(.fmc_ram_code) + SIZEOF(.fmc_ram_code);

    /* All remaining executable/read-only content stays in FLASH. */
    .text :
    {
        . = ALIGN(4);
        *(.text .text.*)
        *(.rodata .rodata.*)
        *(.constdata .constdata.*)
        KEEP(*(.init))
        KEEP(*(.fini))
        . = ALIGN(4);
    } > FLASH

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

    /* C/C++ initialization arrays. */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    /*
     * Startup copy table. startup_M2U51_FMC_ExeInSRAM.S walks this table
     * when __STARTUP_COPY_MULTIPLE is enabled. Each entry is:
     *   { LMA, VMA, size }
     */
    .copy.table :
    {
        . = ALIGN(4);
        __copy_table_start__ = .;
        LONG(LOADADDR(.fmc_ram_code));
        LONG(ADDR(.fmc_ram_code));
        LONG(SIZEOF(.fmc_ram_code));
        LONG(LOADADDR(.data));
        LONG(ADDR(.data));
        LONG(SIZEOF(.data));
        __copy_table_end__ = .;
        . = ALIGN(4);
    } > FLASH

    /* Initialized data executes in RAM1 and is loaded from FLASH. */
    .data :
    {
        . = ALIGN(4);
        __data_start__ = .;
        _sdata = .;
        *(.data .data.*)
        *(.RamFunc .RamFunc.*)
        . = ALIGN(4);
        __data_end__ = .;
        _edata = .;
    } > RAM1 AT > FLASH

    __data_load__ = LOADADDR(.data);
    _sidata = LOADADDR(.data);

    /* Zero-initialized data in RAM1. */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        _sbss = .;
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
        _ebss = .;
    } > RAM1

    .noinit (NOLOAD) :
    {
        . = ALIGN(4);
        *(.noinit .noinit.*)
        . = ALIGN(4);
    } > RAM1

    /* Heap starts after RW/ZI, matching the scatter file's AlignExpr(+0, 8). */
    .heap (NOLOAD) :
    {
        . = ALIGN(8);
        __HeapBase = .;
        . += HEAP_SIZE;
        __HeapLimit = .;
    } > RAM1

    /* Stack is reserved at the top of SRAM. */
    __StackTop   = ORIGIN(RAM1) + LENGTH(RAM1);
    __StackLimit = __StackTop - STACK_SIZE;
    PROVIDE(__stack = __StackTop);

    ASSERT(__HeapLimit <= __StackLimit,
           "RAM1 overflow: RW/ZI/heap overlaps stack");

    ASSERT(SIZEOF(.fmc_ram_code) <= LENGTH(RAM0),
           "RAM0 overflow: fmc.o RO does not fit in RAM0");
}

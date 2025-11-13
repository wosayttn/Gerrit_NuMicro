/****************************************************************************//**
 * @file     startup_M2U51.S
 * @version  V1.00
 * @brief    CMSIS Cortex-M23 Core Device Startup File for M2U51
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

	.syntax	unified
	.arch	armv8-m.base

	.section .stack
	.align	3
#ifdef __STACK_SIZE
	.equ	Stack_Size, __STACK_SIZE
#else
	.equ	Stack_Size, 0x00000400
#endif
	.globl	__StackTop
	.globl	__StackLimit
__StackLimit:
	.space	Stack_Size
	.size	__StackLimit, . - __StackLimit
__StackTop:
	.size	__StackTop, . - __StackTop

	.section .heap
	.align	3
#ifdef __HEAP_SIZE
	.equ	Heap_Size, __HEAP_SIZE
#else
	.equ	Heap_Size, 0x00000100
#endif
	.globl	__HeapBase
	.globl	__HeapLimit
__HeapBase:
	.if	Heap_Size
	.space	Heap_Size
	.endif
	.size	__HeapBase, . - __HeapBase
__HeapLimit:
	.size	__HeapLimit, . - __HeapLimit

	.section .vectors
	.align	2
	.globl	__Vectors
__Vectors:
	.long	__StackTop              /* Top of Stack                  */
	.long	Reset_Handler           /* Reset Handler                 */
	.long	NMI_Handler             /* NMI Handler                   */
	.long	HardFault_Handler       /* Hard Fault Handler            */
	.long	MemManage_Handler       /* MPU Fault Handler             */
	.long	BusFault_Handler        /* Bus Fault Handler             */
	.long	UsageFault_Handler      /* Usage Fault Handler           */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	SVC_Handler             /* SVCall Handler                */
	.long	0                       /* Reserved                      */
	.long	0                       /* Reserved                      */
	.long	PendSV_Handler          /* PendSV Handler                */
	.long	SysTick_Handler         /* SysTick Handler               */

	/* External interrupts */
    .long   BOD_IRQHandler          /* 0: Brown Out detection       */
    .long   IRC_IRQHandler          /* 1: Internal RC               */
    .long   PWRWU_IRQHandler        /* 2: Power down wake up        */
    .long   Default_Handler         /* 3:                           */
    .long   CLKFAIL_IRQHandler      /* 4: Clock detection fail      */
    .long   FMC_IRQHandler          /* 5: FMC (ISP)                 */
    .long   RTC_IRQHandler          /* 6: Real Time Clock           */
    .long   Default_Handler         /* 7:                           */
    .long   WDT_IRQHandler          /* 8: Watchdog timer            */
    .long   WWDT_IRQHandler         /* 9: Window watchdog timer     */
    .long   EINT0_IRQHandler        /* 10: External Input 0         */
    .long   EINT1_IRQHandler        /* 11: External Input 1         */
    .long   EINT2_IRQHandler        /* 12: External Input 2         */
    .long   EINT3_IRQHandler        /* 13: External Input 3         */
    .long   EINT4_IRQHandler        /* 14: External Input 4         */
    .long   EINT5_IRQHandler        /* 15: External Input 5         */
    .long   GPA_IRQHandler          /* 16: GPIO Port A              */
    .long   GPB_IRQHandler          /* 17: GPIO Port B              */
    .long   GPC_IRQHandler          /* 18: GPIO Port C              */
    .long   GPD_IRQHandler          /* 19: GPIO Port D              */
    .long   GPE_IRQHandler          /* 20: GPIO Port E              */
    .long   GPF_IRQHandler          /* 21: GPIO Port F              */
    .long   ETI_IRQHandler          /* 22: ETI                      */
    .long   SPI0_IRQHandler         /* 23: SPI0                     */
    .long   GPG_IRQHandler          /* 24: GPIO Port G              */
    .long   EINT6_IRQHandler        /* 25: External Input 6         */
    .long   BRAKE0_IRQHandler       /* 26: BRAKE0                   */
    .long   PWM0P0_IRQHandler       /* 27: PWM0P0                   */
    .long   PWM0P1_IRQHandler       /* 28: PWM0P1                   */
    .long   PWM0P2_IRQHandler       /* 29: PWM0P2                   */
    .long   TMR0_IRQHandler         /* 30: Timer 0                  */
    .long   TMR1_IRQHandler         /* 31: Timer 1                  */
    .long   TMR2_IRQHandler         /* 32: Timer 2                  */
    .long   TMR3_IRQHandler         /* 33: Timer 3                  */
    .long   UART0_IRQHandler        /* 34: UART0                    */
    .long   UART1_IRQHandler        /* 35: UART1                    */
    .long   I2C0_IRQHandler         /* 36: I2C0                     */
    .long   I2C1_IRQHandler         /* 37: I2C1                     */
    .long   PDMA0_IRQHandler        /* 38: Peripheral DMA 0         */
    .long   Default_Handler         /* 39:                          */

    .long   ADC0_INT0_IRQHandler    /* 40: ADC0 interrupt source 0  */
    .long   Default_Handler         /* 41:                          */
    .long   ACMP01_IRQHandler       /* 42: ACMP0 and ACMP1          */
    .long   BPWM0_IRQHandler        /* 43: BPWM0                    */
    .long   GPH_IRQHandler          /* 44: GPIO Port H              */
    .long   EINT7_IRQHandler        /* 45: External Input 7         */
    .long   UART2_IRQHandler        /* 46: UART2                    */
    .long   Default_Handler         /* 47:                          */
    .long   USCI0_IRQHandler        /* 48: USCI0                    */
    .long   SPI1_IRQHandler         /* 49: SPI1                     */
    .long   SPI2_IRQHandler         /* 50: SPI2                     */
    .long   Default_Handler         /* 51:                          */
    .long   Default_Handler         /* 52:                          */
    .long   Default_Handler         /* 53:                          */
    .long   Default_Handler         /* 54:                          */
    .long   CRPT_IRQHandler         /* 55: CRYPTO                   */
    .long   Default_Handler         /* 56:                          */
    .long   I2C2_IRQHandler         /* 57: I2C2                     */
    .long   Default_Handler         /* 58:                          */
    .long   LCD_IRQHandler          /* 59: LCD                      */
    .long   CRC0_IRQHandler         /* 60: CRC0                     */

	.size	__Vectors, . - __Vectors

	.text
	.thumb
	.thumb_func
	.align	2
	.globl	Reset_Handler
	.type	Reset_Handler, %function
Reset_Handler:
/*  Firstly it copies data from read only memory to RAM. There are two schemes
 *  to copy. One can copy more than one sections. Another can only copy
 *  one section.  The former scheme needs more instructions and read-only
 *  data to implement than the latter.
 *  Macro __STARTUP_COPY_MULTIPLE is used to choose between two schemes.  */

#ifdef __STARTUP_COPY_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of triplets, each of which specify:
 *    offset 0: LMA of start of a section to copy from
 *    offset 4: VMA of start of a section to copy to
 *    offset 8: size of the section to copy. Must be multiply of 4
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r4, =__copy_table_start__
	ldr	r5, =__copy_table_end__

.L_loop0:
	cmp	r4, r5
	bge	.L_loop0_done
	ldr	r1, [r4]
	ldr	r2, [r4, #4]
	ldr	r3, [r4, #8]

.L_loop0_0:
	subs	r3, #4
	blt	.L_loop0_0_done
	ldr	r0, [r1,r3]
	str	r0, [r2,r3]
	b	.L_loop0_0

.L_loop0_0_done:
	adds	r4, #12
	b	.L_loop0

.L_loop0_done:
#else
/*  Single section scheme.
 *
 *  The ranges of copy from/to are specified by following symbols
 *    __etext: LMA of start of the section to copy from. Usually end of text
 *    __data_start__: VMA of start of the section to copy to
 *    __data_end__: VMA of end of the section to copy to
 *
 *  All addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__etext
	ldr	r2, =__data_start__
	ldr	r3, =__data_end__

	subs	r3, r2
	ble	.L_loop1_done

.L_loop1:
	subs	r3, #4
	ldr	r0, [r1,r3]
	str	r0, [r2,r3]
	bgt	.L_loop1

.L_loop1_done:

#endif /*__STARTUP_COPY_MULTIPLE */

/*  This part of work usually is done in C library startup code. Otherwise,
 *  define this macro to enable it in this startup.
 *
 *  There are two schemes too. One can clear multiple BSS sections. Another
 *  can only clear one section. The former is more size expensive than the
 *  latter.
 *
 *  Define macro __STARTUP_CLEAR_BSS_MULTIPLE to choose the former.
 *  Otherwise define macro __STARTUP_CLEAR_BSS to choose the later.
 */
#ifdef __STARTUP_CLEAR_BSS_MULTIPLE
/*  Multiple sections scheme.
 *
 *  Between symbol address __copy_table_start__ and __copy_table_end__,
 *  there are array of tuples specifying:
 *    offset 0: Start of a BSS section
 *    offset 4: Size of this BSS section. Must be multiply of 4
 */
	ldr	r3, =__zero_table_start__
	ldr	r4, =__zero_table_end__

.L_loop2:
	cmp	r3, r4
	bge	.L_loop2_done
	ldr	r1, [r3]
	ldr	r2, [r3, #4]
	movs	r0, 0

.L_loop2_0:
	subs	r2, #4
	str 	r0, [r1, r2]
	bgt	.L_loop2_0

	adds	r3, #8
	b	.L_loop2
.L_loop2_done:

#elif defined (__STARTUP_CLEAR_BSS)
/*  Single BSS section scheme.
 *
 *  The BSS section is specified by following symbols
 *    __bss_start__: start of the BSS section.
 *    __bss_end__: end of the BSS section.
 *
 *  Both addresses must be aligned to 4 bytes boundary.
 */
	ldr	r1, =__bss_start__
	ldr	r2, =__bss_end__

    movs    r0, 0

    subs    r2, r1
    ble .L_loop3_done

.L_loop3:
    subs    r2, #4
    str r0, [r1, r2]
    bgt .L_loop3
.L_loop3_done:
#endif /* __STARTUP_CLEAR_BSS_MULTIPLE || __STARTUP_CLEAR_BSS */



#ifndef __NO_SYSTEM_INIT
	bl	SystemInit
#endif


#ifndef __START
#define __START _start
#endif
	bl	__START

	.pool
	.size	Reset_Handler, . - Reset_Handler

	.align	1
	.thumb_func
	.weak	Default_Handler
	.type	Default_Handler, %function
Default_Handler:
	b	.
	.size	Default_Handler, . - Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
	.macro	def_irq_handler	handler_name
	.weak	\handler_name
	.set	\handler_name, Default_Handler
	.endm

	def_irq_handler	NMI_Handler
    def_irq_handler	HardFault_Handler
    def_irq_handler	MemManage_Handler
    def_irq_handler	BusFault_Handler
    def_irq_handler	UsageFault_Handler
    def_irq_handler	SVC_Handler
    def_irq_handler	PendSV_Handler
    def_irq_handler	SysTick_Handler

    def_irq_handler BOD_IRQHandler
    def_irq_handler IRC_IRQHandler
    def_irq_handler PWRWU_IRQHandler
    def_irq_handler CLKFAIL_IRQHandler
    def_irq_handler FMC_IRQHandler
    def_irq_handler RTC_IRQHandler
    def_irq_handler WDT_IRQHandler
    def_irq_handler WWDT_IRQHandler
    def_irq_handler EINT0_IRQHandler
    def_irq_handler EINT1_IRQHandler
    def_irq_handler EINT2_IRQHandler
    def_irq_handler EINT3_IRQHandler
    def_irq_handler EINT4_IRQHandler
    def_irq_handler EINT5_IRQHandler
    def_irq_handler GPA_IRQHandler
    def_irq_handler GPB_IRQHandler
    def_irq_handler GPC_IRQHandler
    def_irq_handler GPD_IRQHandler
    def_irq_handler GPE_IRQHandler
    def_irq_handler GPF_IRQHandler
    def_irq_handler ETI_IRQHandler
    def_irq_handler SPI0_IRQHandler
    def_irq_handler GPG_IRQHandler
    def_irq_handler EINT6_IRQHandler
    def_irq_handler BRAKE0_IRQHandler
    def_irq_handler PWM0P0_IRQHandler
    def_irq_handler PWM0P1_IRQHandler
    def_irq_handler PWM0P2_IRQHandler
    def_irq_handler TMR0_IRQHandler
    def_irq_handler TMR1_IRQHandler
    def_irq_handler TMR2_IRQHandler
    def_irq_handler TMR3_IRQHandler
    def_irq_handler UART0_IRQHandler
    def_irq_handler UART1_IRQHandler
    def_irq_handler I2C0_IRQHandler
    def_irq_handler I2C1_IRQHandler
    def_irq_handler PDMA0_IRQHandler
    def_irq_handler ADC0_INT0_IRQHandler
    def_irq_handler ACMP01_IRQHandler
    def_irq_handler BPWM0_IRQHandler
    def_irq_handler GPH_IRQHandler
    def_irq_handler EINT7_IRQHandler
    def_irq_handler UART2_IRQHandler
    def_irq_handler USCI0_IRQHandler
    def_irq_handler SPI1_IRQHandler
    def_irq_handler SPI2_IRQHandler
    def_irq_handler CRPT_IRQHandler
    def_irq_handler I2C2_IRQHandler
    def_irq_handler LCD_IRQHandler
    def_irq_handler CRC0_IRQHandler

    .end

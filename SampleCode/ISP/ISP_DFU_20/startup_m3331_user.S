/**************************************************************************//**
 * @file     startup_m3331_user.S
 * @version  V3.00
 * @brief    M3331 Series Startup Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*
//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
*/



    .section .bss.STACK, "aw", %nobits
    .align 3
    .global __initial_sp
#ifndef Stack_Size
// <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Stack_Size, 0x00001000
#endif
Stack_Mem:
    .space   Stack_Size
__initial_sp:


    .section .bss.HEAP, "aw", %nobits
    .align  3
    .global Heap_Mem
    .global __heap_base
    .global __heap_limit
#ifndef Heap_Size
// <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
    .equ    Heap_Size, 0x00000100
#endif
__heap_base:
Heap_Mem:
    .space  Heap_Size
__heap_limit:

    .eabi_attribute Tag_ABI_align_preserved, 1
    .thumb


// ; Vector Table Mapped to Address 0 at Reset
    .section RESET, "ax"
    .global     __Vectors
    .global     __Vectors_End
    .global     __Vectors_Size

__Vectors:
    .word     __initial_sp              // ; Top of Stack
    .word     Reset_Handler             // ; Reset Handler
    .word     NMI_Handler               // ; NMI Handler
    .word     HardFault_Handler         // ; Hard Fault Handler
    .word     MemManage_Handler         // ; MPU Fault Handler
    .word     BusFault_Handler          // ; Bus Fault Handler
    .word     UsageFault_Handler        // ; Usage Fault Handler
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     0                         // ; Reserved
    .word     SVC_Handler               // ; SVCall Handler
    .word     DebugMon_Handler          // ; Debug Monitor Handler
    .word     0                         // ; Reserved
    .word     PendSV_Handler            // ; PendSV Handler
    .word     SysTick_Handler           // ; SysTick Hand

    // // ; External Interrupts


__Vectors_End:
    .equ    __Vectors_Size, __Vectors_End - __Vectors

    .section .text, "ax"

// ; Reset Handler

    .global Reset_Handler
    .global  SystemInit
    .global  __main
    .type   Reset_Handler, "function"
Reset_Handler:
        LDR     R0, =SystemInit
        BLX     R0
        LDR     R0, =__main
        BX      R0


// ; Dummy Exception Handlers (infinite loops which can be modified)

    .weak   NMI_Handler
    .type   NMI_Handler, "function"
NMI_Handler:
        B       .

    .weak   HardFault_Handler
    .type   HardFault_Handler, "function"
HardFault_Handler:
        B       .

    .weak   MemManage_Handler, "function"
MemManage_Handler:
                B       .

    .weak   BusFault_Handler, "function"
BusFault_Handler:
                B       .


    .weak   UsageFault_Handler, "function"
UsageFault_Handler:
                B       .

    .weak   SVC_Handler, "function"
SVC_Handler:
        B       .

    .weak   DebugMon_Handler, "function"
DebugMon_Handler:
                B       .

    .weak   PendSV_Handler, "function"
PendSV_Handler:
                B       .

    .weak   SysTick_Handler, "function"
SysTick_Handler:
                B       .

    .weak  Default_Handler, "function"
    .weak  BOD_IRQHandler, "function"
    .weak  IRC_IRQHandler, "function"
    .weak  PWRWU_IRQHandler, "function"
    .weak  RAMPE_IRQHandler, "function"
    .weak  CKFAIL_IRQHandler, "function"
    .weak  ISP_IRQHandler, "function"
    .weak  RTC_IRQHandler, "function"
    .weak  WDT0_IRQHandler, "function"
    .weak  WWDT0_IRQHandler, "function"
    .weak  EINT0_IRQHandler, "function"
    .weak  EINT1_IRQHandler, "function"
    .weak  EINT2_IRQHandler, "function"
    .weak  EINT3_IRQHandler, "function"
    .weak  EINT4_IRQHandler, "function"
    .weak  EINT5_IRQHandler, "function"
    .weak  GPA_IRQHandler, "function"
    .weak  GPB_IRQHandler, "function"
    .weak  GPC_IRQHandler, "function"
    .weak  GPD_IRQHandler, "function"
    .weak  GPE_IRQHandler, "function"
    .weak  GPF_IRQHandler, "function"
    .weak  QSPI0_IRQHandler, "function"
    .weak  SPI0_IRQHandler, "function"
    .weak  BRAKE0_IRQHandler, "function"
    .weak  EPWM0P0_IRQHandler, "function"
    .weak  EPWM0P1_IRQHandler, "function"
    .weak  EPWM0P2_IRQHandler, "function"
    .weak  BRAKE1_IRQHandler, "function"
    .weak  EPWM1P0_IRQHandler, "function"
    .weak  EPWM1P1_IRQHandler, "function"
    .weak  EPWM1P2_IRQHandler, "function"
    .weak  TMR0_IRQHandler, "function"
    .weak  TMR1_IRQHandler, "function"
    .weak  TMR2_IRQHandler, "function"
    .weak  TMR3_IRQHandler, "function"
    .weak  UART0_IRQHandler, "function"
    .weak  UART1_IRQHandler, "function"
    .weak  I2C0_IRQHandler, "function"
    .weak  I2C1_IRQHandler, "function"
    .weak  PDMA0_IRQHandler, "function"
    .weak  EADC00_IRQHandler, "function"
    .weak  EADC01_IRQHandler, "function"
    .weak  ACMP01_IRQHandler, "function"
    .weak  EADC02_IRQHandler, "function"
    .weak  EADC03_IRQHandler, "function"
    .weak  UART2_IRQHandler, "function"
    .weak  UART3_IRQHandler, "function"
    .weak  SPI1_IRQHandler, "function"
    .weak  SPI2_IRQHandler, "function"
    .weak  OHCI_IRQHandler, "function"
    .weak  ETI_IRQHandler, "function"
    .weak  CRC_IRQHandler, "function"
    .weak  NS_ISP_IRQHandler, "function"
    .weak  SCU_IRQHandler, "function"
    .weak  SDH0_IRQHandler, "function"
    .weak  USBD20_IRQHandler, "function"
    .weak  WDT1_IRQHandler, "function"
    .weak  I2S0_IRQHandler, "function"
    .weak  GPG_IRQHandler, "function"
    .weak  EINT6_IRQHandler, "function"
    .weak  UART4_IRQHandler, "function"
    .weak  USCI0_IRQHandler, "function"
    .weak  USCI1_IRQHandler, "function"
    .weak  BPWM0_IRQHandler, "function"
    .weak  BPWM1_IRQHandler, "function"
    .weak  I2C2_IRQHandler, "function"
    .weak  EQEI0_IRQHandler, "function"
    .weak  ECAP0_IRQHandler, "function"
    .weak  GPH_IRQHandler, "function"
    .weak  EINT7_IRQHandler, "function"
    .weak  WWDT1_IRQHandler, "function"
    .weak  EHCI_IRQHandler, "function"
    .weak  USBOTG20_IRQHandler, "function"
    .weak  NS_RAMPE_IRQHandler, "function"
    .weak  I3C0_IRQHandler, "function"
    .weak  CANFD00_IRQHandler, "function"
    .weak  CANFD01_IRQHandler, "function"
    .weak  CANFD10_IRQHandler, "function"
    .weak  CANFD11_IRQHandler, "function"
    .weak  SPB_IRQHandler, "function"
    .weak  LLSI0_IRQHandler, "function"
    .weak  LLSI1_IRQHandler, "function"
    .weak  LLSI2_IRQHandler, "function"
    .weak  LLSI3_IRQHandler, "function"
    .weak  LLSI4_IRQHandler, "function"
    .weak  LLSI5_IRQHandler, "function"
    .weak  LLSI6_IRQHandler, "function"
    .weak  LLSI7_IRQHandler, "function"
    .weak  LLSI8_IRQHandler, "function"
    .weak  LLSI9_IRQHandler, "function"
    .weak  ELLSI0_IRQHandler, "function"
    .weak  BPWM2_IRQHandler, "function"
    .weak  BPWM3_IRQHandler, "function"
    .weak  BPWM4_IRQHandler, "function"
    .weak  BPWM5_IRQHandler, "function"


Default__IRQHandler:
BOD_IRQHandler:
IRC_IRQHandler:
PWRWU_IRQHandler:
RAMPE_IRQHandler:
CKFAIL_IRQHandler:
ISP_IRQHandler:
RTC_IRQHandler:
WDT0_IRQHandler:
WWDT0_IRQHandler:
EINT0_IRQHandler:
EINT1_IRQHandler:
EINT2_IRQHandler:
EINT3_IRQHandler:
EINT4_IRQHandler:
EINT5_IRQHandler:
GPA_IRQHandler:
GPB_IRQHandler:
GPC_IRQHandler:
GPD_IRQHandler:
GPE_IRQHandler:
GPF_IRQHandler:
QSPI0_IRQHandler:
SPI0_IRQHandler:
BRAKE0_IRQHandler:
EPWM0P0_IRQHandler:
EPWM0P1_IRQHandler:
EPWM0P2_IRQHandler:
BRAKE1_IRQHandler:
EPWM1P0_IRQHandler:
EPWM1P1_IRQHandler:
EPWM1P2_IRQHandler:
TMR0_IRQHandler:
TMR1_IRQHandler:
TMR2_IRQHandler:
TMR3_IRQHandler:
UART0_IRQHandler:
UART1_IRQHandler:
I2C0_IRQHandler:
I2C1_IRQHandler:
PDMA0_IRQHandler:
EADC00_IRQHandler:
EADC01_IRQHandler:
ACMP01_IRQHandler:
EADC02_IRQHandler:
EADC03_IRQHandler:
UART2_IRQHandler:
UART3_IRQHandler:
SPI1_IRQHandler:
SPI2_IRQHandler:
OHCI_IRQHandler:
ETI_IRQHandler:
CRC_IRQHandler:
NS_ISP_IRQHandler:
//SCU_IRQHandler:
SDH0_IRQHandler:
USBD20_IRQHandler:
WDT1_IRQHandler:
I2S0_IRQHandler:
GPG_IRQHandler:
EINT6_IRQHandler:
UART4_IRQHandler:
USCI0_IRQHandler:
USCI1_IRQHandler:
BPWM0_IRQHandler:
BPWM1_IRQHandler:
I2C2_IRQHandler:
EQEI0_IRQHandler:
ECAP0_IRQHandler:
GPH_IRQHandler:
EINT7_IRQHandler:
WWDT1_IRQHandler:
EHCI_IRQHandler:
USBOTG20_IRQHandler:
NS_RAMPE_IRQHandler:
I3C0_IRQHandler:
CANFD00_IRQHandler:
CANFD01_IRQHandler:
CANFD10_IRQHandler:
CANFD11_IRQHandler:
SPB_IRQHandler:
LLSI0_IRQHandler:
LLSI1_IRQHandler:
LLSI2_IRQHandler:
LLSI3_IRQHandler:
LLSI4_IRQHandler:
LLSI5_IRQHandler:
LLSI6_IRQHandler:
LLSI7_IRQHandler:
LLSI8_IRQHandler:
LLSI9_IRQHandler:
ELLSI0_IRQHandler:
BPWM2_IRQHandler:
BPWM3_IRQHandler:
BPWM4_IRQHandler:
BPWM5_IRQHandler:
Default_Handler:
        B       .

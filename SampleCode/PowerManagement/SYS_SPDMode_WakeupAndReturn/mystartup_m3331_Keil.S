/**************************************************************************//**
 * @file     mystartup_m3331_Keil.S
 * @version  V3.00
 * @brief    M3331 Series Startup Source File
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
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
    .word     BOD_IRQHandler            // ; 0: Brown Out detection
    .word     IRC_IRQHandler            // ; 1: Internal RC
    .word     PWRWU_IRQHandler          // ; 2: Power down wake up
    .word     RAMPE_IRQHandler          // ; 3: RAM parity error
    .word     CKFAIL_IRQHandler         // ; 4: Clock detection fail
    .word     ISP_IRQHandler            // ; 5: ISP
    .word     RTC_IRQHandler            // ; 6: Real Time Clock
    .word     Default_Handler           // ; 7:
    .word     WDT0_IRQHandler           // ; 8: Watchdog timer 0
    .word     WWDT0_IRQHandler          // ; 9: Window watchdog timer 0
    .word     EINT0_IRQHandler          // ; 10: External Input 0
    .word     EINT1_IRQHandler          // ; 11: External Input 1
    .word     EINT2_IRQHandler          // ; 12: External Input 2
    .word     EINT3_IRQHandler          // ; 13: External Input 3
    .word     EINT4_IRQHandler          // ; 14: External Input 4
    .word     EINT5_IRQHandler          // ; 15: External Input 5
    .word     GPA_IRQHandler            // ; 16: GPIO Port A
    .word     GPB_IRQHandler            // ; 17: GPIO Port B
    .word     GPC_IRQHandler            // ; 18: GPIO Port C
    .word     GPD_IRQHandler            // ; 19: GPIO Port D
    .word     GPE_IRQHandler            // ; 20: GPIO Port E
    .word     GPF_IRQHandler            // ; 21: GPIO Port F
    .word     QSPI0_IRQHandler          // ; 22: QSPI0
    .word     SPI0_IRQHandler           // ; 23: SPI0
    .word     BRAKE0_IRQHandler         // ; 24: EPWM0 brake
    .word     EPWM0P0_IRQHandler        // ; 25: EPWM0 pair 0
    .word     EPWM0P1_IRQHandler        // ; 26: EPWM0 pair 1
    .word     EPWM0P2_IRQHandler        // ; 27: EPWM0 pair 2
    .word     BRAKE1_IRQHandler         // ; 28: EPWM1 brake
    .word     EPWM1P0_IRQHandler        // ; 29: EPWM1 pair 0
    .word     EPWM1P1_IRQHandler        // ; 30: EPWM1 pair 1
    .word     EPWM1P2_IRQHandler        // ; 31: EPWM1 pair 2
    .word     TMR0_IRQHandler           // ; 32: Timer 0
    .word     TMR1_IRQHandler           // ; 33: Timer 1
    .word     TMR2_IRQHandler           // ; 34: Timer 2
    .word     TMR3_IRQHandler           // ; 35: Timer 3
    .word     UART0_IRQHandler          // ; 36: UART0
    .word     UART1_IRQHandler          // ; 37: UART1
    .word     I2C0_IRQHandler           // ; 38: I2C0
    .word     I2C1_IRQHandler           // ; 39: I2C1
    .word     PDMA0_IRQHandler          // ; 40: Peripheral DMA 0
    .word     Default_Handler           // ; 41:
    .word     EADC00_IRQHandler         // ; 42: EADC0 interrupt source 0
    .word     EADC01_IRQHandler         // ; 43: EADC0 interrupt source 1
    .word     ACMP01_IRQHandler         // ; 44: ACMP0 and ACMP1
    .word     Default_Handler           // ; 45:
    .word     EADC02_IRQHandler         // ; 46: EADC0 interrupt source 2
    .word     EADC03_IRQHandler         // ; 47: EADC0 interrupt source 3
    .word     UART2_IRQHandler          // ; 48: UART2
    .word     UART3_IRQHandler          // ; 49: UART3
    .word     Default_Handler           // ; 50:
    .word     SPI1_IRQHandler           // ; 51: SPI1
    .word     SPI2_IRQHandler           // ; 52: SPI2
    .word     Default_Handler           // ; 53:
    .word     OHCI_IRQHandler           // ; 54: OHCI
    .word     Default_Handler           // ; 55:
    .word     ETI_IRQHandler            // ; 56: ETI
    .word     CRC_IRQHandler            // ; 57: CRC
    .word     Default_Handler           // ; 58:
    .word     NS_ISP_IRQHandler         // ; 59: Non-secure ISP
    .word     SCU_IRQHandler            // ; 60: SCU
    .word     Default_Handler           // ; 61:
    .word     Default_Handler           // ; 62:
    .word     Default_Handler           // ; 63:
    .word     SDH0_IRQHandler           // ; 64: SDH0
    .word     USBD20_IRQHandler         // ; 65: USBD20
    .word     Default_Handler           // ; 66:
    .word     WDT1_IRQHandler           // ; 67: Watchdog timer 1
    .word     I2S0_IRQHandler           // ; 68: I2S0
    .word     Default_Handler           // ; 69:
    .word     Default_Handler           // ; 70:
    .word     Default_Handler           // ; 71:
    .word     GPG_IRQHandler            // ; 72: GPIO Port G
    .word     EINT6_IRQHandler          // ; 73: External Input 6
    .word     UART4_IRQHandler          // ; 74: UART4
    .word     Default_Handler           // ; 75:
    .word     USCI0_IRQHandler          // ; 76: USCI0
    .word     USCI1_IRQHandler          // ; 77: USCI1
    .word     BPWM0_IRQHandler          // ; 78: BPWM0
    .word     BPWM1_IRQHandler          // ; 79: BPWM1
    .word     Default_Handler           // ; 80:
    .word     Default_Handler           // ; 81:
    .word     I2C2_IRQHandler           // ; 82: I2C2
    .word     Default_Handler           // ; 83:
    .word     EQEI0_IRQHandler          // ; 84: EQEI0
    .word     Default_Handler           // ; 85:
    .word     ECAP0_IRQHandler          // ; 86: ECAP0
    .word     Default_Handler           // ; 87:
    .word     GPH_IRQHandler            // ; 88: GPIO Port H
    .word     EINT7_IRQHandler          // ; 89: External Input 7
    .word     Default_Handler           // ; 90:
    .word     WWDT1_IRQHandler          // ; 91: Window watchdog timer 1
    .word     EHCI_IRQHandler           // ; 92: EHCI
    .word     USBOTG20_IRQHandler       // ; 93: HSOTG
    .word     Default_Handler           // ; 94:
    .word     Default_Handler           // ; 95:
    .word     NS_RAMPE_IRQHandler       // ; 96: Non-secure RAM parity error
    .word     Default_Handler           // ; 97:
    .word     Default_Handler           // ; 98:
    .word     Default_Handler           // ; 99:
    .word     Default_Handler           // ; 100:
    .word     Default_Handler           // ; 101:
    .word     Default_Handler           // ; 102:
    .word     Default_Handler           // ; 103:
    .word     Default_Handler           // ; 104:
    .word     Default_Handler           // ; 105:
    .word     Default_Handler           // ; 106:
    .word     Default_Handler           // ; 107:
    .word     I3C0_IRQHandler           // ; 108: I3C0
    .word     Default_Handler           // ; 109:
    .word     Default_Handler           // ; 110:
    .word     Default_Handler           // ; 111:
    .word     CANFD00_IRQHandler        // ; 112: CANFD0 interrupt source 0
    .word     CANFD01_IRQHandler        // ; 113: CANFD0 interrupt source 1
    .word     CANFD10_IRQHandler        // ; 114: CANFD1 interrupt source 0
    .word     CANFD11_IRQHandler        // ; 115: CANFD1 interrupt source 1
    .word     Default_Handler           // ; 116:
    .word     Default_Handler           // ; 117:
    .word     Default_Handler           // ; 118:
    .word     Default_Handler           // ; 119:
    .word     Default_Handler           // ; 120:
    .word     Default_Handler           // ; 121:
    .word     Default_Handler           // ; 122:
    .word     Default_Handler           // ; 123:
    .word     Default_Handler           // ; 124:
    .word     Default_Handler           // ; 125:
    .word     Default_Handler           // ; 126:
    .word     Default_Handler           // ; 127:
    .word     LLSI0_IRQHandler          // ; 128: LLSI0
    .word     LLSI1_IRQHandler          // ; 129: LLSI1
    .word     LLSI2_IRQHandler          // ; 130: LLSI2
    .word     LLSI3_IRQHandler          // ; 131: LLSI3
    .word     LLSI4_IRQHandler          // ; 132: LLSI4
    .word     LLSI5_IRQHandler          // ; 133: LLSI5
    .word     LLSI6_IRQHandler          // ; 134: LLSI6
    .word     LLSI7_IRQHandler          // ; 135: LLSI7
    .word     LLSI8_IRQHandler          // ; 136: LLSI8
    .word     LLSI9_IRQHandler          // ; 137: LLSI9
    .word     ELLSI0_IRQHandler         // ; 138: ELLSI0
    .word     BPWM2_IRQHandler          // ; 139: BPWM2
    .word     BPWM3_IRQHandler          // ; 140: BPWM3
    .word     BPWM4_IRQHandler          // ; 141: BPWM3
    .word     BPWM5_IRQHandler          // ; 142: BPWM4


__Vectors_End:
    .equ    __Vectors_Size, __Vectors_End - __Vectors

    .section .text, "ax"

// ; Reset Handler

    .global Reset_Handler
    .global  SystemInit
    .global  __main
    .type   Reset_Handler, "function"
Reset_Handler:

                LDR     r0, =0x40000294 /* Check RTC wake-up from SPD flag */
                LDR     r0, [r0, #0]
                MOVS    r1, #4
                ANDS    r0, r0, r1
                BEQ     NORMAL

SPD:                                    /* Wake-up from SPD */
                SUB     sp, sp, #12
                POP     {PC}            /* Execute __SPD_Wakeup */

NORMAL:                                 /* Normal Power-on process */
                MOV     r0, #0          /* Reserve 3 words stack space to retain data */
                PUSH    {r0}
                PUSH    {r0}
                PUSH    {r0}

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
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

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

    .global __Enter_SPD
    .type   __Enter_SPD, "function"
__Enter_SPD:                                /* Enter to PD */
                LDR     r0, =__SPD_Wakeup   /* Save SP, LR and __SPD_Wakeup */
                ADD     r0, #1
                MOV     r1, lr
                MOV     r2, sp
                MOV     r3, #0
                LDR     r3, [r3]
                MOV     sp, r3
                PUSH    {r0-r2}
                WFI
                POP     {PC}                /* Execute __SPD_Wakeup */
__SPD_Wakeup:                               /* Restore SP and LR */
                POP     {r1,r2}
                MOV     sp, r2
                BX      r1

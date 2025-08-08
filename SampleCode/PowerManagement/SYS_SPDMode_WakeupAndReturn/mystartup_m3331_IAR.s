;/******************************************************************************
; * @file     mystartup_m3331_IAR.s
; * @version  V3.00
; * @brief    CMSIS Cortex-M33 Core Device Startup File for m3331
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  ProcessHardFault
        EXTERN  SystemInit
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     MemManage_Handler
        DCD     BusFault_Handler
        DCD     UsageFault_Handler
__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     BOD_IRQHandler            ; 0: Brown Out detection
        DCD     IRC_IRQHandler            ; 1: Internal RC
        DCD     PWRWU_IRQHandler          ; 2: Power down wake up
        DCD     RAMPE_IRQHandler          ; 3: RAM parity error
        DCD     CKFAIL_IRQHandler         ; 4: Clock detection fail
        DCD     ISP_IRQHandler            ; 5: ISP
        DCD     RTC_IRQHandler            ; 6: Real Time Clock
        DCD     Default_Handler           ; 7:
        DCD     WDT0_IRQHandler           ; 8: Watchdog timer 0
        DCD     WWDT0_IRQHandler          ; 9: Window watchdog timer 0
        DCD     EINT0_IRQHandler          ; 10: External Input 0
        DCD     EINT1_IRQHandler          ; 11: External Input 1
        DCD     EINT2_IRQHandler          ; 12: External Input 2
        DCD     EINT3_IRQHandler          ; 13: External Input 3
        DCD     EINT4_IRQHandler          ; 14: External Input 4
        DCD     EINT5_IRQHandler          ; 15: External Input 5
        DCD     GPA_IRQHandler            ; 16: GPIO Port A
        DCD     GPB_IRQHandler            ; 17: GPIO Port B
        DCD     GPC_IRQHandler            ; 18: GPIO Port C
        DCD     GPD_IRQHandler            ; 19: GPIO Port D
        DCD     GPE_IRQHandler            ; 20: GPIO Port E
        DCD     GPF_IRQHandler            ; 21: GPIO Port F
        DCD     QSPI0_IRQHandler          ; 22: QSPI0
        DCD     SPI0_IRQHandler           ; 23: SPI0
        DCD     BRAKE0_IRQHandler         ; 24: EPWM0 brake
        DCD     EPWM0P0_IRQHandler        ; 25: EPWM0 pair 0
        DCD     EPWM0P1_IRQHandler        ; 26: EPWM0 pair 1
        DCD     EPWM0P2_IRQHandler        ; 27: EPWM0 pair 2
        DCD     BRAKE1_IRQHandler         ; 28: EPWM1 brake
        DCD     EPWM1P0_IRQHandler        ; 29: EPWM1 pair 0
        DCD     EPWM1P1_IRQHandler        ; 30: EPWM1 pair 1
        DCD     EPWM1P2_IRQHandler        ; 31: EPWM1 pair 2
        DCD     TMR0_IRQHandler           ; 32: Timer 0
        DCD     TMR1_IRQHandler           ; 33: Timer 1
        DCD     TMR2_IRQHandler           ; 34: Timer 2
        DCD     TMR3_IRQHandler           ; 35: Timer 3
        DCD     UART0_IRQHandler          ; 36: UART0
        DCD     UART1_IRQHandler          ; 37: UART1
        DCD     I2C0_IRQHandler           ; 38: I2C0
        DCD     I2C1_IRQHandler           ; 39: I2C1
        DCD     PDMA0_IRQHandler          ; 40: Peripheral DMA 0
        DCD     Default_Handler           ; 41:
        DCD     EADC00_IRQHandler         ; 42: EADC0 interrupt source 0
        DCD     EADC01_IRQHandler         ; 43: EADC0 interrupt source 1
        DCD     ACMP01_IRQHandler         ; 44: ACMP0 and ACMP1
        DCD     Default_Handler           ; 45:
        DCD     EADC02_IRQHandler         ; 46: EADC0 interrupt source 2
        DCD     EADC03_IRQHandler         ; 47: EADC0 interrupt source 3
        DCD     UART2_IRQHandler          ; 48: UART2
        DCD     UART3_IRQHandler          ; 49: UART3
        DCD     Default_Handler           ; 50:
        DCD     SPI1_IRQHandler           ; 51: SPI1
        DCD     SPI2_IRQHandler           ; 52: SPI2
        DCD     Default_Handler           ; 53:
        DCD     OHCI_IRQHandler           ; 54: OHCI
        DCD     Default_Handler           ; 55:
        DCD     ETI_IRQHandler            ; 56: ETI
        DCD     CRC_IRQHandler            ; 57: CRC
        DCD     Default_Handler           ; 58:
        DCD     NS_ISP_IRQHandler         ; 59: Non-secure ISP
        DCD     SCU_IRQHandler            ; 60: SCU
        DCD     Default_Handler           ; 61:
        DCD     Default_Handler           ; 62:
        DCD     Default_Handler           ; 63:
        DCD     SDH0_IRQHandler           ; 64: SDH0
        DCD     USBD20_IRQHandler         ; 65: USBD20
        DCD     Default_Handler           ; 66:
        DCD     WDT1_IRQHandler           ; 67: Watchdog timer 1
        DCD     I2S0_IRQHandler           ; 68: I2S0
        DCD     Default_Handler           ; 69:
        DCD     Default_Handler           ; 70:
        DCD     Default_Handler           ; 71:
        DCD     GPG_IRQHandler            ; 72: GPIO Port G
        DCD     EINT6_IRQHandler          ; 73: External Input 6
        DCD     UART4_IRQHandler          ; 74: UART4
        DCD     Default_Handler           ; 75:
        DCD     USCI0_IRQHandler          ; 76: USCI0
        DCD     USCI1_IRQHandler          ; 77: USCI1
        DCD     BPWM0_IRQHandler          ; 78: BPWM0
        DCD     BPWM1_IRQHandler          ; 79: BPWM1
        DCD     Default_Handler           ; 80:
        DCD     Default_Handler           ; 81:
        DCD     I2C2_IRQHandler           ; 82: I2C2
        DCD     Default_Handler           ; 83:
        DCD     EQEI0_IRQHandler          ; 84: EQEI0
        DCD     Default_Handler           ; 85:
        DCD     ECAP0_IRQHandler          ; 86: ECAP0
        DCD     Default_Handler           ; 87:
        DCD     GPH_IRQHandler            ; 88: GPIO Port H
        DCD     EINT7_IRQHandler          ; 89: External Input 7
        DCD     Default_Handler           ; 90:
        DCD     WWDT1_IRQHandler          ; 91: Window watchdog timer 1
        DCD     EHCI_IRQHandler           ; 92: EHCI
        DCD     USBOTG20_IRQHandler       ; 93: HSOTG
        DCD     Default_Handler           ; 94:
        DCD     Default_Handler           ; 95:
        DCD     NS_RAMPE_IRQHandler       ; 96: Non-secure RAM parity error
        DCD     Default_Handler           ; 97:
        DCD     Default_Handler           ; 98:
        DCD     Default_Handler           ; 99:
        DCD     Default_Handler           ; 100:
        DCD     Default_Handler           ; 101:
        DCD     Default_Handler           ; 102:
        DCD     Default_Handler           ; 103:
        DCD     Default_Handler           ; 104:
        DCD     Default_Handler           ; 105:
        DCD     Default_Handler           ; 106:
        DCD     Default_Handler           ; 107:
        DCD     I3C0_IRQHandler           ; 108: I3C0
        DCD     Default_Handler           ; 109:
        DCD     Default_Handler           ; 110:
        DCD     Default_Handler           ; 111:
        DCD     CANFD00_IRQHandler        ; 112: CANFD0 interrupt source 0
        DCD     CANFD01_IRQHandler        ; 113: CANFD0 interrupt source 1
        DCD     CANFD10_IRQHandler        ; 114: CANFD1 interrupt source 0
        DCD     CANFD11_IRQHandler        ; 115: CANFD1 interrupt source 1
        DCD     Default_Handler           ; 116:
        DCD     Default_Handler           ; 117:
        DCD     Default_Handler           ; 118:
        DCD     Default_Handler           ; 119:
        DCD     Default_Handler           ; 120:
        DCD     Default_Handler           ; 121:
        DCD     Default_Handler           ; 122:
        DCD     Default_Handler           ; 123:
        DCD     Default_Handler           ; 124:
        DCD     Default_Handler           ; 125:
        DCD     Default_Handler           ; 126:
        DCD     Default_Handler           ; 127:
        DCD     LLSI0_IRQHandler          ; 128: LLSI0
        DCD     LLSI1_IRQHandler          ; 129: LLSI1
        DCD     LLSI2_IRQHandler          ; 130: LLSI2
        DCD     LLSI3_IRQHandler          ; 131: LLSI3
        DCD     LLSI4_IRQHandler          ; 132: LLSI4
        DCD     LLSI5_IRQHandler          ; 133: LLSI5
        DCD     LLSI6_IRQHandler          ; 134: LLSI6
        DCD     LLSI7_IRQHandler          ; 135: LLSI7
        DCD     LLSI8_IRQHandler          ; 136: LLSI8
        DCD     LLSI9_IRQHandler          ; 137: LLSI9
        DCD     ELLSI0_IRQHandler         ; 138: ELLSI0
        DCD     BPWM2_IRQHandler          ; 139: BPWM2
        DCD     BPWM3_IRQHandler          ; 140: BPWM3
        DCD     BPWM4_IRQHandler          ; 141: BPWM4
        DCD     BPWM5_IRQHandler          ; 142: BPWM5
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
Reset_Handler

        LDR     r0, =0x40000294 ; Check RTC wake-up from SPD flag
        LDR     r0, [r0, #0]
        MOVS    r1, #4
        ANDS    r0, r0, r1
        BEQ     NORMAL
SPD                             ; Wake-up from SPD
        SUB     sp, sp, #12
        POP     {PC}            ; Execute __SPD_Wakeup
NORMAL                          ; Normal Power-on process
        MOV     r0, #0          ; Reserve 3 words stack space to retain data
        PUSH    {r0}
        PUSH    {r0}
        PUSH    {r0}

        LDR      R0, =SystemInit
        BLX      R0
        LDR      R0, =__iar_program_start
        BX       R0

        PUBWEAK HardFault_Handler
HardFault_Handler\

        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

          PUBWEAK ProcessHardFaultx
ProcessHardFaultx\
        B       .

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK MemManage_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
MemManage_Handler
        B MemManage_Handler

        PUBWEAK BusFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
BusFault_Handler
        B BusFault_Handler

        PUBWEAK UsageFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
UsageFault_Handler
        B UsageFault_Handler

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SVC_Handler
        B SVC_Handler

        PUBWEAK DebugMon_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
DebugMon_Handler
        B DebugMon_Handler

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK  BOD_IRQHandler
        PUBWEAK  IRC_IRQHandler
        PUBWEAK  PWRWU_IRQHandler
        PUBWEAK  RAMPE_IRQHandler
        PUBWEAK  CKFAIL_IRQHandler
        PUBWEAK  ISP_IRQHandler
        PUBWEAK  RTC_IRQHandler
        PUBWEAK  WDT0_IRQHandler
        PUBWEAK  WWDT0_IRQHandler
        PUBWEAK  EINT0_IRQHandler
        PUBWEAK  EINT1_IRQHandler
        PUBWEAK  EINT2_IRQHandler
        PUBWEAK  EINT3_IRQHandler
        PUBWEAK  EINT4_IRQHandler
        PUBWEAK  EINT5_IRQHandler
        PUBWEAK  GPA_IRQHandler
        PUBWEAK  GPB_IRQHandler
        PUBWEAK  GPC_IRQHandler
        PUBWEAK  GPD_IRQHandler
        PUBWEAK  GPE_IRQHandler
        PUBWEAK  GPF_IRQHandler
        PUBWEAK  QSPI0_IRQHandler
        PUBWEAK  SPI0_IRQHandler
        PUBWEAK  BRAKE0_IRQHandler
        PUBWEAK  EPWM0P0_IRQHandler
        PUBWEAK  EPWM0P1_IRQHandler
        PUBWEAK  EPWM0P2_IRQHandler
        PUBWEAK  BRAKE1_IRQHandler
        PUBWEAK  EPWM1P0_IRQHandler
        PUBWEAK  EPWM1P1_IRQHandler
        PUBWEAK  EPWM1P2_IRQHandler
        PUBWEAK  TMR0_IRQHandler
        PUBWEAK  TMR1_IRQHandler
        PUBWEAK  TMR2_IRQHandler
        PUBWEAK  TMR3_IRQHandler
        PUBWEAK  UART0_IRQHandler
        PUBWEAK  UART1_IRQHandler
        PUBWEAK  I2C0_IRQHandler
        PUBWEAK  I2C1_IRQHandler
        PUBWEAK  PDMA0_IRQHandler
        PUBWEAK  EADC00_IRQHandler
        PUBWEAK  EADC01_IRQHandler
        PUBWEAK  ACMP01_IRQHandler
        PUBWEAK  EADC02_IRQHandler
        PUBWEAK  EADC03_IRQHandler
        PUBWEAK  UART2_IRQHandler
        PUBWEAK  UART3_IRQHandler
        PUBWEAK  SPI1_IRQHandler
        PUBWEAK  SPI2_IRQHandler
        PUBWEAK  OHCI_IRQHandler
        PUBWEAK  ETI_IRQHandler
        PUBWEAK  CRC_IRQHandler
        PUBWEAK  NS_ISP_IRQHandler
        PUBWEAK  SCU_IRQHandler
        PUBWEAK  SDH0_IRQHandler
        PUBWEAK  USBD20_IRQHandler
        PUBWEAK  WDT1_IRQHandler
        PUBWEAK  I2S0_IRQHandler
        PUBWEAK  GPG_IRQHandler
        PUBWEAK  EINT6_IRQHandler
        PUBWEAK  UART4_IRQHandler
        PUBWEAK  USCI0_IRQHandler
        PUBWEAK  USCI1_IRQHandler
        PUBWEAK  BPWM0_IRQHandler
        PUBWEAK  BPWM1_IRQHandler
        PUBWEAK  I2C2_IRQHandler
        PUBWEAK  EQEI0_IRQHandler
        PUBWEAK  ECAP0_IRQHandler
        PUBWEAK  GPH_IRQHandler
        PUBWEAK  EINT7_IRQHandler
        PUBWEAK  WWDT1_IRQHandler
        PUBWEAK  EHCI_IRQHandler
        PUBWEAK  USBOTG20_IRQHandler
        PUBWEAK  NS_RAMPE_IRQHandler
        PUBWEAK  I3C0_IRQHandler
        PUBWEAK  CANFD00_IRQHandler
        PUBWEAK  CANFD01_IRQHandler
        PUBWEAK  CANFD10_IRQHandler
        PUBWEAK  CANFD11_IRQHandler
        PUBWEAK  LLSI0_IRQHandler
        PUBWEAK  LLSI1_IRQHandler
        PUBWEAK  LLSI2_IRQHandler
        PUBWEAK  LLSI3_IRQHandler
        PUBWEAK  LLSI4_IRQHandler
        PUBWEAK  LLSI5_IRQHandler
        PUBWEAK  LLSI6_IRQHandler
        PUBWEAK  LLSI7_IRQHandler
        PUBWEAK  LLSI8_IRQHandler
        PUBWEAK  LLSI9_IRQHandler
        PUBWEAK  ELLSI0_IRQHandler
        PUBWEAK  BPWM2_IRQHandler
        PUBWEAK  BPWM3_IRQHandler
        PUBWEAK  BPWM4_IRQHandler
        PUBWEAK  BPWM5_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(2)

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CKFAIL_IRQHandler
ISP_IRQHandler
RTC_IRQHandler
WDT0_IRQHandler
WWDT0_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT4_IRQHandler
EINT5_IRQHandler
GPA_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPD_IRQHandler
GPE_IRQHandler
GPF_IRQHandler
QSPI0_IRQHandler
SPI0_IRQHandler
BRAKE0_IRQHandler
EPWM0P0_IRQHandler
EPWM0P1_IRQHandler
EPWM0P2_IRQHandler
BRAKE1_IRQHandler
EPWM1P0_IRQHandler
EPWM1P1_IRQHandler
EPWM1P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA0_IRQHandler
EADC00_IRQHandler
EADC01_IRQHandler
ACMP01_IRQHandler
EADC02_IRQHandler
EADC03_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
OHCI_IRQHandler
ETI_IRQHandler
CRC_IRQHandler
NS_ISP_IRQHandler
SCU_IRQHandler
SDH0_IRQHandler
USBD20_IRQHandler
WDT1_IRQHandler
I2S0_IRQHandler
GPG_IRQHandler
EINT6_IRQHandler
UART4_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
I2C2_IRQHandler
EQEI0_IRQHandler
ECAP0_IRQHandler
GPH_IRQHandler
EINT7_IRQHandler
WWDT1_IRQHandler
EHCI_IRQHandler
USBOTG20_IRQHandler
NS_RAMPE_IRQHandler
I3C0_IRQHandler
CANFD00_IRQHandler
CANFD01_IRQHandler
CANFD10_IRQHandler
CANFD11_IRQHandler
LLSI0_IRQHandler
LLSI1_IRQHandler
LLSI2_IRQHandler
LLSI3_IRQHandler
LLSI4_IRQHandler
LLSI5_IRQHandler
LLSI6_IRQHandler
LLSI7_IRQHandler
LLSI8_IRQHandler
LLSI9_IRQHandler
ELLSI0_IRQHandler
BPWM2_IRQHandler
BPWM3_IRQHandler
BPWM4_IRQHandler
BPWM5_IRQHandler
Default_Handler
        B Default_Handler

  PUBLIC      __Enter_SPD
__Enter_SPD                                     ; Enter to PD
                LDR     r0, =__SPD_Wakeup       ; Save SP, LR and __SPD_Wakeup
                MOV     r1, lr
                MOV     r2, sp
                MOV     r3, #0
                LDR     r3, [r3]
                MOV     sp, r3
                PUSH    {r0-r2}
                WFI
                POP     {PC}                    ; Execute __SPD_Wakeup
__SPD_Wakeup                                    ; Restore SP and LR
                POP     {r1,r2}
                MOV     sp, r2
                BX      r1



        END

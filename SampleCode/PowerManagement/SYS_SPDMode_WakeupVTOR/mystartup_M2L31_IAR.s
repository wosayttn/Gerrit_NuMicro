;/******************************************************************************
; * @file     mystartup_M2L31_IAR.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2L31 sample code SYS_SPDMode_WakeupVTOR.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

INIVTOR         EQU     0x40000310
SPD_Mem         EQU     0x20000000
  
        EXTERN  __iar_program_start
        ;EXTERN  HardFault_Handler
        EXTERN  ProcessHardFault
        EXTERN  SystemInit
        PUBLIC  __vector_table
       ; PUBLIC  __vector_table_0x1c
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
;__vector_table_0x1c
        DCD     0
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     BOD_IRQHandler              ; 0: Brown Out detection
        DCD     IRC_IRQHandler              ; 1: Internal RC
        DCD     PWRWU_IRQHandler            ; 2: Power down wake up
        DCD     RAMPE_IRQHandler            ; 3: RAM parity error
        DCD     CLKFAIL_IRQHandler          ; 4: Clock detection fail
        DCD     RRMC_IRQHandler             ; 5: RRMC (ISP)
        DCD     RTC_IRQHandler              ; 6: Real Time Clock
        DCD     TAMPER_IRQHandler           ; 7: Tamper interrupt
        DCD     WDT_IRQHandler              ; 8: Watchdog timer
        DCD     WWDT_IRQHandler             ; 9: Window watchdog timer
        DCD     EINT0_IRQHandler            ; 10: External Input 0
        DCD     EINT1_IRQHandler            ; 11: External Input 1
        DCD     EINT2_IRQHandler            ; 12: External Input 2
        DCD     EINT3_IRQHandler            ; 13: External Input 3
        DCD     EINT4_IRQHandler            ; 14: External Input 4
        DCD     EINT5_IRQHandler            ; 15: External Input 5
        DCD     GPA_IRQHandler              ; 16: GPIO Port A
        DCD     GPB_IRQHandler              ; 17: GPIO Port B
        DCD     GPC_IRQHandler              ; 18: GPIO Port C
        DCD     GPD_IRQHandler              ; 19: GPIO Port D
        DCD     GPE_IRQHandler              ; 20: GPIO Port E
        DCD     GPF_IRQHandler              ; 21: GPIO Port F
        DCD     QSPI0_IRQHandler            ; 22: QSPI0
        DCD     SPI0_IRQHandler             ; 23: SPI0
        DCD     EBRAKE0_IRQHandler          ; 24:
        DCD     EPWM0P0_IRQHandler          ; 25:
        DCD     EPWM0P1_IRQHandler          ; 26:
        DCD     EPWM0P2_IRQHandler          ; 27:
        DCD     EBRAKE1_IRQHandler          ; 28:
        DCD     EPWM1P0_IRQHandler          ; 29:
        DCD     EPWM1P1_IRQHandler          ; 30:
        DCD     EPWM1P2_IRQHandler          ; 31:
        DCD     TMR0_IRQHandler             ; 32: Timer 0
        DCD     TMR1_IRQHandler             ; 33: Timer 1
        DCD     TMR2_IRQHandler             ; 34: Timer 2
        DCD     TMR3_IRQHandler             ; 35: Timer 3
        DCD     UART0_IRQHandler            ; 36: UART0
        DCD     UART1_IRQHandler            ; 37: UART1
        DCD     I2C0_IRQHandler             ; 38: I2C0
        DCD     I2C1_IRQHandler             ; 39: I2C1
        DCD     PDMA0_IRQHandler            ; 40: Peripheral DMA 0
        DCD     DAC_IRQHandler              ; 41: DAC
        DCD     EADC0_INT0_IRQHandler       ; 42: EADC0 interrupt source 0
        DCD     EADC0_INT1_IRQHandler       ; 43: EADC0 interrupt source 1
        DCD     ACMP01_IRQHandler           ; 44: ACMP0 and ACMP1
        DCD     ACMP2_IRQHandler            ; 45: ACMP2
        DCD     EADC0_INT2_IRQHandler       ; 46: EADC0 interrupt source 2
        DCD     EADC0_INT3_IRQHandler       ; 47: EADC0 interrupt source 3
        DCD     UART2_IRQHandler            ; 48: UART2
        DCD     UART3_IRQHandler            ; 49: UART3
        DCD     Default_Handler             ; 50:
        DCD     SPI1_IRQHandler             ; 51: SPI1
        DCD     SPI2_IRQHandler             ; 52: SPI2
        DCD     USBD_IRQHandler             ; 53: USB device
        DCD     USBH_IRQHandler             ; 54: USB host
        DCD     USBOTG_IRQHandler           ; 55: USB OTG
        DCD     ETI_IRQHandler              ; 56: ETI
        DCD     CRC0_IRQHandler             ; 57: CRC0
        DCD     Default_Handler             ; 58:
        DCD     Default_Handler             ; 59:
        DCD     Default_Handler             ; 60:
        DCD     Default_Handler             ; 61:
        DCD     SPI3_IRQHandler             ; 62: SPI3
        DCD     TK_IRQHandler               ; 63: Touch Key
        DCD     Default_Handler             ; 64:
        DCD     Default_Handler             ; 65:
        DCD     Default_Handler             ; 66:
        DCD     Default_Handler             ; 67:
        DCD     Default_Handler             ; 68:
        DCD     Default_Handler             ; 69:
        DCD     OPA012_IRQHandler           ; 70: OPA012
        DCD     CRPT_IRQHandler             ; 71: CRPT
        DCD     GPG_IRQHandler              ; 72: GPIO Port G
        DCD     EINT6_IRQHandler            ; 73: External Input 6
        DCD     UART4_IRQHandler            ; 74: UART4
        DCD     UART5_IRQHandler            ; 75: UART5
        DCD     USCI0_IRQHandler            ; 76: USCI0
        DCD     USCI1_IRQHandler            ; 77: USCI1
        DCD     Default_Handler             ; 78:
        DCD     Default_Handler             ; 79:
        DCD     Default_Handler             ; 80:
        DCD     Default_Handler             ; 81:
        DCD     I2C2_IRQHandler             ; 82: I2C2
        DCD     I2C3_IRQHandler             ; 83: I2C3
        DCD     EQEI0_IRQHandler            ; 84: EQEI0
        DCD     EQEI1_IRQHandler            ; 85: EQEI1
        DCD     ECAP0_IRQHandler            ; 86: ECAP0
        DCD     ECAP1_IRQHandler            ; 87: ECAP1
        DCD     GPH_IRQHandler              ; 88: GPIO Port H
        DCD     EINT7_IRQHandler            ; 89: External Input 7
        DCD     Default_Handler             ; 90:
        DCD     Default_Handler             ; 91:
        DCD     Default_Handler             ; 92:
        DCD     Default_Handler             ; 93:
        DCD     Default_Handler             ; 94:
        DCD     Default_Handler             ; 95:
        DCD     Default_Handler             ; 96:
        DCD     Default_Handler             ; 97:
        DCD     LPPDMA0_IRQHandler          ; 98: LPPDMA0
        DCD     Default_Handler             ; 99:
        DCD     Default_Handler             ; 100:
        DCD     TRNG_IRQHandler             ; 101: TRNG
        DCD     UART6_IRQHandler            ; 102: UART6
        DCD     UART7_IRQHandler            ; 103: UART7
        DCD     Default_Handler             ; 104:
        DCD     Default_Handler             ; 105:
        DCD     Default_Handler             ; 106:
        DCD     Default_Handler             ; 107:
        DCD     UTCPD_IRQHandler            ; 108: UTCPD
        DCD     Default_Handler             ; 109:
        DCD     Default_Handler             ; 110:
        DCD     Default_Handler             ; 111:
        DCD     CANFD00_IRQHandler          ; 112: CAN FD 00
        DCD     CANFD01_IRQHandler          ; 113: CAN FD 01
        DCD     CANFD10_IRQHandler          ; 114: CAN FD 10
        DCD     CANFD11_IRQHandler          ; 115: CAN FD 11
        DCD     Default_Handler             ; 116:
        DCD     Default_Handler             ; 117:
        DCD     Default_Handler             ; 118:
        DCD     Default_Handler             ; 119:
        DCD     Default_Handler             ; 120:
        DCD     Default_Handler             ; 121:
        DCD     Default_Handler             ; 122:
        DCD     Default_Handler             ; 123:
        DCD     Default_Handler             ; 124:
        DCD     Default_Handler             ; 125:
        DCD     Default_Handler             ; 126:
        DCD     Default_Handler             ; 127:
        DCD     BRAKE0_IRQHandler           ; 128: BRAKE0
        DCD     PWM0P0_IRQHandler           ; 129: PWM0P0
        DCD     PWM0P1_IRQHandler           ; 130: PWM0P1
        DCD     PWM0P2_IRQHandler           ; 131: PWM0P2
        DCD     BRAKE1_IRQHandler           ; 132: BRAKE1
        DCD     PWM1P0_IRQHandler           ; 133: PWM1P0
        DCD     PWM1P1_IRQHandler           ; 134: PWM1P1
        DCD     PWM1P2_IRQHandler           ; 135: PWM1P2
        DCD     LPADC0_IRQHandler           ; 136: LPADC0
        DCD     LPUART0_IRQHandler          ; 137: LPUART0
        DCD     LPI2C0_IRQHandler           ; 138: LPI2C0
        DCD     LPSPI0_IRQHandler           ; 139: LPSPI0
        DCD     LPTMR0_IRQHandler           ; 140: LPTMR0
        DCD     LPTMR1_IRQHandler           ; 141: LPTMR1
        DCD     TTMR0_IRQHandler            ; 142: TTMR0
        DCD     TTMR1_IRQHandler            ; 143: TTMR1

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
        ; Unlock Register
        LDR     R0, =0x40000100
        LDR     R1, =0x59
        STR     R1, [R0]
        LDR     R1, =0x16
        STR     R1, [R0]
        LDR     R1, =0x88
        STR     R1, [R0]

        LDR     R0, =SystemInit
        BLX     R0

        ; Init POR
        LDR     R2, =0x40000024
        LDR     R1, =0x00005AA5
        STR     R1, [R2]

	LDR     R2, =0x400001EC
        STR     R1, [R2]

        ; Lock register
        LDR     R0, =0x40000100
        MOVS    R1, #0
        STR     R1, [R0]

        LDR     R0, =__iar_program_start
        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
NMI_Handler
        B NMI_Handler

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER:NOROOT(2)
HardFault_Handler
        MOV     R0, LR
        MRS     R1, MSP
        MRS     R2, PSP
        LDR     R3, =ProcessHardFault
        BLX     R3
        BX      R0

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

        PUBWEAK      BOD_IRQHandler
        PUBWEAK      IRC_IRQHandler
        PUBWEAK      PWRWU_IRQHandler
        PUBWEAK      RAMPE_IRQHandler
        PUBWEAK      CLKFAIL_IRQHandler
        PUBWEAK      RRMC_IRQHandler
        PUBWEAK      RTC_IRQHandler
        PUBWEAK      TAMPER_IRQHandler
        PUBWEAK      WDT_IRQHandler
        PUBWEAK      WWDT_IRQHandler
        PUBWEAK      EINT0_IRQHandler
        PUBWEAK      EINT1_IRQHandler
        PUBWEAK      EINT2_IRQHandler
        PUBWEAK      EINT3_IRQHandler
        PUBWEAK      EINT4_IRQHandler
        PUBWEAK      EINT5_IRQHandler
        PUBWEAK      GPA_IRQHandler
        PUBWEAK      GPB_IRQHandler
        PUBWEAK      GPC_IRQHandler
        PUBWEAK      GPD_IRQHandler
        PUBWEAK      GPE_IRQHandler
        PUBWEAK      GPF_IRQHandler
        PUBWEAK      QSPI0_IRQHandler
        PUBWEAK      SPI0_IRQHandler
        PUBWEAK      EBRAKE0_IRQHandler
        PUBWEAK      EPWM0P0_IRQHandler
        PUBWEAK      EPWM0P1_IRQHandler
        PUBWEAK      EPWM0P2_IRQHandler
        PUBWEAK      EBRAKE1_IRQHandler
        PUBWEAK      EPWM1P0_IRQHandler
        PUBWEAK      EPWM1P1_IRQHandler
        PUBWEAK      EPWM1P2_IRQHandler
        PUBWEAK      TMR0_IRQHandler
        PUBWEAK      TMR1_IRQHandler
        PUBWEAK      TMR2_IRQHandler
        PUBWEAK      TMR3_IRQHandler
        PUBWEAK      UART0_IRQHandler
        PUBWEAK      UART1_IRQHandler
        PUBWEAK      I2C0_IRQHandler
        PUBWEAK      I2C1_IRQHandler
        PUBWEAK      PDMA0_IRQHandler
        PUBWEAK      DAC_IRQHandler
        PUBWEAK      EADC0_INT0_IRQHandler
        PUBWEAK      EADC0_INT1_IRQHandler
        PUBWEAK      ACMP01_IRQHandler
        PUBWEAK      ACMP2_IRQHandler
        PUBWEAK      EADC0_INT2_IRQHandler
        PUBWEAK      EADC0_INT3_IRQHandler
        PUBWEAK      UART2_IRQHandler
        PUBWEAK      UART3_IRQHandler
        PUBWEAK      SPI1_IRQHandler
        PUBWEAK      SPI2_IRQHandler
        PUBWEAK      USBD_IRQHandler
        PUBWEAK      USBH_IRQHandler
        PUBWEAK      USBOTG_IRQHandler
        PUBWEAK      ETI_IRQHandler
        PUBWEAK      CRC0_IRQHandler
        PUBWEAK      SPI3_IRQHandler
        PUBWEAK      TK_IRQHandler
        PUBWEAK      OPA012_IRQHandler
        PUBWEAK      CRPT_IRQHandler
        PUBWEAK      GPG_IRQHandler
        PUBWEAK      EINT6_IRQHandler
        PUBWEAK      UART4_IRQHandler
        PUBWEAK      UART5_IRQHandler
        PUBWEAK      USCI0_IRQHandler
        PUBWEAK      USCI1_IRQHandler
        PUBWEAK      I2C2_IRQHandler
        PUBWEAK      I2C3_IRQHandler
        PUBWEAK      EQEI0_IRQHandler
        PUBWEAK      EQEI1_IRQHandler
        PUBWEAK      ECAP0_IRQHandler
        PUBWEAK      ECAP1_IRQHandler
        PUBWEAK      GPH_IRQHandler
        PUBWEAK      EINT7_IRQHandler
        PUBWEAK      LPPDMA0_IRQHandler
        PUBWEAK      TRNG_IRQHandler
        PUBWEAK      UART6_IRQHandler
        PUBWEAK      UART7_IRQHandler
        PUBWEAK      UTCPD_IRQHandler
        PUBWEAK      CANFD00_IRQHandler
        PUBWEAK      CANFD01_IRQHandler
        PUBWEAK      CANFD10_IRQHandler
        PUBWEAK      CANFD11_IRQHandler
        PUBWEAK      BRAKE0_IRQHandler
        PUBWEAK      PWM0P0_IRQHandler
        PUBWEAK      PWM0P1_IRQHandler
        PUBWEAK      PWM0P2_IRQHandler
        PUBWEAK      BRAKE1_IRQHandler
        PUBWEAK      PWM1P0_IRQHandler
        PUBWEAK      PWM1P1_IRQHandler
        PUBWEAK      PWM1P2_IRQHandler
        PUBWEAK      LPADC0_IRQHandler
        PUBWEAK      LPUART0_IRQHandler
        PUBWEAK      LPI2C0_IRQHandler
        PUBWEAK      LPSPI0_IRQHandler
        PUBWEAK      LPTMR0_IRQHandler
        PUBWEAK      LPTMR1_IRQHandler
        PUBWEAK      TTMR0_IRQHandler
        PUBWEAK      TTMR1_IRQHandler

        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CLKFAIL_IRQHandler
RRMC_IRQHandler
RTC_IRQHandler
TAMPER_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
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
EBRAKE0_IRQHandler
EPWM0P0_IRQHandler
EPWM0P1_IRQHandler
EPWM0P2_IRQHandler
EBRAKE1_IRQHandler
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
DAC_IRQHandler
EADC0_INT0_IRQHandler
EADC0_INT1_IRQHandler
ACMP01_IRQHandler
ACMP2_IRQHandler
EADC0_INT2_IRQHandler
EADC0_INT3_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
USBD_IRQHandler
USBH_IRQHandler
USBOTG_IRQHandler
ETI_IRQHandler
CRC0_IRQHandler
SPI3_IRQHandler
TK_IRQHandler
OPA012_IRQHandler
CRPT_IRQHandler
GPG_IRQHandler
EINT6_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
I2C2_IRQHandler
I2C3_IRQHandler
EQEI0_IRQHandler
EQEI1_IRQHandler
ECAP0_IRQHandler
ECAP1_IRQHandler
GPH_IRQHandler
EINT7_IRQHandler
LPPDMA0_IRQHandler
TRNG_IRQHandler
UART6_IRQHandler
UART7_IRQHandler
UTCPD_IRQHandler
CANFD00_IRQHandler
CANFD01_IRQHandler
CANFD10_IRQHandler
CANFD11_IRQHandler
BRAKE0_IRQHandler
PWM0P0_IRQHandler
PWM0P1_IRQHandler
PWM0P2_IRQHandler
BRAKE1_IRQHandler
PWM1P0_IRQHandler
PWM1P1_IRQHandler
PWM1P2_IRQHandler
LPADC0_IRQHandler
LPUART0_IRQHandler
LPI2C0_IRQHandler
LPSPI0_IRQHandler
LPTMR0_IRQHandler
LPTMR1_IRQHandler
TTMR0_IRQHandler
TTMR1_IRQHandler

Default_Handler
    B Default_Handler


;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
        PUBWEAK SH_DoCommand
        SECTION .text:CODE:REORDER:ROOT(2)
SH_DoCommand
        IMPORT  SH_Return

        BKPT    0xAB                ; Wait ICE or HardFault
        LDR     R3, =SH_Return
	PUSH    {R3 ,lr}
        BLX     R3                  ; Call SH_Return. The return value is in R0
	POP     {R3 ,PC}            ; Return value = R0

        ; *** Add for SPD Wakeup And Return sample code.
  PUBLIC      __PC
__PC          
        MOV     r0, lr
        BLX     lr

  PUBLIC     __Enter_SPD
__Enter_SPD                                 ; Enter to PD

             PUSH    {r0-r1}

             ; Disable interrupt
             MOVS    r0, #1
             MSR PRIMASK, r0

             ; Set INIVTOR
             LDR     r0, =INIVTOR
             LDR     r1, =0x20000000
             STR     r1, [r0]

             ;Set __SPD_Wakeup
             LDR     r0, =__SPD_Wakeup
             LDR     r1, =SPD_Mem
             STR     r0, [r1, #4]

             POP     {r0-r1}

             ; Backup r0-r7, lr to stack
             PUSH    {r0-r7,lr}

             LDR     R1, =SPD_Mem
             MOV     R0, sp
             STR     R0, [R1]

             WFI

             ; Enable interrupt
             PUSH    {r0}
             MOVS    r0, #0
             MSR     PRIMASK, r0
             POP     {r0}

__SPD_Wakeup                                ; Execute __SPD_Wakeup
             POP     {r0-r7,pc}             ; Restore all registers and return
        ; *** End of SPD Wakeup And Return sample code.

        END

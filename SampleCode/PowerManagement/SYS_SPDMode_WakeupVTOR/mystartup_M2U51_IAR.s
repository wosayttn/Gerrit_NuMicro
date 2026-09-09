;/******************************************************************************
; * @file     mystartup_M2U51_IAR.s
; * @version  V0.10
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2U51 sample code SYS_SPDMode_WakeupVTOR.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

    MODULE  ?cstartup

    ;; Forward declaration of sections.
    SECTION CSTACK:DATA:NOROOT(3)   ;; 8 bytes alignment

    SECTION .intvec:CODE:NOROOT(2)  ;; 4 bytes alignment

INIVTOR         EQU     0x40000310
SPD_Mem         EQU     0x20000000

    EXTERN  __iar_program_start
    ;EXTERN  HardFault_Handler
    EXTERN  ProcessHardFault
    EXTERN  SystemInit
    PUBLIC  __vector_table
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
    DCD     Default_Handler             ; 3
    DCD     CLKFAIL_IRQHandler          ; 4: Clock detection fail
    DCD     FMC_IRQHandler              ; 5: FMC (ISP)
    DCD     RTC_IRQHandler              ; 6: Real Time Clock
    DCD     Default_Handler             ; 7
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
    DCD     ETI_IRQHandler              ; 22: ETI
    DCD     SPI0_IRQHandler             ; 23: SPI0
    DCD     GPG_IRQHandler              ; 24: GPIO Port G
    DCD     EINT6_IRQHandler            ; 25: External Input 6
    DCD     BRAKE0_IRQHandler           ; 26: BRAKE0
    DCD     PWM0P0_IRQHandler           ; 27: PWM0P0
    DCD     PWM0P1_IRQHandler           ; 28: PWM0P1
    DCD     PWM0P2_IRQHandler           ; 29: PWM0P2
    DCD     TMR0_IRQHandler             ; 30: Timer 0
    DCD     TMR1_IRQHandler             ; 31: Timer 1
    DCD     TMR2_IRQHandler             ; 32: Timer 2
    DCD     TMR3_IRQHandler             ; 33: Timer 3
    DCD     UART0_IRQHandler            ; 34: UART0
    DCD     UART1_IRQHandler            ; 35: UART1
    DCD     I2C0_IRQHandler             ; 36: I2C0
    DCD     I2C1_IRQHandler             ; 37: I2C1
    DCD     PDMA0_IRQHandler            ; 38: Peripheral DMA 0
    DCD     Default_Handler             ; 39
    DCD     ADC0_INT0_IRQHandler        ; 40: ADC0 interrupt source 0
    DCD     Default_Handler             ; 41
    DCD     ACMP01_IRQHandler           ; 42: ACMP0 and ACMP1
    DCD     BPWM0_IRQHandler            ; 43: BPWM0
    DCD     GPH_IRQHandler              ; 44: GPIO Port H
    DCD     EINT7_IRQHandler            ; 45: External Input 7
    DCD     UART2_IRQHandler            ; 46: UART2
    DCD     Default_Handler             ; 47
    DCD     USCI0_IRQHandler            ; 48: USCI0
    DCD     SPI1_IRQHandler             ; 49: SPI1
    DCD     SPI2_IRQHandler             ; 50: SPI2
    DCD     Default_Handler             ; 51
    DCD     Default_Handler             ; 52
    DCD     Default_Handler             ; 53
    DCD     Default_Handler             ; 54
    DCD     CRYPTO_IRQHandler           ; 55: CRYPTO
    DCD     Default_Handler             ; 56
    DCD     I2C2_IRQHandler             ; 57: I2C2
    DCD     Default_Handler             ; 58
    DCD     LCD_IRQHandler              ; 59: LCD
    DCD     CRC0_IRQHandler             ; 60: CRC0

__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB
        PUBWEAK Reset_Handler
        SECTION .text:CODE:NOROOT:REORDER(2)    ; 4 bytes alignment
Reset_Handler
        ; Unlock Register
        LDR     R0, =0x40000100
        LDR     R1, =0x59
        STR     R1, [R0]
        LDR     R1, =0x16
        STR     R1, [R0]
        LDR     R1, =0x88
        STR     R1, [R0]

        LDR      R0, =SystemInit
        BLX      R0

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

        LDR      R0, =__iar_program_start
        BX       R0

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

        PUBWEAK     BOD_IRQHandler              ; 0: Brown Out detection
        PUBWEAK     IRC_IRQHandler              ; 1: Internal RC
        PUBWEAK     PWRWU_IRQHandler            ; 2: Power down wake up
        PUBWEAK     CLKFAIL_IRQHandler          ; 4: Clock detection fail
        PUBWEAK     FMC_IRQHandler              ; 5: FMC (ISP)
        PUBWEAK     RTC_IRQHandler              ; 6: Real Time Clock
        PUBWEAK     WDT_IRQHandler              ; 8: Watchdog timer
        PUBWEAK     WWDT_IRQHandler             ; 9: Window watchdog timer
        PUBWEAK     EINT0_IRQHandler            ; 10: External Input 0
        PUBWEAK     EINT1_IRQHandler            ; 11: External Input 1
        PUBWEAK     EINT2_IRQHandler            ; 12: External Input 2
        PUBWEAK     EINT3_IRQHandler            ; 13: External Input 3
        PUBWEAK     EINT4_IRQHandler            ; 14: External Input 4
        PUBWEAK     EINT5_IRQHandler            ; 15: External Input 5
        PUBWEAK     GPA_IRQHandler              ; 16: GPIO Port A
        PUBWEAK     GPB_IRQHandler              ; 17: GPIO Port B
        PUBWEAK     GPC_IRQHandler              ; 18: GPIO Port C
        PUBWEAK     GPD_IRQHandler              ; 19: GPIO Port D
        PUBWEAK     GPE_IRQHandler              ; 20: GPIO Port E
        PUBWEAK     GPF_IRQHandler              ; 21: GPIO Port F
        PUBWEAK     ETI_IRQHandler              ; 22: ETI
        PUBWEAK     SPI0_IRQHandler             ; 23: SPI0
        PUBWEAK     GPG_IRQHandler              ; 24: GPIO Port G
        PUBWEAK     EINT6_IRQHandler            ; 25: External Input 6
        PUBWEAK     BRAKE0_IRQHandler           ; 26: BRAKE0
        PUBWEAK     PWM0P0_IRQHandler           ; 27: PWM0P0
        PUBWEAK     PWM0P1_IRQHandler           ; 28: PWM0P1
        PUBWEAK     PWM0P2_IRQHandler           ; 29: PWM0P2
        PUBWEAK     TMR0_IRQHandler             ; 30: Timer 0
        PUBWEAK     TMR1_IRQHandler             ; 31: Timer 1
        PUBWEAK     TMR2_IRQHandler             ; 32: Timer 2
        PUBWEAK     TMR3_IRQHandler             ; 33: Timer 3
        PUBWEAK     UART0_IRQHandler            ; 34: UART0
        PUBWEAK     UART1_IRQHandler            ; 35: UART1
        PUBWEAK     I2C0_IRQHandler             ; 36: I2C0
        PUBWEAK     I2C1_IRQHandler             ; 37: I2C1
        PUBWEAK     PDMA0_IRQHandler            ; 38: Peripheral DMA 0
        PUBWEAK     ADC0_INT0_IRQHandler        ; 40: ADC0 interrupt source 0
        PUBWEAK     ACMP01_IRQHandler           ; 42: ACMP0 and ACMP1
        PUBWEAK     BPWM0_IRQHandler            ; 43: BPWM0
        PUBWEAK     GPH_IRQHandler              ; 44: GPIO Port H
        PUBWEAK     EINT7_IRQHandler            ; 45: External Input 7
        PUBWEAK     UART2_IRQHandler            ; 46: UART2
        PUBWEAK     USCI0_IRQHandler            ; 48: USCI0
        PUBWEAK     SPI1_IRQHandler             ; 49: SPI1
        PUBWEAK     SPI2_IRQHandler             ; 50: SPI2
        PUBWEAK     CRYPTO_IRQHandler           ; 55: CRYPTO
        PUBWEAK     I2C2_IRQHandler             ; 57: I2C2
        PUBWEAK     LCD_IRQHandler              ; 59: LCD
        PUBWEAK     CRC0_IRQHandler             ; 60: CRC0

        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
CLKFAIL_IRQHandler
FMC_IRQHandler
RTC_IRQHandler
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
ETI_IRQHandler
SPI0_IRQHandler
GPG_IRQHandler
EINT6_IRQHandler
BRAKE0_IRQHandler
PWM0P0_IRQHandler
PWM0P1_IRQHandler
PWM0P2_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA0_IRQHandler
ADC0_INT0_IRQHandler
ACMP01_IRQHandler
BPWM0_IRQHandler
GPH_IRQHandler
EINT7_IRQHandler
UART2_IRQHandler
USCI0_IRQHandler
SPI1_IRQHandler
SPI2_IRQHandler
CRYPTO_IRQHandler
I2C2_IRQHandler
LCD_IRQHandler
CRC0_IRQHandler

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

;/******************************************************************************
; * @file     startup_M471.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M4 Core Device Startup File for M471
; *
; * @note
; * SPDX-License-Identifier: Apache-2.0  
; * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
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
        DCD     FMC_IRQHandler            ; 5: FMC
        DCD     RTC_IRQHandler            ; 6: Real Time Clock
        DCD     Default_Handler           ; 7: 
        DCD     WDT_IRQHandler            ; 8: Watchdog timer
        DCD     WWDT_IRQHandler           ; 9: Window watchdog timer
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
        DCD     Default_Handler           ; 22: 
        DCD     SPI0_IRQHandler           ; 23: SPI0
        DCD     BRAKE0_IRQHandler         ; 24:
        DCD     EPWM0P0_IRQHandler        ; 25:
        DCD     EPWM0P1_IRQHandler        ; 26:
        DCD     EPWM0P2_IRQHandler        ; 27:
        DCD     BRAKE1_IRQHandler         ; 28:
        DCD     EPWM1P0_IRQHandler        ; 29:
        DCD     EPWM1P1_IRQHandler        ; 30:
        DCD     EPWM1P2_IRQHandler        ; 31:
        DCD     TMR0_IRQHandler           ; 32: Timer 0
        DCD     TMR1_IRQHandler           ; 33: Timer 1
        DCD     TMR2_IRQHandler           ; 34: Timer 2
        DCD     TMR3_IRQHandler           ; 35: Timer 3
        DCD     UART0_IRQHandler          ; 36: UART0
        DCD     UART1_IRQHandler          ; 37: UART1
        DCD     I2C0_IRQHandler           ; 38: I2C0
        DCD     I2C1_IRQHandler           ; 39: I2C1
        DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
        DCD     DAC_IRQHandler            ; 41: DAC
        DCD     EADC0_INT0_IRQHandler     ; 42: EADC0 interrupt source 0
        DCD     EADC0_INT1_IRQHandler     ; 43: EADC0 interrupt source 1
        DCD     ACMP01_IRQHandler         ; 44: ACMP0 and ACMP1
        DCD     Default_Handler           ; 45: Reserved
        DCD     EADC0_INT2_IRQHandler     ; 46: EADC0 interrupt source 2
        DCD     EADC0_INT3_IRQHandler     ; 47: EADC0 interrupt source 3
        DCD     UART2_IRQHandler          ; 48: UART2
        DCD     UART3_IRQHandler          ; 49: UART3
        DCD     Default_Handler           ; 50: 
        DCD     SPI1_IRQHandler           ; 51: SPI1
        DCD     Default_Handler           ; 52: 
        DCD     Default_Handler           ; 53: 
        DCD     Default_Handler           ; 54: 
        DCD     Default_Handler           ; 55: 
        DCD     Default_Handler           ; 56: 
        DCD     Default_Handler           ; 57: 
        DCD     Default_Handler           ; 58: 
        DCD     Default_Handler           ; 59: 
        DCD     Default_Handler           ; 60: 
        DCD     Default_Handler           ; 61:
        DCD     Default_Handler           ; 62: 
        DCD     Default_Handler           ; 63:
        DCD     Default_Handler           ; 64: 
        DCD     Default_Handler           ; 65: 
        DCD     Default_Handler           ; 66: 
        DCD     Default_Handler           ; 67: 
        DCD     Default_Handler           ; 68: 
        DCD     Default_Handler           ; 69: 
        DCD     Default_Handler           ; 70: 
        DCD     PRNG_IRQHandler           ; 71: PRNG
        DCD     GPG_IRQHandler            ; 72:
        DCD     EINT6_IRQHandler          ; 73:
        DCD     UART4_IRQHandler          ; 74: UART4
        DCD     UART5_IRQHandler          ; 75: UART5
        DCD     Default_Handler           ; 76: 
        DCD     Default_Handler           ; 77: 
        DCD     BPWM0_IRQHandler          ; 78: BPWM0
        DCD     BPWM1_IRQHandler          ; 79: BPWM1
        DCD     Default_Handler           ; 80: 
        DCD     Default_Handler           ; 81: 
        DCD     Default_Handler           ; 82: 
        DCD     Default_Handler           ; 83:
        DCD     Default_Handler           ; 84: 
        DCD     Default_Handler           ; 85: 
        DCD     Default_Handler           ; 86: 
        DCD     Default_Handler           ; 87: 
        DCD     GPH_IRQHandler            ; 88: GPH
        DCD     EINT7_IRQHandler          ; 89: EINT7
        DCD     Default_Handler           ; 90: 
        DCD     Default_Handler           ; 91:
        DCD     Default_Handler           ; 92: 
        DCD     Default_Handler           ; 93: 
        DCD     Default_Handler           ; 94:
        DCD     Default_Handler           ; 95:
        DCD     Default_Handler           ; 96:
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
        DCD     Default_Handler           ; 108: 
        DCD     Default_Handler           ; 109: 
        DCD     GPI_IRQHandler            ; 110: GPI
        DCD     CIR_IRQHandler            ; 111: CIR
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

	#ifndef ENABLE_SPIM_CACHE
        LDR     R0, =0x40000200            ; R0 = Clock Controller Register Base Address
        LDR     R1, [R0,#0x4]              ; R1 = 0x40000204  (AHBCLK)
        ORR     R1, R1, #0x4000              
        STR     R1, [R0,#0x4]              ; CLK->AHBCLK |= CLK_AHBCLK_SPIMCKEN_Msk;
                
        LDR     R0, =0x40007000            ; R0 = SPIM Register Base Address
        LDR     R1, [R0,#4]                ; R1 = SPIM->CTL1
        ORR     R1, R1,#2                  ; R1 |= SPIM_CTL1_CACHEOFF_Msk
        STR     R1, [R0,#4]                ; _SPIM_DISABLE_CACHE()
        LDR     R1, [R0,#4]                ; R1 = SPIM->CTL1
        ORR     R1, R1, #4                 ; R1 |= SPIM_CTL1_CCMEN_Msk
        STR     R1, [R0,#4]                ; _SPIM_ENABLE_CCM()
	#endif

        LDR     R0, =SystemInit
        BLX     R0

        ; Init POR
        ; LDR     R2, =0x40000024
        ; LDR     R1, =0x00005AA5
        ; STR     R1, [R2]

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

        PUBWEAK  BOD_IRQHandler
        PUBWEAK  IRC_IRQHandler
        PUBWEAK  PWRWU_IRQHandler
        PUBWEAK  RAMPE_IRQHandler
        PUBWEAK  CKFAIL_IRQHandler
        PUBWEAK  FMC_IRQHandler
        PUBWEAK  RTC_IRQHandler
        PUBWEAK  WDT_IRQHandler
        PUBWEAK  WWDT_IRQHandler
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
        PUBWEAK  PDMA_IRQHandler
        PUBWEAK  DAC_IRQHandler
        PUBWEAK  EADC0_INT0_IRQHandler
        PUBWEAK  EADC0_INT1_IRQHandler
        PUBWEAK  ACMP01_IRQHandler
        PUBWEAK  EADC0_INT2_IRQHandler
        PUBWEAK  EADC0_INT3_IRQHandler
        PUBWEAK  UART2_IRQHandler
        PUBWEAK  UART3_IRQHandler
        PUBWEAK  SPI1_IRQHandler
        PUBWEAK  PRNG_IRQHandler
        PUBWEAK  GPG_IRQHandler        
        PUBWEAK  EINT6_IRQHandler
        PUBWEAK  UART4_IRQHandler
        PUBWEAK  UART5_IRQHandler
        PUBWEAK  BPWM0_IRQHandler
        PUBWEAK  BPWM1_IRQHandler
        PUBWEAK  GPH_IRQHandler
        PUBWEAK  EINT7_IRQHandler
        PUBWEAK  GPI_IRQHandler
        PUBWEAK  CIR_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
RAMPE_IRQHandler
CKFAIL_IRQHandler
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
PDMA_IRQHandler
DAC_IRQHandler
EADC0_INT0_IRQHandler
EADC0_INT1_IRQHandler
ACMP01_IRQHandler
EADC0_INT2_IRQHandler
EADC0_INT3_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
SPI1_IRQHandler
PRNG_IRQHandler
GPG_IRQHandler
EINT6_IRQHandler
UART4_IRQHandler
UART5_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
GPH_IRQHandler
EINT7_IRQHandler
GPI_IRQHandler
CIR_IRQHandler
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

        END
;/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

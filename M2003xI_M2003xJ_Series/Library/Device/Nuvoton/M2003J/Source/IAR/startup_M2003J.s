;/******************************************************************************
; * @file     startup_M2003J.s
; * @version  V0.10
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2003J
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
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
        DCD     DebugMon_Handler
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     BOD_IRQHandler            ; 0: Brown Out detection
        DCD     IRC_IRQHandler            ; 1: Internal RC Trim
        DCD     PWRWU_IRQHandler          ; 2: Power down wake up
        DCD     SR0ECC_Handler            ; 3: SRAM0 error correcting code
        DCD     CLKFAIL_IRQHandler        ; 4: Clock detection fail
        DCD     FMC_IRQHandler            ; 5: FMC(ISP)
        DCD     RTC_IRQHandler            ; 6: Real Time Clock
        DCD     SR1ECC_Handler            ; 7: SRAM1 error correcting code
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
        DCD     0                         ; 22: Reserved
        DCD     0                         ; 23: Reserved
        DCD     0                         ; 24: Reserved
        DCD     0                         ; 25: Reserved
        DCD     0                         ; 26: Reserved
        DCD     0                         ; 27: Reserved
        DCD     0                         ; 28: Reserved
        DCD     0                         ; 29: Reserved
        DCD     0                         ; 30: Reserved
        DCD     0                         ; 31: Reserved
        DCD     TMR0_IRQHandler           ; 32: Timer 0
        DCD     TMR1_IRQHandler           ; 33: Timer 1
        DCD     TMR2_IRQHandler           ; 34: Timer 2
        DCD     TMR3_IRQHandler           ; 35: Timer 3
        DCD     UART0_IRQHandler          ; 36: UART0
        DCD     UART1_IRQHandler          ; 37: UART1
        DCD     I2C0_IRQHandler           ; 38: I2C0
        DCD     I2C1_IRQHandler           ; 39: I2C1
        DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
        DCD     0                         ; 41: Reserved
        DCD     ADC_IRQHandler            ; 42: ADC interrupt
        DCD     0                         ; 43: Reserved
        DCD     ACMP01_IRQHandler         ; 44: ACMP0 and ACMP1
        DCD     0                         ; 45: Reserved
        DCD     0                         ; 46: Reserved
        DCD     0                         ; 47: Reserved
        DCD     UART2_IRQHandler          ; 48: UART2
        DCD     UART3_IRQHandler          ; 49: UART3
        DCD     0                         ; 50: Reserved
        DCD     0                         ; 51: Reserved
        DCD     USCI0_IRQHandler          ; 52: USCI0
        DCD     USCI1_IRQHandler          ; 53: USCI1
        DCD     0                         ; 54: Reserved
        DCD     0                         ; 55: Reserved
        DCD     0                         ; 56: Reserved
        DCD     CRC_IRQHandler            ; 57: CRC
        DCD     0                         ; 58: Reserved
        DCD     0                         ; 59: Reserved
        DCD     0                         ; 60: Reserved
        DCD     USCI2_IRQHandler          ; 61: USCI2
        DCD     USCI3_IRQHandler          ; 62: USCI3
        DCD     USCI4_IRQHandler          ; 63: USCI4
        DCD     TMR4_IRQHandler           ; 64: Timer 4
        DCD     TMR5_IRQHandler           ; 65: Timer 5
        DCD     TMR6_IRQHandler           ; 66: Timer 6
        DCD     TMR7_IRQHandler           ; 67: Timer 7
        DCD     TMR8_IRQHandler           ; 68: Timer 8
        DCD     0                         ; 69: Reserved
        DCD     0                         ; 70: Reserved
        DCD     0                         ; 71: Reserved
        DCD     GPG_IRQHandler            ; 72: GPIO Port G
        DCD     0                         ; 73: Reserved
        DCD     UART4_IRQHandler          ; 74: UART4
        DCD     0                         ; 75: Reserved
        DCD     0                         ; 76: Reserved
        DCD     BPWM0_IRQHandler          ; 77: BPWM0
        DCD     BPWM1_IRQHandler          ; 78: BPWM1
        DCD     0                         ; 79: Reserved
        DCD     0                         ; 80: Reserved
        DCD     DFMC_IRQHandler           ; 81: DFMC
        DCD     I2C2_IRQHandler           ; 82: I2C2
        DCD     0                         ; 83: Reserved
        DCD     0                         ; 84: Reserved
        DCD     0                         ; 85: Reserved
        DCD     0                         ; 86: Reserved
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

        PUBWEAK  BOD_IRQHandler
        PUBWEAK  IRC_IRQHandler
        PUBWEAK  PWRWU_IRQHandler
        PUBWEAK  SR0ECC_Handler
        PUBWEAK  CLKFAIL_IRQHandler
        PUBWEAK  FMC_IRQHandler
        PUBWEAK  RTC_IRQHandler
        PUBWEAK  SR1ECC_Handler
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
        PUBWEAK  TMR0_IRQHandler
        PUBWEAK  TMR1_IRQHandler
        PUBWEAK  TMR2_IRQHandler
        PUBWEAK  TMR3_IRQHandler
        PUBWEAK  UART0_IRQHandler
        PUBWEAK  UART1_IRQHandler
        PUBWEAK  I2C0_IRQHandler
        PUBWEAK  I2C1_IRQHandler
        PUBWEAK  PDMA_IRQHandler
        PUBWEAK  ADC_IRQHandler
        PUBWEAK  ACMP01_IRQHandler
        PUBWEAK  UART2_IRQHandler
        PUBWEAK  UART3_IRQHandler
        PUBWEAK  USCI0_IRQHandler
        PUBWEAK  USCI1_IRQHandler
        PUBWEAK  CRC_IRQHandler
        PUBWEAK  USCI2_IRQHandler
        PUBWEAK  USCI3_IRQHandler
        PUBWEAK  USCI4_IRQHandler
        PUBWEAK  TMR4_IRQHandler
        PUBWEAK  TMR5_IRQHandler
        PUBWEAK  TMR6_IRQHandler
        PUBWEAK  TMR7_IRQHandler
        PUBWEAK  TMR8_IRQHandler
        PUBWEAK  GPG_IRQHandler
        PUBWEAK  UART4_IRQHandler
        PUBWEAK  BPWM0_IRQHandler
        PUBWEAK  BPWM1_IRQHandler
        PUBWEAK  DFMC_IRQHandler
        PUBWEAK  I2C2_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
SR0ECC_Handler
CLKFAIL_IRQHandler
FMC_IRQHandler
RTC_IRQHandler
SR1ECC_Handler
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
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
PDMA_IRQHandler
ADC_IRQHandler
ACMP01_IRQHandler
UART2_IRQHandler
UART3_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
CRC_IRQHandler
USCI2_IRQHandler
USCI3_IRQHandler
USCI4_IRQHandler
TMR4_IRQHandler
TMR5_IRQHandler
TMR6_IRQHandler
TMR7_IRQHandler
TMR8_IRQHandler
GPG_IRQHandler
UART4_IRQHandler
BPWM0_IRQHandler
BPWM1_IRQHandler
DFMC_IRQHandler
I2C2_IRQHandler
DEFAULT_IRQHandler
        B DEFAULT_IRQHandler 

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
          PUBWEAK SH_DoCommand
          SECTION .text:CODE:REORDER:ROOT(2)
SH_DoCommand   
                IMPORT      SH_Return
                    
                BKPT    0xAB                ; Wait ICE or HardFault
                LDR     R3, =SH_Return			        
		PUSH    {R3 ,lr}
                BLX     R3                  ; Call SH_Return. The return value is in R0
		POP     {R3 ,PC}            ; Return value = R0

        END
;/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

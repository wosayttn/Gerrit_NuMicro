;/******************************************************************************
; * @file     startup_M2003.s
; * @version  V0.10
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2003
; *
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
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     BOD_IRQHandler            ; 0: Brown Out detection
        DCD     DEFAULT_IRQHandler        ; 1:
        DCD     PWRWU_IRQHandler          ; 2: Power down wake up
        DCD     DEFAULT_IRQHandler        ; 3:
        DCD     CLKFAIL_IRQHandler        ; 4: Clock fail detected
        DCD     ISP_IRQHandler            ; 5: FMC(ISP)
        DCD     DEFAULT_IRQHandler        ; 6:
        DCD     DEFAULT_IRQHandler        ; 7:
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
        DCD     QSPI0_IRQHandler          ; 22: QSPI0
        DCD     DEFAULT_IRQHandler        ; 23:
        DCD     DEFAULT_IRQHandler        ; 24:
        DCD     PWM0_IRQHandler           ; 25: PWM0
        DCD     PWM1_IRQHandler           ; 26: PWM1
        DCD     DEFAULT_IRQHandler        ; 27:
        DCD     DEFAULT_IRQHandler        ; 28:
        DCD     DEFAULT_IRQHandler        ; 29:
        DCD     DEFAULT_IRQHandler        ; 30:
        DCD     DEFAULT_IRQHandler        ; 31:
        DCD     TMR0_IRQHandler           ; 32: Timer 0
        DCD     TMR1_IRQHandler           ; 33: Timer 1
        DCD     TMR2_IRQHandler           ; 34: Timer 2
        DCD     TMR3_IRQHandler           ; 35: Timer 3
        DCD     UART0_IRQHandler          ; 36: UART0
        DCD     UART1_IRQHandler          ; 37: UART1
        DCD     I2C0_IRQHandler           ; 38: I2C0
        DCD     DEFAULT_IRQHandler        ; 39:
        DCD     PDMA_IRQHandler           ; 40: PDMA
        DCD     DEFAULT_IRQHandler        ; 41:
        DCD     ADC_IRQHandler            ; 42: ADC interrupt
        DCD     DEFAULT_IRQHandler        ; 43:
        DCD     DEFAULT_IRQHandler        ; 44:
        DCD     DEFAULT_IRQHandler        ; 45:
        DCD     DEFAULT_IRQHandler        ; 46:
        DCD     DEFAULT_IRQHandler        ; 47:
        DCD     UART2_IRQHandler          ; 48: UART2
        DCD     DEFAULT_IRQHandler        ; 49:
        DCD     DEFAULT_IRQHandler        ; 50:
        DCD     DEFAULT_IRQHandler        ; 51:
        DCD     USCI0_IRQHandler          ; 52: USCI0
        DCD     USCI1_IRQHandler          ; 53: USCI1
        DCD     USCI2_IRQHandler          ; 54: USCI2
        DCD     USCI3_IRQHandler          ; 55: USCI3
        DCD     CAN0_IRQHandler           ; 56: CAN0
        DCD     CRC0_IRQHandler           ; 57: CRC0
        DCD     DEFAULT_IRQHandler        ; 58:
        DCD     DEFAULT_IRQHandler        ; 59:
        DCD     ECAP0_IRQHandler          ; 60: ECAP0
        DCD     DEFAULT_IRQHandler        ; 61:
        DCD     DEFAULT_IRQHandler        ; 62:
        DCD     DEFAULT_IRQHandler        ; 63:
        DCD     DEFAULT_IRQHandler        ; 64:
        DCD     DEFAULT_IRQHandler        ; 65:
        DCD     DEFAULT_IRQHandler        ; 66:
        DCD     DEFAULT_IRQHandler        ; 67:
        DCD     DEFAULT_IRQHandler        ; 68:
        DCD     DEFAULT_IRQHandler        ; 69:
        DCD     DEFAULT_IRQHandler        ; 70:
        DCD     DEFAULT_IRQHandler        ; 71:
        DCD     GPG_IRQHandler            ; 72: GPIO Port G




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

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
PendSV_Handler
        B PendSV_Handler

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER:NOROOT(1)
SysTick_Handler
        B SysTick_Handler

        PUBWEAK  BOD_IRQHandler
        PUBWEAK  PWRWU_IRQHandler
        PUBWEAK  CLKFAIL_IRQHandler
        PUBWEAK  ISP_IRQHandler
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
        PUBWEAK  QSPI0_IRQHandler
        PUBWEAK  PWM0_IRQHandler
        PUBWEAK  PWM1_IRQHandler
        PUBWEAK  TMR0_IRQHandler
        PUBWEAK  TMR1_IRQHandler
        PUBWEAK  TMR2_IRQHandler
        PUBWEAK  TMR3_IRQHandler
        PUBWEAK  UART0_IRQHandler
        PUBWEAK  UART1_IRQHandler
        PUBWEAK  I2C0_IRQHandler
        PUBWEAK  PDMA_IRQHandler
        PUBWEAK  ADC_IRQHandler
        PUBWEAK  UART2_IRQHandler
        PUBWEAK  USCI0_IRQHandler
        PUBWEAK  USCI1_IRQHandler
        PUBWEAK  USCI2_IRQHandler
        PUBWEAK  USCI3_IRQHandler
        PUBWEAK  CAN0_IRQHandler
        PUBWEAK  CRC0_IRQHandler
        PUBWEAK  ECAP0_IRQHandler
        PUBWEAK  GPG_IRQHandler
        SECTION .text:CODE:REORDER:NOROOT(1)

BOD_IRQHandler
PWRWU_IRQHandler
CLKFAIL_IRQHandler
ISP_IRQHandler
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
PWM0_IRQHandler
PWM1_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
PDMA_IRQHandler
ADC_IRQHandler
UART2_IRQHandler
USCI0_IRQHandler
USCI1_IRQHandler
USCI2_IRQHandler
USCI3_IRQHandler
CAN0_IRQHandler
CRC0_IRQHandler
ECAP0_IRQHandler
GPG_IRQHandler

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

;/******************************************************************************
; * @file     mystartup_M2U51_Keil.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2U51
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00001000
    ENDIF

				AREA	|._SPD_MEM|, DATA, READWRITE, ALIGN=3
INIVTOR			EQU		0x40000310
SPD_Mem			SPACE	8

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

    IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000100
    ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp                ; Top of Stack
                DCD     Reset_Handler               ; Reset Handler
                DCD     NMI_Handler                 ; NMI Handler
                DCD     HardFault_Handler           ; Hard Fault Handler
                DCD     MemManage_Handler           ; MPU Fault Handler
                DCD     BusFault_Handler            ; Bus Fault Handler
                DCD     UsageFault_Handler          ; Usage Fault Handler
                DCD     0                           ; Reserved
                DCD     0                           ; Reserved
                DCD     0                           ; Reserved
                DCD     0                           ; Reserved
                DCD     SVC_Handler                 ; SVCall Handler
                DCD     0                           ; Debug Monitor Handler
                DCD     0                           ; Reserved
                DCD     PendSV_Handler              ; PendSV Handler
                DCD     SysTick_Handler             ; SysTick Handler

                ; External Interrupts
                DCD     BOD_IRQHandler              ; 0: Brown Out detection
                DCD     IRC_IRQHandler              ; 1: Internal RC
                DCD     PWRWU_IRQHandler            ; 2: Power down wake up
                DCD     Default_Handler             ; 3:
                DCD     CLKFAIL_IRQHandler          ; 4: Clock detection fail
                DCD     FMC_IRQHandler              ; 5: FMC (ISP)
                DCD     RTC_IRQHandler              ; 6: Real Time Clock
                DCD     Default_Handler             ; 7:
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
                DCD     Default_Handler             ; 39:
                DCD     ADC0_INT0_IRQHandler        ; 40: ADC0 interrupt source 0
                DCD     Default_Handler             ; 41:
                DCD     ACMP01_IRQHandler           ; 42: ACMP0 and ACMP1
                DCD     BPWM0_IRQHandler            ; 43: BPWM0
                DCD     GPH_IRQHandler              ; 44: GPIO Port H
                DCD     EINT7_IRQHandler            ; 45: External Input 7
                DCD     UART2_IRQHandler            ; 46: UART2
                DCD     Default_Handler             ; 47:
                DCD     USCI0_IRQHandler            ; 48: USCI0
                DCD     SPI1_IRQHandler             ; 49: SPI1
                DCD     SPI2_IRQHandler             ; 50: SPI2
                DCD     Default_Handler             ; 51:
                DCD     Default_Handler             ; 52:
                DCD     Default_Handler             ; 53:
                DCD     Default_Handler             ; 54:
                DCD     CRYPTO_IRQHandler           ; 55: CRYPTO
                DCD     Default_Handler             ; 56:
                DCD     I2C2_IRQHandler             ; 57: I2C2
                DCD     Default_Handler             ; 58:
                DCD     LCD_IRQHandler              ; 59: LCD
                DCD     CRC0_IRQHandler             ; 60: CRC0

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0

                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                IMPORT  ProcessHardFault
                EXPORT  HardFault_Handler         [WEAK]
;                B       .
                MOV     R0, LR
                MRS     R1, MSP
                MRS     R2, PSP
                LDR     R3, =ProcessHardFault
                BLX     R3
                BX      R0
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT      BOD_IRQHandler              [WEAK]
                EXPORT      IRC_IRQHandler              [WEAK]
                EXPORT      PWRWU_IRQHandler            [WEAK]
                EXPORT      Default_Handler             [WEAK]
                EXPORT      CLKFAIL_IRQHandler          [WEAK]
                EXPORT      FMC_IRQHandler              [WEAK]
                EXPORT      RTC_IRQHandler              [WEAK]
                EXPORT      WDT_IRQHandler              [WEAK]
                EXPORT      WWDT_IRQHandler             [WEAK]
                EXPORT      EINT0_IRQHandler            [WEAK]
                EXPORT      EINT1_IRQHandler            [WEAK]
                EXPORT      EINT2_IRQHandler            [WEAK]
                EXPORT      EINT3_IRQHandler            [WEAK]
                EXPORT      EINT4_IRQHandler            [WEAK]
                EXPORT      EINT5_IRQHandler            [WEAK]
                EXPORT      GPA_IRQHandler              [WEAK]
                EXPORT      GPB_IRQHandler              [WEAK]
                EXPORT      GPC_IRQHandler              [WEAK]
                EXPORT      GPD_IRQHandler              [WEAK]
                EXPORT      GPE_IRQHandler              [WEAK]
                EXPORT      GPF_IRQHandler              [WEAK]
                EXPORT      ETI_IRQHandler              [WEAK]
                EXPORT      SPI0_IRQHandler             [WEAK]
                EXPORT      GPG_IRQHandler              [WEAK]
                EXPORT      EINT6_IRQHandler            [WEAK]
                EXPORT      BRAKE0_IRQHandler           [WEAK]
                EXPORT      PWM0P0_IRQHandler           [WEAK]
                EXPORT      PWM0P1_IRQHandler           [WEAK]
                EXPORT      PWM0P2_IRQHandler           [WEAK]
                EXPORT      TMR0_IRQHandler             [WEAK]
                EXPORT      TMR1_IRQHandler             [WEAK]
                EXPORT      TMR2_IRQHandler             [WEAK]
                EXPORT      TMR3_IRQHandler             [WEAK]
                EXPORT      UART0_IRQHandler            [WEAK]
                EXPORT      UART1_IRQHandler            [WEAK]
                EXPORT      I2C0_IRQHandler             [WEAK]
                EXPORT      I2C1_IRQHandler             [WEAK]
                EXPORT      PDMA0_IRQHandler            [WEAK]
                EXPORT      ADC0_INT0_IRQHandler        [WEAK]
                EXPORT      ACMP01_IRQHandler           [WEAK]
                EXPORT      BPWM0_IRQHandler            [WEAK]
                EXPORT      GPH_IRQHandler              [WEAK]
                EXPORT      EINT7_IRQHandler            [WEAK]
                EXPORT      UART2_IRQHandler            [WEAK]
                EXPORT      USCI0_IRQHandler            [WEAK]
                EXPORT      SPI1_IRQHandler             [WEAK]
                EXPORT      SPI2_IRQHandler             [WEAK]
                EXPORT      CRYPTO_IRQHandler           [WEAK]
                EXPORT      I2C2_IRQHandler             [WEAK]
                EXPORT      LCD_IRQHandler              [WEAK]
                EXPORT      CRC0_IRQHandler             [WEAK]

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

                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
SH_DoCommand    PROC

                EXPORT      SH_DoCommand
                IMPORT      SH_Return

                BKPT   0xAB                ; Wait ICE or HardFault
                LDR    R3, =SH_Return
                PUSH   {R3 ,lr}
                BLX    R3                  ; Call SH_Return. The return value is in R0
                POP    {R3 ,PC}            ; Return value = R0

                ENDP

__PC            PROC
                EXPORT      __PC

                MOV     r0, lr
                BLX     lr
                ALIGN

                ENDP

				; *** Add for SPD Wakeup And Return by VTOR function sample code.
__Enter_SPD     PROC                        ; Enter to PD
                EXPORT      __Enter_SPD

				PUSH    {r0-r1}

                ; Disable interrupt
                MOVS    r0, #1
                MSR     PRIMASK, r0

                ; Set INIVTOR
                LDR     r0, =INIVTOR
                LDR     r1, =0x20000000
                STR     r1, [r0]

                ;Set __SPD_Wakeup at 0x20000004
                LDR     r0, =__SPD_Wakeup
                LDR     r1, =SPD_Mem
                STR     r0, [r1, #4]

                POP     {r0-r1}

                ; Backup r0-r7, lr to stack
                PUSH    {r0-r7,lr}

				; Save stack pointer at 0x20000000
                LDR     R1, =SPD_Mem
                MOV     R0, sp
                STR     R0, [R1]

                WFI

                ; Enable interrupt
                PUSH    {r0}
                MOVS    r0, #0
                MSR     PRIMASK, r0
                POP     {r0}

__SPD_Wakeup                                ; Wake-up from SPD
                ; Restore all registers and return
                POP     {r0-r7,pc}

                ENDP
				; *** End of SPD Wakeup And Return by VTOR function sample code.

                END
;/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

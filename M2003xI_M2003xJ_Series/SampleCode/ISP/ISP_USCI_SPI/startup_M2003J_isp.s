;/******************************************************************************
; * @file     startup_M2003J.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M Core Device Startup File for M2003J
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000400
	ENDIF

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

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

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
                DCD     Default_Handler           ; 22:
                DCD     Default_Handler           ; 23:
                DCD     Default_Handler           ; 24:
                DCD     Default_Handler           ; 25:
                DCD     Default_Handler           ; 26:
                DCD     Default_Handler           ; 27:
                DCD     Default_Handler           ; 28:
                DCD     Default_Handler           ; 29:
                DCD     Default_Handler           ; 30:
                DCD     Default_Handler           ; 31:
                DCD     TMR0_IRQHandler           ; 32: Timer 0
                DCD     TMR1_IRQHandler           ; 33: Timer 1
                DCD     TMR2_IRQHandler           ; 34: Timer 2
                DCD     TMR3_IRQHandler           ; 35: Timer 3
                DCD     UART0_IRQHandler          ; 36: UART0
                DCD     UART1_IRQHandler          ; 37: UART1
                DCD     I2C0_IRQHandler           ; 38: I2C0
                DCD     I2C1_IRQHandler           ; 39: I2C1
                DCD     PDMA_IRQHandler           ; 40: Peripheral DMA
                DCD     Default_Handler           ; 41:
                DCD     ADC_IRQHandler            ; 42: ADC interrupt
                DCD     Default_Handler           ; 43:
                DCD     ACMP01_IRQHandler         ; 44: ACMP0 and ACMP1
                DCD     Default_Handler           ; 45:
                DCD     Default_Handler           ; 46:
                DCD     Default_Handler           ; 47:
                DCD     UART2_IRQHandler          ; 48: UART2
                DCD     UART3_IRQHandler          ; 49: UART3
                DCD     Default_Handler           ; 50:
                DCD     Default_Handler           ; 51:
                DCD     USCI0_IRQHandler          ; 52: USCI0
                DCD     USCI1_IRQHandler          ; 53: USCI1
                DCD     Default_Handler           ; 54:
                DCD     Default_Handler           ; 55:
                DCD     Default_Handler           ; 56:
                DCD     CRC_IRQHandler            ; 57: CRC
                DCD     Default_Handler           ; 58:
                DCD     Default_Handler           ; 59:
                DCD     Default_Handler           ; 60:
                DCD     USCI2_IRQHandler          ; 61: USCI2
                DCD     USCI3_IRQHandler          ; 62: USCI3
                DCD     USCI4_IRQHandler          ; 63: USCI4
                DCD     TMR4_IRQHandler           ; 64: Timer 4
                DCD     TMR5_IRQHandler           ; 65: Timer 5
                DCD     TMR6_IRQHandler           ; 66: Timer 6
                DCD     TMR7_IRQHandler           ; 67: Timer 7
                DCD     TMR8_IRQHandler           ; 68: Timer 8
                DCD     Default_Handler           ; 69:
                DCD     Default_Handler           ; 70:
                DCD     Default_Handler           ; 71:
                DCD     GPG_IRQHandler            ; 72: GPIO Port G
                DCD     Default_Handler           ; 73:
                DCD     UART4_IRQHandler          ; 74: UART4
                DCD     Default_Handler           ; 75:
                DCD     Default_Handler           ; 76:
                DCD     BPWM0_IRQHandler          ; 77: BPWM0
                DCD     BPWM1_IRQHandler          ; 78: BPWM1
                DCD     Default_Handler           ; 79:
                DCD     Default_Handler           ; 80:
                DCD     DFMC_IRQHandler           ; 81: DFMC
                DCD     I2C2_IRQHandler           ; 82: I2C2
                DCD     Default_Handler           ; 83:
                DCD     Default_Handler           ; 84:
                DCD     Default_Handler           ; 85:
                DCD     Default_Handler           ; 86:

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                 LDR     R0, =0x40000100
                ; Unlock Register

                LDR     R1, =0x59
                STR     R1, [R0]
                LDR     R1, =0x16
                STR     R1, [R0]
                LDR     R1, =0x88
                STR     R1, [R0]

                ; enable SRAM1
                LDR     R3, =0x40000204
                LDR     R1, [R3]
                MOVW    R2, #0x200
                ORRS    R1,R1,R2
                STR     R1, [R3]


                ; Lock register
                MOVS    R1, #0
                STR     R1, [R0]

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
                EXPORT  HardFault_Handler         [WEAK]
                B       .
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

                EXPORT     BOD_IRQHandler           [WEAK]
                EXPORT     IRC_IRQHandler           [WEAK]
                EXPORT     PWRWU_IRQHandler         [WEAK]
                EXPORT     SR0ECC_Handler           [WEAK]
                EXPORT     CLKFAIL_IRQHandler       [WEAK]
                EXPORT     FMC_IRQHandler           [WEAK]
                EXPORT     RTC_IRQHandler           [WEAK]
                EXPORT     SR1ECC_Handler           [WEAK]
                EXPORT     WDT_IRQHandler           [WEAK]
                EXPORT     WWDT_IRQHandler          [WEAK]
                EXPORT     EINT0_IRQHandler         [WEAK]
                EXPORT     EINT1_IRQHandler         [WEAK]
                EXPORT     EINT2_IRQHandler         [WEAK]
                EXPORT     EINT3_IRQHandler         [WEAK]
                EXPORT     EINT4_IRQHandler         [WEAK]
                EXPORT     EINT5_IRQHandler         [WEAK]
                EXPORT     GPA_IRQHandler           [WEAK]
                EXPORT     GPB_IRQHandler           [WEAK]
                EXPORT     GPC_IRQHandler           [WEAK]
                EXPORT     GPD_IRQHandler           [WEAK]
                EXPORT     GPE_IRQHandler           [WEAK]
                EXPORT     GPF_IRQHandler           [WEAK]
                EXPORT     TMR0_IRQHandler          [WEAK]
                EXPORT     TMR1_IRQHandler          [WEAK]
                EXPORT     TMR2_IRQHandler          [WEAK]
                EXPORT     TMR3_IRQHandler          [WEAK]
                EXPORT     UART0_IRQHandler         [WEAK]
                EXPORT     UART1_IRQHandler         [WEAK]
                EXPORT     I2C0_IRQHandler          [WEAK]
                EXPORT     I2C1_IRQHandler          [WEAK]
                EXPORT     PDMA_IRQHandler          [WEAK]
                EXPORT     ADC_IRQHandler           [WEAK]
                EXPORT     ACMP01_IRQHandler        [WEAK]
                EXPORT     UART2_IRQHandler         [WEAK]
                EXPORT     UART3_IRQHandler         [WEAK]
                EXPORT     USCI0_IRQHandler         [WEAK]
                EXPORT     USCI1_IRQHandler         [WEAK]
                EXPORT     USCI2_IRQHandler         [WEAK]
                EXPORT     USCI3_IRQHandler         [WEAK]
                EXPORT     USCI4_IRQHandler         [WEAK]
                EXPORT     TMR4_IRQHandler          [WEAK]
                EXPORT     TMR5_IRQHandler          [WEAK]
                EXPORT     TMR6_IRQHandler          [WEAK]
                EXPORT     TMR7_IRQHandler          [WEAK]
                EXPORT     TMR8_IRQHandler          [WEAK]
                EXPORT     GPG_IRQHandler           [WEAK]
                EXPORT     UART4_IRQHandler         [WEAK]
                EXPORT     BPWM0_IRQHandler         [WEAK]
                EXPORT     BPWM1_IRQHandler         [WEAK]
                EXPORT     DFMC_IRQHandler          [WEAK]
                EXPORT     I2C2_IRQHandler          [WEAK]
                EXPORT     CRC_IRQHandler           [WEAK]

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
CRC_IRQHandler

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

                END
;/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

;/******************************************************************************
; * @file     startup_M471.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M4 Core Device Startup File for M471
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000800
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
                DCD     IRC_IRQHandler            ; 1: Internal RC
                DCD     PWRWU_IRQHandler          ; 2: Power down wake up
                DCD     RAMPE_IRQHandler          ; 3: RAM parity error
                DCD     CKFAIL_IRQHandler         ; 4: Clock detection fail
                DCD     FMC_IRQHandler            ; 5: FMC Handler
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
                DCD     Default_Handler           ; 45: 
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

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                ; Unlock Register
                LDR     R0, =0x40000100
                LDR     R1, =0x59
                STR     R1, [R0]
                LDR     R1, =0x16
                STR     R1, [R0]
                LDR     R1, =0x88
                STR     R1, [R0]

	IF :LNOT: :DEF: ENABLE_SPIM_CACHE
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
	ENDIF

                LDR     R0, =SystemInit
                BLX     R0

                ; Init POR
                ; LDR     R2, =0x40000024
                ; LDR     R1, =0x00005AA5
                ; STR     R1, [R2]

                ; Lock
                LDR     R0, =0x40000100
                LDR     R1, =0
                STR     R1, [R0]

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

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  IRC_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  RAMPE_IRQHandler          [WEAK]
                EXPORT  CKFAIL_IRQHandler         [WEAK]
                EXPORT  FMC_IRQHandler            [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT4_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPA_IRQHandler            [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPD_IRQHandler            [WEAK]
                EXPORT  GPE_IRQHandler            [WEAK]
                EXPORT  GPF_IRQHandler            [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  BRAKE0_IRQHandler         [WEAK]
                EXPORT  EPWM0P0_IRQHandler        [WEAK]
                EXPORT  EPWM0P1_IRQHandler        [WEAK]
                EXPORT  EPWM0P2_IRQHandler        [WEAK]
                EXPORT  BRAKE1_IRQHandler         [WEAK]
                EXPORT  EPWM1P0_IRQHandler        [WEAK]
                EXPORT  EPWM1P1_IRQHandler        [WEAK]
                EXPORT  EPWM1P2_IRQHandler        [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  I2C1_IRQHandler           [WEAK]
                EXPORT  PDMA_IRQHandler           [WEAK]
                EXPORT  DAC_IRQHandler            [WEAK]
                EXPORT  EADC0_INT0_IRQHandler     [WEAK]
                EXPORT  EADC0_INT1_IRQHandler     [WEAK]
                EXPORT  ACMP01_IRQHandler         [WEAK]
                EXPORT  EADC0_INT2_IRQHandler     [WEAK]
                EXPORT  EADC0_INT3_IRQHandler     [WEAK]
                EXPORT  UART2_IRQHandler          [WEAK]
                EXPORT  UART3_IRQHandler          [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  PRNG_IRQHandler           [WEAK]
                EXPORT  GPG_IRQHandler            [WEAK]
                EXPORT  EINT6_IRQHandler          [WEAK]
                EXPORT  UART4_IRQHandler          [WEAK]
                EXPORT  UART5_IRQHandler          [WEAK]
                EXPORT  BPWM0_IRQHandler          [WEAK]
                EXPORT  BPWM1_IRQHandler          [WEAK]
                EXPORT  GPH_IRQHandler            [WEAK]
                EXPORT  EINT7_IRQHandler          [WEAK]
                EXPORT  GPI_IRQHandler            [WEAK]
                EXPORT  CIR_IRQHandler            [WEAK]
					
Default__IRQHandler
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

                END
;/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

;/******************************************************************************
; * @file     mystartup_M2L31_Keil.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M23 Core Device Startup File for M2L31 sample code SYS_SPDMode_WakeupAndReturn.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
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
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

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

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

				; *** Add for SPD Wakeup And Return sample code.
                LDR     r0, =0x40000294 ; Check RTC wake-up from SPD flag
                LDR     r0, [r0, #0]
                MOVS    r1, #4
                ANDS    r0, r0, r1
                BEQ     NORMAL
SPD                                     ; Wake-up from SPD
                SUB     sp, sp, #12
                POP     {PC}            ; Execute __SPD_Wakeup
NORMAL                                  ; Normal Power-on process
                MOV     r0, #0          ; Reserve 3 words stack space to retain data
                PUSH    {r0}
                PUSH    {r0}
                PUSH    {r0}
				; *** End of SPD Wakeup And Return sample code.

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
                EXPORT      RAMPE_IRQHandler            [WEAK]
                EXPORT      CLKFAIL_IRQHandler           [WEAK]
                EXPORT      RRMC_IRQHandler             [WEAK]
                EXPORT      RTC_IRQHandler              [WEAK]
                EXPORT      TAMPER_IRQHandler           [WEAK]
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
                EXPORT      QSPI0_IRQHandler            [WEAK]
                EXPORT      SPI0_IRQHandler             [WEAK]
                EXPORT      EBRAKE0_IRQHandler          [WEAK]
                EXPORT      EPWM0P0_IRQHandler          [WEAK]
                EXPORT      EPWM0P1_IRQHandler          [WEAK]
                EXPORT      EPWM0P2_IRQHandler          [WEAK]
                EXPORT      EBRAKE1_IRQHandler          [WEAK]
                EXPORT      EPWM1P0_IRQHandler          [WEAK]
                EXPORT      EPWM1P1_IRQHandler          [WEAK]
                EXPORT      EPWM1P2_IRQHandler          [WEAK]
                EXPORT      TMR0_IRQHandler             [WEAK]
                EXPORT      TMR1_IRQHandler             [WEAK]
                EXPORT      TMR2_IRQHandler             [WEAK]
                EXPORT      TMR3_IRQHandler             [WEAK]
                EXPORT      UART0_IRQHandler            [WEAK]
                EXPORT      UART1_IRQHandler            [WEAK]
                EXPORT      I2C0_IRQHandler             [WEAK]
                EXPORT      I2C1_IRQHandler             [WEAK]
                EXPORT      PDMA0_IRQHandler            [WEAK]
                EXPORT      DAC_IRQHandler              [WEAK]
                EXPORT      EADC0_INT0_IRQHandler       [WEAK]
                EXPORT      EADC0_INT1_IRQHandler       [WEAK]
                EXPORT      ACMP01_IRQHandler           [WEAK]
                EXPORT      ACMP2_IRQHandler            [WEAK]
                EXPORT      EADC0_INT2_IRQHandler       [WEAK]
                EXPORT      EADC0_INT3_IRQHandler       [WEAK]
                EXPORT      UART2_IRQHandler            [WEAK]
                EXPORT      UART3_IRQHandler            [WEAK]
                EXPORT      SPI1_IRQHandler             [WEAK]
                EXPORT      SPI2_IRQHandler             [WEAK]
                EXPORT      USBD_IRQHandler             [WEAK]
                EXPORT      USBH_IRQHandler             [WEAK]
                EXPORT      USBOTG_IRQHandler           [WEAK]
                EXPORT      ETI_IRQHandler              [WEAK]
                EXPORT      CRC0_IRQHandler             [WEAK]
                EXPORT      SPI3_IRQHandler             [WEAK]
                EXPORT      TK_IRQHandler               [WEAK]
                EXPORT      OPA012_IRQHandler           [WEAK]
                EXPORT      CRPT_IRQHandler             [WEAK]
                EXPORT      GPG_IRQHandler              [WEAK]
                EXPORT      EINT6_IRQHandler            [WEAK]
                EXPORT      UART4_IRQHandler            [WEAK]
                EXPORT      UART5_IRQHandler            [WEAK]
                EXPORT      USCI0_IRQHandler            [WEAK]
                EXPORT      USCI1_IRQHandler            [WEAK]
                EXPORT      I2C2_IRQHandler             [WEAK]
                EXPORT      I2C3_IRQHandler             [WEAK]
                EXPORT      EQEI0_IRQHandler            [WEAK]
                EXPORT      EQEI1_IRQHandler            [WEAK]
                EXPORT      ECAP0_IRQHandler            [WEAK]
                EXPORT      ECAP1_IRQHandler            [WEAK]
                EXPORT      GPH_IRQHandler              [WEAK]
                EXPORT      EINT7_IRQHandler            [WEAK]
                EXPORT      LPPDMA0_IRQHandler          [WEAK]
                EXPORT      TRNG_IRQHandler             [WEAK]
                EXPORT      UART6_IRQHandler            [WEAK]
                EXPORT      UART7_IRQHandler            [WEAK]
                EXPORT      UTCPD_IRQHandler            [WEAK]
                EXPORT      CANFD00_IRQHandler          [WEAK]
                EXPORT      CANFD01_IRQHandler          [WEAK]
                EXPORT      CANFD10_IRQHandler          [WEAK]
                EXPORT      CANFD11_IRQHandler          [WEAK]
                EXPORT      BRAKE0_IRQHandler           [WEAK]
                EXPORT      PWM0P0_IRQHandler           [WEAK]
                EXPORT      PWM0P1_IRQHandler           [WEAK]
                EXPORT      PWM0P2_IRQHandler           [WEAK]
                EXPORT      BRAKE1_IRQHandler           [WEAK]
                EXPORT      PWM1P0_IRQHandler           [WEAK]
                EXPORT      PWM1P1_IRQHandler           [WEAK]
                EXPORT      PWM1P2_IRQHandler           [WEAK]
                EXPORT      LPADC0_IRQHandler           [WEAK]
                EXPORT      LPUART0_IRQHandler          [WEAK]
                EXPORT      LPI2C0_IRQHandler           [WEAK]
                EXPORT      LPSPI0_IRQHandler           [WEAK]
                EXPORT      LPTMR0_IRQHandler           [WEAK]
                EXPORT      LPTMR1_IRQHandler           [WEAK]
                EXPORT      TTMR0_IRQHandler            [WEAK]
                EXPORT      TTMR1_IRQHandler            [WEAK]

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

				; *** Add for SPD Wakeup And Return sample code.
__Enter_SPD     PROC                        ; Enter to PD
                EXPORT      __Enter_SPD

                LDR     r0, =__SPD_Wakeup   ; Save SP, LR and __SPD_Wakeup
                MOV     r1, lr
                MOV     r2, sp
                MOV     r3, #0
                LDR     r3, [r3]
                MOV     sp, r3
                PUSH    {r0-r2}
                WFI
                POP     {PC}                ; Execute __SPD_Wakeup
__SPD_Wakeup                                ; Restore SP and LR
                POP     {r1,r2}
                MOV     sp, r2
                BX      r1

                ENDP
				; *** End of SPD Wakeup And Return sample code.

				NOP

                END

;/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

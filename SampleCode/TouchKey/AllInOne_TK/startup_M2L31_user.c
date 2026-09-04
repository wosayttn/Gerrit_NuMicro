
/**************************************************************************//**
 * @file     startup_M2L31.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M2L31 Series
 *
 * SPDX-License-Identifier: Apache-2.0
 *****************************************************************************/

#include <inttypes.h>
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void);
__NO_RETURN void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handlers
 *----------------------------------------------------------------------------*/
/* Core System Handlers */
void NMI_Handler(void)              __attribute__((weak, alias("Default_Handler"))); /* NMI Handler */
void HardFault_Handler(void)        __attribute__((weak));
void MemManage_Handler(void)        __attribute__((weak, alias("Default_Handler"))); /* MPU Fault Handler */
void BusFault_Handler(void)         __attribute__((weak, alias("Default_Handler"))); /* Bus Fault Handler */
void UsageFault_Handler(void)       __attribute__((weak, alias("Default_Handler"))); /* Usage Fault Handler */
void SVC_Handler(void)              __attribute__((weak, alias("Default_Handler"))); /* SVCall Handler */
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler"))); /* PendSV Handler */
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler"))); /* SysTick Handler */

/* Peripheral Interrupt Handlers */
void BOD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 0: Brown Out detection */
void IRC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 1: Internal RC */
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 2: Power down wake up */
void RAMPE_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 3: RAM parity error */
void CLKFAIL_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 4: Clock detection fail */
void RRMC_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 5: RRMC (ISP) */
void RTC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 6: Real Time Clock */
void TAMPER_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));  /* 7: Tamper interrupt */
void WDT_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 8: Watchdog timer */
void WWDT_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 9: Window watchdog timer */
void EINT0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 10: External Input 0 */
void EINT1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 11: External Input 1 */
void EINT2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 12: External Input 2 */
void EINT3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 13: External Input 3 */
void EINT4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 14: External Input 4 */
void EINT5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 15: External Input 5 */
void GPA_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 16: GPIO Port A */
void GPB_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 17: GPIO Port B */
void GPC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 18: GPIO Port C */
void GPD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 19: GPIO Port D */
void GPE_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 20: GPIO Port E */
void GPF_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 21: GPIO Port F */
void QSPI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 22: QSPI0 */
void SPI0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 23: SPI0 */
void EBRAKE0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 24: */
void EPWM0P0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 25: */
void EPWM0P1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 26: */
void EPWM0P2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 27: */
void EBRAKE1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 28: */
void EPWM1P0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 29: */
void EPWM1P1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 30: */
void EPWM1P2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 31: */
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 32: Timer 0 */
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 33: Timer 1 */
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 34: Timer 2 */
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 35: Timer 3 */
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 36: UART0 */
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 37: UART1 */
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 38: I2C0 */
void I2C1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 39: I2C1 */
void PDMA0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 40: Peripheral DMA 0 */
void DAC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 41: DAC */
void EADC0_INT0_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));  /* 42: EADC0 interrupt source 0 */
void EADC0_INT1_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));  /* 43: EADC0 interrupt source 1 */
void ACMP01_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));  /* 44: ACMP0 and ACMP1 */
void ACMP2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 45: ACMP2 */
void EADC0_INT2_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));  /* 46: EADC0 interrupt source 2 */
void EADC0_INT3_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));  /* 47: EADC0 interrupt source 3 */
void UART2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 48: UART2 */
void UART3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 49: UART3 */
void SPI1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 51: SPI1 */
void SPI2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 52: SPI2 */
void USBD_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 53: USB device */
void USBH_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 54: USB host */
void USBOTG_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));  /* 55: USB OTG */
void ETI_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 56: ETI */
void CRC0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 57: CRC0 */
void SPI3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 62: SPI3 */
void TK_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 63: Touch Key */
void OPA012_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));  /* 70: OPA012 */
void CRPT_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 71: CRPT */
void GPG_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 721: GPIO Port G */
void EINT6_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 73: EINT6 */
void UART4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 74: UART4 */
void UART5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 75: UART5 */
void USCI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 76: USCI0 */
void USCI1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 77: USCI1 */
void I2C2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 82: I2C2 */
void I2C3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 83: I2C2 */
void EQEI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void EQEI1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void ECAP0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void ECAP1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void GPH_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));
void EINT7_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void LPPDMA0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void TRNG_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));
void UART6_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void UART7_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void UTCPD_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));
void CANFD00_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void CANFD01_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void CANFD10_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void CANFD11_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void BRAKE0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM0P0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM0P1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM0P2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void BRAKE1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM1P0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM1P1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void PWM1P2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void LPADC0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void LPUART0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));
void LPI2C0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void LPSPI0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void LPTMR0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void LPTMR1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));
void TTMR0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 142: TTMR0 */
void TTMR1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 143: TTMR1 */

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector Table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP), /* Initial Stack Pointer */
    Reset_Handler,                      /* Reset Handler */
    NMI_Handler,                        /* NMI Handler */
    HardFault_Handler,                  /* Hard Fault Handler */
    MemManage_Handler,                  /* MPU Fault Handler */
    BusFault_Handler,                   /* Bus Fault Handler */
    UsageFault_Handler,                 /* Usage Fault Handler */
    0,                                  /* Reserved */
    0,                                  /* Reserved */
    0,                                  /* Reserved */
    0,                                  /* Reserved */
    SVC_Handler,                        /* SVCall Handler */
    0,                                  /* Reserved */
    0,                                  /* Reserved */
    PendSV_Handler,                     /* PendSV Handler */
    SysTick_Handler,                    /* SysTick Handler */

    /* Peripheral Interrupts */
    BOD_IRQHandler,                     /* 0: Brown Out detection */
    IRC_IRQHandler,                     /* 1: Internal RC */
    PWRWU_IRQHandler,                   /* 2: Power down wake up */
	RAMPE_IRQHandler,            		/* 3: RAM parity error */
	CLKFAIL_IRQHandler,                 /* 4: Clock detection fail */
	RRMC_IRQHandler,                    /* 5: RRMC (ISP) */
	RTC_IRQHandler,                     /* 6: Real Time Clock */
	TAMPER_IRQHandler,                  /* 7: Tamper interrupt */
	WDT_IRQHandler,                     /* 8: Watchdog timer */
	WWDT_IRQHandler,                    /* 9: Window watchdog timer */
	EINT0_IRQHandler,                   /* 10: External Input 0 */
	EINT1_IRQHandler,                   /* 11: External Input 1 */
	EINT2_IRQHandler,                   /* 12: External Input 2 */
	EINT3_IRQHandler,                   /* 13: External Input 3 */
	EINT4_IRQHandler,                   /* 14: External Input 4 */
	EINT5_IRQHandler,                   /* 15: External Input 5 */
	GPA_IRQHandler,                     /* 16: GPIO Port A */
	GPB_IRQHandler,                     /* 17: GPIO Port B */
	GPC_IRQHandler,                     /* 18: GPIO Port C */
	GPD_IRQHandler,                     /* 19: GPIO Port D */
	GPE_IRQHandler,                     /* 20: GPIO Port E */
	GPF_IRQHandler,                     /* 21: GPIO Port F */
	QSPI0_IRQHandler,                   /* 22: QSPI0 */
	SPI0_IRQHandler,                    /* 23: SPI0 */
	EBRAKE0_IRQHandler,                 /* 24: */
	EPWM0P0_IRQHandler,                 /* 25: */
	EPWM0P1_IRQHandler,                 /* 26: */
	EPWM0P2_IRQHandler,                 /* 27: */
	EBRAKE1_IRQHandler,                 /* 28: */
	EPWM1P0_IRQHandler,                 /* 29: */
	EPWM1P1_IRQHandler,                 /* 30: */
	EPWM1P2_IRQHandler,                 /* 31: */
	TMR0_IRQHandler,                    /* 32: Timer 0 */
	TMR1_IRQHandler,                    /* 33: Timer 1 */
	TMR2_IRQHandler,                    /* 34: Timer 2 */
	TMR3_IRQHandler,                    /* 35: Timer 3 */
	UART0_IRQHandler,                   /* 36: UART0 */
	UART1_IRQHandler,                   /* 37: UART1 */
	I2C0_IRQHandler,                    /* 38: I2C0 */
	I2C1_IRQHandler,                    /* 39: I2C1 */
	PDMA0_IRQHandler,                   /* 40: Peripheral DMA 0 */
	DAC_IRQHandler,                     /* 41: DAC */
	EADC0_INT0_IRQHandler,              /* 42: EADC0 interrupt source 0 */
	EADC0_INT1_IRQHandler,              /* 43: EADC0 interrupt source 1 */
	ACMP01_IRQHandler,                  /* 44: ACMP0 and ACMP1 */
	ACMP2_IRQHandler,                   /* 45: ACMP2 */
	EADC0_INT2_IRQHandler,              /* 46: EADC0 interrupt source 2 */
	EADC0_INT3_IRQHandler,              /* 47: EADC0 interrupt source 3 */
	UART2_IRQHandler,                   /* 48: UART2 */
	UART3_IRQHandler,                   /* 49: UART3 */
	Default_Handler,                    /* 50: */
	SPI1_IRQHandler,                    /* 51: SPI1 */
	SPI2_IRQHandler,                    /* 52: SPI2 */
	USBD_IRQHandler,                    /* 53: USB device */
	USBH_IRQHandler,                    /* 54: USB host */
	USBOTG_IRQHandler,                  /* 55: USB OTG */
	ETI_IRQHandler,                     /* 56: ETI */
	CRC0_IRQHandler,                    /* 57: CRC0 */
	Default_Handler,                    /* 58: */
	Default_Handler,                    /* 59: */
	Default_Handler,                    /* 60: */
	Default_Handler,                    /* 61: */
	SPI3_IRQHandler,                    /* 62: SPI3	*/
	TK_IRQHandler,                      /* 63: Touch Key */
	Default_Handler,                    /* 64: */
	Default_Handler,                    /* 65: */
	Default_Handler,                    /* 66: */
	Default_Handler,                    /* 67: */
	Default_Handler,                    /* 68: */
	Default_Handler,                    /* 69: */
	OPA012_IRQHandler,                  /* 70: OPA012 */
	CRPT_IRQHandler,                    /* 71: CRPT */
	GPG_IRQHandler,                     /* 72: GPIO Port G */
	EINT6_IRQHandler,                   /* 73: External Input 6 */
	UART4_IRQHandler,                   /* 74: UART4 */
	UART5_IRQHandler,                   /* 75: UART5 */
	USCI0_IRQHandler,                   /* 76: USCI0 */
	USCI1_IRQHandler,                   /* 77: USCI1 */
	Default_Handler,                    /* 78: */
	Default_Handler,                    /* 79: */
	Default_Handler,                    /* 80: */
	Default_Handler,                    /* 81: */
	I2C2_IRQHandler,                    /* 82: I2C2 */
	I2C3_IRQHandler,                    /* 83: I2C3 */
	EQEI0_IRQHandler,                   /* 84: EQEI0 */
	EQEI1_IRQHandler,                   /* 85: EQEI1 */
	ECAP0_IRQHandler,                   /* 86: ECAP0 */
	ECAP1_IRQHandler,                   /* 87: ECAP1 */
	GPH_IRQHandler,                     /* 88: GPIO Port H */
	EINT7_IRQHandler,                   /* 89: External Input 7 */
	Default_Handler,                    /* 90: */
	Default_Handler,                    /* 91: */
	Default_Handler,                    /* 92: */
	Default_Handler,                    /* 93: */
	Default_Handler,                    /* 94: */
	Default_Handler,                    /* 95: */
	Default_Handler,                    /* 96: */
	Default_Handler,                    /* 97: */
	LPPDMA0_IRQHandler,                 /* 98: LPPDMA0 */
	Default_Handler,                    /* 99: */
	Default_Handler,                    /* 100: */
	TRNG_IRQHandler,                    /* 101: TRNG */
	UART6_IRQHandler,                   /* 102: UART6 */
	UART7_IRQHandler,                   /* 103: UART7 */
	Default_Handler,                    /* 104: */
	Default_Handler,                    /* 105: */
	Default_Handler,                    /* 106: */
	Default_Handler,                    /* 107: */
	UTCPD_IRQHandler,                   /* 108: UTCPD */
	Default_Handler,                    /* 109: */
	Default_Handler,                    /* 110: */
	Default_Handler,                    /* 111: */
	CANFD00_IRQHandler,                 /* 112: CAN FD 00 */
	CANFD01_IRQHandler,                 /* 113: CAN FD 01 */
	CANFD10_IRQHandler,                 /* 114: CAN FD 10 */
	CANFD11_IRQHandler,                 /* 115: CAN FD 11 */
	Default_Handler,                    /* 116: */
	Default_Handler,                    /* 117: */
	Default_Handler,                    /* 118: */
	Default_Handler,                    /* 119: */
	Default_Handler,                    /* 120: */
	Default_Handler,                    /* 121: */
	Default_Handler,                    /* 122: */
	Default_Handler,                    /* 123: */
	Default_Handler,                    /* 124: */
	Default_Handler,                    /* 125: */
	Default_Handler,                    /* 126: */
	Default_Handler,                    /* 127: */
	BRAKE0_IRQHandler,                  /* 128: BRAKE0 */
	PWM0P0_IRQHandler,                  /* 129: PWM0P0 */
	PWM0P1_IRQHandler,                  /* 130: PWM0P1 */
	PWM0P2_IRQHandler,                  /* 131: PWM0P2 */
	BRAKE1_IRQHandler,                  /* 132: BRAKE1 */
	PWM1P0_IRQHandler,                  /* 133: PWM1P0 */
	PWM1P1_IRQHandler,                  /* 134: PWM1P1 */
	PWM1P2_IRQHandler,                  /* 135: PWM1P2 */
	LPADC0_IRQHandler,                  /* 136: LPADC0 */
	LPUART0_IRQHandler,                 /* 137: LPUART0 */
	LPI2C0_IRQHandler,                  /* 138: LPI2C0 */
	LPSPI0_IRQHandler,                  /* 139: LPSPI0 */
	LPTMR0_IRQHandler,                  /* 140: LPTMR0 */
	LPTMR1_IRQHandler,                  /* 141: LPTMR1 */
	TTMR0_IRQHandler,                   /* 142: TTMR0 */
    TTMR1_IRQHandler                    /* 143: TTMR1 */
};

#if defined ( __GNUC__ )
    #pragma GCC diagnostic pop
#endif

__WEAK void Reset_Handler_PreInit(void)
{
    // Empty function
}

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
    __set_PSP((uint32_t)(&__INITIAL_SP));
    __set_MSP((uint32_t)(&__STACK_LIMIT));
    __set_PSP((uint32_t)(&__STACK_LIMIT));

    Reset_Handler_PreInit();
    /* Unlock protected registers */
    SYS_UnlockReg();

    SystemInit();               /* CMSIS System Initialization */

    /* Init POR */
    SYS->PORCTL = 0x5AA5;

    /* Lock protected registers */
    SYS_LockReg();

    __PROGRAM_START();          /* Enter PreMain (C library entry point) */
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*---------------------------------------------------------------------------
  Hard Fault Handler
 *---------------------------------------------------------------------------*/
__WEAK void HardFault_Handler(void)
{
    __ASM(
        "MOV     R0, LR  \n"
        "MRS     R1, MSP \n"
        "MRS     R2, PSP \n"
        "LDR     R3, =ProcessHardFault \n"
        "BLX     R3 \n"
        "BX      R0 \n"
    );
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void)
{
    while (1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic pop
#endif

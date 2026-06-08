
/**************************************************************************//**
 * @file     startup_M2003.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M2003 Series
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
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 2: Power down wake up */
void CLKFAIL_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));  /* 4: Clock fail detected */
void ISP_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 5: ISP */
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
void PWM0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 25: PWM0 */
void PWM1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 26: PWM1 */
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 32: Timer 0 */
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 33: Timer 1 */
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 34: Timer 2 */
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 35: Timer 3 */
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 36: UART0 */
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 37: UART1 */
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 38: I2C0 */
void PDMA_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 40: PDMA */
void ADC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 42: ADC */
void UART2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 48: UART2 */
void USCI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 52: USCI0 */
void USCI1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 53: USCI1 */
void USCI2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 54: USCI2 */
void USCI3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 55: USCI3 */
void CAN0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 56: CAN0 */
void CRC0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 57: CRC0 */
void ECAP0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 60: ECAP0 */
void GPG_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 72: GPIO Port G */


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
    Default_Handler,                    /* 1: */
    PWRWU_IRQHandler,                   /* 2: Power down wake up */
	Default_Handler,                    /* 3: */
	CLKFAIL_IRQHandler,                 /* 4: Clock fail detected */
	ISP_IRQHandler,                     /* 5: ISP */
	Default_Handler,                    /* 6: */
	Default_Handler,                    /* 7: */
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
	Default_Handler,                    /* 23: */
	Default_Handler,                    /* 24: */
	PWM0_IRQHandler,                    /* 25: PWM0 */
	PWM1_IRQHandler,                    /* 26: PWM1 */
	Default_Handler,                    /* 27: */
	Default_Handler,                    /* 28: */
	Default_Handler,                    /* 29: */
	Default_Handler,                    /* 30: */
	Default_Handler,                    /* 31: */
	TMR0_IRQHandler,                    /* 32: Timer 0 */
	TMR1_IRQHandler,                    /* 33: Timer 1 */
	TMR2_IRQHandler,                    /* 34: Timer 2 */
	TMR3_IRQHandler,                    /* 35: Timer 3 */
	UART0_IRQHandler,                   /* 36: UART0 */
	UART1_IRQHandler,                   /* 37: UART1 */
	I2C0_IRQHandler,                    /* 38: I2C0 */
	Default_Handler,                    /* 39: */
	PDMA_IRQHandler,                    /* 40: PDMA */
	Default_Handler,                    /* 41: */
	ADC_IRQHandler,                     /* 42: ADC */
	Default_Handler,                    /* 43: */
	Default_Handler,                    /* 44: */
	Default_Handler,                    /* 45: */
	Default_Handler,                    /* 46: */
	Default_Handler,                    /* 47: */
	UART2_IRQHandler,                   /* 48: UART2 */
	Default_Handler,                    /* 49: */
	Default_Handler,                    /* 50: */
	Default_Handler,                    /* 51: */
	USCI0_IRQHandler,                   /* 52: USCI0 */
	USCI1_IRQHandler,                   /* 53: USCI1 */
	USCI2_IRQHandler,                   /* 54: USCI2 */
	USCI3_IRQHandler,                   /* 55: USCI3 */
	CAN0_IRQHandler,                    /* 56: CAN0 */
	CRC0_IRQHandler,                    /* 57: CRC0 */
	Default_Handler,                    /* 58: */
	Default_Handler,                    /* 59: */
	ECAP0_IRQHandler,                   /* 60: ECAP0 */
	Default_Handler,                    /* 61: */
	Default_Handler,                    /* 62: */
	Default_Handler,                    /* 63: */
	Default_Handler,                    /* 64: */
	Default_Handler,                    /* 65: */
	Default_Handler,                    /* 66: */
	Default_Handler,                    /* 67: */
	Default_Handler,                    /* 68: */
	Default_Handler,                    /* 69: */
	Default_Handler,                    /* 70: */
	Default_Handler,                    /* 71: */
	GPG_IRQHandler,                     /* 72: GPIO Port G */
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
    SYS->PORCTL0 = 0x5AA5;
    SYS->PORCTL1 = 0x5AA5;

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

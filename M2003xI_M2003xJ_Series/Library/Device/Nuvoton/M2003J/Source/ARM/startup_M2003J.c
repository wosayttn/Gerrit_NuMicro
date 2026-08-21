
/**************************************************************************//**
 * @file     startup_M2003J.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M2003J Series
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
void DebugMon_Handler(void)         __attribute__((weak, alias("Default_Handler"))); /* DebugMon_Handler */
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler"))); /* PendSV Handler */
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler"))); /* SysTick Handler */

/* Peripheral Interrupt Handlers */
void BOD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 0: Brown Out detection */
void IRC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 1: Internal RC Trim */
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 2: Power down wake up */
void SR0ECC_Handler(void)            __attribute__((weak, alias("Default_Handler")));  /* 3: SRAM0 error correcting code */
void CLKFAIL_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));  /* 4: Clock detection fail */
void FMC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 5: FMC(ISP) */
void RTC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 6: Real Time Clock */
void SR1ECC_Handler(void)            __attribute__((weak, alias("Default_Handler")));  /* 7: SRAM1 error correcting code */
void WDT_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 8: Watchdog timer */
void WWDT_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 9: Window watchdog timer */
void EINT0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 10: External Input 0 */
void EINT1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 11: External Input 1 */
void EINT2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 12: External Input 2 */
void EINT3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 13: External Input 3 */
void EINT4_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 14: External Input 4 */
void EINT5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 15: External Input 5 */
void GPA_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 16: GPIO Port A */
void GPB_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 17: GPIO Port B */
void GPC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 18: GPIO Port C */
void GPD_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 19: GPIO Port D */
void GPE_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 20: GPIO Port E */
void GPF_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 21: GPIO Port F */
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 32: Timer 0 */
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 33: Timer 1 */
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 34: Timer 2 */
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 35: Timer 3 */
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 36: UART0 */
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 37: UART1 */
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 38: I2C0 */
void I2C1_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 39: I2C1 */
void PDMA_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 40: Peripheral DMA */
void ADC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 42: ADC interrupt */
void ACMP01_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));  /* 44: ACMP0 and ACMP1 */
void UART2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 48: UART2 */
void UART3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 49: UART3 */
void USCI0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 52: USCI0 */
void USCI1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 53: USCI1 */
void CRC_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 57: CRC */
void USCI2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 61: USCI2 */
void USCI3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 62: USCI3 */
void USCI4_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 63: USCI4 */
void TMR4_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 64: Timer 4 */
void TMR5_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 65: Timer 5 */
void TMR6_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 66: Timer 6 */
void TMR7_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 67: Timer 7 */
void TMR8_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 68: Timer 8 */
void GPG_IRQHandler(void)            __attribute__((weak, alias("Default_Handler")));  /* 72: GPIO Port G */
void UART4_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 74: UART4 */
void BPWM0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 77: BPWM0 */
void BPWM1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));  /* 78: BPWM1 */
void DFMC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 81: DFMC */
void I2C2_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));  /* 82: I2C2 */


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
    DebugMon_Handler,                   /* DebugMon Handler */
    0,                                  /* Reserved */
    PendSV_Handler,                     /* PendSV Handler */
    SysTick_Handler,                    /* SysTick Handler */

    /* Peripheral Interrupts */
    BOD_IRQHandler,                     /* 0: Brown Out detection */
    IRC_IRQHandler,                     /* 1: Internal RC Trim */
    PWRWU_IRQHandler,                   /* 2: Power down wake up */
    SR0ECC_Handler,                     /* 3: SRAM0 error correcting code */
    CLKFAIL_IRQHandler,                 /* 4: Clock detection fail */
    FMC_IRQHandler,                     /* 5: FMC(ISP) */
    RTC_IRQHandler,                     /* 6: Real Time Clock */
    SR1ECC_Handler,                     /* 7: SRAM1 error correcting code */
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
    0,                                  /* 22: */
    0,                                  /* 23: */
    0,                                  /* 24: */
    0,                                  /* 25: */
    0,                                  /* 26: */
    0,                                  /* 27: */
    0,                                  /* 28: */
    0,                                  /* 29: */
    0,                                  /* 30: */
    0,                                  /* 31: */
    TMR0_IRQHandler,                    /* 32: Timer 0 */
    TMR1_IRQHandler,                    /* 33: Timer 1 */
    TMR2_IRQHandler,                    /* 34: Timer 2 */
    TMR3_IRQHandler,                    /* 35: Timer 3 */
    UART0_IRQHandler,                   /* 36: UART0 */
    UART1_IRQHandler,                   /* 37: UART1 */
    I2C0_IRQHandler,                    /* 38: I2C0 */
    I2C1_IRQHandler,                    /* 39: I2C1 */
    PDMA_IRQHandler,                    /* 40: Peripheral DMA */
    0,                                  /* 41: Reserved */
    ADC_IRQHandler,                     /* 42: ADC interrupt */
    0,                                  /* 43: Reserved */
    ACMP01_IRQHandler,                  /* 44: ACMP0 and ACMP1 */
    0,                                  /* 45: Reserved */
    0,                                  /* 46: Reserved */
    0,                                  /* 47: Reserved */
    UART2_IRQHandler,                   /* 48: UART2 */
    UART3_IRQHandler,                   /* 49: UART3 */
    0,                                  /* 50: Reserved */
    0,                                  /* 51: Reserved */
    USCI0_IRQHandler,                   /* 52: USCI0 */
    USCI1_IRQHandler,                   /* 53: USCI1 */
    0,                                  /* 54: Reserved */
    0,                                  /* 55: Reserved */
    0,                                  /* 56: Reserved */
    CRC_IRQHandler,                     /* 57: CRC */
    0,                                  /* 58: Reserved */
    0,                                  /* 59: Reserved */
    0,                                  /* 60: Reserved */
    USCI2_IRQHandler,                   /* 61: USCI2 */
    USCI3_IRQHandler,                   /* 62: USCI3 */
    USCI4_IRQHandler,                   /* 63: USCI4 */
    TMR4_IRQHandler,                    /* 64: Timer 4 */
    TMR5_IRQHandler,                    /* 65: Timer 5 */
    TMR6_IRQHandler,                    /* 66: Timer 6 */
    TMR7_IRQHandler,                    /* 67: Timer 7 */
    TMR8_IRQHandler,                    /* 68: Timer 8 */
    0,                                  /* 69: Reserved */
    0,                                  /* 70: Reserved */
    0,                                  /* 71: Reserved */
    GPG_IRQHandler,                     /* 72: GPIO Port G */
    0,                                  /* 73: Reserved */
    UART4_IRQHandler,                   /* 74: UART4 */
    0,                                  /* 75: Reserved */
    0,                                  /* 76: Reserved */
    BPWM0_IRQHandler,                   /* 77: BPWM0 */
    BPWM1_IRQHandler,                   /* 78: BPWM1 */
    0,                                  /* 79: Reserved */
    0,                                  /* 80: Reserved */
    DFMC_IRQHandler,                    /* 81: DFMC */
    I2C2_IRQHandler,                    /* 82: I2C2 */
    0,                                  /* 83: Reserved */
    0,                                  /* 84: Reserved */
    0,                                  /* 85: Reserved */
    0,                                  /* 86: Reserved */
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
    SYS->PORCTL = 0x5AA55AA5;

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

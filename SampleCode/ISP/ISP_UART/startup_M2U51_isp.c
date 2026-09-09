/**************************************************************************//**
 * @file     startup_M2U51.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M2U51
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
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
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void)        __attribute__((weak));
void MemManage_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void)       __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)              __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler")));

/* External Interrupts */
void BOD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 0: Brown Out detection
void IRC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 1: Internal RC
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 2: Power down wake up
void CLKFAIL_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 4: Clock detection fail
void FMC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 5: FMC Handler
void RTC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 6: Real Time Clock
void WDT_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 8: Watchdog timer
void WWDT_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 9: Window watchdog timer
void EINT0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 10: External Input 0
void EINT1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 11: External Input 1
void EINT2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 12: External Input 2
void EINT3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 13: External Input 3
void EINT4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 14: External Input 4
void EINT5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 15: External Input 5
void GPA_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 16: GPIO Port A
void GPB_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 17: GPIO Port B
void GPC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 18: GPIO Port C
void GPD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 19: GPIO Port D
void GPE_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 20: GPIO Port E
void GPF_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 21: GPIO Port F
void ETI_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 22: ETI
void SPI0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 23: SPI0
void GPG_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 24: GPIO Port G
void EINT6_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 25: External Input 6
void BRAKE0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 26: BRAKE0
void PWM0P0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 27: PWM0P0
void PWM0P1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 28: PWM0P1
void PWM0P2_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 29: PWM0P2
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 30: Timer 0
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 31: Timer 1
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 32: Timer 2
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 33: Timer 3
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 34: UART0
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 35: UART1
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 36: I2C0
void I2C1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 37: I2C1
void PDMA0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 38: Peripheral DMA 0
void ADC0_INT0_IRQHandler(void)     __attribute__((weak, alias("Default_Handler")));    // 40: ADC0 interrupt source 0
void ACMP01_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 42: ACMP0 and ACMP1
void BPWM0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 43: BPWM0
void GPH_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 44: GPIO Port H
void EINT7_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 45: External Input 7
void UART2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 46: UART2
void USCI0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 48: USCI0
void SPI1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 49: SPI1
void SPI2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 50: SPI2
void CRYPTO_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 55: CRYPTO
void I2C2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 57: I2C2
void LCD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 59: LCD
void CRC0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 60: CRC0

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif


extern const VECTOR_TABLE_Type __VECTOR_TABLE[];
#if 1
const VECTOR_TABLE_Type __VECTOR_TABLE[] __VECTOR_TABLE_ATTRIBUTE =
{
    (VECTOR_TABLE_Type)(&__INITIAL_SP),     /*       Initial Stack Pointer          */
    Reset_Handler,                          /*       Reset Handler                  */
    NMI_Handler,                            /*   -14 NMI Handler                    */
    HardFault_Handler,                      /*   -13 Hard Fault Handler             */
    MemManage_Handler,                      /*   -12 MPU Fault Handler              */
    BusFault_Handler,                       /*   -11 Bus Fault Handler              */
    UsageFault_Handler,                     /*   -10 Usage Fault Handler            */
    0,                                      /*    -9 Reserved                       */
    0,                                      /*    -8 Reserved                       */
    0,                                      /*    -7 Reserved                       */
    0,                                      /*    -6 Reserved                       */
    SVC_Handler,                            /*    -5 SVC Handler                    */
    0,                                      /*    -4 Reserved                       */
    0,                                      /*    -3 Reserved                       */
    PendSV_Handler,                         /*    -2 PendSV Handler Handler         */
    SysTick_Handler,                        /*    -1 SysTick Handler                */

    /* Interrupts */
    BOD_IRQHandler,                         /*    0: Brown Out detection            */
    IRC_IRQHandler,                         /*    1: Internal RC                    */
    PWRWU_IRQHandler,                       /*    2: Power down wake up             */
    Default_Handler,                        /*    3:                                */
    CLKFAIL_IRQHandler,                     /*    4: Clock detection fail           */
    FMC_IRQHandler,                         /*    5: FMC Handler                    */
    RTC_IRQHandler,                         /*    6: Real Time Clock                */
    Default_Handler,                        /*    7: Reserved                       */
    WDT_IRQHandler,                         /*    8: Watchdog timer                 */
    WWDT_IRQHandler,                        /*    9: Window watchdog timer          */
    EINT0_IRQHandler,                       /*    10: External Input 0              */
    EINT1_IRQHandler,                       /*    11: External Input 1              */
    EINT2_IRQHandler,                       /*    12: External Input 2              */
    EINT3_IRQHandler,                       /*    13: External Input 3              */
    EINT4_IRQHandler,                       /*    14: External Input 4              */
    EINT5_IRQHandler,                       /*    15: External Input 5              */
    GPA_IRQHandler,                         /*    16: GPIO Port A                   */
    GPB_IRQHandler,                         /*    17: GPIO Port B                   */
    GPC_IRQHandler,                         /*    18: GPIO Port C                   */
    GPD_IRQHandler,                         /*    19: GPIO Port D                   */
    GPE_IRQHandler,                         /*    20: GPIO Port E                   */
    GPF_IRQHandler,                         /*    21: GPIO Port F                   */
    ETI_IRQHandler,                         /*    22: ETI                           */
    SPI0_IRQHandler,                        /*    23: SPI0                          */
    GPG_IRQHandler,                         /*    24: GPIO Port G                   */
    EINT6_IRQHandler,                       /*    25: External Input 6              */
    BRAKE0_IRQHandler,                      /*    26: BRAKE0                        */
    PWM0P0_IRQHandler,                      /*    27: PWM0P0                        */
    PWM0P1_IRQHandler,                      /*    28: PWM0P1                        */
    PWM0P2_IRQHandler,                      /*    29: PWM0P2                        */
    TMR0_IRQHandler,                        /*    30: Timer 0                       */
    TMR1_IRQHandler,                        /*    31: Timer 1                       */
    TMR2_IRQHandler,                        /*    32: Timer 2                       */
    TMR3_IRQHandler,                        /*    33: Timer 3                       */
    UART0_IRQHandler,                       /*    34: UART0                         */
    UART1_IRQHandler,                       /*    35: UART1                         */
    I2C0_IRQHandler,                        /*    36: I2C0                          */
    I2C1_IRQHandler,                        /*    37: I2C1                          */
    PDMA0_IRQHandler,                       /*    38: Peripheral DMA 0              */
    Default_Handler,                        /*    39:                               */
    ADC0_INT0_IRQHandler,                   /*    40: ADC0 interrupt source 0       */
    Default_Handler,                        /*    41:                               */
    ACMP01_IRQHandler,                      /*    42: ACMP0 and ACMP1               */
    BPWM0_IRQHandler,                       /*    43: BPWM0                         */
    GPH_IRQHandler,                         /*    44: GPIO Port H                   */
    EINT7_IRQHandler,                       /*    45: External Input 7              */
    UART2_IRQHandler,                       /*    46: UART2                         */
    Default_Handler,                        /*    47:                               */
    USCI0_IRQHandler,                       /*    48: USCI0                         */
    SPI1_IRQHandler,                        /*    49: SPI1                          */
    SPI2_IRQHandler,                        /*    50: SPI2                          */
    Default_Handler,                        /*    51:                               */
    Default_Handler,                        /*    52:                               */
    Default_Handler,                        /*    53:                               */
    Default_Handler,                        /*    54:                               */
    CRYPTO_IRQHandler,                      /*    55: CRYPTO                        */
    Default_Handler,                        /*    56:                               */
    I2C2_IRQHandler,                        /*    57: I2C2                          */
    Default_Handler,                        /*    58:                               */
    LCD_IRQHandler,                         /*    59: LCD                           */
    CRC0_IRQHandler,                        /*    60: CRC0                          */
};
#endif

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

#if 0
    /* Init POR */
    SYS->PORDISAN = 0x5AA5;
#endif

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
    while (1);
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

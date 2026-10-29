/**************************************************************************//**
 * @file     startup_M471.c
 * @version  V1.00
 * @brief    CMSIS Device Startup File for NuMicro M471
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
void DebugMon_Handler(void)         __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)           __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)          __attribute__((weak, alias("Default_Handler")));
    
/* External Interrupts */
void BOD_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 0: Brown Out detection
void IRC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 1: Internal RC
void PWRWU_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 2: Power down wake up
void RAMPE_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 3: RAM parity error
void CKFAIL_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 4: Clock detection fail
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
void SPI0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 23: SPI0
void BRAKE0_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 24: BRAKE0
void EPWM0P0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 25: EPWM0 P0
void EPWM0P1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 26: EPWM0 P1
void EPWM0P2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 27: EPWM0 P2
void BRAKE1_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 28: BRAKE1
void EPWM1P0_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 29: EPWM1 P0
void EPWM1P1_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 30: EPWM1 P1
void EPWM1P2_IRQHandler(void)       __attribute__((weak, alias("Default_Handler")));    // 31: EPWM1 P2
void TMR0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 32: Timer 0
void TMR1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 33: Timer 1
void TMR2_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 34: Timer 2
void TMR3_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 35: Timer 3
void UART0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 36: UART0
void UART1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 37: UART1
void I2C0_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 38: I2C0
void I2C1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 39: I2C1
void PDMA_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 40: Peripheral DMA
void DAC_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 41: DAC
void EADC0_INT0_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));    // 42: EADC0 interrupt source 0
void EADC0_INT1_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));    // 43: EADC0 interrupt source 1
void ACMP01_IRQHandler(void)        __attribute__((weak, alias("Default_Handler")));    // 44: ACMP0 and ACMP1
void EADC0_INT2_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));    // 46: EADC0 interrupt source 2
void EADC0_INT3_IRQHandler(void)    __attribute__((weak, alias("Default_Handler")));    // 47: EADC0 interrupt source 3
void UART2_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 48: UART2
void UART3_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 49: UART3
void SPI1_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 51: SPI1
void PRNG_IRQHandler(void)          __attribute__((weak, alias("Default_Handler")));    // 71: PRNG
void GPG_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 72: GPG
void EINT6_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 72: EINT6
void UART4_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 74: UART4
void UART5_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 75: UART5
void BPWM0_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 78: BPWM0
void BPWM1_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 79: BPWM1
void GPH_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 88: GPH
void EINT7_IRQHandler(void)         __attribute__((weak, alias("Default_Handler")));    // 89: EINT7
void GPI_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 110: GPI
void CIR_IRQHandler(void)           __attribute__((weak, alias("Default_Handler")));    // 111: CIR

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
    (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*       Initial Stack Pointer                            */
    Reset_Handler,                            /*       Reset Handler                                    */
    NMI_Handler,                              /*   -14 NMI Handler                                      */
    HardFault_Handler,                        /*   -13 Hard Fault Handler                               */
    MemManage_Handler,                        /*   -12 MPU Fault Handler                                */
    BusFault_Handler,                         /*   -11 Bus Fault Handler                                */
    UsageFault_Handler,                       /*   -10 Usage Fault Handler                              */
    0,                                        /*    -9 Reserved                                         */
    0,                                        /*    -8 Reserved                                         */
    0,                                        /*    -7 Reserved                                         */
    0,                                        /*    -6 Reserved                                         */
    SVC_Handler,                              /*    -5 SVC Handler                                      */
    DebugMon_Handler,                         /*    -4 Reserved                                         */
    0,                                        /*    -3 Reserved                                         */
    PendSV_Handler,                           /*    -2 PendSV Handler Handler                           */
    SysTick_Handler,                          /*    -1 SysTick Handler                                  */

    /* Interrupts */
    BOD_IRQHandler,                           /*    0: Brown Out detection                               */
    IRC_IRQHandler,                           /*    1: Internal RC                                       */
    PWRWU_IRQHandler,                         /*    2: Power down wake up                                */
    RAMPE_IRQHandler,                         /*    3: RAM parity error                                  */
    CKFAIL_IRQHandler,                        /*    4: Clock detection fail                              */
    FMC_IRQHandler,                           /*    5: FMC Handler                                       */
    RTC_IRQHandler,                           /*    6: Real Time Clock                                   */
    Default_Handler,                          /*    7: Reserved                                          */
    WDT_IRQHandler,                           /*    8: Watchdog timer                                    */
    WWDT_IRQHandler,                          /*    9: Window watchdog timer                             */
    EINT0_IRQHandler,                         /*    10: External Input 0                                 */
    EINT1_IRQHandler,                         /*    11: External Input 1                                 */
    EINT2_IRQHandler,                         /*    12: External Input 2                                 */
    EINT3_IRQHandler,                         /*    13: External Input 3                                 */
    EINT4_IRQHandler,                         /*    14: External Input 4                                 */
    EINT5_IRQHandler,                         /*    15: External Input 5                                 */
    GPA_IRQHandler,                           /*    16: GPIO Port A                                      */
    GPB_IRQHandler,                           /*    17: GPIO Port B                                      */
    GPC_IRQHandler,                           /*    18: GPIO Port C                                      */
    GPD_IRQHandler,                           /*    19: GPIO Port D                                      */
    GPE_IRQHandler,                           /*    20: GPIO Port E                                      */
    GPF_IRQHandler,                           /*    21: GPIO Port F                                      */
    Default_Handler,                          /*    22: Reserved                                         */
    SPI0_IRQHandler,                          /*    23: SPI0                                             */
    BRAKE0_IRQHandler,                        /*    24: BRAKE0                                           */
    EPWM0P0_IRQHandler,                       /*    25: PWM0P0                                           */
    EPWM0P1_IRQHandler,                       /*    26: PWM0P1                                           */
    EPWM0P2_IRQHandler,                       /*    27: PWM0P2                                           */
    BRAKE1_IRQHandler,                        /*    28: BRAKE1                                           */
    EPWM1P0_IRQHandler,                       /*    29: PWM1P0                                           */
    EPWM1P1_IRQHandler,                       /*    30: PWM1P1                                           */
    EPWM1P2_IRQHandler,                       /*    31: PWM1P2                                           */
    TMR0_IRQHandler,                          /*    32: Timer 0                                          */
    TMR1_IRQHandler,                          /*    33: Timer 1                                          */
    TMR2_IRQHandler,                          /*    34: Timer 2                                          */
    TMR3_IRQHandler,                          /*    35: Timer 3                                          */
    UART0_IRQHandler,                         /*    36: UART0                                            */
    UART1_IRQHandler,                         /*    37: UART1                                            */
    I2C0_IRQHandler,                          /*    38: I2C0                                             */
    I2C1_IRQHandler,                          /*    39: I2C1                                             */
    PDMA_IRQHandler,                          /*    40: Peripheral DMA                                   */
    DAC_IRQHandler,                           /*    41: DAC                                              */
    EADC0_INT0_IRQHandler,                    /*    42: EADC interrupt source 0                          */
    EADC0_INT1_IRQHandler,                    /*    43: EADC interrupt source 1                          */
    ACMP01_IRQHandler,                        /*    44: ACMP0 and ACMP1                                  */
    Default_Handler,                          /*    45: Reserved                                         */
    EADC0_INT2_IRQHandler,                    /*    46: EADC interrupt source 2                          */
    EADC0_INT3_IRQHandler,                    /*    47: EADC interrupt source 3                          */
    UART2_IRQHandler,                         /*    48: UART2                                            */
    UART3_IRQHandler,                         /*    49: UART3                                            */
    Default_Handler,                          /*    50: Reserved                                         */
    SPI1_IRQHandler,                          /*    51: SPI1                                             */
    Default_Handler,                          /*    52: Reserved                                         */
    Default_Handler,                          /*    53: Reserved                                         */
    Default_Handler,                          /*    54: Reserved                                         */
    Default_Handler,                          /*    55: Reserved                                         */
    Default_Handler,                          /*    56: Reserved                                         */
    Default_Handler,                          /*    57: Reserved                                         */
    Default_Handler,                          /*    58: Reserved                                         */
    Default_Handler,                          /*    59: Reserved                                         */
    Default_Handler,                          /*    60: Reserved                                         */
    Default_Handler,                          /*    61: Reserved                                         */
    Default_Handler,                          /*    62: Reserved                                         */
    Default_Handler,                          /*    63: Reserved                                         */
    Default_Handler,                          /*    64: Reserved                                         */    
    Default_Handler,                          /*    65: Reserved                                         */
    Default_Handler,                          /*    66: Reserved                                         */
    Default_Handler,                          /*    67: Reserved                                         */
    Default_Handler,                          /*    68: Reserved                                         */
    Default_Handler,                          /*    69: Reserved                                         */
    Default_Handler,                          /*    70: Reserved                                         */
    PRNG_IRQHandler,                          /*    71: PRNG                                             */
    GPG_IRQHandler,                           /*    72: GPG                                              */
    EINT6_IRQHandler,                         /*    73: EINT6                                            */
    UART4_IRQHandler,                         /*    74: UART4                                            */
    UART5_IRQHandler,                         /*    75: UART5                                            */
    Default_Handler,                          /*    76: Reserved                                         */
    Default_Handler,                          /*    77: Reserved                                         */
    BPWM0_IRQHandler,                         /*    78: BPWM0                                            */
    BPWM1_IRQHandler,                         /*    79: BPWM1                                            */
    Default_Handler,                          /*    80: Reserved                                         */
    Default_Handler,                          /*    81: Reserved                                         */
    Default_Handler,                          /*    82: Reserved                                         */
    Default_Handler,                          /*    83: Reserved                                         */
    Default_Handler,                          /*    84: Reserved                                         */
    Default_Handler,                          /*    85: Reserved                                         */
    Default_Handler,                          /*    86: Reserved                                         */
    Default_Handler,                          /*    87: Reserved                                         */
    GPH_IRQHandler,                           /*    88: GPH                                              */
    EINT7_IRQHandler,                         /*    89: EINT7                                            */
    Default_Handler,                          /*    90: Reserved                                         */
    Default_Handler,                          /*    91: Reserved                                         */
    Default_Handler,                          /*    92: Reserved                                         */
    Default_Handler,                          /*    93: Reserved                                         */
    Default_Handler,                          /*    94: Reserved                                         */
    Default_Handler,                          /*    95: Reserved                                         */
    Default_Handler,                          /*    96: Reserved                                         */
    Default_Handler,                          /*    97: Reserved                                         */
    Default_Handler,                          /*    98: Reserved                                         */
    Default_Handler,                          /*    99: Reserved                                         */
    Default_Handler,                          /*    100: Reserved                                        */
    Default_Handler,                          /*    101: Reserved                                        */
    Default_Handler,                          /*    102: Reserved                                        */
    Default_Handler,                          /*    103: Reserved                                        */
    Default_Handler,                          /*    104: Reserved                                        */
    Default_Handler,                          /*    105: Reserved                                        */
    Default_Handler,                          /*    106: Reserved                                        */
    Default_Handler,                          /*    107: Reserved                                        */
    Default_Handler,                          /*    108: Reserved                                        */
    Default_Handler,                          /*    109: Reserved                                        */
    GPI_IRQHandler,                           /*    110: GPI                                             */
    CIR_IRQHandler                            /*    111: CIR                                             */    
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

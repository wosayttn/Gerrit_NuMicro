/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M3351
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__

/* -------------------  Interrupt Number Definition  ------------------------------ */

typedef enum IRQn
{
    /* -----------------------  Processor Exceptions Numbers  ----------------------------- */
    NonMaskableInt_IRQn           = -14,      /*  2 */ /*!< Non Maskable Interrupt          */
    HardFault_IRQn                = -13,      /*  3 */ /*!< HardFault Interrupt             */
    MemoryManagement_IRQn         = -12,      /*  4 */ /*!< Memory Management Interrupt     */
    BusFault_IRQn                 = -11,      /*  5 */ /*!< Bus Fault Interrupt             */
    UsageFault_IRQn               = -10,      /*  6 */ /*!< Usage Fault Interrupt           */
    SecureFault_IRQn              =  -9,      /*  7 */ /*!< Secure Fault Interrupt          */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    SVCall_IRQn                   =  -5,      /* 11 */ /*!< SVC Interrupt                   */
    DebugMonitor_IRQn             =  -4,      /* 12 */ /*!< Debug Monitor Interrupt         */
    /* Reserved                                                                             */
    PendSV_IRQn                   =  -2,      /* 14 */ /*!< Pend SV Interrupt               */
    SysTick_IRQn                  =  -1,      /* 15 */ /*!< System Tick Interrupt           */

    /* -----------------------  Processor Interrupt Numbers  ------------------------------ */
    BODOUT_IRQn                   =   0,      /*!< Brown-Out Low Voltage Detected Interrupt */
    IRC_IRQn                      =   1,      /*!< IRC TRIM Interrupt                       */
    PWRWU_IRQn                    =   2,      /*!< Power Down Wake Up Interrupt             */
    SRAMPERR_IRQn                 =   3,      /*!< SRAM Parity Check Error Interrupt        */
    CKFAIL_IRQn                   =   4,      /*!< Clock Fail Detected Interrupt            */
    ISP_IRQn                      =   5,      /*!< FMC ISP Interrupt                        */
    RTC_IRQn                      =   6,      /*!< Real Time Clock Interrupt                */
    /* Reserved                                                                             */
    WDT0_IRQn                     =   8,      /*!< Watchdog Timer 0 Interrupt               */
    WWDT0_IRQn                    =   9,      /*!< Window Watchdog Timer 0 Interrupt        */

    EINT0_IRQn                    =  10,      /*!< External Input 0 Interrupt               */
    EINT1_IRQn                    =  11,      /*!< External Input 1 Interrupt               */
    EINT2_IRQn                    =  12,      /*!< External Input 2 Interrupt               */
    EINT3_IRQn                    =  13,      /*!< External Input 3 Interrupt               */
    EINT4_IRQn                    =  14,      /*!< External Input 4 Interrupt               */
    EINT5_IRQn                    =  15,      /*!< External Input 5 Interrupt               */
    GPA_IRQn                      =  16,      /*!< GPIO Port A Interrupt                    */
    GPB_IRQn                      =  17,      /*!< GPIO Port B Interrupt                    */
    GPC_IRQn                      =  18,      /*!< GPIO Port C Interrupt                    */
    GPD_IRQn                      =  19,      /*!< GPIO Port D Interrupt                    */

    GPE_IRQn                      =  20,      /*!< GPIO Port E Interrupt                    */
    GPF_IRQn                      =  21,      /*!< GPIO Port F Interrupt                    */
    QSPI0_IRQn                    =  22,      /*!< QSPI0 Interrupt                          */
    SPI0_IRQn                     =  23,      /*!< SPI0 Interrupt                           */
    BRAKE0_IRQn                   =  24,      /*!< PWM0 Brake Interrupt                     */
    PWM0P0_IRQn                   =  25,      /*!< PWM0 Pair 0 Interrupt                    */
    PWM0P1_IRQn                   =  26,      /*!< PWM0 Pair 1 Interrupt                    */
    PWM0P2_IRQn                   =  27,      /*!< PWM0 Pair 2 Interrupt                    */
    BRAKE1_IRQn                   =  28,      /*!< PWM1 Brake Interrupt                     */
    PWM1P0_IRQn                   =  29,      /*!< PWM1 Pair 0 Interrupt                    */

    PWM1P1_IRQn                   =  30,      /*!< PWM1 Pair 1 Interrupt                    */
    PWM1P2_IRQn                   =  31,      /*!< PWM1 Pair 2 Interrupt                    */
    TIMER0_IRQn                   =  32,      /*!< Timer0 Interrupt                         */
    TIMER1_IRQn                   =  33,      /*!< Timer1 Interrupt                         */
    TIMER2_IRQn                   =  34,      /*!< Timer2 Interrupt                         */
    TIMER3_IRQn                   =  35,      /*!< Timer3 Interrupt                         */
    UART0_IRQn                    =  36,      /*!< UART0 Interrupt                          */
    UART1_IRQn                    =  37,      /*!< UART1 Interrupt                          */
    I2C0_IRQn                     =  38,      /*!< I2C0 Interrupt                           */
    I2C1_IRQn                     =  39,      /*!< I2C1 Interrupt                           */

    PDMA0_IRQn                    =  40,      /*!< PDMA0 Interrupt                          */
    DAC0_IRQn                     =  41,      /*!< DAC0 Interrupt                           */
    EADC00_IRQn                   =  42,      /*!< EADC0 Interrupt 0                        */
    EADC01_IRQn                   =  43,      /*!< EADC0 Interrupt 1                        */
    ACMP01_IRQn                   =  44,      /*!< ACMP0 and ACMP1 Interrupt                */
    /* Reserved                                                                             */
    EADC02_IRQn                   =  46,      /*!< EADC0 Interrupt 2                        */
    EADC03_IRQn                   =  47,      /*!< EADC0 Interrupt 3                        */
    UART2_IRQn                    =  48,      /*!< UART2 Interrupt                          */
    UART3_IRQn                    =  49,      /*!< UART3 Interrupt                          */

    /* Reserved                                                                             */
    SPI1_IRQn                     =  51,      /*!< SPI1 Interrupt                           */
    /* Reserved                                                                             */
    USBD_IRQn                     =  53,      /*!< USB Device Interrupt                     */
    USBH_IRQn                     =  54,      /*!< USB Host Interrupt                       */
    /* Reserved                                                                             */
    ETI_IRQn                      =  56,      /*!< ETI Interrupt                            */
    CRC_IRQn                      =  57,      /*!< CRC Interrupt                            */
    /* Reserved                                                                             */
    NS_ISP_IRQn                   =  59,      /*!< FMC_NS ISP Interrupt                     */

    SCU_IRQn                      =  60,      /*!< SCU Interrupt                            */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    WDT1_IRQn                     =  67,      /*!< Watchdog Timer 1 Interrupt               */
    /* Reserved                                                                             */
    /* Reserved                                                                             */

    /* Reserved                                                                             */
    CRYPTO_IRQn                   =  71,      /*!< CRYPTO Interrupt                         */
    GPG_IRQn                      =  72,      /*!< GPIO Port G Interrupt                    */
    EINT6_IRQn                    =  73,      /*!< External Input 6 Interrupt               */
    UART4_IRQn                    =  74,      /*!< UART4 Interrupt                          */
    UART5_IRQn                    =  75,      /*!< UART5 Interrupt                          */
    USCI0_IRQn                    =  76,      /*!< USCI0 Interrupt                          */
    USCI1_IRQn                    =  77,      /*!< USCI1 Interrupt                          */
    BPWM0_IRQn                    =  78,      /*!< BPWM0 Interrupt                          */
    BPWM1_IRQn                    =  79,      /*!< BPWM1 Interrupt                          */

    /* Reserved                                                                             */
    /* Reserved                                                                             */
    I2C2_IRQn                     =  82,      /*!< I2C2 Interrupt                           */
    /* Reserved                                                                             */
    EQEI0_IRQn                    =  84,      /*!< EQEI0 Interrupt                          */
    EQEI1_IRQn                    =  85,      /*!< EQEI1 Interrupt                          */
    ECAP0_IRQn                    =  86,      /*!< ECAP0 Interrupt                          */
    /* Reserved                                                                             */
    GPH_IRQn                      =  88,      /*!< GPIO Port H Interrupt                    */
    EINT7_IRQn                    =  89,      /*!< External Input 7 Interrupt               */

    /* Reserved                                                                             */
    WWDT1_IRQn                    =  91,      /*!< Window Watchdog Timer 1 Interrupt        */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    NS_SRAMPE_IRQn                =  96,      /*!< NS SRAM parity check failed Interrupt    */
    CACHE_IRQn                    =  97,      /*!< CACHE parity error interrupt             */
    /* Reserved                                                                             */
    UART8_IRQn                    =  99,      /*!< UART8 Interrupt                          */

    UART9_IRQn                    = 100,      /*!< UART9 Interrupt                          */
    TRNG_IRQn                     = 101,      /*!< TRNG Interrupt                           */
    UART6_IRQn                    = 102,      /*!< UART6 Interrupt                          */
    UART7_IRQn                    = 103,      /*!< UART7 Interrupt                          */
    EADC10_IRQn                   = 104,      /*!< EADC1 Interrupt 0                        */
    EADC11_IRQn                   = 105,      /*!< EADC1 Interrupt 1                        */
    EADC12_IRQn                   = 106,      /*!< EADC1 Interrupt 2                        */
    EADC13_IRQn                   = 107,      /*!< EADC1 Interrupt 3                        */
    I3C0_IRQn                     = 108,      /*!< I3C0 Interrupt                           */
    /* Reserved                                                                             */

    /* Reserved                                                                             */
    DFMC_IRQn                     = 111,      /*!< DFMC Interrupt                           */
    CANFD00_IRQn                  = 112,      /*!< CANFD00 Interrupt                        */
    CANFD01_IRQn                  = 113,      /*!< CANFD01 Interrupt                        */
    CANFD10_IRQn                  = 114,      /*!< CANFD10 Interrupt                        */
    CANFD11_IRQn                  = 115,      /*!< CANFD11 Interrupt                        */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    NS_DFMC_IRQn                  = 119,      /*!< NS DFMC Interrupt                        */

    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    /* Reserved                                                                             */
    LLSI0_IRQn                    = 128,      /*!< LLSI0 Interrupt                          */
    LLSI1_IRQn                    = 129,      /*!< LLSI1 Interrupt                          */

    LLSI2_IRQn                    = 130,      /*!< LLSI2 Interrupt                          */
    LLSI3_IRQn                    = 131,      /*!< LLSI3 Interrupt                          */
    LLSI4_IRQn                    = 132,      /*!< LLSI4 Interrupt                          */
    LLSI5_IRQn                    = 133,      /*!< LLSI5 Interrupt                          */
    LLSI6_IRQn                    = 134,      /*!< LLSI6 Interrupt                          */
    LLSI7_IRQn                    = 135,      /*!< LLSI7 Interrupt                          */
    LLSI8_IRQn                    = 136,      /*!< LLSI8 Interrupt                          */
    LLSI9_IRQn                    = 137,      /*!< LLSI9 Interrupt                          */
    ELLSI0_IRQn                   = 138,      /*!< ELLSI0 Interrupt                         */
    ELLSI1_IRQn                   = 139,      /*!< ELLSI1 Interrupt                         */

    /** @cond DOXYGEN_IGNORE */
    IRQ_OFFSET                    = 16,       /* To omit MISRA_C Rule 10.4                  */
    TOTAL_IRQn_CNT                = ELLSI1_IRQn + IRQ_OFFSET,
    /** @endcond */
} IRQn_Type;

#endif /* __IRQN_H__ */
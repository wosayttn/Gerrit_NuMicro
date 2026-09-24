/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M2U51
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __IRQN_H__
#define __IRQN_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

/**
 * @details  Interrupt Number Definition.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers *****************************/
    NonMaskableInt_IRQn = -14,  /*!< 2 Non Maskable Interrupt                   */
    HardFault_IRQn      = -13,  /*!< 3 Cortex-M0 Hard Fault Interrupt           */
    SVCall_IRQn         = -5,   /*!< 11 Cortex-M0 SV Call Interrupt             */
    PendSV_IRQn         = -2,   /*!< 14 Cortex-M0 Pend SV Interrupt             */
    SysTick_IRQn        = -1,   /*!< 15 Cortex-M0 System Tick Interrupt         */

    /******  ARMIKMCU Swift specific Interrupt Numbers **************************/
    BOD_IRQn            = 0,    /*!< Brown-Out low voltage detected interrupt   */
    IRC_IRQn            = 1,    /*!< IRC TRIM interrupt                         */
    PWRWU_IRQn          = 2,    /*!< Clock controller interrupt for chip wake-up from power-down state */
    CLKFAIL_IRQn        = 4,    /*!< Clock fail detected interrupt              */
    FMC_IRQn            = 5,    /*!< Flash Memory Controller interrupt          */
    RTC_IRQn            = 6,    /*!< Real time clock interrupt                  */
    WDT_IRQn            = 8,    /*!< Watchdog Timer interrupt                   */
    WWDT_IRQn           = 9,    /*!< Window Watchdog Timer interrupt            */
    EINT0_IRQn          = 10,   /*!< External interrupt from INT0 pins          */
    EINT1_IRQn          = 11,   /*!< External interrupt from INT1 pins          */
    EINT2_IRQn          = 12,   /*!< External interrupt from INT2 pin           */
    EINT3_IRQn          = 13,   /*!< External interrupt from INT3 pin           */
    EINT4_IRQn          = 14,   /*!< External interrupt from INT4 pin           */
    EINT5_IRQn          = 15,   /*!< External interrupt from INT5 pin           */
    GPA_IRQn            = 16,   /*!< External interrupt from PA[15:0] pin       */
    GPB_IRQn            = 17,   /*!< External interrupt from PB[15:0] pin       */
    GPC_IRQn            = 18,   /*!< External interrupt from PC[15:0] pin       */
    GPD_IRQn            = 19,   /*!< External interrupt from PD[15:0] pin       */
    GPE_IRQn            = 20,   /*!< External interrupt from PE[15:0] pin       */
    GPF_IRQn            = 21,   /*!< External interrupt from PF[15:0] pin       */
    ETI_IRQn            = 22,   /*!< ETI interrupt                              */
    SPI0_IRQn           = 23,   /*!< SPI0 interrupt                             */
    GPG_IRQn            = 24,   /*!< External interrupt from PG[15:0] pin       */
    EINT6_IRQn          = 25,   /*!< External interrupt from INT6 pin           */
    BRAKE0_IRQn         = 26,   /*!< PWM0 brake interrupt                       */
    PWM0_P0_IRQn        = 27,   /*!< PWM0 pair 0 interrupt                      */
    PWM0_P1_IRQn        = 28,   /*!< PWM0 pair 1 interrupt                      */
    PWM0_P2_IRQn        = 29,   /*!< PWM0 pair 2 interrupt                      */
    TMR0_IRQn           = 30,   /*!< Timer 0 interrupt                          */
    TMR1_IRQn           = 31,   /*!< Timer 1 interrupt                          */
    TMR2_IRQn           = 32,   /*!< Timer 2 interrupt                          */
    TMR3_IRQn           = 33,   /*!< Timer 3 interrupt                          */
    UART0_IRQn          = 34,   /*!< UART0 interrupt                            */
    UART1_IRQn          = 35,   /*!< UART1 interrupt                            */
    I2C0_IRQn           = 36,   /*!< I2C0 interrupt                             */
    I2C1_IRQn           = 37,   /*!< I2C1 interrupt                             */
    PDMA0_IRQn          = 38,   /*!< PDMA0 interrupt                            */
    ADC0_INT0_IRQn      = 40,   /*!< ADC0 interrupt source 0                    */
    ACMP01_IRQn         = 42,   /*!< ACMP0 and ACMP1 interrupt                  */
    BPWM0_IRQn          = 43,   /*!< BPWM0 interrupt                            */
    GPH_IRQn            = 44,   /*!< External interrupt from PH[15:0] pin       */
    EINT7_IRQn          = 45,   /*!< External interrupt from INT7 pin           */
    UART2_IRQn          = 46,   /*!< UART2 interrupt                            */
    USCI0_IRQn          = 48,   /*!< USCI0 interrupt                            */
    SPI1_IRQn           = 49,   /*!< SPI1 interrupt                             */
    SPI2_IRQn           = 50,   /*!< SPI2 interrupt                             */
    CRPT_IRQn           = 55,   /*!< Crypto interrupt                           */
    I2C2_IRQn           = 57,   /*!< I2C2 interrupt                             */
    LCD_IRQn            = 59,   /*!< LCD interrupt                              */
    CRC0_IRQn           = 60,   /*!< CRC0 interrupt                             */
} IRQn_Type;

#endif /* __IRQN_H__ */
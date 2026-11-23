/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M2003
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
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
    /******  Cortex-M23 Processor Exceptions Numbers **********************************************/
    NonMaskableInt_IRQn       = -14,    /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,    /*!< 3 Cortex-M23 Hard Fault Interrupt                    */
    SVCall_IRQn               = -5,     /*!< 11 Cortex-M23 SV Call Interrupt                      */
    PendSV_IRQn               = -2,     /*!< 14 Cortex-M23 Pend SV Interrupt                      */
    SysTick_IRQn              = -1,     /*!< 15 Cortex-M23 System Tick Interrupt                  */

    /******  ARMIKMCU Swift specific Interrupt Numbers ********************************************/
    BOD_IRQn                  = 0,      /*!< Brown-Out Low Voltage Detected Interrupt             */
    PWRWU_IRQn                = 2,      /*!< Power down wake up Interrupt                         */
    CLKFAIL_IRQn              = 4,      /*!< Clock fail detected Interrupt                        */
    ISP_IRQn                  = 5,      /*!< FMC (ISP)                                            */
    WDT_IRQn                  = 8,      /*!< Watch Dog Timer Interrupt                            */
    WWDT_IRQn                 = 9,      /*!< Window Watch Dog Timer Interrupt                     */
    EINT0_IRQn                = 10,     /*!< External Input 0 Interrupt                           */
    EINT1_IRQn                = 11,     /*!< External Input 1 Interrupt                           */
    EINT2_IRQn                = 12,     /*!< External Input 2 Interrupt                           */
    EINT3_IRQn                = 13,     /*!< External Input 3 Interrupt                           */
    EINT4_IRQn                = 14,     /*!< External Input 4 Interrupt                           */
    EINT5_IRQn                = 15,     /*!< External Input 5 Interrupt                           */
    GPA_IRQn                  = 16,     /*!< GPIO PORT A Interrupt                                */
    GPB_IRQn                  = 17,     /*!< GPIO PORT B Interrupt                                */
    GPC_IRQn                  = 18,     /*!< GPIO PORT C Interrupt                                */
    GPD_IRQn                  = 19,     /*!< GPIO PORT D Interrupt                                */
    GPE_IRQn                  = 20,     /*!< GPIO PORT E Interrupt                                */
    GPF_IRQn                  = 21,     /*!< GPIO PORT F Interrupt                                */
    QSPI0_IRQn                = 22,     /*!< QSPI0 Interrupt                                      */
    PWM0_IRQn                 = 25,     /*!< PWM0 Interrupt                                       */
    PWM1_IRQn                 = 26,     /*!< PWM1 Interrupt                                       */
    TMR0_IRQn                 = 32,     /*!< TIMER0  Interrupt                                    */
    TMR1_IRQn                 = 33,     /*!< TIMER1  Interrupt                                    */
    TMR2_IRQn                 = 34,     /*!< TIMER2  Interrupt                                    */
    TMR3_IRQn                 = 35,     /*!< TIMER3  Interrupt                                    */
    UART0_IRQn                = 36,     /*!< UART0  Interrupt                                     */
    UART1_IRQn                = 37,     /*!< UART1  Interrupt                                     */
    I2C0_IRQn                 = 38,     /*!< I2C0  Interrupt                                      */
    PDMA0_IRQn                = 40,     /*!< PDMA Interrupt                                       */
    ADC_IRQn                  = 42,     /*!< ADC Interrupt                                        */
    UART2_IRQn                = 48,     /*!< UART2 Interrupt                                      */
    USCI0_IRQn                = 52,     /*!< USCI0 Interrupt                                      */
    USCI1_IRQn                = 53,     /*!< USCI1 Interrupt                                      */
    USCI2_IRQn                = 54,     /*!< USCI2 Interrupt                                      */
    USCI3_IRQn                = 55,     /*!< USCI3 Interrupt                                      */
    CAN0_IRQn                 = 56,     /*!< CAN0 Interrupt                                       */
    CRC0_IRQn                 = 57,     /*!< CRC0 Interrupt                                       */
    ECAP0_IRQn                = 60,     /*!< ECAP0 Interrupt                                      */
    GPG_IRQn                  = 72,     /*!< GPIO PORT G Interrupt                                */
} IRQn_Type;

#endif /* __IRQN_H__ */
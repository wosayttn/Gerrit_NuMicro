/**************************************************************************//**
 * @file     irqn.h
 * @version  V1.00
 * @brief    IRQ number definition for M2L31
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
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
    /******  Cortex-M0 Processor Exceptions Numbers ***********************************************/
    NonMaskableInt_IRQn = -14,  /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn      = -13,  /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn         = -5,   /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn         = -2,   /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn        = -1,   /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ********************************************/
    BOD_IRQn            = 0,    /*!< Brown-Out low voltage detected interrupt   */
    IRCTRIM_IRQn        = 1,    /*!< IRC TRIM interrupt                         */
    PWRWU_IRQn          = 2,    /*!< Clock controller interrupt for chip wake-up from power-down state */
    SRAM_PERR_IRQn      = 3,    /*!< SRAM parity check error interrupt          */
    CLKFAIL_IRQn        = 4,    /*!< Clock fail detected interrupt              */
    RMC_IRQn            = 5,    /*!< RRAM Memory Controller interrupt           */
    RTC_IRQn            = 6,    /*!< Real time clock interrupt                  */
    TAMPER_IRQn         = 7,    /*!< Backup register tamper interrupt           */
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
    QSPI0_IRQn          = 22,   /*!< QSPI0 interrupt                            */
    SPI0_IRQn           = 23,   /*!< SPI0 interrupt                             */
    EBRAKE0_IRQn        = 24,   /*!< EPWM0 brake interrupt                      */
    EPWM0_P0_IRQn       = 25,   /*!< EPWM0 pair 0 interrupt                     */
    EPWM0_P1_IRQn       = 26,   /*!< EPWM0 pair 1 interrupt                     */
    EPWM0_P2_IRQn       = 27,   /*!< EPWM0 pair 2 interrupt                     */
    EBRAKE1_IRQn        = 28,   /*!< EPWM1 brake interrupt                      */
    EPWM1_P0_IRQn       = 29,   /*!< EPWM1 pair 0 interrupt                     */
    EPWM1_P1_IRQn       = 30,   /*!< EPWM1 pair 1 interrupt                     */
    EPWM1_P2_IRQn       = 31,   /*!< EPWM1 pair 2 interrupt                     */
    TMR0_IRQn           = 32,   /*!< Timer 0 interrupt                          */
    TMR1_IRQn           = 33,   /*!< Timer 1 interrupt                          */
    TMR2_IRQn           = 34,   /*!< Timer 2 interrupt                          */
    TMR3_IRQn           = 35,   /*!< Timer 3 interrupt                          */
    UART0_IRQn          = 36,   /*!< UART0 interrupt                            */
    UART1_IRQn          = 37,   /*!< UART1 interrupt                            */
    I2C0_IRQn           = 38,   /*!< I2C0 interrupt                             */
    I2C1_IRQn           = 39,   /*!< I2C1 interrupt                             */
    PDMA0_IRQn          = 40,   /*!< PDMA0 interrupt                            */
    DAC_IRQn            = 41,   /*!< DAC interrupt                              */
    EADC0_INT0_IRQn     = 42,   /*!< EADC0 interrupt source 0                   */
    EADC0_INT1_IRQn     = 43,   /*!< EADC0 interrupt source 1                   */
    ACMP01_IRQn         = 44,   /*!< ACMP0 and ACMP1 interrupt                  */
    ACMP2_IRQn          = 45,   /*!< ACMP2 interrupt                            */
    EADC0_INT2_IRQn     = 46,   /*!< EADC0 interrupt source 2                   */
    EADC0_INT3_IRQn     = 47,   /*!< EADC0 interrupt source 3                   */
    UART2_IRQn          = 48,   /*!< UART2 interrupt                            */
    UART3_IRQn          = 49,   /*!< UART3 interrupt                            */
    RESERVED0           = 50,   /*!< Reserved                                   */
    SPI1_IRQn           = 51,   /*!< SPI1 interrupt                             */
    SPI2_IRQn           = 52,   /*!< SPI2 interrupt                             */
    USBD_IRQn           = 53,   /*!< USB device interrupt                       */
    USBH_IRQn           = 54,   /*!< USB host interrupt                         */
    USBOTG_IRQn         = 55,   /*!< USB OTG interrupt                          */
    ETI_IRQn            = 56,   /*!< External Trigger Interface interrupt       */
    CRC0_IRQn           = 57,   /*!< CRC0 interrupt                             */
    RESERVED1           = 58,   /*!< Reserved                                   */
    RESERVED2           = 59,   /*!< Reserved                                   */
    RESERVED3           = 60,   /*!< Reserved                                   */
    RESERVED4           = 61,   /*!< Reserved                                   */
    SPI3_IRQn           = 62,   /*!< SPI3 interrupt                             */
    TK_IRQn             = 63,   /*!< Touchkey interrupt                         */
    RESERVED5           = 64,   /*!< Reserved                                   */
    RESERVED6           = 65,   /*!< Reserved                                   */
    RESERVED7           = 66,   /*!< Reserved                                   */
    RESERVED8           = 67,   /*!< Reserved                                   */
    RESERVED9           = 68,   /*!< Reserved                                   */
    RESERVED10          = 69,   /*!< Reserved                                   */
    OPA_IRQn            = 70,   /*!< Analog OPA interrupt                       */
    CRPT_IRQn           = 71,   /*!< Crypto interrupt                           */
    GPG_IRQn            = 72,   /*!< External interrupt from PG[15:0] pin       */
    EINT6_IRQn          = 73,   /*!< External interrupt from INT6 pin           */
    UART4_IRQn          = 74,   /*!< UART4 interrupt                            */
    UART5_IRQn          = 75,   /*!< UART5 interrupt                            */
    USCI0_IRQn          = 76,   /*!< USCI0 interrupt                            */
    USCI1_IRQn          = 77,   /*!< USCI1 interrupt                            */
    RESERVED11          = 78,   /*!< Reserved                                   */
    RESERVED12          = 79,   /*!< Reserved                                   */
    RESERVED13          = 80,   /*!< Reserved                                   */
    RESERVED14          = 81,   /*!< Reserved                                   */
    I2C2_IRQn           = 82,   /*!< I2C2 interrupt                             */
    I2C3_IRQn           = 83,   /*!< I2C3 interrupt                             */
    EQEI0_IRQn          = 84,   /*!< EQEI0 interrupt                            */
    EQEI1_IRQn          = 85,   /*!< EQEI1 interrupt                            */
    ECAP0_IRQn          = 86,   /*!< ECAP0 interrupt                            */
    ECAP1_IRQn          = 87,   /*!< ECAP1 interrupt                            */
    GPH_IRQn            = 88,   /*!< External interrupt from PH[15:0] pin       */
    EINT7_IRQn          = 89,   /*!< External interrupt from INT7 pin           */
    RESERVED15          = 90,   /*!< Reserved                                   */
    RESERVED16          = 91,   /*!< Reserved                                   */
    RESERVED17          = 92,   /*!< Reserved                                   */
    RESERVED18          = 93,   /*!< Reserved                                   */
    RESERVED19          = 94,   /*!< Reserved                                   */
    RESERVED20          = 95,   /*!< Reserved                                   */
    RESERVED21          = 96,   /*!< Reserved                                   */
    RESERVED22          = 97,   /*!< Reserved                                   */
    LPPDMA0_IRQn        = 98,   /*!< LPPDMA0 interrupt                          */
    RESERVED23          = 99,   /*!< Reserved                                   */
    RESERVED24          = 100,  /*!< Reserved                                   */
    TRNG_IRQn           = 101,  /*!< TRNG interrupt                             */
    UART6_IRQn          = 102,  /*!< UART6 interrupt                            */
    UART7_IRQn          = 103,  /*!< UART7 interrupt                            */
    RESERVED25          = 104,  /*!< Reserved                                   */
    RESERVED26          = 105,  /*!< Reserved                                   */
    RESERVED27          = 106,  /*!< Reserved                                   */
    RESERVED28          = 107,  /*!< Reserved                                   */
    UTCPD_IRQn          = 108,  /*!< UTCPD interrupt                            */
    RESERVED29          = 109,  /*!< Reserved                                   */
    RESERVED30          = 110,  /*!< Reserved                                   */
    RESERVED31          = 111,  /*!< Reserved                                   */
    CANFD00_IRQn        = 112,  /*!< CANFD00 interrupt                          */
    CANFD01_IRQn        = 113,  /*!< CANFD01 interrupt                          */
    CANFD10_IRQn        = 114,  /*!< CANFD10 interrupt                          */
    CANFD11_IRQn        = 115,  /*!< CANFD11 interrupt                          */
    RESERVED32          = 116,  /*!< Reserved                                   */
    RESERVED33          = 117,  /*!< Reserved                                   */
    RESERVED34          = 118,  /*!< Reserved                                   */
    RESERVED35          = 119,  /*!< Reserved                                   */
    RESERVED36          = 120,  /*!< Reserved                                   */
    RESERVED37          = 121,  /*!< Reserved                                   */
    RESERVED38          = 122,  /*!< Reserved                                   */
    RESERVED39          = 123,  /*!< Reserved                                   */
    RESERVED40          = 124,  /*!< Reserved                                   */
    RESERVED41          = 125,  /*!< Reserved                                   */
    RESERVED42          = 126,  /*!< Reserved                                   */
    RESERVED43          = 127,  /*!< Reserved                                   */
    BRAKE0_IRQn         = 128,  /*!< PWM0 brake interrupt                       */
    PWM0_P0_IRQn        = 129,  /*!< PWM0 pair 0 interrupt                      */
    PWM0_P1_IRQn        = 130,  /*!< PWM0 pair 1 interrupt                      */
    PWM0_P2_IRQn        = 131,  /*!< PWM0 pair 2 interrupt                      */
    BRAKE1_IRQn         = 132,  /*!< PWM1 brake interrupt                       */
    PWM1_P0_IRQn        = 133,  /*!< PWM1 pair 0 interrupt                      */
    PWM1_P1_IRQn        = 134,  /*!< PWM1 pair 1 interrupt                      */
    PWM1_P2_IRQn        = 135,  /*!< PWM1 pair 2 interrupt                      */
    LPADC0_IRQn         = 136,  /*!< LPADC0 interrupt                           */
    LPUART0_IRQn        = 137,  /*!< LPUART0 interrupt                          */
    LPI2C0_IRQn         = 138,  /*!< LPI2C0 interrupt                           */
    LPSPI0_IRQn         = 139,  /*!< LPSPI0 interrupt                           */
    LPTMR0_IRQn         = 140,  /*!< LPTMR0 interrupt                           */
    LPTMR1_IRQn         = 141,  /*!< LPTMR1 interrupt                           */
    TTMR0_IRQn          = 142,  /*!< TTMR0 interrupt                            */
    TTMR1_IRQn          = 143,  /*!< TTMR1 interrupt                            */
} IRQn_Type;


#endif /* __IRQN_H__ */
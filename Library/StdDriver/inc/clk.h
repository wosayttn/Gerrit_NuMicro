/**************************************************************************//**
 * @file     clk.h
 * @version  V0.10
 * @brief    Clock Controller (CLK) driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__


#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup CLK_Driver CLK Driver
  @{
*/

/** @addtogroup CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  Frequency constant definitions.                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define FREQ_2MHZ       (  2000000UL)
#define FREQ_4MHZ       (  4000000UL)
#define FREQ_8MHZ       (  8000000UL)
#define FREQ_12MHZ      ( 12000000UL)
#define FREQ_21MHZ      ( 21000000UL)
#define FREQ_24MHZ      ( 24000000UL)
#define FREQ_25MHZ      ( 25000000UL)
#define FREQ_32MHZ      ( 32000000UL)
#define FREQ_40MHZ      ( 40000000UL)
#define FREQ_48MHZ      ( 48000000UL)
#define FREQ_50MHZ      ( 50000000UL)
#define FREQ_64MHZ      ( 64000000UL)
#define FREQ_75MHZ      ( 75000000UL)
#define FREQ_84MHZ      ( 84000000UL)
#define FREQ_96MHZ      ( 96000000UL)
#define FREQ_128MHZ     (128000000UL)
#define FREQ_144MHZ     (144000000UL)
#define FREQ_160MHZ     (160000000UL)
#define FREQ_200MHZ     (200000000UL)


/*---------------------------------------------------------------------------------------------------------*/
/*  HCLKSEL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_HCLKSEL_HCLKSEL_HIRC        (0x0UL<<CLK_HCLKSEL_HCLKSEL_Pos)    /*!< Setting HCLK clock source as HIRC */
#define CLK_HCLKSEL_HCLKSEL_LIRC        (0x1UL<<CLK_HCLKSEL_HCLKSEL_Pos)    /*!< Setting HCLK clock source as LIRC */
#define CLK_HCLKSEL_HCLKSEL_LXT         (0x2UL<<CLK_HCLKSEL_HCLKSEL_Pos)    /*!< Setting HCLK clock source as LXT */
#define CLK_HCLKSEL_HCLKSEL_HXT         (0x3UL<<CLK_HCLKSEL_HCLKSEL_Pos)    /*!< Setting HCLK clock source as HXT */
#define CLK_HCLKSEL_HCLKSEL_PLL         (0x4UL<<CLK_HCLKSEL_HCLKSEL_Pos)    /*!< Setting HCLK clock source as PLL */

#define CLK_CLKSEL0_HCLKSEL_HIRC        (CLK_HCLKSEL_HCLKSEL_HIRC)  /*!< Setting HCLK clock source as HIRC */
#define CLK_CLKSEL0_HCLKSEL_LIRC        (CLK_HCLKSEL_HCLKSEL_LIRC)  /*!< Setting HCLK clock source as LIRC */
#define CLK_CLKSEL0_HCLKSEL_LXT         (CLK_HCLKSEL_HCLKSEL_LXT)   /*!< Setting HCLK clock source as LXT */
#define CLK_CLKSEL0_HCLKSEL_HXT         (CLK_HCLKSEL_HCLKSEL_HXT)   /*!< Setting HCLK clock source as HXT */
#define CLK_CLKSEL0_HCLKSEL_PLL         (CLK_HCLKSEL_HCLKSEL_PLL)   /*!< Setting HCLK clock source as PLL */

/*---------------------------------------------------------------------------------------------------------*/
/*  IP clock source select constant definitions.                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_BPWMSEL_BPWM0SEL_PCLK0      (0x0UL << CLK_BPWMSEL_BPWM0SEL_Pos)     /*!< Select BPWM0 clock source from PCLK0 \hideinitializer */
#define CLK_BPWMSEL_BPWM0SEL_HCLK       (0x1UL << CLK_BPWMSEL_BPWM0SEL_Pos)     /*!< Select BPWM0 clock source from HCLK \hideinitializer */

#define CLK_BPWMSEL_BPWM1SEL_PCLK1      (0x0UL << CLK_BPWMSEL_BPWM1SEL_Pos)     /*!< Select BPWM1 clock source from PCLK1 \hideinitializer */
#define CLK_BPWMSEL_BPWM1SEL_HCLK       (0x1UL << CLK_BPWMSEL_BPWM1SEL_Pos)     /*!< Select BPWM1 clock source from HCLK \hideinitializer */

#define CLK_CLKOSEL_CLKOSEL_HCLK        (0x0UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from HCLK \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_PCLK0       (0x1UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from PCLK0 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_PCLK1       (0x2UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from PCLK1 \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_LIRC        (0x3UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from LIRC \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_LXT         (0x4UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from LXT \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HXT         (0x5UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from HXT \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_HIRC        (0x6UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from HIRC \hideinitializer */
#define CLK_CLKOSEL_CLKOSEL_PLL         (0x7UL << CLK_CLKOSEL_CLKOSEL_Pos)      /*!< Select CLKO clock source from PLL \hideinitializer */

#define CLK_TMRSEL0_TMR0SEL_PCLK0       (0x0UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as PCLK0 */
#define CLK_TMRSEL0_TMR0SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as LIRC */
#define CLK_TMRSEL0_TMR0SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as LXT */
#define CLK_TMRSEL0_TMR0SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as HXT */
#define CLK_TMRSEL0_TMR0SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as HIRC */
#define CLK_TMRSEL0_TMR0SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR0SEL_Pos)        /*!< Setting Timer 0 clock source as EXT */

#define CLK_TMRSEL0_TMR1SEL_PCLK0       (0x0UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as PCLK0 */
#define CLK_TMRSEL0_TMR1SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as LIRC */
#define CLK_TMRSEL0_TMR1SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as LXT */
#define CLK_TMRSEL0_TMR1SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as HXT */
#define CLK_TMRSEL0_TMR1SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as HIRC */
#define CLK_TMRSEL0_TMR1SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR1SEL_Pos)        /*!< Setting Timer 1 clock source as EXT */

#define CLK_TMRSEL0_TMR2SEL_PCLK1       (0x0UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as PCLK1 */
#define CLK_TMRSEL0_TMR2SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as LIRC */
#define CLK_TMRSEL0_TMR2SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as LXT */
#define CLK_TMRSEL0_TMR2SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as HXT */
#define CLK_TMRSEL0_TMR2SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as HIRC */
#define CLK_TMRSEL0_TMR2SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR2SEL_Pos)        /*!< Setting Timer 2 clock source as EXT */

#define CLK_TMRSEL0_TMR3SEL_PCLK1       (0x0UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as PCLK1 */
#define CLK_TMRSEL0_TMR3SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as LIRC */
#define CLK_TMRSEL0_TMR3SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as LXT */
#define CLK_TMRSEL0_TMR3SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as HXT */
#define CLK_TMRSEL0_TMR3SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as HIRC */
#define CLK_TMRSEL0_TMR3SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR3SEL_Pos)        /*!< Setting Timer 3 clock source as EXT */

#define CLK_TMRSEL0_TMR4SEL_PCLK0       (0x0UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as PCLK0 */
#define CLK_TMRSEL0_TMR4SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as LIRC */
#define CLK_TMRSEL0_TMR4SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as LXT */
#define CLK_TMRSEL0_TMR4SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as HXT */
#define CLK_TMRSEL0_TMR4SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as HIRC */
#define CLK_TMRSEL0_TMR4SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR4SEL_Pos)        /*!< Setting Timer 4 clock source as EXT */

#define CLK_TMRSEL0_TMR5SEL_PCLK0       (0x0UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as PCLK0 */
#define CLK_TMRSEL0_TMR5SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as LIRC */
#define CLK_TMRSEL0_TMR5SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as LXT */
#define CLK_TMRSEL0_TMR5SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as HXT */
#define CLK_TMRSEL0_TMR5SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as HIRC */
#define CLK_TMRSEL0_TMR5SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR5SEL_Pos)        /*!< Setting Timer 5 clock source as EXT */

#define CLK_TMRSEL0_TMR6SEL_PCLK1       (0x0UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as PCLK1 */
#define CLK_TMRSEL0_TMR6SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as LIRC */
#define CLK_TMRSEL0_TMR6SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as LXT */
#define CLK_TMRSEL0_TMR6SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as HXT */
#define CLK_TMRSEL0_TMR6SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as HIRC */
#define CLK_TMRSEL0_TMR6SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR6SEL_Pos)        /*!< Setting Timer 6 clock source as EXT */

#define CLK_TMRSEL0_TMR7SEL_PCLK1       (0x0UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as PCLK1 */
#define CLK_TMRSEL0_TMR7SEL_LIRC        (0x1UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as LIRC */
#define CLK_TMRSEL0_TMR7SEL_LXT         (0x2UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as LXT */
#define CLK_TMRSEL0_TMR7SEL_HXT         (0x3UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as HXT */
#define CLK_TMRSEL0_TMR7SEL_HIRC        (0x4UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as HIRC */
#define CLK_TMRSEL0_TMR7SEL_EXT         (0x5UL<<CLK_TMRSEL0_TMR7SEL_Pos)        /*!< Setting Timer 7 clock source as EXT */

#define CLK_TMRSEL1_TMR8SEL_PCLK1       (0x0UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as PCLK1 */
#define CLK_TMRSEL1_TMR8SEL_LIRC        (0x1UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as LIRC */
#define CLK_TMRSEL1_TMR8SEL_LXT         (0x2UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as LXT */
#define CLK_TMRSEL1_TMR8SEL_HXT         (0x3UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as HXT */
#define CLK_TMRSEL1_TMR8SEL_HIRC        (0x4UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as HIRC */
#define CLK_TMRSEL1_TMR8SEL_EXT         (0x5UL<<CLK_TMRSEL1_TMR8SEL_Pos)        /*!< Setting Timer 8 clock source as EXT */

#define CLK_UARTSEL_UART0SEL_PCLK0      (0x0UL<<CLK_UARTSEL_UART0SEL_Pos)       /*!< Setting UART 0 clock source as PCLK0 */
#define CLK_UARTSEL_UART0SEL_LIRC       (0x1UL<<CLK_UARTSEL_UART0SEL_Pos)       /*!< Setting UART 0 clock source as LIRC */
#define CLK_UARTSEL_UART0SEL_LXT        (0x2UL<<CLK_UARTSEL_UART0SEL_Pos)       /*!< Setting UART 0 clock source as LXT */
#define CLK_UARTSEL_UART0SEL_HXT        (0x3UL<<CLK_UARTSEL_UART0SEL_Pos)       /*!< Setting UART 0 clock source as HXT */
#define CLK_UARTSEL_UART0SEL_HIRC       (0x4UL<<CLK_UARTSEL_UART0SEL_Pos)       /*!< Setting UART 0 clock source as HIRC */

#define CLK_UARTSEL_UART1SEL_PCLK1      (0x0UL<<CLK_UARTSEL_UART1SEL_Pos)       /*!< Setting UART 1 clock source as PCLK1 */
#define CLK_UARTSEL_UART1SEL_LIRC       (0x1UL<<CLK_UARTSEL_UART1SEL_Pos)       /*!< Setting UART 1 clock source as LIRC */
#define CLK_UARTSEL_UART1SEL_LXT        (0x2UL<<CLK_UARTSEL_UART1SEL_Pos)       /*!< Setting UART 1 clock source as LXT */
#define CLK_UARTSEL_UART1SEL_HXT        (0x3UL<<CLK_UARTSEL_UART1SEL_Pos)       /*!< Setting UART 1 clock source as HXT */
#define CLK_UARTSEL_UART1SEL_HIRC       (0x4UL<<CLK_UARTSEL_UART1SEL_Pos)       /*!< Setting UART 1 clock source as HIRC */

#define CLK_UARTSEL_UART2SEL_PCLK0      (0x0UL<<CLK_UARTSEL_UART2SEL_Pos)       /*!< Setting UART 2 clock source as PCLK0 */
#define CLK_UARTSEL_UART2SEL_LIRC       (0x1UL<<CLK_UARTSEL_UART2SEL_Pos)       /*!< Setting UART 2 clock source as LIRC */
#define CLK_UARTSEL_UART2SEL_LXT        (0x2UL<<CLK_UARTSEL_UART2SEL_Pos)       /*!< Setting UART 2 clock source as LXT */
#define CLK_UARTSEL_UART2SEL_HXT        (0x3UL<<CLK_UARTSEL_UART2SEL_Pos)       /*!< Setting UART 2 clock source as HXT */
#define CLK_UARTSEL_UART2SEL_HIRC       (0x4UL<<CLK_UARTSEL_UART2SEL_Pos)       /*!< Setting UART 2 clock source as HIRC */

#define CLK_UARTSEL_UART3SEL_PCLK1      (0x0UL<<CLK_UARTSEL_UART3SEL_Pos)       /*!< Setting UART 3 clock source as PCLK1 */
#define CLK_UARTSEL_UART3SEL_LIRC       (0x1UL<<CLK_UARTSEL_UART3SEL_Pos)       /*!< Setting UART 3 clock source as LIRC */
#define CLK_UARTSEL_UART3SEL_LXT        (0x2UL<<CLK_UARTSEL_UART3SEL_Pos)       /*!< Setting UART 3 clock source as LXT */
#define CLK_UARTSEL_UART3SEL_HXT        (0x3UL<<CLK_UARTSEL_UART3SEL_Pos)       /*!< Setting UART 3 clock source as HXT */
#define CLK_UARTSEL_UART3SEL_HIRC       (0x4UL<<CLK_UARTSEL_UART3SEL_Pos)       /*!< Setting UART 3 clock source as HIRC */

#define CLK_UARTSEL_UART4SEL_PCLK0      (0x0UL<<CLK_UARTSEL_UART4SEL_Pos)       /*!< Setting UART 4 clock source as PCLK0 */
#define CLK_UARTSEL_UART4SEL_LIRC       (0x1UL<<CLK_UARTSEL_UART4SEL_Pos)       /*!< Setting UART 4 clock source as LIRC */
#define CLK_UARTSEL_UART4SEL_LXT        (0x2UL<<CLK_UARTSEL_UART4SEL_Pos)       /*!< Setting UART 4 clock source as LXT */
#define CLK_UARTSEL_UART4SEL_HXT        (0x3UL<<CLK_UARTSEL_UART4SEL_Pos)       /*!< Setting UART 4 clock source as HXT */
#define CLK_UARTSEL_UART4SEL_HIRC       (0x4UL<<CLK_UARTSEL_UART4SEL_Pos)       /*!< Setting UART 4 clock source as HIRC */

#define CLK_WDTSEL_WDT0SEL_HCLK_DIV2048 (0x0UL<<CLK_WDTSEL_WDT0SEL_Pos)         /*!< Setting WDT clock source as HCLK/2048 */
#define CLK_WDTSEL_WDT0SEL_LXT          (0x1UL<<CLK_WDTSEL_WDT0SEL_Pos)         /*!< Setting WDT clock source as LXT */
#define CLK_WDTSEL_WDT0SEL_LIRC         (0x2UL<<CLK_WDTSEL_WDT0SEL_Pos)         /*!< Setting WDT clock source as LIRC */

#define CLK_WWDTSEL_WWDT0SEL_HCLK_DIV2048   (0x0UL<<CLK_WWDTSEL_WWDT0SEL_Pos)   /*!< Setting WWDT clock source as HCLK/2048 */
#define CLK_WWDTSEL_WWDT0SEL_LIRC           (0x1UL<<CLK_WWDTSEL_WWDT0SEL_Pos)   /*!< Setting WWDT clock source as LIRC */

#define RTC_LXTCTL_RTCCKSEL_LXT         (0x0UL<<RTC_LXTCTL_RTCCKSEL_Pos)        /*!< Setting RTC clock source as LXT */
#define RTC_LXTCTL_RTCCKSEL_LIRC        (0x1UL<<RTC_LXTCTL_RTCCKSEL_Pos)        /*!< Setting RTC clock source as LIRC */


/*---------------------------------------------------------------------------------------------------------*/
/*  Clock divider constant definitions.                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_HCLKDIV_HCLK(x)     (((x)-1UL) << CLK_HCLKDIV_HCLKDIV_Pos)  /*!< HCLKDIV Setting for HCLK clock divider. It could be 1~16 */
#define CLK_ADCDIV_ADC0(x)      (((x)-1UL) << CLK_ADCDIV_ADC0DIV_Pos)   /*!< ADCDIV Setting for ADC0 clock divider. It could be 1~256 */
#define CLK_UARTDIV_UART0(x)    (((x)-1UL) << CLK_UARTDIV_UART0DIV_Pos) /*!< UARTDIV Setting for UART0 clock divider. It could be 1~16 */
#define CLK_UARTDIV_UART1(x)    (((x)-1UL) << CLK_UARTDIV_UART1DIV_Pos) /*!< UARTDIV Setting for UART1 clock divider. It could be 1~16 */
#define CLK_UARTDIV_UART2(x)    (((x)-1UL) << CLK_UARTDIV_UART2DIV_Pos) /*!< UARTDIV Setting for UART2 clock divider. It could be 1~16 */
#define CLK_UARTDIV_UART3(x)    (((x)-1UL) << CLK_UARTDIV_UART3DIV_Pos) /*!< UARTDIV Setting for UART3 clock divider. It could be 1~16 */
#define CLK_UARTDIV_UART4(x)    (((x)-1UL) << CLK_UARTDIV_UART4DIV_Pos) /*!< UARTDIV Setting for UART4 clock divider. It could be 1~16 */


/*---------------------------------------------------------------------------------------------------------*/
/*  PCLKDIV constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PCLKDIV_PCLK0DIV_DIV1   (0x0UL<<CLK_PCLKDIV_PCLK0DIV_Pos)   /*!< PCLKDIV Setting for PCLK0 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_PCLK0DIV_DIV2   (0x1UL<<CLK_PCLKDIV_PCLK0DIV_Pos)   /*!< PCLKDIV Setting for PCLK0 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_PCLK0DIV_DIV4   (0x2UL<<CLK_PCLKDIV_PCLK0DIV_Pos)   /*!< PCLKDIV Setting for PCLK0 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_PCLK0DIV_DIV8   (0x3UL<<CLK_PCLKDIV_PCLK0DIV_Pos)   /*!< PCLKDIV Setting for PCLK0 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_PCLK0DIV_DIV16  (0x4UL<<CLK_PCLKDIV_PCLK0DIV_Pos)   /*!< PCLKDIV Setting for PCLK0 clock divider  16. \hideinitializer */

#define CLK_PCLKDIV_PCLK1DIV_DIV1   (0x0UL<<CLK_PCLKDIV_PCLK1DIV_Pos)   /*!< PCLKDIV Setting for PCLK1 clock divider  1. \hideinitializer */
#define CLK_PCLKDIV_PCLK1DIV_DIV2   (0x1UL<<CLK_PCLKDIV_PCLK1DIV_Pos)   /*!< PCLKDIV Setting for PCLK1 clock divider  2. \hideinitializer */
#define CLK_PCLKDIV_PCLK1DIV_DIV4   (0x2UL<<CLK_PCLKDIV_PCLK1DIV_Pos)   /*!< PCLKDIV Setting for PCLK1 clock divider  4. \hideinitializer */
#define CLK_PCLKDIV_PCLK1DIV_DIV8   (0x3UL<<CLK_PCLKDIV_PCLK1DIV_Pos)   /*!< PCLKDIV Setting for PCLK1 clock divider  8. \hideinitializer */
#define CLK_PCLKDIV_PCLK1DIV_DIV16  (0x4UL<<CLK_PCLKDIV_PCLK1DIV_Pos)   /*!< PCLKDIV Setting for PCLK1 clock divider  16. \hideinitializer */


/*---------------------------------------------------------------------------------------------------------*/
/*  PLLCTL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PLLCTL_STBSEL_CLKSRC_4MHz   (0x0UL<<CLK_PLLCTL_STBSEL_Pos)      /*!< STBSEL Setting for HXT <= 4MHz. \hideinitializer */
#define CLK_PLLCTL_STBSEL_CLKSRC_12MHz  (0x1UL<<CLK_PLLCTL_STBSEL_Pos)      /*!< STBSEL Setting for 4MHz < HXT <= 12MHz. \hideinitializer */
#define CLK_PLLCTL_STBSEL_CLKSRC_24MHz  (0x2UL<<CLK_PLLCTL_STBSEL_Pos)      /*!< STBSEL Setting for 12MHz < HXT <= 244MHz. \hideinitializer */
#define CLK_PLLCTL_STBSEL_CLKSRC        (CLK_PLLCTL_STBSEL_CLKSRC_12MHz)    /*!< STBSEL Setting for default HXT. \hideinitializer */

#define CLK_PLLCTL_NF(x)        (((x)-1UL)<<CLK_PLLCTL_FBDIV_Pos)
#define CLK_PLLCTL_NR(x)        (((x)-1UL)<<CLK_PLLCTL_INDIV_Pos)

#define CLK_PLLCTL_32MHz_HXT    (CLK_PLLCTL_NR(3UL) | CLK_PLLCTL_NF(8UL))   /*!< Predefined PLLCTL setting for 40MHz PLL output with FIN=12MHz */
#define CLK_PLLCTL_40MHz_HXT    (CLK_PLLCTL_NR(3UL) | CLK_PLLCTL_NF(10UL))  /*!< Predefined PLLCTL setting for 40MHz PLL output with FIN=12MHz */


/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_CLKCTL_BASE      (CLK_BASE+0x080UL)  /*!< The address of first register for IP clock enable \hideinitializer */
#define MODULE_CLKSEL_BASE      (CLK_BASE+0x100UL)  /*!< The address of first register for IP clock select \hideinitializer */
#define MODULE_CLKDIV_BASE      (CLK_BASE+0x180UL)  /*!< The address of first register for IP clock divider \hideinitializer */

#define MODULE_CLKCTL(x)        (((x) >> 47) & 0xffUL)  /*!< Calculate CLKCTL offset \hideinitializer */
#define MODULE_CLKSEL(x)        (((x) >> 39) & 0xffUL)  /*!< Calculate CLKSEL offset \hideinitializer */
#define MODULE_CLKDIV(x)        (((x) >> 31) & 0xffUL)  /*!< Calculate CLKDIV offset \hideinitializer */
#define MODULE_CLKSEL_Msk(x)    (((x) >> 23) & 0xffUL)  /*!< Calculate CLKSEL mask offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Msk(x)    (((x) >> 15) & 0xffUL)  /*!< Calculate CLKDIV mask offset on MODULE index \hideinitializer */
#define MODULE_CLKEN_Pos(x)     (((x) >> 10) & 0x1fUL)  /*!< Calculate CLKEN offset on MODULE index \hideinitializer */
#define MODULE_CLKSEL_Pos(x)    (((x) >>  5) & 0x1fUL)  /*!< Calculate CLKSEL position offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Pos(x)    (((x) >>  0) & 0x1fUL)  /*!< Calculate CLKDIV position offset on MODULE index \hideinitializer */

#define MODULE_NoMsk            (0x0ULL)                /*!< Not mask on MODULE index \hideinitializer */
#define NA                      (MODULE_NoMsk)          /*!< Not Available \hideinitializer */

#define MODULE_CLKCTL_ENC(x)        (((x) & 0xffUL) << 47)  /*!< CLKCTL offset in the number of registers \hideinitializer */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0xffUL) << 39)  /*!< CLKSEL offset in the number of registers \hideinitializer */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0xffUL) << 31)  /*!< CLKDIV offset in the number of registers \hideinitializer */
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0xffUL) << 23)  /*!< CLKSEL mask offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xffUL) << 15)  /*!< CLKDIV mask offset on MODULE index \hideinitializer */
#define MODULE_CLKEN_Pos_ENC(x)     (((x) & 0x1fUL) << 10)  /*!< CLKEN offset on MODULE index \hideinitializer */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1fUL) <<  5)  /*!< CLKSEL position offset on MODULE index \hideinitializer */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1fUL) <<  0)  /*!< CLKDIV position offset on MODULE index \hideinitializer */


#define ADC0_MODULE     (MODULE_CLKCTL_ENC( 0ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_ADCCTL_ADC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC( 4ULL)|MODULE_CLKDIV_Msk_ENC(0xFFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_ADCDIV_ADC0DIV_Pos))

#define BPWM0_MODULE    (MODULE_CLKCTL_ENC( 1ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_BPWMCTL_BPWM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 1ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_BPWMSEL_BPWM0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define BPWM1_MODULE    (MODULE_CLKCTL_ENC( 1ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_BPWMCTL_BPWM1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 1ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_BPWMSEL_BPWM1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define CLKO_MODULE     (MODULE_CLKCTL_ENC(   NA)|MODULE_CLKEN_Pos_ENC((uint64_t)NA)|\
                         MODULE_CLKSEL_ENC( 2ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_CLKOSEL_CLKOSEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define CRC0_MODULE     (MODULE_CLKCTL_ENC( 2ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_CRCCTL_CRC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define FMC0_MODULE     (MODULE_CLKCTL_ENC( 3ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_FMCCTL_FMC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define DFMC0_MODULE    (MODULE_CLKCTL_ENC( 3ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_FMCCTL_DFMC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define ISP0_MODULE     (MODULE_CLKCTL_ENC( 3ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_FMCCTL_ISP0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOA_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOACKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOB_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOBCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOC_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOCCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOD_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIODCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOE_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOECKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOF_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOFCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define GPIOG_MODULE    (MODULE_CLKCTL_ENC( 4ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_GPIOCTL_GPIOGCKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define I2C0_MODULE     (MODULE_CLKCTL_ENC( 5ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define I2C1_MODULE     (MODULE_CLKCTL_ENC( 5ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define I2C2_MODULE     (MODULE_CLKCTL_ENC( 5ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_I2CCTL_I2C2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define PDMA0_MODULE    (MODULE_CLKCTL_ENC( 6ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_PDMACTL_PDMA0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define RTC0_MODULE     (MODULE_CLKCTL_ENC( 7ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_RTCCTL_RTC0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define SRAM0_MODULE    (MODULE_CLKCTL_ENC( 8ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define SRAM1_MODULE    (MODULE_CLKCTL_ENC( 8ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_SRAMCTL_SRAM1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define ST0_MODULE      (MODULE_CLKCTL_ENC( 9ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_STCTL_ST0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR0_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR1_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR1SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR2_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR2SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR3_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR3SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR4_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR4CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR4SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR5_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR5CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR5SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR6_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR6CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR6SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR7_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR7CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 3ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL0_TMR7SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define TMR8_MODULE     (MODULE_CLKCTL_ENC(10ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_TMRCTL_TMR8CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 4ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_TMRSEL1_TMR8SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define UART0_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL_UART0SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV_UART0DIV_Pos))

#define UART1_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL_UART1SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV_UART1DIV_Pos))

#define UART2_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL_UART2SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV_UART2DIV_Pos))

#define UART3_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL_UART3SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV_UART3DIV_Pos))

#define UART4_MODULE    (MODULE_CLKCTL_ENC(11ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_UARTCTL_UART4CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 5ULL)|MODULE_CLKSEL_Msk_ENC( 0x7ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_UARTSEL_UART4SEL_Pos)|\
                         MODULE_CLKDIV_ENC( 5ULL)|MODULE_CLKDIV_Msk_ENC( 0xFULL)|MODULE_CLKDIV_Pos_ENC((uint64_t)CLK_UARTDIV_UART4DIV_Pos))

#define USCI0_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define USCI1_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI1CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define USCI2_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI2CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define USCI3_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI3CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define USCI4_MODULE    (MODULE_CLKCTL_ENC(12ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_USCICTL_USCI4CKEN_Pos)|\
                         MODULE_CLKSEL_ENC(   NA)|MODULE_CLKSEL_Msk_ENC(     NA)|MODULE_CLKSEL_Pos_ENC(NA)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define WDT0_MODULE     (MODULE_CLKCTL_ENC(13ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WDTCTL_WDT0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 6ULL)|MODULE_CLKSEL_Msk_ENC( 0x3ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WDTSEL_WDT0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define WWDT0_MODULE    (MODULE_CLKCTL_ENC(14ULL)|MODULE_CLKEN_Pos_ENC((uint64_t)CLK_WWDTCTL_WWDT0CKEN_Pos)|\
                         MODULE_CLKSEL_ENC( 7ULL)|MODULE_CLKSEL_Msk_ENC( 0x1ULL)|MODULE_CLKSEL_Pos_ENC((uint64_t)CLK_WWDTSEL_WWDT0SEL_Pos)|\
                         MODULE_CLKDIV_ENC(   NA)|MODULE_CLKDIV_Msk_ENC(     NA)|MODULE_CLKDIV_Pos_ENC(NA))

#define CLK_TIMEOUT_ERR             (-1)    /*!< Clock timeout error value \hideinitializer */

/**@}*/ /* end of group CLK_EXPORTED_CONSTANTS */

extern int32_t g_CLK_i32ErrCode;

/** @addtogroup CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* static inline functions                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare these inline functions here to avoid MISRA C 2004 rule 8.1 error */
__STATIC_INLINE void CLK_SysTickDelay(uint32_t us);
__STATIC_INLINE void CLK_SysTickLongDelay(uint32_t us);


/**
  * @brief      This function execute delay function.
  * @param[in]  us  Delay time. The Max value is (2^24-1) / CPU Clock(MHz). Ex:
  *                             96MHz => 174762us, 84MHz => 199728us,
  *                             64MHz => 262143us, 48MHz => 349525us ...
  * @return     None
  * @details    Use the SysTick to generate the delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
__STATIC_INLINE void CLK_SysTickDelay(uint32_t us)
{
    CLK->STCTL |= CLK_STCTL_ST0CKEN_Msk;

    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL  = (0x0UL);
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL)
    {
    }

    /* Disable SysTick counter */
    SysTick->CTRL = 0UL;

    CLK->STCTL &= ~CLK_STCTL_ST0CKEN_Msk;
}

/**
  * @brief      This function execute long delay function.
  * @param[in]  us  Delay time.
  * @return     None
  * @details    Use the SysTick to generate the long delay time and the UNIT is in us.
  *             The SysTick clock source is from HCLK, i.e the same as system core clock.
  *             User can use SystemCoreClockUpdate() to calculate CyclesPerUs automatically before using this function.
  */
__STATIC_INLINE void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t u32Delay;

    /* It should <= 65536us for each delay loop */
    u32Delay = 65536UL;

    do
    {
        if(us > u32Delay)
        {
            us -= u32Delay;
        }
        else
        {
            u32Delay = us;
            us = 0UL;
        }

        SysTick->LOAD = u32Delay * CyclesPerUs;
        SysTick->VAL  = (0x0UL);
        SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

        /* Waiting for down-count to zero */
        while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0UL);

        /* Disable SysTick counter */
        SysTick->CTRL = 0UL;

    }
    while(us > 0UL);
}


void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint64_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint64_t u32ModuleIdx);
void CLK_DisableModuleClock(uint64_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_GetModuleClockSource(uint64_t u32ModuleIdx);
uint32_t CLK_GetModuleClockDivider(uint64_t u32ModuleIdx);

/**@}*/ /* end of group CLK_EXPORTED_FUNCTIONS */

/**@}*/ /* end of group CLK_Driver */

/**@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif /* __CLK_H__ */

/**************************************************************************//**
 * @file     gpio.h
 * @version  V1.00
 * @brief    M2U51 series GPIO driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup GPIO_EXPORTED_CONSTANTS GPIO Exported Constants
  @{
*/

#define GPIO_PIN_MAX            16UL    /*!< Specify Maximum Pins of Each GPIO Port */

/*--------------------------------------------------------------------------*/
/*  GPIO_MODE Constant Definitions                                          */
/*--------------------------------------------------------------------------*/
#define GPIO_MODE_INPUT         0x0UL   /*!< Input Mode \hideinitializer */
#define GPIO_MODE_OUTPUT        0x1UL   /*!< Output Mode \hideinitializer */
#define GPIO_MODE_OPEN_DRAIN    0x2UL   /*!< Open-Drain Mode \hideinitializer */
#define GPIO_MODE_QUASI         0x3UL   /*!< Quasi-bidirectional Mode \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO_INTTYPE Constant Definitions                                       */
/*--------------------------------------------------------------------------*/
#define GPIO_INTTYPE_EDGE       0UL     /*!< GPIO_INTTYPE Setting for Edge Trigger Mode \hideinitializer */
#define GPIO_INTTYPE_LEVEL      1UL     /*!< GPIO_INTTYPE Setting for Edge Level Mode \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO Interrupt Type Constant Definitions                                */
/*--------------------------------------------------------------------------*/
#define GPIO_INT_RISING         ((GPIO_INTTYPE_EDGE <<24)|(GPIO_INTEN_RHIEN0_Msk))  /*!< Interrupt enable by Rising Edge \hideinitializer */
#define GPIO_INT_FALLING        ((GPIO_INTTYPE_EDGE <<24)|(GPIO_INTEN_FLIEN0_Msk))  /*!< Interrupt enable by Falling Edge \hideinitializer */
#define GPIO_INT_BOTH_EDGE      ((GPIO_INTTYPE_EDGE <<24)|(GPIO_INTEN_RHIEN0_Msk|GPIO_INTEN_FLIEN0_Msk))    /*!< Interrupt enable by both Rising Edge and Falling Edge \hideinitializer */
#define GPIO_INT_HIGH           ((GPIO_INTTYPE_LEVEL<<24)|(GPIO_INTEN_RHIEN0_Msk))  /*!< Interrupt enable by Level-High \hideinitializer */
#define GPIO_INT_LOW            ((GPIO_INTTYPE_LEVEL<<24)|(GPIO_INTEN_FLIEN0_Msk))  /*!< Interrupt enable by Level-Low \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO Slew Rate Type Constant Definitions                                */
/*--------------------------------------------------------------------------*/
#define GPIO_SLEWCTL_NORMAL     0x0UL   /*!< GPIO slew setting for normal Mode \hideinitializer */
#define GPIO_SLEWCTL_HIGH       0x1UL   /*!< GPIO slew setting for high Mode \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO Pull-up And Pull-down Type Constant Definitions                    */
/*--------------------------------------------------------------------------*/
#define GPIO_PUSEL_DISABLE      0x0UL   /*!< GPIO PUSEL setting for Disable Mode \hideinitializer */
#define GPIO_PUSEL_PULL_UP      0x1UL   /*!< GPIO PUSEL setting for Pull-up Mode \hideinitializer */
#define GPIO_PUSEL_PULL_DOWN    0x2UL   /*!< GPIO PUSEL setting for Pull-down Mode \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO_DBCTL Constant Definitions                                         */
/*--------------------------------------------------------------------------*/
#define GPIO_DBCTL_ICLK_ON          GPIO_DBCTL_ICLKON_Msk   /*!< GPIO_DBCTL setting for all IO pins edge detection circuit is always active after reset \hideinitializer */
#define GPIO_DBCTL_ICLK_OFF         0x00000000UL            /*!< GPIO_DBCTL setting for edge detection circuit is active only if IO pin corresponding GPIOx_IEN bit is set to 1 \hideinitializer */

#define GPIO_DBCTL_DBCLKSRC_LIRC    GPIO_DBCTL_DBCLKSRC_Msk /*!< GPIO_DBCTL setting for de-bounce counter clock source is the internal 10 kHz \hideinitializer */
#define GPIO_DBCTL_DBCLKSRC_HCLK    0x00000000UL            /*!< GPIO_DBCTL setting for de-bounce counter clock source is the HCLK \hideinitializer */

#define GPIO_DBCTL_DBCLKSEL_1       0x0UL   /*!< GPIO_DBCTL setting for sampling cycle = 1 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_2       0x1UL   /*!< GPIO_DBCTL setting for sampling cycle = 2 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_4       0x2UL   /*!< GPIO_DBCTL setting for sampling cycle = 4 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_8       0x3UL   /*!< GPIO_DBCTL setting for sampling cycle = 8 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_16      0x4UL   /*!< GPIO_DBCTL setting for sampling cycle = 16 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_32      0x5UL   /*!< GPIO_DBCTL setting for sampling cycle = 32 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_64      0x6UL   /*!< GPIO_DBCTL setting for sampling cycle = 64 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_128     0x7UL   /*!< GPIO_DBCTL setting for sampling cycle = 128 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_256     0x8UL   /*!< GPIO_DBCTL setting for sampling cycle = 256 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_512     0x9UL   /*!< GPIO_DBCTL setting for sampling cycle = 512 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_1024    0xAUL   /*!< GPIO_DBCTL setting for sampling cycle = 1024 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_2048    0xBUL   /*!< GPIO_DBCTL setting for sampling cycle = 2048 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_4096    0xCUL   /*!< GPIO_DBCTL setting for sampling cycle = 4096 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_8192    0xDUL   /*!< GPIO_DBCTL setting for sampling cycle = 8192 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_16384   0xEUL   /*!< GPIO_DBCTL setting for sampling cycle = 16384 clocks \hideinitializer */
#define GPIO_DBCTL_DBCLKSEL_32768   0xFUL   /*!< GPIO_DBCTL setting for sampling cycle = 32768 clocks \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO EINT Edge Detect Control Type Constant Definitions                 */
/*--------------------------------------------------------------------------*/
#define GPIO_INT_EDETCTL_DISABLE    0x0UL   /*!< GPIO INT_EDETCTL setting for No edge detect \hideinitializer */
#define GPIO_INT_EDETCTL_RISING     0x1UL   /*!< GPIO INT_EDETCTL setting for Rising edge detect \hideinitializer */
#define GPIO_INT_EDETCTL_FALLING    0x2UL   /*!< GPIO INT_EDETCTL setting for Falling edge detect \hideinitializer */
#define GPIO_INT_EDETCTL_BOTH_EDGE  0x3UL   /*!< GPIO INT_EDETCTL setting for Both edge detect \hideinitializer */

#define GPIO_INT_NFSEL_HCLK_DIV_1   0x0UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_2   0x1UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/2 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_4   0x2UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/4 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_8   0x3UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/8 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_16  0x4UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/16 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_32  0x5UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/32 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_64  0x6UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/64 \hideinitializer */
#define GPIO_INT_NFSEL_HCLK_DIV_128 0x7UL   /*!< GPIO INT_INNF setting for de-bounce counter clock source is the HCLK/128 \hideinitializer */

/*--------------------------------------------------------------------------*/
/*  GPIO Auto Operation Control Type Constant Definitions                   */
/*--------------------------------------------------------------------------*/
#define GPIO_AUTO_ALL               (0xFFUL)                    /*!< GPIO Auto Operation mode setting for all GPIO ports \hideinitializer */
#define GPIO_AUTO_PA                (GPIO_AUTOCTL_PAAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PA port \hideinitializer */
#define GPIO_AUTO_PB                (GPIO_AUTOCTL_PBAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PB port \hideinitializer */
#define GPIO_AUTO_PC                (GPIO_AUTOCTL_PCAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PC port \hideinitializer */
#define GPIO_AUTO_PD                (GPIO_AUTOCTL_PDAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PD port \hideinitializer */
#define GPIO_AUTO_PE                (GPIO_AUTOCTL_PEAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PE port \hideinitializer */
#define GPIO_AUTO_PF                (GPIO_AUTOCTL_PFAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PF port \hideinitializer */
#define GPIO_AUTO_PG                (GPIO_AUTOCTL_PGAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PG port \hideinitializer */
#define GPIO_AUTO_PH                (GPIO_AUTOCTL_PHAUTOEN_Msk) /*!< GPIO Auto Operation mode setting for all PH port \hideinitializer */


/* Define GPIO Pin Data Input/Output. It could be used to control each I/O pin by pin address mapping.
   Example 1:

       PA0 = 1;

   It is used to set GPIO PA.0 to high;

   Example 2:

       if (PA0)
           PA0 = 0;

   If GPIO PA.0 pin status is high, then set GPIO PA.0 data output to low.
 */
#define GPIO_PIN_DATA(port, pin)    (*((volatile uint32_t *)((GPIO_PIN_DATA_BASE+(0x40*(port))) + ((pin)<<2)))) /*!< Pin Data Input/Output \hideinitializer */
#define PA0             GPIO_PIN_DATA(0, 0 ) /*!< Specify PA.0 Pin Data Input/Output \hideinitializer */
#define PA1             GPIO_PIN_DATA(0, 1 ) /*!< Specify PA.1 Pin Data Input/Output \hideinitializer */
#define PA2             GPIO_PIN_DATA(0, 2 ) /*!< Specify PA.2 Pin Data Input/Output \hideinitializer */
#define PA3             GPIO_PIN_DATA(0, 3 ) /*!< Specify PA.3 Pin Data Input/Output \hideinitializer */
#define PA4             GPIO_PIN_DATA(0, 4 ) /*!< Specify PA.4 Pin Data Input/Output \hideinitializer */
#define PA5             GPIO_PIN_DATA(0, 5 ) /*!< Specify PA.5 Pin Data Input/Output \hideinitializer */
#define PA6             GPIO_PIN_DATA(0, 6 ) /*!< Specify PA.6 Pin Data Input/Output \hideinitializer */
#define PA7             GPIO_PIN_DATA(0, 7 ) /*!< Specify PA.7 Pin Data Input/Output \hideinitializer */
#define PA8             GPIO_PIN_DATA(0, 8 ) /*!< Specify PA.8 Pin Data Input/Output \hideinitializer */
#define PA9             GPIO_PIN_DATA(0, 9 ) /*!< Specify PA.9 Pin Data Input/Output \hideinitializer */
#define PA10            GPIO_PIN_DATA(0, 10) /*!< Specify PA.10 Pin Data Input/Output \hideinitializer */
#define PA11            GPIO_PIN_DATA(0, 11) /*!< Specify PA.11 Pin Data Input/Output \hideinitializer */
#define PA12            GPIO_PIN_DATA(0, 12) /*!< Specify PA.12 Pin Data Input/Output \hideinitializer */
#define PA13            GPIO_PIN_DATA(0, 13) /*!< Specify PA.13 Pin Data Input/Output \hideinitializer */
#define PA14            GPIO_PIN_DATA(0, 14) /*!< Specify PA.14 Pin Data Input/Output \hideinitializer */
#define PA15            GPIO_PIN_DATA(0, 15) /*!< Specify PA.15 Pin Data Input/Output \hideinitializer */
#define PB0             GPIO_PIN_DATA(1, 0 ) /*!< Specify PB.0 Pin Data Input/Output \hideinitializer */
#define PB1             GPIO_PIN_DATA(1, 1 ) /*!< Specify PB.1 Pin Data Input/Output \hideinitializer */
#define PB2             GPIO_PIN_DATA(1, 2 ) /*!< Specify PB.2 Pin Data Input/Output \hideinitializer */
#define PB3             GPIO_PIN_DATA(1, 3 ) /*!< Specify PB.3 Pin Data Input/Output \hideinitializer */
#define PB4             GPIO_PIN_DATA(1, 4 ) /*!< Specify PB.4 Pin Data Input/Output \hideinitializer */
#define PB5             GPIO_PIN_DATA(1, 5 ) /*!< Specify PB.5 Pin Data Input/Output \hideinitializer */
#define PB6             GPIO_PIN_DATA(1, 6 ) /*!< Specify PB.6 Pin Data Input/Output \hideinitializer */
#define PB7             GPIO_PIN_DATA(1, 7 ) /*!< Specify PB.7 Pin Data Input/Output \hideinitializer */
#define PB8             GPIO_PIN_DATA(1, 8 ) /*!< Specify PB.8 Pin Data Input/Output \hideinitializer */
#define PB9             GPIO_PIN_DATA(1, 9 ) /*!< Specify PB.9 Pin Data Input/Output \hideinitializer */
#define PB10            GPIO_PIN_DATA(1, 10) /*!< Specify PB.10 Pin Data Input/Output \hideinitializer */
#define PB11            GPIO_PIN_DATA(1, 11) /*!< Specify PB.11 Pin Data Input/Output \hideinitializer */
#define PB12            GPIO_PIN_DATA(1, 12) /*!< Specify PB.12 Pin Data Input/Output \hideinitializer */
#define PB13            GPIO_PIN_DATA(1, 13) /*!< Specify PB.13 Pin Data Input/Output \hideinitializer */
#define PB14            GPIO_PIN_DATA(1, 14) /*!< Specify PB.14 Pin Data Input/Output \hideinitializer */
#define PB15            GPIO_PIN_DATA(1, 15) /*!< Specify PB.15 Pin Data Input/Output \hideinitializer */
#define PC0             GPIO_PIN_DATA(2, 0 ) /*!< Specify PC.0 Pin Data Input/Output \hideinitializer */
#define PC1             GPIO_PIN_DATA(2, 1 ) /*!< Specify PC.1 Pin Data Input/Output \hideinitializer */
#define PC2             GPIO_PIN_DATA(2, 2 ) /*!< Specify PC.2 Pin Data Input/Output \hideinitializer */
#define PC3             GPIO_PIN_DATA(2, 3 ) /*!< Specify PC.3 Pin Data Input/Output \hideinitializer */
#define PC4             GPIO_PIN_DATA(2, 4 ) /*!< Specify PC.4 Pin Data Input/Output \hideinitializer */
#define PC5             GPIO_PIN_DATA(2, 5 ) /*!< Specify PC.5 Pin Data Input/Output \hideinitializer */
#define PC6             GPIO_PIN_DATA(2, 6 ) /*!< Specify PC.6 Pin Data Input/Output \hideinitializer */
#define PC7             GPIO_PIN_DATA(2, 7 ) /*!< Specify PC.7 Pin Data Input/Output \hideinitializer */
#define PC8             GPIO_PIN_DATA(2, 8 ) /*!< Specify PC.8 Pin Data Input/Output \hideinitializer */
#define PC9             GPIO_PIN_DATA(2, 9 ) /*!< Specify PC.9 Pin Data Input/Output \hideinitializer */
#define PC10            GPIO_PIN_DATA(2, 10) /*!< Specify PC.10 Pin Data Input/Output \hideinitializer */
#define PC11            GPIO_PIN_DATA(2, 11) /*!< Specify PC.11 Pin Data Input/Output \hideinitializer */
#define PC12            GPIO_PIN_DATA(2, 12) /*!< Specify PC.12 Pin Data Input/Output \hideinitializer */
#define PC14            GPIO_PIN_DATA(2, 14) /*!< Specify PC.14 Pin Data Input/Output \hideinitializer */
#define PD0             GPIO_PIN_DATA(3, 0 ) /*!< Specify PD.0 Pin Data Input/Output \hideinitializer */
#define PD1             GPIO_PIN_DATA(3, 1 ) /*!< Specify PD.1 Pin Data Input/Output \hideinitializer */
#define PD2             GPIO_PIN_DATA(3, 2 ) /*!< Specify PD.2 Pin Data Input/Output \hideinitializer */
#define PD3             GPIO_PIN_DATA(3, 3 ) /*!< Specify PD.3 Pin Data Input/Output \hideinitializer */
#define PD4             GPIO_PIN_DATA(3, 4 ) /*!< Specify PD.4 Pin Data Input/Output \hideinitializer */
#define PD5             GPIO_PIN_DATA(3, 5 ) /*!< Specify PD.5 Pin Data Input/Output \hideinitializer */
#define PD6             GPIO_PIN_DATA(3, 6 ) /*!< Specify PD.6 Pin Data Input/Output \hideinitializer */
#define PD7             GPIO_PIN_DATA(3, 7 ) /*!< Specify PD.7 Pin Data Input/Output \hideinitializer */
#define PD8             GPIO_PIN_DATA(3, 8 ) /*!< Specify PD.8 Pin Data Input/Output \hideinitializer */
#define PD9             GPIO_PIN_DATA(3, 9 ) /*!< Specify PD.9 Pin Data Input/Output \hideinitializer */
#define PD10            GPIO_PIN_DATA(3, 10) /*!< Specify PD.10 Pin Data Input/Output \hideinitializer */
#define PD11            GPIO_PIN_DATA(3, 11) /*!< Specify PD.11 Pin Data Input/Output \hideinitializer */
#define PD12            GPIO_PIN_DATA(3, 12) /*!< Specify PD.12 Pin Data Input/Output \hideinitializer */
#define PD13            GPIO_PIN_DATA(3, 13) /*!< Specify PD.13 Pin Data Input/Output \hideinitializer */
#define PD15            GPIO_PIN_DATA(3, 15) /*!< Specify PD.15 Pin Data Input/Output \hideinitializer */
#define PE0             GPIO_PIN_DATA(4, 0 ) /*!< Specify PE.0 Pin Data Input/Output \hideinitializer */
#define PE1             GPIO_PIN_DATA(4, 1 ) /*!< Specify PE.1 Pin Data Input/Output \hideinitializer */
#define PE2             GPIO_PIN_DATA(4, 2 ) /*!< Specify PE.2 Pin Data Input/Output \hideinitializer */
#define PE3             GPIO_PIN_DATA(4, 3 ) /*!< Specify PE.3 Pin Data Input/Output \hideinitializer */
#define PE4             GPIO_PIN_DATA(4, 4 ) /*!< Specify PE.4 Pin Data Input/Output \hideinitializer */
#define PE5             GPIO_PIN_DATA(4, 5 ) /*!< Specify PE.5 Pin Data Input/Output \hideinitializer */
#define PE6             GPIO_PIN_DATA(4, 6 ) /*!< Specify PE.6 Pin Data Input/Output \hideinitializer */
#define PE7             GPIO_PIN_DATA(4, 7 ) /*!< Specify PE.7 Pin Data Input/Output \hideinitializer */
#define PE8             GPIO_PIN_DATA(4, 8 ) /*!< Specify PE.8 Pin Data Input/Output \hideinitializer */
#define PE9             GPIO_PIN_DATA(4, 9 ) /*!< Specify PE.9 Pin Data Input/Output \hideinitializer */
#define PE10            GPIO_PIN_DATA(4, 10) /*!< Specify PE.10 Pin Data Input/Output \hideinitializer */
#define PE11            GPIO_PIN_DATA(4, 11) /*!< Specify PE.11 Pin Data Input/Output \hideinitializer */
#define PE12            GPIO_PIN_DATA(4, 12) /*!< Specify PE.12 Pin Data Input/Output \hideinitializer */
#define PE13            GPIO_PIN_DATA(4, 13) /*!< Specify PE.13 Pin Data Input/Output \hideinitializer */
#define PE14            GPIO_PIN_DATA(4, 14) /*!< Specify PE.14 Pin Data Input/Output \hideinitializer */
#define PE15            GPIO_PIN_DATA(4, 15) /*!< Specify PE.15 Pin Data Input/Output \hideinitializer */
#define PF0             GPIO_PIN_DATA(5, 0 ) /*!< Specify PF.0 Pin Data Input/Output \hideinitializer */
#define PF1             GPIO_PIN_DATA(5, 1 ) /*!< Specify PF.1 Pin Data Input/Output \hideinitializer */
#define PF2             GPIO_PIN_DATA(5, 2 ) /*!< Specify PF.2 Pin Data Input/Output \hideinitializer */
#define PF3             GPIO_PIN_DATA(5, 3 ) /*!< Specify PF.3 Pin Data Input/Output \hideinitializer */
#define PF4             GPIO_PIN_DATA(5, 4 ) /*!< Specify PF.4 Pin Data Input/Output \hideinitializer */
#define PF5             GPIO_PIN_DATA(5, 5 ) /*!< Specify PF.5 Pin Data Input/Output \hideinitializer */
#define PF6             GPIO_PIN_DATA(5, 6 ) /*!< Specify PF.6 Pin Data Input/Output \hideinitializer */
#define PF7             GPIO_PIN_DATA(5, 7 ) /*!< Specify PF.7 Pin Data Input/Output \hideinitializer */
#define PF8             GPIO_PIN_DATA(5, 8 ) /*!< Specify PF.8 Pin Data Input/Output \hideinitializer */
#define PF9             GPIO_PIN_DATA(5, 9 ) /*!< Specify PF.9 Pin Data Input/Output \hideinitializer */
#define PF14            GPIO_PIN_DATA(5, 14) /*!< Specify PF.14 Pin Data Input/Output \hideinitializer */
#define PG2             GPIO_PIN_DATA(6, 2 ) /*!< Specify PG.2 Pin Data Input/Output \hideinitializer */
#define PG3             GPIO_PIN_DATA(6, 3 ) /*!< Specify PG.3 Pin Data Input/Output \hideinitializer */
#define PG4             GPIO_PIN_DATA(6, 4 ) /*!< Specify PG.4 Pin Data Input/Output \hideinitializer */
#define PH8             GPIO_PIN_DATA(7, 8 ) /*!< Specify PH.8 Pin Data Input/Output \hideinitializer */
#define PH9             GPIO_PIN_DATA(7, 9 ) /*!< Specify PH.9 Pin Data Input/Output \hideinitializer */

/*@}*/ /* end of group GPIO_EXPORTED_CONSTANTS */

/** @addtogroup GPIO_EXPORTED_MACROS GPIO Exported Macros
  @{
*/

/**
 * @brief       Clear GPIO Pin Interrupt Flag
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Clear the interrupt status of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_CLR_INT_FLAG(port, u32PinMask)         ((port)->INTSRC = (u32PinMask))

/**
 * @brief       Disable Pin De-bounce Function
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Disable the interrupt de-bounce function of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_DISABLE_DEBOUNCE(port, u32PinMask)     ((port)->DBEN &= ~(u32PinMask))

/**
 * @brief       Enable Pin De-bounce Function
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Enable the interrupt de-bounce function of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_ENABLE_DEBOUNCE(port, u32PinMask)      ((port)->DBEN |= (u32PinMask))

/**
 * @brief       Disable I/O Digital Input Path
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O digital input path of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_DISABLE_DIGITAL_PATH(port, u32PinMask) ((port)->DINOFF |= ((u32PinMask)<<GPIO_DINOFF_DINOFF0_Pos))

/**
 * @brief       Enable I/O Digital Input Path
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O digital input path of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_ENABLE_DIGITAL_PATH(port, u32PinMask)  ((port)->DINOFF &= ~((u32PinMask)<<GPIO_DINOFF_DINOFF0_Pos))

/**
 * @brief       Disable I/O DOUT mask
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Disable I/O DOUT mask of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_DISABLE_DOUT_MASK(port, u32PinMask)    ((port)->DATMSK &= ~(u32PinMask))

/**
 * @brief       Enable I/O DOUT mask
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @return      None
 *
 * @details     Enable I/O DOUT mask of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_ENABLE_DOUT_MASK(port, u32PinMask) ((port)->DATMSK |= (u32PinMask))

/**
 * @brief       Get GPIO Pin Interrupt Flag
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
 *                          It could be \ref BIT0 ~ \ref BIT15 for PA, PB, PD and PE port.
 *                          It could be \ref BIT0 ~ \ref BIT14 for PC and PF GPIO port.
 *                          It could be \ref BIT2 ~ \ref BIT4  for PG GPIO port.
 *                          It could be \ref BIT8 ~ \ref BIT9  for PH GPIO port.
 *
 * @retval      0           No interrupt at specified GPIO pin
 * @retval      1           The specified GPIO pin generate an interrupt
 *
 * @details     Get the interrupt status of specified GPIO pin.
 * \hideinitializer
 */
#define GPIO_GET_INT_FLAG(port, u32PinMask)     ((port)->INTSRC & (u32PinMask))

/**
 * @brief       Set De-bounce Sampling Cycle Time
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32ClkSrc   The de-bounce counter clock source. It could be
 *                            - \ref GPIO_DBCTL_DBCLKSRC_HCLK
 *                              \ref GPIO_DBCTL_DBCLKSRC_LIRC
 * @param[in]   u32ClkSel   The de-bounce sampling cycle selection. It could be
 *                            - \ref GPIO_DBCTL_DBCLKSEL_1
 *                            - \ref GPIO_DBCTL_DBCLKSEL_2
 *                            - \ref GPIO_DBCTL_DBCLKSEL_4
 *                            - \ref GPIO_DBCTL_DBCLKSEL_8
 *                            - \ref GPIO_DBCTL_DBCLKSEL_16
 *                            - \ref GPIO_DBCTL_DBCLKSEL_32
 *                            - \ref GPIO_DBCTL_DBCLKSEL_64
 *                            - \ref GPIO_DBCTL_DBCLKSEL_128
 *                            - \ref GPIO_DBCTL_DBCLKSEL_256
 *                            - \ref GPIO_DBCTL_DBCLKSEL_512
 *                            - \ref GPIO_DBCTL_DBCLKSEL_1024
 *                            - \ref GPIO_DBCTL_DBCLKSEL_2048
 *                            - \ref GPIO_DBCTL_DBCLKSEL_4096
 *                            - \ref GPIO_DBCTL_DBCLKSEL_8192
 *                            - \ref GPIO_DBCTL_DBCLKSEL_16384
 *                            - \ref GPIO_DBCTL_DBCLKSEL_32768
 *
 * @return      None
 *
 * @details     Set the interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: _GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_4). \n
 *              It's meaning the De-debounce counter clock source is internal 32 KHz and sampling cycle selection is 4. \n
 *              Then the target de-bounce sampling cycle time is (4)*(1/(32*1000)) s = 125 us,
 *              and system will sampling interrupt input once per 125 us.
 */
#define GPIO_SET_DEBOUNCE_TIME(port, u32ClkSrc, u32ClkSel)    ((port)->DBCTL = (GPIO_DBCTL_ICLKON_Msk | (u32ClkSrc) | (u32ClkSel)))

/**
 * @brief       Get GPIO Port IN Data
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 *
 * @return      The specified port data
 *
 * @details     Get the PIN register of specified GPIO port.
 * \hideinitializer
 */
#define GPIO_GET_IN_DATA(port)  ((port)->PIN)

/**
 * @brief       Set GPIO Port OUT Data
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF, \ref PG or \ref PH.
 * @param[in]   u32Data     GPIO port data.
 *
 * @return      None
 *
 * @details     Set the Data into specified GPIO port.
 * \hideinitializer
 */
#define GPIO_SET_OUT_DATA(port, u32Data)    ((port)->DOUT = (u32Data))

/**
 * @brief       Toggle Specified GPIO pin
 *
 * @param[in]   u32Pin      Pxy. For exampe, \ref PA1.
 *
 * @return      None
 *
 * @details     Toggle the specified GPIO pint.
 * \hideinitializer
 */
#define GPIO_TOGGLE(u32Pin) ((u32Pin) ^= 1)

/**
 * @brief       Clear GPIO External Interrupt Flag
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 7 for EINT0, EINT1, EINT2, EINT3, EINT4, EINT5, EINT6 and EINT7.
 *
 * @return      None
 *
 * @details     Clear the external interrupt status of specified EINT number.
 * \hideinitializer
 */
#define GPIO_CLR_EINT_FLAG(u32EINTn)         (GPIO->INT_EDSTS = (GPIO_INT_EDSTS_EDIF0_Msk << (u32EINTn)))

/**
 * @brief       Get GPIO External Interrupt Flag
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 7 for EINT0, EINT1, EINT2, EINT3, EINT4, EINT5, EINT6 and EINT7.
 *
 * @retval      0           No interrupt at specified EINT number
 * @retval      1           The specified EINT number generate an interrupt
 *
 * @details     Get the external interrupt status of specified EINT number.
 * \hideinitializer
 */
#define GPIO_GET_EINT_FLAG(u32EINTn)     (GPIO->INT_EDSTS & (GPIO_INT_EDSTS_EDIF0_Msk << (u32EINTn)))

/**
 * @brief       Set EINT De-bounce Sampling Cycle Time
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 7 for EINT0, EINT1, EINT2, EINT3, EINT4, EINT5, EINT6 and EINT7.
 * @param[in]   u32ClkSrc   The de-bounce counter clock source. It could be
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_1
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_2
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_4
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_8
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_16
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_32
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_64
 *                            - \ref GPIO_INT_NFSEL_HCLK_DIV_128
 * @param[in]   u32ClkSel   The de-bounce sampling cycle selection. It could be 0 ~ 7
 *
 * @return      None
 *
 * @details     Set the external interrupt de-bounce sampling cycle time based on the debounce counter clock source. \n
 *              Example: GPIO_SET_EINT_DEBOUNCE_TIME(0, GPIO_INT_NFSEL_HCLK_DIV_4, 5). \n
 *              It's meaning the De-debounce counter clock source is HCLK/4 and sampling cycle selection is 5. \n
 *              Then the target de-bounce sampling cycle time is ((1/(HCLK/4))*5) s = 0.5 us if HCLK is 40MHz
 *              System will sampling interrupt input once per 0.5 us.
 */
#define GPIO_SET_EINT_DEBOUNCE_TIME(u32EINTn, u32ClkSrc, u32ClkSel)     (GPIO->INT_INNF[(u32EINTn)] = (((u32ClkSrc)<<GPIO_INT_INNF_NFSEL_Pos) | ((u32ClkSel)<<GPIO_INT_INNF_NFCNT_Pos)))

/**
 * @brief       Disable EINT Pin De-bounce Function
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 7 for EINT0, EINT1, EINT2, EINT3, EINT4, EINT5, EINT6 and EINT7.
 *
 * @return      None
 *
 * @details     Disable the external interrupt de-bounce function of specified EINT number.
 * \hideinitializer
 */
#define GPIO_DISABLE_EINT_DEBOUNCE(u32EINTn)    (GPIO->INT_INNF[(u32EINTn)] &= ~(GPIO_INT_INNF_NFEN_Msk))

/**
 * @brief       Enable EINT Pin De-bounce Function
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 7 for EINT0, EINT1, EINT2, EINT3, EINT4, EINT5, EINT6 and EINT7.
 *
 * @return      None
 *
 * @details     Enable the external interrupt de-bounce function of specified EINT number.
 * \hideinitializer
 */
#define GPIO_ENABLE_EINT_DEBOUNCE(u32EINTn)     (GPIO->INT_INNF[(u32EINTn)] |= (GPIO_INT_INNF_NFEN_Msk))

/*@}*/ /* end of group GPIO_EXPORTED_MACROS */

/** @addtogroup GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/
void GPIO_SetMode    (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt  (GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt (GPIO_T *port, uint32_t u32Pin);
void GPIO_SetSlewCtl (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_SetPullCtl (GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableAuto (uint32_t u32PortMask);
void GPIO_DisableAuto(uint32_t u32PortMask);
void GPIO_EnableEINT (uint32_t u32EINTn, uint32_t u32IntAttribs);
void GPIO_DisableEINT(uint32_t u32EINTn);

/*@}*/ /* end of group GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group GPIO_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __GPIO_H__ */

/*** (C) COPYRIGHT 2024 Nuvoton Technology Corp. ***/

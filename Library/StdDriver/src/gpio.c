/**************************************************************************//**
 * @file     gpio.c
 * @version  V1.00
 * @brief    M2003J series GPIO driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NuMicro.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup GPIO_Driver GPIO Driver
  @{
*/

/** @addtogroup GPIO_EXPORTED_FUNCTIONS GPIO Exported Functions
  @{
*/

/**
  * @brief      Set GPIO operation mode
  *
  * @param[in]  port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF or \ref PG.
  * @param[in]  u32PinMask  The single or multiple pins of specified GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT15 for PA, PB and PE GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT12 and \ref BIT14 for PC GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT13 and \ref BIT15 for PD GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT7, \ref BIT14 and \ref BIT15 for PF GPIO port.
  *                         - It could be \ref BIT10 ~ \ref BIT12 for PG GPIO port.
  * @param[in]  u32Mode     Operation mode.  It could be \n
  *                         - \ref GPIO_MODE_INPUT
  *                         - \ref GPIO_MODE_OUTPUT
  *                         - \ref GPIO_MODE_OPEN_DRAIN
  *                         - \ref GPIO_MODE_QUASI
  *
  * @return     None
  *
  * @details    This function is used to set specified GPIO operation mode.
  */
void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if((u32PinMask & (1ul << i))==(1ul << i))
        {
            port->MODE = (port->MODE & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
  * @brief      Enable GPIO interrupt
  *
  * @param[in]  port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF or \ref PG.
  * @param[in]  u32Pin      The pin of specified GPIO port.
  *                         - It could be 0 ~ 15 for PA, PB and PE port.
  *                         - It could be 0 ~ 12, 14 for PC GPIO port.
  *                         - It could be 0 ~ 13, 15 for PD GPIO port.
  *                         - It could be 0 ~ 7, 14, 15  for PF GPIO port.
  *                         - It could be 10 ~ 12  for PG GPIO port.
  * @param[in]  u32IntAttribs   The interrupt attribute of specified GPIO pin. It could be \n
  *                             - \ref GPIO_INT_RISING
  *                             - \ref GPIO_INT_FALLING
  *                             - \ref GPIO_INT_BOTH_EDGE
  *                             - \ref GPIO_INT_HIGH
  *                             - \ref GPIO_INT_LOW
  *
  * @return     None
  *
  * @details    This function is used to enable specified GPIO pin interrupt.
  */
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs)
{
    port->INTTYPE = (port->INTTYPE&~(1ul<<u32Pin)) | (((u32IntAttribs >> 24) & 0xFFUL) << u32Pin);
    port->INTEN = (port->INTEN&~(0x00010001ul<<u32Pin)) | ((u32IntAttribs & 0xFFFFFFUL) << u32Pin);
}

/**
  * @brief      Disable GPIO interrupt
  *
  * @param[in]  port    GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF or \ref PG.
  * @param[in]  u32Pin  The pin of specified GPIO port.
  *                         - It could be 0 ~ 15 for PA, PB and PE port.
  *                         - It could be 0 ~ 12, 14 for PC GPIO port.
  *                         - It could be 0 ~ 13, 15 for PD GPIO port.
  *                         - It could be 0 ~ 7, 14, 15  for PF GPIO port.
  *                         - It could be 10 ~ 12  for PG GPIO port.
  *
  * @return     None
  *
  * @details    This function is used to disable specified GPIO pin interrupt.
  */
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin)
{
    port->INTTYPE &= ~(1UL << u32Pin);
    port->INTEN &= ~((0x00010001UL) << u32Pin);
}

/**
 * @brief       Set GPIO slew rate control
 *
 * @param[in]   port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF or \ref PG.
 * @param[in]   u32PinMask  The single or multiple pins of specified GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT15 for PA, PB and PE GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT12 and \ref BIT14 for PC GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT13 and \ref BIT15 for PD GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT7, \ref BIT14 and \ref BIT15 for PF GPIO port.
  *                         - It could be \ref BIT10 ~ \ref BIT12 for PG GPIO port.
 * @param[in]   u32Mode     Slew rate mode.
 *                          - \ref GPIO_SLEWCTL_NORMAL
 *                          - \ref GPIO_SLEWCTL_HIGH
 *
 * @return      None
 *
 * @details     This function is used to set specified GPIO operation mode.
 */
void GPIO_SetSlewCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1ul << i))
        {
            port->SLEWCTL = (port->SLEWCTL & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
  * @brief      Set GPIO Pull-up and Pull-down control
  *
  * @param[in]  port        GPIO port. It could be \ref PA, \ref PB, \ref PC, \ref PD, \ref PE, \ref PF or \ref PG.
  * @param[in]  u32PinMask  The single or multiple pins of specified GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT15 for PA, PB and PE GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT12 and \ref BIT14 for PC GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT13 and \ref BIT15 for PD GPIO port.
  *                         - It could be \ref BIT0 ~ \ref BIT7, \ref BIT14 and \ref BIT15 for PF GPIO port.
  *                         - It could be \ref BIT10 ~ \ref BIT12 for PG GPIO port.
  * @param[in]  u32Mode     The pin mode of specified GPIO pin. It could be
  *                         - \ref GPIO_PUSEL_DISABLE
  *                         - \ref GPIO_PUSEL_PULL_UP
  *                         - \ref GPIO_PUSEL_PULL_DOWN
  *
  * @return     None
  *
  * @details    Set the pin mode of specified GPIO pin.
  */
void GPIO_SetPullCtl(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for(i = 0ul; i < GPIO_PIN_MAX; i++)
    {
        if(u32PinMask & (1ul << i))
        {
            port->PUSEL = (port->PUSEL & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

/**
 * @brief       Enable External GPIO interrupt
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 5 for EINT0, EINT1, EINT2, EINT3, EINT4 and EINT5.
 * @param[in]   u32IntAttribs   The interrupt attribute of specified EINT. It could be
 *                            - \ref GPIO_INT_EDETCTL_DISABLE
 *                            - \ref GPIO_INT_EDETCTL_RISING
 *                            - \ref GPIO_INT_EDETCTL_FALLING
 *                            - \ref GPIO_INT_EDETCTL_BOTH_EDGE
 *
 * @details     This function is used to enable specified EINT interrupt.
 * \hideinitializer
 */
void GPIO_EnableEINT(uint32_t u32EINTn, uint32_t u32IntAttribs)
{
    switch (u32EINTn)
    {
    case 0:
        GPIO->INT0_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT0_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    case 1:
        GPIO->INT1_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT1_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    case 2:
        GPIO->INT2_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT2_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    case 3:
        GPIO->INT3_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT3_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    case 4:
        GPIO->INT4_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT4_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    case 5:
        GPIO->INT5_EDETCTL = (u32IntAttribs & GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT5_EDINTEN |= GPIO_INT_EDINTEN_EDIEN_Msk;
        break;

    default:
        break;
    }
}

/**
 * @brief       Disable External GPIO interrupt
 *
 * @param[in]   u32EINTn    The specified EINT.
 *                          It could be 0 ~ 5 for EINT0, EINT1, EINT2, EINT3, EINT4 and EINT5.
 *
 * @details     This function is used to disable specified EINT interrupt.
 * \hideinitializer
 */
void GPIO_DisableEINT(uint32_t u32EINTn)
{
    switch (u32EINTn)
    {
    case 0:
        GPIO->INT0_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT0_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    case 1:
        GPIO->INT1_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT1_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    case 2:
        GPIO->INT2_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT2_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    case 3:
        GPIO->INT3_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT3_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    case 4:
        GPIO->INT4_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT4_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    case 5:
        GPIO->INT5_EDETCTL &= ~(GPIO_INT_EDETCTL_EDETCTL_Msk);
        GPIO->INT5_EDINTEN &= ~(GPIO_INT_EDINTEN_EDIEN_Msk);
        break;

    default:
        break;
    }
}

/*@}*/ /* end of group GPIO_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group GPIO_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

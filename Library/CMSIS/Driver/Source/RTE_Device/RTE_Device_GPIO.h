/**************************************************************************//**
 * @file     RTE_Device_GPIO.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M3351 GPIO
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __RTE_DEVICE_GPIO_H__
#define __RTE_DEVICE_GPIO_H__

#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_GPIO.h"

// Pin mapping, Ex: PB12 maps to GPIO_PORTB(12)
#define GPIO_PORTA(n)   (  0U + (n))
#define GPIO_PORTB(n)   ( 16U + (n))
#define GPIO_PORTC(n)   ( 32U + (n))
#define GPIO_PORTD(n)   ( 48U + (n))
#define GPIO_PORTE(n)   ( 64U + (n))
#define GPIO_PORTF(n)   ( 80U + (n))
#define GPIO_PORTG(n)   ( 96U + (n))
#define GPIO_PORTH(n)   (112U + (n))


// GPIO Driver access structure
extern ARM_DRIVER_GPIO Driver_GPIO0;

#ifdef  __cplusplus
}
#endif

#endif /* __RTE_DEVICE_GPIO_H__ */

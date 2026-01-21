/**************************************************************************//**
 * @file     RTE_Device_GPIO_LPGPIO.h
 * @version  V1.00
 * @brief    RTE Device Configuration for Nuvoton M5531 LPGPIO
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_GPIO_LPGPIO_H__
#define __RTE_DEVICE_GPIO_LPGPIO_H__


#ifdef  __cplusplus
extern "C"
{
#endif

#include "Driver_GPIO.h"


// GPIO Driver access structure
extern ARM_DRIVER_GPIO Driver_GPIO1;

#ifdef  __cplusplus
}
#endif

// <e0> GPIO (General-purpose Input/Output Interface 1) [Driver_GPIO1]
// <i> Configuration settings for Driver_I2C0 in component ::CMSIS Driver:I2C
//   <o1> LPIO0 Pin
//     <0=> PA0 <1=> PE0
//   <o2> LPIO1 Pin
//     <0=> PA1 <1=> PE1
//   <o3> LPIO2 Pin
//     <0=> PB0 <1=> PF0
//   <o4> LPIO3 Pin
//     <0=> PB1 <1=> PF1
//   <o5> LPIO4 Pin
//     <0=> PA6 <1=> PC0
//   <o6> LPIO5 Pin
//     <0=> PA7 <1=> PC1
//   <o7> LPIO6 Pin
//     <0=> PB2 <1=> PD0
//   <o8> LPIO7 Pin
//     <0=> PB3 <1=> PD1
// </e>

#define RTE_LPIO  1
#define RTE_LPIO_PIN0_ID  0
#define RTE_LPIO_PIN1_ID  0
#define RTE_LPIO_PIN2_ID  0
#define RTE_LPIO_PIN3_ID  0
#define RTE_LPIO_PIN4_ID  0
#define RTE_LPIO_PIN5_ID  0
#define RTE_LPIO_PIN6_ID  0
#define RTE_LPIO_PIN7_ID  0

#if (RTE_LPIO_PIN0_ID == 0)
    #define SET_LPIO0_PIN           SET_LPIO0_PA0
#elif (RTE_LPIO_PIN0_ID == 1)
    #define SET_LPIO0_PIN           SET_LPIO0_PE0
#else
    #error "Invalid LPIO Pin0 Configuration!"
#endif

#if (RTE_LPIO_PIN1_ID == 0)
    #define SET_LPIO1_PIN           SET_LPIO1_PA1
#elif (RTE_LPIO_PIN1_ID == 1)
    #define SET_LPIO1_PIN           SET_LPIO1_PE1
#else
    #error "Invalid LPIO Pin1 Configuration!"
#endif

#if (RTE_LPIO_PIN2_ID == 0)
    #define SET_LPIO2_PIN           SET_LPIO2_PB0
#elif (RTE_LPIO_PIN2_ID == 1)
    #define SET_LPIO2_PIN           SET_LPIO2_PF0
#else
    #error "Invalid LPIO Pin2 Configuration!"
#endif

#if (RTE_LPIO_PIN3_ID == 0)
    #define SET_LPIO3_PIN           SET_LPIO3_PB1
#elif (RTE_LPIO_PIN3_ID == 1)
    #define SET_LPIO3_PIN           SET_LPIO3_PF1
#else
    #error "Invalid LPIO Pin3 Configuration!"
#endif

#if (RTE_LPIO_PIN4_ID == 0)
    #define SET_LPIO4_PIN           SET_LPIO4_PA6
#elif (RTE_LPIO_PIN4_ID == 1)
    #define SET_LPIO4_PIN           SET_LPIO4_PC0
#else
    #error "Invalid LPIO Pin4 Configuration!"
#endif


#if (RTE_LPIO_PIN5_ID == 0)
    #define SET_LPIO5_PIN           SET_LPIO5_PA7
#elif (RTE_LPIO_PIN5_ID == 1)
    #define SET_LPIO5_PIN           SET_LPIO5_PC1
#else
    #error "Invalid LPIO Pin5 Configuration!"
#endif

#if (RTE_LPIO_PIN6_ID == 0)
    #define SET_LPIO6_PIN           SET_LPIO6_PB2
#elif (RTE_LPIO_PIN6_ID == 1)
    #define SET_LPIO6_PIN           SET_LPIO6_PD0
#else
    #error "Invalid LPIO Pin6 Configuration!"
#endif

#if (RTE_LPIO_PIN7_ID == 0)
    #define SET_LPIO7_PIN           SET_LPIO7_PB3
#elif (RTE_LPIO_PIN7_ID == 1)
    #define SET_LPIO7_PIN           SET_LPIO7_PD1
#else
    #error "Invalid LPIO Pin7 Configuration!"
#endif


#endif // #ifndef __RTE_DEVICE_GPIO_LPGPIO_H__

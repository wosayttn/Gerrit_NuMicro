
/**************************************************************************//**
* @file     RTE_Device_USART.h
* @version  V1.00
* @brief    RTE Device Configuration for Nuvoton M5531 USART
*
* @copyright SPDX-License-Identifier: Apache-2.0
* @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
******************************************************************************/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_USART_PIN_CONFIG_H
#define __RTE_DEVICE_USART_PIN_CONFIG_H

#include "Driver_USART.h"
#include "NuMicro.h"
#include "RTE_Device/RTE_Device.h"
// <h> USART0 (UART0)
// <i> Configuration settings for Driver_USART0 in component ::CMSIS Driver:USART
//   <o> USART0_TX Pin <0=>Not Used <1=>PA1 <2=>PA5 <3=>PA7 <4=>PA14 <5=>PB9 <6=>PB13 <7=>PC12 <8=>PD3 <9=>PF0 <10=>PF3 <11=>PH10
#define RTE_USART0_TX_ID                 1
#if    (RTE_USART0_TX_ID == 0)
    #define RTE_SET_USART0_TX_PIN()
#elif  (RTE_USART0_TX_ID == 1)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PA1()
#elif  (RTE_USART0_TX_ID == 2)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PA5()
#elif  (RTE_USART0_TX_ID == 3)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PA7()
#elif  (RTE_USART0_TX_ID == 4)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PA14()
#elif  (RTE_USART0_TX_ID == 5)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PB9()
#elif  (RTE_USART0_TX_ID == 6)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PB13()
#elif  (RTE_USART0_TX_ID == 7)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PC12()
#elif  (RTE_USART0_TX_ID == 8)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PD3()
#elif  (RTE_USART0_TX_ID == 9)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PF0()
#elif  (RTE_USART0_TX_ID == 10)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PF3()
#elif  (RTE_USART0_TX_ID == 11)
    #define RTE_SET_USART0_TX_PIN()         SET_UART0_TXD_PH10()
#else
    #error "Invalid UART0_TX Pin Configuration!"
#endif

//   <o> USART0_RX Pin <0=>Not Used <1=>PA0 <2=>PA4 <3=>PA6 <4=>PA15 <5=>PB8 <6=>PB12 <7=>PC11 <8=>PD2 <9=>PF1 <10=>PF2 <11=>PG3 <12=>PH11
#define RTE_USART0_RX_ID                1
#if    (RTE_USART0_RX_ID == 0)
    #define RTE_SET_USART0_RX_PIN()
#elif  (RTE_USART0_RX_ID == 1)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PA0()
#elif  (RTE_USART0_RX_ID == 2)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PA4()
#elif  (RTE_USART0_RX_ID == 3)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PA6()
#elif  (RTE_USART0_RX_ID == 4)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PA15()
#elif  (RTE_USART0_RX_ID == 5)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PB8()
#elif  (RTE_USART0_RX_ID == 6)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PB12()
#elif  (RTE_USART0_RX_ID == 7)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PC11()
#elif  (RTE_USART0_RX_ID == 8)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PD2()
#elif  (RTE_USART0_RX_ID == 9)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PF1()
#elif  (RTE_USART0_RX_ID == 10)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PF2()
#elif  (RTE_USART0_RX_ID == 11)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PG3()
#elif  (RTE_USART0_RX_ID == 12)
    #define RTE_SET_USART0_RX_PIN()         SET_UART0_RXD_PH11()
#else
    #error "Invalid USART0_RX Pin Configuration!"
#endif
//   <o> USART0_CTS Pin <0=>Not Used <1=>PA5 <2=>PB11 <3=>PB15 <4=>PC7 <5=>PF5
#define RTE_USART0_CTS_ID               0
#if    (RTE_USART0_CTS_ID == 0)
    #define RTE_SET_USART0_CTS_PIN()
#elif  (RTE_USART0_CTS_ID == 1)
    #define RTE_SET_USART0_CTS_PIN()        SET_UART0_nCTS_PA5()
#elif  (RTE_USART0_CTS_ID == 2)
    #define RTE_SET_USART0_CTS_PIN()        SET_UART0_nCTS_PB11()
#elif  (RTE_USART0_CTS_ID == 3)
    #define RTE_SET_USART0_CTS_PIN()        SET_UART0_nCTS_PB15()
#elif  (RTE_USART0_CTS_ID == 4)
    #define RTE_SET_USART0_CTS_PIN()        SET_UART0_nCTS_PC7()
#elif  (RTE_USART0_CTS_ID == 5)
    #define RTE_SET_USART0_CTS_PIN()        SET_UART0_nCTS_PF5()
#else
    #error "Invalid UART0_CTS Pin Configuration!"
#endif
//   <o> USART0_RTS Pin <0=>Not Used <1=>PA4 <2=>PB10 <3=>PB14 <4=>PC6 <5=>PF4
#define RTE_USART0_RTS_ID               0
#if    (RTE_USART0_RTS_ID == 0)
    #define RTE_SET_USART0_RTS_PIN()
#elif  (RTE_USART0_RTS_ID == 1)
    #define RTE_SET_USART0_RTS_PIN()        SET_UART0_nRTS_PA4()
#elif  (RTE_USART0_RTS_ID == 2)
    #define RTE_SET_USART0_RTS_PIN()        SET_UART0_nRTS_PB10()
#elif  (RTE_USART0_RTS_ID == 3)
    #define RTE_SET_USART0_RTS_PIN()        SET_UART0_nRTS_PB14()
#elif  (RTE_USART0_RTS_ID == 4)
    #define RTE_SET_USART0_RTS_PIN()        SET_UART0_nRTS_PC6()
#elif  (RTE_USART0_RTS_ID == 5)
    #define RTE_SET_USART0_RTS_PIN()        SET_UART0_nRTS_PF4()
#else
    #error "Invalid UART0_RTS Pin Configuration!"
#endif
// </h>
// <h> USART1 (UART1)
// <i> Configuration settings for Driver_USART1 in component ::CMSIS Driver:USART
//   <o> USART1_TX Pin <0=>Not Used <1=>PA3 <2=>PA9 <3=>PB3 <4=>PB7 <5=>PD11 <6=>PE13 <7=>PF0 <8=>PG0 <9=>PG13 <10=>PH8
#define RTE_USART1_TX_ID                 4
#if    (RTE_USART1_TX_ID == 0)
    #define RTE_SET_USART1_TX_PIN()
#elif  (RTE_USART1_TX_ID == 1)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PA3()
#elif  (RTE_USART1_TX_ID == 2)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PA9()
#elif  (RTE_USART1_TX_ID == 3)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PB3()
#elif  (RTE_USART1_TX_ID == 4)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PB7()
#elif  (RTE_USART1_TX_ID == 5)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PD11()
#elif  (RTE_USART1_TX_ID == 6)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PE13()
#elif  (RTE_USART1_TX_ID == 7)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PF0()
#elif  (RTE_USART1_TX_ID == 8)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PG0()
#elif  (RTE_USART1_TX_ID == 9)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PG13()
#elif  (RTE_USART1_TX_ID == 10)
    #define RTE_SET_USART1_TX_PIN()         SET_UART1_TXD_PH8()
#else
    #error "Invalid USART1_TX Pin Configuration!"
#endif
//   <o> USART1_RX Pin <0=>Not Used <1=>PA2 <2=>PA8 <3=>PB2 <4=>PB6 <5=>PC8 <6=>PD10 <7=>PF1 <8=>PG1 <9=>PG14 <10=>PH9
#define RTE_USART1_RX_ID                4
#if    (RTE_USART1_RX_ID == 0)
    #define RTE_SET_USART1_RX_PIN()
#elif  (RTE_USART1_RX_ID == 1)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PA2()
#elif  (RTE_USART1_RX_ID == 2)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PA8()
#elif  (RTE_USART1_RX_ID == 3)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PB2()
#elif  (RTE_USART1_RX_ID == 4)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PB6()
#elif  (RTE_USART1_RX_ID == 5)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PC8()
#elif  (RTE_USART1_RX_ID == 6)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PD10()
#elif  (RTE_USART1_RX_ID == 7)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PF1()
#elif  (RTE_USART1_RX_ID == 8)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PG1()
#elif  (RTE_USART1_RX_ID == 9)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PG14()
#elif  (RTE_USART1_RX_ID == 10)
    #define RTE_SET_USART1_RX_PIN()         SET_UART1_RXD_PH9()
#else
    #error "Invalid USART1_RX Pin Configuration!"
#endif
//   <o> USART1_CTS Pin <0=>Not Used <1=>PA1 <2=>PB9 <3=>PE11
#define RTE_USART1_CTS_ID               0
#if    (RTE_USART1_CTS_ID == 0)
    #define RTE_SET_USART1_CTS_PIN()
#elif  (RTE_USART1_CTS_ID == 1)
    #define RTE_SET_USART1_CTS_PIN()        SET_UART1_nCTS_PA1()
#elif  (RTE_USART1_CTS_ID == 2)
    #define RTE_SET_USART1_CTS_PIN()        SET_UART1_nCTS_PB9()
#elif  (RTE_USART1_CTS_ID == 3)
    #define RTE_SET_USART1_CTS_PIN()        SET_UART1_nCTS_PE11()
#else
    #error "Invalid USART1_CTS Pin Configuration!"
#endif
//   <o> USART1_RTS Pin <0=>Not Used <1=>PA0 <2=>PB8 <3=>PE12
#define RTE_USART1_RTS_ID               0
#if    (RTE_USART1_RTS_ID == 0)
    #define RTE_USART1_RTS                  0
    #define RTE_SET_USART1_RTS_PIN()
#elif  (RTE_USART1_RTS_ID == 1)
    #define RTE_SET_USART1_RTS_PIN()        SET_UART1_nRTS_PA0()
#elif  (RTE_USART1_RTS_ID == 2)
    #define RTE_SET_USART1_RTS_PIN()        SET_UART1_nRTS_PB8()
#elif  (RTE_USART1_RTS_ID == 3)
    #define RTE_SET_USART1_RTS_PIN()        SET_UART1_nRTS_PE12()
#else
    #error "Invalid USART1_RTS Pin Configuration!"
#endif
// </h>
// <h> USART2 (UART2)
// <i> Configuration settings for Driver_USART2 in component ::CMSIS Driver:USART
// <o> USART2_TX Pin <0=>Not Used <1=>PB1 <2=>PB5 <3=>PC1 <4=>PC5 <5=>PC13 <6=>PE14 <7=>PE8 <8=>PF0 <9=>PF4 <10=>PG1 <11=>PI10 <12=>PI6
#define RTE_USART2_TX_ID                1
#if    (RTE_USART2_TX_ID == 0)
    #define RTE_USART2_TX                   0
    #define RTE_SET_USART2_TX_PIN()
#elif  (RTE_USART2_TX_ID == 1)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PB1()
#elif  (RTE_USART2_TX_ID == 2)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PB5()
#elif  (RTE_USART2_TX_ID == 3)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PC1()
#elif  (RTE_USART2_TX_ID == 4)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PC5()
#elif  (RTE_USART2_TX_ID == 5)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PC13()
#elif  (RTE_USART2_TX_ID == 6)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PE14()
#elif  (RTE_USART2_TX_ID == 7)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PE8()
#elif  (RTE_USART2_TX_ID == 8)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PF0()
#elif  (RTE_USART2_TX_ID == 9)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PF4()
#elif  (RTE_USART2_TX_ID == 10)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PG1()
#elif  (RTE_USART2_TX_ID == 11)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PI10()
#elif  (RTE_USART2_TX_ID == 12)
    #define RTE_SET_USART2_TX_PIN()         SET_UART2_TXD_PI6()
#else
    #error "Invalid USART2_TX Pin Configuration!"
#endif

//   <o> USART2_RX Pin <0=>Not Used <1=>PB0 <2=>PB4 <3=>PC0 <4=>PC4 <5=>PD12 <6=>PE15 <7=>PE9 <8=>PF1 <9=>PF5 <10=>PG0 <11=>PI11 <12=>PI7
#define RTE_USART2_RX_ID                1
#if    (RTE_USART2_RX_ID == 0)
    #define RTE_SET_USART2_RX_PIN()
#elif  (RTE_USART2_RX_ID == 1)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PB0()
#elif  (RTE_USART2_RX_ID == 2)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PB4()
#elif  (RTE_USART2_RX_ID == 3)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PC0()
#elif  (RTE_USART2_RX_ID == 4)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PC4()
#elif  (RTE_USART2_RX_ID == 5)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PD12()
#elif  (RTE_USART2_RX_ID == 6)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PE15()
#elif  (RTE_USART2_RX_ID == 7)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PE9()
#elif  (RTE_USART2_RX_ID == 8)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PF1()
#elif  (RTE_USART2_RX_ID == 9)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PF5()
#elif  (RTE_USART2_RX_ID == 10)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PG0()
#elif  (RTE_USART2_RX_ID == 11)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PI11()
#elif  (RTE_USART2_RX_ID == 12)
    #define RTE_SET_USART2_RX_PIN()         SET_UART2_RXD_PI7()
#else
    #error "Invalid USART2_RX Pin Configuration!"
#endif
//   <o> USART2_CTS Pin <0=>Not Used <1=>PC2 <2=>PD9 <3=>PF5 <4=>PI9
#define RTE_USART2_CTS_ID               0
#if    (RTE_USART2_CTS_ID == 0)
    #define RTE_SET_USART2_CTS_PIN()
#elif  (RTE_USART2_CTS_ID == 1)
    #define RTE_SET_USART2_CTS_PIN()        SET_UART2_nCTS_PC2()
#elif  (RTE_USART2_CTS_ID == 2)
    #define RTE_SET_USART2_CTS_PIN()        SET_UART2_nCTS_PD9()
#elif  (RTE_USART2_CTS_ID == 3)
    #define RTE_SET_USART2_CTS_PIN()        SET_UART2_nCTS_PF5()
#elif  (RTE_USART2_CTS_ID == 4)
    #define RTE_SET_USART2_CTS_PIN()        SET_UART2_nCTS_PI9()
#else
    #error "Invalid USART2_CTS Pin Configuration!"
#endif

//   <o> USART2_RTS Pin <0=>Not Used <1=>PC3 <2=>PD8 <3=>PF4 <4=>PI8
#define RTE_USART2_RTS_ID               0
#if    (RTE_USART2_RTS_ID == 0)
    #define RTE_SET_USART2_RTS_PIN()
#elif  (RTE_USART2_RTS_ID == 1)
    #define RTE_SET_USART2_RTS_PIN()        SET_UART2_nRTS_PC3()
#elif  (RTE_USART2_RTS_ID == 2)
    #define RTE_SET_USART2_RTS_PIN()        SET_UART2_nRTS_PD8()
#elif  (RTE_USART2_RTS_ID == 3)
    #define RTE_SET_USART2_RTS_PIN()        SET_UART2_nRTS_PF4()
#elif  (RTE_USART2_RTS_ID == 4)
    #define RTE_SET_USART2_RTS_PIN()        SET_UART2_nRTS_PI8()
#else
    #error "Invalid USART2_RTS Pin Configuration!"
#endif

// </h>
// <h> USART3 (UART3)
// <i> Configuration settings for Driver_USART3 in component ::CMSIS Driver:USART
//   <o> USART3_TX Pin <0=>Not Used <1=>PB15 <2=>PC3 <3=>PC10 <4=>PD1 <5=>PE1 <6=>PE10 <7=>PE14 <8=>PF5 <9=>PI14
#define RTE_USART3_TX_ID                1
#if    (RTE_USART3_TX_ID == 0)
    #define RTE_SET_USART3_TX_PIN()
#elif  (RTE_USART3_TX_ID == 1)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PB15()
#elif  (RTE_USART3_TX_ID == 2)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PC3()
#elif  (RTE_USART3_TX_ID == 3)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PC10()
#elif  (RTE_USART3_TX_ID == 4)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PD1()
#elif  (RTE_USART3_TX_ID == 5)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PE1()
#elif  (RTE_USART3_TX_ID == 6)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PE10()
#elif  (RTE_USART3_TX_ID == 7)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PE14()
#elif  (RTE_USART3_TX_ID == 8)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PF5()
#elif  (RTE_USART3_TX_ID == 9)
    #define RTE_SET_USART3_TX_PIN()         SET_UART3_TXD_PI14()
#else
    #error "Invalid USART3_TX Pin Configuration!"
#endif
//   <o> USART3_RX Pin <0=>Not Used <1=>PB14 <2=>PC2 <3=>PC9 <4=>PD0 <5=>PE0 <6=>PE11 <7=>PE15 <8=>PF4 <9=>PI15
#define RTE_USART3_RX_ID                1
#if    (RTE_USART3_RX_ID == 0)
    #define RTE_SET_USART3_RX_PIN()
#elif  (RTE_USART3_RX_ID == 1)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PB14()
#elif  (RTE_USART3_RX_ID == 2)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PC2()
#elif  (RTE_USART3_RX_ID == 3)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PC9()
#elif  (RTE_USART3_RX_ID == 4)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PD0()
#elif  (RTE_USART3_RX_ID == 5)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PE0()
#elif  (RTE_USART3_RX_ID == 6)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PE11()
#elif  (RTE_USART3_RX_ID == 7)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PE15()
#elif  (RTE_USART3_RX_ID == 8)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PF4()
#elif  (RTE_USART3_RX_ID == 9)
    #define RTE_SET_USART3_RX_PIN()         SET_UART3_RXD_PI15()
#else
    #error "Invalid USART3_RX Pin Configuration!"
#endif
//   <o> USART3_CTS Pin <0=>Not Used <1=>PB12 <2=>PD2 <3=>PH9
#define RTE_USART3_CTS_ID               0
#if    (RTE_USART3_CTS_ID == 0)
    #define RTE_SET_USART3_CTS_PIN()
#elif  (RTE_USART3_CTS_ID == 1)
    #define RTE_SET_USART3_CTS_PIN()        SET_UART3_nCTS_PB12()
#elif  (RTE_USART3_CTS_ID == 2)
    #define RTE_SET_USART3_CTS_PIN()        SET_UART3_nCTS_PD2()
#elif  (RTE_USART3_CTS_ID == 3)
    #define RTE_SET_USART3_CTS_PIN()        SET_UART3_nCTS_PH9()
#else
    #error "Invalid USART3_CTS Pin Configuration!"
#endif

//   <o> USART3_RTS Pin <0=>Not Used <1=>PB13 <2=>PD3 <3=>PH8
#define RTE_USART3_RTS_ID               0
#if    (RTE_USART3_RTS_ID == 0)
    #define RTE_SET_USART3_RTS_PIN()
#elif  (RTE_USART3_RTS_ID == 1)
    #define RTE_SET_USART3_RTS_PIN()        SET_UART3_nRTS_PB13()
#elif  (RTE_USART3_RTS_ID == 2)
    #define RTE_SET_USART3_RTS_PIN()        SET_UART3_nRTS_PD3()
#elif  (RTE_USART3_RTS_ID == 3)
    #define RTE_SET_USART3_RTS_PIN()        SET_UART3_nRTS_PH8()
#else
    #error "Invalid USART3_RTS Pin Configuration!"
#endif

// </h>
// <h> USART4 (UART4)
// <i> Configuration settings for Driver_USART4 in component ::CMSIS Driver:USART
//   <o> USART4_TX Pin <0=>Not Used <1=>PA3 <2=>PA12 <3=>PB11 <4=>PC5 <5=>PC7 <6=>PF7 <7=>PH2 <8=>PH10 <9=>PI12
#define RTE_USART4_TX_ID                1
#if    (RTE_USART4_TX_ID == 0)
    #define RTE_USART4_TX                   0
    #define RTE_SET_USART4_TX_PIN()
#elif  (RTE_USART4_TX_ID == 1)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PA3()
#elif  (RTE_USART4_TX_ID == 2)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PA12()
#elif  (RTE_USART4_TX_ID == 3)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PB11()
#elif  (RTE_USART4_TX_ID == 4)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PC5()
#elif  (RTE_USART4_TX_ID == 5)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PC7()
#elif  (RTE_USART4_TX_ID == 6)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PF7()
#elif  (RTE_USART4_TX_ID == 7)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PH2()
#elif  (RTE_USART4_TX_ID == 8)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PH10()
#elif  (RTE_USART4_TX_ID == 9)
    #define RTE_SET_USART4_TX_PIN()         SET_UART4_TXD_PI12()
#else
    #error "Invalid USART4_TX Pin Configuration!"
#endif

//   <o> USART4_RX Pin <0=>Not Used <1=>PA2 <2=>PA13 <3=>PB10 <4=>PC4 <5=>PC6 <6=>PF6 <7=>PG3 <8=>PH3 <9=>PH11 <10=>PI13
#define RTE_USART4_RX_ID                1
#if    (RTE_USART4_RX_ID == 0)
    #define RTE_USART4_RX                   0
    #define RTE_SET_USART4_RX_PIN()
#elif  (RTE_USART4_RX_ID == 1)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PA2()
#elif  (RTE_USART4_RX_ID == 2)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PA13()
#elif  (RTE_USART4_RX_ID == 3)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PB10()
#elif  (RTE_USART4_RX_ID == 4)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PC4()
#elif  (RTE_USART4_RX_ID == 5)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PC6()
#elif  (RTE_USART4_RX_ID == 6)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PF6()
#elif  (RTE_USART4_RX_ID == 7)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PG3()
#elif  (RTE_USART4_RX_ID == 8)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PH3()
#elif  (RTE_USART4_RX_ID == 9)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PH11()
#elif  (RTE_USART4_RX_ID == 10)
    #define RTE_SET_USART4_RX_PIN()         SET_UART4_RXD_PI13()
#else
    #error "Invalid USART4_RX Pin Configuration!"
#endif

//   <o> USART4_CTS Pin <0=>Not Used <1=>PC8 <2=>PE1 <3=>PE14
#define RTE_USART4_CTS_ID               0
#if    (RTE_USART4_CTS_ID == 0)
    #define RTE_SET_USART4_CTS_PIN()
#elif  (RTE_USART4_CTS_ID == 1)
    #define RTE_SET_USART4_CTS_PIN()        SET_UART4_nCTS_PC8()
#elif  (RTE_USART4_CTS_ID == 2)
    #define RTE_SET_USART4_CTS_PIN()        SET_UART4_nCTS_PE1()
#elif  (RTE_USART4_CTS_ID == 3)
    #define RTE_SET_USART4_CTS_PIN()        SET_UART4_nCTS_PE14()
#else
    #error "Invalid USART4_CTS Pin Configuration!"
#endif
//   <o> USART4_RTS Pin <0=>Not Used <1=>PE0 <2=>PE13 <3=>PE15
#define RTE_USART4_RTS_ID               0
#if    (RTE_USART4_RTS_ID == 0)
    #define RTE_SET_USART4_RTS_PIN()
#elif  (RTE_USART4_RTS_ID == 1)
    #define RTE_SET_USART4_RTS_PIN()        SET_UART4_nRTS_PE0()
#elif  (RTE_USART4_RTS_ID == 2)
    #define RTE_SET_USART4_RTS_PIN()        SET_UART4_nRTS_PE13()
#elif  (RTE_USART4_RTS_ID == 3)
    #define RTE_SET_USART4_RTS_PIN()        SET_UART4_nRTS_PE15()
#else
    #error "Invalid USART4_RTS Pin Configuration!"
#endif
// </h>
// <h> USART5 (UART5)
// <i> Configuration settings for Driver_USART5 in component ::CMSIS Driver:USART
//   <o> USART5_TX Pin <0=>Not Used <1=>PA5 <2=>PB5 <3=>PE7 <4=>PF11 <5=>PH0
#define RTE_USART5_TX_ID                1
#if    (RTE_USART5_TX_ID == 0)
    #define RTE_SET_USART5_TX_PIN()
#elif  (RTE_USART5_TX_ID == 1)
    #define RTE_SET_USART5_TX_PIN()         SET_UART5_TXD_PA5()
#elif  (RTE_USART5_TX_ID == 2)
    #define RTE_SET_USART5_TX_PIN()         SET_UART5_TXD_PB5()
#elif  (RTE_USART5_TX_ID == 3)
    #define RTE_SET_USART5_TX_PIN()         SET_UART5_TXD_PE7()
#elif  (RTE_USART5_TX_ID == 4)
    #define RTE_SET_USART5_TX_PIN()         SET_UART5_TXD_PF11()
#elif  (RTE_USART5_TX_ID == 5)
    #define RTE_SET_USART5_TX_PIN()         SET_UART5_TXD_PH0()
#else
    #error "Invalid USART5_TX Pin Configuration!"
#endif
//   <o> USART5_RX Pin <0=>Not Used <1=>PA4 <2=>PB4 <3=>PE6 <4=>PF10 <5=>PH1
#define RTE_USART5_RX_ID                1
#if    (RTE_USART5_RX_ID == 0)
    #define RTE_SET_USART5_RX_PIN()
#elif  (RTE_USART5_RX_ID == 1)
    #define RTE_SET_USART5_RX_PIN()         SET_UART5_RXD_PA4()
#elif  (RTE_USART5_RX_ID == 2)
    #define RTE_SET_USART5_RX_PIN()         SET_UART5_RXD_PB4()
#elif  (RTE_USART5_RX_ID == 3)
    #define RTE_SET_USART5_RX_PIN()         SET_UART5_RXD_PE6()
#elif  (RTE_USART5_RX_ID == 4)
    #define RTE_SET_USART5_RX_PIN()         SET_UART5_RXD_PF10()
#elif  (RTE_USART5_RX_ID == 5)
    #define RTE_SET_USART5_RX_PIN()         SET_UART5_RXD_PH1()
#else
    #error "Invalid USART5_RX Pin Configuration!"
#endif
//   <o> USART5_CTS Pin <0=>Not Used <1=>PB2 <2=>PF8 <3=>PH3
#define RTE_USART5_CTS_ID               0
#if    (RTE_USART5_CTS_ID == 0)
    #define RTE_SET_USART5_CTS_PIN()
#elif  (RTE_USART5_CTS_ID == 1)
    #define RTE_SET_USART5_CTS_PIN()        SET_UART5_nCTS_PB2()
#elif  (RTE_USART5_CTS_ID == 2)
    #define RTE_SET_USART5_CTS_PIN()        SET_UART5_nCTS_PF8()
#elif  (RTE_USART5_CTS_ID == 3)
    #define RTE_SET_USART5_CTS_PIN()        SET_UART5_nCTS_PH3()
#else
    #error "Invalid USART5_CTS Pin Configuration!"
#endif
//   <o> USART5_RTS Pin <0=>Not Used <1=>PB3 <2=>PF9 <3=>PH2
#define RTE_USART5_RTS_ID               0
#if    (RTE_USART5_RTS_ID == 0)
    #define RTE_SET_USART5_RTS_PIN()
#elif  (RTE_USART5_RTS_ID == 1)
    #define RTE_SET_USART5_RTS_PIN()        SET_UART5_nRTS_PB3()
#elif  (RTE_USART5_RTS_ID == 2)
    #define RTE_SET_USART5_RTS_PIN()        SET_UART5_nRTS_PF9()
#elif  (RTE_USART5_RTS_ID == 3)
    #define RTE_SET_USART5_RTS_PIN()        SET_UART5_nRTS_PH2()
#else
    #error "Invalid USART5_RTS Pin Configuration!"
#endif
// </h>
// <h> USART6 (UART6)
// <i> Configuration settings for Driver_USART6 in component ::CMSIS Driver:USART
//   <o> USART6_TX Pin <0=>Not Used <1=>PA11 <2=>PC7 <3=>PC12 <4=>PD7 <5=>PE5 <6=>PE14 <7=>PH4 <8=>PJ10
#define RTE_USART6_TX_ID                1
#if    (RTE_USART6_TX_ID == 0)
    #define RTE_SET_USART6_TX_PIN()
#elif  (RTE_USART6_TX_ID == 1)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PA11()
#elif  (RTE_USART6_TX_ID == 2)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PC7()
#elif  (RTE_USART6_TX_ID == 3)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PC12()
#elif  (RTE_USART6_TX_ID == 4)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PD7()
#elif  (RTE_USART6_TX_ID == 5)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PE5()
#elif  (RTE_USART6_TX_ID == 6)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PE14()
#elif  (RTE_USART6_TX_ID == 7)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PH4()
#elif  (RTE_USART6_TX_ID == 8)
    #define RTE_SET_USART6_TX_PIN()         SET_UART6_TXD_PJ10()
#else
    #error "Invalid USART6_TX Pin Configuration!"
#endif
//   <o> USART6_RX Pin <0=>Not Used <1=>PA10 <2=>PC6 <3=>PC11 <4=>PD6 <5=>PD13 <6=>PE4 <7=>PE15 <8=>PH5 <9=>PJ11
#define RTE_USART6_RX_ID                1
#if    (RTE_USART6_RX_ID == 0)
    #define RTE_SET_USART6_RX_PIN()
#elif  (RTE_USART6_RX_ID == 1)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PA10()
#elif  (RTE_USART6_RX_ID == 2)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PC6()
#elif  (RTE_USART6_RX_ID == 3)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PC11()
#elif  (RTE_USART6_RX_ID == 4)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PD6()
#elif  (RTE_USART6_RX_ID == 5)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PD13()
#elif  (RTE_USART6_RX_ID == 6)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PE4()
#elif  (RTE_USART6_RX_ID == 7)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PE12()
#elif  (RTE_USART6_RX_ID == 8)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PH5()
#elif  (RTE_USART6_RX_ID == 9)
    #define RTE_SET_USART6_RX_PIN()         SET_UART6_RXD_PJ11()
#else
    #error "Invalid USART6_RX Pin Configuration!"
#endif
//   <o> USART6_CTS Pin <0=>Not Used <1=>PC9 <2=>PE2
#define RTE_USART6_CTS_ID               0
#if    (RTE_USART6_CTS_ID == 0)
    #define RTE_SET_USART6_CTS_PIN()
#elif  (RTE_USART6_CTS_ID == 1)
    #define RTE_SET_USART6_CTS_PIN()        SET_UART6_nCTS_PC9()
#elif  (RTE_USART6_CTS_ID == 2)
    #define RTE_SET_USART6_CTS_PIN()        SET_UART6_nCTS_PE2()
#else
    #error "Invalid USART6_CTS Pin Configuration!"
#endif
//   <o> USART6_RTS Pin <0=>Not Used <1=>PC10 <2=>PE3
#define RTE_USART6_RTS_ID               0
#if    (RTE_USART6_RTS_ID == 0)
    #define RTE_SET_USART6_RTS_PIN()
#elif  (RTE_USART6_RTS_ID == 1)
    #define RTE_SET_USART6_RTS_PIN()        SET_UART6_nRTS_PC10()
#elif  (RTE_USART6_RTS_ID == 2)
    #define RTE_SET_USART6_RTS_PIN()        SET_UART6_nRTS_PE3()
#else
    #error "Invalid USART6_RTS Pin Configuration!"
#endif
// </h>
// <h> USART7 (UART7)
// <i> Configuration settings for Driver_USART7 in component ::CMSIS Driver:USART
//   <o> USART7_TX Pin <0=>Not Used <1=>PA9 <2=>PB9 <3=>PD9 <4=>PE3 <5=>PG11 <6=>PH6 <7=>PJ8
#define RTE_USART7_TX_ID                2
#if    (RTE_USART7_TX_ID == 0)
    #define RTE_SET_USART7_TX_PIN()
#elif  (RTE_USART7_TX_ID == 1)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PA9()
#elif  (RTE_USART7_TX_ID == 2)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PB9()
#elif  (RTE_USART7_TX_ID == 3)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PD9()
#elif  (RTE_USART7_TX_ID == 4)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PE3()
#elif  (RTE_USART7_TX_ID == 5)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PG11()
#elif  (RTE_USART7_TX_ID == 6)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PH6()
#elif  (RTE_USART7_TX_ID == 7)
    #define RTE_SET_USART7_TX_PIN()         SET_UART7_TXD_PJ8()
#else
    #error "Invalid USART7_TX Pin Configuration!"
#endif
//   <o> USART7_RX Pin <0=>Not Used <1=>PA8 <2=>PB8 <3=>PD8 <4=>PE2 <5=>PG12 <6=>PH7 <7=>PJ9
#define RTE_USART7_RX_ID                2
#if    (RTE_USART7_RX_ID == 0)
    #define RTE_SET_USART7_RX_PIN()
#elif  (RTE_USART7_RX_ID == 1)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PA8()
#elif  (RTE_USART7_RX_ID == 2)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PB8()
#elif  (RTE_USART7_RX_ID == 3)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PD8()
#elif  (RTE_USART7_RX_ID == 4)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PE2()
#elif  (RTE_USART7_RX_ID == 5)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PG12()
#elif  (RTE_USART7_RX_ID == 6)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PH7()
#elif  (RTE_USART7_RX_ID == 7)
    #define RTE_SET_USART7_RX_PIN()         SET_UART7_RXD_PJ9()
#else
    #error "Invalid USART7_RX Pin Configuration!"
#endif
//   <o> USART7_CTS Pin <0=>Not Used <1=>PE4 <2=>PH5
#define RTE_USART7_CTS_ID               0
#if    (RTE_USART7_CTS_ID == 0)
    #define RTE_SET_USART7_CTS_PIN()
#elif  (RTE_USART7_CTS_ID == 1)
    #define RTE_SET_USART7_CTS_PIN()        SET_UART7_nCTS_PE4()
#elif  (RTE_USART7_CTS_ID == 2)
    #define RTE_SET_USART7_CTS_PIN()        SET_UART7_nCTS_PH5()
#else
    #error "Invalid USART7_CTS Pin Configuration!"
#endif
//   <o> USART7_RTS Pin <0=>Not Used <1=>PE5 <2=>PH4
#define RTE_USART7_RTS_ID               0
#if    (RTE_USART7_RTS_ID == 0)
    #define RTE_SET_USART7_RTS_PIN()
#elif  (RTE_USART7_RTS_ID == 1)
    #define RTE_SET_USART7_RTS_PIN()        SET_UART7_nRTS_PE5()
#elif  (RTE_USART7_RTS_ID == 2)
    #define RTE_SET_USART7_RTS_PIN()        SET_UART7_nRTS_PH4()
#else
    #error "Invalid USART7_RTS Pin Configuration!"
#endif
// </h>
// <h> USART8 (UART8)
// <i> Configuration settings for Driver_USART8 in component ::CMSIS Driver:USART
//   <o> USART8_TX Pin <0=>Not Used <1=>PD11 <2=>PE1 <3=>PE14 <4=>PJ0 <5=>PJ4
#define RTE_USART8_TX_ID                2
#if    (RTE_USART8_TX_ID == 0)
    #define RTE_SET_USART8_TX_PIN()
#elif  (RTE_USART8_TX_ID == 1)
    #define RTE_SET_USART8_TX_PIN()         SET_UART8_TXD_PD11()
#elif  (RTE_USART8_TX_ID == 2)
    #define RTE_SET_USART8_TX_PIN()         SET_UART8_TXD_PE1()
#elif  (RTE_USART8_TX_ID == 3)
    #define RTE_SET_USART8_TX_PIN()         SET_UART8_TXD_PE14()
#elif  (RTE_USART8_TX_ID == 4)
    #define RTE_SET_USART8_TX_PIN()         SET_UART8_TXD_PJ0()
#elif  (RTE_USART8_TX_ID == 5)
    #define RTE_SET_USART8_TX_PIN()         SET_UART8_TXD_PJ4()
#else
    #error "Invalid USART8_TX Pin Configuration!"
#endif
//   <o> USART8_RX Pin <0=>Not Used <1=>PD10 <2=>PE0 <3=>PE15 <4=>PJ1 <5=>PJ5
#define RTE_USART8_RX_ID                2
#if    (RTE_USART8_RX_ID == 0)
    #define RTE_SET_USART8_RX_PIN()
#elif  (RTE_USART8_RX_ID == 1)
    #define RTE_SET_USART8_RX_PIN()         SET_UART8_RXD_PD10()
#elif  (RTE_USART8_RX_ID == 2)
    #define RTE_SET_USART8_RX_PIN()         SET_UART8_RXD_PE0()
#elif  (RTE_USART8_RX_ID == 3)
    #define RTE_SET_USART8_RX_PIN()         SET_UART8_RXD_PE15()
#elif  (RTE_USART8_RX_ID == 4)
    #define RTE_SET_USART8_RX_PIN()         SET_UART8_RXD_PJ1()
#elif  (RTE_USART8_RX_ID == 5)
    #define RTE_SET_USART8_RX_PIN()         SET_UART8_RXD_PJ5()
#else
    #error "Invalid USART8_RX Pin Configuration!"
#endif
//   <o> USART8_CTS Pin <0=>Not Used <1=>PC13 <2=>PE3 <3=>PI14 <4=>PJ2
#define RTE_USART8_CTS_ID               0
#if    (RTE_USART8_CTS_ID == 0)
    #define RTE_SET_USART8_CTS_PIN()
#elif  (RTE_USART8_CTS_ID == 1)
    #define RTE_SET_USART8_CTS_PIN()        SET_UART8_nCTS_PC13()
#elif  (RTE_USART8_CTS_ID == 2)
    #define RTE_SET_USART8_CTS_PIN()        SET_UART8_nCTS_PE3()
#elif  (RTE_USART8_CTS_ID == 3)
    #define RTE_SET_USART8_CTS_PIN()        SET_UART8_nCTS_PI14()
#elif  (RTE_USART8_CTS_ID == 4)
    #define RTE_SET_USART8_CTS_PIN()        SET_UART8_nCTS_PJ2()
#else
    #error "Invalid USART8_CTS Pin Configuration!"
#endif
//   <o> USART8_RTS Pin <0=>Not Used <1=>PD12 <2=>PE2 <3=>PI15  <4=>PJ3
#define RTE_USART8_RTS_ID               0
#if    (RTE_USART8_RTS_ID == 0)
    #define RTE_SET_USART8_RTS_PIN()
#elif  (RTE_USART8_RTS_ID == 1)
    #define RTE_SET_USART8_RTS_PIN()        SET_UART8_nRTS_PD12()
#elif  (RTE_USART8_RTS_ID == 2)
    #define RTE_SET_USART8_RTS_PIN()        SET_UART8_nRTS_PE2()
#elif  (RTE_USART8_RTS_ID == 3)
    #define RTE_SET_USART8_RTS_PIN()        SET_UART8_nRTS_PI15()
#elif  (RTE_USART8_RTS_ID == 4)
    #define RTE_SET_USART8_RTS_PIN()        SET_UART8_nRTS_PJ3()
#else
    #error "Invalid USART8_RTS Pin Configuration!"
#endif
// </h>
// <h> USART9 (UART9)
// <i> Configuration settings for Driver_USART9 in component ::CMSIS Driver:USART
//   <o> USART9_TX Pin <0=>Not Used <1=>PE5 <2=>PF3 <3=>PF9 <4=>PH10 <5=>PH12
#define RTE_USART9_TX_ID                1
#if    (RTE_USART9_TX_ID == 0)
    #define RTE_SET_USART9_TX_PIN()
#elif  (RTE_USART9_TX_ID == 1)
    #define RTE_SET_USART9_TX_PIN()         SET_UART9_TXD_PE5()
#elif  (RTE_USART9_TX_ID == 2)
    #define RTE_SET_USART9_TX_PIN()         SET_UART9_TXD_PF3()
#elif  (RTE_USART9_TX_ID == 3)
    #define RTE_SET_USART9_TX_PIN()         SET_UART9_TXD_PF9()
#elif  (RTE_USART9_TX_ID == 4)
    #define RTE_SET_USART9_TX_PIN()         SET_UART9_TXD_PH10()
#elif  (RTE_USART9_TX_ID == 5)
    #define RTE_SET_USART9_TX_PIN()         SET_UART9_TXD_PH12()
#else
    #error "Invalid USART9_TX Pin Configuration!"
#endif
//   <o> USART9_RX Pin <0=>Not Used <1=>PE4 <2=>PF2 <3=>PF8 <4=>PH11 <5=>PH13 <6=>PH2
#define RTE_USART9_RX_ID                1
#if    (RTE_USART9_RX_ID == 0)
    #define RTE_SET_USART9_RX_PIN()
#elif  (RTE_USART9_RX_ID == 1)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PE4()
#elif  (RTE_USART9_RX_ID == 2)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PF2()
#elif  (RTE_USART9_RX_ID == 3)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PF8()
#elif  (RTE_USART9_RX_ID == 4)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PH11()
#elif  (RTE_USART9_RX_ID == 5)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PH13()
#elif  (RTE_USART9_RX_ID == 6)
    #define RTE_SET_USART9_RX_PIN()         SET_UART9_RXD_PH2()
#else
    #error "Invalid USART9_RX Pin Configuration!"
#endif
//   <o> USART9_CTS Pin <0=>Not Used <1=>PE7 <2=>PF11 <3=>PH6 <4=>PH8 <5=>PJ6
#define RTE_USART9_CTS_ID               0
#if    (RTE_USART9_CTS_ID == 0)
    #define RTE_SET_USART9_CTS_PIN()
#elif  (RTE_USART9_CTS_ID == 1)
    #define RTE_SET_USART9_CTS_PIN()        SET_UART9_nCTS_PE7()
#elif  (RTE_USART9_CTS_ID == 2)
    #define RTE_SET_USART9_CTS_PIN()        SET_UART9_nCTS_PF11()
#elif  (RTE_USART9_CTS_ID == 3)
    #define RTE_SET_USART9_CTS_PIN()        SET_UART9_nCTS_PH6()
#elif  (RTE_USART9_CTS_ID == 4)
    #define RTE_SET_USART9_CTS_PIN()        SET_UART9_nCTS_PH8()
#elif  (RTE_USART9_CTS_ID == 5)
    #define RTE_SET_USART9_CTS_PIN()        SET_UART9_nCTS_PJ6()
#else
    #error "Invalid USART9_CTS Pin Configuration!"
#endif
//   <o> USART9_RTS Pin <0=>Not Used <1=>PE6 <2=>PF10 <3=>PH7 <4=>PH9 <5=>PJ7
#define RTE_USART9_RTS_ID               0
#if    (RTE_USART9_RTS_ID == 0)
    #define RTE_SET_USART9_RTS_PIN()
#elif  (RTE_USART9_RTS_ID == 1)
    #define RTE_SET_USART9_RTS_PIN()        SET_UART9_nRTS_PE6()
#elif  (RTE_USART9_RTS_ID == 2)
    #define RTE_SET_USART9_RTS_PIN()        SET_UART9_nRTS_PF10()
#elif  (RTE_USART9_RTS_ID == 3)
    #define RTE_SET_USART9_RTS_PIN()        SET_UART9_nRTS_PH7()
#elif  (RTE_USART9_RTS_ID == 4)
    #define RTE_SET_USART9_RTS_PIN()        SET_UART9_nRTS_PH9()
#elif  (RTE_USART9_RTS_ID == 5)
    #define RTE_SET_USART9_RTS_PIN()        SET_UART9_nRTS_PJ7()
#else
    #error "Invalid USART9_RTS Pin Configuration!"
#endif
// </h>
// <h> USART10 (SCUART0)
// <i> Configuration settings for Driver_USART10 in component ::CMSIS Driver:USART
// </h>
// <h> USART11 (SCUART1)
// <i> Configuration settings for Driver_USART11 in component ::CMSIS Driver:USART
// </h>
// <h> USART12 (SCUART2)
// <i> Configuration settings for Driver_USART12 in component ::CMSIS Driver:USART
// </h>
// <h> USART13 (LPUART0)
// <i> Configuration settings for Driver_USART13 in component ::CMSIS Driver:USART
//   <o> USART13_TX Pin <0=>Not Used <1=>PA1 <2=>PA5 <3=>PA7 <4=>PB9 <5=>PB13 <6=>PC12 <7=>PD3 <8=>PF0 <9=>PF3 <10=>PH10
#define RTE_USART13_TX_ID                1
#if    (RTE_USART13_TX_ID == 0)
    #define RTE_SET_USART13_TX_PIN()
#elif  (RTE_USART13_TX_ID == 1)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PA1()
#elif  (RTE_USART13_TX_ID == 2)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PA5()
#elif  (RTE_USART13_TX_ID == 3)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PA7()
#elif  (RTE_USART13_TX_ID == 4)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PB9()
#elif  (RTE_USART13_TX_ID == 5)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PB13()
#elif  (RTE_USART13_TX_ID == 6)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PC12()
#elif  (RTE_USART13_TX_ID == 7)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PD3()
#elif  (RTE_USART13_TX_ID == 8)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PF0()
#elif  (RTE_USART13_TX_ID == 9)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PF3()
#elif  (RTE_USART13_TX_ID == 10)
    #define RTE_SET_USART13_TX_PIN()         SET_LPUART0_TXD_PH10()
#else
    #error "Invalid USART13_TX Pin Configuration!"
#endif
//   <o> USART13_RX Pin <0=>Not Used <1=>PA0 <2=>PA4 <3=>PA6 <4=>PB8 <5=>PB12 <6=>PC11 <7=>PD2 <8=>PF1 <9=>PF2 <10=>PH11
#define RTE_USART13_RX_ID                1
#if    (RTE_USART13_RX_ID == 0)
    #define RTE_SET_USART13_RX_PIN()
#elif  (RTE_USART13_RX_ID == 1)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PA0()
#elif  (RTE_USART13_RX_ID == 2)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PA4()
#elif  (RTE_USART13_RX_ID == 3)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PA6()
#elif  (RTE_USART13_RX_ID == 4)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PB8()
#elif  (RTE_USART13_RX_ID == 5)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PB12()
#elif  (RTE_USART13_RX_ID == 6)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PC11()
#elif  (RTE_USART13_RX_ID == 7)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PD2()
#elif  (RTE_USART13_RX_ID == 8)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PF1()
#elif  (RTE_USART13_RX_ID == 9)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PF2()
#elif  (RTE_USART13_RX_ID == 10)
    #define RTE_SET_USART13_RX_PIN()         SET_LPUART0_RXD_PH11()
#else
    #error "Invalid USART13_RX Pin Configuration!"
#endif
//   <o> USART13_CTS Pin <0=>Not Used <1=>PA5 <2=>PB11 <3=>PB15 <4=>PC7
#define RTE_USART13_CTS_ID               0
#if    (RTE_USART13_CTS_ID == 0)
    #define RTE_SET_USART13_CTS_PIN()
#elif  (RTE_USART13_CTS_ID == 1)
    #define RTE_SET_USART13_CTS_PIN()        SET_UART9_nCTS_PA5()
#elif  (RTE_USART13_CTS_ID == 2)
    #define RTE_SET_USART13_CTS_PIN()        SET_UART9_nCTS_PB11()
#elif  (RTE_USART13_CTS_ID == 3)
    #define RTE_SET_USART13_CTS_PIN()        SET_UART9_nCTS_PB15()
#elif  (RTE_USART13_CTS_ID == 4)
    #define RTE_SET_USART13_CTS_PIN()        SET_UART9_nCTS_PC7()
#else
    #error "Invalid USART13_CTS Pin Configuration!"
#endif
//   <o> USART13_RTS Pin <0=>Not Used <1=>PA4 <2=>PB10 <3=>PB14 <4=>PC6
#define RTE_USART13_RTS_ID               0
#if    (RTE_USART13_RTS_ID == 0)
    #define RTE_SET_USART13_RTS_PIN()
#elif  (RTE_USART13_RTS_ID == 1)
    #define RTE_SET_USART13_RTS_PIN()        SET_UART9_nRTS_PA4()
#elif  (RTE_USART13_RTS_ID == 2)
    #define RTE_SET_USART13_RTS_PIN()        SET_UART9_nRTS_PB10()
#elif  (RTE_USART13_RTS_ID == 3)
    #define RTE_SET_USART13_RTS_PIN()        SET_UART9_nRTS_PB14()
#elif  (RTE_USART13_RTS_ID == 4)
    #define RTE_SET_USART13_RTS_PIN()        SET_UART9_nRTS_PC6()
#else
    #error "Invalid USART13_RTS Pin Configuration!"
#endif
// </h>
// <h> USART14 (UUART0)
// <i> Configuration settings for Driver_USART14 in component ::CMSIS Driver:USART
// <o> USART14_TX Pin <0=>Not Used <1=>PA9 <2=>PB14 <3=>PD2 <4=>PE4 <5=>PF1
#define RTE_USART14_TX_ID                1
#if    (RTE_USART14_TX_ID == 0)
    #define RTE_SET_USART14_TX_PIN()
#elif  (RTE_USART14_TX_ID == 1)
    #define RTE_SET_USART14_TX_PIN()         SET_USCI0_DAT1_PA9()
#elif  (RTE_USART14_TX_ID == 2)
    #define RTE_SET_USART14_TX_PIN()         SET_USCI0_DAT1_PB14()
#elif  (RTE_USART14_TX_ID == 3)
    #define RTE_SET_USART14_TX_PIN()         SET_USCI0_DAT1_PD2()
#elif  (RTE_USART14_TX_ID == 4)
    #define RTE_SET_USART14_TX_PIN()         SET_USCI0_DAT1_PE4()
#elif  (RTE_USART14_TX_ID == 5)
    #define RTE_SET_USART14_TX_PIN()         SET_USCI0_DAT1_PF1()
#else
    #error "Invalid USART14_TX Pin Configuration!"
#endif
//   <o> USART14_RX Pin <0=>Not Used <1=>PA10 <2=>PB13 <3=>PD1 <4=>PE3
#define RTE_USART14_RX_ID                1
#if    (RTE_USART14_RX_ID == 0)
    #define RTE_SET_USART14_RX_PIN()
#elif  (RTE_USART14_RX_ID == 1)
    #define RTE_SET_USART14_RX_PIN()         SET_USCI0_DAT0_PA10()
#elif  (RTE_USART14_RX_ID == 2)
    #define RTE_SET_USART14_RX_PIN()         SET_USCI0_DAT0_PB13()
#elif  (RTE_USART14_RX_ID == 3)
    #define RTE_SET_USART14_RX_PIN()         SET_USCI0_DAT0_PD1()
#elif  (RTE_USART14_RX_ID == 4)
    #define RTE_SET_USART14_RX_PIN()         SET_USCI0_DAT0_PE3()
#else
    #error "Invalid USART14_RX Pin Configuration!"
#endif

//   <o> USART14_CTS Pin <0=>Not Used <1=>PB0 <2=>PB13 <3=>PC13 <4=>PD4 <5=>PE6
#define RTE_USART14_CTS_ID               0
#if    (RTE_USART14_CTS_ID == 0)
    #define RTE_SET_USART14_CTS_PIN()
#elif  (RTE_USART14_CTS_ID == 1)
    #define RTE_SET_USART14_CTS_PIN()        SET_USCI0_CTL0_PB0()
#elif  (RTE_USART14_CTS_ID == 2)
    #define RTE_SET_USART14_CTS_PIN()        SET_USCI0_CTL0_PB13()
#elif  (RTE_USART14_CTS_ID == 3)
    #define RTE_SET_USART14_CTS_PIN()        SET_USCI0_CTL0_PC13()
#elif  (RTE_USART14_CTS_ID == 4)
    #define RTE_SET_USART14_CTS_PIN()        SET_USCI0_CTL0_PD4()
#elif  (RTE_USART14_CTS_ID == 5)
    #define RTE_SET_USART14_CTS_PIN()        SET_USCI0_CTL0_PE6()
#else
    #error "Invalid USART14_CTS Pin Configuration!"
#endif

//   <o> USART14_RTS Pin <0=>Not Used <1=>PA8 <2=>PB15 <3=>PD3 <4=>PE5 <5=>PF0
#define RTE_USART14_RTS_ID               0
#if    (RTE_USART14_RTS_ID == 0)
    #define RTE_SET_USART14_RTS_PIN()
#elif  (RTE_USART14_RTS_ID == 1)
    #define RTE_SET_USART14_RTS_PIN()        SET_USCI0_CTL1_PA8()
#elif  (RTE_USART14_RTS_ID == 2)
    #define RTE_SET_USART14_RTS_PIN()        SET_USCI0_CTL1_PB15()
#elif  (RTE_USART14_RTS_ID == 3)
    #define RTE_SET_USART14_RTS_PIN()        SET_USCI0_CTL1_PD3()
#elif  (RTE_USART14_RTS_ID == 4)
    #define RTE_SET_USART14_RTS_PIN()        SET_USCI0_CTL1_PE5()
#elif  (RTE_USART14_RTS_ID == 5)
    #define RTE_SET_USART14_RTS_PIN()        SET_USCI0_CTL1_PF0()
#else
    #error "Invalid USART14_RTS Pin Configuration!"
#endif

// </h>
#endif

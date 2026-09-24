/***************************************************************************//**
 * @file     targetdev.h
 * @version  V1.00
 * @brief    ISP support function header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "NuMicro.h"
#include "isp_user.h"

#define DetectPin                   PB12

/* rename for uart_transfer.c */
#define UART_N                          UART0
#define UART_N_IRQHandler       UART02_IRQHandler
#define UART_N_IRQn                 UART02_IRQn

#define CONFIG_SIZE 8 // in bytes

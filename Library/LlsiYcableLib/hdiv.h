/******************************************************************************
 * @file     hdiv.h
 * @version  V3.00
 * @brief    Hardware divider header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __HDIV_H__
#define __HDIV_H__

#include <stdint.h>

int32_t HDIV_Div(int32_t x, int16_t y);
int16_t HDIV_Mod(int32_t x, int16_t y);

#endif  /* __HDIV_H__ */

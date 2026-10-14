/******************************************************************************//**
 * @file     hdiv.c
 * @version  V3.00
 * @brief    Hardware divider sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdlib.h>
#include "hdiv.h"

int32_t HDIV_Div(int32_t x, int16_t y)
{
    return (x/y);
}

int16_t HDIV_Mod(int32_t x, int16_t y)
{
    return (x%y);
}

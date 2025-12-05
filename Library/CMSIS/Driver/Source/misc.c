/**************************************************************************//**
 * @file     misc.c
 * @version  V1.00
 * @brief    Miscellaneous functions for Nuvoton M3351 CMSIS-Driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>

#include "NuMicro.h"
#include "misc.h"

static void systick_init(uint32_t ticks_1ms)
{
    SysTick->LOAD = ticks_1ms - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

/*  Roughly delay function.
        If support RTOS, please use RTOS's delay function instead.
*/
__WEAK void delay_ms(uint32_t ms)
{
    uint32_t u32interval_ms;

    if (!(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk))
    {
        uint32_t u32ticks_1ms;
        u32ticks_1ms = SystemCoreClock / 1000;
        systick_init(u32ticks_1ms);
    }

    u32interval_ms = 1000 / (SystemCoreClock / (SysTick->LOAD + 1));

    if (u32interval_ms == 0)
        u32interval_ms = 1;

    uint32_t u32CurentVal, u32prevVal;

    u32CurentVal = SysTick->VAL & SysTick_VAL_CURRENT_Msk;

    for (uint32_t i = 0; i < ms; i += u32interval_ms)
    {
        u32prevVal = u32CurentVal;

        while (1)
        {
            u32CurentVal = SysTick->VAL & SysTick_VAL_CURRENT_Msk;

            if (u32CurentVal > u32prevVal)
                break;

            u32prevVal = u32CurentVal;
        }
    }
}

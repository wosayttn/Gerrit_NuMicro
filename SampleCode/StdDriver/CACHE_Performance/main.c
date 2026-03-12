/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the performance of CACHE.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

volatile uint32_t s_u32TickCnt = 0;

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

/* Pi calculation function */
int32_t g_f[2801];
const uint16_t pi_result[] = {3141, 5926, 5358, 9793, 2384, 6264, 3383, 2795,  288, 4197, 1693, 9937, 5105, 8209, 7494, 4592, 3078, 1640, 6286, 2089, 9862, 8034, 8253, 4211, 7067, 9821, 4808, 6513, 2823,  664, 7093, 8446,  955,  582, 2317, 2535, 9408, 1284, 8111, 7450, 2841,  270, 1938, 5211,  555, 9644, 6229, 4895, 4930, 3819, 6442, 8810, 9756, 6593, 3446, 1284, 7564, 8233, 7867, 8316, 5271, 2019,  914, 5648, 5669, 2346,  348, 6104, 5432, 6648, 2133, 9360, 7260, 2491, 4127, 3724, 5870,  660, 6315, 5881, 7488, 1520, 9209, 6282, 9254,  917, 1536, 4367, 8925, 9036,   11, 3305, 3054, 8820, 4665, 2138, 4146, 9519, 4151, 1609, 4330, 5727,  365, 7595, 9195, 3092, 1861, 1738, 1932, 6117, 9310, 5118, 5480, 7446, 2379, 9627, 4956, 7351, 8857, 5272, 4891, 2279, 3818, 3011, 9491, 2983, 3673, 3624, 4065, 6643,  860, 2139, 4946, 3952, 2473, 7190, 7021, 7986,  943, 7027, 7053, 9217, 1762, 9317, 6752, 3846, 7481, 8467, 6694,  513, 2000, 5681, 2714, 5263, 5608, 2778, 5771, 3427, 5778, 9609, 1736, 3717, 8721, 4684, 4090, 1224, 9534, 3014, 6549, 5853, 7105,  792, 2796, 8925, 8923, 5420, 1995, 6112, 1290, 2196,  864,  344, 1815, 9813, 6297, 7477, 1309, 9605, 1870, 7211, 3499, 9999, 8372, 9780, 4995, 1059, 7317, 3281, 6096, 3185};

int32_t pi_cal(void)
{
    int a = 10000, b = 0, c = 2800, d = 0, e = 0, g = 0;
    int32_t i;
    uint16_t r;

    for (; b - c;)
        g_f[b++] = a / 5;

    for (i = 0; d = 0, g = c * 2; c -= 14)
    {
        for (b = c; d += g_f[b] * a, g_f[b] = d % --g, d /= g--, --b;)
        {
            d *= b;
        }

        r = e + d / a;

        if (r != pi_result[i++])
        {
            printf("Pi calculation error at %d\n", i);
            return -1;
        }

        e = d % a;
    }

    return 0;
}

int32_t main(void)
{
    uint32_t u32StartTicks;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    SysTick_Config(SystemCoreClock / 1000);
    CACHE_Enable();
    CACHE_EnableHitMonitor();
    CACHE_EnableMissMonitor();
    u32StartTicks = s_u32TickCnt;
    pi_cal();
    printf("CPU run time w/ CACHE: %d ms\n", s_u32TickCnt - u32StartTicks);
    printf("CACHE Hit: %d\n", CACHE_GetHitCount());
    printf("CACHE Miss: %d\n", CACHE_GetMissCount());

    CACHE_Disable();
    u32StartTicks = s_u32TickCnt;
    pi_cal();
    printf("CPU run time w/o CACHE: %d ms\n", s_u32TickCnt - u32StartTicks);

    printf("End of CACHE Performance Test\n");

    /* Infinite loop to prevent from exiting */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

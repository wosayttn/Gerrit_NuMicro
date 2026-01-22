/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demo how to use XOM library
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "xomlib.h"

/* XOM limitation : After return from XOM region function, need to delay one cycle */
/* The marco XOM_CALL is using for avoid XOM limitation. */
#define XOM_CALL(pfunc, ret, ...)      { ret = pfunc(__VA_ARGS__); __NOP(); }

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

int32_t main(void)
{
    uint32_t u32Data = 0;
    int32_t  ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    InitDebugUart();
    /*
        This sample code is used to show how to call XOM libary.

        The XOM libary is build by XOMLib project.
        User need to add include path of xomlib.h and add object file xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC)
        to using XOM library built by XOMLib project.
    */

    printf("\n\n");
    printf("+-------------------------------------------+\n");
    printf("|  Demo how to use XOM library Sample Code  |\n");
    printf("+-------------------------------------------+\n");

    XOM_CALL(XOM_Add, u32Data, 100, 200);
    printf(" 100 + 200 = %u\n", u32Data);

    XOM_CALL(XOM_Sub, u32Data, 500, 100);
    printf(" 500 - 100 = %u\n", u32Data);

    XOM_CALL(XOM_Mul, u32Data, 200, 100);
    printf(" 200 * 100 = %u\n", u32Data);

    XOM_CALL(XOM_Div, u32Data, 1000, 250);
    printf("1000 / 250 = %u\n", u32Data);

    u32Data = XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0]));
    printf("Sum of ai32NumArray = %d\n", u32Data);

    printf("Done\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

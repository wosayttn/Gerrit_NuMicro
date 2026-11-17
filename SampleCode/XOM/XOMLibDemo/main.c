/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demo how to use XOM library in dual bank/Segment.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "xomlib.h"

/* XOM limitation : After return from XOM region function, need to delay one cycle */
/* The marco XOM_CALL is using for avoid XOM limitation. */
#define XOM_CALL(pfunc, ret, ...)      { ret = pfunc(__VA_ARGS__); __NOP(); }


void SYS_Init(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

int main(void)
{
    uint32_t u32Data = 0;
    int32_t  ai32NumArray[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Set Vector Table Offset Register */
    SCB->VTOR = 0x8000;

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code is used to show how to call XOM library in dual bank/segment.

        The XOM library is build by XOMLib project.
        Users need to add include path of xomlib.h and add object file xomlib.lib(Keil)/xomlib.a(IAR)/libXOMLib.a(GCC)
        to using XOM library built by XOMLib project.
    */

    printf("\n\n");
    printf("+------------------------------------------------+\n");
    printf("|  Demo how to use XOM library in Bank/Segment%d  |\n", DL_BANK);
    printf("+------------------------------------------------+\n");

    XOM_CALL(XOM_Add, u32Data, 100, (200 + DL_BANK));
    printf(" 100 + %d = %d\n", (200 + DL_BANK), u32Data);

    XOM_CALL(XOM_Sub, u32Data, 500, (100 + DL_BANK));
    printf(" 500 - %d = %d\n", (100 + DL_BANK), u32Data);

    XOM_CALL(XOM_Mul, u32Data, 200, (100 + DL_BANK));
    printf(" 200 * %d = %d\n", (100 + DL_BANK), u32Data);

    XOM_CALL(XOM_Div, u32Data, 1000, ((DL_BANK == 0) ? 250 : 500));
    printf("1000 / %d = %d\n", ((DL_BANK == 0) ? 250 : 500), u32Data);

    u32Data = XOM_Sum(ai32NumArray, sizeof(ai32NumArray) / sizeof(ai32NumArray[0]));
    printf("Sum of ai32NumArray = %d\n", u32Data);

    printf("Done\n");

    while (1);
}


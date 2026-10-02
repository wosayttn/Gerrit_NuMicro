/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a LLSI demo for marquee display in software mode.
 *           It needs to be used with WS2812 LED strip.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define HCLK_CLK    FREQ_180MHZ
#define TEST_COUNT  1

volatile uint32_t g_au32RED_Marquee[TEST_COUNT] = {0x00000000};
volatile uint32_t g_u32PatternToggle = 0;
volatile uint32_t g_u32DataCount = 0;

void LLSI0_IRQHandler()
{
    if(LLSI_GetIntFlag(LLSI0, LLSI_TXTH_INT_MASK))
    {
        while(g_u32DataCount < TEST_COUNT)
        {
            if(g_u32DataCount == (TEST_COUNT - 1))
                LLSI_SET_LAST_DATA(LLSI0);
            LLSI_WRITE_DATA(LLSI0, g_au32RED_Marquee[g_u32DataCount++]);
        }
        if(g_u32DataCount >= TEST_COUNT)
        {
            LLSI_DisableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to HCLK_CLK Hz */
    CLK_SetCoreClock(HCLK_CLK);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable LLSI0 module clock */
    CLK_EnableModuleClock(LLSI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PB multi-function pin for LLSI0 */
    SET_LLSI0_OUT_PB15();
}

void UART0_Init()
{
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as software mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. LLSI clock rate = 180MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_GRB, HCLK_CLK, 1250, 400, 850, 50000, 6, LLSI_IDLE_LOW);

    /* Set TX FIFO threshold */
    LLSI_SetFIFO(LLSI0, 2);

    /* Enable reset command function */
    LLSI_ENABLE_RESET_COMMAND(LLSI0);

    NVIC_EnableIRQ(LLSI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    LLSI Marquee Sample Code (Software Mode)    |\n");
    printf("+------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init LLSI */
    LLSI_Init();

    g_u32PatternToggle = 0;
    while(g_u32PatternToggle < 7)
    {
        g_u32DataCount = 0;

        /* Write 4 word data to LLSI_DATA */
        if(g_u32PatternToggle == 0)
        {
            LLSI_WRITE_DATA(LLSI0, 0x000000FF);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if(g_u32PatternToggle == 1)
        {
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if(g_u32PatternToggle == 2)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00FF0000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if(g_u32PatternToggle == 3)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x0000FF00);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if(g_u32PatternToggle == 4)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x000000FF);
        }
        else if(g_u32PatternToggle == 5)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
        }
        else if(g_u32PatternToggle == 6)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }

        /* Enable TX FIFO threshold interrupt */
        LLSI_EnableInt(LLSI0, LLSI_TXTH_INT_MASK);

        CLK_SysTickDelay(50000);

        g_u32PatternToggle++;
    }

    /* Close LLSI0 */
    LLSI_Close(LLSI0);

    printf("Exit LLSI sample code.\n");

    while(1);
}

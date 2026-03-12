/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is a LLSI demo for marquee display in software mode.
 *           It needs to be used with WS2812 LED strip.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  1

volatile uint32_t g_au32RED_Marquee[TEST_COUNT] = {0x00000000};
volatile uint32_t g_u32PatternToggle = 0;
volatile uint32_t g_u32DataCount = 0;

void LLSI0_IRQHandler()
{
    if (LLSI_GetIntFlag(LLSI0, LLSI_TXTH_INT_MASK))
    {
        while (g_u32DataCount < TEST_COUNT)
        {
            if (g_u32DataCount == (TEST_COUNT - 1))
                LLSI_SET_LAST_DATA(LLSI0);

            LLSI_WRITE_DATA(LLSI0, g_au32RED_Marquee[g_u32DataCount++]);
        }

        if (g_u32DataCount >= TEST_COUNT)
        {
            LLSI_DisableInt(LLSI0, LLSI_TXTH_INT_MASK);
        }
    }
}

void SYS_Init(void)
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
    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable LLSI0 module clock */
    CLK_EnableModuleClock(LLSI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set PB multi-function pin for LLSI0 */
    SET_LLSI0_OUT_PB15();
    /* Lock protected registers */
    SYS_LockReg();
}

void LLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init LLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as software mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. LLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    LLSI_Open(LLSI0, LLSI_MODE_SW, LLSI_FORMAT_GRB, CLK_GetPCLK0Freq(), 1250, 400, 850, 50000, 6, LLSI_IDLE_LOW);

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
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    LLSI Marquee Sample Code (Software Mode)    |\n");
    printf("+------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init LLSI */
    LLSI_Init();

    g_u32PatternToggle = 0;

    /*Enable LLSI Tx*/
    LLSI_ENABLE(LLSI0);

    while (g_u32PatternToggle < 7)
    {
        g_u32DataCount = 0;

        /* Write 4 word data to LLSI_DATA */
        if (g_u32PatternToggle == 0)
        {
            LLSI_WRITE_DATA(LLSI0, 0x000000FF);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 1)
        {
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 2)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00FF0000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 3)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x0000FF00);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 4)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x000000FF);
        }
        else if (g_u32PatternToggle == 5)
        {
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0x00000000);
            LLSI_WRITE_DATA(LLSI0, 0xFF000000);
        }
        else if (g_u32PatternToggle == 6)
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

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an ELLSI demo for marquee display in software mode.
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

void ELLSI0_IRQHandler()
{
    if (ELLSI_GetIntFlag(ELLSI0, ELLSI_TXTH_INT_MASK))
    {
        while (g_u32DataCount < TEST_COUNT)
        {
            if (g_u32DataCount == (TEST_COUNT - 1))
                ELLSI_SET_LAST_DATA(ELLSI0);

            ELLSI_WRITE_DATA(ELLSI0, g_au32RED_Marquee[g_u32DataCount++]);
        }

        if (g_u32DataCount >= TEST_COUNT)
        {
            ELLSI_DisableInt(ELLSI0, ELLSI_TXTH_INT_MASK);
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

    /* Enable ELLSI0 module clock */
    CLK_EnableModuleClock(ELLSI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PB multi-function pin for ELLSI0 */
    SET_ELLSI0_OUT_PB9();
    /* Lock protected registers */
    SYS_LockReg();
}

void ELLSI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init ELLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as software mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. ELLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    ELLSI_Open(ELLSI0, ELLSI_MODE_SW, ELLSI_FORMAT_GRB, CLK_GetPCLK0Freq(), 1250, 400, 850, 50000, 6, ELLSI_IDLE_LOW);

    /* Set TX FIFO threshold */
    ELLSI_SetFIFO(ELLSI0, 2);

    /* Enable reset command function */
    ELLSI_ENABLE_RESET_COMMAND(ELLSI0);

    NVIC_EnableIRQ(ELLSI0_IRQn);
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
    printf("+-------------------------------------------------+\n");
    printf("|    ELLSI Marquee Sample Code (Software Mode)    |\n");
    printf("+-------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init ELLSI */
    ELLSI_Init();

    g_u32PatternToggle = 0;

    /*Enable ELLSI Tx*/
    ELLSI_ENABLE(ELLSI0);

    while (g_u32PatternToggle < 7)
    {
        g_u32DataCount = 0;

        /* Write 4 word data to ELLSI_DATA */
        if (g_u32PatternToggle == 0)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x000000FF);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 1)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0xFF000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 2)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00FF0000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 3)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x0000FF00);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
        }
        else if (g_u32PatternToggle == 4)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x000000FF);
        }
        else if (g_u32PatternToggle == 5)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0xFF000000);
        }
        else if (g_u32PatternToggle == 6)
        {
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
            ELLSI_WRITE_DATA(ELLSI0, 0x00000000);
        }

        /* Enable TX FIFO threshold interrupt */
        ELLSI_EnableInt(ELLSI0, ELLSI_TXTH_INT_MASK);

        CLK_SysTickDelay(50000);

        g_u32PatternToggle++;
    }

    /* Close ELLSI0 */
    ELLSI_Close(ELLSI0);

    printf("Exit ELLSI sample code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

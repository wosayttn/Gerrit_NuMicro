/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an ELLSI demo for marquee display in PDMA mode.
 *           It needs to be used with WS2812 LED strip.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  5

volatile uint32_t g_au32RED_Marquee0[TEST_COUNT] = {0x000000FF, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee1[TEST_COUNT] = {0xFF000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee2[TEST_COUNT] = {0x00000000, 0x00FF0000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee3[TEST_COUNT] = {0x00000000, 0x00000000, 0x0000FF00, 0x00000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee4[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0x000000FF, 0x00000000};
volatile uint32_t g_au32RED_Marquee5[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0xFF000000, 0x00000000};
volatile uint32_t g_au32RED_Marquee6[TEST_COUNT] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
volatile uint32_t g_u32PatternToggle = 0;

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

    /* Enable PDMA0 peripheral clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

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
    /* Configure as PDMA mode, GRB output format, 6 pixels in a frame and idle output low */
    /* Set clock divider. ELLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 50000ns */
    ELLSI_Open(ELLSI0, ELLSI_MODE_PDMA, ELLSI_FORMAT_GRB, CLK_GetPCLK0Freq(), 1250, 400, 850, 50000, 6, ELLSI_IDLE_LOW);

    /* Enable reset command function */
    ELLSI_ENABLE_RESET_COMMAND(ELLSI0);
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
    printf("|      ELLSI Marquee Sample Code (PDMA Mode)      |\n");
    printf("+-------------------------------------------------+\n");
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init ELLSI */
    ELLSI_Init();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Reset PDMA0 module */
    SYS_ResetModule(PDMA0_RST);
    /* Lock protected registers */
    SYS_LockReg();

    /* Open Channel 0 */
    PDMA_Open(PDMA0, 1 << 0);
    /* Transfer type is single transfer */
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);

    g_u32PatternToggle = 0;

    while (g_u32PatternToggle < 7)
    {
        /*Disable ELLSI Tx before PDMA setting*/
        ELLSI_DISABLE(ELLSI0);

        if (g_u32PatternToggle == 0)
        {
            /* Set source address is g_au32RED_Marquee0, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee0, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 1)
        {
            /* Set source address is g_au32RED_Marquee1, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee1, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 2)
        {
            /* Set source address is g_au32RED_Marquee2, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee2, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 3)
        {
            /* Set source address is g_au32RED_Marquee3, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee3, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 4)
        {
            /* Set source address is g_au32RED_Marquee4, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee4, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 5)
        {
            /* Set source address is g_au32RED_Marquee5, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee5, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }
        else if (g_u32PatternToggle == 6)
        {
            /* Set source address is g_au32RED_Marquee6, destination address is &ELLSI0->DATA */
            PDMA_SetTransferAddr(PDMA0, 0, (uint32_t)g_au32RED_Marquee6, PDMA_SAR_INC, (uint32_t)&ELLSI0->DATA, PDMA_DAR_FIX);
        }

        /* Transfer count is TEST_COUNT, transfer width is 32 bits(one word) */
        PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_32, TEST_COUNT);

        /* Operation mode is basic mode, Request source is ELLSI0 */
        PDMA_SetTransferMode(PDMA0, 0, PDMA_ELLSI0_TX, FALSE, 0);

        /*Enable ELLSI Tx after PDMA setting*/
        ELLSI_ENABLE(ELLSI0);

        CLK_SysTickDelay(50000);

        g_u32PatternToggle++;
    }

    /* Close ELLSI0 */
    ELLSI_Close(ELLSI0);

    printf("Exit ELLSI sample code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

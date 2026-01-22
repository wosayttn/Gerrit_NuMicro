/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an ELLSI demo for marquee display in PDMA mode.
 *           It needs to be used with AP6112Y LED strip.
 *           The AP6112Y LED strip will be initialized through automatic hardware settings.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT      5
#define LED_STRIP_COUNT 3

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
    S_ELLSI_CONFIG_T sConfig;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init ELLSI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as PDMA mode, GRB output format and idle output low */
    /* Set clock divider. ELLSI clock rate = 72MHz */
    /* Set data transfer period. T_Period = 1250ns */
    /* Set duty period. T_T0H = 400ns; T_T1H = 850ns */
    /* Set reset command period. T_ResetPeriod = 250000ns */
    sConfig.u32ELLSIMode = ELLSI_MODE_PDMA;
    sConfig.u32OutputFormat = ELLSI_FORMAT_GRB;
    sConfig.sTimeInfo.u32BusClock = CLK_GetPCLK0Freq();
    sConfig.sTimeInfo.u32TransferTimeNsec = 1250;
    sConfig.sTimeInfo.u32T0HTimeNsec = 400;
    sConfig.sTimeInfo.u32T1HTimeNsec = 850;
    sConfig.sTimeInfo.u32ResetTimeNsec = 250000;
    sConfig.u32PCNT = 0;
    sConfig.u32IDOS = ELLSI_IDLE_LOW;
    ELLSI_OpenbyConfig(ELLSI0, &sConfig);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Config AP6112Y                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set command clock divider. ELLSI clock rate = 72MHz */
    /* Set enter command period. T_EnterHI = 40000ns; T_EnterLO = 10000ns */
    /* Set command get pulse period. T_GETHIP = 72000ns; T_GETHITOL = 15000ns; T_CMDTOP = 160000ns */
    /* Set feedback command period. T_FBHIP0 = 20000ns; T_FBLOP0 = 10000ns; T_FBHIP1 = 50000ns; T_FBLOP1 = 5000ns */
    /* Set feedback count period. T_FBCNT0 = 8000ns; T_FBC0TOL = 4000ns; T_FBCNT1 = 16000ns; T_FBC1TOL = 8000ns */
    /* Set feedback ID period. T_FBID0 = 8000ns; T_FBID0TOL = 4000ns; T_FBID1 = 32000ns; T_FBID1TOL = 15000ns */
    /* Set TH20 command period. T_TH20HI = 20000ns */
    sConfig.sTimeInfo.u32EnterHTimeNsec = 40000;
    sConfig.sTimeInfo.u32EnterLTimeNsec = 10000;
    sConfig.sTimeInfo.u32GetHTimeNsec = 72000;
    sConfig.sTimeInfo.u32GetHTolTimeNsec = 15000;
    sConfig.sTimeInfo.u32CmdTimeoutTimeNsec = 160000;
    sConfig.sTimeInfo.u32FBH0TimeNsec = 20000;
    sConfig.sTimeInfo.u32FBL0TimeNsec = 10000;
    sConfig.sTimeInfo.u32FBH1TimeNsec = 50000;
    sConfig.sTimeInfo.u32FBL1TimeNsec = 5000;
    sConfig.sTimeInfo.u32FBCNT0TimeNsec = 8000;
    sConfig.sTimeInfo.u32FBCNT0TolTimeNsec = 4000;
    sConfig.sTimeInfo.u32FBCNT1TimeNsec = 16000;
    sConfig.sTimeInfo.u32FBCNT1TolTimeNsec = 8000;
    sConfig.sTimeInfo.u32FBID0TimeNsec = 8000;
    sConfig.sTimeInfo.u32FBID0TolTimeNsec = 4000;
    sConfig.sTimeInfo.u32FBID1TimeNsec = 32000;
    sConfig.sTimeInfo.u32FBID1TolTimeNsec = 15000;
    sConfig.sTimeInfo.u32TH20TimeNsec = 20000;
    ELLSI_Config_Y_Cable(ELLSI0, &sConfig.sTimeInfo);

    /* Enable reset command function */
    ELLSI_ENABLE_RESET_COMMAND(ELLSI0);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_ELLSI_TH20SET_DATA_T sTh20SetData;
    uint32_t i;

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
    printf("Please connect %d AP6112Y LED strips.\n", LED_STRIP_COUNT);
    printf("The first to sixth LEDs will flash red in sequence.\n\n");

    /* Init ELLSI */
    ELLSI_Init();

    /* Set AP6112Y */
    /* Set command mode to AUTO SET mode */
    ELLSI_SET_CMD_MODE(ELLSI0, ELLSI_CMDMODE_AUTO_SET);
    /*Enable ELLSI Tx*/
    ELLSI_ENABLE(ELLSI0);

    /* Set Command Go Bit to start the command */
    ELLSI_SET_CMD_GO(ELLSI0);
    ELLSI_WaitCmdStsDone(ELLSI0, ELLSI_ASETDONE_MASK);

    /* Setting 24-bit TH20 Command data */
    sTh20SetData.BledDimming   = 0x1F;
    sTh20SetData.reserved0     = 0;
    sTh20SetData.SS0           = 0;
    sTh20SetData.SS1           = 0;
    sTh20SetData.RledDimming   = 0x1F;
    sTh20SetData.SS2           = 0;
    sTh20SetData.SS3           = 0;
    sTh20SetData.GledDimming   = 0x1F;
    sTh20SetData.reserved1     = 0;
    sTh20SetData.SS4           = 1;
    sTh20SetData.SS5           = 1;
    ELLSI_SetTH20Data(ELLSI0, &sTh20SetData);

    /* Force TH20 Command to send TH20 24-bit command */
    ELLSI_ENABLE_FORCE_TH20_COMMAND(ELLSI0);

    /* Set command mode to AUTO TH20 SET mode */
    ELLSI_SET_CMD_MODE(ELLSI0, ELLSI_CMDMODE_AUTO_TH20SET);
    /* Set Command Go Bit to start the command */
    ELLSI_SET_CMD_GO(ELLSI0);
    ELLSI_WaitCmdStsDone(ELLSI0, ELLSI_ATHDONE_MASK);

    /* Disable force TH20 Command */
    ELLSI_DISABLE_FORCE_TH20_COMMAND(ELLSI0);
    /*Disable ELLSI Tx*/
    ELLSI_DISABLE(ELLSI0);

    /* Set command mode to normal command mode */
    ELLSI_SET_CMD_MODE(ELLSI0, ELLSI_CMDMODE_NORMAL_CMD);

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

    /* Specify ID and send data */
    for (i = 1; i < (LED_STRIP_COUNT + 1); i++)
    {
        /* Configure 6 pixels in a frame */
        ELLSI_SET_FB_COUNT(ELLSI0, ID(i), 6);

        /* Specify ID */
        ELLSI_SET_NORMAL_CMD(ELLSI0, ELLSI_CMD_SPE_ID, ID(i));

        /* Set special command to send data */
        ELLSI_SET_SPECMD(ELLSI0, ELLSI_SPECMD_SEND_DATA);

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
            /* Set Command Go Bit to start the command */
            ELLSI_SET_CMD_GO(ELLSI0);

            ELLSI_WaitCmdStsDone(ELLSI0, ELLSI_CMDDONE_MASK);

            CLK_SysTickDelay(50000);

            g_u32PatternToggle++;
        }
    }

    /* Close ELLSI0 */
    ELLSI_Close(ELLSI0);

    printf("Exit ELLSI sample code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

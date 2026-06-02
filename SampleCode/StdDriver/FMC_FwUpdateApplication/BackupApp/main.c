/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Firmware udpate sample code (Bank0 App).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"

static volatile uint32_t s_u32ExecBank;
static volatile uint32_t s_u32GetSum;

uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for (u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }

    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

void WDT0_IRQHandler(void)
{
    WDT_RESET_COUNTER(WDT0);

    if (WDT_GET_TIMEOUT_INT_FLAG(WDT0) == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG(WDT0);
    }
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

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);

    /* Enable module clock */
    CLK_EnableModuleClock(ISP_MODULE);

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

int32_t  SelfTest(void)
{
    printf("\n  Self test pass. \n");
    return 0;
}

int32_t main(void)
{
    uint32_t i;
    int32_t i32RetCode;

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Init Debug UART for print message */
    InitDebugUart();

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    NVIC_EnableIRQ(WDT0_IRQn);

    /* Configure WDT settings and start WDT counting */
    WDT_Open(WDT0, WDT_TIMEOUT_2POW18, WDT_RESET_DELAY_18CLK, 1, 0);

    /* Enable WDT interrupt function */
    WDT_EnableInt(WDT0);

    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n Bank%d APP processing (Backup firmware)\n", s_u32ExecBank);

        i32RetCode = SelfTest();

        if (i32RetCode == 0)
        {
            for (i = 0; i < 1000; i++)
            {
                printf(" Firmware processing ... Count [%d]\r", i);
                s_u32GetSum = FuncCrc32(APP_BASE, APP_SIZE);
            }
        }
        else
        {
            printf("\n Enter power down ...\n");
            UART_WAIT_TX_EMPTY(DEBUG_PORT);
            CLK_PowerDown();
        }
    } while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

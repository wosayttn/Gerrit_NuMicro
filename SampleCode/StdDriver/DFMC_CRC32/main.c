/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use DFMC CRC32 ISP command to calculate
 *           the CRC32 checksum of DFMC data flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

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

    /* Enable modules clock */
    CLK_EnableModuleClock(DFMC_MODULE);

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
    uint32_t u32Data, u32ChkSum;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n");
    printf("+----------------------------------+\n");
    printf("|      DFMC CRC32 Sample Code      |\n");
    printf("+----------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    /* DFMC_ReadCID */
    u32Data = DFMC_ReadCID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* DFMC_ReadPID */
    u32Data = DFMC_ReadPID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Erase the first page of Dataflash */
    DFMC_ENABLE_UPDATE();

    DFMC_Erase(DFMC_DFLASH_BASE);

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_Erase failed!\n");
        goto lexit;
    }

    /* Write one word on Dataflash */
    DFMC_Write(DFMC_DFLASH_BASE, 0x55AABBCC);

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_Write failed!\n");
        goto lexit;
    }

    printf("\nDataflash address (0x%lX ~ 0x%lX) CRC32 checksum =>  ", DFMC_DFLASH_BASE, DFMC_DFLASH_END);
    /*
     *  Request DFMC hardware to run CRC32 calculation on Dataflash address DFMC_DFLASH_BASE ~ DFMC_DFLASH_END.
     *  Note that DFMC CRC32 checksum calculation area must page alignment.
     */
    u32ChkSum = DFMC_GetChkSum(DFMC_DFLASH_BASE, DFMC_DFLASH_SIZE);

    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating DFMC data flash CRC32 checksum !\n");
        goto lexit;
    }

    printf("0x%x\n", u32ChkSum);       /* print out Dataflash CRC32 check sum value */

    printf("\nDFMC CRC32 checksum test done.\n");
lexit:
    DFMC_Close();                      /* Disable DFMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

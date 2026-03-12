/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use DFMC Read-All-One ISP command to verify
 *           data flash pages are all 0xFFFFFFFF or not.
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
    uint32_t u32Data, u32RetCode;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|      DFMC_ReadAllOne Sample Code      |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    /* Read company ID. Should be 0xDA. */
    u32Data = DFMC_ReadCID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read product ID. */
    u32Data = DFMC_ReadPID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Enable Update. */
    DFMC_ENABLE_UPDATE();

    /* Erase Dataflash page 0. */
    if (DFMC_Erase(DFMC_DFLASH_BASE) != DFMC_OK)     /* Erase Dataflash page 0. */
    {
        printf("DFMC_Erase(DFMC_DFLASH_BASE) failed!\n");
        goto lexit;
    }

    printf("\n  Check Dataflash is all one after erase\n");
    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32RetCode = DFMC_CheckAllOne(DFMC_DFLASH_BASE, DFMC_DFLASH_PAGE_SIZE);

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if (u32RetCode == DREAD_ALLONE_YES)
        printf("    Check DREAD_ALLONE_YES success.\n");
    else
        printf("    Check DREAD_ALLONE_YES failed!\n");

    DFMC_Write(DFMC_DFLASH_BASE, 0x0);                      /* Program a 0 to Dataflash to make it not all 0xFFFFFFFF. */

    printf("  Check Dataflash is not all one after write\n");
    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32RetCode = DFMC_CheckAllOne(DFMC_DFLASH_BASE, DFMC_DFLASH_PAGE_SIZE);

    if (u32RetCode == DREAD_ALLONE_NOT)
        printf("    Check DREAD_ALLONE_NOT success.\n");   /* DFMC_CheckAllOne() READ_ALLONE_NOT passed on Dataflash page 0. */
    else
        printf("    Check DREAD_ALLONE_NOT failed!\n");    /* DFMC_CheckAllOne() READ_ALLONE_NOT failed on Dataflash page 0. */

    printf("\nDFMC Read-All-One test done.\n");

lexit:
    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

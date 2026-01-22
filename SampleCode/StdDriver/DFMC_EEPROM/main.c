/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to configure, initialize, read and write DFMC EEPROM.
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

int32_t EEPROM_DumpData(void)
{
    uint32_t u32Addr, u32Data;

    printf("\nDump DFMC EEPROM data:\n");

    for (u32Addr = DFMC_EEPROM_BASE; u32Addr < DFMC_EEPROM_END; u32Addr += 4)
    {
        u32Data = DFMC_EEPROM_Read(u32Addr);

        if (g_DFMC_i32ErrCode != DFMC_OK)
        {
            printf("DFMC_Read failed!\n");
            return g_DFMC_i32ErrCode;
        }

        if (u32Data != M32(u32Addr))
        {
            printf("Data compare failed at address 0x%08X (0x%08X != 0x%08X).\n", u32Addr, u32Data, M32(u32Addr));
            return -1;
        }

        if ((u32Addr % 32) == 0)
            printf("\n  [0x%08x] ", u32Addr);

        printf("0x%08x ", u32Data);
    }

    printf("\n\n");

    return 0;
}

int32_t main(void)
{
    uint32_t u32Addr, u32Data, u32Config;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n");
    printf("+-----------------------------------+\n");
    printf("|      DFMC EEPROM Sample Code      |\n");
    printf("+-----------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    FMC_Open();                         /* Enable FMC ISP function */
    u32Config = FMC_Read(FMC_USER_CONFIG_14);

    if (u32Config & BIT5)               /* Check if EEPROM is enabled */
    {
        printf("EEPROM is disabled. Please wait chip reset to enable it ...\n");
        FMC_ENABLE_CFG_UPDATE();        /* Enable User Configuration update */
        FMC_WriteConfig(FMC_USER_CONFIG_14, u32Config & ~BIT5);
        FMC_DISABLE_CFG_UPDATE();       /* Disable User Configuration update */
        SYS_ResetChip();                /* Perform chip reset to make new User Config take effect */
    }

    FMC_Close();                        /* Disable FMC ISP function */

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    /* Initialize DFMC EEPROM */
    if (DFMC_EEPROM_Init() != DFMC_OK)
    {
        printf("DFMC_EEPROM_Init failed!\n");
        goto lexit;
    }

    printf("DFMC EEPROM write pointer:    0x%X\n", (uint32_t)DFMC_EEPROM_GET_WRITE_PTR());
    printf("DFMC EEPROM page erase count: %d\n", (uint32_t)DFMC_EEPROM_GET_PAGE_ERASE_CNT());
    /* Dump EEPROM data */
    EEPROM_DumpData();

    /* Program EEPROM data */
    printf("Program new DFMC EEPROM data ...\n");

    for (u32Addr = DFMC_EEPROM_BASE; u32Addr < DFMC_EEPROM_END; u32Addr += 4)
    {
        if (M32(u32Addr) == 0xFFFFFFFF)     /* If original data is 0xFFFFFFFF, just write address as data */
            u32Data = u32Addr;
        else
            u32Data = M32(u32Addr) + 1;     /* Make data different from original data */

        DFMC_EEPROM_Write(u32Addr, u32Data);

        if (g_DFMC_i32ErrCode != DFMC_OK)
        {
            printf("DFMC_Write failed!\n");
            goto lexit;
        }
    }

    CACHE_Flush();
    /* Dump EEPROM data again */
    EEPROM_DumpData();
lexit:
    DFMC_Close();                       /* Disable DFMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */
    printf("End of DFMC EEPROM sample code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

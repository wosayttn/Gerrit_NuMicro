/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show DFMC read flash IDs, erase, read, and write functions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define TEST_PATTERN                0x5A5A5A5A

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

void run_crc32_checksum()
{
    uint32_t    u32Chksum;

    u32Chksum = DFMC_GetChkSum(DFMC_DFLASH_BASE, DFMC_DFLASH_SIZE);

    printf("  Dataflash CRC32 checksum .............. [0x%08x]\n", u32Chksum);
}

int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (DFMC_Write(u32Addr, u32Pattern) != DFMC_OK)          /* Program flash */
        {
            printf("DFMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }

    return 0;
}

int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr, u32Data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32Data = DFMC_Read(u32Addr);

        if (g_DFMC_i32ErrCode != 0)
        {
            printf("DFMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32Data != u32Pattern)
        {
            printf("\nDFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32Data, u32Pattern);
            return -1;
        }
    }

    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += DFMC_DFLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        if (DFMC_Erase(u32Addr) != DFMC_OK)            /* Erase page */
        {
            printf("DFMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + DFMC_DFLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page */
        if (fill_data_pattern(u32Addr, u32Addr + DFMC_DFLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + DFMC_DFLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        /* Erase page */
        if (DFMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + DFMC_DFLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }

    printf("\r    Flash Test Passed.           \n");
    return 0;
}

int32_t main(void)
{
    uint32_t u32Data;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|      DFMC Read/Write Sample Code      |\n");
    printf("+---------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Check CONFIG14 if DFMC EEPROM is disabled and all data flash is secure */
    FMC_Open();

    if (FMC_Read(FMC_USER_CONFIG_14) != 0xFFFFFFFF)
    {
        printf("CONFIG14 (0x%08X) is not 0xFFFFFFFF.\n", FMC_Read(FMC_USER_CONFIG_14));
        printf("This sample code needs to disable DFMC EEPROM and set all data flash secure.\n");
        printf("Erase CONFIG14 and wait reset chip done to take effect.\n");
        FMC_ENABLE_CFG_UPDATE();
        FMC_EraseConfig(FMC_USER_CONFIG_14);
        SYS_ResetChip();
    }

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    u32Data = DFMC_ReadCID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = DFMC_ReadPID();

    if (g_DFMC_i32ErrCode != DFMC_OK)
    {
        printf("DFMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    run_crc32_checksum();

    printf("\n\nDataflash test =>\n");
    DFMC_ENABLE_UPDATE();

    if (flash_test(DFMC_DFLASH_BASE, DFMC_DFLASH_END, TEST_PATTERN) < 0)
    {
        printf("\n\nDataflash test failed!\n");
        goto lexit;
    }

    DFMC_DISABLE_UPDATE();

    run_crc32_checksum();

lexit:
    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nEnd of DFMC Read/Write Sample Code.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

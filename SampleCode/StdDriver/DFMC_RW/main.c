/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 20/08/10 1:55p $
 * @brief    Show DFMC read flash IDs, erase, read, and write functions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_PATTERN                0x5A5A5A5A

int IsDebugFifoEmpty(void);

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

void run_crc32_checksum()
{
    uint32_t    chksum;

    chksum = DFMC_GetChkSum(DFMC_DFLASH_BASE, DFMC_DFLASH_SIZE);

    printf("  Dataflash CRC32 checksum .............. [0x%08x]\n", chksum);
}

int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (DFMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("DFMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}

int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = DFMC_Read(u32Addr);

        if (g_DFMC_i32ErrCode != 0)
        {
            printf("DFMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32data != u32Pattern)
        {
            printf("\nDFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }
    return 0;
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += DFMC_DFLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        if (DFMC_Erase(u32Addr) != 0)            /* Erase page */
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
    printf("\r    Flash Test Passed.          \n");
    return 0;
}

int main()
{
    uint32_t    u32Data;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|           M471 DFMC Sample Code        |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    u32Data = DFMC_ReadCID();
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }	
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = DFMC_ReadPID();
    if (g_DFMC_i32ErrCode != 0)
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

    printf("\nDFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

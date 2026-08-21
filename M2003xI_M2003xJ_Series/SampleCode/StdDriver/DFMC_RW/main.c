/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show DFMC read flash IDs, erase, read, and write functions.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved..
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

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);

    /* Set core clock to 40MHz */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Enable DFMC0 module clock */
    CLK_EnableModuleClock(DFMC0_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void run_crc32_checksum()
{
    uint32_t    chksum;

    chksum = DFMC_GetChkSum(DFMC_FLASH_BASE, DFMC_FLASH_SIZE);

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

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += DFMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        /* Erase page */
        if (DFMC_Erase(u32Addr) != 0)            /* Erase page */
        {
            printf("DFMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }
        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + DFMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page */
        if (fill_data_pattern(u32Addr, u32Addr + DFMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("Failed to write page 0x%x!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + DFMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");
            return -1;
        }

        /* Erase page */
        if (DFMC_Erase(u32Addr) != 0)
        {
            printf("DFMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }
        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + DFMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
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
    printf("|          M2003J DFMC Sample Code        |\n");
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

    run_crc32_checksum();

    printf("\n\nDataflash test =>\n");
    DFMC_ENABLE_DF_UPDATE();
    if (flash_test(DFMC_FLASH_BASE, DFMC_FLASH_END, TEST_PATTERN) < 0)
    {
        printf("\n\nDataflash test failed!\n");
        goto lexit;
    }
    DFMC_DISABLE_DF_UPDATE();

    run_crc32_checksum();

lexit:

    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nDFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

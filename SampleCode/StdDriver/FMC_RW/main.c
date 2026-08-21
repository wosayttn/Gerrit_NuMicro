/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 20/08/10 3:06p $
 * @brief    Show FMC read flash IDs, erase, read, and write functions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             0x8000
#define TEST_PATTERN                0x5A5A5A5A

uint32_t APROM_TEST_END  = 0x00010000UL;        /* 64KB */
uint32_t LDROM_TEST_SIZE = 0x00000800UL;        /*  2KB */
uint32_t LDROM_TEST_END  = 0x00100800UL;

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

    chksum = FMC_GetChkSum(FMC_APROM_BASE, APROM_TEST_END - FMC_APROM_BASE);

    if (chksum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        return;                    /* failed */
    }
    printf("  APROM CRC32 checksum .................. [0x%08x]\n", chksum);

    chksum = FMC_GetChkSum(FMC_LDROM_BASE, LDROM_TEST_SIZE);

    if (chksum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        return;                    /* failed */
    }
    printf("  LDROM CRC32 checksum .................. [0x%08x]\n", chksum);
}

int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;                  /* flash address */

    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (FMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("FMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}

int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* flash address */
    uint32_t    u32data;               /* flash data    */

    /* Verify if each data word from flash u32StartAddr to u32EndAddr be u32Pattern.  */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);   /* Read a flash word from address u32Addr. */

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32data != u32Pattern)     /* Verify if data matched. */
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;                 /* data verify failed */
        }
    }
    return 0;                          /* data verify OK */
}

int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* flash address */

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);       /* information message */

        if (FMC_Erase(u32Addr) != 0)            /* Erase page */
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }

        /* Write test pattern to fill the whole page. */
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) != 0)
            return -1;

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");                      /* error message */
            return -1;                 /* Program verify failed */
        }

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF after erased. */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }
    }
    printf("\r    Flash Test Passed.          \n");                  /* information message */
    return 0;      /* flash test passed */
}

int main()
{
    uint32_t    i, u32Data;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|           M471 FMC Sample Code         |\n");
    printf("+----------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");
    else
    {
        printf("[LDROM]\n");
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;
    }

    u32Data = FMC_ReadCID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)
    {
        u32Data = FMC_ReadUID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)
    {
        u32Data = FMC_ReadUCID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }
    printf("  User Config 2 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+8));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+8) failed!\n");
        goto lexit;
    }
    run_crc32_checksum();

    printf("\n\nLDROM test =>\n");     /* information message */
    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */
    if (flash_test(FMC_LDROM_BASE, LDROM_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");        /* error message */
        goto lexit;                    /* LDROM test failed. Program aborted. */
    }
    FMC_DISABLE_LD_UPDATE();           /* Disable LDROM update. */

    printf("\n\nAPROM test =>\n");     /* information message */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM update. */
    if (flash_test(APROM_TEST_BASE, APROM_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");
        goto lexit;
    }
    FMC_DISABLE_AP_UPDATE();

    run_crc32_checksum();

lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

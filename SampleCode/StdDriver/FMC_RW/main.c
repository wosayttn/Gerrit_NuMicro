/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Show FMC read Flash IDs, erase, read, and write functions
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

#define APROM_TEST_BASE             (FMC_APROM_BASE + 0x20000)                      /* APROM test start address */
#define APROM_TEST_END              (APROM_TEST_BASE + FMC_FLASH_PAGE_SIZE * 8)     /* APROM test end address   */
#define TEST_PATTERN                0x5A5A5A5A                                      /* Test pattern             */

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

int32_t FillDataPattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;

    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 8)
    {
        if (FMC_Write8Bytes(u32Addr, u32Pattern, u32Pattern) != 0)          /* Program flash */
        {
            printf("FMC write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }

    return 0;
}

int32_t  VerifyData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;
    uint32_t    u32data;

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
            /* Data verify failed */
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;
        }
    }

    return 0;                          /* Data verify OK */
}

int32_t  FlashTest(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);

        if (FMC_Erase(u32Addr) != 0)            /* Erase page */
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            /* Erase verify failed */
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }

        /* Write test pattern to fill the whole page. */
        if (FillDataPattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) != 0)
            return -1;

        /* Verify if page contents are all equal to test pattern */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            /* Program verify failed */
            printf("\nData verify failed!\n");
            return -1;
        }

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF after erased. */
        if (VerifyData(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            /* Erase verify failed */
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);
            return -1;
        }
    }

    printf("\r    Flash Test Passed.           \n");
    return 0;      /* Flash test passed */
}

int32_t main(void)
{
    uint32_t    i, u32Data;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    printf("+--------------------------------------+\n");
    printf("|      FMC Read/Write Sample Code      |\n");
    printf("+--------------------------------------+\n");
    printf("FMC->ISPCTL: 0x%08X\n", FMC->ISPCTL);
    SYS_UnlockReg();    /* Unlock protected registers */
    FMC_Open();         /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Get company ID, should be 0xDA. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Get product ID. */

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }

    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    for (i = 0; i < 3; i++)            /* Get 96-bits UID. */
    {
        u32Data = FMC_ReadUID(i);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUID %d failed!\n", i);
            goto lexit;
        }

        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);
    }

    for (i = 0; i < 4; i++)            /* Get 4 words UCID. */
    {
        u32Data = FMC_ReadUCID(i);

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }

        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);
    }

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_0));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_USER_CONFIG_1));

    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    printf("\n\nData flash test =>\n");
    FMC_ENABLE_DF_UPDATE();            /* Enable data flash update. */

    /* Execute flash program/verify test on data flash. */
    if (FlashTest(FMC_DATA_FLASH_BASE, FMC_DATA_FLASH_END, TEST_PATTERN) < 0)
    {
        /* Data flash test failed. Program aborted. */
        printf("\tData flash test failed!\n");
        goto lexit;
    }

    FMC_DISABLE_DF_UPDATE();           /* Disable data flash update. */

    printf("\n\nLDROM test =>\n");
    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    /* Execute flash program/verify test on LDROM. */
    if (FlashTest(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        /* LDROM test failed. Program aborted. */
        printf("\tLDROM test failed!\n");
        goto lexit;
    }

    FMC_DISABLE_LD_UPDATE();           /* Disable LDROM update. */

    printf("\n\nAPROM test =>\n");
    FMC_ENABLE_AP_UPDATE();            /* Enable APROM update. */

    /* Execute flash program/verify test on APROM. */
    if (FlashTest(APROM_TEST_BASE, APROM_TEST_BASE + FMC_FLASH_PAGE_SIZE * 8, TEST_PATTERN) < 0)
    {
        /* APROM test failed. Program aborted. */
        printf("\tAPROM test failed!\n");
        goto lexit;
    }

    FMC_DISABLE_AP_UPDATE();           /* Disable APROM update. */

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    printf("\nEnd of FMC Read/Write Sample Code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

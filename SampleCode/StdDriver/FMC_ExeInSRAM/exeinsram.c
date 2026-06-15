/****************************************************************************//**
 * @file     exeinsram.c
 * @version  V0.10
 * @brief    Implement the code to execute in SRAM to program embedded flash.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define APROM_TEST_BASE     (FMC_APROM_BASE + 0x10000)
#define TEST_PATTERN        0x5A5A5A5A

static volatile uint32_t g_u32Ticks = 0;

void SysTick_Handler_SRAM(void)
{
    g_u32Ticks++;

    if ((g_u32Ticks % 1000) == 0)
    {
        printf("[%s@0x%08X] Time elapsed: %d ms\n", __func__, __PC(), g_u32Ticks);
    }
}

int32_t FlashAccess_OnSRAM(void)
{
    uint32_t u32Data0, u32Data1, u32ReadData0, u32ReadData1;
    uint32_t u32Addr, u32Cnt;

    printf("[%s] Execution address is 0x%08X\n", __func__, __PC());
    SYS_UnlockReg();
    FMC_Open();
    FMC_ENABLE_AP_UPDATE();

    /* The APROM address for erase/write/read demo */
    u32Addr = APROM_TEST_BASE;
    FMC_Erase(u32Addr); /* Erase page */

    for (u32Cnt = 0; u32Cnt < FMC_FLASH_PAGE_SIZE; u32Cnt += 8)
    {
        /* Write data */
        u32Data0 = u32Cnt + 0x12345678;
        u32Data1 = u32Cnt + 0x12345678 + 4;
        FMC_Write8Bytes(u32Addr + u32Cnt, u32Data0, u32Data1);

        if (((u32Cnt / 4) & 0xf) == 0)
            printf(".");

        /* Read data */
        if (FMC_Read8Bytes(u32Addr + u32Cnt, &u32ReadData0, &u32ReadData1) != FMC_OK)
        {
            printf("Read failed!\n");
            return -1;
        }

        if ((u32Data0 != u32ReadData0) || (u32Data1 != u32ReadData1))
        {
            printf("Check data failed!\n");
            return -1;
        }
    }

    printf("\nISP function demo done.\n");
    FMC_DISABLE_AP_UPDATE();

    return 0;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

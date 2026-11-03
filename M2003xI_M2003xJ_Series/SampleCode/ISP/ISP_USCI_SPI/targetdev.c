/***************************************************************************//**
 * @file     targetdev.c
 * @version  V1.00
 * @brief    ISP support function source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"

uint32_t GetApromSize()
{
    uint32_t size = 0x4000, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if (result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while (1);
}

void GetDataFlashInfo(uint32_t *addr, uint32_t *size)
{
    *addr = DFMC_FLASH_BASE;
    *size = DFMC_FLASH_SIZE;
}

/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/
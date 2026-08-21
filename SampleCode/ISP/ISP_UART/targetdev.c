/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
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
            return (size);
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
    *addr = DFMC_DFLASH_BASE;
    *size = DFMC_DFLASH_SIZE;
}


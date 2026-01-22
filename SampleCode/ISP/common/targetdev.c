/***************************************************************************//**
 * @file     targetdev.c
 * @version  V1.00
 * @brief    ISP support function source file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "targetdev.h"
#include "isp_user.h"

void UserMemCopy(uint8_t pu8Dest[], uint8_t pu8Src[], uint32_t u32Size)
{
    uint32_t i = 0ul;

    while (u32Size--)
    {
        pu8Dest[i] = pu8Src[i];
        __DSB();
        i++;
    }
}

/* Supports maximum 1MB (APROM) */
uint32_t GetApromSize(void)
{
    /* The smallest of APROM u32Size is FMC_FLASH_PAGE_SIZE. */
    uint32_t u32Size = FMC_FLASH_PAGE_SIZE, u32Data;
    int result;

    do
    {
        result = FMC_Read_User(FMC_APROM_BASE + u32Size, &u32Data);

        if (result < 0)
        {
            return u32Size;
        }
        else
        {
            u32Size *= 2;
        }
    } while (1);
}

void GetDataFlashInfo(uint32_t *pu32Addr, uint32_t *pu32Size)
{
    *pu32Addr = FMC_DATA_FLASH_BASE;
    *pu32Size = FMC_DATA_FLASH_SIZE;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

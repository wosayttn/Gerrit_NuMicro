/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"


/* Supports maximum 500K (APROM) */
uint32_t GetApromSize()
{
    /* The smallest of APROM size is 2K. */
    uint32_t size = 0x800, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if(result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while(1);
}

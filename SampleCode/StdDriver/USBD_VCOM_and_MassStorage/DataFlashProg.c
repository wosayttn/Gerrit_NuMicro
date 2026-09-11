/**************************************************************************//**
 * @file     DataFlashProg.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Data Flash Access API
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "DataFlashProg.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
static uint32_t s_au32SectorBuf[BUFFER_PAGE_SIZE / 4];

void DataFlashRead(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level read function of USB Mass Storage */
    uint32_t *pu32;
    uint32_t i;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    pu32 = (uint32_t *)u32Buffer;

    for(i = 0; i < u32Size / 4; i++)
        pu32[i] = M32(u32Addr + i * 4);
}

void DataFlashWrite(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level write function of USB Mass Storage */
    uint32_t i;
    uint32_t *pu32;

    /* Before using RMC function, it should unlock system register first. */
    SYS_UnlockReg();

    /* Source buffer */
    pu32 = (uint32_t *)u32Buffer;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    for(i = 0; i < u32Size / 4; i++)
        RMC_Write(u32Addr + i * 4, pu32[i]);

    SYS_LockReg();
}


/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
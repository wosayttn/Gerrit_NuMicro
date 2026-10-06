/***************************************************************************//**
 * @file     fmc_user.c
 * @brief    M471 series FMC driver source file
 * @version  2.0.0
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    unsigned int u32Addr, Reg;
    FMC_T *pFMC;

    if((addr_start & DFMC_DFLASH_BASE) != 0)
    {
        pFMC = (FMC_T *)DFMC;
    }
    else
    {
        pFMC = FMC;
    }

    for (u32Addr = addr_start; u32Addr < addr_end; data++)
    {
        pFMC->ISPCMD = u32Cmd;
        pFMC->ISPADDR = u32Addr;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            pFMC->ISPDAT = *data;
        }

        pFMC->ISPTRG = 0x1;
        __ISB();

        while (pFMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

        Reg = pFMC->ISPCTL;

        if (Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            pFMC->ISPCTL = Reg;
            return (-1);
        }

        if (u32Cmd == FMC_ISPCMD_READ)
        {
            *data = pFMC->ISPDAT;
        }

        if (u32Cmd == FMC_ISPCMD_PAGE_ERASE)
        {
            if(pFMC == FMC)
                u32Addr += FMC_FLASH_PAGE_SIZE;
            else
                u32Addr += DFMC_DFLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }

    return (0);
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    unsigned int u32Size = CONFIG_SIZE;
    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, Config0, Config0 + 8, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);

    if (res)
    {
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }

    FMC_DISABLE_CFG_UPDATE();
}

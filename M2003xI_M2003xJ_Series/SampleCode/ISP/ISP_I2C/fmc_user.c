/***************************************************************************//**
 * @file     fmc_user.c
 * @version  2.0.0
 * @brief    M2003J series FMC driver source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "fmc_user.h"

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    unsigned int u32Addr, Reg;
    FMC_T * D_FMC = FMC;

    if(addr_start >= DFMC_FLASH_BASE)
    {
        //Processing Data_Flash
        D_FMC = (FMC_T *)DFMC;
        D_FMC->ISPCTL |= (DFMC_ISPCTL_ISPEN_Msk | DFMC_ISPCTL_DATAEN_Msk);
    }

    if((u32Cmd == FMC_ISPCMD_PROGRAM) && (addr_start < DFMC_FLASH_BASE))
    {
        for (u32Addr = addr_start; u32Addr < addr_end; data += 2)
        {
            FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_64;
            FMC->ISPADDR = u32Addr;
            FMC->MPDAT0  = *data;
            FMC->MPDAT1  = *(data+1);
            FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

            while(FMC->ISPSTS & FMC_ISPSTS_ISPBUSY_Msk);

            if(FMC->ISPSTS & FMC_ISPSTS_ISPFF_Msk)
            {
                FMC->ISPSTS |= FMC_ISPSTS_ISPFF_Msk;
                return (-1);
            }

            u32Addr += 8;
        }
    }
    else
    {
    for (u32Addr = addr_start; u32Addr < addr_end; data++)
    {
            D_FMC->ISPCMD = u32Cmd;
            D_FMC->ISPADDR = u32Addr;

        if (u32Cmd == FMC_ISPCMD_PROGRAM)
        {
                D_FMC->ISPDAT = *data;
        }

            D_FMC->ISPTRG = 0x1;
        __ISB();

            while (D_FMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

            Reg = D_FMC->ISPCTL;

        if (Reg & FMC_ISPCTL_ISPFF_Msk)
        {
                D_FMC->ISPCTL = Reg;
                return (-1);
        }

        if (u32Cmd == FMC_ISPCMD_READ)
        {
                *data = D_FMC->ISPDAT;
        }

        if (u32Cmd == FMC_ISPCMD_PAGE_ERASE)
        {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }
    }

    if(addr_start >= DFMC_FLASH_BASE)
    {
        D_FMC->ISPCTL &= ~(DFMC_ISPCTL_ISPEN_Msk | DFMC_ISPCTL_DATAEN_Msk);
    }

    return 0;
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

int32_t FMC_SetVectorAddr(uint32_t u32PageAddr)
{
    FMC->ISPCMD = FMC_ISPCMD_VECMAP;  /* Set ISP Command Code */
    FMC->ISPADDR = u32PageAddr;       /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    FMC->ISPTRG = 0x1u;               /* Trigger to start ISP procedure */

    while (FMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

    return 0;
}
/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/
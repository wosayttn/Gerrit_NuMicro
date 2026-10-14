/***************************************************************************//**
 * @file     rmc_user.c
 * @brief    M2L31 series RMC driver source file
 * @version  2.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "rmc_user.h"

int RMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    uint32_t u32Addr, Reg;

    for (u32Addr = addr_start; u32Addr < addr_end;)
    {
        if (u32Cmd == RMC_ISPCMD_PROGRAM)
        {
            RMC->ISPCMD = RMC_ISPCMD_CLEAR_DATA_BUFFER;
            RMC->ISPADDR = 0x00000000;
            RMC->ISPTRG = 0x1;
            __ISB();

            while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

            Reg = RMC->ISPCTL;

            if (Reg & RMC_ISPCTL_ISPFF_Msk)
            {
                RMC->ISPCTL = Reg;
                return -1;
            }

            RMC->ISPCMD = RMC_ISPCMD_LOAD_DATA_BUFFER;
            RMC->ISPADDR = u32Addr;
            RMC->ISPDAT = *data;
            RMC->ISPTRG = 0x1;
            __ISB();

            while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

            Reg = RMC->ISPCTL;

            if (Reg & RMC_ISPCTL_ISPFF_Msk)
            {
                RMC->ISPCTL = Reg;
                return -1;
            }
        }

        RMC->ISPCMD = u32Cmd;
        RMC->ISPADDR = u32Addr;

        if (u32Cmd == RMC_ISPCMD_PROGRAM)
        {
            RMC->ISPDAT = *data;
        }

        RMC->ISPTRG = 0x1;
        __ISB();

        while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

        Reg = RMC->ISPCTL;

        if (Reg & RMC_ISPCTL_ISPFF_Msk)
        {
            RMC->ISPCTL = Reg;
            return -1;
        }

        if (u32Cmd == RMC_ISPCMD_READ)
        {
            *data = RMC->ISPDAT;
        }
        u32Addr += 4;
        data++;
    }

    return 0;
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    uint32_t u32Size = 44;  //M2L31 support CONFIG10
    RMC_ENABLE_CFG_UPDATE();
    RMC_Proc(RMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);
    if (res)
        RMC_Proc(RMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    RMC_DISABLE_CFG_UPDATE();
}

int32_t RMC_SetVectorAddr(uint32_t u32PageAddr)
{
    RMC->ISPCMD = RMC_ISPCMD_VECMAP;  /* Set ISP Command Code */
    RMC->ISPADDR = u32PageAddr;       /* The address of specified page which will be map to address 0x0. It must be page alignment. */
    RMC->ISPTRG = 0x1u;               /* Trigger to start ISP procedure */

    while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

    return -1;
}

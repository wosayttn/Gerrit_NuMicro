/***************************************************************************//**
 * @file     rmc_user.c
 * @brief    M2L31 series RMC driver source file
 * @version  2.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
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

int ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)    // Read data from flash
{
    return RMC_Proc(RMC_ISPCMD_READ, addr_start, addr_end, data);
}

int WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)  // Write data into flash
{
    return RMC_Proc(RMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
}

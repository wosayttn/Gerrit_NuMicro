/***************************************************************************//**
 * @file     RMC_user.c
 * @brief    M2L31 series RMC driver source file
 * @version  2.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "rmc_user.h"

int RMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data);

int RMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    uint32_t u32Addr, Reg;

    for (u32Addr = addr_start; u32Addr < addr_end;)
    {
        if (u32Cmd == RMC_ISPCMD_PROGRAM || u32Cmd == CMD_ERASE_ALL)
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
            
            if (u32Cmd == CMD_ERASE_ALL)
                RMC->ISPDAT = 0xFFFFFFFF;
            else
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
        if (u32Cmd == CMD_ERASE_ALL)
            RMC->ISPCMD = RMC_ISPCMD_PROGRAM;
        else
            RMC->ISPCMD = u32Cmd;
        
        RMC->ISPADDR = u32Addr;

        if (u32Cmd == RMC_ISPCMD_PROGRAM)
        {
            RMC->ISPDAT = *data;
        }
        else if (u32Cmd == CMD_ERASE_ALL)
        {
            RMC->ISPDAT = 0xFFFFFFFF;
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

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in RMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int RMC_Write_User(uint32_t u32Addr, uint32_t u32Data)
{
    return RMC_Proc(RMC_ISPCMD_PROGRAM, u32Addr, u32Addr + 4, &u32Data);
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function. User can check the status of
 *              Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
int RMC_Read_User(uint32_t u32Addr, uint32_t *data)
{
    return RMC_Proc(RMC_ISPCMD_READ, u32Addr, u32Addr + 4, data);
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function. User can check the status of
 *             Register Write-Protection Function with DrvSYS_IsProtectedRegLocked().
 */
uint32_t RMC_ReadDID(void)
{
    RMC->ISPCMD = RMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    RMC->ISPADDR = 0x00u;                       /* Must keep 0x4 when read PID */
    RMC->ISPTRG = RMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
    /* To make sure ISP/CPU be Synchronized */
    while (RMC->ISPTRG & 0x1) ;  /* Wait for ISP command done. */

    return RMC->ISPDAT;
}

void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)    // Read data from flash
{
    RMC_Proc(RMC_ISPCMD_READ, addr_start, addr_end, data);
    return;
}

void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)  // Write data into flash
{
    RMC_Proc(RMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
    return;
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    uint32_t u32Size = 16;
    RMC_ENABLE_CFG_UPDATE();
    RMC_Proc(RMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);
    if (res)
    {    
        RMC_Proc(RMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }
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


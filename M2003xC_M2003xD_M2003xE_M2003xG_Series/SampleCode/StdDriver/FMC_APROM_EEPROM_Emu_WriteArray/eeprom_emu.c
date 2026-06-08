/****************************************************************************//**
 * @file     eeprom_emu.c
 * @version  V0.10
 * @brief    EEPROM emulation driver using APROM via FMC ISP interface.
 *           This module provides safe read/write/erase APIs to emulate
 *           EEPROM behavior on embedded flash (APROM), including
 *           page-based read-modify-write handling and boundary protection

 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"   
#include "eeprom_emu.h"

static uint8_t g_pageBuf[FMC_FLASH_PAGE_SIZE];

static uint8_t IsOverflow(uint32_t addr, uint32_t len)
{
    return ((addr + len) < addr) ? 1U : 0U;
}

flash_status_t EEPROM_Emu_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC */
    FMC_Open();

    SYS_LockReg();

    return FLASH_STATUS_OK;
}


flash_status_t EEPROM_Emu_ReadByte(uint32_t addr, uint8_t *pData)
{
    if (pData == NULL)
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if (addr > APROM_END_ADDR)
    {
        return FLASH_STATUS_OUT_OF_RANGE;
    }

    *pData = *((volatile uint8_t *)addr);

    return FLASH_STATUS_OK;
}

flash_status_t EEPROM_Emu_ReadArray(uint32_t addr, uint8_t *pData, uint32_t len)
{
    uint32_t i;

    if ((pData == NULL) || (len == 0U))
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if (IsOverflow(addr, len) != 0U)
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if ((addr + len - 1U) > APROM_END_ADDR)
    {
        return FLASH_STATUS_OUT_OF_RANGE;
    }

    for (i = 0U; i < len; i++)
    {
        pData[i] = *((volatile uint8_t *)(addr + i));
    }

    return FLASH_STATUS_OK;
}


flash_status_t EEPROM_Emu_WriteArray(uint32_t addr, const uint8_t *pData, uint32_t len)
{
    uint32_t page_base;
    uint32_t offset;
    uint32_t remain;
    uint32_t write_addr;
    uint32_t copy_len;
    uint32_t i, j;

    if ((pData == NULL) || (len == 0U))
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if (IsOverflow(addr, len) != 0U)
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if ((addr + len - 1U) > APROM_END_ADDR)
    {
        return FLASH_STATUS_OUT_OF_RANGE;
    }

    remain = len;
    write_addr = addr;

    /* Unlock protected registers */
    SYS_UnlockReg();

    FMC_ENABLE_AP_UPDATE();

    while (remain > 0U)
    {
        page_base = write_addr & ~(FMC_FLASH_PAGE_SIZE - 1U);
        offset    = write_addr - page_base;

        /* Read page */
        for (i = 0U; i < FMC_FLASH_PAGE_SIZE; i += 4U)
        {
            *(uint32_t *)&g_pageBuf[i] = FMC_Read(page_base + i);
        }

        copy_len = FMC_FLASH_PAGE_SIZE - offset;

        if (copy_len > remain)
        {
            copy_len = remain;
        }

        /* Modify */
        for (j = 0U; j < copy_len; j++)
        {
            g_pageBuf[offset + j] = pData[j];
        }

        /* Erase */
        if (FMC_Erase(page_base) != 0)
        {
            goto ERROR_EXIT;
        }

        /* Program */
        for (i = 0U; i < FMC_FLASH_PAGE_SIZE; i += 4U)
        {
            if (FMC_Write(page_base + i, *(uint32_t *)&g_pageBuf[i]) != 0)
            {
                goto ERROR_EXIT;
            }
        }

        write_addr += copy_len;
        pData      += copy_len;
        remain     -= copy_len;
    }

    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    return FLASH_STATUS_OK;

ERROR_EXIT:
    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    return FLASH_STATUS_HW_ERROR;
}

flash_status_t EEPROM_Emu_WriteByte(uint32_t addr, uint8_t data)
{
    return EEPROM_Emu_WriteArray(addr, &data, 1U);
}

flash_status_t EEPROM_Emu_Erase(uint32_t addr, uint32_t len)
{
    uint32_t page_base;
    uint32_t offset;
    uint32_t remain;
    uint32_t erase_addr;
    uint32_t erase_len;
    uint32_t i, j;

    if (len == 0U)
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if (IsOverflow(addr, len) != 0U)
    {
        return FLASH_STATUS_INVALID_PARAM;
    }

    if ((addr + len - 1U) > APROM_END_ADDR)
    {
        return FLASH_STATUS_OUT_OF_RANGE;
    }

    remain     = len;
    erase_addr = addr;

    SYS_UnlockReg();
    FMC_ENABLE_AP_UPDATE();

    while (remain > 0U)
    {
        page_base = erase_addr & ~(FMC_FLASH_PAGE_SIZE - 1U);
        offset    = erase_addr - page_base;

        /* -------- Read full page -------- */
        for (i = 0U; i < FMC_FLASH_PAGE_SIZE; i += 4U)
        {
            *(uint32_t *)&g_pageBuf[i] = FMC_Read(page_base + i);
        }

        /* -------- Determine partial erase range -------- */
        erase_len = FMC_FLASH_PAGE_SIZE - offset;

        if (erase_len > remain)
        {
            erase_len = remain;
        }

        /* -------- Clear only requested region (set 0xFF) -------- */
        for (j = 0U; j < erase_len; j++)
        {
            g_pageBuf[offset + j] = 0xFFU;
        }

        /* -------- Erase whole page -------- */
        if (FMC_Erase(page_base) != 0)
        {
            goto ERROR_EXIT;
        }

        /* -------- Write back preserved data -------- */
        for (i = 0U; i < FMC_FLASH_PAGE_SIZE; i += 4U)
        {
            if (FMC_Write(page_base + i, *(uint32_t *)&g_pageBuf[i]) != 0)
            {
                goto ERROR_EXIT;
            }
        }

        erase_addr += erase_len;
        remain     -= erase_len;
    }

    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    return FLASH_STATUS_OK;

ERROR_EXIT:
    FMC_DISABLE_AP_UPDATE();

    SYS_LockReg();

    return FLASH_STATUS_HW_ERROR;

}


flash_status_t EEPROM_Emu_DeInit(void)
{
    SYS_UnlockReg();

    /* Close FMC to release HW resource */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    return FLASH_STATUS_OK;
}


/*** (C) COPYRIGHT 2026 Nuvoton Technology Corp. ***/

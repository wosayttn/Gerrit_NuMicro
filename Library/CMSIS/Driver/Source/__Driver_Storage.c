/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**************************************************************************//**
 * @file     Driver_Storage.c
 * @version  V1.00
 * @brief    CMSIS Storage driver for Nuvoton M55M1
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*! \page Driver_Storage Storage

# Revision History

- Version 1.0
  - Initial release

# Requirements

This driver requires the M55M1 BSP.
The driver instance is mapped to hardware as shown in the table below:

  CMSIS Driver Instance | M55M1 Hardware Resource
  :---------------------|:-----------------------
  Driver_Storage0       | FMC

*/

#include <string.h>
#include "Driver_Storage.h"
#include "NuMicro.h"

#ifndef DEF_ASYNC_MODE
    #define DEF_ASYNC_MODE  0
#endif

#define ARM_STORAGE_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_STORAGE_API_VERSION,
    ARM_STORAGE_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_STORAGE_CAPABILITIES DriverCapabilities =
{
    DEF_ASYNC_MODE, /* Asynchronous Mode */
    0,              /* Supports EraseAll operation */
    0               /* Reserved */
};

#ifndef DRIVER_STORAGE_NUM
    #define DRIVER_STORAGE_NUM        0       /* Default driver number */
#endif

#ifdef _DEBUG
    #define debug_printf(fmt, ...)  printf("[%s] " fmt, __func__, ##__VA_ARGS__)
#else
    #define debug_printf(...)
#endif

extern uint32_t Image$$ER_ROM$$Base, Image$$ER_ROM$$Length;

#define DEF_STORAGE_BASE    (uint32_t)(((uint32_t)(&Image$$ER_ROM$$Length) & (FMC_FLASH_PAGE_SIZE - 1)) ? ((((uint32_t)(&Image$$ER_ROM$$Length) / FMC_FLASH_PAGE_SIZE) + 1) * FMC_FLASH_PAGE_SIZE) : ((uint32_t)(&Image$$ER_ROM$$Length) / FMC_FLASH_PAGE_SIZE) * FMC_FLASH_PAGE_SIZE)
#define DEF_STORAGE_END     (uint32_t)FMC_APROM_SIZE

static const ARM_STORAGE_BLOCK_ATTRIBUTES StorageAttr =
{
    .erasable           = 1,
    .programmable       = 1,
    .executable         = 1,
    .protectable        = 1,
    .erase_unit         = FMC_FLASH_PAGE_SIZE,
    .protection_unit    = FMC_APWPROT_BLOCK_SIZE
};

/* Driver Info */
static const ARM_STORAGE_INFO StorageInfo =
{
    .total_storage          = FMC_APROM_SIZE,                       /* Total Storage Size */
    .program_unit           = 0x4,                                  /* Program Unit */
    .optimal_program_unit   = 0x4,                                  /* Optimal Program Unit */
    .program_cycles         = ARM_STORAGE_PROGRAM_CYCLES_INFINITE,  /* Program Cycles */
    .programmability        = ARM_STORAGE_PROGRAMMABILITY_ERASABLE,
    .retention_level        = ARM_RETENTION_NVM
};

/* Driver Status */
static volatile ARM_STORAGE_STATUS s_StorageStatus;
/* Driver Operation */
static volatile ARM_STORAGE_OPERATION s_StorageOper;

static ARM_Storage_Callback_t s_pfnStorageCallback = NULL;

//
// Functions
//
void ISP_IRQHandler(void)
{
    if (FMC_GET_FAIL_FLAG())
    {
        s_StorageStatus.error = 1;
        FMC_CLR_FAIL_FLAG();
    }

    FMC->ISPSTS = FMC_ISPSTS_INTFLAG_Msk;

    while ((FMC->ISPSTS & (FMC_ISPSTS_INTFLAG_Msk | FMC_ISPSTS_ISPFF_Msk)) != 0)
        ;

    if (s_pfnStorageCallback)
        s_pfnStorageCallback(s_StorageStatus.error, s_StorageOper);
}

static ARM_DRIVER_VERSION ARM_Storage_GetVersion(void)
{
    return DriverVersion;
}

static ARM_STORAGE_CAPABILITIES ARM_Storage_GetCapabilities(void)
{
    return DriverCapabilities;
}

static int32_t ARM_Storage_Initialize(ARM_Storage_Callback_t callback)
{
    debug_printf("Storage range: 0x%08X ~ 0x%08X\n", DEF_STORAGE_BASE, (DEF_STORAGE_END - 1));

    if (callback != NULL)
        s_pfnStorageCallback = callback;

    /* In the case of synchronous execution, control returns after completion with a value of 1. */
    return 1;
}

static int32_t ARM_Storage_Uninitialize(void)
{
    return ARM_DRIVER_OK;
}

static int32_t ARM_Storage_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            SYS_UnlockReg();

            if (DriverCapabilities.asynchronous_ops)
            {
                NVIC_DisableIRQ(ISP_IRQn);
                FMC_DisableINT();
            }

            FMC_DISABLE_AP_UPDATE();
            FMC_Close();
            CLK_DisableModuleClock(ISP0_MODULE);
            break;

        case ARM_POWER_LOW:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

        case ARM_POWER_FULL:
            SYS_UnlockReg();
            CLK_EnableModuleClock(ISP0_MODULE);
            FMC_Open();
            FMC_ENABLE_AP_UPDATE();

            if (DriverCapabilities.asynchronous_ops)
            {
                FMC_EnableINT();
                NVIC_EnableIRQ(ISP_IRQn);
            }

            break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ARM_DRIVER_OK;
}

static int32_t ARM_Storage_ReadData(uint64_t addr, void *data, uint32_t size)
{
    uint32_t u32ReadCnt;
    uint32_t *pu32DataBuf = (uint32_t *)data;

    s_StorageOper = ARM_STORAGE_OPERATION_READ_DATA;

    if (DriverCapabilities.asynchronous_ops)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        for (u32ReadCnt = 0; u32ReadCnt < (size / StorageInfo.program_unit); u32ReadCnt++)
        {
            *pu32DataBuf = FMC_Read(FMC_APROM_BASE + addr + (u32ReadCnt * 4));

            if (g_FMC_i32ErrCode != FMC_OK)
                return ARM_DRIVER_ERROR;

            pu32DataBuf++;
        }

        return size;
    }
}

static int32_t ARM_Storage_ProgramData(uint64_t addr, const void *data, uint32_t size)
{
    uint32_t u32ProgCnt;
    uint32_t *pu32DataBuf = (uint32_t *)data;

    s_StorageOper = ARM_STORAGE_OPERATION_PROGRAM_DATA;

    if (DriverCapabilities.asynchronous_ops)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        for (u32ProgCnt = 0; u32ProgCnt < (size / StorageInfo.program_unit); u32ProgCnt++)
        {
            FMC_Write(FMC_APROM_BASE + addr + (u32ProgCnt * 4), *pu32DataBuf);

            if (g_FMC_i32ErrCode != FMC_OK)
                return ARM_DRIVER_ERROR;

            pu32DataBuf++;
        }

        return size;
    }
}

static int32_t ARM_Storage_Erase(uint64_t addr, uint32_t size)
{
    uint32_t u32EraseAddr;

    s_StorageStatus.error = 0;
    s_StorageOper = ARM_STORAGE_OPERATION_ERASE;

    if ((addr < DEF_STORAGE_BASE) || (addr >= DEF_STORAGE_END))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Check the range to be erased doesn't align with the erase_units of the
       respective start and end blocks */
    if (((addr % StorageAttr.erase_unit) != 0) || ((size % StorageAttr.erase_unit) != 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Check any part of the range is not erasable */
    if (FMC->APWPROT[(addr / FMC_APROM_BANK_SIZE)] & (1 << ((addr % StorageAttr.protection_unit) / StorageAttr.protection_unit)))
        return ARM_STORAGE_ERROR_NOT_ERASABLE;

    if (DriverCapabilities.asynchronous_ops)
    {
        FMC->ISPCMD  = FMC_ISPCMD_PAGE_ERASE;
        FMC->ISPADDR = FMC_APROM_BASE + addr;
        FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;
        s_StorageStatus.busy = 1;

        return ARM_DRIVER_OK;
    }
    else
    {
        for (u32EraseAddr = addr; u32EraseAddr < (addr + size); u32EraseAddr += StorageAttr.erase_unit)
        {
            if (FMC_Erase(FMC_APROM_BASE + u32EraseAddr) != 0)
                return ARM_DRIVER_ERROR;
        }

        return size;
    }
}

static int32_t ARM_Storage_EraseAll(void)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static ARM_STORAGE_STATUS ARM_Storage_GetStatus(void)
{
    return s_StorageStatus;
}

static int32_t ARM_Storage_GetInfo(ARM_STORAGE_INFO *info)
{
    if (info == NULL)
        return ARM_DRIVER_ERROR_PARAMETER;

    memcpy(info, &StorageInfo, sizeof(ARM_STORAGE_INFO));

    return ARM_DRIVER_OK;
}

static uint32_t ARM_Storage_ResolveAddress(uint64_t addr)
{
    if ((addr < DEF_STORAGE_BASE) || (addr >= DEF_STORAGE_END))
        return ARM_STORAGE_INVALID_ADDRESS;
    else
        return (FMC_APROM_BASE + addr);
}

static int32_t ARM_Storage_GetNextBlock(const ARM_STORAGE_BLOCK *prev_block, ARM_STORAGE_BLOCK *next_block)
{
    if (next_block == NULL)
        return ARM_DRIVER_ERROR_PARAMETER;

    if (prev_block == NULL)
    {
        next_block->addr = DEF_STORAGE_BASE;
        next_block->size = StorageAttr.erase_unit;
    }
    else if ((prev_block->addr >= DEF_STORAGE_END) || ((prev_block->addr + FMC_FLASH_PAGE_SIZE) >= DEF_STORAGE_END))
    {
        next_block->addr = ARM_STORAGE_INVALID_OFFSET;
        next_block->size = 0;

        return ARM_DRIVER_ERROR;
    }
    else
    {
        next_block->addr = prev_block->addr + prev_block->size;
        next_block->size = StorageAttr.erase_unit;
    }

    memcpy(&next_block->attributes, &StorageAttr, sizeof(ARM_STORAGE_BLOCK_ATTRIBUTES));
    debug_printf("addr: 0x%llx, size: 0x%llx\n", next_block->addr, next_block->size);

    return ARM_DRIVER_OK;
}

static int32_t ARM_Storage_GetBlock(uint64_t addr, ARM_STORAGE_BLOCK *block)
{
    if (block == NULL)
        return ARM_DRIVER_ERROR_PARAMETER;

    if ((addr >= DEF_STORAGE_BASE) && (addr < DEF_STORAGE_END))
    {
        block->addr = addr - (addr % StorageAttr.erase_unit);
    }

    return ARM_DRIVER_OK;
}

// End Storage Interface

extern ARM_DRIVER_STORAGE ARM_Driver_Storage_(DRIVER_STORAGE_NUM);

ARM_DRIVER_STORAGE ARM_Driver_Storage_(DRIVER_STORAGE_NUM) =
{
    ARM_Storage_GetVersion,
    ARM_Storage_GetCapabilities,
    ARM_Storage_Initialize,
    ARM_Storage_Uninitialize,
    ARM_Storage_PowerControl,
    ARM_Storage_ReadData,
    ARM_Storage_ProgramData,
    ARM_Storage_Erase,
    ARM_Storage_EraseAll,
    ARM_Storage_GetStatus,
    ARM_Storage_GetInfo,
    ARM_Storage_ResolveAddress,
    ARM_Storage_GetNextBlock,
    ARM_Storage_GetBlock
};

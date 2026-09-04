/**************************************************************************//**
 * @file     dataflashprog.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Data flash programming driver header
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DATA_FLASH_PROG_H__
#define __DATA_FLASH_PROG_H__

#define MASS_STORAGE_OFFSET       0x00008000  /* To avoid the code to write APROM */
#define DATA_FLASH_STORAGE_SIZE   (64*1024)   /* Configure the DATA FLASH storage u32Size. To pass USB-IF MSC Test, it needs > 64KB */
#define BUFFER_PAGE_SIZE          512

void DataFlashWrite(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer);
void DataFlashRead(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer);

#endif  /* __DATA_FLASH_PROG_H__ */

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
/***************************************************************************//**
 * @file     fmc_user.h
 * @version  V1.00
 * @brief    Simplified FMC driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __FMC_USER_H__
#define __FMC_USER_H__

#include "targetdev.h"

#ifdef __cplusplus
extern "C" {
#endif

int  FMC_Write_User(uint32_t u32Addr, uint32_t u32Data);
int  FMC_Read_User(uint32_t u32Addr, uint32_t *pu32Data);
int  FMC_Erase_User(uint32_t u32PageAddr);
void ReadData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32Data);
void WriteData(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t *pu32Data);
int  EraseAP(uint32_t u32StartAddr, uint32_t u32EraseSize);
void UpdateConfig(uint32_t *pu32Data, uint32_t *pu32Resp);

#ifdef __cplusplus
}
#endif

#endif  /* __FMC_USER_H__ */

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

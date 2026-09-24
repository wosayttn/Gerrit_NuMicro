/**************************************************************************//**
 * @file     rmc_user.h
 * @brief    M2L31 series RMC driver header file
 * @version  2.0.0
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef RMC_USER_H
#define RMC_USER_H

#include "targetdev.h"

#define Config0         RMC_CONFIG_BASE
#define Config1         (RMC_CONFIG_BASE+4)

#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/*  RMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#define _RMC_ENABLE_CFG_UPDATE()   (RMC->ISPCTL |=  RMC_ISPCTL_CFGUEN_Msk) /*!< Enable CONFIG Update Function  */
#define _RMC_DISABLE_CFG_UPDATE()  (RMC->ISPCTL &= ~RMC_ISPCTL_CFGUEN_Msk) /*!< Disable CONFIG Update Function */
#define RMC_ENABLE_AP_UPDATE()      (RMC->ISPCTL |=  RMC_ISPCTL_APUEN_Msk)      /*!< Enable APROM update        \hideinitializer */
#define RMC_DISABLE_AP_UPDATE()     (RMC->ISPCTL &= ~RMC_ISPCTL_APUEN_Msk)      /*!< Disable APROM update       \hideinitializer */

int RMC_Write_User(uint32_t u32Addr, uint32_t u32Data);
int RMC_Read_User(uint32_t u32Addr, uint32_t *data);
void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
void UpdateConfig(uint32_t *data, uint32_t *res);
void GetDataFlashInfo(uint32_t *addr, uint32_t *size);

#endif


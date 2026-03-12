/***************************************************************************//**
 * @file     targetdev.h
 * @version  V1.00
 * @brief    ISP support function header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __TARGETDEV_H__
#define __TARGETDEV_H__

#include "NuMicro.h"

#define PLL_CLOCK       FREQ_144MHZ
#define I2C_ADDR        0x60
#define DETECT_PIN      PB0
#define TRIM_INIT       (SYS_BASE + 0x10C)
#define ISP_CONFIG_CNT  14

#ifdef __cplusplus
extern "C"
{
#endif

void GetDataFlashInfo(uint32_t *addr, uint32_t *size);
uint32_t GetApromSize(void);

#ifdef __cplusplus
}
#endif

#endif // __TARGETDEV_H__

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

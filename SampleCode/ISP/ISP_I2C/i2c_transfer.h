/**************************************************************************//**
 * @file     i2c_transfer.c
 * @version  V1.00
 * @brief    I2C ISP support function header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

extern volatile uint8_t bI2cDataReady;
extern uint8_t i2c_rcvbuf[];

/*-------------------------------------------------------------*/
void I2C_Init(void);

#ifdef __cplusplus
}
#endif

#endif  /* __I2C_TRANS_H__ */

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

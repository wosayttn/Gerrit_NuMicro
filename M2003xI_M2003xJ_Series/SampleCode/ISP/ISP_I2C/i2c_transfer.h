/******************************************************************************
 * @file     i2c_transfer.h
 * @version  1.0.0
 * @brief    I2C ISP slave header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __I2C_TRANS_H__
#define __I2C_TRANS_H__
#include <stdint.h>

extern volatile uint8_t bI2cDataReady;
extern uint8_t i2c_rcvbuf[];

/*-------------------------------------------------------------*/
void I2C_Init(void);

#endif  /* __I2C_TRANS_H__ */

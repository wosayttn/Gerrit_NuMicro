/****************************************************************************//**
 * @file     i2c_interrupt.h
 * @version  V1.00
 * @brief    M251 series I2C driver header file
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NPD48_I2C_H__
#define __NPD48_I2C_H__

#ifdef __cplusplus
extern "C"
{
#endif

uint8_t I2C_WriteByteTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data);
uint32_t I2C_WriteMultiBytesTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t data[], uint32_t u32wLen);
uint8_t I2C_ReadByteTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr);
uint32_t I2C_ReadMultiBytesTwoRegsProtect(I2C_T *i2c, uint8_t u8SlaveAddr, uint16_t u16DataAddr, uint8_t rdata[], uint32_t u32rLen);

#ifdef __cplusplus
}

#endif
#endif /* __NPD48_I2C_H__ */

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/

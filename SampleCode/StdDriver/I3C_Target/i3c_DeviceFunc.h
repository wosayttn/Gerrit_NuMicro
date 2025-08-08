/**************************************************************************//**
 * @file     i3c_DeviceFunc.c
 * @version  V3.00
 * @brief    i3c_DeviceFunc header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "i3c_cfg.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void     I3C_Init(I3C_DEVICE_T *dev);
void     I3C_IRQFunc(I3C_DEVICE_T *dev);
void     I3C_ControllerRole(I3C_DEVICE_T *dev, uint32_t u32IsInit);
void     I3C_TargetRole(I3C_DEVICE_T *dev, uint32_t u32IsInit);
uint32_t I3C_FuncAddrCCC(I3C_DEVICE_T *dev, uint32_t ccc);
int32_t  I3C_FuncIBIReceived(I3C_DEVICE_T *dev);
int32_t  I3C_FuncCRRequest(I3C_DEVICE_T *dev);
int32_t  I3C_FuncSDRWrite(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t len);
int32_t  I3C_FuncSDRRead(I3C_DEVICE_T *dev, uint8_t tgt, uint8_t *buf, uint16_t *len);
int32_t  I3C_FuncGetDEFTGTS(I3C_DEVICE_T *dev);

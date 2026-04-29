/**************************************************************************//**
 * @file     NPD48.h
 * @version  V1.00
 * @brief    NPD48 peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for NuMicro NPD48 MCU.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2019 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NPD48_H__
#define __NPD48_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/******************************************************************************/
/*                            Register definitions                            */
/******************************************************************************/

#include "npd48_adc_reg.h"
#include "npd48_bc_reg.h"
#include "npd48_chip_reg.h"
#include "npd48_dac_reg.h"
#include "npd48_efuse_reg.h"
#include "npd48_gpio_reg.h"
#include "npd48_i2c_reg.h"
#include "npd48_utcpd_reg.h"


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "npd48_utcpd.h"
#include "i2c_interrupt.h"
//#include "npd48_adc.h"
#ifdef __cplusplus
}
#endif

#endif  /* __NPD48_H__ */

/***************************************************************************//**
 * @file    config.h
 * @version V1.00
 * @brief   I2S driver sample configuration header file.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CONFIG_H__
#define __CONFIG_H__

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        32
#define BUFF_HALF_LEN   (BUFF_LEN / 2)

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

extern uint32_t volatile g_u32BuffPos;

#endif  /* __CONFIG_H__ */

/**************************************************************************//**
 * @file     config.h
 * @version  V3.00
 * @brief    I2S driver sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        32
#define BUFF_HALF_LEN   (BUFF_LEN/2)

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

extern uint32_t volatile g_u32BuffPos;

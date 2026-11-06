/******************************************************************************
 * @file     xomlib.h
 * @version  V3.00
 * @brief    Header file for XOM APIs.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef _XOMLIB_H_
#define _XOMLIB_H_

#ifdef __cplusplus
extern "C"
{
#endif

extern int32_t (*XOM_Add)(int32_t a, int32_t b);
extern int32_t (*XOM_Div)(int32_t a, int32_t b);
extern int32_t (*XOM_Mul)(int32_t a, int32_t b);
extern int32_t (*XOM_Sub)(int32_t a, int32_t b);
extern int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n);

#ifdef __cplusplus
}
#endif

#endif  // _XOMLIB_H_

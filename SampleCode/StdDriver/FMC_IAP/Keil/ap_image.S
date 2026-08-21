;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch armv8-m.main


    .global  loaderImage1Base
    .global  loaderImage1Limit

    .align   4

    .text

loaderImage1Base:
#ifndef BIN_FILE
    .incbin "./fmc_ld_code.bin"
#else
    .incbin BIN_FILE
#endif
loaderImage1Limit:
    .space  4
    .end
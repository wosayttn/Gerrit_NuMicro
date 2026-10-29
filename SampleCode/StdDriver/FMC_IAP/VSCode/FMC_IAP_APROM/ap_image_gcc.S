;/**************************************************************************//**
; * @file     ap_image_gcc.S
; * @version  V1.00
; * @brief    Assembly code include LDROM image.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch   armv7-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit

    .align  4

    .text

loaderImage1Base:
    .incbin  "../../bin/fmc_ld_iap.bin"
loaderImage1Limit:
    .space   4

    .end

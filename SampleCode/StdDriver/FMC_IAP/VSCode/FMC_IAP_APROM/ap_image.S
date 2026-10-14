;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; * @copyright SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

    .section .rodata
    .global loaderImage1Base
    .global loaderImage1Limit
    .align 4
loaderImage1Base:
    .incbin     BINFILE
loaderImage1Limit:

;/**************************************************************************//**
; * @file     ap_image.s
; * @version  V1.00
; * @brief    Assembly code include LDROM image.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/

    .section .rodata
    .global  loaderImage1Base, loaderImage1Limit, loaderImage1Size
    .align   4

loaderImage1Base:
    .incbin  "../../bin/fmc_ld_iap.bin"
loaderImage1Limit:
loaderImage1Size = loaderImage1Limit - loaderImage1Base

    .end

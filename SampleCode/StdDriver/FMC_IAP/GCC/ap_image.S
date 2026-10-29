;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
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
#ifndef _ARM_VSCODE_
    .incbin  "../../LDROM_iap/LDROM_iap.bin"
#else
#ifdef _VSCODE_GCC_
    .incbin  "../../out/fmc_ld_iap/GNUC/release/fmc_ld_iap.bin"      
#else
    .incbin  "../../out/fmc_ld_iap/ARMCLANG/release/fmc_ld_iap.bin"  
#endif    
#endif    
loaderImage1Limit:
    .space   4

    .end


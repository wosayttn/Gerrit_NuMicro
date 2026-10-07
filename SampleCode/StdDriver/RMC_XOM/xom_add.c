/**************************************************************************//**
 * @file     xom_add.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 23/02/24 4:40p $
 * @brief    Show how to use XOM Lirbary
 *
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

int32_t LibXOMAdd(uint32_t a, uint32_t b);

int32_t LibXOMAdd(uint32_t a, uint32_t b)
{
    uint32_t c;
    c =  a + b;
    return (int32_t)c;
}

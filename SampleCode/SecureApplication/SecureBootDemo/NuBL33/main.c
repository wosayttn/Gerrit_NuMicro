/**************************************************************************//**
 * @file     main_ns.c
 * @version  V1.00
 * @brief    Demonstrate how to generate the NuBL33 and can be authenticated by NuBL2.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"
#include "NuBL32/NuBL32.h"

/*----------------------------------------------------------------------------
  Non-secure Callable Functions from Secure Region
 *----------------------------------------------------------------------------*/
uint32_t GetSystemCoreClock(void);   /* NuBL32 Non-secure callable function */

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    printf("\n\nCPU @ %d Hz\n", GetSystemCoreClock());
    printf("+-------------------------------------------+\n");
    printf("|    SecureBootDemo - NuBL33 Sample Code    |\n");
    printf("+-------------------------------------------+\n\n");

    printf("System is executing in NuBL33.\n\n");

    while (1) {}
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

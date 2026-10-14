/**************************************************************************//**
 * @file    active_sc.c
 * @version V1.00
 * @brief   Demonstrate how to active the Secure Conceal Function and
 *          clear I/D-Cache with PMC-100 to prevent potential code leakage.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"

typedef void (*PFN_FUNC)(void);

/* The Secure Conceal Region becomes inaccessible after activation.
 * Therefore, the activation function must be placed outside the Secure Conceal Region.
 * In this example, ActiveSecureConceal will be executed from SRAM. */
void ActiveSecureConceal(uint32_t u32NuBL32Base)
{
    PFN_FUNC pfnNuBL32Entry;

    if (FMC_Read(FMC_USER_CONFIG_6) != 0xFFFFFFFF)
    {
        /* Clear Cache before leave BL2 to prevent
         * potential leakage of Secure Conceal code through the cache. */


        // Ensure all operations are completed
        __ISB();
        __DSB();
        // Activate Secure Conceal Function
        FMC_SET_SC_ACTIVE();
    }

    /* Disable all interrupt */
    __set_PRIMASK(1);

    /* SCB.VTOR points to the NuBL32 vector table base address. */
    SCB->VTOR = u32NuBL32Base;

    /* 2nd entry contains the address of the Reset_Handler (CMSIS-CORE) function */
    pfnNuBL32Entry = ((PFN_FUNC)(*(((uint32_t *)SCB->VTOR) + 1)));
    /* execute NuBL32 FW */
    pfnNuBL32Entry();
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

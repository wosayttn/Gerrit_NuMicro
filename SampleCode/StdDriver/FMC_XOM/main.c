/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    An example of using FMC APIs to set up and erase XOM regions.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/* Need to place xom_add.o in specified XOM region in linker script */
#define XOMR0_BASE    (FMC_APROM_BASE + 0x10000)

extern int32_t Lib_XOM_ADD(uint32_t a, uint32_t b);

int32_t SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();

    /*------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                */
    /*------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Lock protected registers */
    SYS_LockReg();

    return 0;
}

int32_t main(void)
{
    uint32_t    u32Status;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART for print message */
    InitDebugUart();

    /*
     *   This sample code is used to show how to use FMC APIs to enable/erase XOM.
     */
    printf("\n\n");
    printf("+--------------------------------------+\n");
    printf("|  FMC XOM Config & Erase Sample Code  |\n");
    printf("+--------------------------------------+\n");

    SYS_UnlockReg();            /* Unlock protected registers */
    FMC_Open();                 /* Enable FMC ISP function    */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM update        */
    FMC_ENABLE_CFG_UPDATE();    /* Enable Config update       */

    if ((FMC_GetXOMState(XOMR0) == 0) &&
            (FMC_CheckAllOne(XOMR0_BASE, FMC_FLASH_PAGE_SIZE) == READ_ALLONE_YES))
    {
        printf("XOM0 region erased. No program code in XOM0.\n");
        printf("Demo completed.\nPlease re-program flash if you want to run again.\n");

        while (1);
    }

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Press any key to continue ...\n");
    getchar();

    /* Config XOM0 */
    if (FMC_GetXOMState(XOMR0) == 0)
    {
        u32Status = FMC_ConfigXOM(XOMR0, XOMR0_BASE, 1);

        if (u32Status)
            printf("XOM0 Config fail !\n");
        else
            printf("XOM0 Config OK.\n");

        printf("\nPress any key to reset chip to enable XOM0 region ...\n");
        getchar();

        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }
    else
    {
        uint32_t u32Addr;
        // Check CPU data access in XOM region should read all 0xFFFFFFFF
        printf("Check XOM0 [0x%08X ~ 0x%08X] all 0xFFFFFFFF.\n", (uint32_t)XOMR0_BASE, (uint32_t)(XOMR0_BASE + FMC_FLASH_PAGE_SIZE));

        for (u32Addr = XOMR0_BASE; u32Addr < (XOMR0_BASE + FMC_FLASH_PAGE_SIZE); u32Addr += 4)
        {
            if (M32(u32Addr) != 0xFFFFFFFF)
            {
                printf("  Read 0x%08X not 0xFFFFFFFF but 0x%08X !\n", u32Addr, M32(u32Addr));
                break;
            }
        }
    }

    /* Run XOM function */
    printf("\nLib_XOM_ADD: 0x%08X\n", (uint32_t)Lib_XOM_ADD);
    printf("  100 + 200 = %d\n", Lib_XOM_ADD(100, 200));

    printf("\nXOMR0 active success....\n");
    printf("\nPress any key to erase XOM0 ...\n");
    getchar();

    if (FMC_GetXOMState(XOMR0) == 1)
    {
        /* Erase XOM0 region */
        if (FMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOM0 ... OK.\n");
        else
            printf("Erase XOM0 ... Fail !\n");
    }

    printf("Done.\n");
    printf("Please reset chip to check if XOM0 empty.\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

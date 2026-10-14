/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    An example of using FMC APIs to configure/erase XOM regions for
 *           dual bank/segment operation. To ensure XOM functions work after bank remap,
 *           XOM regions must be configured at the corresponding offset in each bank/segment.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2026 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

#define LOADER_BASE         (FMC_APROM_BASE)
#define LOADER_SIZE         (0x4000)
#define FMC_SEGMENT_SIZE    (FMC_APROM_SIZE>>1)

/* Need to place xom_add.o in specified XOM region in linker script */
#define XOMR_PAGE_CNT       1
#define XOMR0_BASE          0x10000
/* Calculate the corresponding XOM offset in another bank/segment */
#if (XOMR0_BASE > FMC_SEGMENT_SIZE)
    #define XOMR1_BASE      ((XOMR0_BASE) - FMC_SEGMENT_SIZE)
#else
    #define XOMR1_BASE      ((XOMR0_BASE) + FMC_SEGMENT_SIZE)
#endif

extern int32_t Lib_XOM_ADD(uint32_t a, uint32_t b);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main()
{
    uint32_t    u32Status, u32Addr, u32Offset, u32ActBank, u32NewBank;
    uint32_t    u32Loader0ChkSum, u32Loader1ChkSum;

    SYS_UnlockReg();                   /* Unlock register lock protect */

    SYS_Init();                        /* Init System, peripheral clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    /*
     *   This sample code is used to show how to use FMC APIs to enable/erase XOM.
     */
    printf("\n\n");
    printf("+--------------------------------------------------------+\n");
    printf("|  FMC Dual Bank/Segment XOM Config & Erase Sample Code  |\n");
    printf("+--------------------------------------------------------+\n");

    SYS_UnlockReg();            /* Unlock protected registers */
    FMC_Open();                 /* Enable FMC ISP function    */
    FMC_ENABLE_AP_UPDATE();     /* Enable APROM update        */
    FMC_ENABLE_CFG_UPDATE();    /* Enable Config update       */

    u32ActBank = FMC_GetBankIdx();
    printf("Bank/Segment%d is active.\n", u32ActBank);
    u32NewBank = u32ActBank ^ 1;

    u32Loader0ChkSum = FMC_GetChkSum(LOADER_BASE, LOADER_SIZE);
    u32Loader1ChkSum = FMC_GetChkSum(LOADER_BASE + FMC_SEGMENT_SIZE, LOADER_SIZE);
    printf("Bank/Segment0 Loader checksum: 0x%08X.\nBank/Segment1 Loader checksum: 0x%08X.\n\n", u32Loader0ChkSum, u32Loader1ChkSum);

    if ((u32ActBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
    {
        printf("Create Bank/Segment%d Loader... \n",  u32NewBank);

        /* Erase loader region in the other bank */
        for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += FMC_FLASH_PAGE_SIZE)
        {
            FMC_Erase(u32Addr + (FMC_SEGMENT_SIZE * u32NewBank));
        }

        /* Create loader in the other bank */
        for (u32Addr = LOADER_BASE; u32Addr < (LOADER_BASE + LOADER_SIZE); u32Addr += 8)
        {
            FMC_Write8Bytes(u32Addr + (FMC_SEGMENT_SIZE * u32NewBank),
                            FMC_Read(u32Addr + (FMC_SEGMENT_SIZE * u32ActBank)),
                            FMC_Read(u32Addr + (FMC_SEGMENT_SIZE * u32ActBank) + 4)
                           );
        }

        printf("Create Bank/Segment%d Loader completed. \n\n", u32NewBank);
    }

    if ((FMC_GetXOMState(XOMR0) == 0) &&
            (FMC_CheckAllOne(XOMR0_BASE, (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)) == READ_ALLONE_YES))
    {
        printf("XOM0 region erased. No program code in XOM0.\n");

        if ((FMC_GetXOMState(XOMR1) == 0) &&
                (FMC_CheckAllOne(XOMR1_BASE, (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)) == READ_ALLONE_YES))
        {
            printf("XOM1 region erased. No program code in XOM1.\n");
            printf("Demo completed.\nPlease re-program flash if you want to run again.\n");

            while (1);
        }
    }

    printf("XOM Status = 0x%X\n", FMC->XOMSTS);
    printf("Press any key to continue ...\n");
    getchar();

    /* Configure XOM1 at the corresponding offset in the other bank */
    if (FMC_GetXOMState(XOMR1) == 0)
    {
        /* Copy XOM code from XOM0 to XOM1 */
        for (u32Offset = 0; u32Offset < (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE); u32Offset += 8)
        {
            if ((u32Offset % FMC_FLASH_PAGE_SIZE) == 0)
            {
                if (FMC_Erase(XOMR1_BASE + u32Offset) != FMC_OK)
                {
                    printf("Failed to erase 0x%08X !\n", (uint32_t)(XOMR1_BASE + u32Offset));

                    while (1) {};
                }
            }

            if (FMC_Write8Bytes(XOMR1_BASE + u32Offset, FMC_Read(XOMR0_BASE + u32Offset), FMC_Read(XOMR0_BASE + u32Offset + 4)) != FMC_OK)
            {
                printf("Failed to write 0x%08X !\n", (uint32_t)(XOMR1_BASE + u32Offset));

                while (1) {};
            }
        }

        u32Status = FMC_ConfigXOM(XOMR1, XOMR1_BASE, XOMR_PAGE_CNT);

        if (u32Status)
            printf("XOM1 Config fail !\n");
        else
            printf("XOM1 Config OK.\n");
    }
    else
    {
        // Check CPU data access in XOM region should read all 0xFFFFFFFF
        printf("Check XOM1 [0x%08X ~ 0x%08X] all 0xFFFFFFFF.\n", (uint32_t)XOMR1_BASE, (uint32_t)(XOMR1_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)));

        for (u32Addr = XOMR1_BASE; u32Addr < (XOMR1_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)); u32Addr += 4)
        {
            if (M32(u32Addr) != 0xFFFFFFFF)
            {
                printf("  Read 0x%08X not 0xFFFFFFFF but 0x%08X !\n", u32Addr, M32(u32Addr));
                break;
            }
        }
    }

    /* Config XOM0 */
    if (FMC_GetXOMState(XOMR0) == 0)
    {
        u32Status = FMC_ConfigXOM(XOMR0, XOMR0_BASE, XOMR_PAGE_CNT);

        if (u32Status)
            printf("XOM0 Config fail !\n");
        else
            printf("XOM0 Config OK.\n");
    }
    else
    {
        // Check CPU data access in XOM region should read all 0xFFFFFFFF
        printf("Check XOM0 [0x%08X ~ 0x%08X] all 0xFFFFFFFF.\n", (uint32_t)XOMR0_BASE, (uint32_t)(XOMR0_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)));

        for (u32Addr = XOMR0_BASE; u32Addr < (XOMR0_BASE + (XOMR_PAGE_CNT * FMC_FLASH_PAGE_SIZE)); u32Addr += 4)
        {
            if (M32(u32Addr) != 0xFFFFFFFF)
            {
                printf("  Read 0x%08X not 0xFFFFFFFF but 0x%08X !\n", u32Addr, M32(u32Addr));
                break;
            }
        }
    }

    /* Reset chip to enable XOM region. */
    if ((FMC_GetXOMState(XOMR0) == 0) || (FMC_GetXOMState(XOMR1) == 0))
    {
        printf("\nPress any key to reset chip to enable XOM0 and XOM1 region ...\n");
        getchar();
        /* Reset chip to enable XOM region. */
        SYS_ResetChip();

        while (1) {};
    }

    printf("\n* Bank/Segment%d is active.\n", FMC_GetBankIdx());
    /* Run XOM function in XOM0 */
    printf("Lib_XOM_ADD: 0x%08X\n", (uint32_t)Lib_XOM_ADD);
    printf("  100 + 200 = %d\n", Lib_XOM_ADD(100, 200));
    printf("XOMR0 active success.\n");

    if (FMC_RemapBank(u32NewBank) != FMC_OK)
    {
        printf("\n* Remap to Bank/Segment%d failed !\n", u32NewBank);

        while (1) {};
    }
    else
    {
        /* Flush CACHE to ensure the CPU fetches updated instructions/data from the remapped bank */
        CACHE_Flush();
        printf("\n* Bank/Segment%d is active.\n", FMC_GetBankIdx());
    }

    /* Run XOM function in XOM1 */
    printf("Lib_XOM_ADD: 0x%08X\n", (uint32_t)Lib_XOM_ADD);
    printf("  123 + 234 = %d\n", Lib_XOM_ADD(123, 234));
    printf("XOMR1 active success.\n");

    printf("\nPress any key to erase XOM0 and XOM1 ...\n");
    getchar();

    if (FMC_GetXOMState(XOMR0) == 1)
    {
        /* Erase XOM0 region */
        if (FMC_EraseXOM(XOMR0) == 0)
            printf("Erase XOM0 ... OK.\n");
        else
            printf("Erase XOM0 ... Fail !\n");
    }

    if (FMC_GetXOMState(XOMR1) == 1)
    {
        /* Erase XOM1 region */
        if (FMC_EraseXOM(XOMR1) == 0)
            printf("Erase XOM1 ... OK.\n");
        else
            printf("Erase XOM1 ... Fail !\n");
    }

    printf("Done.\n");
    printf("Please reset chip to check if XOM0 and XOM1 are empty.\n");

    while (1) {};
}

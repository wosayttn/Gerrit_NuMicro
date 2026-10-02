/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use FMC Read-All-One ISP command to verify APROM/LDROM pages are all 0xFFFFFFFF or not.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 96MHz from PLL */
    CLK_SetCoreClock(FREQ_96MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk))    |       \
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main(void)
{
    uint32_t u32Data, u32ret;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+------------------------------------------+\n");
    printf("|        FMC_ReadAllOne Sample Code        |\n");
    printf("+------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    /* Read company ID. Should be 0xDA. */
    u32Data = FMC_ReadCID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read product ID. */
    u32Data = FMC_ReadPID();
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG2 */
    printf("  User Config 2 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE + 8));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+8) failed!\n");
        goto lexit;
    }

    /* Enable LDROM update. */
    FMC_ENABLE_LD_UPDATE();

    /* Erase LDROM page 0. */
    if (FMC_Erase(FMC_LDROM_BASE) != 0)     /* Erase LDROM page 0. */
    {
        printf("FMC_Erase(FMC_LDROM_BASE) failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if (u32ret == READ_ALLONE_YES)
        printf("READ_ALLONE_YES success.\n");
    else
        printf("READ_ALLONE_YES failed!\n");

    /* Program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    if (FMC_Write(FMC_LDROM_BASE, 0) != 0)       /* program a 0 to LDROM to make it not all 0xFFFFFFFF. */
    {
        printf("FMC_Write FMC_LDROM_BASE failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are not all 0xFFFFFFFF. */
    u32ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_CheckAllOne failed!\n");
        goto lexit;
    }

    if (u32ret == READ_ALLONE_NOT)
        printf("READ_ALLONE_NOT success.\n");
    else
        printf("READ_ALLONE_NOT failed!\n");

    printf("\nFMC Read-All-One test done.\n");
lexit:

    /* Disable FMC ISP function */
    FMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/

/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use DFMC CRC32 ISP command to calculate the CRC32 checksum of Dataflash.
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
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n");
    printf("+------------------------------------+\n");
    printf("|   M471 DFMC CRC32 Sample Demo      |\n");
    printf("+------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    /* DFMC_ReadCID */
    u32Data = DFMC_ReadCID();
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* DFMC_ReadPID */
    u32Data = DFMC_ReadPID();
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Erase the first page of Dataflash */
    DFMC_ENABLE_UPDATE();

    DFMC_Erase(DFMC_DFLASH_BASE);
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_Erase failed!\n");
        goto lexit;
    }
    /* Write one word on Dataflash */
    DFMC_Write(DFMC_DFLASH_BASE, 0x55AABBCC);
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_Write failed!\n");
        goto lexit;
    }
    printf("\nDataflash address (0x%lX ~ 0x%lX) CRC32 checksum =>  ",DFMC_DFLASH_BASE, DFMC_DFLASH_END);
    /*
     *  Request DFMC hardware to run CRC32 calculation on APROM address 0x00400000 ~ 0x00408000.
     *  Note that DFMC CRC32 checksum calculation area must page alignment.
     */
    u32ChkSum = DFMC_GetChkSum(DFMC_DFLASH_BASE, DFMC_DFLASH_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out Dataflash CRC32 check sum value */

    printf("\nDFMC CRC32 checksum test done.\n");
lexit:
    DFMC_Close();                       /* Disable DFMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/



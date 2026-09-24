/****************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use DFMC Read-All-One ISP command to verify Dataflash pages are all 0xFFFFFFFF or not.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved..
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

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);

    /* Set core clock to 40MHz */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Enable DFMC0 module clock */
    CLK_EnableModuleClock(DFMC0_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

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
    printf("|        DFMC_ReadAllOne Sample Code       |\n");
    printf("+------------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable DFMC ISP function. Before using DFMC function, it should unlock system register first. */
    DFMC_Open();

    /* Read company ID. Should be 0xDA. */
    u32Data = DFMC_ReadCID();
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Enable Update. */
    DFMC_ENABLE_DF_UPDATE();

    /* Erase Dataflash page 0. */
    if (DFMC_Erase(DFMC_FLASH_BASE) != 0)     /* Erase Dataflash page 0. */
    {
        printf("DFMC_Erase(DFMC_FLASH_BASE) failed!\n");
        goto lexit;
    }

    /* Run and check flash contents are all 0xFFFFFFFF. */
    u32ret = DFMC_CheckAllOne(DFMC_FLASH_BASE, DFMC_FLASH_SIZE);
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_CheckAllOne failed!\n");
        goto lexit;
    }
    if (u32ret == DREAD_ALLONE_YES)
        printf("DREAD_ALLONE_YES success.\n");
    else
        printf("DREAD_ALLONE_YES failed!\n");

    printf("\nDFMC Read-All-One test done.\n");

lexit:
    /* Disable DFMC ISP function */
    DFMC_Close();

    /* Lock protected registers */
    SYS_LockReg();

    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

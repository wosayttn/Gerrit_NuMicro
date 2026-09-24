/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use DFMC CRC32 ISP command to calculate the CRC32 checksum of Dataflash.
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
    printf("|   M2003J DFMC CRC32 Sample Demo     |\n");
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

    /* Erase the first page of Dataflash */
    DFMC_ENABLE_DF_UPDATE();

    DFMC_Erase(DFMC_FLASH_BASE);
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_Erase failed!\n");
        goto lexit;
    }
    /* Write one word on Dataflash */
    DFMC_Write(DFMC_FLASH_BASE, 0x55AABBCC);
    if (g_DFMC_i32ErrCode != 0)
    {
        printf("DFMC_Write failed!\n");
        goto lexit;
    }
    printf("\nDataflash address (0x%lX ~ 0x%lX) CRC32 checksum =>  ",DFMC_FLASH_BASE, DFMC_FLASH_END);
    /*
     *  Request DFMC hardware to run CRC32 calculation on APROM address 0x00400000 ~ 0x00408000.
     *  Note that DFMC CRC32 checksum calculation area must page alignment.
     */
    u32ChkSum = DFMC_GetChkSum(DFMC_FLASH_BASE, DFMC_FLASH_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out Dataflash CRC32 checksum value */

    printf("\nDFMC CRC32 checksum test done.\n");
lexit:
    DFMC_Close();                       /* Disable DFMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/



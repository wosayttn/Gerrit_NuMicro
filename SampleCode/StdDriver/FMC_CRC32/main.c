/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Demonstrate how to use FMC CRC32 ISP command to calculate the CRC32 checksum of APROM and LDROM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
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

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main()
{
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    SYS_Init();                        /* Init System, peripheral clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("+------------------------------------+\n");
    printf("|      FMC CRC32 Sample Demo         |\n");
    printf("+------------------------------------+\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC_Open();

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0x530000DA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    printf("\nLDROM (0xF100000 ~ 0xF102000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_LDROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 checksum value */


    printf("\nAPROM (0x0 ~ 0x80000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, 0x80000);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 checksum value */

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}
/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/



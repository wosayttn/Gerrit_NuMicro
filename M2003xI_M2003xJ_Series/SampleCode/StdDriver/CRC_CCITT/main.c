/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement CRC in CRC-CCITT mode and get the CRC checksum result.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
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

    /* Enable CRC0 clock */
    CLK_EnableModuleClock(CRC0_MODULE);

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
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    const uint16_t u16CRCSrcPattern[] = {0x3231, 0x3433, 0x3635, 0x3837};
    uint32_t i, u32TargetChecksum = 0xA12B, u32CalChecksum = 0;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------+\n");
    printf("|    CRC-CCITT Polynomial Mode Sample Code    |\n");
    printf("+---------------------------------------------+\n\n");

    printf("# Calculate [0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38] CRC-CCITT checksum value.\n");
    printf("    - Seed value is 0xFFFF             \n");
    printf("    - CPU write data length is 16 bits\n");
    printf("    - Checksum complement disable    \n");
    printf("    - Checksum reverse disable       \n");
    printf("    - Write data complement disable  \n");
    printf("    - Write data reverse disable     \n");
    printf("    - Checksum should be 0x%x        \n\n", u32TargetChecksum);

    /* Configure CRC controller for CRC-CCITT CPU mode */
    CRC_Open(CRC_CCITT, 0, 0xFFFF, CRC_CPU_WDATA_16);

    /* Start to execute CRC-CCITT operation */
    for(i = 0; i < sizeof(u16CRCSrcPattern) / sizeof(u16CRCSrcPattern[0]); i++)
    {
        CRC_WRITE_DATA((u16CRCSrcPattern[i]));
    }

    /* Get CRC-CCITT checksum value */
    u32CalChecksum = CRC_GetChecksum();
    printf("CRC checksum is 0x%x ... %s.\n", u32CalChecksum, (u32CalChecksum == u32TargetChecksum) ? "PASS" : "FAIL");

    /* Disable CRC0 function */
    CLK_DisableModuleClock(CRC0_MODULE);

    while(1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

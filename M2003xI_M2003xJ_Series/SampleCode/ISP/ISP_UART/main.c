/***************************************************************************//**
 * @file     main.c
 * @version  V1.0
 * @brief    ISP tool main function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "uart_transfer.h"

int32_t FMC_SetVectorAddr(uint32_t u32PageAddr);

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->HCLKSEL = (CLK->HCLKSEL & (~CLK_HCLKSEL_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->HCLKDIV = (CLK->HCLKDIV & (~CLK_HCLKDIV_HCLKDIV_Msk)) | CLK_HCLKDIV_HCLK(1);

    /* Enable UART module clock */
    CLK->UARTCTL |= CLK_UARTCTL_UART0CKEN_Msk;
    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->UARTSEL = (CLK->UARTSEL & (~CLK_UARTSEL_UART0SEL_Msk)) | CLK_UARTSEL_UART0SEL_HIRC;
    CLK->UARTDIV = (CLK->UARTDIV & (~CLK_UARTDIV_UART0DIV_Msk)) | CLK_UARTDIV_UART0(1);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 */
    UART_Init();

    CLK->FMCCTL |= CLK_FMCCTL_ISP0CKEN_Msk | CLK_FMCCTL_DFMC0CKEN_Msk;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while (1)
    {
        if ((bufhead >= 4) || (bUartDataReady == TRUE))
        {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT)
            {
                goto _ISP;
            }
            else
            {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bUartDataReady == TRUE)
        {
            bUartDataReady = FALSE;
            ParseCmd(uart_rcvbuf, 64);
            PutString();
        }
    }

_APROM:
    FMC_SetVectorAddr(FMC_APROM_BASE);
    FMC_SET_APROM_BOOT();
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}

/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/
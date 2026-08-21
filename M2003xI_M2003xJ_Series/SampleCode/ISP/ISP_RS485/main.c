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

#define nRTSPin                 (PB8)
#define RECEIVE_MODE            (0)
#define TRANSMIT_MODE           (1)

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
    CLK->UARTCTL |= CLK_UARTCTL_UART1CKEN_Msk;

    /* Enable GPB clock */
    CLK->GPIOCTL |= CLK_GPIOCTL_GPIOBCKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->UARTSEL = (CLK->UARTSEL & (~CLK_UARTSEL_UART1SEL_Msk)) | CLK_UARTSEL_UART1SEL_HIRC;
    CLK->UARTDIV = (CLK->UARTDIV & (~CLK_UARTDIV_UART1DIV_Msk)) | CLK_UARTDIV_UART1(1);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    PB->MODE = (PB->MODE & ~(0x3ul << (8 << 1))) | (GPIO_MODE_OUTPUT << (8 << 1));
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk );
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB2MFP_UART1_RXD | SYS_GPB_MFPL_PB3MFP_UART1_TXD );

    nRTSPin = RECEIVE_MODE;

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
            NVIC_DisableIRQ(UART1_IRQn);
            nRTSPin = TRANSMIT_MODE;
            PutString();

            while ((UART1->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) == 0);

            nRTSPin = RECEIVE_MODE;
            NVIC_EnableIRQ(UART1_IRQn);
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
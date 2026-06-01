/***************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    ISP tool main function
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

int32_t FMC_SetVectorAddr(uint32_t u32PageAddr);

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}

void TIMER_Init(void)
{
    uint32_t u32Prescale = 40;
    /* Set timer frequency to 1HZ */
    TIMER0->CTL = (TIMER0->CTL & ~(TIMER_CTL_OPMODE_Msk | TIMER_CTL_PSC_Msk)) |
                  (TIMER_PERIODIC_MODE | (u32Prescale - 1) | TIMER_CTL_INTEN_Msk);
    TIMER0->CMP = __HIRC / u32Prescale;
    /* Start Timer 0 */
    TIMER0->CTL |= TIMER_CTL_CNTEN_Msk;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK->SRCCTL |= CLK_SRCCTL_HIRCEN_Msk;

    /* Waiting for HIRC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->HCLKSEL = (CLK->HCLKSEL & ~CLK_HCLKSEL_HCLKSEL_Msk) | CLK_HCLKSEL_HCLKSEL_HIRC;
    CLK->HCLKDIV = (CLK->HCLKDIV & (~CLK_HCLKDIV_HCLKDIV_Msk)) | CLK_HCLKDIV_HCLK(1);
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV2 | CLK_PCLKDIV_PCLK1DIV_DIV2);
    /* Enable USPI0 clock */
    CLK->USCICTL |= CLK_USCICTL_USCI0CKEN_Msk;
    /* Enable TMR0 clock */
    CLK->TMRCTL |= CLK_TMRCTL_TMR0CKEN_Msk;
    /* Select IP clock source */
    CLK->TMRSEL0 = (CLK->TMRSEL0 & ~CLK_TMRSEL0_TMR0SEL_Msk) | CLK_TMRSEL0_TMR0SEL_HIRC;
    /* Update System Core Clock */
    SystemCoreClock = __HIRC;
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup SPI0 multi-function pins */
    /* PB.0 is USPI0_SS,   PA.11 is USPI0_CLK,
       PA.9 is USPI0_MISO, PA.10 is USPI0_MOSI*/
    SYS->GPA_MFPH = (SYS->GPA_MFPH & ~(SYS_GPA_MFPH_PA11MFP_Msk |
                                       SYS_GPA_MFPH_PA10MFP_Msk |
                                       SYS_GPA_MFPH_PA9MFP_Msk)) |
                    (SYS_GPA_MFPH_PA11MFP_USCI0_CLK |
                     SYS_GPA_MFPH_PA10MFP_USCI0_DAT0 |
                     SYS_GPA_MFPH_PA9MFP_USCI0_DAT1);

    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk)) | SYS_GPB_MFPL_PB0MFP_USCI0_CTL0;

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t cmd_buff[16];

    SYS_Init();

    CLK->FMCCTL |= CLK_FMCCTL_ISP0CKEN_Msk | CLK_FMCCTL_DFMC0CKEN_Msk;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    USPI_Init();
    TIMER_Init();

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            goto _ISP;
        }

        if (TIMER0->INTSTS & TIMER_INTSTS_TIF_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bSpiDataReady == 1)
        {
            /* Disable SPI IRQ until ParseCmd() is finished to prevent returning incomplete data prematurely */
            NVIC_DisableIRQ(USCI0_IRQn);
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
            bISPDataReady = 1;
            NVIC_EnableIRQ(USCI0_IRQn);
        }
    }

_APROM:
    FMC_SetVectorAddr(FMC_APROM_BASE);
    FMC_SET_APROM_BOOT();
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}

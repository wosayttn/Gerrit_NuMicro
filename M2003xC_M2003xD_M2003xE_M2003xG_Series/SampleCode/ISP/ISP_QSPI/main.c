/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x34
 * @date     14, June, 2017
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2017-2018 Nuvoton Technology Corp. All rights reserved.
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
    uint32_t u32Prescale = 32;
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
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
    /* Select PCLK0 as the clock source of QSPI0 */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_QSPI0SEL_Msk)) | CLK_CLKSEL2_QSPI0SEL_PCLK0;
    /* Enable QSPI0 clock */
    CLK->APBCLK0 |= (CLK_APBCLK0_TMR0CKEN_Msk | CLK_APBCLK0_QSPI0CKEN_Msk);
    /* Enable GPA clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPACKEN_Msk;
    /* Select IP clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_HIRC;
    /* Update System Core Clock */
    SystemCoreClock = __HIRC;
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Setup QSPI0 multi-function pins */
    /* PA.3 is QSPI0_SS,   PA.2 is QSPI0_CLK,
       PA.1 is QSPI0_MISO, PA.0 is QSPI0_MOSI*/
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~(SYS_GPA_MFPL_PA3MFP_Msk |
                                       SYS_GPA_MFPL_PA2MFP_Msk |
                                       SYS_GPA_MFPL_PA1MFP_Msk |
                                       SYS_GPA_MFPL_PA0MFP_Msk)) |
                    (SYS_GPA_MFPL_PA3MFP_QSPI0_SS |
                     SYS_GPA_MFPL_PA2MFP_QSPI0_CLK |
                     SYS_GPA_MFPL_PA1MFP_QSPI0_MISO0 |
                     SYS_GPA_MFPL_PA0MFP_QSPI0_MOSI0);

    /* Enable QSPI0 pin schmitt trigger */
    PA->SMTEN |= (GPIO_SMTEN_SMTEN3_Msk | GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN1_Msk | GPIO_SMTEN_SMTEN0_Msk);

    /* Lock protected registers */
    SYS_LockReg();

}

int main(void)
{
    uint32_t cmd_buff[16];

    SYS_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function. Before using FMC function, it should unlock system register first. */
    FMC->ISPCTL |= (FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk);

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    QSPI_Init();
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
            NVIC_DisableIRQ(QSPI0_IRQn);
            memcpy(cmd_buff, spi_rcvbuf, 64);
            bSpiDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
            bISPDataReady = 1;
            NVIC_EnableIRQ(QSPI0_IRQn);
        }
    }

_APROM:
    FMC_SetVectorAddr(FMC_APROM_BASE);
    FMC_SET_APROM_BOOT();
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}

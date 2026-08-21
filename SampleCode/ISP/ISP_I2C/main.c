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


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) != CLK_STATUS_HIRCSTB_Msk);

    /* Switch FMC access cycle to maximum value for safe */
    FMC->CYCCTL = 6;

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);

    /* Switch FMC access cycle to suitable value base on HCLK */
    FMC->CYCCTL = 3;

    /* Enable I2C controller */
    CLK->APBCLK0 |= CLK_APBCLK0_I2C0CKEN_Msk;

    /* Enable GPIO clock */
    CLK->AHBCLK |= CLK_AHBCLK_GPCCKEN_Msk;

    /* Update System Core Clock */
    SystemCoreClock = __HIRC;
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set I2C0 multi-function pins */
    SYS->GPC_MFPL = (SYS->GPC_MFPL & ~(SYS_GPC_MFPL_PC0MFP_Msk | SYS_GPC_MFPL_PC1MFP_Msk)) |
                    (SYS_GPC_MFPL_PC0MFP_I2C0_SDA | SYS_GPC_MFPL_PC1MFP_I2C0_SCL);

    /* I2C pin enable schmitt trigger */
    PC->SMTEN |= (GPIO_SMTEN_SMTEN0_Msk | GPIO_SMTEN_SMTEN1_Msk);

#ifdef ReadyPin
    PB->MODE = (PB->MODE & ~(GPIO_MODE_MODE0_Msk << (6 << 1))) | (GPIO_MODE_OUTPUT << (6 << 1));
    ReadyPin = 1;
#endif

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
    DFMC->ISPCTL |= (DFMC_ISPCTL_ISPEN_Msk | DFMC_ISPCTL_DATAEN_Msk);	

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    I2C_Init();
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            goto _ISP;
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

_ISP:

    while (1)
    {
        if (bI2cDataReady == 1)
        {
            /* Disable I2C IRQ until ParseCmd() is finished to prevent returning incomplete data prematurely */
            NVIC_DisableIRQ(I2C0_IRQn);        	
            memcpy(cmd_buff, i2c_rcvbuf, 64);
            bI2cDataReady = 0;
            ParseCmd((unsigned char *)cmd_buff, 64);
            bISPDataReady = 1;
            NVIC_EnableIRQ(I2C0_IRQn);            
#ifdef ReadyPin
            ReadyPin = 0;
#endif
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}

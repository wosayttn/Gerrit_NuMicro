/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use LXT to trim HIRC.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void TrimHIRC(void);
void IRC_IRQHandler(void);
void SYS_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
void IRC_IRQHandler(void)
{
    if (SYS->HIRCTISTS & SYS_HIRCTISTS_TFAILIF_Msk)  /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTISTS = SYS_HIRCTISTS_TFAILIF_Msk;
    }

    if (SYS->HIRCTISTS & SYS_HIRCTISTS_CLKERRIF_Msk)  /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART0 module clock */
    SetDebugUartCLK();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();

    /* Set PB multi-function pin for CLKO(PB.14) */
    SET_CLKO_PB14();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRC_IRQn);

    /* Trim HIRC to 48MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->HIRCTCTL = 0;
    printf("Disable IRC Trim\n");

    while (1);
}

void TrimHIRC(void)
{
    uint32_t u32TimeOutCnt;

    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTIEN |= (SYS_HIRCTIEN_CLKEIEN_Msk | SYS_HIRCTIEN_TFAILIEN_Msk);
    SYS->HIRCTCTL = (SYS->HIRCTCTL & ~SYS_HIRCTCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while ((SYS->HIRCTISTS & SYS_HIRCTISTS_FREQLOCK_Msk) == 0)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("HIRC Trim failed\n");
            return;
        }
    }

    printf("HIRC Frequency Lock\n");

    /* Clear Trim Lock */
    SYS->HIRCTISTS = SYS_HIRCTISTS_FREQLOCK_Msk;

    /* Unlock protected registers for CLKO clock source setting */
    SYS_UnlockReg();

    /* Enable CLKO and output frequency = HIRC / 2^(1 + 1) */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 1, FALSE);
}

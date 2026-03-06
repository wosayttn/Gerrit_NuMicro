/******************************************************************************
* @file     main.c
* @version  V3.00
* $Revision: 9 $
* $Date: 18/07/16 3:45p $
* @brief    Demonstrate how to use LXT to trim HIRC
*
* @note
* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       HIRC Trim IRQ
 * @param       None
 * @return      None
 * @details     The CLKFAIL_IRQHandler default IRQ
 */
void CLKFAIL_IRQHandler()
{
    if (SYS->IRCTISTS & SYS_IRCTISTS_TFAILIF_Msk)
    {
        /* Get Trim Failure Interrupt */
        /* Display HIRC 12M trim status */
        printf("HIRC12M Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_TFAILIF_Msk;
    }

    if (SYS->IRCTISTS & SYS_IRCTISTS_CLKERRIF_Msk)
    {
        /* Get Clock Error Interrupt */
        /* Display HIRC 12M trim status */
        printf("Clock Error Interrupt\n");
        /* Clear Clock Error Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable External Low speed crystal (LXT) */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for External Low speed clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void TrimHIRC12M()
{
    /*  Enable IRC Trim, set HIRC 12MHz clock and enable interrupt */
    SYS->IRCTIEN |= (SYS_IRCTIEN_CLKEIEN_Msk | SYS_IRCTIEN_TFAILIEN_Msk);
    SYS->IRCTCTL = (SYS->IRCTCTL & ~SYS_IRCTCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC12M Frequency Lock */
    while (1)
    {
        if (SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk)
        {
            printf("HIRC12M Frequency Lock\n");
            SYS->IRCTISTS = SYS_IRCTISTS_FREQLOCK_Msk;     /* Clear Trim Lock */
            break;
        }
    }
}


/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    /* Enable Interrupt */
    NVIC_EnableIRQ(CLKFAIL_IRQn);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Trim HIRC to 12MHz */
    TrimHIRC12M();

    /* Disable IRC Trim */
    SYS->IRCTCTL = 0;
    printf("Disable HIRC12M Trim\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

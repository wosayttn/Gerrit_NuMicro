/******************************************************************************
* @file     main.c
* @version  V3.00
* $Revision: 9 $
* $Date: 18/07/16 3:45p $
* @brief    Demonstrate how to use LXT to trim HIRC48M
*
* @note
* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       HIRC48M Trim IRQ
 * @param       None
 * @return      None
 * @details     The CLKFAIL_IRQHandler default IRQ
 */
void CLKFAIL_IRQHandler()
{
    if (SYS->HIRCTISTS & SYS_HIRCTISTS_TFAILIF_Msk)
    {
        /* Get Trim Failure Interrupt */
        /* Display HIRC48M trim status */
        printf("HIRC48M Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->HIRCTISTS = SYS_HIRCTISTS_TFAILIF_Msk;
    }

    if (SYS->HIRCTISTS & SYS_HIRCTISTS_CLKERRIF_Msk)
    {
        /* Get Clock Error Interrupt */
        /* Display HIRC48M trim status */
        printf("Clock Error Interrupt\n");
        /* Clear Clock Error Interrupt */
        SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk;
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

    /* Enable HIRC48M clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

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

void TrimHIRC48M()
{
    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->HIRCTIEN |= (SYS_HIRCTIEN_CLKEIEN_Msk | SYS_HIRCTIEN_TFAILIEN_Msk);
    SYS->HIRCTCTL = (SYS->HIRCTCTL & ~SYS_HIRCTCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    while (1)
    {
        if (SYS->HIRCTISTS & SYS_HIRCTISTS_FREQLOCK_Msk)
        {
            printf("HIRC48M Frequency Lock\n");
            SYS->HIRCTISTS = SYS_HIRCTISTS_FREQLOCK_Msk;     /* Clear Trim Lock */
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

    /* Trim HIRC48M to 48MHz */
    TrimHIRC48M();

    /* Disable HIRC48M Trim */
    SYS->HIRCTCTL = 0;
    printf("Disable HIRC48M Trim\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

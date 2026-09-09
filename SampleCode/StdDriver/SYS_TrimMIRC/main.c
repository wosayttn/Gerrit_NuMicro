/******************************************************************************
* @file     main.c
* @version  V3.00
* $Revision: 9 $
* $Date: 18/07/16 3:45p $
* @brief    Demonstrate how to use LXT to trim MIRC
*
* @note
* SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/**
 * @brief       MIRC Trim IRQ
 * @param       None
 * @return      None
 * @details     The CLKFAIL_IRQHandler default IRQ
 */
void CLKFAIL_IRQHandler()
{
    if (SYS->MIRCTISTS & SYS_MIRCTISTS_TFAILIF_Msk)
    {
        /* Get Trim Failure Interrupt */
        /* Display MIRC trim status */
        printf("MIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->MIRCTISTS = SYS_MIRCTISTS_TFAILIF_Msk;
    }

    if (SYS->MIRCTISTS & SYS_MIRCTISTS_CLKERRIF_Msk)
    {
        /* Get Clock Error Interrupt */
        /* Display MIRC trim status */
        printf("Clock Error Interrupt\n");
        /* Clear Clock Error Interrupt */
        SYS->MIRCTISTS = SYS_MIRCTISTS_CLKERRIF_Msk;
    }
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Enable LXT */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Waiting for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable MIRC clock & Set MIRC to 16MHz */
    CLK_EnableMIRC(CLK_PWRCTL_MIRCFSEL_16M);

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void TrimMIRC()
{
    /*  Enable IRC Trim, set MIRC 12MHz clock and enable interrupt */
    SYS->MIRCTIEN |= (SYS_MIRCTIEN_CLKEIEN_Msk | SYS_MIRCTIEN_TFAILIEN_Msk);
    SYS->MIRCTCTL = SYS->MIRCTCTL | SYS_MIRCTCTL_AUTRIMEN_Msk;

    CLK_SysTickDelay(2000); /* Waiting for MIRC Frequency Lock */

    /* Get MIRC Frequency Lock */
    while (1)
    {
        if (SYS->MIRCTISTS & SYS_MIRCTISTS_FREQLOCK_Msk)
        {
            printf("MIRC Frequency Lock\n");
            SYS->MIRCTISTS = SYS_MIRCTISTS_FREQLOCK_Msk;     /* Clear Trim Lock */
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

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|       M2U51 MIRC Trim Sample Code      |\n");
    printf("+----------------------------------------+\n");

    /* Enable Interrupt */
    NVIC_EnableIRQ(CLKFAIL_IRQn);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Trim MIRC */
    TrimMIRC();

    /* Disable MIRC Trim */
    SYS->MIRCTCTL = 0;

    printf("Disable MIRC Trim\n");

    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

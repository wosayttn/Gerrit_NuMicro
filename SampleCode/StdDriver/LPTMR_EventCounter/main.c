/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use LPTM0 pin to demonstrates LPTMR event counter function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void LPTMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete !\n");
    LPTMR_ClearIntFlag(LPTMR0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

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

    /* Select LPTMR0 clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Enable IP clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set LPTMR0 event counting pin */
    SYS->GPB_MFP1 = (SYS->GPB_MFP1 & ~(SYS_GPB_MFP1_PB5MFP_Msk)) |
                    (SYS_GPB_MFP1_PB5MFP_LPTM0);

    /* Lock protected registers */
    SYS_LockReg();
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

int main(void)
{
    int i;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code use LPTM0 (PB.5) to count PB.4 input event\n");
    printf("Please connect PB.5 to PB.4, press any key to continue\n");
    getchar();

    PB4 = 1;    /* Set init state to high */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    LPTMR_Open(LPTMR0, LPTMR_ONESHOT_MODE, 100);

    /* Update prescale and compare value to what we need in event counter mode. */
    LPTMR_SET_PRESCALE_VALUE(LPTMR0, 0);
    LPTMR_SET_CMP_VALUE(LPTMR0, 1000);
    /* Counter increase on falling edge */
    LPTMR_EnableEventCounter(LPTMR0, LPTMR_COUNTER_EVENT_FALLING);
    /* Start Timer 0 */
    LPTMR_Start(LPTMR0);
    /* Enable timer interrupt */
    LPTMR_EnableInt(LPTMR0);
    NVIC_EnableIRQ(LPTMR0_IRQn);

    for(i = 0; i < 1000; i++)
    {
        PB4 = 0;
        CLK_SysTickDelay(1);
        PB4 = 1;
        CLK_SysTickDelay(1);
    }

    while(1);
}

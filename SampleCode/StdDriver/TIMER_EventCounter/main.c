/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use TM0 pin to demonstrates timer event counter function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete !\n");
    TIMER_ClearIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select TIMER0 clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable TIMER0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /*----------------------------------------------*/
    /* Init I/O Multi-function                      */
    /*----------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set timer event counting pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_TM0);

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

    printf("\nThis sample code use TM0 (PB.5) to count PB.4 input event\n");
    printf("Please connect PB.5 to PB.4, press any key to continue\n");
    getchar();

    PB4 = 1;    /* Set init state to high */
    GPIO_SetMode(PB, BIT4, GPIO_MODE_OUTPUT);

    /* Give a dummy target frequency here. Will over write prescale and compare value with macro */
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    /* Update prescale and compare value to what we need in event counter mode. */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 1000);
    /* Counter increase on falling edge */
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_EVENT_FALLING);
    /* Start Timer 0 */
    TIMER_Start(TIMER0);
    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    for(i = 0; i < 1000; i++)
    {
        PB4 = 0;
        CLK_SysTickDelay(1);
        PB4 = 1;
        CLK_SysTickDelay(1);
    }

    while(1);
}

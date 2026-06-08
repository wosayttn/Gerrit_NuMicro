/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use TM0 pin to demonstrates timer event counter function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/


void SYS_Init(void)
{
    /*------------------------------------------*/
    /* Init System Clock                        */
    /*------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable TIMER peripheral clock */
    CLK_SetModuleClock(TMR0_MODULE, CLK_TMRSEL0_TMR0SEL_HIRC, 0);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

    /*----------------------------------*/
    /* Init I/O Multi-function          */
    /*----------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set timer event counting pin */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB5MFP_TM0);

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void TMR0_IRQHandler(void)
{
    printf("Count 1000 falling events! Test complete !\n");
    TIMER_ClearIntFlag(TIMER0);
}

void TIMER_FunctionTest()
{
    uint32_t i;

    printf("This sample code use TM0 (PB.5) to count PB.4 input event.\n");
    printf("Please connect PB.5 to PB.4 and press any key to continue ...\n");
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

    /* Enable timer interrupt */
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Start Timer 0 */
    TIMER_Start(TIMER0);

    for(i = 0; i < 1000; i++)
    {
        PB4 = 0;
        CLK_SysTickDelay(1);
        PB4 = 1;
        CLK_SysTickDelay(1);
    }
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to print message */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    /* TIMER function test */
    TIMER_FunctionTest();

    printf("Exit TIMER sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

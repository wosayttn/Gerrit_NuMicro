/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use the timer pin PB.5 to demonstrate inter timer trigger mode function.
 *          Also display the measured input frequency to UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"
/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int complete = 0;

// Timer 0 is working in event count mode, and Timer 1 in capture mode, so we read the
// capture value from Timer 1, not timer 0.
void TIMER1_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    TIMER_ClearCaptureIntFlag(TIMER1);
    // Timer counter value records the duration for 100 event counts.
    printf("Event frequency is %d Hz\n", 100000000 / TIMER_GetCounter(TIMER1) * 100);

    complete = 1;

    while (TIMER_GetCaptureIntFlag(TIMER1))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER1 IntFlag time-out!\n");
            break;
        }
    }
}

static void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /*------------------------------------------------------------------------*/
    /* Init System Clock                                                      */
    /*------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    SetDebugUartCLK();
    /* Select TIMER clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_PCLK0, 0);
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    SET_TM0_PB5();
    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("System core clock = %d\n", SystemCoreClock);
    /* This sample code demonstrate inter timer trigger mode using Timer0 and Timer1.
     * In this mode, Timer0 is working as counter, and triggers Timer1.
     * Using Timer1 to calculate the amount of time used by Timer0 to count specified amount of events.
     * By dividing the time period recorded in Timer1 by the event counts, we get the event frequency.
     */
    printf("Inter timer trigger mode demo code\n");
    printf("Please connect input source with Timer 0 counter pin PB.5, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write prescale and compare value with macro
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 100);

    // Update prescale and compare value. Calculate average frequency every 100 events
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 100);

    // Update Timer 1 prescale value.
    TIMER_SET_PRESCALE_VALUE(TIMER1, 0);

    // We need capture interrupt
    NVIC_EnableIRQ(TIMER1_IRQn);

    while (1)
    {
        complete = 0;
        // Count event by timer 0, disable drop count (set to 0), disable timeout (set to 0). Enable interrupt after complete
        TIMER_EnableFreqCounter(TIMER0, 0, 0, TRUE);

        while (complete == 0);
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

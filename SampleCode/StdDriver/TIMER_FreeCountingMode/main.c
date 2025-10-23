/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Use the timer pin PA.11 to demonstrate timer free counting mode function.
 *           And displays the measured input frequency to UART console.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

void TIMER0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    static int cnt = 0;
    static uint32_t t0, t1;

    TIMER_ClearCaptureIntFlag(TIMER0);

    if (cnt == 0)
    {
        t0 = TIMER_GetCaptureData(TIMER0);
        cnt++;
    }
    else if (cnt == 1)
    {
        t1 = TIMER_GetCaptureData(TIMER0);
        cnt++;

        if (t0 >= t1)
        {
            // over run, drop this data and do nothing
        }
        else
        {
            printf("Input frequency is %dHz\n", 100000000 / (t1 - t0));
        }
    }
    else
    {
        cnt = 0;
    }

    while (TIMER_GetCaptureIntFlag(TIMER0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TIMER0 IntFlag time-out!\n");
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
    /* Enable TIMER module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set timer capture pin */
    SET_TM0_EXT_PA11();
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
    printf("\nThis sample code demonstrate timer free counting mode.\n");
    printf("Please connect input source with Timer 0 capture pin PA.11, press any key to continue\n");
    getchar();

    // Give a dummy target frequency here. Will over write capture resolution with macro
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);

    // Update prescale to set proper resolution.
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);

    // Set compare value as large as possible, so don't need to worry about counter overrun too frequently.
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);

    // Configure Timer 0 free counting mode, capture TDR value on rising edge
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_EVENT_RISING);

    // Start Timer 0
    TIMER_Start(TIMER0);

    // Enable timer interrupt
    TIMER_EnableCaptureInt(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

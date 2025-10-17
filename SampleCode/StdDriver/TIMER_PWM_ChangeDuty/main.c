/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Change duty cycle and period of output waveform in PWM down count type.
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
static volatile uint32_t gu32Period;

void TIMER0_IRQHandler(void)
{
    static uint32_t u32Toggle = 0;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    if (TPWM_GET_PERIOD_INT_FLAG(TIMER0))
    {
        TPWM_CLEAR_PERIOD_INT_FLAG(TIMER0);

        if (u32Toggle == 0)
        {
            /* Set PWM period to generate output frequency 36000 Hz */
            TPWM_SET_PERIOD(TIMER0, ((gu32Period / 2) - 1));

            /* Set PWM duty, 40% */
            TPWM_SET_CMPDAT(TIMER0, (((gu32Period / 2) * 4) / 10));
        }
        else
        {
            /* Set PWM period to generate output frequency 18000 Hz */
            TPWM_SET_PERIOD(TIMER0, (gu32Period - 1));

            /* Set PWM duty, 50% */
            TPWM_SET_CMPDAT(TIMER0, (gu32Period / 2));
        }

        u32Toggle ^= 1;
    }

    while (TPWM_GET_PERIOD_INT_FLAG(TIMER0))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for TPWM0 IntFlag time-out!\n");
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
    CLK_EnableModuleClock(GPB_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set Timer0 PWM CH0(TM0) pin */
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
    printf("+-----------------------------------------------+\n");
    printf("|    Timer PWM Change Duty Cycle Sample Code    |\n");
    printf("+-----------------------------------------------+\n\n");

    printf("# Timer0 PWM_CH0 frequency of first period is 18000 Hz and duty is 50%%.\n");
    printf("# Timer0 PWM_CH0 frequency of second period is 36000 Hz and duty is 40%%.\n");
    printf("# I/O configuration:\n");
    printf("    - Timer0 PWM_CH0 on PB.5\n\n");

    /* Change Timer to PWM counter mode */
    TPWM_ENABLE_PWM_MODE(TIMER0);

    /* Set Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    TPWM_ConfigOutputFreqAndDuty(TIMER0, 18000, 50);

    /* Get initial period and comparator value */
    gu32Period = TPWM_GET_PERIOD(TIMER0) + 1;

    /* Enable output of PWM_CH0 */
    TPWM_ENABLE_OUTPUT(TIMER0, TPWM_CH0);

    /* Enable period event interrupt */
    TPWM_ENABLE_PERIOD_INT(TIMER0);
    NVIC_EnableIRQ(TIMER0_IRQn);

    /* Start Timer PWM counter */
    TPWM_START_COUNTER(TIMER0);

    printf("*** Check Timer0 PWM_CH0 output waveform by oscilloscope ***\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Change duty cycle and period of output waveform by PWM Double Buffer function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_OUT_CH    0UL

/*
    PWM0 channel 0 waveform of this sample shown below(up counter type):

    |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
    |<-  CMR clk ->|                 CMR = 200 CLKs
                                  |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                  |<-CMR->|           CMR = 40 CLKs

     ______________                _______          ____
    |      200     |_____200______|   40  |____60__|     PWM waveform

*/
void PWM0P0_IRQHandler(void)
{
    static int32_t i32Toggle = 0;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Clear channel 0 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM0, PWM_OUT_CH);

    /* Update PWM0 channel 0 period and duty */
    if (i32Toggle == 0)
    {
        PWM_SET_CNR(PWM0, PWM_OUT_CH, 99);
        PWM_SET_CMR(PWM0, PWM_OUT_CH, 40);
    }
    else
    {
        PWM_SET_CNR(PWM0, PWM_OUT_CH, 399);
        PWM_SET_CMR(PWM0, PWM_OUT_CH, 200);
    }

    i32Toggle ^= 1;

    while (PWM_GetPeriodIntFlag(PWM0, PWM_OUT_CH))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM0 IntFlag time-out!\n");
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
    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL2_PWM0SEL_PCLK0, 0);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM0_CH0_PA5();

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

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                    PWM DoubleBuffer Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PA.5)\n");
    printf("\nUse double buffer feature.\n");

    /* PWM0 channel 0 frequency is 160000Hz, duty 50%, */
    PWM_ConfigOutputChannel(PWM0, PWM_OUT_CH, 160000, 50);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM_EnablePeriodInt(PWM0, PWM_OUT_CH, 0);
    NVIC_EnableIRQ(PWM0P0_IRQn);

    /* Start */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

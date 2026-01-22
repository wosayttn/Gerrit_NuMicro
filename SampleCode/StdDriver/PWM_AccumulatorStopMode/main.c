/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate PWM accumulator stop mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_OUT_CH    0UL

void PWM1P0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    PWM_ClearAccInt(PWM1, PWM_OUT_CH);
    printf("Check if output toggles 11 times then stop toggles.\n");

    while (PWM_GetAccInt(PWM1, PWM_OUT_CH))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM1 IntFlag time-out!\n");
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
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPC_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM1_CH0_PC5();
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
    printf("|             PWM AccumulatorStopMode Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code demonstrate PWM1 channel 0 accumulator stop mode.\n");
    printf("  When accumulator interrupt happens, the output of PWM1 channel 0 (PC5) stops.\n");
    printf("  Since interrupt accumulator count is set to 10, the output toggles 11 times then stops. \n");

    printf("\n\nPress any key to start PWM1 channel 0.(PC5)\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the PWM1 Channel 0 as PWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set PWM1 channel 0 output configuration */
    PWM_ConfigOutputChannel(PWM1, PWM_OUT_CH, 300, 30);

    /* Enable PWM Output path for PWM1 channel 0 */
    PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

    /* Enable PWM1 channel 0 accumulator, interrupt count 10, accumulator source select to zero point */
    PWM_EnableAcc(PWM1, PWM_OUT_CH, 10, PWM_IFA_ZERO_POINT);

    /* Enable PWM1 channel 0 accumulator interrupt */
    PWM_EnableAccInt(PWM1, PWM_OUT_CH);

    /* Enable PWM1 channel 0 interrupt in the NVIC interrupt controller */
    NVIC_EnableIRQ(PWM1P0_IRQn);

    /* Enable PWM1 channel 0 accumulator stop mode */
    PWM_EnableAccStopMode(PWM1, PWM_OUT_CH);

    /* Enable Timer for PWM1 channel 0 */
    PWM_Start(PWM1, PWM_CH_0_MASK);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

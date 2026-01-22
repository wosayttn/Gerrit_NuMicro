/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use PWM counter synchronous start function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

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
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL2_PWM1SEL_PCLK1, 0);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPA_MODULE);
    CLK_EnableModuleClock(GPC_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM0_CH0_PA5();
    SET_PWM0_CH1_PA4();
    SET_PWM0_CH2_PA3();
    SET_PWM0_CH3_PA2();
    SET_PWM0_CH4_PA1();
    SET_PWM0_CH5_PA0();

    SET_PWM1_CH0_PC5();
    SET_PWM1_CH1_PC4();
    SET_PWM1_CH2_PC3();
    SET_PWM1_CH3_PC2();
    SET_PWM1_CH4_PC1();
    SET_PWM1_CH5_PC0();

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
    printf("|                      PWM SyncStart Sample Code                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PA.5), PWM0_CH1(PA.4), PWM0_CH2(PA.3), PWM0_CH3(PA.2), PWM0_CH4(PA.1), PWM0_CH5(PA.0)\n");
    printf("                         PWM1_CH0(PC.5), PWM1_CH1(PC.4), PWM1_CH2(PC.3), PWM1_CH3(PC.2), PWM1_CH4(PC.1), PWM1_CH5(PC.0)\n");


    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */
    PWM_ConfigOutputChannel(PWM0, 0, 1000, 50);
    PWM_ConfigOutputChannel(PWM0, 1, 1000, 50);
    PWM_ConfigOutputChannel(PWM0, 2, 1000, 50);
    PWM_ConfigOutputChannel(PWM0, 3, 1000, 50);
    PWM_ConfigOutputChannel(PWM0, 4, 1000, 50);
    PWM_ConfigOutputChannel(PWM0, 5, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 0, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 1, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 2, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 3, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 4, 1000, 50);
    PWM_ConfigOutputChannel(PWM1, 5, 1000, 50);

    /* Enable counter synchronous start function for PWM0 and PWM1 channel 0~5 */
    PWM_ENABLE_TIMER_SYNC(PWM0, 0x3F, PWM_SSCTL_SSRC_PWM0);
    PWM_ENABLE_TIMER_SYNC(PWM1, 0x3F, PWM_SSCTL_SSRC_PWM0);

    /* Enable output of PWM0 and PWM1 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);
    PWM_EnableOutput(PWM1, 0x3F);

    printf("Press any key to start.\n");
    getchar();

    /* Trigger PWM counter synchronous start by PWM0 */
    PWM_TRIGGER_SYNC_START(PWM0);

    printf("Done.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

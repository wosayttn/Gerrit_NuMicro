/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use PWM counter output waveform.
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
    printf("|                  PWM OutputWaveform Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with PWM0 and PWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  PWM0 channel 0: 360000 Hz, duty 90%%.\n");
    printf("  PWM0 channel 1: 320000 Hz, duty 80%%.\n");
    printf("  PWM0 channel 2: 250000 Hz, duty 75%%.\n");
    printf("  PWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  PWM0 channel 4: 160000 Hz, duty 60%%.\n");
    printf("  PWM0 channel 5: 150000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 0: 120000 Hz, duty 50%%.\n");
    printf("  PWM1 channel 1: 100000 Hz, duty 40%%.\n");
    printf("  PWM1 channel 2:  90000 Hz, duty 30%%.\n");
    printf("  PWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  PWM1 channel 4:  45000 Hz, duty 20%%.\n");
    printf("  PWM1 channel 5:  30000 Hz, duty 10%%.\n");
    printf("    waveform output pin: PWM0_CH0(PA.5), PWM0_CH1(PA.4), PWM0_CH2(PA.3), PWM0_CH3(PA.2), PWM0_CH4(PA.1), PWM0_CH5(PA.0)\n");
    printf("                         PWM1_CH0(PC.5), PWM1_CH1(PC.4), PWM1_CH2(PC.3), PWM1_CH3(PC.2), PWM1_CH4(PC.1), PWM1_CH5(PC.0)\n");

    /* PWM0 and PWM1 channel 0~5 frequency and duty configuration are as follows */
    PWM_ConfigOutputChannel(PWM0, 0, 360000, 90);
    PWM_ConfigOutputChannel(PWM0, 1, 320000, 80);
    PWM_ConfigOutputChannel(PWM0, 2, 250000, 75);
    PWM_ConfigOutputChannel(PWM0, 3, 180000, 70);
    PWM_ConfigOutputChannel(PWM0, 4, 160000, 60);
    PWM_ConfigOutputChannel(PWM0, 5, 150000, 50);
    PWM_ConfigOutputChannel(PWM1, 0, 120000, 50);
    PWM_ConfigOutputChannel(PWM1, 1, 100000, 40);
    PWM_ConfigOutputChannel(PWM1, 2, 90000, 30);
    PWM_ConfigOutputChannel(PWM1, 3, 60000, 25);
    PWM_ConfigOutputChannel(PWM1, 4, 45000, 20);
    PWM_ConfigOutputChannel(PWM1, 5, 30000, 10);

    /* Enable output of PWM0 and PWM1 channel 0~5 */
    PWM_EnableOutput(PWM0, 0x3F);
    PWM_EnableOutput(PWM1, 0x3F);

    /* Start PWM0 counter */
    PWM_Start(PWM0, 0x3F);
    /* Start PWM1 counter */
    PWM_Start(PWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Stop PWM0 counter */
    PWM_ForceStop(PWM0, 0x3F);
    /* Stop PWM1 counter */
    PWM_ForceStop(PWM1, 0x3F);

    printf("Done.\n");

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use PWM Dead Zone function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_CH01_DZ_CH    0UL
#define PWM_CH23_DZ_CH    2UL

void PWM0P0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Clear channel 0 period interrupt flag */
    PWM_ClearPeriodIntFlag(PWM0, PWM_CH01_DZ_CH);

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if (++cnt == 100)
    {
        if (out)
            PWM_EnableOutput(PWM0, PWM_CH_0_MASK | PWM_CH_1_MASK | PWM_CH_2_MASK | PWM_CH_3_MASK);
        else
            PWM_DisableOutput(PWM0, PWM_CH_0_MASK | PWM_CH_1_MASK | PWM_CH_2_MASK | PWM_CH_3_MASK);

        out ^= 1;
        cnt = 0;
    }

    while (PWM_GetPeriodIntFlag(PWM0, PWM_CH01_DZ_CH))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM PeriodIntFlag time-out!\n");
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
    SET_PWM0_CH1_PA4();
    SET_PWM0_CH2_PA3();
    SET_PWM0_CH3_PA2();

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
    printf("|                        PWM DeadTime Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PA.5), PWM0_CH1(PA.4), PWM0_CH2(PA.3), PWM0_CH3(PA.2)\n");

    /* Set Pwm mode as complementary mode */
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /* PWM0 channel 0 frequency is 100Hz, duty 30% */
    PWM_ConfigOutputChannel(PWM0, PWM_CH01_DZ_CH, 100, 30);

    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, PWM_CH01_DZ_CH, 400);
    SYS_LockReg();

    /* PWM0 channel 2 frequency is 3000Hz, duty 50% */
    PWM_ConfigOutputChannel(PWM0, PWM_CH23_DZ_CH, 3000, 50);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, PWM_CH23_DZ_CH, 200);
    SYS_LockReg();

    /* Enable output of PWM0 channel 0~3 */
    PWM_EnableOutput(PWM0, 0xF);

    /* Enable PWM0 channel 0 period interrupt, use channel 0 to measure time. */
    PWM_EnablePeriodInt(PWM0, PWM_CH01_DZ_CH, 0);
    NVIC_EnableIRQ(PWM0P0_IRQn);

    /* Start */
    PWM_Start(PWM0, 0xF);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

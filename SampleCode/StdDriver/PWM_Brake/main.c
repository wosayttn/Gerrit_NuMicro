/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Demonstrate how to use PWM brake function.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_OUT_CH    0UL
#define PWM_BRAKE_PIN 0UL

void BRAKE0_IRQHandler(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (PWM0 channel 0 output will toggle again)\n");
    getchar();

    SYS_UnlockReg();
    /* Disable brake and interrupt */
    (PWM0)->BRKCTL[(PWM_OUT_CH >> 1UL)] = 0;
    PWM_DisableFaultBrakeInt(PWM0, PWM_FB_EDGE);
    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);

    while (PWM_GetFaultBrakeIntFlag(PWM0, PWM_FB_EDGE))
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM0 IntFlag time-out!\n");
            break;
        }
    }

    SYS_LockReg();
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
    CLK_EnableModuleClock(GPD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for PWM */
    SET_PWM0_CH0_PA5();

    /* Set multi-function pin for PWM brake pin */
    SET_PWM0_BRAKE0_PB1();
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
    printf("\nConnet PB.1 (PWM0 brake pin 0) to PD.5.\n");
    printf("It will generate brake interrupt and PWM0 channel 0 output PA.5 stop toggling.\n");

    GPIO_SetMode(PD, BIT5, GPIO_MODE_OUTPUT);
    PD5 = 0;

    /* PWM0 frequency is 100Hz, duty 30%, */
    PWM_ConfigOutputChannel(PWM0, PWM_OUT_CH, 100, 30);

    /* Enable output of all PWM channels */
    PWM_EnableOutput(PWM0, 0x3F);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable brake and interrupt */
    PWM_EnableFaultBrake(PWM0, PWM_CH_0_MASK, 1, PWM_FB_EDGE_BKP0);
    PWM_EnableFaultBrakeInt(PWM0, PWM_FB_EDGE);
    /* Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128 */
    PWM_EnableBrakeNoiseFilter(PWM0, PWM_BRAKE_PIN, 7, PWM_NF_CLK_DIV_128);
    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);
    SYS_LockReg();

    NVIC_EnableIRQ(BRAKE0_IRQn);

    /* Start */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD5 = 1;

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

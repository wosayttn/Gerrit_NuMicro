/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Change duty cycle of output waveform by configured period.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define BPWM_OUT_CH    0UL

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

    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);
    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    /* Enable GPIO clock */
    CLK_EnableModuleClock(GPE_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    SetDebugUartMFP();
    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PE2();

    /* Lock protected registers */
    SYS_LockReg();
}

/**
 * @brief       Calculate the comparator value of new duty by configured period
 *
 * @param       bpwm                 The pointer of the specified BPWM module
 *
 * @param       u32ChannelNum        BPWM channel number. Valid values are between 0~5
 *
 * @param       u32DutyCycle         Target generator duty cycle percentage. Valid range are between 0 ~ u32CycleResolution.
 *                                   If u32CycleResolution is 100, and u32DutyCycle is 10 means 10%, 20 means 20% ...
 *
 * @param       u32CycleResolution   Target generator duty cycle resolution. The value in general is 100.
 *
 * @return      The compatator value by new duty cycle
 */
uint32_t CalNewDutyCMR(BPWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
    NVT_UNUSED(u32ChannelNum);

    return (u32DutyCycle * (BPWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution);
}

int main(void)
{
    uint8_t  u8Option;
    uint32_t u32NewDutyCycle = 0, u32NewCMR = 0;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                      BPWM SwitchDuty Sample Code                                  |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PE.2)\n");
    printf("\nOutput waveform is 1800Hz and it's duty is 50%%.\n");

    /* BPWM0 channel 0 frequency is 1800Hz, duty 50%, */
    BPWM_ConfigOutputChannel(BPWM0, BPWM_OUT_CH, 1800, 50);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Start BPWM counter */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    while (1)
    {
        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = (uint8_t)getchar();

        if (u8Option == '1')
        {
            u32NewDutyCycle = 100;
        }
        else if (u8Option == '2')
        {
            u32NewDutyCycle = 75;
        }
        else if (u8Option == '3')
        {
            u32NewDutyCycle = 25;
        }
        else if (u8Option == '4')
        {
            u32NewDutyCycle = 0;
        }
        else
        {
            printf("Exit\n");
            break;
        }

        /* Get new comparator value by call CalNewDutyCMR() */
        u32NewCMR = CalNewDutyCMR(BPWM0, BPWM_OUT_CH, u32NewDutyCycle, 100);
        /* Set new comparator value to register */
        BPWM_SET_CMR(BPWM0, BPWM_OUT_CH, u32NewCMR);
    }

    /* Stop BPWM counter */
    BPWM_Stop(BPWM0, BPWM_CH_0_MASK);
    /* Disable output of BPWM0 channel 0 */
    BPWM_DisableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Got no where to go, just loop forever */
    while (1) ;
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

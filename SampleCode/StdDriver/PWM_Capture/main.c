/**************************************************************************//**
 * @file    main.c
 * @version V1.00
 * @brief   Capture the PWM1 Channel 0 waveform by PWM1 Channel 2.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

#define PWM_CAP_CH    2UL
#define PWM_OUT_CH    0UL

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* au32Count[4] : Keep the internal counter value when input signal rising / falling    */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
static int32_t CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t au16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime;
    uint16_t u16FallingTime;
    uint16_t u16HighPeriod;
    uint16_t u16LowPeriod;
    uint16_t u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    /* Wait for Capture Falling Indicator  */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while (PWM_GetCaptureIntFlag(PWM, u32Ch) < 2U)//0 No capture, 1 Rising, 2 Falling, 3 Rising and falling
    {
        if (--u32TimeOutCnt == 0U)
        {
            printf("Wait for PWM Capture Falling Indicator time-out!\n");
            return (-1);
        }
    }

    /* Clear Capture Falling Indicator (Time B)*/
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0U;

    while (u32i < 4U)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (PWM_GetCaptureIntFlag(PWM, u32Ch) < 2U)//0 No capture, 1 Rising, 2 Falling, 3 Rising and falling
        {
            if (--u32TimeOutCnt == 0U)
            {
                printf("Wait for PWM Capture Falling Indicator time-out!\n");
                return (-1);
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        au16Count[u32i] = (uint16_t)PWM_GET_CAPTURE_FALLING_DATA(PWM, u32Ch);
        u32i++;

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (PWM_GetCaptureIntFlag(PWM, u32Ch) < 1U)//0 No capture, 1 Rising, 2 Falling, 3 Rising and falling
        {
            if (--u32TimeOutCnt == 0U)
            {
                printf("Wait for PWM Capture Rising Indicator time-out!\n");
                return (-1);
            }
        }

        /* Clear Capture Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        au16Count[u32i] = (uint16_t)PWM_GET_CAPTURE_RISING_DATA(PWM, u32Ch);
        u32i++;
    }

    u16RisingTime = au16Count[1];

    u16FallingTime = au16Count[0];

    u16HighPeriod = au16Count[1] - au16Count[2];

    u16LowPeriod = (uint16_t)(0x10000U - au16Count[3]);

    u16TotalPeriod = (uint16_t)(0x10000U - au16Count[2]);

    printf("\nPWM generate: \nHigh Period=17279 ~ 17281, Low Period=40319 ~ 40321, Total Period=57599 ~ 57601\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 17279U) || (u16HighPeriod > 17281U) || (u16LowPeriod < 40319U) || (u16LowPeriod > 40321U) || (u16TotalPeriod < 57599U) || (u16TotalPeriod > 57601U))
    {
        printf("Capture Test Fail!!\n");
        return (-1);
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
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
    SET_PWM1_CH2_PC3();

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    uint32_t u32ExitLoop = 0U;
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init Debug UART to 115200-8N1 for print message */
    InitDebugUart();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

    printf("\n\nCPU @ %uHz(PLL@ %uHz)\n", CLK_GetHCLKFreq(), CLK_GetPLLClockFreq());
    printf("+------------------------------------------------------------------------+\n");
    printf("|                         PWM Capture Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM1 channel 2 to capture\n  the signal from PWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    PWM1 channel 2(PC.3) <--> PWM1 channel 0(PC.5)\n\n");
    printf("Use PWM1 Channel 2(PC.3) to capture the PWM1 Channel 0(PC.5) Waveform\n");

    while (u32ExitLoop == 0U)
    {
        printf("\n\nPress any key to start PWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 Channel 0 as PWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.(up counter type)
           duty ratio = (CMR)/(CNR+1)
           cycle time = CNR+1
           High level = CMR
           PWM clock source frequency = CLK_GetPCLK1Freq()
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = (PWM clock source frequency/prescaler/PWM output frequency) - 1
           duty ratio = 30% ==> (CMR)/(CNR+1) = 30%
           CMR = (CNR+1) * 30 / 100
           Prescale value is 4 : prescaler= 5
        */

        /* Set PWM1 channel 0 output configuration */
        PWM_ConfigOutputChannel(PWM1, PWM_OUT_CH, 250, 30);

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

        /* Enable Timer for PWM1 channel 0 */
        PWM_Start(PWM1, PWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 channel 2 for capture function                                         */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = CLK_GetPCLKFreq() in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1000000000/Capture clock source frequency*prescaler
        */

        /* Set PWM1 channel 2 capture configuration */
        if (CLK_GetPCLK1Freq() != 0U)
        {
            PWM_ConfigCaptureChannel(PWM1, PWM_CAP_CH, 1000000000UL / CLK_GetPCLK1Freq() * 5U, 0U);
        }

        PWM_SET_PRESCALER(PWM1, PWM_CAP_CH, PWM_GET_PRESCALER(PWM1, PWM_OUT_CH));

        /* Enable Timer for PWM1 channel 2 */
        PWM_Start(PWM1, PWM_CH_2_MASK);

        /* Enable Capture Function for PWM1 channel 2 */
        PWM_EnableCapture(PWM1, PWM_CH_2_MASK);

        /* Enable falling capture reload */
        PWM1->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

        /* Wait until PWM1 channel 2 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

        while (((PWM1->CNT[PWM_CAP_CH]) == 0U) && (u32ExitLoop == 0U))
        {
            if (--u32TimeOutCnt == 0U)
            {
                printf("Wait for PWM timer start to count time-out!\n");
                u32ExitLoop = 1U;
            }
        }

        /* Capture the Input Waveform Data */
        if ((u32ExitLoop == 0U) && (CalPeriodTime(PWM1, PWM_CAP_CH) < 0))
        {
            u32ExitLoop = 1U;
        }

        if (u32ExitLoop == 0U)
        {
            /*------------------------------------------------------------------------------------------------------------*/
            /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                      */
            /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
            /*------------------------------------------------------------------------------------------------------------*/

            /* Set PWM1 channel 0 loaded value as 0 */
            PWM_Stop(PWM1, PWM_CH_0_MASK);

            /* Wait until PWM1 channel 0 Timer Stop */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (((PWM1->CNT[PWM_OUT_CH] & PWM_CNT_CNT_Msk) != 0U) && (u32ExitLoop == 0U))
            {
                if (--u32TimeOutCnt == 0U)
                {
                    printf("Wait for PWM timer stop time-out!\n");
                    u32ExitLoop = 1U;
                }
            }

            /* Disable Timer for PWM1 channel 0 */
            PWM_ForceStop(PWM1, PWM_CH_0_MASK);

            /* Disable PWM Output path for PWM1 channel 0 */
            PWM_DisableOutput(PWM1, PWM_CH_0_MASK);

            /*------------------------------------------------------------------------------------------------------------*/
            /* Stop PWM1 channel 2 (Recommended procedure method 1)                                                      */
            /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
            /*------------------------------------------------------------------------------------------------------------*/

            /* Set loaded value as 0 for PWM1 channel 2 */
            PWM_Stop(PWM1, PWM_CH_2_MASK);

            /* Wait until PWM1 channel 2 current counter reach to 0 */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

            while (((PWM1->CNT[PWM_CAP_CH] & PWM_CNT_CNT_Msk) != 0U) && (u32ExitLoop == 0U))
            {
                if (--u32TimeOutCnt == 0U)
                {
                    printf("Wait for PWM current counter reach to 0 time-out!\n");
                    u32ExitLoop = 1U;
                }
            }

            /* Disable Timer for PWM1 channel 2 */
            PWM_ForceStop(PWM1, PWM_CH_2_MASK);

            /* Disable Capture Function and Capture Input path for PWM1 channel 2 */
            PWM_DisableCapture(PWM1, PWM_CH_2_MASK);

            /* Clear Capture Interrupt flag for PWM1 channel 2 */
            PWM_ClearCaptureIntFlag(PWM1, PWM_CAP_CH, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);
        }
    }

    /* Got no where to go, just loop forever */
    while (1)
    {
    }
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

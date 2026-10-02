/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Use BPWM0 Channel 0(PA.0) to capture the Timer0 (PB.5) Waveform.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u16Count[4] : Keep the internal counter value when input signal rising / falling     */
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
void CalPeriodTime(BPWM_T *BPWM, uint32_t u32Ch)
{
    uint16_t u16Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCount;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for Capture Falling Indicator  */
    while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
    {
        if (u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");

            while (1);
        }

        u32TimeOutCount--;
    }

    /* Clear Capture Falling Indicator (Time B)*/
    BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    for (u32i = 0 ; u32i < 4 ;)
    {
        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait for Capture Falling Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 2)
        {
            if (u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");

                while (1);
            }

            u32TimeOutCount--;
        }

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(BPWM, u32Ch);

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait for Capture Rising Indicator */
        while (BPWM_GetCaptureIntFlag(BPWM, u32Ch) < 1)
        {
            if (u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");

                while (1);
            }

            u32TimeOutCount--;
        }

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(BPWM, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u16Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(BPWM, u32Ch);
    }

    u16RisingTime = u16Count[1];

    u16FallingTime = u16Count[0];

    u16HighPeriod = u16Count[1] - u16Count[2];

    u16LowPeriod = 0x10000 - u16Count[1];

    u16TotalPeriod = 0x10000 - u16Count[2];

    printf("\nPWM generate: \nHigh Period=9999 ~ 10001, Low Period=9999 ~ 10001, Total Period=19998 ~ 20002\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);

    if ((u16HighPeriod < 9999) || (u16HighPeriod > 10001) || (u16LowPeriod < 9999) || (u16LowPeriod > 10001) || (u16TotalPeriod < 19999) || (u16TotalPeriod > 20001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch the core clock to 40 MHz from the MIRC */
    CLK_SetCoreClock(FREQ_40MHZ);

    /* Set both PCLK0 and PCLK1 as HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Switch UART0 clock source to HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA.0 multi-function pin for BPWM0 channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;

    /* Set PB.5 multi-function pin for TIMER0 toggle out */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB5MFP_Msk) | SYS_GPB_MFPL_PB5MFP_TM0;

    /* Lock protected registers */
    SYS_LockReg();
}


void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init Debug UART */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to capture\n  the signal from TIMER0 toggle out pin.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(PA.0 BPWM0 channel 0) <--> TM0(PB.5 TIMER0 toggle out pin)\n\n");
    printf("Use BPWM0 Channel 0(PA.0) to capture the TIMER0 toggle out pin(PB.5) Waveform\n");

    while (1)
    {
        printf("Press any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the TM0 as BPWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* To generate 500 HZ toggle output, timer frequency must set to 1000 Hz.
           Because toggle output state change on every timer timeout event */
        if (TIMER_Open(TIMER0, TIMER_TOGGLE_MODE, 1000) != 1000)
        {
            printf("Set the frequency different from the user\n");
        }

        /* Start Timer Counting */
        TIMER_Start(TIMER0);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 500 Hz, user can calculate capture settings by follows.
           Capture clock source frequency = PCLK = 40000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 40000000/4/500 = 20000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */
        /* set BPWM0 channel 0 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, 0, 80, 0);

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, BPWM_CH_0_MASK);

        /* Enable Capture Function for BPWM0 channel 0 */
        BPWM_EnableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Enable falling capture reload */
        BPWM0->CAPCTL |= BPWM_CAPCTL_FCRLDEN0_Msk;

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until BPWM0 channel 0 Timer start to count */
        while ((BPWM0->CNT) == 0)
        {
            if (u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");

                while (1);
            }

            u32TimeOutCount--;
        }

        /* Capture the Input Waveform Data */
        CalPeriodTime(BPWM0, 0);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Disable Timer0 Counting */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Stop Timer Counting */
        TIMER_Stop(TIMER0);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set BPWM Timer loaded value(Period) as 0. When BPWM internal counter(CNT) reaches to 0, disable BPWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for BPWM0 channel 0 */
        BPWM_Stop(BPWM0, BPWM_CH_0_MASK);

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until BPWM0 channel 0 current counter reach to 0 */
        while ((BPWM0->CNT & BPWM_CNT_CNT_Msk) != 0)
        {
            if (u32TimeOutCount == 0)
            {
                printf("\nSomething is wrong, please check if pin connection is correct. \n");

                while (1);
            }

            u32TimeOutCount--;
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, BPWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 0 */
        BPWM_DisableCapture(BPWM0, BPWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for BPWM0 channel 0 */
        BPWM_ClearCaptureIntFlag(BPWM0, 0, BPWM_CAPTURE_INT_FALLING_LATCH);
    }
}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 18/07/19 2:15p $
 * @brief    Capture the PWM0 Channel 0 waveform by PWM0 Channel 2.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_CLK             20000000
#define PWM_OUTPUT_FREQ     250
#define PWM_OUTPUT_DUTY     30
#define PWM_HIGH            (PWM_CLK/PWM_OUTPUT_FREQ)*PWM_OUTPUT_DUTY/100/2
#define PWM_LOW             (PWM_CLK/PWM_OUTPUT_FREQ)*(100-PWM_OUTPUT_DUTY)/100/2
#define PWM_PERIOD          PWM_HIGH+PWM_LOW
#define DEVIATION           2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
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
void CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCount;

    /* Clear Capture Falling Indicator (Time A) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    /* setup timeout */
    u32TimeOutCount = SystemCoreClock;

    /* Wait for Capture Falling Indicator  */
    while((PWM0->CAPIF & PWM_CAPIF_CFLIF2_Msk) == 0)
    {
        if(u32TimeOutCount == 0)
        {
            printf("\nSomething is wrong, please check if pin connection is correct. \n");
            while(1);
        }
        u32TimeOutCount--;
    }

    /* Clear Capture Falling Indicator (Time B)*/
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Falling and Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2);

        /* Clear Capture Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeriod = u32Count[1] - u32Count[2];

    u16LowPeriod = 0x10000 - u32Count[1];

    u16TotalPeriod = 0x10000 - u32Count[2];

    printf("\nPWM generate: \nHigh Period=%d, Low Period=%d, Total Period=%d\n", PWM_HIGH, PWM_LOW, PWM_PERIOD);
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d\n \
               High Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < (PWM_HIGH-DEVIATION)) || (u16HighPeriod > (PWM_HIGH+DEVIATION))
            || (u16LowPeriod < (uint16_t)(PWM_LOW-DEVIATION)) || (u16LowPeriod > (uint16_t)(PWM_LOW+DEVIATION))
            || (u16TotalPeriod < (uint16_t)(PWM_PERIOD-DEVIATION)) || (u16TotalPeriod > (uint16_t)(PWM_PERIOD+DEVIATION)))
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

    /* Switch the core clock to 40MHz from the MIRC */
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

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for PWM0 Channel 0/2 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA3MFP_Msk) | SYS_GPA_MFPL_PA3MFP_PWM0_CH2;
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

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 2 to capture\n  the signal from PWM0 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    PWM0 channel 2(PA.3) <--> PWM0 channel 0(PA.5)\n\n");
    printf("Use PWM0 Channel 2(PA.3) to capture the PWM0 Channel 0(PA.5) Waveform\n");

    while(1)
    {
        printf("\n\nPress any key to start PWM Capture Test\n");
        getchar();

        /*-------------------- ------------------------------------------------------------------*/
        /* Set the PWM0 Channel 0 as PWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency from MIRC/2 is 20,000,000
           (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
                   = 20,000,000/2/250 = 40,000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 40,000
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 12,000
           Prescale value is 1 : prescaler= 2
        */

        /* set PWM0 channel 0 output configuration */
        PWM_ConfigOutputChannel(PWM0, 0, PWM_OUTPUT_FREQ, PWM_OUTPUT_DUTY);

        /* Enable PWM Output path for PWM0 channel 0 */
        PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

        /* Enable Timer for PWM0 channel 0 */
        PWM_Start(PWM0, PWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM0 channel 2 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/
        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = MIRC/2 = 20,000,000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/minimum input frequency
                   = 20,000,000/2/250 = 40,000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)

           Capture unit time = 1/Capture clock source frequency/prescaler
           100.0ns = 1/20,000,000/2
        */

        /* set PWM0 channel 2 capture configuration */
        PWM_ConfigCaptureChannel(PWM0, 2, 100, 0);

        /* Enable Timer for PWM0 channel 2 */
        PWM_Start(PWM0, PWM_CH_2_MASK);

        /* Enable Capture Function for PWM0 channel 2 */
        PWM_EnableCapture(PWM0, PWM_CH_2_MASK);

        /* Enable falling capture reload */
        PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN2_Msk;

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until PWM0 channel 2 Timer start to count */
        while((PWM0->CNT[2]) == 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("PWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Capture the Input Waveform Data */
        CalPeriodTime(PWM0, 2);
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/
        /* Set PWM0 channel 0 loaded value as 0 */
        PWM_Stop(PWM0, PWM_CH_0_MASK);

        /* Wait until PWM0 channel 0 Timer Stop */
        while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("PWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for PWM0 channel 0 */
        PWM_ForceStop(PWM0, PWM_CH_0_MASK);

        /* Disable PWM Output path for PWM0 channel 0 */
        PWM_DisableOutput(PWM0, PWM_CH_0_MASK);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM0 channel 2 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set loaded value as 0 for PWM0 channel 2 */
        PWM_Stop(PWM0, PWM_CH_2_MASK);

        /* setup timeout */
        u32TimeOutCount = SystemCoreClock;

        /* Wait until PWM0 channel 2 current counter reach to 0 */
        while((PWM0->CNT[2] & PWM_CNT_CNT_Msk) != 0)
        {
            if(u32TimeOutCount == 0)
            {
                printf("PWM encounters some errors, please check it. \n");
                while(1);
            }
            u32TimeOutCount--;
        }

        /* Disable Timer for PWM0 channel 2 */
        PWM_ForceStop(PWM0, PWM_CH_2_MASK);

        /* Disable Capture Function and Capture Input path for  PWM0 channel 2*/
        PWM_DisableCapture(PWM0, PWM_CH_2_MASK);

        /* Clear Capture Interrupt flag for PWM0 channel 2 */
        PWM_ClearCaptureIntFlag(PWM0, 2, PWM_CAPTURE_INT_FALLING_LATCH);
    }
}

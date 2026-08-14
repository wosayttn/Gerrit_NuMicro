/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 18/07/17 6:05p $
 * @brief
 *           Change duty cycle of output waveform by configured period.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


uint32_t CalNewDutyCMR(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution);

/**
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0_IRQHandler(void)
{
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

    /* Set PA multi-function pins for PWM0 Channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;
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

/**
 * @brief       Calculate the comparator value of new duty by configured period
 *
 * @param       pwm                  The pointer of the specified PWM module
 *
 * @param       u32ChannelNum        PWM channel number. Valid values are between 0~5
 *
 * @param       u32DutyCycle         Target generator duty cycle percentage. Valid range are between 0 ~ u32CycleResolution.
 *                                   If u32CycleResolution is 100, and u32DutyCycle is 10 means 10%, 20 means 20% ...
 *
 * @param       u32CycleResolution   Target generator duty cycle resolution. The value in general is 100.
 *
 * @return      The compatator value by new duty cycle
 */
uint32_t CalNewDutyCMR(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
//    return (u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum) + 1) / u32CycleResolution);
    return (u32DutyCycle * (PWM_GET_CNR(pwm, u32ChannelNum)) / u32CycleResolution);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t  u8Option;
    uint32_t u32NewCMR = 0;

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

    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                                   |\n");
    printf("|                                                                                   |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PA.5)\n");
    printf("\nOutput waveform is 1000 Hz and it's duty is 50%%. \n");

    /*
      Configure PWM0 channel 0 init period and duty(up counter type).
      Period is MIRC/2/(prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 20 MHz / (1 * (19,999 + 1)) = 1000 Hz
      Duty ratio = (10,000) / (19,999 + 1) = 50%
    */

    /* PWM0 channel 0 frequency is 1000 Hz, duty 50%, */
    PWM_ConfigOutputChannel(PWM0, 0, 1000, 50);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* ZeroLevel: Low, CmpUpLevel: High, PeriodLevel: nothing, CmpDownLevel: nothing */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING);

    /* Start PWM counter */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    while(1)
    {
        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = getchar();

        if(u8Option == '1')
        {
            u32NewCMR = 0;
        }
        else if(u8Option == '2')
        {
            u32NewCMR = PWM_GET_CNR(PWM0, 0);
            u32NewCMR = u32NewCMR * (100-75)/100;
        }
        else if(u8Option == '3')
        {
            u32NewCMR = PWM_GET_CNR(PWM0, 0);
            u32NewCMR = u32NewCMR * (100-25)/100;
        }
        else if(u8Option == '4')
        {
            u32NewCMR = PWM_GET_CNR(PWM0, 0) + 1;
        }
        else
        {
            printf("Exit\n");
            break;
        }
        /* Set new comparator value to register */
        PWM_SET_CMR(PWM0, 0, u32NewCMR);
    }

    /* Stop PWM counter */
    PWM_Stop(PWM0, PWM_CH_0_MASK);

    /* Disable output of PWM0 channel 0 */
    PWM_DisableOutput(PWM0, PWM_CH_0_MASK);

    while(1);

}

/**************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Change duty cycle of output waveform by configured period.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


uint32_t CalNewDutyCMR(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution);

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

    /* Set both PCLK0 and PCLK1 as HCLK */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

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

    /* Set PA multi-function pins for BPWM0 Channel0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & (~SYS_GPA_MFPL_PA0MFP_Msk)) | SYS_GPA_MFPL_PA0MFP_BPWM0_CH0;
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
 * @param       bpwm                  The pointer of the specified BPWM module
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
uint32_t CalNewDutyCMR(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32DutyCycle, uint32_t u32CycleResolution)
{
    if (u32DutyCycle)
        return (u32DutyCycle * (BPWM_GET_CNR(bpwm, u32ChannelNum)) / u32CycleResolution);
    else
        return (BPWM_GET_CNR(bpwm, u32ChannelNum) + 1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32NewDutyCycle = 0;

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
    printf("|                          BPWM Driver Sample Code                                  |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform, and switch duty cycle.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PA.0)\n");
    printf("\nOutput waveform is 1250Hz and it's duty is 50%%.\n");

    /*
      Configure BPWM0 channel 0 init period and duty(down counting type).
      Period is HIRC_Freq / (prescaler * (CNR + 1))
      Duty ratio = (CMR) / (CNR + 1)
      Period = 40 MHz / (1 * (31999 + 1)) = 1250 Hz
      Duty ratio = (16000) / (31399 + 1) = 50%
    */

    /* BPWM0 channel 0 frequency is 1250Hz, duty 50%, */
    BPWM_ConfigOutputChannel(BPWM0, 0, 1250, 50);

    /* Enable output of BPWM0 channel 0 */
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    /* Start BPWM counter */
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    while (1)
    {
        uint8_t  u8Option;
        uint32_t u32NewCMR;

        printf("\nSelect new duty: \n");
        printf("[1] 100%% \n");
        printf("[2] 75%% \n");
        printf("[3] 25%% \n");
        printf("[4] 0%% \n");
        printf("[Other] Exit \n");
        u8Option = getchar();
        printf("Select : %d \n", u8Option - 48);

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
        u32NewCMR = CalNewDutyCMR(BPWM0, 0, u32NewDutyCycle, 100);
        /* Set new comparator value to register */
        BPWM_SET_CMR(BPWM0, 0, u32NewCMR);
    }

    /* Stop BPWM counter */
    BPWM_Stop(BPWM0, BPWM_CH_0_MASK);
    /* Disable output of BPWM0 channel 0 */
    BPWM_DisableOutput(BPWM0, BPWM_CH_0_MASK);

    while (1);
}

/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/

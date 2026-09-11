/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 18/07/05 4:59p $
 * @brief
 *           Demonstrate how to set PWM0 channel 0 outputs 240 kHz waveform and
 *                       50 % duty.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/



void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable Internal RC 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable PDMA module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0SEL_PCLK0, 0);

    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for PWM0 Channel 0 */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_PWM0_CH0;
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

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("PWM0 clock is from %s\n", (CLK->CLKSEL3 & CLK_CLKSEL3_PWM0SEL_Msk) ? "CPU" : "PLL");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                                   |\n");
    printf("|                                                                                   |\n");
    printf("+-----------------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output 240 kHz waveform,               \n");
    printf("  and user can change its duty.                                                      \n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PA.0)\n");

    /* PWM0 channel 0 frequency prescaler to 1 */
    PWM_SET_PRESCALER(PWM0, 0, 1-1);

    /* Set PWM0 channel 0 output configuration to be 240 kHz and 50% duty */
    PWM_ConfigOutputChannel(PWM0, 0, 240000, 50);

    /* PWM0 channel 0 is edge-aligned and down counter type */
    PWM_SET_ALIGNED_TYPE(PWM0, BIT0, PWM_EDGE_ALIGNED);

    /* ZeroLevel: Low, CmpUpLevel: nothing, PeriodLevel: nothing, CmpDownLevel: High */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING, PWM_OUTPUT_NOTHING, PWM_OUTPUT_HIGH);

    /* Enable output of PWM0 channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Start PWM counter */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    while(1);
}

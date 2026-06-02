/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use PWM brake function.
 *
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/


void PWM0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       BRAKE0_IRQHandler Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 Brake0 interrupt event
 */
void BRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (PWM0 channel 0 output will toggle again)\n");
    getchar();

    printf("\nPWM brake is unlocked!\n");
    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);
}

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

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Enable GPIO Port D clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* PWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0SEL_PCLK0, 0);

    /* Reset PWM0 module */
    SYS_ResetModule(PWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART1 RXD=PA.8 and TXD=PA.9 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for PWM0 Channel 0 */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_PWM0_CH0;

    /* Set PB multi-function pin for PWM0 brake pin 0 */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~SYS_GPB_MFP0_PB1MFP_Msk) | SYS_GPB_MFP0_PB1MFP_PWM0_BRAKE0;
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nPA.0 is PWM0 channel 0.\n");
    printf("\nConnet PB.1 (PWM0 brake pin 0) to PD.3.\n");
    printf("It will generate brake interrupt and PWM0 channel 0 output stop toggling.\n");

    /* Set PD.3 as output mode and 0 as initail state */
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* PWM0 Channels 0 frequency is 100 Hz, and duty is 30%, */
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);

    /* Enable output of PWM channel 0 */
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable brake function when input Brake0 pin is in High state, and braked state is High */
    PWM_EnableFaultBrake(PWM0, PWM_CH_0_MASK, 1, PWM_FB_EDGE_BKP0);
    
    /* Enable brake interrupt */    
    PWM_EnableFaultBrakeInt(PWM0, 0);

    /* Enable brake noise filter : brake pin 0, filter count = 7, filter clock = HCLK/128 */
    PWM_EnableBrakeNoiseFilter(PWM0, 0, 7, PWM_NF_CLK_DIV_128);

    /* Clear brake interrupt flag */
    PWM_ClearFaultBrakeIntFlag(PWM0, PWM_FB_EDGE);

    NVIC_EnableIRQ(BRAKE0_IRQn);

    /* Start PWM0 channel 0 */
    PWM_Start(PWM0, PWM_CH_0_MASK);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD3 = 1;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

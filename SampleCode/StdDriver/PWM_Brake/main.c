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

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for PWM0 Channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA5MFP_Msk) | SYS_GPA_MFPL_PA5MFP_PWM0_CH0;

    /* Set PB multi-function pin for PWM0 brake pin 0 */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~SYS_GPB_MFPL_PB1MFP_Msk) | SYS_GPB_MFPL_PB1MFP_PWM0_BRAKE0;
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

    printf("\nPA.5 is PWM0 channel 0.\n");
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

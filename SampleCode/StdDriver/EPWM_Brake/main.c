/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use EPWM brake function.
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


void BRAKE0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       EPWM0 Brake0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 Brake0 interrupt event
 */
void EBRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (EPWM0 channel 0 output will toggle again)\n");
    getchar();

    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
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

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Enable GPIO module clock */
    CLK_EnableModuleClock(GPD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* EPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    /* Reset EPWM0 and EPWM1 module */
    SYS_ResetModule(EPWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA5 multi-function pin for EPWM0 Channel 0 */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EPWM0_CH0;

    /* Set PB1 multi-function pin for EPWM0 brake pin 0 */
    SYS->GPB_MFP0 = (SYS->GPB_MFP0 & ~SYS_GPB_MFP0_PB1MFP_Msk) | SYS_GPB_MFP0_PB1MFP_EPWM0_BRAKE0;
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

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

    printf("\nPA.5 is EPWM0 channel 0.\n");
    printf("\nConnet PB.1 (EPWM0 brake pin 0) to PD.3.\n");
    printf("It will generate brake interrupt and EPWM0 channel 0 output stop toggling.\n");

    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* EPWM0 Channels 0/1/2/3 frequency are 100/200/300/400 Hz, and duty are 30/40/50/60 %, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    /* Enable output of EPWM channels 0/1/2/3 */
    EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable brake and interrupt */
    EPWM_EnableFaultBrake(EPWM0, EPWM_CH_0_MASK, 1, EPWM_FB_EDGE_BKP0);
    EPWM_EnableFaultBrakeInt(EPWM0, 0);

    /* Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128 */
    EPWM_EnableBrakeNoiseFilter(EPWM0, 0, 7, EPWM_NF_CLK_DIV_128);

    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);

    NVIC_EnableIRQ(EBRAKE0_IRQn);

    /* Start */
    EPWM_Start(EPWM0, EPWM_CH_0_MASK);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD3 = 1;

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

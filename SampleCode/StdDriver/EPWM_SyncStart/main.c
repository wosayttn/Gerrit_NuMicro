/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use EPWM counter synchronous start function.
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


void SYS_Init(void);
void UART0_Init(void);


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

    /* Enable EPWM0 and EPWM1 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);
    CLK_EnableModuleClock(EPWM1_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* EPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /* Reset EPWM0 and EPWM1 module */
    SYS_ResetModule(EPWM0_RST);
    SYS_ResetModule(EPWM1_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for EPWM0 Channel0~5 */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EPWM0_CH0;
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_EPWM0_CH1;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_EPWM0_CH2;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_EPWM0_CH3;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA1MFP_Msk)) | SYS_GPA_MFP0_PA1MFP_EPWM0_CH4;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA0MFP_Msk)) | SYS_GPA_MFP0_PA0MFP_EPWM0_CH5;

    /* Set PC multi-function pins for EPWM1 Channel0~5 */
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC5MFP_Msk)) | SYS_GPC_MFP1_PC5MFP_EPWM1_CH0;
    SYS->GPC_MFP1 = (SYS->GPC_MFP1 & (~SYS_GPC_MFP1_PC4MFP_Msk)) | SYS_GPC_MFP1_PC4MFP_EPWM1_CH1;
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC3MFP_Msk)) | SYS_GPC_MFP0_PC3MFP_EPWM1_CH2;
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC2MFP_Msk)) | SYS_GPC_MFP0_PC2MFP_EPWM1_CH3;
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC1MFP_Msk)) | SYS_GPC_MFP0_PC1MFP_EPWM1_CH4;
    SYS->GPC_MFP0 = (SYS->GPC_MFP0 & (~SYS_GPC_MFP0_PC0MFP_Msk)) | SYS_GPC_MFP0_PC0MFP_EPWM1_CH5;
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
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with EPWM0 and EPWM1 channel 0~5 at the same time.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(PA.5), EPWM0_CH1(PA.4), EPWM0_CH2(PA.3), EPWM0_CH3(PA.2), EPWM0_CH4(PA.1), EPWM0_CH5(PA.0)\n");
    printf("                         EPWM1_CH0(PC.5), EPWM1_CH1(PC.4), EPWM1_CH2(PC.3), EPWM1_CH3(PC.2), EPWM1_CH4(PC.1), EPWM1_CH5(PC.0)\n");


    /* EPWM0 and EPWM1 channel 0~5 frequency and duty configuration are as follows */
    EPWM_ConfigOutputChannel(EPWM0, 0, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 1, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 2, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 3, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 4, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM0, 5, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 0, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 1, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 2, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 3, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 4, 1000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 5, 1000, 50);

    /* Enable counter synchronous start function for EPWM0 and EPWM1 channel 0~5 */
    EPWM_ENABLE_TIMER_SYNC(EPWM0, 0x3F, EPWM_SSCTL_SSRC_EPWM0);
    EPWM_ENABLE_TIMER_SYNC(EPWM1, 0x3F, EPWM_SSCTL_SSRC_EPWM0);

    /* Enable output of EPWM0 and EPWM1 channel 0~5 */
    EPWM_EnableOutput(EPWM0, 0x3F);
    EPWM_EnableOutput(EPWM1, 0x3F);

    printf("Press any key to start.\n");
    getchar();

    /* Trigger EPWM counter synchronous start by EPWM0 */
    EPWM_TRIGGER_SYNC_START(EPWM0);

    printf("Done.");
    while(1);

}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

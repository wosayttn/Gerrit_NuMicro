/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use EPWM Dead Zone function.
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


void EPWM0P0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       EPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 interrupt event
 */
void EPWM0P0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100)
    {
        if(out)
            EPWM_EnableOutput(EPWM0, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK);
        else
            EPWM_DisableOutput(EPWM0, EPWM_CH_0_MASK | EPWM_CH_1_MASK | EPWM_CH_2_MASK | EPWM_CH_3_MASK);
        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    EPWM_ClearPeriodIntFlag(EPWM0, 0);
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

    /*---------------------------------------------------------------------------------------------------------*/
    /* EPWM clock frequency configuration                                                                      */
    /*---------------------------------------------------------------------------------------------------------*/
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    /* Reset EPWM0 module */
    SYS_ResetModule(EPWM0_RST);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    Uart0DefaultMPF();

    /* Set PA multi-function pins for EPWM0 Channel0~3 */
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA5MFP_Msk)) | SYS_GPA_MFP1_PA5MFP_EPWM0_CH0;
    SYS->GPA_MFP1 = (SYS->GPA_MFP1 & (~SYS_GPA_MFP1_PA4MFP_Msk)) | SYS_GPA_MFP1_PA4MFP_EPWM0_CH1;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA3MFP_Msk)) | SYS_GPA_MFP0_PA3MFP_EPWM0_CH2;
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & (~SYS_GPA_MFP0_PA2MFP_Msk)) | SYS_GPA_MFP0_PA2MFP_EPWM0_CH3;
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

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output EPWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead time function of all EPWM0 pairs.\n");
    printf("  And also enable/disable EPWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(PA.5), EPWM0_CH1(PA.4), EPWM0_CH2(PA.3), EPWM0_CH3(PA.2)\n");

    /* Set Pwm mode as complementary mode */
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM0);

    /* EPWM0 channel 0 frequency is 100Hz, duty 30% */
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    SYS_UnlockReg();
    EPWM_EnableDeadZone(EPWM0, 0, 400);
    SYS_LockReg();

    /* EPWM0 channel 2 frequency is 3000Hz, duty 50% */
    EPWM_ConfigOutputChannel(EPWM0, 2, 3000, 50);
    SYS_UnlockReg();
    EPWM_EnableDeadZone(EPWM0, 2, 200);
    SYS_LockReg();

    /* Enable output of EPWM0 channel 0~3 */
    EPWM_EnableOutput(EPWM0, 0xF);

    /* Enable EPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    EPWM_EnablePeriodInt(EPWM0, 0, 0);
    NVIC_EnableIRQ(EPWM0_P0_IRQn);

    /* Start */
    EPWM_Start(EPWM0, 0xF);

    while(1);
}

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

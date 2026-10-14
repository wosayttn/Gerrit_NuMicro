/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate TIMER PWM accumulator interrupt to stop counting.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t gu32Period;


void LPTMR0_IRQHandler(void)
{
    LPTPWM_ClearAccInt(LPTMR0);

    printf("\nCheck if output toggles 11 times then stop toggles.\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 72MHz */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable LPTMR module clock */
    CLK_EnableModuleClock(LPTMR0_MODULE);

    /* Select LPTMR clock source */
    CLK_SetModuleClock(LPTMR0_MODULE, LPSCC_CLKSEL0_LPTMR0SEL_HIRC, 0);

    /* Set LPTMR0 PWM CH0(TM0) pin */
    SYS->GPB_MFP1 |= SYS_GPB_MFP1_PB5MFP_LPTM0;
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------------------------------------+\n");
    printf("|    Low Power Timer PWM Accumulator Inerrupt Stop Mode Sample Code    |\n");
    printf("+----------------------------------------------------------------------+\n\n");

    printf("  This sample code demonstrate Low Power Timer0 PWM accumulator stop mode.\n");
    printf("  When accumulator interrupt happens, the output of Low Power Timer0 PWM stops.\n");
    printf("  Since interrupt accumulator count is set to 10, the output toggles 11 times then stops.\n");
    printf("    - LPTMR0 PWM_CH0 on PB.5\n");

    printf("\n\nPress any key to start Low Power Timer0 PWM.\n");
    getchar();

    /* Change Low Power Timer to PWM counter mode */
    LPTPWM_ENABLE_PWM_MODE(LPTMR0);

    /* Set Low Power Timer0 PWM output frequency is 18000 Hz, duty 50% in up count type */
    LPTPWM_ConfigOutputFreqAndDuty(LPTMR0, 18000, 50);

    /* Enable output of PWM_CH0 */
    LPTPWM_ENABLE_OUTPUT(LPTMR0, TPWM_CH0);

    /* Enable Low Power Timer0 PWM accumulator function, interrupt count 10, accumulator source select to period point */
    LPTPWM_EnableAcc(LPTMR0, 10, LPTPWM_IFA_PERIOD_POINT);

    /* Enable Low Power Timer0 PWM accumulator interrupt */
    LPTPWM_EnableAccInt(LPTMR0);

    /* Enable Low Power Timer0 PWM accumulator stop mode */
    LPTPWM_EnableAccStopMode(LPTMR0);

    /* Enable Low Power Timer0 interrupt */
    NVIC_EnableIRQ(LPTMR0_IRQn);

    /* Start Low Power Timer PWM counter */
    LPTPWM_START_COUNTER(LPTMR0);

    while(1) {}
}

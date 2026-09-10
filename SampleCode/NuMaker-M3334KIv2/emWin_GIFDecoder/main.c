/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    To utilize emWin library to demonstrate interactive feature.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#include "GUI.h"

extern volatile GUI_TIMER_TIME OS_TimeMS;
extern void MainTask(void);


/*********************************************************************
*
*       TMR0_IRQHandler
*/
void TMR0_IRQHandler(void)
{
    OS_TimeMS++;

    TIMER_ClearIntFlag(TIMER0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 180MHz */
    CLK_SetCoreClock(FREQ_180MHZ);

#ifdef __DEMO_320x240__
    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk;
    /* Enable GPIO Port A clock */
    CLK_EnableModuleClock(GPA_MODULE);
    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);
#endif

    /* Enable GPIO Port D, E, H clocks for LCD pins (PD14=RST, PE0/1=SPI1 MOSI/MISO, PH8~11=SPI1 CLK/SS/DC/LED) */
    CLK_EnableModuleClock(GPD_MODULE);
    CLK_EnableModuleClock(GPE_MODULE);
    CLK_EnableModuleClock(GPH_MODULE);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

     /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);
}

/* Main */
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    //
    // Enable Timer0 clock and select Timer0 clock source
    //
    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);
    //
    // Initial Timer0 to periodic mode with 1000Hz
    //
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000);
    //
    // Enable Timer0 interrupt
    //
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 1);
    NVIC_EnableIRQ(TMR0_IRQn);
    //
    // Start Timer0
    //
    TIMER_Start(TIMER0);

#ifdef __DEMO_320x240__
    SYS->GPB_MFP1 &= ~(SYS_GPB_MFP1_PB4MFP_Msk);
    SYS->GPB_MFP0 &= ~(SYS_GPB_MFP0_PB1MFP_Msk | SYS_GPB_MFP0_PB0MFP_Msk);
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT4);
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT1);
    GPIO_ENABLE_DIGITAL_PATH(PB, BIT0);
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT1, GPIO_MODE_INPUT);
    GPIO_SetMode(PB, BIT0, GPIO_MODE_INPUT);
#endif

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|               emWin Simple GIF Decode Demo                             |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("\n\n");

    printf("Run MainTask...!!!\n");
    MainTask();
}

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate the usage of TIMER_Delay() API to generate a 1
 *           second delay.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


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

    /* Select TIMER0 clock source */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable TIMER0 clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /*----------------------------------------------*/
    /* Init I/O Multi-function                      */
    /*----------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

/*----------------------------------------------------------------------*/
/* Init UART0                                                           */
/*----------------------------------------------------------------------*/
void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nThis sample code use timer to create a small delay \n");
    while(1)
    {
        printf("Delay 1 second\n");
        TIMER_Delay(TIMER0, 1000000);
    }
}

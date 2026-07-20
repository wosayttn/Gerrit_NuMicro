/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use TTMR to wake up system from Power-down mode periodically.
 *           Please refer to the sample code SYS_PowerDown_MinCurrent to set
 *           the minimum current of the system in Power-down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void TTMR0_IRQHandler(void)
{
    /* Clear wake up flag */
    TTMR_ClearWakeupFlag(TTMR0);
    /* Clear interrupt flag */
    TTMR_ClearIntFlag(TTMR0);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
    PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

    /* Enable HIRC clock (Internal RC 48 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock as 72MHz from PLL */
    CLK_SetCoreClock(FREQ_72MHZ);

    /* Set PCLK0/PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable TTMR module clock */
    CLK_SetModuleClock(TTMR0_MODULE, LPSCC_CLKSEL0_TTMR0SEL_LIRC, 0);
    CLK_EnableModuleClock(TTMR0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
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
    int i = 0;

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("TTMR power down/wake up sample code\n");
    while(!UART_IS_TX_EMPTY(UART0));

    /* Output selected clock to CKO, CKO Clock = HCLK / 1 */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 0, 1);

    /* Initial TTMR0 to periodic mode with 1Hz, since system is fast (12MHz)
       and TTMR is slow (32KHz), and following function calls all modified TTMR's
       CTL register, so add extra delay between each function call and make sure the
       setting take effect */
    TTMR_Open(TTMR0, TTMR_PERIODIC_MODE, 1);
    CLK_SysTickDelay(50);
    /* Enable TTMR0 wake up system */
    TTMR_EnableWakeup(TTMR0);
    CLK_SysTickDelay(50);
    /* Enable TTMR0 interrupt */
    TTMR_EnableInt(TTMR0);
    CLK_SysTickDelay(50);
    NVIC_EnableIRQ(TTMR0_IRQn);
    /* Start TTMR0 counting */
    TTMR_Start(TTMR0);
    CLK_SysTickDelay(50);
    /* Unlock protected registers */
    SYS_UnlockReg();
    while(1)
    {
        printf("Enter Power-down !\n");
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();
        printf("Wake %d\n", i++);
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_SysTickDelay(1000000);
    }
}

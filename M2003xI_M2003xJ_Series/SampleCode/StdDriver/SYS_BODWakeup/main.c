/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to wake up system from Power-down mode by brown-out detector interrupt.
 *           Please refer to the sample code SYS_PowerDown_MinCurrent to set
 *           the minimum current of the system in Power-down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/

void SYS_Init(void)
{
    /*------------------------------------------*/
    /* Init System Clock                        */
    /*------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_SRCCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to HIRC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_HCLKDIV_HCLK(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /*----------------------------------*/
    /* Init I/O Multi-function          */
    /*----------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*--------------------------------------*/
/*  Brown Out Detector IRQ Handler      */
/*--------------------------------------*/
void BOD_IRQHandler(void)
{
    /* Clear BOD Interrupt Flag */
    SYS_CLEAR_BOD_INT_FLAG();

    printf("Brown Out is Detected.\n");
}

void BOD_FunctionTest()
{
    printf("+------------------------------------------------+\n");
    printf("|        Power-down and Wake-up Sample Code      |\n");
    printf("+------------------------------------------------+\n");

    /* Unlock protected registers before setting Brown-out detector function and Power-down mode */
    SYS_UnlockReg();

    /* Enable Brown-out detector function */
    SYS_ENABLE_BOD();

    /* Set Brown-out detector voltage level as 2.7V */
    SYS_SET_BOD_LEVEL(SYS_BODCTL_BODVL_2_7V);

    /* Enable Brown-out detector interrupt function */
    SYS_DISABLE_BOD_RST();

    /* Enable Brown-out Detector power drop wake-up function */
    SYS_SET_BOD_WAKEUP(SYS_BODCTL_BODWKEN_DROP);

    /* Select Brown-out Detector Output De-glitch Time */
    /* The de-glitch time MUST change to LIRC before system enters Power-down mode */
    SYS_SET_BODDGSEL(SYS_BODCTL_BODDGSEL_LIRC);

    /* Enable Brown-out detector interrupt */
    NVIC_EnableIRQ(BOD_IRQn);
    NVIC_ClearPendingIRQ(BOD_IRQn);

    printf("System enter to Power-down mode.\n");
    printf("System wake-up if VDD voltage is lower than 2.7V.\n\n");
    UART_WAIT_TX_EMPTY(UART0);  /* Check if all the debug messages are finished */

    /* Enter to Power-down mode */
    CLK_PowerDown();

    printf("Wakeup !!\n");
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to print message */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    /* Set PB14 as CLKO pin to output HCLK */
    SET_CLKO_PB14();
    CLK_SetModuleClock(CLKO_MODULE, CLK_CLKOSEL_CLKOSEL_HCLK, (uint32_t)NULL);
    CLK_EnableCKO(CLK_CLKOSEL_CLKOSEL_HCLK, 0, 1);

    /* BOD function test */
    BOD_FunctionTest();

    printf("Exit BOD sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

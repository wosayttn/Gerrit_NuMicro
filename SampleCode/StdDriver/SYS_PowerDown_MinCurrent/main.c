/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define GPIO_P0_TO_P15              0xFFFF

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for disable internal analog POR circuit                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Disable_AnalogPORCircuit(void)
{
    SYS_DISABLE_POR();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Set Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_NPD2);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void GPB_IRQHandler(void)
{
    volatile uint32_t temp;

    /* To check if PB.3 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT3))
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("PB.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        temp = PB->INTSRC;
        PB->INTSRC = temp;
        printf("Un-expected interrupts.\n");
    }
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

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPB_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
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

int main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.3 Sample Code   |\n");
    printf("+-------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                      |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                               |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                      |\n");
    printf("|  3. Must enable LVR                                                     |\n");
    printf("|  4. Disable analog function, e.g. ADC, ACMP, and POR module.            |\n");
    printf("|  5. Enter to Power-Down                                                 |\n");
    printf("|  6. Wait for PB.3 falling-edge interrupt event to wakeup the MCU        |\n");
    printf("+-------------------------------------------------------------------------+\n\n");

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(UART0);

    /* Configure all GPIO as Quasi-bidirectional Mode*/
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Configure PB.3 as Quasi-bidirectional mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(PB, BIT3, GPIO_MODE_QUASI);
    GPIO_EnableInt(PB, 3, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPB_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, BIT3);

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* LVR must be enabled and the POR will be enabled automatically */
    SYS_ENABLE_LVR();

    /* BOD must be disabled */
    SYS_DISABLE_BOD();

    /* Turn off internal analog POR circuit */
    SYS_Disable_AnalogPORCircuit();
    /* Disable Power-on Reset */
    SYS_DISABLE_POR();

    printf("Enter to Power-Down ......\n");
    /* Enter to Power-down mode */
    PowerDownFunction();
    /* Waiting for PB.3 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);
}

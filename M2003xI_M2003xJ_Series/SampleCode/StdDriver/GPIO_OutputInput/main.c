/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
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

    /* Enable GPIO Port B clock */
    CLK_EnableModuleClock(GPIOB_MODULE);

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

void GPIO_FunctionTest()
{
    int32_t i32Err, i32TimeOutCnt;

    printf("+-----------------------------------------------+\n");
    printf("|       GPIO Output and Input Sample Code       |\n");
    printf("+-----------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("  >> Please connect PB.5 (output pin) and PB.4 (input pin) first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    /* Configure PB.5 as Output mode and PB.4 as Input mode then close it */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PB, BIT4, GPIO_MODE_INPUT);

    i32Err = 0;

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set PB.5 output pin value is low */
    PB5 = 0;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PB.4 input pin status is low for a while */
    while(PB4 != 0)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Set PB.5 output pin value is high */
    PB5 = 1;

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for PB.4 input pin status is high for a while */
    while(PB4 != 1)
    {
        if(i32TimeOutCnt > 0)
        {
            i32TimeOutCnt--;
        }
        else
        {
            i32Err = 1;
            break;
        }
    }

    /* Print test result */
    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure PB.5 and PB.4 to default Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT4, GPIO_MODE_QUASI);
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to print message */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    /* GPIO function test */
    GPIO_FunctionTest();

    printf("Exit GPIO sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2017-2026 Nuvoton Technology Corp. ***/

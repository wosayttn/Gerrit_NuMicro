/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to set LPGPIO pin mode and use pin data input/output control.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

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

    /* Set HCLK1 to HIRC/1 */
    CLK_SetModuleClock(HCLK1_MODULE, CLK_CLKSEL0_HCLK1SEL_HIRC, LPSCC_CLKDIV0_HCLK1(1));

    /* Enable HCLK1 clock */
    CLK_EnableModuleClock(HCLK1_MODULE);

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPGPIO_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Configure the GPA0 - GPA1 to LPGPIO pins.  */
    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA0MFP_Msk)) |
                    (SYS_GPA_MFP0_PA0MFP_LPIO0);

    SYS->GPA_MFP0 = (SYS->GPA_MFP0 & ~(SYS_GPA_MFP0_PA1MFP_Msk)) |
                    (SYS_GPA_MFP0_PA1MFP_LPIO1);

    /* Lock protected registers */
    SYS_LockReg();
}

void LPGPIO_FunctionTest()
{
    int32_t i32Err, i32TimeOutCnt;

    printf("\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|               LPGPIO Output / Input sample code               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /*------------------------------------------------------------------------------*/
    /* LPGPIO Basic Mode Test --- Use Pin Data Input/Output to control GPIO pin     */
    /*------------------------------------------------------------------------------*/
    printf("  >> Please connect PA.0 (LPIO0) and PA.1 (LPIO1) first << \n");
    printf("     Press any key to start test by using [Pin Data Input/Output Control] \n\n");
    getchar();

    LPGPIO_SetMode(LPGPIO, BIT0, LPGPIO_MODE_OUTPUT);
    LPGPIO_SetMode(LPGPIO, BIT1, LPGPIO_MODE_INPUT);

    i32Err = 0;
    printf("LPGPIO LPIO0 PA.0 (output mode) connect to LPIO1 PA.1 (input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Set LPIO0 output pin value to low */
    LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) & ~BIT0));

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for LPIO1 input pin status is low for a while */
    while((LPGPIO_GET_IN_DATA(LPGPIO) & BIT1) != 0)
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

    /* Set LPIO0 output pin value to high */
    LPGPIO_SET_OUT_DATA(LPGPIO, (LPGPIO_GET_IN_DATA(LPGPIO) | BIT0));

    /* Set time out counter */
    i32TimeOutCnt = 100;

    /* Wait for LPIO1 input pin status is high for a while */
    while((LPGPIO_GET_IN_DATA(LPGPIO) & BIT1) != BIT1)
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
        printf(" [FAIL].\n");
    }
    else
    {
        printf(" [OK].\n");
    }
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

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* LPGPIO function test */
    LPGPIO_FunctionTest();

    /* Disable LPGPIO IP clock */
    CLK_DisableModuleClock(LPGPIO_MODULE);

    printf("Exit LPGPIO sample code\n");

    while(1);
}

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;

/*----------------------------------------------*/
/* ADC interrupt handler                        */
/*----------------------------------------------*/
void ADC_IRQHandler(void)
{
    if(ADC_GET_INT_FLAG(ADC, ADC_CMP0_INT))
    {
        g_u32AdcCmp0IntFlag = 1;
        ADC_CLR_INT_FLAG(ADC, ADC_CMP0_INT);    /* Clear the A/D compare flag 0 */
    }

    if(ADC_GET_INT_FLAG(ADC, ADC_CMP1_INT))
    {
        g_u32AdcCmp1IntFlag = 1;
        ADC_CLR_INT_FLAG(ADC, ADC_CMP1_INT);    /* Clear the A/D compare flag 1 */
    }

    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
}

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

    /* The Maximum ADC clock frequency is 16 MHz.
     * Hence, we set PLL to 32 MHz, HCLK = PLL/1, and PCLK1 = HCLK/1 = 32 MHz.
     * ADC clock source is from PCLK1/2 = 16 MHz.
     */
    /* Set PLL to 32MHz */
    CLK_EnablePLL((uint32_t)NULL, FREQ_32MHZ);

    /* Switch HCLK clock source to PLL and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_HCLKDIV_HCLK(1));

    /* Set PCLK0 = HCLK/1 and PCLK1 = HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_PCLK0DIV_DIV1 | CLK_PCLKDIV_PCLK1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_UARTSEL_UART0SEL_HIRC, CLK_UARTDIV_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set ADC clock source and divider */
    CLK_SetModuleClock(ADC0_MODULE, (uint32_t)NULL, CLK_ADCDIV_ADC0(2));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /*----------------------------------*/
    /* Init I/O Multi-function          */
    /*----------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    Uart0DefaultMPF();

    /* Set PB.2 to input mode */
    GPIO_SetMode(PB, BIT2, GPIO_MODE_INPUT);
    /* Configure the PB.2 ADC analog input pins.  */
    SET_ADC0_CH2_PB2();
    /* Disable the PB.2 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT2);

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

void ADC_FunctionTest()
{
    uint32_t u32ConversionData;

    printf("+----------------------------------------------------------------------+\n");
    printf("|            ADC compare function (result monitor) sample code         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end, Single mode, and select channel 2 */
    ADC_Open(ADC, 0, ADC_ADCR_ADMD_SINGLE, BIT2);

    /* Enable ADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    ADC_ENABLE_CMP0(ADC, 2, ADC_ADCMPR_CMPCOND_LESS_THAN, 0x800, 5);

    /* Enable ADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1: channel 2 is greater than or equal to 0x800; match count is 5.\n");
    ADC_ENABLE_CMP1(ADC, 2, ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 5);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the sample module interrupt */
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);  /* Enable sample module A/D interrupt. */
    NVIC_EnableIRQ(ADC_IRQn);

    /* Clear the ADC comparator 0 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_CMP0_INT);
    /* Enable ADC comparator 0 interrupt */
    ADC_ENABLE_INT(ADC, ADC_CMP0_INT);

    /* Clear the ADC comparator 1 interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_CMP1_INT);
    /* Enable ADC comparator 1 interrupt */
    ADC_ENABLE_INT(ADC, ADC_CMP1_INT);

    /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32AdcIntFlag = 0;
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC compare interrupt */
    while(1)
    {
        if (g_u32AdcIntFlag == 1)
        {
            u32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 2);
            printf("Conversion result of channel 2: 0x%03X (%d)\n", u32ConversionData, u32ConversionData);

            if ((g_u32AdcCmp0IntFlag == 1) || (g_u32AdcCmp1IntFlag == 1))
                break;

            g_u32AdcIntFlag = 0;
            ADC_START_CONV(ADC);
        }
    }

    /* Disable ADC comparator interrupt */
    ADC_DISABLE_INT(ADC, ADC_CMP0_INT);
    ADC_DISABLE_INT(ADC, ADC_CMP1_INT);
    /* Disable compare function */
    ADC_DISABLE_CMP0(ADC);
    ADC_DISABLE_CMP1(ADC);

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }
}

int main()
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to print message */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz\n", SystemCoreClock);

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable ADC Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    /* Got nowhere to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2016-2026 Nuvoton Technology Corp. ***/

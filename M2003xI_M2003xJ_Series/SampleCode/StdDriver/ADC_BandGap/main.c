/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Convert Band-gap (channel 29) and print conversion result.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2017-2026 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

/*----------------------------------------------*/
/* ADC interrupt handler                        */
/*----------------------------------------------*/
void ADC_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
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
    int32_t  i32ConversionData;

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  ADC for Band-gap test                                |\n");
    printf("+-----------------------------------------------------------------------+\n");

    printf("+-----------------------------------------------------------------------+\n");
    printf("|   ADC clock source = PCLK1  = 32 MHz                                  |\n");
    printf("|   ADC clock divider         = 2                                       |\n");
    printf("|   ADC clock                 = 32 MHz / 2 = 16 MHz (62.5 ns per clock) |\n");
    printf("|   ADC conversion rate for Vbg MUST <= 100 KSPS !                      |\n");
    printf("|   ADC conversion clock      = 16                                      |\n");
    printf("|   ADC total conversion time = 1s / 100 KSPS = 10 us                   |\n");
    printf("|                             --> 10 us / 62.5 ns = 160 ADC clocks      |\n");
    printf("|   ADC extended sampling clocks = 160 - 16 = 144 ADC clocks            |\n");
    printf("+-----------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end, Single mode, and select channel 29 (band-gap voltage) */
    ADC_Open(ADC, 0, ADC_ADCR_ADMD_SINGLE, BIT29);

    /* Set sample module extend sampling time to 144 ADC clocks */
    ADC_SetExtendSampleTime(ADC, 0, 144);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the sample module interrupt.  */
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);   /* Enable sample module A/D interrupt. */
    NVIC_EnableIRQ(ADC_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the A/D interrupt */
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    /* Get the conversion result of the channel 29 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);
    printf("ADC Conversion result of Band-gap: 0x%x (%d)\n", i32ConversionData, i32ConversionData);
    printf("Band-gap voltage is %dmV if Reference voltage is 3.3V\n", (3300*i32ConversionData)/4095);
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

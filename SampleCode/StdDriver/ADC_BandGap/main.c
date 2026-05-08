/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Convert Band-gap (channel 16) and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*------------------------------------------------------*/
/* Define global variables and constants                */
/*------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


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

    /* ADC clock source is 16 MHz from HIRC, set divider to 1, ADC clock is 16/1 MHz */
    CLK_SetModuleClock(ADC0_MODULE, CLK_CLKSEL1_ADC0SEL_HIRC, CLK_CLKDIV_ADC0(1));

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Force enable internal voltage Band-gap. */
    SYS->VREFCTL |= SYS_VREFCTL_VBGFEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

void ADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                  ADC for Band-gap test                            |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 16 MHz                             |\n");
    printf("|   ADC clock divider          = 1                                  |\n");
    printf("|   ADC clock                  = 16 MHz / 1 = 16 MHz                |\n");
    printf("|   If the internal channel for band-gap voltage is active,         |\n");
    printf("|      the maximum sampling rate will be 100 KSPS.                  |\n");
    printf("|   ADC extended sampling time = 144                                |\n");
    printf("|   ADC conversion time = 16 + ADC extended sampling time = 160     |\n");
    printf("|   ADC conversion rate = 16 MHz / 160 = 100 ksps                   |\n");
    printf("+-------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set ADC to Single mode, and select channel 16 (band-gap voltage) */
    ADC_Open(ADC, (uint32_t)NULL, ADC_ADCR_ADMD_SINGLE, BIT16);

    /* If the internal channel for band-gap voltage is active,
       the maximum sampling rate will be 100 KSPS. */
    /* Set sample module external sampling time to 144 */
    ADC_SetExtendSampleTime(ADC, 0, 144);

    /* Clear the A/D interrupt flag for safe */
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

    /* Enable the sample module interrupt.  */
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);   /* Enable sample module A/D interrupt. */
    NVIC_EnableIRQ(ADC0_INT0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the A/D interrupt */
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    /* Get the conversion result of the channel 16 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 16);
    printf("ADC Conversion result of Band-gap: 0x%X (%d)\n", i32ConversionData, i32ConversionData);
    printf("Band-gap  voltage is %4dmV if Reference voltage is 3300mV\n", (3300*i32ConversionData)/4095);
    printf("Reference voltage is %4dmV if Band-gap  voltage is  814mV\n", (814*4095)/i32ConversionData);
}

void ADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT); /* Clear the A/D interrupt flag */
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

    /* ADC function test */
    ADC_FunctionTest();

    /* Disable ADC IP clock */
    CLK_DisableModuleClock(ADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC0_INT0_IRQn);

    printf("Exit ADC sample code\n");

    while(1);
}

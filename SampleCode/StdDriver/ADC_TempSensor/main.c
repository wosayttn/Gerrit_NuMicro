/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Convert temperature sensor and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
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

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

float my_fabs(float x)
{
    return (x < 0) ? -x : x;
}

float my_round(float x)
{
    return (x >= 0.0f) ? (int)(x + 0.5f) : (int)(x - 0.5f);
}

void ADC_FunctionTest()
{
    int32_t  i32ConversionData;
    float temperature;

    printf("\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                   ADC Temperature sensor test                     |\n");
    printf("+-------------------------------------------------------------------+\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 16 MHz                             |\n");
    printf("|   ADC clock divider          = 1                                  |\n");
    printf("|   ADC clock                  = 16 MHz / 1 = 16 MHz                |\n");
    printf("|   If the internal channel for temperature sensor is active,       |\n");
    printf("|      the maximum sampling rate will be 100 KSPS.                  |\n");
    printf("|   ADC extended sampling time = 144                                |\n");
    printf("|   ADC conversion time = 16 + ADC extended sampling time = 160     |\n");
    printf("|   ADC conversion rate = 16 MHz / 160 = 100 ksps                   |\n");
    printf("+-------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set ADC to Single mode, and select channel 17 (temperature sensor) */
    ADC_Open(ADC, (uint32_t)NULL, ADC_ADCR_ADMD_SINGLE, BIT17);

    /* If the internal channel for temperature sensor is active,
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

    /* Get the conversion result of the channel 17 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 17);
    printf("ADC Conversion result of Temperature Sensor: 0x%X (%d)\n", i32ConversionData, i32ConversionData);

    /* The equation of converting to real temperature is as below
     *      Vtemp = Tc * (temperature - Ta) + Vtemp_os
     *      Vtemp = EADC_result / 4095 * ADC_Vref
     *      so, temperature = Ta + (Vtemp - Vtemp_os) / Tc
     *                      = Ta + ((EADC_result / 4095 * ADC_Vref) - Vtemp_os) / Tc
     *      where Vtemp_os (offset voltage), Tc (temperature coefficient), and Ta (ambient temperature)
     *            can be got from the data sheet document.
     *            ADC_Vref is the ADC Vref that according to the configuration of SYS and ADC.
     */
    temperature = 25+(((float)i32ConversionData/4095*3300)-684)/(-1.72);
    #if defined (__CC_ARM)
        /* for ARM 5 */
        printf("Current Temperature = %2.1f degrees Celsius if EADC Vref = 3300mV\n\n", temperature);
    #elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
        /* for ARM 6 that __ARMCC_VERSION >= 6010050 or VSCode ARMCLANG */
        printf("Current Temperature = %2.1f degrees Celsius if EADC Vref = 3300mV\n\n", temperature);
    #elif defined (__ICCARM__)
        /* for IAR */
        printf("Current Temperature = %2.1f degrees Celsius if EADC Vref = 3300mV\n\n", temperature);
    #elif defined (__GNUC__)
        /* for VSCode GNUC */
        /* Convert one floating to two integers since printf() don't support format %f */
        {
            double intPart, fracPart;
            fracPart = modf(temperature, &intPart);
            printf("Current Temperature = %d.%d degrees Celsius if EADC Vref = 3300mV\n\n", (uint32_t)intPart, (uint32_t)my_round(my_fabs(fracPart*10)));
        }
    #endif
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

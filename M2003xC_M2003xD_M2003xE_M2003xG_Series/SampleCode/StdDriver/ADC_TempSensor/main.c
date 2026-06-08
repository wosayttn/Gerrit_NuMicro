/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Convert temperature sensor and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <math.h>
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

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select UART clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL2_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Set ADC clock source and divider */
#if (CHIP_TYPE == CHIP_TYPE_M2003C)
    /* Maximum ADC clock = 24MHz */
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_HIRC, CLK_CLKDIV0_ADC(2));
#else
    /* Maximum ADC clock = 16MHz */
  #if (__HIRC == 24000000)
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_HIRC, CLK_CLKDIV0_ADC(2));
  #elif (__HIRC == 32000000)
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_HIRC, CLK_CLKDIV0_ADC(2));
  #elif (__HIRC == 40000000)
    CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_HIRC, CLK_CLKDIV0_ADC(4));
  #endif
#endif

    /* Enable ADC module clock */
    CLK_EnableModuleClock(ADC_MODULE);

    /*----------------------------------*/
    /* Init I/O Multi-function          */
    /*----------------------------------*/
    Uart0DefaultMPF();

    /* Enable temperature sensor */
    SYS->IVSCTL |= SYS_IVSCTL_VTEMPEN_Msk;

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

    printf("+------------------------------------------------------------------------+\n");
    printf("|                  ADC Temperature sensor test                           |\n");
    printf("+------------------------------------------------------------------------+\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end, Single mode, and select channel 30 (temperature sensor) */
    ADC_Open(ADC, 0, ADC_ADCR_ADMD_SINGLE, BIT30);

#if (CHIP_TYPE == CHIP_TYPE_M2003C)
    printf("+------------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 24 MHz                                  |\n");
    printf("|   ADC clock divider          = 2                                       |\n");
    printf("|   ADC clock                  = 24 MHz / 2 = 12 MHz (83.33 ns per clock)|\n");
    printf("|   ADC extended sampling time = 221 ADC clocks                          |\n");
    printf("|   Aysnc ADC conversion time  =                                         |\n");
    printf("|       1 us + (7 + ADC extended sampling time) clocks =                 |\n");
    printf("|       1 us + 228 clocks = 1 us + (83.33 ns * 228) = 20 us              |\n");
    printf("|   ADC conversion rate = 1 second / 20 us = 50 KSPS                     |\n");
    printf("+------------------------------------------------------------------------+\n");
    /* Set sample module extend sampling time */
    ADC_SetExtendSampleTime(ADC, 0, 221);
#else
  #if (__HIRC == 24000000)
    printf("+------------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 24 MHz                                  |\n");
    printf("|   ADC clock divider          = 2                                       |\n");
    printf("|   ADC clock                  = 24 MHz / 2 = 12 MHz (83.33 ns per clock)|\n");
    printf("|   ADC extended sampling time = 224 ADC clocks                          |\n");
    printf("|   Sync ADC conversion time   =                                         |\n");
    printf("|             (16 + ADC extended sampling time) clocks =                 |\n");
    printf("|             240 clocks = (83.33 ns * 240) = 20 us                      |\n");
    printf("|   ADC conversion rate = 1 second / 20 us = 50 KSPS                     |\n");
    printf("+------------------------------------------------------------------------+\n");
    /* Set sample module extend sampling time */
    ADC_SetExtendSampleTime(ADC, 0, 224);
  #elif (__HIRC == 32000000)
    printf("+------------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 32 MHz                                  |\n");
    printf("|   ADC clock divider          = 2                                       |\n");
    printf("|   ADC clock                  = 32 MHz / 2 = 16 MHz (62.5 ns per clock) |\n");
    printf("|   ADC extended sampling time = 304 ADC clocks                          |\n");
    printf("|   Sync ADC conversion time   =                                         |\n");
    printf("|             (16 + ADC extended sampling time) clocks =                 |\n");
    printf("|             320 clocks = (62.5 ns * 320) = 20 us                       |\n");
    printf("|   ADC conversion rate = 1 second / 20 us = 50 KSPS                     |\n");
    printf("+------------------------------------------------------------------------+\n");
    /* Set sample module extend sampling time */
    ADC_SetExtendSampleTime(ADC, 0, 304);
  #elif (__HIRC == 40000000)
    printf("+------------------------------------------------------------------------+\n");
    printf("|   ADC clock source -> HIRC   = 40 MHz                                  |\n");
    printf("|   ADC clock divider          = 4                                       |\n");
    printf("|   ADC clock                  = 40 MHz / 4 = 10 MHz (100 ns per clock)  |\n");
    printf("|   ADC extended sampling time = 184 ADC clocks                          |\n");
    printf("|   Sync ADC conversion time   =                                         |\n");
    printf("|             (16 + ADC extended sampling time) clocks =                 |\n");
    printf("|             200 clocks = (100 ns * 200) = 20 us                        |\n");
    printf("|   ADC conversion rate = 1 second / 20 us = 50 KSPS                     |\n");
    printf("+------------------------------------------------------------------------+\n");
    /* Set sample module extend sampling time */
    ADC_SetExtendSampleTime(ADC, 0, 184);
  #endif
#endif

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

    /* Get the conversion result of the channel 30 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 30);
    printf("ADC Conversion result of Band-gap: 0x%X (%d)\n", i32ConversionData, i32ConversionData);

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
    CLK_DisableModuleClock(ADC_MODULE);

    /* Disable ADC Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    /* Got no where to go, just loop forever */
    while (1);
}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/

/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to calculate analog voltage (AVdd) by using band-gap.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------*/
/* Define global variables and constants        */
/*----------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;
volatile uint32_t g_u32BandGapConvValue;

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

/**
  * @brief      Read Built-in Band-Gap conversion value
  * @param[in]  None
  * @return     Built-in Band-Gap conversion value
  * @details    This function is used to read Band-Gap conversion value.
  */
__STATIC_INLINE uint32_t FMC_ReadBandGap(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_UID;            /* Set ISP Command Code */
    FMC->ISPADDR = 0x70u;                         /* Must keep 0x70 when read Band-Gap */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;           /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                            /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) {}  /* Waiting for ISP Done */

    return FMC->ISPDAT & 0xFFF;
}

void ADC_FunctionTest()
{
    int32_t  i32ConversionData;
    int32_t  i32BuiltInData;

    printf("+------------------------------------------------------------------------+\n");
    printf("|     ADC for calculate analog voltage (AVdd) by using band-gap test     |\n");
    printf("+------------------------------------------------------------------------+\n\n");

    /* Enable ADC converter */
    ADC_POWER_ON(ADC);

    /* Set input mode as single-end, Single mode, and select channel 29 (band-gap voltage) */
    ADC_Open(ADC, 0, ADC_ADCR_ADMD_SINGLE, BIT29);

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

    /* Get the conversion result of the channel 29 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 29);

    /* Enable FMC ISP function to read built-in band-gap A/D conversion result*/
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();

    /* Use Conversion result of Band-gap to calculating AVdd */
    printf("    3072 (mV) / 4095 = 1200 (mV) / Built-In Vbg Conversion Data\n");
    printf("    AVdd (mV) / 4095 = 1200 (mV) / Current  Vbg Conversion Data\n");
    printf("--> 3072 * Built-In Vbg Conversion Data = 1200 * 4095 = AVdd * Current Vbg Conversion Data\n");
    printf("--> AVdd = 3072 * Built-In Vbg Conversion Data / Current Vbg Conversion Data\n\n");

    printf("Built-in band-gap A/D conversion result at AVdd = 3072mV: 0x%X (%d) \n", i32BuiltInData, i32BuiltInData);
    printf("Current  band-gap A/D conversion result at current AVdd : 0x%X (%d) \n", i32ConversionData, i32ConversionData);
    printf("Current  AVdd = 3072 * %d / %d = %d mV \n\n", i32BuiltInData, i32ConversionData, 3072*i32BuiltInData/i32ConversionData);
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

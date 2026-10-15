/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to calculate battery voltage( AVdd ) by using band-gap.
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
volatile uint32_t g_u32BandGapConvValue;

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
    int32_t  i32BuiltInData;

    printf("\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("| ADC for calculate battery voltage( AVdd ) by using band-gap test  |\n");
    printf("+-------------------------------------------------------------------+\n\n");

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
    ADC_ENABLE_INT(ADC, ADC_ADF_INT);   // Enable sample module A/D interrupt.
    NVIC_EnableIRQ(ADC0_INT0_IRQn);

    printf(" Press any key to start the test\n\n");
    getchar();

    /* Reset the ADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32AdcIntFlag = 0;
    ADC_START_CONV(ADC);

    /* Wait ADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the A/D interrupt */
    ADC_DISABLE_INT(ADC, ADC_ADF_INT);

    /* Get the conversion result of the channel 16 */
    i32ConversionData = ADC_GET_CONVERSION_DATA(ADC, 16);

    /* Enable FMC ISP function to read built-in band-gap A/D conversion result*/
    SYS_UnlockReg();
    FMC_Open();
    i32BuiltInData = FMC_ReadBandGap();

    /* Use Conversion result of Band-gap to calculating AVdd */

    printf("      AVdd           i32BuiltInData                   \n");
    printf("   ---------- = -------------------------             \n");
    printf("      3072          i32ConversionData                 \n");
    printf("                                                      \n");
    printf("AVdd =  3072 * i32BuiltInData / i32ConversionData     \n\n");

    printf("Built-in band-gap A/D conversion result: 0x%X (%d) \n", i32BuiltInData, i32BuiltInData);
    printf("Conversion result of Band-gap:           0x%X (%d) \n\n", i32ConversionData, i32ConversionData);

    printf("AVdd = 3072 * %d / %d = %d mV \n\n", i32BuiltInData, i32ConversionData, 3072*i32BuiltInData/i32ConversionData);
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

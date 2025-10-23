/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Convert Band-gap (Sample module 24) and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    extern void initialise_monitor_handles(void);
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 to HCLK/2 */
    CLK_SET_PCLK0DIV(CLK_PCLKDIV_APB0DIV_DIV2);
    /* Set PCLK1 to HCLK/2 */
    CLK_SET_PCLK1DIV(CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 144MHz */
    CLK_SetCoreClock(FREQ_144MHZ);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /* Select EADC peripheral clock source(maximum clock frequency = divider 4) */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HCLK, CLK_CLKDIV0_EADC0(20));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Debug UART clock setting*/
    SetDebugUartCLK();

    /* Set PB multi-function pins for Debug UART RXD and TXD */
    SetDebugUartMFP();

    /* Set reference voltage to external pin */
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);

}



void EADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|                   Band-gap test                   |\n");
    printf("+---------------------------------------------------+\n");

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 24 external sampling time to 0xF */
    EADC_SetExtendSampleTime(EADC0, 24, 0xF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 24 interrupt.  */
    EADC_ENABLE_INT(EADC0, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT24);
    NVIC_EnableIRQ(EADC00_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 24 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC0, BIT24);

    /* Wait EADC conversion done */
    while (g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC0, BIT0);

    /* Get the conversion result of the sample module 24 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC0, 24);
    printf("Conversion result of Band-gap: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);

    /* The equation of converting to real temperature is as below
     * i32ConversionData/4095*3.3, 3.3 means ADCVREF=3.3V
     * If ADCREF set to 1.6V, the equation should be updated as below
     * i32ConversionData/4095*1.6, 1.6 means ADCVREF=1.6V
     */
    printf("Band-gap = %.2f V\n\n", (i32ConversionData / 4095) * 3.3);
}

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

#if defined (__GNUC__) && !defined(__ARMCC_VERSION) && defined(OS_USE_SEMIHOSTING)
    initialise_monitor_handles();
#endif

#if !(defined(DEBUG_ENABLE_SEMIHOST))
    /* Init Debug UART for printf */
    InitDebugUart();
#endif

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);
    /* Lock protected registers */
    SYS_LockReg();
    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while (1);
}

/*** (C) COPYRIGHT 2025 Nuvoton Technology Corp. ***/

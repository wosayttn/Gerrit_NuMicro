/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Convert Band-gap and print conversion result.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*----------------------------------------------------------------------*/
/* Define global variables and constants                                */
/*----------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag;


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

    /* Set PCLK0/PCLK1 to HCLK/1 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV1 | CLK_PCLKDIV_APB1DIV_DIV1);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Select UART clock source from HIRC */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL4_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable UART clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable EADC peripheral clock */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_HIRC, CLK_CLKDIV0_EADC0(2));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Set reference voltage to external pin */
    SYS_SetVRef(SYS_VREFCTL_VREF_PIN);

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

void EADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+---------------------------------------------------+\n");
    printf("|                   Band-gap test                   |\n");
    printf("+---------------------------------------------------+\n");

    /* Force to enable internal voltage band-gap. */
    SYS_UnlockReg();
    SYS->VREFCTL |= SYS_VREFCTL_VBGFEN_Msk;
    SYS_LockReg();

    /* Set input mode as single-end and enable the A/D converter */
    EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);

    /* Set sample module 28 external sampling time to 0xF */
    EADC_SetExtendSampleTime(EADC, 28, 0xF);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);

    /* Enable the sample module 28 interrupt.  */
    EADC_ENABLE_INT(EADC, BIT0);
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, BIT28);
    NVIC_EnableIRQ(EADC0_INT0_IRQn);

    /* Reset the ADC interrupt indicator and trigger sample module 28 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, BIT28);

    /* Wait EADC conversion done */
    while(g_u32AdcIntFlag == 0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, BIT0);

    /* Get the conversion result of the sample module 28 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC, 28);
    printf("Conversion result of Band-gap: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);

    /* The equation of converting to real temperature is as below
     * i32ConversionData/4095*3.3, 3.3 means ADCVREF=3.3V
     * If ADCREF set to 1.6V, the equation should be updated as below
     * i32ConversionData/4095*1.6, 1.6 means ADCVREF=1.6V
     */
    printf("Band-gap = %.2f V\n\n", ((float)i32ConversionData/4095)*(float)(3.3));
}

void EADC0_INT0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_INT0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}

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


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32LpadcIntFlag;


void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Waiting for HIRC clock ready */
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

    /* Set PCLK2 to HCLK1/1 */
    LPSCC->CLKDIV0 = (LPSCC->CLKDIV0 & ~(LPSCC_CLKDIV0_APB2DIV_Msk)) |
                     (LPSCC_CLKDIV0_APB2DIV_DIV1);

    /* LPADC clock source is PCLK2 = 12MHz, set divider to 1, LPADC clock is 12/1 MHz */
    CLK_SetModuleClock(LPADC0_MODULE, LPSCC_CLKSEL0_LPADC0SEL_PCLK2, LPSCC_CLKDIV0_LPADC0(1));

    /* Enable LPADC module clock */
    CLK_EnableModuleClock(LPADC0_MODULE);

    /*----------------------------------------------------------------------*/
    /* Init I/O Multi-function                                              */
    /*----------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD(PB.12) and TXD(PB.13) */
    Uart0DefaultMPF();

    /* Lock protected registers */
    SYS_LockReg();
}

void LPADC_FunctionTest()
{
    int32_t  i32ConversionData;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                  LPADC for Band-gap test                             |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("+----------------------------------------------------------------------+\n");
    printf("|   LPADC clock source -> PCLK2  = 12 MHz                              |\n");
    printf("|   LPADC clock divider          = 1                                   |\n");
    printf("|   LPADC clock                  = 12 MHz / 1 = 12 MHz                 |\n");
    printf("|   LPADC extended sampling time = 20                                  |\n");
    printf("|   LPADC conversion time = 20 + LPADC extended sampling time = 40     |\n");
    printf("|   LPADC conversion rate = 12 MHz / 40 = 300 kSPS                     |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Force to enable internal voltage band-gap. */
    SYS_UnlockReg();
    SYS->VREFCTL |= SYS_VREFCTL_VBGFEN_Msk;
    SYS_LockReg();

    /* Enable LPADC converter */
    LPADC_POWER_ON(LPADC0);

    /* Set input mode as single-end, Single mode, and select channel 29 (band-gap voltage) */
    LPADC_Open(LPADC0, LPADC_ADCR_DIFFEN_SINGLE_END, LPADC_ADCR_ADMD_SINGLE, BIT29);

    /* The maximum sampling rate will be 300 kSPS for Band-gap. */
    /* Set sample module extended sampling time to 20. */
    LPADC_SetExtendSampleTime(LPADC0, 0, 20);

    /* Clear the A/D interrupt flag for safe */
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT);

    /* Enable the sample module interrupt. */
    LPADC_ENABLE_INT(LPADC0, LPADC_ADF_INT);
    NVIC_EnableIRQ(LPADC0_IRQn);

    /* Reset the LPADC interrupt indicator and trigger sample module to start A/D conversion */
    g_u32LpadcIntFlag = 0;
    LPADC_START_CONV(LPADC0);

    /* Wait LPADC conversion done */
    while(g_u32LpadcIntFlag == 0);

    /* Disable the A/D interrupt */
    LPADC_DISABLE_INT(LPADC0, LPADC_ADF_INT);

    /* Get the conversion result of the channel 29 */
    i32ConversionData = LPADC_GET_CONVERSION_DATA(LPADC0, 29);
    printf("LPADC Conversion result of Band-gap: 0x%X (%d)\n", i32ConversionData, i32ConversionData);
    printf("Band-gap voltage is %dmV if Reference voltage is 3.3V\n", (3300*i32ConversionData)/4095);
}

void LPADC0_IRQHandler(void)
{
    g_u32LpadcIntFlag = 1;
    LPADC_CLR_INT_FLAG(LPADC0, LPADC_ADF_INT); /* Clear the A/D interrupt flag */
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

    /* LPADC function test */
    LPADC_FunctionTest();

    /* Disable LPADC IP clock */
    CLK_DisableModuleClock(LPADC0_MODULE);

    /* Disable LPADC Interrupt */
    NVIC_DisableIRQ(LPADC0_IRQn);

    printf("Exit LPADC sample code\n");

    while(1);
}
